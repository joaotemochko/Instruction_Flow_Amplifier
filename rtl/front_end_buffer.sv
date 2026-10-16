`include "ifa_pkg.sv"

import ifa_pkg::*;

module front_end_buffer #(
    parameter int FETCH_WIDTH  = 2,   // largura de busca (CVA6=2, Ibex=1)
    // RESERVE_MODE=1 (hosts SEM reorder buffer, ex.: Ibex):
    //   `dispatched` significa RESERVADA. A entrada permanece no buffer ate' o
    //   host CONFIRMAR o consumo (`commit_i`). Isso permite ao consumidor
    //   RETER uma escolha (necessario para reordenar num host single-issue)
    //   sem risco de perder a instrucao: ela so' sai do FEB depois de
    //   efetivamente executada.
    // RESERVE_MODE=0 (CVA6): dreno imediato, comportamento original.
    parameter bit RESERVE_MODE = 1'b0
) (
    input  logic clk,
    input  logic rst_n,

    input  logic flush_i,
    // Com `flush_i`: preserva as entradas com sequencia MAIS VELHA ou igual a
    // `flush_keep_seq_i` e apaga o resto. Ver `parcial_*` abaixo.
    input  logic       flush_partial_i,
    input  logic [7:0] flush_keep_seq_i,

    // Interface com o estágio Fetch (Vector Push)
    input  logic [FETCH_WIDTH-1:0]             push_en,
    input  ifa_ctrl_packet_t [FETCH_WIDTH-1:0] push_data,
    // WRITE-THROUGH: a entrada e' gravada JA' despachada — ela saiu para o
    // host no mesmo ciclo em que entrou. Fica no buffer ate' o `head` passar
    // por ela, como qualquer despachada.
    input  logic [FETCH_WIDTH-1:0]             push_dispatched,
    output logic                               full,

    // Interface de leitura contínua (DAU e Dispatcher)
    output ifa_feb_entry_t [WINDOW_SIZE-1:0] window_data,
    output logic [$clog2(WINDOW_SIZE)-1:0]   head_ptr,
    output logic [$clog2(WINDOW_SIZE)-1:0]   tail_ptr,

    // Flags de ready vindas do DAU (combinacional — não armazenadas aqui)
    input  logic [WINDOW_SIZE-1:0] dau_ready_flags,

    // Interface com o Dispatcher Unit (pop escalar fora de ordem)
    input  logic dispatch_en,
    input  logic [$clog2(WINDOW_SIZE)-1:0] dispatch_index,
    // SEGUNDO canal, para host superescalar. Sem ele, a instrucao entregue na
    // porta 1 sairia para o id_stage e PERMANECERIA no buffer — seria
    // despachada de novo, corrompendo a execucao.
    input  logic dispatch_en2,
    input  logic [$clog2(WINDOW_SIZE)-1:0] dispatch_index2,
    // COMMIT POR INDICE (RESERVE_MODE).
    //
    // O `do_drain` original so' libera `buffer[head]`. Com reordenacao a
    // instrucao entregue ao host pode NAO ser a cabeca — e entao ela nunca era
    // liberada: o buffer entupia e o `slot_free` bloqueava as escolhas
    // seguintes (medido: 10.082 de 10.156 mortas ali).
    //
    // Confirmando por INDICE, cada entrada e' liberada individualmente, na
    // ordem em que o host realmente as consome. E' o que permite ter mais de
    // uma instrucao retida sem ambiguidade de posse — a causa das tres quebras
    // anteriores da fila de saida.
    input  logic commit_i,
    input  logic [$clog2(WINDOW_SIZE)-1:0] commit_index_i,
    // ROLLBACK: devolve as reservas que o consumidor NAO chegou a entregar ao
    // host. E' o que permite ao consumidor manter uma FILA de escolhas sem
    // criar dupla posse da instrucao: no flush ele devolve o que reteve e o
    // FEB volta a ser o dono unico.
    //
    // `rollback_keep_head` protege a instrucao que JA' esta' executando no
    // host — essa nao pode voltar a ser candidata (seria executada duas vezes).
    input  logic rollback_i,
    input  logic rollback_keep_head,

    // REPARO DA PILHA DE RETORNO. Quantas previsoes de retorno (`ras_pop`)
    // morreriam se houvesse flush AGORA, contando so' as que ainda nao sairam
    // para o host.
    //
    // O host tira o endereco da pilha na BUSCA, e desfaz isso no flush olhando
    // so' os estagios D e E — porque sem buffer um return so' pode estar num
    // deles. Com o FEB entre a busca e o decode, um return ja' consumido pode
    // passar dezenas de ciclos aqui dentro; se o flush o mata nessa janela o
    // consumo nunca e' devolvido, o ponteiro fica torto PARA SEMPRE e todo
    // return seguinte le' o endereco errado.
    //
    // As despachadas ficam de fora: essas estao em D/E e o reparo original do
    // host ja' as cobre — conta-las de novo torceria o ponteiro para o outro
    // lado.
    output logic [$clog2(WINDOW_SIZE):0] flush_ras_cnt_o
);

    ifa_feb_entry_t buffer [WINDOW_SIZE-1:0];

    logic [$clog2(WINDOW_SIZE)-1:0] head;
    logic [$clog2(WINDOW_SIZE)-1:0] tail;
    logic [$clog2(WINDOW_SIZE):0]   count;

    assign head_ptr = head;
    assign tail_ptr = tail;
    
    // O buffer sinaliza FULL se não tiver espaço suficiente para receber 
    // um pacote inteiro (FETCH_WIDTH) no pior caso.
    // SEM `& ~flush_i`.
    //
    // O `full` alimenta o `IFAReadyF`, que entra no `StallF` do hazard. Com o
    // `flush_i` (= FlushD) aqui, fecha laco combinacional:
    //   FlushD -> full -> IFAReadyF -> StallF -> ... -> icache/ebu -> FlushD
    //
    // O gate era otimizacao: no ciclo do flush o buffer sera' limpo de
    // qualquer forma, entao declarar "nao cheio" adiantava um ciclo. Custa um
    // ciclo perder isso, mas o laco combinacional e' impeditivo.
    // MARGEM DE UMA POSICAO.
    //
    // O `full` chega ao `StallF` do hazard com um ciclo de atraso. Nesse ciclo
    // o fetch ja' avancou o PC, e a instrucao e' descartada aqui (`!full` no
    // push) sem que o PC volte — PERDA SILENCIOSA.
    //
    // Detectada no traco: `800091dc (00880813) -> 80009220`, 68 bytes / 17
    // instrucoes, apos um `addi` que nao e' desvio.
    //
    // Declarar cheio uma posicao antes garante espaco para a instrucao em
    // transito.
    assign full = ((WINDOW_SIZE - 32'(count)) <= FETCH_WIDTH);

    // Exposição combinacional do buffer inteiro para DAU e Dispatcher
    always_comb begin
        for (int k = 0; k < WINDOW_SIZE; k++) window_data[k] = buffer[k];
    end

    // ------------------------------------------------------------------
    // Atualização Síncrona (Ponteiros e Dados)
    // ------------------------------------------------------------------
    logic [$clog2(FETCH_WIDTH+1)-1:0] num_pushes;
    logic [$clog2(WINDOW_SIZE)-1:0]   next_tail;
    logic do_drain;
    logic commit_off_head;
    assign commit_off_head = RESERVE_MODE & commit_i & (commit_index_i != head) &
                             buffer[commit_index_i].valid & ~flush_i;

    // DRENO SEM BOLHA: drena a head se ela ja foi despachada (ciclo anterior)
    // OU se ela esta sendo despachada AGORA (dispatch_en ja inclui dispatch_ready).
    // Sem o segundo termo, a head despachada fica 1 ciclo extra parada -> bolha
    // por instrucao (pior em FIFO) -> overhead de throughput (ate ~2x ciclos).
    // Em RESERVE_MODE a cabeca so' e' liberada quando o host confirma.
    // Em RESERVE_MODE a cabeca avanca quando ela propria e' confirmada; as
    // demais entradas confirmadas sao invalidadas no lugar (ver abaixo).
    assign do_drain = RESERVE_MODE
                    ? (buffer[head].valid & ~flush_i & buffer[head].dispatched &
                       commit_i & (commit_index_i == head))
                    : (buffer[head].valid & ~flush_i &
                       ( buffer[head].dispatched
                       | (dispatch_en & (dispatch_index == head)) ));

    // ------------------------------------------------------------------
    // FLUSH PARCIAL (erro de previsao de um desvio despachado fora de ordem)
    //
    // O desvio B pode ter sido despachado ANTES de uma instrucao mais velha H
    // que estava travada. Se B erra a previsao, o flush total apagaria H
    // junto com o caminho errado — e H e' mais velha, pertence ao caminho
    // certo e nunca mais seria buscada.
    //
    // Aqui so' se apaga o que e' MAIS NOVO que B. Como o buffer esta' em
    // ordem de programa a partir do `head`, o que sobra e' um PREFIXO
    // contiguo: `tail` vai para logo depois dele.
    //
    // Idade por diferenca modulo 256: `e` e' mais nova que `b` sse
    // (e - b) mod 256 esta' em [1,127].
    // ------------------------------------------------------------------
    logic [WINDOW_SIZE-1:0]         parcial_manter;
    logic [$clog2(WINDOW_SIZE):0]   parcial_count;
    always_comb begin
        parcial_count = '0;
        for (int k = 0; k < WINDOW_SIZE; k++) begin
            logic [7:0] d;
            d = buffer[k].data.seq - flush_keep_seq_i;
            parcial_manter[k] = buffer[k].valid && ((d == 8'd0) || d[7]);
            if (parcial_manter[k]) parcial_count = parcial_count + 1'b1;
        end
    end

    // No flush parcial sobrevive o prefixo mais velho, e o que sobrevive NAO e'
    // devolvido; no flush total nao sobra nada.
    always_comb begin
        flush_ras_cnt_o = '0;
        for (int k = 0; k < WINDOW_SIZE; k++) begin
            if (buffer[k].valid && !buffer[k].dispatched &&
                buffer[k].data.ras_pop &&
                !(flush_partial_i && parcial_manter[k]))
                flush_ras_cnt_o = flush_ras_cnt_o + 1'b1;
        end
    end

    always_comb begin
        num_pushes = 0;
        for (int i = 0; i < FETCH_WIDTH; i++) begin
            num_pushes += $clog2(FETCH_WIDTH+1)'(push_en[i]);
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            head  <= '0;
            tail  <= '0;
            count <= '0;
            for (int i = 0; i < WINDOW_SIZE; i++) begin
                buffer[i].valid      <= 1'b0;
                buffer[i].dispatched <= 1'b0;
                buffer[i].data       <= '{default: '0};
            end
        end else if (flush_i && flush_partial_i) begin
            // Mantem o prefixo mais velho que o desvio; `head` nao muda.
            for (int i = 0; i < WINDOW_SIZE; i++) begin
                if (!parcial_manter[i]) begin
                    buffer[i].valid      <= 1'b0;
                    buffer[i].dispatched <= 1'b0;
                end
            end
            count <= parcial_count;
            tail  <= head + $clog2(WINDOW_SIZE)'(parcial_count);
        end else if (flush_i) begin
            // Um flush por pc_set invalida o caminho ESPECULATIVO inteiro: as
            // instrucoes do FEB nao pertencem mais ao fluxo e nao devem voltar.
            // O `rollback` NAO se aplica aqui — ele existe para o consumidor
            // devolver escolhas fora de flush.
            head  <= '0;
            tail  <= '0;
            count <= '0;
            for (int i = 0; i < WINDOW_SIZE; i++) begin
                buffer[i].valid      <= 1'b0;
                buffer[i].dispatched <= 1'b0;
            end
        end else if (RESERVE_MODE && rollback_i) begin
            // ROLLBACK (fora de flush): o consumidor descartou escolhas que
            // reteve mas nao entregou. Elas voltam a ser candidatas — o FEB
            // volta a ser dono unico da instrucao.
            //
            // Prioridade ABAIXO do flush: num flush o caminho inteiro e'
            // invalido e nada deve voltar.
            //
            // `rollback_keep_head` protege a que JA' esta' executando no host:
            // devolve-la faria o host executa-la duas vezes — foi o que
            // aconteceu na tentativa anterior (2.789.191 despachos contra
            // 1.130.256 aceites, PC saindo para 0x10ae58).
            for (int i = 0; i < WINDOW_SIZE; i++) begin
                if (!(rollback_keep_head && (i[$clog2(WINDOW_SIZE)-1:0] == head)))
                    buffer[i].dispatched <= 1'b0;
            end
        end else begin
            // Atualiza contador baseando-se em quantos entraram e se 1 saiu
            // `commit_off_head` = confirmacao de uma entrada que NAO e' a
            // cabeca: ela e' invalidada no lugar, entao tambem reduz o count.
            // Sem isso o contador infla e o `full` bloqueia o buffer.
            logic head_auto_advances;
            head_auto_advances = RESERVE_MODE && head != tail && !buffer[head].valid && !do_drain;

            count <= count + ($clog2(WINDOW_SIZE)+1)'(num_pushes)
                           - ($clog2(WINDOW_SIZE)+1)'(do_drain)
                           - ($clog2(WINDOW_SIZE)+1)'(head_auto_advances);

            // VECTOR PUSH, indexado por `tail + i`.
            //
            // `push_en` e' sempre um PREFIXO contiguo — quem o gera valida um
            // prefixo do bloco buscado, parando na primeira instrucao que nao
            // cabe ou que e' barreira. Entao a entrada do slot `i` vai para
            // `tail + i`, sem depender de quantos slots anteriores estavam
            // ativos.
            //
            // A versao anterior acumulava o indice dentro do laco
            // (`next_tail = next_tail + 1`). Como as atribuicoes sao
            // nao-bloqueantes e o indice de um slot dependia dos anteriores, a
            // sintese era obrigada a montar, para CADA uma das 16 entradas, um
            // mux de PRIORIDADE de N niveis sobre `push_data`, mais os
            // somadores em cadeia. Sobre um pacote largo isso domina a area do
            // bloco inteiro. Com `tail + i` os indices sao provadamente
            // distintos e o mux vira paralelo.
            //
            // `ifa_top` faz a MESMA indexacao no `payload_mem` e verifica o
            // prefixo por assercao — se os dois divergirem, controle e payload
            // passam a pertencer a instrucoes diferentes.
            /* verilator lint_off BLKSEQ */
            next_tail = tail;   // variavel local de loop; nao e' estado
            for (int i = 0; i < FETCH_WIDTH; i++) begin
                if (push_en[i] && !full) begin
                    buffer[tail + $clog2(WINDOW_SIZE)'(i)].data       <= push_data[i];
                    buffer[tail + $clog2(WINDOW_SIZE)'(i)].valid      <= 1'b1;
                    buffer[tail + $clog2(WINDOW_SIZE)'(i)].dispatched <= push_dispatched[i];
                    next_tail                    = next_tail + 1;
                    /* verilator lint_on BLKSEQ */
                end
            end
            tail <= next_tail;

            // COMMIT fora da cabeca: invalida a entrada confirmada onde ela
            // esta'. O `head` so' avanca quando a propria cabeca e' confirmada
            // (do_drain), preservando a ordem do buffer circular.
            if (RESERVE_MODE && commit_i && (commit_index_i != head)) begin
                buffer[commit_index_i].valid      <= 1'b0;
                buffer[commit_index_i].dispatched <= 1'b0;
            end

            // Marca instrução escolhida pelo Dispatcher como despachada
            // Segunda entrada do par: mesma marcacao.
            if (dispatch_en2 && buffer[dispatch_index2].valid) begin
                buffer[dispatch_index2].dispatched <= 1'b1;
            end

            if (dispatch_en && buffer[dispatch_index].valid) begin
                buffer[dispatch_index].dispatched <= 1'b1;
            end

            // Drain: retira a instrução mais velha (head) se já foi despachada
            if (do_drain) begin
                buffer[head].valid      <= 1'b0;
                buffer[head].dispatched <= 1'b0;
                head                    <= head + 1;
            end else if (RESERVE_MODE && head != tail && !buffer[head].valid) begin
                // O head precisa avancar se a instrucao ja' foi comitada fora de ordem
                head <= head + 1;
            end
        end
    end

    /* verilator lint_off UNUSEDSIGNAL */
    logic unused_ready;
    assign unused_ready = ^dau_ready_flags;
    /* verilator lint_on UNUSEDSIGNAL */

endmodule
