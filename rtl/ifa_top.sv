`include "ifa_pkg.sv"

import ifa_pkg::*;

module ifa_top #(
    parameter int PAYLOAD_WIDTH      = 256,
    parameter int PAYLOAD_BP_WIDTH   = 1,
    parameter int PAYLOAD_BASE_WIDTH = 1,
    /* verilator lint_off UNUSEDPARAM */
    parameter int NR_COMMIT_PORTS = 2,   // interface CVA6; nao usado no Ibex
    /* verilator lint_on UNUSEDPARAM */
    parameter int FETCH_WIDTH     = 2,
    parameter bit RESERVE_MODE    = 1'b0,
    // 1 = despacho estritamente EM ORDEM. Isola o ganho do buffer (§III-A) do
    // ganho da reordenacao (§III-C).
    parameter bit FIFO_MODE       = 1'b0,  // ver front_end_buffer.sv
    // 1 = host RV32: quatro encodings comprimidos mudam de significado.
    // Ver o cabecalho do ifa_pre_decode.
    parameter bit RV32            = 1'b0
) (
    input  logic              clk,
    input  logic              rst_n,
    input  logic              flush_i,
    // FLUSH PARCIAL: junto com `flush_i`, preserva as entradas MAIS VELHAS
    // que `flush_keep_seq_i` (o numero de sequencia do desvio que errou).
    // So' vale para erro de previsao; trap/ret/fence continuam flush total.
    input  logic              flush_partial_i,
    input  logic [7:0]        flush_keep_seq_i,
    // Sequencia da instrucao despachada neste ciclo (acompanha o payload).
    output logic [7:0]        dispatch_seq_o,
    // HOIST ESPECULATIVO: `dispatch_spec_o`=1 marca que a escolha deste ciclo
    // passou a frente do desvio da cabeca. O host tem de informar em
    // `hoist_pend_i` que o proximo despacho aceito deve ser a cabeca.
    output logic              dispatch_spec_o,
    input  logic              hoist_pend_i,

    // Interface de Entrada Vetorial (Vector Fetch)
    input  logic [FETCH_WIDTH-1:0]                    fetch_valid_i,
    input  logic [FETCH_WIDTH-1:0][31:0]              fetch_instr_i,
    input  logic [FETCH_WIDTH-1:0][PAYLOAD_WIDTH-1:0] fetch_payload_i,
    // PAYLOAD EXCLUSIVO DO SLOT 0 (ver ifa_wally_pkg.sv).
    //
    // O host garante que so' a primeira instrucao do bloco pode precisar
    // destes campos — no Wally, porque o bloco termina no primeiro desvio.
    // Por isso a fonte e' UNICA: nao ha' mux N:1 por entrada, que e' o que
    // dominava a area com busca larga. As demais entradas do bloco leem zero,
    // marcadas por um unico bit de validade.
    input  logic [PAYLOAD_BP_WIDTH-1:0]               fetch_payload_bp_i,
    output logic [PAYLOAD_BP_WIDTH-1:0]               dispatch_payload_bp_o,
    // BASE COMUM AS N INSTRUCOES DO BLOCO (no Wally, o PC da primeira).
    //
    // Vai para TODAS as entradas empurradas neste ciclo com o MESMO valor,
    // entao nao ha' o que multiplexar: a sintese resolve com o enable de cada
    // flop. O que difere entre elas viaja no payload comum, estreito.
    input  logic [PAYLOAD_BASE_WIDTH-1:0]             fetch_base_i,
    output logic [PAYLOAD_BASE_WIDTH-1:0]             dispatch_base_o,
    // 1 = a BUSCA previu que esta instrucao desvia (o proximo PC nao e' o
    // sequencial). Vale mesmo quando a instrucao NAO e' desvio: e' o caso de
    // classe mal prevista pelo BTB, e e' justamente ele que faz o host
    // REDIRECIONAR o PC quando descobre o engano.
    //
    // Um redirecionamento descarta o buffer inteiro. Se houver uma instrucao
    // MAIS VELHA ainda no buffer (deixada para tras por uma reordenacao), ela
    // e' descartada e nunca rebuscada — perdida. Por isso a entrada com este
    // bit entra como BARREIRA, igual a um desvio de verdade.
    //
    // Hosts sem preditor ligam em 0.
    input  logic [FETCH_WIDTH-1:0]                    fetch_pred_redirect_i,
    // WRITE-THROUGH: 1 = esta instrucao entra no FEB e sai para o host no
    // MESMO ciclo (o host so' pede isso com nada pendente no buffer). Ela e'
    // gravada ja' despachada e recebe numero de sequencia normalmente.
    input  logic [FETCH_WIDTH-1:0]                    fetch_dispatched_i,
    // 1 = a BUSCA desta instrucao faltou (page/access fault). O campo de
    // instrucao e' lixo e o pre-decode poderia classifica-lo como ALU e
    // reordena-lo; entao ela entra como SERIALIZANTE: nada passa por ela e ela
    // so' sai com nada mais velho pendente, e o trap fica preciso.
    input  logic [FETCH_WIDTH-1:0]                    fetch_serialize_i,
    // 1 = a busca desta instrucao CONSUMIU uma previsao de retorno do host.
    // Hosts sem pilha de retorno ligam em 0. Ver `flush_ras_cnt_o`.
    input  logic [FETCH_WIDTH-1:0]                    fetch_ras_pop_i,
    // Quantos desses consumos o flush deste ciclo jogaria fora, para o host
    // devolve-los a' pilha.
    output logic [$clog2(WINDOW_SIZE):0]              flush_ras_cnt_o,
    // Sequencia que a proxima instrucao empurrada vai receber (e' a da
    // instrucao em write-through).
    output logic [7:0]                                push_seq_o,
    output logic                                      fetch_ready_o,

    // Interface de Saída Escalar (Scalar Dispatch)
    output logic                                      dispatch_valid_o,
    // Indice da entrada escolhida: o consumidor guarda e devolve em commit_index_i.
    output logic [$clog2(WINDOW_SIZE)-1:0]            dispatch_index_o,
    output logic [PAYLOAD_WIDTH-1:0]                  dispatch_payload_o,
    input  logic                                      dispatch_ready_i,
    // `ready` da segunda porta. O FEB precisa saber que DUAS sairam.
    input  logic                                      dispatch_ready2_i,

    input  logic [31:0]                               scoreboard_reg_busy_i,
    input  logic                                      commit_i,
    input  logic [$clog2(WINDOW_SIZE)-1:0]            commit_index_i,
    input  logic                                      rollback_i,
    input  logic                                      rollback_keep_head_i,   // consumo confirmado
    input  logic                                      multdiv_inflight_i,
    input  logic                                      lsu_busy_i,
    input  logic                                      multdiv_busy_i,
    input  logic                                      host_stalled_i,
    input  logic                                      mem_inflight_i,
    // FEB REALMENTE vazio (nada valido no buffer).
    //
    // NAO confundir com `~dispatch_valid_o`: com RESERVE_MODE o dispatcher
    // pode nao escolher nada enquanto o buffer esta' CHEIO de entradas
    // reservadas. Usar o valid como proxy de vazio faz o consumidor ativar o
    // bypass nesse estado — a instrucao nao entra no FEB, o buffer nunca
    // esvazia, e o core trava (medido: push_en congelado em 50, ifa_full=21,
    // Cycles parado em 78).
    output logic                                      feb_empty_o,
    // BURACO na janela: existe entrada valida NAO despachada com alguma MAIS
    // NOVA ja' despachada atras dela. Nesse estado o commit esta' fora de
    // ordem e um flush por IRQ apagaria a mais velha para sempre — o `mepc`
    // apontaria para depois dela.
    //
    // ATENCAO A DIRECAO: varrendo do head (mais VELHA primeiro), o padrao
    // "despachada -> depois nao-despachada" e' o estado NORMAL de fila em
    // ordem. O buraco e' o padrao INVERSO.
    output logic                                      hole_o,
    output logic                                      reorder_o,
    // Segunda instrucao do par (host superescalar). `dispatch_valid2_o = 0`
    // quando nao ha' segunda candidata.
    output logic                                      dispatch_valid2_o,
    output logic [PAYLOAD_WIDTH-1:0]                  dispatch_payload2_o,  // despacho fora de ordem
    output logic                                      dbg_cand_ready,
    output logic                                      dbg_blk_unsafe,
    output logic                                      dbg_blk_barrier,
    output logic [4:0]                                dbg_notready,
    output logic                                      dbg_head_notready,
    output logic                                      dbg_head_alt,
    output logic                                      dbg_any_ready,
    output logic [4:0]                                dbg_ready_nothead,
    output logic [4:0]                                dbg_lost_barrier,
    output logic [4:0]                                dbg_avail_raw,
    // Ocupacao do FEB: entradas validas e AINDA NAO despachadas.
    // E' o numero que responde "o FEB enche?".
    output logic [4:0]                                dbg_ocup
);

    ifa_feb_entry_t [WINDOW_SIZE-1:0] internal_window_data;
    logic [$clog2(WINDOW_SIZE)-1:0]   internal_head_ptr;
    logic [$clog2(WINDOW_SIZE)-1:0]   internal_tail_ptr;
    logic [WINDOW_SIZE-1:0]           internal_ready_flags;
    logic [WINDOW_SIZE-1:0]           dau_ready_raw;
    logic [WINDOW_SIZE-1:0]           dau_spec_ready;   // hoist especulativo (ver DAU)

    // BUFFER PURO: ignora o DAU. Ver ifa_config_pkg.sv.
    assign internal_ready_flags = dau_ready_raw;
    logic                             internal_dispatch_en2;
    logic [$clog2(WINDOW_SIZE)-1:0]   internal_dispatch_index2;
    /* verilator lint_off UNUSEDSIGNAL */
    ifa_ctrl_packet_t                 unused_disp_inst2;
    /* verilator lint_on UNUSEDSIGNAL */
    /* verilator lint_off UNUSEDSIGNAL */
    // O payload vem de `payload_mem`, nao do dispatcher; esta saida do
    // dispatcher_unit fica sem uso. Dummy em vez de `()` porque o build do
    // CVA6 trata PINCONNECTEMPTY como erro.
    ifa_ctrl_packet_t                 unused_disp_inst;
    /* verilator lint_on UNUSEDSIGNAL */
    logic                             internal_dispatch_en;
    logic [$clog2(WINDOW_SIZE)-1:0]   internal_dispatch_index;
    logic                             internal_buffer_empty;
    
    ifa_ctrl_packet_t [FETCH_WIDTH-1:0] pre_decode_packet;
    logic [FETCH_WIDTH-1:0]             feb_push_en;
    logic [7:0]                         seq_q;       // proximo numero de sequencia
    logic                               feb_full_w;   // `full` do FEB
    // (ifa_full removido: era um alias redundante de feb_full_w)

    // Com RESERVE_MODE, a posicao de `head_ptr` pode estar valida porem ja'
    // RESERVADA. "Vazio" para o dispatcher significa: nada a despachar.
    logic [WINDOW_SIZE-1:0] feb_any_valid;
    for (genvar v = 0; v < WINDOW_SIZE; v++) begin : g_feb_valid
        assign feb_any_valid[v] = internal_window_data[v].valid;
    end

    logic any_pending;
    always_comb begin
        any_pending = 1'b0;
        for (int e = 0; e < WINDOW_SIZE; e++)
            if (internal_window_data[e].valid && !internal_window_data[e].dispatched)
                any_pending = 1'b1;
    end
    assign internal_buffer_empty = ~any_pending;
    assign fetch_ready_o = ~feb_full_w;

    // =========================================================================
    // Multi-Pre-Decode e Roteamento de Entrada
    // =========================================================================
    for (genvar i = 0; i < FETCH_WIDTH; i++) begin : gen_pre_decoders
        logic [4:0] pd_rs1, pd_rs2, pd_rd;
        logic pd_uses_rs1, pd_uses_rs2, pd_writes_rd;
        logic pd_is_branch, pd_is_syscall_atomic, pd_is_mem, pd_is_csr, pd_is_multdiv;
        logic pd_is_idempotent;

        ifa_pre_decode #(.RV32(RV32)) u_pd (
            .instruction      (fetch_instr_i[i]),
            .rs1              (pd_rs1), 
            .rs2              (pd_rs2), 
            .rd               (pd_rd),
            .uses_rs1         (pd_uses_rs1), 
            .uses_rs2         (pd_uses_rs2), 
            .writes_rd        (pd_writes_rd),
            .is_branch        (pd_is_branch), 
            .is_mem           (pd_is_mem), 
            .is_syscall_atomic(pd_is_syscall_atomic), 
            .is_csr           (pd_is_csr),
            .is_multdiv       (pd_is_multdiv),
            .is_idempotent    (pd_is_idempotent)
        );

        assign feb_push_en[i] = fetch_valid_i[i] & fetch_ready_o & ~flush_i;

        always_comb begin
            pre_decode_packet[i]                   = '0;
            pre_decode_packet[i].rs1               = pd_rs1;
            pre_decode_packet[i].rs2               = pd_rs2;
            pre_decode_packet[i].rd                = pd_rd;
            pre_decode_packet[i].uses_rs1          = pd_uses_rs1;
            pre_decode_packet[i].uses_rs2          = pd_uses_rs2;
            pre_decode_packet[i].writes_rd         = pd_writes_rd;
            pre_decode_packet[i].is_branch         = pd_is_branch | fetch_pred_redirect_i[i];
            pre_decode_packet[i].is_syscall_atomic = pd_is_syscall_atomic | fetch_serialize_i[i];
            pre_decode_packet[i].is_mem            = pd_is_mem;
            pre_decode_packet[i].is_csr            = pd_is_csr;
            pre_decode_packet[i].is_multdiv        = pd_is_multdiv;
            pre_decode_packet[i].is_idempotent     = pd_is_idempotent;
            pre_decode_packet[i].seq               = seq_q + 8'(i);
            pre_decode_packet[i].ras_pop           = fetch_ras_pop_i[i];
        end
    end

    // Contador de sequencia: avanca uma unidade por instrucao que ENTRA.
    // Modulo 256 basta: entre uma instrucao entrar e seu desvio ser resolvido
    // passam no maximo ~WINDOW_SIZE + profundidade do pipeline instrucoes.
    always_ff @(posedge clk) begin
        if (!rst_n) seq_q <= '0;
        else begin
            logic [7:0] n;
            n = '0;
            for (int s = 0; s < FETCH_WIDTH; s++)
                if (feb_push_en[s] & ~feb_full_w) n = n + 8'd1;
            seq_q <= seq_q + n;
        end
    end

    // =========================================================================
    // Front-End Buffer (O coração do IFA)
    // =========================================================================
    front_end_buffer #(
        .FETCH_WIDTH(FETCH_WIDTH),
        .RESERVE_MODE (RESERVE_MODE)
    ) u_feb (
        .clk             (clk),
        .rst_n           (rst_n),
        .flush_i         (flush_i),
        .flush_partial_i (flush_partial_i),
        .flush_keep_seq_i(flush_keep_seq_i),
        .push_en         (feb_push_en),
        .push_data       (pre_decode_packet),
        .push_dispatched (fetch_dispatched_i),
        .full            (feb_full_w),
        .window_data     (internal_window_data),
        .head_ptr        (internal_head_ptr),
        .tail_ptr        (internal_tail_ptr), 
        .dau_ready_flags (internal_ready_flags),
        // HANDSHAKE FIX: so marca dispatched quando o issue stage ACEITA
        // (dispatch_ready_i=1). Sem isso, sob backpressure do scoreboard a
        // instrucao era marcada despachada sem o core consumir -> perdida
        // -> registrador/store com valor velho -> B Response Errored.
        .dispatch_en     (internal_dispatch_en & dispatch_ready_i),
        .dispatch_en2    (internal_dispatch_en2 & dispatch_ready2_i),
        .dispatch_index2 (internal_dispatch_index2),   
        .dispatch_index  (internal_dispatch_index),
        .commit_i        (commit_i),
        .commit_index_i  (commit_index_i),
        .rollback_i      (rollback_i),
        .rollback_keep_head (rollback_keep_head_i),
        .flush_ras_cnt_o (flush_ras_cnt_o)
    );

    // =========================================================================
    // Shadow Memory - Armazena o payload pesado do core hospedeiro
    // Aceita gravação de N vias por ciclo simulando a lógica de tail do FEB
    // =========================================================================
    // =====================================================================
    // BANCARIZACAO POR FAIXA DE BUSCA.
    //
    // O payload so' e' lido para UMA entrada — a despachada. Isso e' o padrao
    // de uma memoria, nao de um banco de registradores, e numa FPGA deveria
    // cair em LUTRAM (ordens de grandeza mais denso). Nao caia: com N faixas
    // de busca a memoria tinha N PORTAS DE ESCRITA, e LUTRAM so' tem uma.
    // A sintese entao gerava flops mais um mux N:1 em cada uma das
    // WINDOW_SIZE entradas — medido na Artix-7: 96 LUTRAMs apenas, e o
    // payload inteiro em flip-flops.
    //
    // A saida e' notar que as entradas escritas num ciclo sao CONSECUTIVAS
    // (`tail`, `tail+1`, ...). Pondo a entrada `k` no banco `k % N`, duas
    // entradas do mesmo ciclo nunca caem no mesmo banco: cada banco recebe no
    // maximo UMA escrita por ciclo. Uma porta de escrita, uma de leitura —
    // agora e' LUTRAM.
    //
    // O banco `b` e' escrito pela faixa `(b - tail) mod N`, entao sobra um mux
    // N:1 por BANCO (N deles) em vez de um por ENTRADA (WINDOW_SIZE deles).
    // Na leitura, todos os bancos usam o mesmo endereco (`idx / N`) e um unico
    // mux N:1 escolhe o banco (`idx % N`).
    // =====================================================================
    localparam int NBANK   = FETCH_WIDTH;
    localparam int BDEPTH  = WINDOW_SIZE / NBANK;
    localparam int BSEL    = (NBANK > 1) ? $clog2(NBANK) : 1;
    localparam int BADDR   = (BDEPTH > 1) ? $clog2(BDEPTH) : 1;

    logic [PAYLOAD_WIDTH-1:0]      payload_mem [NBANK-1:0][BDEPTH-1:0];
    logic [PAYLOAD_BASE_WIDTH-1:0] base_mem    [NBANK-1:0][BDEPTH-1:0];
    logic [PAYLOAD_BP_WIDTH-1:0]   paybp_mem   [WINDOW_SIZE-1:0];
    logic [WINDOW_SIZE-1:0]        paybp_val;   // 0 = esta entrada le' zero

    // Qual faixa escreve cada banco neste ciclo, e em que posicao.
    logic [NBANK-1:0]             bank_we;
    logic [NBANK-1:0][BSEL-1:0]   bank_lane;
    logic [NBANK-1:0][BADDR-1:0]  bank_addr;
    always_comb begin
        for (int b = 0; b < NBANK; b++) begin
            logic [$clog2(WINDOW_SIZE)-1:0] ent;
            int lane;
            // `(b - tail) mod N`: a faixa cuja entrada cai neste banco.
            lane         = (b - int'(internal_tail_ptr)) % NBANK;
            if (lane < 0) lane = lane + NBANK;
            ent          = internal_tail_ptr + $clog2(WINDOW_SIZE)'(lane);
            bank_lane[b] = BSEL'(lane);
            bank_addr[b] = BADDR'(int'(ent) / NBANK);
            bank_we[b]   = feb_push_en[lane] & ~feb_full_w;
        end
    end

    // -------------------------------------------------------------------
    // PAYLOAD DE PREVISAO: uma fonte so', logo nenhum mux.
    //
    // O slot 0 grava em `tail`; os demais slots do bloco apenas LIMPAM o bit
    // de validade da sua entrada. Sem essa limpeza a entrada herdaria a
    // previsao do ocupante anterior daquela posicao — uma instrucao que nao e'
    // desvio receberia `redirect=1` de um desvio antigo, o sucessor previsto
    // sairia errado e o `BPWrongE` dispararia sem motivo.
    //
    // Limpar 1 bit custa um decodificador de 16; gravar 139 zeros custaria o
    // mux que este desenho existe para evitar.
    // -------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            paybp_val <= '0;
        end else if (!flush_i) begin
            for (int i = 0; i < FETCH_WIDTH; i++) begin
                if (feb_push_en[i] & ~feb_full_w) begin
                    // `tail + i`, nao a soma encadeada: ver a nota do push.
                    paybp_val[internal_tail_ptr + $clog2(WINDOW_SIZE)'(i)] <= (i == 0);
                    if (i == 0) paybp_mem[internal_tail_ptr] <= fetch_payload_bp_i;
                end
            end
        end
    end

    // PREFIXO CONTIGUO. `feb_push_en` nunca tem buraco: o extrator de bloco do
    // host valida um PREFIXO (a primeira que nao cabe ou que e' barreira
    // encerra o bloco), entao as entradas deste ciclo vao para `tail`,
    // `tail+1`, ... sem depender de quantas vieram antes.
    //
    // Isso importa para AREA, nao so' para elegancia. Com o indice acumulado
    // (`mem_tail = mem_tail + 1` dentro do laco), cada slot escreve num indice
    // que depende dos anteriores, e a semantica de atribuicao nao-bloqueante
    // obriga a sintetizar um mux de PRIORIDADE de N niveis por entrada, com os
    // somadores em cadeia junto. Com `tail + i` os indices sao provadamente
    // distintos e vira um mux paralelo.
    //
    // A assercao existe para que isto quebre ALTO se alguem mudar o extrator
    // para gerar buracos — sem ela, o payload e o pacote de controle passariam
    // a pertencer a instrucoes diferentes, silenciosamente.
    `ifndef SYNTHESIS
    always_ff @(posedge clk) begin
        if (rst_n && !flush_i)
            for (int i = 1; i < FETCH_WIDTH; i++)
                if (feb_push_en[i] && !feb_push_en[i-1])
                    $error("IFA: feb_push_en=%b nao e' prefixo contiguo", feb_push_en);
    end
    `endif

    // Uma escrita por banco por ciclo: e' o que permite a inferencia de LUTRAM.
    // Sem reset e sem leitura da memoria inteira — as duas coisas tambem
    // impedem a inferencia.
    //
    // O enable de cada banco vem de `feb_push_en`, EXATAMENTE o mesmo sinal que
    // o FEB usa para gravar o pacote de controle. Quando as duas condicoes
    // divergem por um ciclo, o `tail` avanca num e nao no outro — e a partir
    // dai o pacote e o payload da mesma posicao pertencem a instrucoes
    // DIFERENTES, permanentemente. O `dispatch_index` entrega entao a
    // instrucao errada ao ID. Sintoma: caracteres perdidos e trocados na saida
    // ("299008" em vez de "2990088", "0xe9fv5" em vez de "0xe9f5") com a
    // barreira de memoria intacta (FORA DE ORDEM = 0).
    always_ff @(posedge clk) begin
        if (!flush_i) begin
            for (int b = 0; b < NBANK; b++) begin
                if (bank_we[b]) begin
                    payload_mem[b][bank_addr[b]] <= fetch_payload_i[bank_lane[b]];
                    // Mesma base para todo o bloco: aqui nem mux ha', so' o
                    // enable do banco.
                    base_mem[b][bank_addr[b]]    <= fetch_base_i;
                end
            end
        end
    end

    // =========================================================================
    // Unidade de Análise de Dependência e Dispatcher
    // =========================================================================
    // =========================================================================
    // CASAMENTO IFA<->scoreboard: SHADOW BUSY TABLE
    // O reg_busy do scoreboard so sobe quando o load e' EMITIDO (~1-2 ciclos
    // apos o IFA despachar). Nessa janela o DAU nao sabe que o load esta
    // pendente -> despacha o dependente (que o issue do CVA6 trava) em vez de
    // reordenar a independente -> reorder perde a janela. O shadow marca o rd
    // do load como pendente JA no despacho, por 2 ciclos, ate o scoreboard
    // assumir. So afeta a DECISAO de reorder (perf); o issue do CVA6 continua
    // garantindo a dependencia real (zero risco de correcao).
    // =========================================================================
    /* verilator lint_off UNUSEDSIGNAL */
    logic        disp_is_load;
    logic [4:0]  disp_rd;
    logic [31:0] shadow_busy_q, shadow_busy_q2;
    /* verilator lint_on UNUSEDSIGNAL */

    assign disp_is_load = internal_dispatch_en & dispatch_ready_i &
                          internal_window_data[internal_dispatch_index].data.is_mem &
                          internal_window_data[internal_dispatch_index].data.writes_rd &
                          (internal_window_data[internal_dispatch_index].data.rd != 5'd0);
    assign disp_rd = internal_window_data[internal_dispatch_index].data.rd;

    logic [31:0] reg_busy;
    assign reg_busy = scoreboard_reg_busy_i;

    // ---------------------------------------------------------------------
    // CABECA REAL para o DAU e para o dispatcher.
    //
    // BUG: o DAU usa `head_ptr` na comparacao de idade circular
    // (`j_is_older`). Com RESERVE_MODE o `head_ptr` fica PARADO na entrada
    // reservada ate' o commit, enquanto a fila avanca — a janela de idade sai
    // deslocada e o DAU classifica dependencias sobre uma ordem errada.
    //
    // Isso corrompe TODO o `ready_flags`, que e' a base do reorder e das
    // metricas. Explica a contradicao medida: `CABECA travada com alternativa
    // = 0` convivendo com `PRONTAS fora da cabeca` em 64% dos ciclos — os dois
    // nao podem ser verdadeiros ao mesmo tempo.
    //
    // A referencia correta e' a mais VELHA ainda NAO despachada.
    // ---------------------------------------------------------------------
    logic [$clog2(WINDOW_SIZE)-1:0] dau_head;
    always_comb begin
        logic found;
        found    = 1'b0;
        dau_head = internal_head_ptr;
        for (int o = 0; o < WINDOW_SIZE; o++) begin
            logic [$clog2(WINDOW_SIZE)-1:0] k;
            k = internal_head_ptr + $unsigned(o[$clog2(WINDOW_SIZE)-1:0]);
            if (!found && internal_window_data[k].valid &&
                !internal_window_data[k].dispatched) begin
                found    = 1'b1;
                dau_head = k;
            end
        end
    end

`ifndef SYNTHESIS
    // ---------------------------------------------------------------------
    // SONDA INDEPENDENTE (fora do always_comb do dispatcher).
    //
    // As sondas anteriores viviam dentro do bloco combinacional do
    // dispatcher, junto de `dispatch_index`, `found_dispatch` etc. Variavel
    // de loop compartilhada num always_comb grande e' fonte classica de
    // resultado errado — e o par (CABECA travada = 142.550, com alternativa
    // = 0) contra (PRONTAS fora da cabeca = 560.251 ciclos) e' impossivel:
    // dois eventos com 3,5% e 13,6% de ocorrencia nao tem intersecao ZERO.
    //
    // Aqui os dois sao recalculados do zero, no mesmo lugar, sobre os mesmos
    // sinais — se a intersecao continuar nula, e' propriedade real do fluxo.
    // ---------------------------------------------------------------------
    logic       probe_head_stalled;
    logic       probe_has_alt;
    logic [4:0] probe_alt_count;

    always_comb begin
        logic found;
        logic [$clog2(WINDOW_SIZE)-1:0] rh;
        probe_head_stalled = 1'b0;
        probe_has_alt      = 1'b0;
        probe_alt_count    = 5'd0;

        // USA O MESMO `dau_head` que alimenta o DAU.
        //
        // Recalcular a cabeca aqui criava discordancia de um ciclo com o que o
        // DAU usou para classificar idade — e a contabilidade nao fechava:
        // ha' 0,97 instrucoes PRONTAS por ciclo, mas nos 142.550 ciclos de
        // cabeca travada a sonda reportava ZERO alternativas. As prontas
        // "sumiam". Com a mesma referencia, os dois passam a falar da mesma
        // janela.
        rh    = dau_head;
        found = internal_window_data[rh].valid &
                ~internal_window_data[rh].dispatched;

        if (found && !internal_ready_flags[rh]) begin
            probe_head_stalled = 1'b1;
            for (int a = 0; a < WINDOW_SIZE; a++) begin
                if (internal_window_data[a].valid &&
                    !internal_window_data[a].dispatched &&
                    internal_ready_flags[a] &&
                    (a[$clog2(WINDOW_SIZE)-1:0] != rh)) begin
                    probe_has_alt   = 1'b1;
                    probe_alt_count = probe_alt_count + 5'd1;
                end
            end
        end
    end

    // JANELA EFETIVA DE ANALISE
    //
    // WINDOW_SIZE=16, mas com RESERVE_MODE as entradas RESERVADAS continuam
    // ocupando espaco ate' o commit. O DAU so' analisa as NAO despachadas —
    // se sobrarem poucas, nao ha' de onde tirar candidata, e nenhum criterio
    // de dispatch resolve.
    //
    // `probe_analisaveis` = entradas validas e nao despachadas (o que o DAU
    // realmente ve). Se a media for ~1-2, a janela util e' minuscula e o
    // caminho e' liberar espaco (commit mais cedo), nao afrouxar regra.
    logic [4:0] probe_analisaveis;
    always_comb begin
        probe_analisaveis = 5'd0;
        for (int q = 0; q < WINDOW_SIZE; q++)
            if (internal_window_data[q].valid && !internal_window_data[q].dispatched)
                probe_analisaveis = probe_analisaveis + 5'd1;
    end

    // Histograma da OCUPACAO do FEB (validas e nao despachadas). E' a medida
    // direta de "o buffer esta' cheio?" — sem ocupacao nao ha' janela e o
    // reorder nao tem de onde escolher.
    integer pr_anal_sum, pr_anal_ge2, pr_anal_ge4, pr_anal_zero, pr_anal_ge8, pr_anal_ge12;
    // Quantas PRONTAS existem na janela nos ciclos em que a cabeca trava.
    // Se der > 0 mas `com alternativa` continuar 0, a pronta E' a propria
    // cabeca — o que seria contradicao direta (ela esta' travada).
    integer pr_rdy_when_stall;
    // Os stalls do ID sao 63% LSU e 17% mult/div. Se a CABECA for a instrucao
    // de memoria/muldiv que vai travar, e houver uma ALU PURA pronta atras
    // dela, existe reordenacao possivel — e o teto seria a fracao desses
    // 898.553 ciclos.
    integer pr_head_mem, pr_head_mem_alu, pr_head_md, pr_head_md_alu;
    logic probe_head_is_mem, probe_head_is_md, probe_alu_ready;
    always_comb begin
        probe_head_is_mem = 1'b0;
        probe_head_is_md  = 1'b0;
        probe_alu_ready   = 1'b0;
        if (internal_window_data[dau_head].valid &&
            !internal_window_data[dau_head].dispatched) begin
            probe_head_is_mem = internal_window_data[dau_head].data.is_mem;
            probe_head_is_md  = internal_window_data[dau_head].data.is_multdiv;
            for (int m = 0; m < WINDOW_SIZE; m++)
                if (internal_window_data[m].valid &&
                    !internal_window_data[m].dispatched &&
                    internal_ready_flags[m] &&
                    (m[$clog2(WINDOW_SIZE)-1:0] != dau_head) &&
                    !internal_window_data[m].data.is_mem &&
                    !internal_window_data[m].data.is_branch &&
                    !internal_window_data[m].data.is_csr &&
                    !internal_window_data[m].data.is_syscall_atomic &&
                    !internal_window_data[m].data.is_multdiv)
                    probe_alu_ready = 1'b1;
        end
    end
    logic [4:0] probe_rdy_total;
    always_comb begin
        probe_rdy_total = 5'd0;
        for (int t = 0; t < WINDOW_SIZE; t++)
            if (internal_window_data[t].valid &&
                !internal_window_data[t].dispatched &&
                internal_ready_flags[t])
                probe_rdy_total = probe_rdy_total + 5'd1;
    end
    always_ff @(posedge clk) begin
        if (!rst_n) begin pr_anal_sum <= 0; pr_anal_ge2 <= 0; pr_anal_ge4 <= 0;
                          pr_anal_zero <= 0; pr_anal_ge8 <= 0; pr_anal_ge12 <= 0;
                          pr_rdy_when_stall <= 0;
                          pr_head_mem <= 0; pr_head_mem_alu <= 0;
                          pr_head_md <= 0; pr_head_md_alu <= 0; end
        else begin
            pr_anal_sum <= pr_anal_sum + 32'(probe_analisaveis);
            if (probe_analisaveis == 5'd0)  pr_anal_zero <= pr_anal_zero + 1;
            if (probe_analisaveis >= 5'd2)  pr_anal_ge2  <= pr_anal_ge2 + 1;
            if (probe_analisaveis >= 5'd4)  pr_anal_ge4  <= pr_anal_ge4 + 1;
            if (probe_analisaveis >= 5'd8)  pr_anal_ge8  <= pr_anal_ge8 + 1;
            if (probe_analisaveis >= 5'd12) pr_anal_ge12 <= pr_anal_ge12 + 1;
            if (probe_head_stalled)
                pr_rdy_when_stall <= pr_rdy_when_stall + 32'(probe_rdy_total);
            // "e trava" de verdade: exige a cabeca NAO pronta. Antes contava
            // so' "a cabeca e' MEM", e nesses ciclos ela estava pronta — ou
            // seja, despacha-la em ordem ja' era o otimo, nao havia o que
            // reordenar.
            if (probe_head_is_mem && probe_head_stalled) begin
                pr_head_mem <= pr_head_mem + 1;
                if (probe_alu_ready) pr_head_mem_alu <= pr_head_mem_alu + 1;
            end
            if (probe_head_is_md && probe_head_stalled) begin
                pr_head_md <= pr_head_md + 1;
                if (probe_alu_ready) pr_head_md_alu <= pr_head_md_alu + 1;
            end
        end
    end

    // O wrapper reporta `in=50` congelado com todas as travas em ZERO e
    // dvalid=3: o `ifa_disp_valid` nao sobe. Estes contadores dizem por que.
    //   d_en      : dispatcher sinalizou despacho
    //   d_ready   : ...e o consumidor estava pronto
    //   d_empty   : buffer visto como vazio
    //   d_headdisp: a cabeca (dau_head) esta' marcada `dispatched`
    //   d_feb_cnt : entradas validas no FEB
    integer d_en, d_ready, d_empty, d_headdisp, d_feb_cnt;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            d_en<=0; d_ready<=0; d_empty<=0; d_headdisp<=0; d_feb_cnt<=0;
        end else begin
            if (internal_dispatch_en)                d_en    <= d_en + 1;
            if (internal_dispatch_en & dispatch_ready_i) d_ready <= d_ready + 1;
            if (internal_buffer_empty)               d_empty <= d_empty + 1;
            if (internal_window_data[dau_head].valid &
                internal_window_data[dau_head].dispatched)
                                                     d_headdisp <= d_headdisp + 1;
            d_feb_cnt <= d_feb_cnt + 32'(probe_analisaveis);
        end
    end


    // ---------------------------------------------------------------------
    // POR QUE A CABECA TRAVADA NAO TEM ALTERNATIVA?
    //
    // Com a cabeca nao-pronta, varre em ordem de idade ate' a PRIMEIRA
    // ALU/MULDIV mais nova e classifica o que a impede:
    //   sem_candidata : nao ha' ALU/MULDIV nenhuma atras da cabeca
    //   barrada_mem   : ha' load/store entre a cabeca e ela
    //   barrada_ctrl  : ha' desvio/CSR/syscall entre a cabeca e ela
    //   dependente    : caminho livre, mas depende de algo mais velho
    //   pronta        : pronta — oportunidade real
    //   perdida       : pronta, e mesmo assim o despacho aceito foi outro
    // ---------------------------------------------------------------------
    integer st_sem, st_mem, st_ctrl, st_dep, st_pronta, st_perdida;
    // Composicao do buffer ATRAS da cabeca travada: somas por classe. Serve
    // para conferir o `sem` — se ele for alto com o buffer cheio, ou o
    // codigo e' mesmo so' memoria/desvio, ou alguma classificacao esta'
    // errada (ex.: pre-decode marcando ALU como barreira).
    integer cp_n, cp_mem, cp_br, cp_csr, cp_sys, cp_alu, cp_md;
    logic [4:0] c_mem, c_br, c_csr, c_sys, c_alu, c_md;
    always_comb begin
        c_mem = '0; c_br = '0; c_csr = '0; c_sys = '0; c_alu = '0; c_md = '0;
        for (int o = 1; o < WINDOW_SIZE; o++) begin
            logic [$clog2(WINDOW_SIZE)-1:0] k;
            k = dau_head + $unsigned(o[$clog2(WINDOW_SIZE)-1:0]);
            if (internal_window_data[k].valid && !internal_window_data[k].dispatched) begin
                if      (internal_window_data[k].data.is_syscall_atomic) c_sys = c_sys + 5'd1;
                else if (internal_window_data[k].data.is_csr)            c_csr = c_csr + 5'd1;
                else if (internal_window_data[k].data.is_branch)         c_br  = c_br  + 5'd1;
                else if (internal_window_data[k].data.is_mem)            c_mem = c_mem + 5'd1;
                else if (internal_window_data[k].data.is_multdiv)        c_md  = c_md  + 5'd1;
                else                                                     c_alu = c_alu + 5'd1;
            end
        end
    end
    logic [2:0] st_class;   // 0 sem,1 mem,2 ctrl,3 dep,4 pronta
    always_comb begin
        logic achou, viu_mem, viu_ctrl;
        st_class = 3'd0;
        achou = 1'b0; viu_mem = 1'b0; viu_ctrl = 1'b0;
        for (int o = 1; o < WINDOW_SIZE; o++) begin
            logic [$clog2(WINDOW_SIZE)-1:0] k;
            k = dau_head + $unsigned(o[$clog2(WINDOW_SIZE)-1:0]);
            if (!achou && internal_window_data[k].valid &&
                !internal_window_data[k].dispatched) begin
                if (!internal_window_data[k].data.is_mem &&
                    !internal_window_data[k].data.is_branch &&
                    !internal_window_data[k].data.is_csr &&
                    !internal_window_data[k].data.is_syscall_atomic) begin
                    achou = 1'b1;
                    if      (viu_ctrl)                   st_class = 3'd2;
                    else if (viu_mem)                    st_class = 3'd1;
                    else if (!internal_ready_flags[k])   st_class = 3'd3;
                    else                                 st_class = 3'd4;
                end else begin
                    if (internal_window_data[k].data.is_mem) viu_mem = 1'b1;
                    if (internal_window_data[k].data.is_branch ||
                        internal_window_data[k].data.is_csr    ||
                        internal_window_data[k].data.is_syscall_atomic) viu_ctrl = 1'b1;
                end
            end
        end
    end
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            st_sem <= 0; st_mem <= 0; st_ctrl <= 0; st_dep <= 0;
            st_pronta <= 0; st_perdida <= 0;
            cp_n <= 0; cp_mem <= 0; cp_br <= 0; cp_csr <= 0; cp_sys <= 0;
            cp_alu <= 0; cp_md <= 0;
        end else if (probe_head_stalled && dispatch_ready_i) begin
            cp_n   <= cp_n + 1;
            cp_mem <= cp_mem + 32'(c_mem);
            cp_br  <= cp_br  + 32'(c_br);
            cp_csr <= cp_csr + 32'(c_csr);
            cp_sys <= cp_sys + 32'(c_sys);
            cp_alu <= cp_alu + 32'(c_alu);
            cp_md  <= cp_md  + 32'(c_md);
            // Conta so' ciclos em que o host ACEITA despacho: com StallD alto
            // nao ha' escolha a fazer e a classificacao seria ruido.
            case (st_class)
                3'd0: st_sem    <= st_sem + 1;
                3'd1: st_mem    <= st_mem + 1;
                3'd2: st_ctrl   <= st_ctrl + 1;
                3'd3: st_dep    <= st_dep + 1;
                default: begin
                    st_pronta <= st_pronta + 1;
                    if (!reorder_o) st_perdida <= st_perdida + 1;
                end
            endcase
        end
    end


    // ---------------------------------------------------------------------
    // AMOSTRA DA JANELA: a partir do ciclo 1.500.000, imprime as 40 primeiras
    // vezes em que a cabeca trava com o decode livre. Para cada entrada NAO
    // despachada, em ordem de idade: PC (se o payload for o do Wally), a
    // instrucao, se o DAU a marcou pronta e os campos de dependencia. Serve
    // para ler contra o objdump e ver por que nada sai no lugar da cabeca.
    // ---------------------------------------------------------------------
    integer amostras;
    always_ff @(posedge clk) begin
        if (!rst_n) amostras <= 0;
        else if (dbg_ciclos > 1500000 && amostras < 40 &&
                 probe_head_stalled && dispatch_ready_i) begin
            amostras <= amostras + 1;
            $display("---- AMOSTRA %0d (ciclo %0d) ----", amostras, dbg_ciclos);
            for (int o = 0; o < WINDOW_SIZE; o++) begin
                logic [$clog2(WINDOW_SIZE)-1:0] k;
                k = dau_head + $unsigned(o[$clog2(WINDOW_SIZE)-1:0]);
                if (internal_window_data[k].valid && !internal_window_data[k].dispatched) begin
                    // O endereco nao esta' mais no payload: ele e' `base` do
                    // bloco mais o deslocamento da entrada (ver o pacote).
                    if (PAYLOAD_BASE_WIDTH >= 32)
                        $write("  base=%h ", le_base(k)[31:0]);
                    // A instrucao vem do payload do host, nao da janela: ela
                    // nao e' mais replicada no pacote de controle.
                    $display("i=%h %s %s%s%s%s  rd=%0d%s rs1=%0d%s rs2=%0d%s",
                        le_payload(k)[PAYLOAD_WIDTH-1 -: 32],
                        internal_ready_flags[k] ? "PRONTA" : "......",
                        internal_window_data[k].data.is_mem ?
                            (internal_window_data[k].data.writes_rd ? "L" : "S") : "-",
                        internal_window_data[k].data.is_branch ? "B" : "-",
                        internal_window_data[k].data.is_csr ? "C" : "-",
                        internal_window_data[k].data.is_syscall_atomic ? "X" : "-",
                        internal_window_data[k].data.rd,
                        internal_window_data[k].data.writes_rd ? "w" : " ",
                        internal_window_data[k].data.rs1,
                        internal_window_data[k].data.uses_rs1 ? "u" : " ",
                        internal_window_data[k].data.rs2,
                        internal_window_data[k].data.uses_rs2 ? "u" : " ");
                end
            end
        end
    end

    // Contadores para o dump PERIODICO (ver nota abaixo).
    integer dbg_ciclos, dbg_reorder, dbg_spec;

    // BURACO NO FLUSH — a hipotese de corrupcao do modo reordenado.
    //
    // `hole_o` marca o estado "existe entrada valida NAO despachada com uma
    // MAIS NOVA ja' despachada atras dela". Se um flush acontecer nesse
    // estado, a mais velha e' descartada com o buffer e o host rebusca a
    // partir do redirecionamento — que esta' ADIANTE dela. Ela nunca executa.
    //
    // `dbg_flush_buraco` conta os flushes nesse estado e `dbg_perdidas` soma
    // quantas entradas foram perdidas em cada um.
    integer dbg_flush_buraco, dbg_perdidas, dbg_flush_total;
    logic [4:0] n_perdidas;
    always_comb begin
        logic viu_pendente;
        n_perdidas   = 5'd0;
        viu_pendente = 1'b0;
        for (int off = 0; off < WINDOW_SIZE; off++) begin
            logic [$clog2(WINDOW_SIZE)-1:0] k;
            k = internal_head_ptr + $unsigned(off[$clog2(WINDOW_SIZE)-1:0]);
            if (internal_window_data[k].valid) begin
                if (!internal_window_data[k].dispatched) viu_pendente = 1'b1;
                else if (viu_pendente) n_perdidas = n_perdidas + 5'd1;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            dbg_ciclos <= 0; dbg_reorder <= 0; dbg_spec <= 0;
            dbg_flush_buraco <= 0; dbg_perdidas <= 0; dbg_flush_total <= 0;
        end else begin
            dbg_ciclos <= dbg_ciclos + 1;
            // So' conta reordenacao que o host ACEITOU. `reorder_o` sozinho
            // sobe tambem nos ciclos de StallD, em que a escolha e' refeita.
            if (reorder_o & internal_dispatch_en & dispatch_ready_i)
                dbg_reorder <= dbg_reorder + 1;
            if (dispatch_spec_o & internal_dispatch_en & dispatch_ready_i)
                dbg_spec <= dbg_spec + 1;
            if (flush_i) begin
                dbg_flush_total <= dbg_flush_total + 1;
                if (hole_o) begin
                    dbg_flush_buraco <= dbg_flush_buraco + 1;
                    dbg_perdidas     <= dbg_perdidas + 32'(n_perdidas);
                end
            end
        end
    end

    integer pr_head, pr_alt, pr_altsum;
    always_ff @(posedge clk) begin
        if (!rst_n) begin pr_head <= 0; pr_alt <= 0; pr_altsum <= 0; end
        else begin
            if (probe_head_stalled) pr_head <= pr_head + 1;
            if (probe_has_alt)      pr_alt  <= pr_alt + 1;
            pr_altsum <= pr_altsum + 32'(probe_alt_count);
        end
    end
    // ======================================================================
    // DUMP PERIODICO, nao em `final`.
    //
    // O testbench do Wally termina com `$stop` (testbench.sv:413). Nesse caso
    // os blocos `final` sao ignorados pelo simulador e as metricas nunca
    // apareciam, dando a impressao de que o IFA nao estava ativo.
    //
    // Mesma armadilha encontrada no CVA6, cujo testharness usa exit().
    // ======================================================================
    always_ff @(posedge clk) begin
      if (rst_n && (dbg_ciclos % 1000000 == 0) && (dbg_ciclos != 0)) begin
        $display("");
        $display("======== IFA / WALLY @ %0d ciclos ========", dbg_ciclos);
        $display("[FEB ] disp_en=%0d ready=%0d empty=%0d head_disp=%0d soma_validas=%0d",
                 d_en, d_ready, d_empty, d_headdisp, d_feb_cnt);
        $display("[IFA ] REORDER efetivado     : %0d  (dos quais hoist especulativo: %0d)",
                 dbg_reorder, dbg_spec);
        $display("[IFA ] FLUSH total=%0d  COM BURACO=%0d  entradas PERDIDAS=%0d",
                 dbg_flush_total, dbg_flush_buraco, dbg_perdidas);
        $display("[PROBE] cabeca travada        : %0d", pr_head);
        $display("[PROBE]   ...com alternativa  : %0d", pr_alt);
        $display("[PROBE]   soma de alternativas: %0d", pr_altsum);
        // `soma/ciclos` da' a ocupacao media. Nao multiplicar por 1000 antes:
        // a soma passa de 2^31 e o inteiro estoura (deu media negativa).
        $display("[OCUP] media=%0d (soma=%0d)  VAZIO=%0d  >=2: %0d  >=4: %0d  >=8: %0d  >=12: %0d  (de %0d ciclos)",
                 pr_anal_sum / (dbg_ciclos == 0 ? 1 : dbg_ciclos), pr_anal_sum,
                 pr_anal_zero, pr_anal_ge2, pr_anal_ge4, pr_anal_ge8, pr_anal_ge12,
                 dbg_ciclos);
        $display("[PROBE] PRONTAS quando a cabeca trava: soma=%0d", pr_rdy_when_stall);
        $display("[TRAVA] cabeca travada c/ decode livre -> 1a ALU/MULDIV atras: sem=%0d  barrada_MEM=%0d  barrada_CTRL=%0d  dependente=%0d  PRONTA=%0d (perdida pelo dispatcher=%0d)",
                 st_sem, st_mem, st_ctrl, st_dep, st_pronta, st_perdida);
        $display("[COMP] em %0d ciclos de cabeca travada, soma de entradas ATRAS: mem=%0d desvio=%0d csr=%0d sys/fp/atomic=%0d alu=%0d muldiv=%0d",
                 cp_n, cp_mem, cp_br, cp_csr, cp_sys, cp_alu, cp_md);
        $display("[PROBE] cabeca e' MEM e trava    : %0d", pr_head_mem);
        $display("[PROBE]   ...com ALU pura pronta : %0d  <-- alvo do stall de LSU", pr_head_mem_alu);
        $display("[PROBE] cabeca e' MULDIV e trava : %0d", pr_head_md);
        $display("[PROBE]   ...com ALU pura pronta : %0d", pr_head_md_alu);
        $display("=========================================");
      end
    end
`endif

    // Com BUFFER_PURO, o DAU e' ignorado: tudo pronto, sempre.
    dependency_analysis_unit u_dau (
        .window_data (internal_window_data),
        .head_ptr    (dau_head),
        .reg_busy    (reg_busy),
        .lsu_busy    (lsu_busy_i),
        // Usa o flag do IFA (marcado NO DESPACHO) OU o do host. O do host
        // sobe tarde — quando ele sobe, o ID ja' esta' ocupado e nao ha' para
        // onde mandar a alternativa.
        // `host_stalled_i` = o ID nao aceita agora (back_end_ready=0).
        //
        // MEDIDO: em 73.696 ciclos a cabeca e' uma MUL/DIV com ALU pura pronta
        // atras — mas `multdiv_inflight` era 0 nesses casos (ele so' sobe
        // depois que UMA MUL foi despachada; quando a cabeca E' a primeira,
        // nao ha' nenhuma em voo). Por isso a regra nunca disparava e a
        // sonda por ready_flags reportava so' 74.
        //
        // A condicao correta e': a cabeca e' MUL/DIV **e o host esta'
        // ocupado agora**. Nesse instante despachar a MUL so' a coloca numa
        // fila — melhor mandar a ALU, que o host vai poder executar.
        // `host_stalled_i` REMOVIDO da condicao.
        //
        // Ele levou as candidatas de 74 para 10.156 — mas o rastreio do
        // handshake mostrou que 10.082 delas morrem no `slot_free` (o inflight
        // esta' ocupado exatamente quando a oportunidade aparece). Sem a fila
        // de saida, que se mostrou incompativel com RESERVE_MODE, essas
        // escolhas nao podem ser capturadas: adiar a MUL so' acrescenta
        // latencia sem entregar a alternativa.
        // `host_stalled_i` de volta: com a fila de saida ha' onde capturar a
        // alternativa. Ele leva as candidatas de 74 para 10.156 (medido).
        // `host_stalled_i` ativo: com a fila de 2 e commit por indice, ha' um
        // segundo slot onde capturar a alternativa. Ele leva as candidatas de
        // 74 para 10.156 (medido).
        .multdiv_busy(multdiv_busy_i | multdiv_inflight_i | host_stalled_i),
        .ready_flags (dau_ready_raw),
        .spec_ready_flags (dau_spec_ready)
    );

    // FIFO_MODE propagado: com 1, so' a `dau_head` (primeira PENDENTE) pode
    // ser escolhida — despacho estritamente em ordem.
    // FIFO_MODE TEM de ser repassado: o padrao do dispatcher_unit e' 0
    // (reordenar), entao sem esta ligacao ate' o build "FIFO" reordena.
    dispatcher_unit #(.FIFO_MODE(FIFO_MODE)) u_du (
        .window_data    (internal_window_data),
        .head_ptr       (dau_head),
        .ready_flags    (internal_ready_flags),
        .spec_ready_flags (dau_spec_ready),
        .hoist_pend     (hoist_pend_i),
        .dispatch_spec  (dispatch_spec_o),
        .buffer_empty   (internal_buffer_empty),
        .multdiv_inflight (multdiv_inflight_i),
        .mem_inflight     (mem_inflight_i),
        .dispatch_en    (internal_dispatch_en),
        .reorder_o      (reorder_o),
        .dbg_cand_ready (dbg_cand_ready),
        .dbg_blk_unsafe (dbg_blk_unsafe),
        .dbg_blk_barrier(dbg_blk_barrier),
        .dbg_notready   (dbg_notready),
        .dbg_head_notready(dbg_head_notready),
        .dbg_head_alt   (dbg_head_alt),
        .dbg_any_ready  (dbg_any_ready),
        .dbg_ready_nothead(dbg_ready_nothead),
        .dbg_lost_barrier (dbg_lost_barrier),
        .dbg_avail_raw    (dbg_avail_raw),
        .dispatch_index (internal_dispatch_index),
        .dispatch_en2   (internal_dispatch_en2),
        .dispatch_index2(internal_dispatch_index2),
        .dispatch_inst2 (unused_disp_inst2),
        .dispatch_inst  (unused_disp_inst)
    );

    // =========================================================================
    // Saída Escalar para o ID_STAGE
    // =========================================================================
    // Em RESERVE_MODE a entrada permanece no FEB depois de escolhida. Quem
    // impede a reapresentacao e' o DISPATCHER (ele ja' ignora `dispatched`),
    // que le' o registrador — sem laco combinacional. Nao mascarar aqui:
    // `valid -> ready -> dispatched -> valid` trava o pipeline.
    assign feb_empty_o        = ~(|feb_any_valid);

    always_comb begin
        dbg_ocup = 5'd0;
        for (int o = 0; o < WINDOW_SIZE; o++)
            if (internal_window_data[o].valid && !internal_window_data[o].dispatched)
                dbg_ocup = dbg_ocup + 5'd1;
    end

    always_comb begin
        logic seen_pending;
        hole_o       = 1'b0;
        seen_pending = 1'b0;
        for (int off = 0; off < WINDOW_SIZE; off++) begin
            logic [$clog2(WINDOW_SIZE)-1:0] k;
            k = internal_head_ptr + $unsigned(off[$clog2(WINDOW_SIZE)-1:0]);
            if (internal_window_data[k].valid) begin
                if (!internal_window_data[k].dispatched) seen_pending = 1'b1;
                else if (seen_pending)                   hole_o       = 1'b1;
            end
        end
    end
    assign dispatch_valid_o   = internal_dispatch_en;
    // A segunda so' e' apresentada se a PRIMEIRA tambem estiver sendo aceita.
    //
    // Sem isso, num ciclo em que a porta 0 e' recusada e a 1 aceita, a
    // instrucao mais nova sairia sem a mais velha — ordem de programa
    // quebrada, e o FEB marcaria a errada como despachada.
    assign dispatch_valid2_o   = internal_dispatch_en2 & internal_dispatch_en &
                                 dispatch_ready_i;
    // LEITURA BANCARIZADA: todos os bancos leem a MESMA posicao (`idx / N`) e
    // um unico mux N:1 escolhe o banco (`idx % N`). Um mux no total, no lugar
    // de um por entrada.
    function automatic logic [PAYLOAD_WIDTH-1:0] le_payload(
            input logic [$clog2(WINDOW_SIZE)-1:0] idx);
        return payload_mem[int'(idx) % NBANK][BADDR'(int'(idx) / NBANK)];
    endfunction
    function automatic logic [PAYLOAD_BASE_WIDTH-1:0] le_base(
            input logic [$clog2(WINDOW_SIZE)-1:0] idx);
        return base_mem[int'(idx) % NBANK][BADDR'(int'(idx) / NBANK)];
    endfunction

    assign dispatch_payload2_o = le_payload(internal_dispatch_index2);
    assign dispatch_index_o   = internal_dispatch_index;
    assign dispatch_payload_o = le_payload(internal_dispatch_index);
    // Entrada sem previsao propria (extra de bloco) le' zero — e' o que o
    // host espera de uma instrucao que, por construcao, nao e' desvio.
    assign dispatch_payload_bp_o = paybp_val[internal_dispatch_index] ?
                                   paybp_mem[internal_dispatch_index] : '0;
    assign dispatch_base_o       = le_base(internal_dispatch_index);
    assign dispatch_seq_o     = internal_window_data[internal_dispatch_index].data.seq;
    assign push_seq_o         = seq_q;

endmodule
