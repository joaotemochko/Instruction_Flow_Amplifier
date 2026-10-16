`include "ifa_pkg.sv"

import ifa_pkg::*;

module dispatcher_unit #(
    // Emissao dupla (host superescalar). Ver nota no segundo scan.
    parameter bit DUAL_ISSUE = 1'b0,
    // 1 = despacho estritamente EM ORDEM: so' a CABECA pode ser escolhida.
    //
    // Isola o ganho do BUFFER (§III-A) do ganho da REORDENACAO (§III-C). Com
    // FIFO_MODE=1 o IFA vira um buffer de desacoplamento puro.
    parameter bit FIFO_MODE  = 1'b0
) (
    input  ifa_feb_entry_t [WINDOW_SIZE-1:0] window_data,
    input  logic [$clog2(WINDOW_SIZE)-1:0] head_ptr,
    input  logic [WINDOW_SIZE-1:0] ready_flags,
    // ALU/MUL que so' e' barrada pelo desvio na cabeca (ver DAU).
    input  logic [WINDOW_SIZE-1:0] spec_ready_flags,
    // 1 = no despacho ANTERIOR saiu uma instrucao especulativa: agora tem de
    //     sair a cabeca (o desvio), e nada mais.
    input  logic hoist_pend,
    // 1 = a escolha deste ciclo e' um hoist especulativo.
    output logic dispatch_spec,
    input  logic buffer_empty,
    // 1 = ja' ha' uma MUL/DIV despachada e ainda nao concluida. O dispatcher
    // evita escolher outra que precise do mesmo recurso e procura uma ALU
    // independente — que e' onde o ganho de reordenacao aparece (MatMult).
    input  logic multdiv_inflight,
    // 1 = ja' ha' um LOAD/STORE despachado e ainda nao concluido. O LSU e' a
    // maior fonte de bloqueio do Ibex (703.227 ciclos, 63% do `ocupado`).
    // Marcando no IFA (nao esperando o sinal do host), o dispatcher escolhe
    // uma ALU independente enquanto a memoria responde.
    input  logic mem_inflight,

    output logic dispatch_en,
    // 1 = a escolha PULOU alguma instrucao mais velha ainda NAO reservada.
    //
    // NAO usar `idx != head_ptr`: com RESERVE_MODE o `head_ptr` so' avanca no
    // commit, entao a cabeca fica reservada no buffer e o dispatcher escolhe a
    // posicao seguinte — em ORDEM DE PROGRAMA, mas com idx != head. Esse
    // criterio reportava 90% de reordenacao (1.890.681 de 2.098.265) enquanto
    // a execucao era identica ao baseline, o que e' impossivel.
    output logic reorder_o,
    // ---- Diagnostico: por que a candidata nao e' escolhida ----
    // dbg_cand_ready : havia entrada PRONTA depois de uma nao-pronta
    // dbg_blk_unsafe : ...mas foi barrada por skipped_unsafe (mem/branch/csr
    //                  no caminho) — indica que a barreira de excecao precisa
    //                  ser mais fina
    // dbg_blk_barrier: ...ou por barrier_active (branch ja' despachado)
    output logic dbg_cand_ready,
    output logic dbg_blk_unsafe,
    output logic dbg_blk_barrier,
    output logic [4:0] dbg_notready,   // entradas validas NAO-PRONTAS na janela
    // DECISIVO: a CABECA (mais velha nao despachada) esta' travada?
    //   dbg_head_notready : cabeca valida, nao despachada e NAO pronta
    //   dbg_head_alt      : ...e existe OUTRA entrada pronta na janela
    // Se head_notready ~ 0, o scan sempre pega a cabeca na 1a posicao, nunca
    // pula ninguem, e NENHUMA regra de dispatch pode gerar reordenacao.
    output logic dbg_head_notready,
    output logic dbg_head_alt,
    // Existe QUALQUER entrada valida, nao despachada e PRONTA na janela?
    // Combinado com "ID travado", da' o teto real de reordenacao.
    output logic dbg_any_ready,
    // Numero de entradas PRONTAS que NAO sao a cabeca. Resolve a contradicao
    // entre `dbg_head_alt` (deu 0) e `dbg_any_ready` (deu 756.110): se este
    // for sempre 0, a UNICA pronta e' a propria cabeca e nao ha' alternativa
    // nenhuma — nenhuma regra de dispatch pode inventar uma.
    output logic [4:0] dbg_ready_nothead,
    // Quantas entradas PRONTAS o scan encontrou mas nao pode escolher porque
    // um BRANCH ja' tinha levantado a barreira. Se for alto, a barreira de
    // controle e' o limitante — e nao a falta de candidata.
    output logic [4:0] dbg_lost_barrier,
    // Entradas PRONTAS apos a cabeca, ignorando qualquer barreira. E' o teto
    // absoluto de reordenacao neste ciclo.
    output logic [4:0] dbg_avail_raw,
    output logic [$clog2(WINDOW_SIZE)-1:0] dispatch_index,
    output ifa_ctrl_packet_t dispatch_inst,

    // ---- SEGUNDA ESCOLHA (host superescalar) ----
    // O CVA6 tem NrIssuePorts=2 e emite DUAS por ciclo. Entregar uma so'
    // SERIALIZA o core: medido 10.246 reorders com CoreMark 2,89 contra 2,93
    // do baseline — o custo da serializacao supera o ganho do reorder.
    //
    // Dependencia ENTRE as duas e' resolvida pelo `issue_read_operands` do
    // CVA6, que trata stall_raw[0] e stall_raw[1] separadamente.
    output logic                           dispatch_en2,
    output logic [$clog2(WINDOW_SIZE)-1:0] dispatch_index2,
    output ifa_ctrl_packet_t               dispatch_inst2
);

    logic found_dispatch;
    logic skipped_unsafe;   // pulei algo que pode faltar ou desviar
    logic is_pure_alu;
    logic can_bypass;
    logic barrier_active;
    logic skipped_insts;     // há instrução não-despachada antes da atual no scan
    logic [$clog2(WINDOW_SIZE)-1:0] idx;

    /* verilator lint_off LATCH */
    always_comb begin
        logic is_critical;
        logic conflict;

        dispatch_en      = 1'b0;
        reorder_o        = 1'b0;
        skipped_unsafe   = 1'b0;
        is_pure_alu      = 1'b0;
        can_bypass       = 1'b0;
        dbg_cand_ready   = 1'b0;
        dbg_blk_unsafe   = 1'b0;
        dbg_blk_barrier  = 1'b0;
        dbg_notready      = 5'd0;
        dbg_head_notready = 1'b0;
        dbg_head_alt      = 1'b0;
        dbg_any_ready     = 1'b0;
        dbg_ready_nothead = 5'd0;
        dbg_lost_barrier  = 5'd0;
        dbg_avail_raw     = 5'd0;
        for (int b = 0; b < WINDOW_SIZE; b++)
            if (window_data[b].valid && !window_data[b].dispatched &&
                ready_flags[b] && (b[$clog2(WINDOW_SIZE)-1:0] != head_ptr))
                dbg_ready_nothead = dbg_ready_nothead + 5'd1;
        for (int a = 0; a < WINDOW_SIZE; a++)
            if (window_data[a].valid && !window_data[a].dispatched &&
                ready_flags[a])
                dbg_any_ready = 1'b1;

        // ATENCAO: NAO usar `window_data[head_ptr]` como "cabeca".
        //
        // Com RESERVE_MODE a entrada permanece no FEB apos ser despachada e o
        // `head_ptr` so' avanca no COMMIT. Entao a posicao apontada por
        // head_ptr esta' quase sempre com dispatched=1, e a condicao
        // `!dispatched` nunca era satisfeita — o contador media a posicao do
        // ponteiro, nao o estado real (reportou 35 em 4,1M ciclos, enquanto
        // havia 5,75 entradas nao-prontas por ciclo).
        //
        // A CABECA REAL e' a mais velha AINDA NAO DESPACHADA, varrendo em
        // ordem de idade a partir de head_ptr.
        begin
            logic found_head;
            logic [$clog2(WINDOW_SIZE)-1:0] real_head;
            found_head = 1'b0;
            real_head  = head_ptr;
            for (int o = 0; o < WINDOW_SIZE; o++) begin
                logic [$clog2(WINDOW_SIZE)-1:0] k;
                k = head_ptr + $unsigned(o[$clog2(WINDOW_SIZE)-1:0]);
                if (!found_head && window_data[k].valid &&
                    !window_data[k].dispatched) begin
                    found_head = 1'b1;
                    real_head  = k;
                end
            end

            if (found_head && !ready_flags[real_head]) begin
                dbg_head_notready = 1'b1;
                for (int h = 0; h < WINDOW_SIZE; h++)
                    if (window_data[h].valid && !window_data[h].dispatched &&
                        ready_flags[h] &&
                        (h[$clog2(WINDOW_SIZE)-1:0] != real_head))
                        dbg_head_alt = 1'b1;
            end
        end

        for (int d = 0; d < WINDOW_SIZE; d++)
            if (window_data[d].valid && !window_data[d].dispatched &&
                !ready_flags[d])
                dbg_notready = dbg_notready + 5'd1;
        dispatch_index  = '0;
        dispatch_en2    = 1'b0;
        dispatch_index2 = '0;
        dispatch_inst2  = '0;
        dispatch_inst  = '{default: '0};
        found_dispatch = 1'b0;
        dispatch_spec  = 1'b0;
        barrier_active = 1'b0;
        skipped_insts  = 1'b0;
        is_critical    = 1'b0;
        conflict       = 1'b0;
        idx            = '0;

        if (!buffer_empty) begin
            for (int offset = 0; offset < WINDOW_SIZE; offset++) begin
                logic [$clog2(WINDOW_SIZE)-1:0] off_u;
                off_u    = $unsigned(offset[$clog2(WINDOW_SIZE)-1:0]);
                idx      = head_ptr + off_u;
                conflict = 1'b0;

                if (window_data[idx].valid && !window_data[idx].dispatched) begin

                    // ----------------------------------------------------------
                    // Instrução crítica = precisa de barreira total:
                    // · is_syscall_atomic: ECALL, EBREAK, WFI, MRET
                    //   (CSR* NÃO entra aqui depois do fix no ibex_core.sv)
                    // · is_branch: branch/JAL/JALR
                    // ----------------------------------------------------------
                    // `is_csr` ENTRA na lista (no Ibex tinha sido tirado).
                    //
                    // No Wally o `CSRWriteFenceM` limpa D/E/M e rebusca a
                    // partir do sucessor do CSR. Deixar uma instrucao mais
                    // velha para tras quando o CSR e' despachado faz essa
                    // instrucao ser descartada pelo flush e nunca rebuscada.
                    is_critical = window_data[idx].data.is_syscall_atomic ||
                                  window_data[idx].data.is_csr            ||
                                  window_data[idx].data.is_branch;

                    // A ORDEM e' decidida so' pelo DAU (ver o cabecalho dele):
                    // se ele marcou `ready_flags[idx]`, a instrucao nao depende
                    // de nenhuma mais velha pendente e pode sair. O dispatcher
                    // tinha regras proprias duplicadas (`is_pure_alu`,
                    // `skipped_unsafe`, barreira de critica) que recusavam
                    // candidatas que o DAU ja' tinha liberado.

                    // RECURSO: uma segunda MUL/DIV nao adianta nada enquanto a
                    // primeira ocupa o multiplicador. Marcando como conflito, o
                    // scan continua e pode achar uma ALU independente para
                    // ocupar o host nesse meio tempo. Seguro por construcao: a
                    // ALU escolhida ja' passou por RAW/WAR/WAW no DAU, nao e'
                    // especulativa (nenhum branch pendente a frente dela) e nao
                    // pode faltar.
                    if (window_data[idx].data.is_multdiv && multdiv_inflight) begin
                        conflict = 1'b1;
                        // NAO ativa barreira: conflito de RECURSO nao e'
                        // conflito de ORDEM. O scan deve continuar e achar uma
                        // ALU independente — que e' exatamente o ponto.
                    end

                    // Mesma ideia para a porta de memoria. Uma segunda
                    // instrucao de memoria nao progride enquanto a primeira
                    // ocupa o LSU — melhor ceder a vez a uma ALU independente.
                    //
                    // Seguranca: a ALU escolhida ja' passou por RAW/WAR/WAW no
                    // DAU, nao e' especulativa (a barreira de controle impede
                    // ultrapassar branch), nao acessa memoria e nao pode faltar.
                    // A ordem entre acessos a memoria e' preservada, porque a
                    // segunda so' e' adiada — nunca antecipada.
                    // DESATIVADO: adiar memoria exige que uma ALU possa
                    // ultrapassar um load pendente — e isso so' e' seguro com
                    // recuperacao de estado. Sem `is_mem` no can_fault do DAU
                    // a execucao corrompe (MCAUSE=0x5 medido); COM `is_mem`
                    // nada ultrapassa o load e o adiamento nao gera
                    // oportunidade. As duas pontas foram medidas.
                    //
                    // MUL/DIV nao tem esse problema: nao falta, entao a ALU que
                    // passa na frente nunca precisa ser desfeita.
                    if (1'b0 && window_data[idx].data.is_mem && mem_inflight) begin
                        conflict = 1'b1;
                    end

                    // Despacha se: sem conflito, sem barreira ativa,
                    // DAU marcou como ready, e ainda não despachamos ninguém.
                    // ALU PURA = nao mem, nao branch, nao csr, nao syscall,
                    // nao mul/div (o multiplicador e' recurso disputado).
                    is_pure_alu = !window_data[idx].data.is_mem     &&
                                  !window_data[idx].data.is_branch  &&
                                  !window_data[idx].data.is_csr     &&
                                  !window_data[idx].data.is_syscall_atomic &&
                                  !window_data[idx].data.is_multdiv;

                    // Uma ALU pura pode passar por cima de instrucoes apenas
                    // NAO-PRONTAS; qualquer outra continua exigindo que nada
                    // tenha sido pulado (comportamento anterior).
                    // AGRESSIVO: uma ALU pura so' e' barrada por algo que possa
                    // FALTAR ou DESVIAR (skipped_unsafe). Instrucoes apenas
                    // nao-prontas (esperando operando) NAO a bloqueiam — o DAU
                    // ja' garantiu RAW/WAR/WAW contra elas.
                    //
                    // Medido: com este criterio, UNSAFE=0 e BARREIRA=0 em 4,1M
                    // ciclos, ou seja, nenhuma candidata foi recusada. O
                    // limitante passou a ser a existencia de candidata (533),
                    // nao o criterio.
                    // MUL/DIV tambem pode ultrapassar. O dispatcher usava
                    // `is_pure_alu`, que EXCLUI mult/div, e recusava uma
                    // candidata que o DAU ja' tinha liberado — o DAU marca
                    // pronta, o dispatcher joga fora, a cabeca trava.
                    //
                    // Mult/div nao falta (divisao por zero em RISC-V devolve
                    // valor, nao trap), nao desvia e nao toca memoria: para
                    // efeito de ordem e' exatamente uma ALU que demora mais.
                    // RAW/WAR/WAW contra o que ficou para tras ja' foram
                    // checados no DAU.
                    can_bypass = 1'b1;   // a decisao e' do DAU
                    // Diagnostico: entrada PRONTA, apos algo pulado, que NAO
                    // foi escolhida — e o motivo.
                    if (ready_flags[idx] && !found_dispatch && skipped_insts) begin
                        dbg_cand_ready = 1'b1;
                        if (is_pure_alu && skipped_unsafe) dbg_blk_unsafe  = 1'b1;
                        if (barrier_active)                dbg_blk_barrier = 1'b1;
                    end

                    // Diagnostico: pronta, apos a cabeca, e nao escolhida.
                    if (ready_flags[idx] && !found_dispatch && skipped_insts) begin
                        dbg_avail_raw = dbg_avail_raw + 5'd1;
                        if (barrier_active) dbg_lost_barrier = dbg_lost_barrier + 5'd1;
                    end

                    // A MAIS VELHA PRONTA sai. Com FIFO_MODE, ou logo apos um
                    // hoist especulativo, so' a cabeca.
                    if (!conflict && ready_flags[idx] && !found_dispatch &&
                        (!(FIFO_MODE || hoist_pend) || (idx == head_ptr))) begin
                        found_dispatch = 1'b1;
                        dispatch_en    = 1'b1;
                        dispatch_index = idx;
                        dispatch_inst  = window_data[idx].data;
                        // `skipped_insts` conta apenas entradas NAO despachadas
                        // que ficaram para tras no scan — exatamente "pulei uma
                        // mais velha que ainda nao foi executada". E' o criterio
                        // correto de reordenacao.
                        reorder_o      = skipped_insts;
                    end

                    // ----------------------------------------------------------
                    // FIX 1: skipped_insts só sobe para entradas não-despachadas.
                    // Entradas com dispatched=1 já saíram para o Ibex — não
                    // representam uma instrução "na frente" para fins de barreira.
                    // ----------------------------------------------------------
                    // (a condição !window_data[idx].dispatched já está garantida
                    //  pelo if externo, então basta setar aqui se não despachamos
                    //  esta instrução agora)
                    if (!found_dispatch || dispatch_index != idx) begin
                        skipped_insts = 1'b1;

                        // MODO AGRESSIVO
                        //
                        // `skipped_unsafe` separa "pulei alguma coisa" de
                        // "pulei algo que pode FALTAR ou DESVIAR".
                        //
                        // Uma ALU pura pode ultrapassar instrucoes que estao
                        // apenas esperando operando (nao-prontas), porque:
                        //  · o DAU ja' garantiu RAW/WAR/WAW contra elas;
                        //  · ela nao acessa memoria nem desvia;
                        //  · ela nao pode gerar excecao, entao nao ha' o que
                        //    desfazer se alguma das puladas faltar depois.
                        //
                        // O que ela NAO pode ultrapassar e' algo que possa
                        // faltar (memoria, CSR, syscall) ou desviar (branch):
                        // nesses casos a excecao/flush ficaria imprecisa.
                        if (window_data[idx].data.is_mem     ||
                            window_data[idx].data.is_branch  ||
                            window_data[idx].data.is_csr     ||
                            window_data[idx].data.is_syscall_atomic) begin
                            skipped_unsafe = 1'b1;
                        end
                    end

                    // -------------------------------------------------------
                    // BARREIRA DE CONTROLE — com JANELA DE BYPASS
                    //
                    // Um branch/jump na cabeca bloqueava TUDO atras dele. Mas
                    // uma instrucao ALU PURA posterior pode executar antes do
                    // branch ser resolvido sem qualquer efeito observavel:
                    //   · nao escreve o PC (nao interfere no desvio);
                    //   · nao acessa memoria (nao antecipa efeito colateral);
                    //   · nao pode gerar excecao (nao quebra precisao);
                    //   · o DAU ja' garantiu RAW/WAR/WAW contra as mais velhas.
                    //
                    // Se o branch for TOMADO, o flush descarta o que veio
                    // depois — inclusive o resultado dessa ALU. Como ela nao
                    // tem efeito fora do register file e o Ibex re-executa a
                    // partir do alvo, isso e' seguro... EXCETO se ela escrever
                    // um registrador que o caminho correto vai LER. Por isso a
                    // regra abaixo exige `writes_rd == 0` OU que o destino seja
                    // recomputado pelo proprio caminho (nao verificavel aqui) —
                    // adotamos o criterio conservador: so' ALU SEM escrita
                    // arquitetural visivel, isto e', rd = x0.
                    //
                    // MEDIDO: stall de branch+jump = 223.915 ciclos. E' a maior
                    // fonte de bloqueio que NAO depende de reg_busy (que no
                    // Ibex e' sempre 0).
                    if (is_critical) begin
                        barrier_active = 1'b1;
                    end
                end
            end

            // -----------------------------------------------------------------
            // HOIST ESPECULATIVO DE 1 NIVEL
            //
            // Caso dominante no CoreMark: a CABECA e' um desvio que le' o valor
            // que acabou de ser carregado (`lbu a4; beqz a4`) e trava por um
            // ciclo. Logo atras dele ha' ALUs independentes, mas estao DEPOIS
            // do desvio — caminho previsto, ainda nao confirmado.
            //
            // Aqui uma delas (a mais velha marcada `spec_ready`) ocupa a bolha.
            // O desvio sai OBRIGATORIAMENTE no ciclo seguinte (`hoist_pend`
            // restringe a escolha a' cabeca), entao a ALU fica exatamente um
            // estagio a frente: quando o desvio resolve no E ela esta' no M, sem
            // ter escrito no banco. Se ele errar, o host a mata com FlushW.
            // So' ALU/MUL (sem memoria) — e' o que se mata no M sem efeito.
            // -----------------------------------------------------------------
            if (!FIFO_MODE && !found_dispatch && !hoist_pend &&
                window_data[head_ptr].valid && !window_data[head_ptr].dispatched &&
                window_data[head_ptr].data.is_branch && !ready_flags[head_ptr]) begin
                for (int s = 1; s < WINDOW_SIZE; s++) begin
                    logic [$clog2(WINDOW_SIZE)-1:0] k;
                    k = head_ptr + $unsigned(s[$clog2(WINDOW_SIZE)-1:0]);
                    if (!found_dispatch && window_data[k].valid &&
                        !window_data[k].dispatched && spec_ready_flags[k]) begin
                        found_dispatch = 1'b1;
                        dispatch_en    = 1'b1;
                        dispatch_index = k;
                        dispatch_inst  = window_data[k].data;
                        dispatch_spec  = 1'b1;
                        reorder_o      = 1'b1;
                    end
                end
            end

            // -----------------------------------------------------------------
            // VALVULA DE SEGURANCA (CVA6): se NADA ficou pronto mas o buffer
            // nao esta vazio, despacha a HEAD (a mais velha) assim mesmo.
            // No CVA6 isso e seguro — o scoreboard segura o operando pendente —
            // e impede o IFA de matar o core de fome (sem isso, head travada por
            // operando em voo + nenhuma jovem pronta => deadlock/starvation).
            // Mantem ordem de programa nesse caso (despacha a mais velha).
            // -----------------------------------------------------------------
            if (!found_dispatch &&
                window_data[head_ptr].valid && !window_data[head_ptr].dispatched) begin
                dispatch_en    = 1'b1;
                dispatch_index = head_ptr;
                dispatch_inst  = window_data[head_ptr].data;
                reorder_o      = 1'b0;   // valvula: sempre a mais velha
            end
        end

        // ==================================================================
        // SEGUNDO SCAN — a outra instrucao do par.
        //
        // Laco separado, depois de a primeira escolha estar fixada. Pega a
        // primeira PRONTA que nao seja ela, parando em instrucao critica
        // (syscall/CSR/branch nao podem ser emitidos junto com o que vem
        // depois).
        // ==================================================================
        // A segunda TEM que ser MAIS NOVA que a primeira.
        //
        // O `id_stage` do CVA6 assume que a porta 0 precede a porta 1 em
        // ordem de programa — o forwarding entre elas depende disso. Entregar
        // invertido corrompe a execucao (medido: "B Response Errored", escrita
        // em endereco invalido).
        //
        // O laco varre do head_ptr, entao se a primeira escolha foi uma
        // posicao adiante (reorder), as anteriores sao MAIS VELHAS e precisam
        // ser puladas. `depois_da_primeira` marca quando passamos por ela.
        // ==================================================================
        // EMISSAO DUPLA: parametrizada para isolar do reorder.
        //
        // Quebrou duas vezes com "B Response Errored" (escrita AXI em endereco
        // invalido), cada vez mais cedo. Duas variaveis mudaram juntas —
        // reordenacao E emissao dupla — e nao da' para saber qual quebrou.
        //
        // `DUAL_ISSUE = 0` desliga a segunda porta e volta ao estado que
        // rodava (10.246 reorders, CoreMark 2,89).
        // ==================================================================
        if (DUAL_ISSUE && dispatch_en) begin
            logic [$clog2(WINDOW_SIZE)-1:0] idx2;
            logic bar2, achou2, depois_da_primeira;
            bar2 = 1'b0; achou2 = 1'b0; depois_da_primeira = 1'b0;

            for (int off2 = 0; off2 < WINDOW_SIZE; off2++) begin
                idx2 = head_ptr + $unsigned(off2[$clog2(WINDOW_SIZE)-1:0]);

                if (idx2 == dispatch_index) depois_da_primeira = 1'b1;

                if (window_data[idx2].valid && !window_data[idx2].dispatched) begin
                    // DEPENDENCIA ENTRE AS DUAS DO PAR.
                    //
                    // O DAU valida cada entrada contra o BUFFER, nao uma contra
                    // a outra. Se a segunda le' o `rd` da primeira, ela nao pode
                    // ir no mesmo ciclo — o CVA6 trata stall_raw por porta, mas
                    // nao ha' forwarding dentro do mesmo estagio de issue.
                    //
                    // Tambem barramos WAW (mesmo destino) e WAR (a segunda
                    // escreve o que a primeira le').
                    if (!bar2 && !achou2 && ready_flags[idx2] &&
                        depois_da_primeira && (idx2 != dispatch_index) &&
                        !(dispatch_inst.writes_rd && (dispatch_inst.rd != 5'd0) &&
                          ((window_data[idx2].data.uses_rs1 &&
                            window_data[idx2].data.rs1 == dispatch_inst.rd) ||
                           (window_data[idx2].data.uses_rs2 &&
                            window_data[idx2].data.rs2 == dispatch_inst.rd) ||
                           (window_data[idx2].data.writes_rd &&
                            window_data[idx2].data.rd == dispatch_inst.rd))) &&
                        !(window_data[idx2].data.writes_rd &&
                          (window_data[idx2].data.rd != 5'd0) &&
                          ((dispatch_inst.uses_rs1 &&
                            dispatch_inst.rs1 == window_data[idx2].data.rd) ||
                           (dispatch_inst.uses_rs2 &&
                            dispatch_inst.rs2 == window_data[idx2].data.rd)))) begin
                        achou2          = 1'b1;
                        dispatch_en2    = 1'b1;
                        dispatch_index2 = idx2;
                        dispatch_inst2  = window_data[idx2].data;
                    end
                    if (window_data[idx2].data.is_syscall_atomic ||
                        window_data[idx2].data.is_csr            ||
                        window_data[idx2].data.is_branch) bar2 = 1'b1;
                end
            end
        end
    end
    /* verilator lint_on LATCH */

endmodule
