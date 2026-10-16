// =============================================================================
// REGRA: toda instrucao que NAO depende de outra mais velha pode sair antes
// dela. O que impede uma instrucao `i` de sair antes de uma mais velha `j`
// ainda nao despachada e' somente:
//
//   REGISTRADOR  RAW, WAR ou WAW entre as duas.
//   MEMORIA      as duas acessam memoria e uma delas e' STORE. Sem o endereco
//                no despacho, nao da' para provar que nao se sobrepoem; load
//                passando load e' livre.
//   DESVIO       `j` e' um desvio ainda NAO despachado. O que vem depois dele
//                e' caminho especulativo: no Wally, algo despachado antes do
//                desvio chega ao W antes de ele resolver, e nao ha' desfazer.
//                (O contrario e' livre: um desvio pode sair antes de mais
//                velhas — o host faz flush PARCIAL se ele errar.)
//   SERIALIZANTE CSR, syscall, atomico e ponto flutuante: nada passa por
//                eles e eles so' saem com nada mais velho pendente.
//
// Load e store NAO sao barreira para instrucoes sem memoria, e o desvio nao e'
// obrigado a sair em ordem.
//
// EXCECAO PRECISA (necessaria para Linux, onde page fault e' rotina):
//   - load/store sai em ordem em relacao as mais velhas — se passasse uma e
//     faltasse, a mais velha nunca executaria. Excecao: o HOIST ESPECULATIVO
//     a frente do desvio da cabeca, que tem replay proprio no host.
//   - quem passa um load/store mais velho tem de ser REEXECUTAVEL, porque o
//     retorno do trap desse load o executa de novo.
//   - instrucao com falta de BUSCA chega como serializante (o campo de
//     instrucao e' lixo; ver `fetch_serialize_i` no ifa_top).
// =============================================================================
`include "ifa_pkg.sv"

import ifa_pkg::*;

module dependency_analysis_unit (
    input  ifa_feb_entry_t [WINDOW_SIZE-1:0] window_data,
    input  logic [$clog2(WINDOW_SIZE)-1:0]   head_ptr,
    input  logic [31:0]                      reg_busy,
    // ---- Recursos do HOST ocupados ----
    // O `reg_busy` so' conhece dependencia de DADOS. Mas o que trava o ID do
    // Ibex e' RECURSO: LSU 703k ciclos, mult/div 195k, branch/jump 224k.
    // MEDIDO: em 757.245 ciclos o ID estava travado E havia instrucao pronta
    // na janela — oportunidade que o DAU nao enxergava.
    //
    // Marcando como NAO-PRONTA a instrucao que precisa de um recurso ocupado,
    // a cabeca fica travada de verdade e o dispatcher procura uma ALU
    // independente. E' escalonamento, nao correcao: se o recurso esta' ocupado
    // ela travaria no ID de qualquer forma.
    input  logic                             lsu_busy,
    input  logic                             multdiv_busy,
    output logic [WINDOW_SIZE-1:0]           ready_flags,
    // PRONTA ESPECULATIVA: ALU/MUL que estaria pronta se o desvio na CABECA
    // nao fosse barreira. So' faz sentido quando a cabeca e' um desvio
    // travado; o dispatcher usa isto para o hoist especulativo de 1 nivel
    // (ver dispatcher_unit). Todas as dependencias de registrador contra a
    // cabeca e contra as demais mais velhas continuam valendo.
    output logic [WINDOW_SIZE-1:0]           spec_ready_flags
);

    // Cabeca e' um desvio puro (nao CSR/syscall/memoria)?
    logic cabeca_desvio;
    assign cabeca_desvio = window_data[head_ptr].valid &&
                           !window_data[head_ptr].dispatched &&
                           window_data[head_ptr].data.is_branch &&
                           !window_data[head_ptr].data.is_csr &&
                           !window_data[head_ptr].data.is_syscall_atomic &&
                           !window_data[head_ptr].data.is_mem;
    localparam int IDX_W = $clog2(WINDOW_SIZE);

    logic [IDX_W-1:0] i_u, j_u;
    logic             stall, j_is_older, j_can_fault, i_can_fault;
    logic             barr, barr_nc;  // barreira de desvio/serializante; `_nc` ignora o desvio da cabeca
    logic             i_reexec;       // `i` pode ser executada duas vezes sem mudar o resultado
    logic             i_is_store, j_is_store;

    always_comb begin
        ready_flags = '0;
        spec_ready_flags = '0;
        barr = 1'b0; barr_nc = 1'b0; i_reexec = 1'b0;
        i_u = '0; j_u = '0; stall = 1'b0; j_is_older = 1'b0; j_can_fault = 1'b0;
        i_can_fault = 1'b0;
        i_is_store = 1'b0; j_is_store = 1'b0;

        for (int i = 0; i < WINDOW_SIZE; i++) begin
            i_u   = i[IDX_W-1:0];
            stall = 1'b0;
            barr  = 1'b0;
            barr_nc = 1'b0;

            if (window_data[i].valid && !window_data[i].dispatched) begin

                // STORE = acesso a memoria que NAO escreve registrador inteiro.
                // Nao usar `instruction[6:0] == 0100011`: isso so' reconhece
                // store de 32 bits, e `c.sw`/`c.sd`/`c.swsp`/`c.sdsp` passariam
                // por load — e poderiam trocar de ordem com outro acesso a' mesma
                // posicao. (Load de ponto flutuante tambem cai aqui; e' so'
                // conservador.)
                i_is_store = window_data[i].data.is_mem && !window_data[i].data.writes_rd;

                // i é faltável crítico? (Barreiras de sistema/controle fortes)
                // `is_mem` E' OBRIGATORIO aqui em host SEM reorder buffer.
                //
                // O comentario original ("removido para permitir bypass
                // agressivo") vale no CVA6, que pode desfazer estado. No Ibex
                // nao ha' nada para desfazer: se um load/store despachado fora
                // de ordem faltar DEPOIS que uma instrucao mais nova commitou,
                // a excecao e' imprecisa e o estado fica corrompido.
                // Sintoma medido ao adiar memoria sem esta linha: MCAUSE=0x5
                // (load access fault) com MTVAL=0x20 — ponteiro destruido.
                // `i` SERIALIZANTE: so' sai com nada mais velho pendente.
                i_can_fault = window_data[i].data.is_csr |
                              window_data[i].data.is_syscall_atomic;

                // REEXECUTAVEL: se um load/store MAIS VELHO que `i` faltar
                // depois de `i` ja' ter executado, o retorno do trap executa
                // `i` de novo. So' e' seguro se o resultado for o mesmo: sem
                // memoria, sem CSR/syscall, e `rd` fora das fontes
                // (`addi a5,a5,1` nao pode; `jalr ra,0(ra)` tambem nao).
                i_reexec = !window_data[i].data.is_mem &&
                           !window_data[i].data.is_csr &&
                           !window_data[i].data.is_syscall_atomic &&
                           !(window_data[i].data.writes_rd && window_data[i].data.rd != 5'd0 &&
                             ((window_data[i].data.uses_rs1 &&
                               window_data[i].data.rs1 == window_data[i].data.rd) ||
                              (window_data[i].data.uses_rs2 &&
                               window_data[i].data.rs2 == window_data[i].data.rd)));

                // ---- (0) RECURSO do host ocupado ----
                if (window_data[i].data.is_mem && lsu_busy)
                    stall = 1'b1;

                // ---- (1) RAW pendente no back-end (trava a head se o dado não está pronto) ----
                if (window_data[i].data.uses_rs1 && window_data[i].data.rs1 != 5'd0 &&
                    reg_busy[window_data[i].data.rs1])
                    stall = 1'b1;
                if (window_data[i].data.uses_rs2 && window_data[i].data.rs2 != 5'd0 &&
                    reg_busy[window_data[i].data.rs2])
                    stall = 1'b1;

                // ---- (2)+(3) Hazards/ordem contra mais velhas NÃO despachadas no Buffer ----
                for (int j = 0; j < WINDOW_SIZE; j++) begin
                    j_u = j[IDX_W-1:0];
                    if (i != j && window_data[j].valid && !window_data[j].dispatched) begin

                        if (head_ptr <= i_u)
                            j_is_older = (j_u >= head_ptr) && (j_u < i_u);
                        else
                            j_is_older = (j_u >= head_ptr) || (j_u < i_u);

                        if (j_is_older) begin
                            // Descodificação do Store da instrução mais velha
                            j_is_store = window_data[j].data.is_mem && !window_data[j].data.writes_rd;

                            // `j` e' BARREIRA: desvio ainda nao despachado ou
                            // serializante. Memoria NAO e' barreira aqui — a
                            // ordem entre acessos e' tratada logo abaixo.
                            j_can_fault = window_data[j].data.is_branch |
                                          window_data[j].data.is_csr    |
                                          window_data[j].data.is_syscall_atomic;

                            // RAW: i lê o rd de j
                            if (window_data[j].data.writes_rd && window_data[j].data.rd != 5'd0) begin
                                if (window_data[i].data.uses_rs1 &&
                                    window_data[i].data.rs1 == window_data[j].data.rd) stall = 1'b1;
                                if (window_data[i].data.uses_rs2 &&
                                    window_data[i].data.rs2 == window_data[j].data.rd) stall = 1'b1;
                            end

                            // WAR: i escreve num reg que j (mais velha) lê
                            if (window_data[i].data.writes_rd && window_data[i].data.rd != 5'd0) begin
                                if (window_data[j].data.uses_rs1 &&
                                    window_data[j].data.rs1 == window_data[i].data.rd) stall = 1'b1;
                                if (window_data[j].data.uses_rs2 &&
                                    window_data[j].data.rs2 == window_data[i].data.rd) stall = 1'b1;
                            end

                            // WAW: i e j escrevem no mesmo rd
                            if (window_data[i].data.writes_rd && window_data[i].data.rd != 5'd0 &&
                                window_data[j].data.writes_rd && window_data[j].data.rd != 5'd0 &&
                                window_data[i].data.rd == window_data[j].data.rd) stall = 1'b1;

                            // Ordenação de Memória Relaxada (Load-Bypass-Load)
                            // Apenas gera stall se Pelo Menos Uma for STORE
                            if (window_data[i].data.is_mem && window_data[j].data.is_mem && 
                                (i_is_store || j_is_store)) stall = 1'b1;

                            // BARREIRA de `j` fica separada das dependencias:
                            // `barr` entra no `ready` normal; `barr_nc` e' a
                            // mesma coisa, exceto quando `j` e' o desvio da
                            // CABECA — e' ela que o hoist especulativo ignora.
                            if (j_can_fault) begin
                                barr = 1'b1;
                                if (!(cabeca_desvio && (j_u == head_ptr))) barr_nc = 1'b1;
                            end

                            // EXCECAO PRECISA (Linux) — dois casos:
                            //
                            // (a) `i` e' load/store: sai em ORDEM. Se ele
                            //     passasse `j` e faltasse, o `mepc` apontaria
                            //     para ele e `j` (mais velha) nunca executaria.
                            //     Unica excecao: o hoist especulativo a frente
                            //     do desvio da cabeca, que tem replay proprio
                            //     (por isso vai so' em `barr`, nao em `barr_nc`
                            //     quando `j` e' esse desvio).
                            if (window_data[i].data.is_mem) begin
                                barr = 1'b1;
                                if (!(cabeca_desvio && (j_u == head_ptr))) barr_nc = 1'b1;
                            end
                            // (b) `j` e' load/store e `i` nao e' reexecutavel:
                            //     se `j` faltar depois, o retorno do trap roda
                            //     `i` de novo.
                            if (window_data[j].data.is_mem && !i_reexec) stall = 1'b1;

                            // Exceção Precisa: I faltável despacha em ordem
                            if (i_can_fault) stall = 1'b1;
                        end
                    end
                end // j

                ready_flags[i] = ~(stall | barr);

                // Especulativa: sem dependencia nenhuma, so' barrada pelo
                // desvio da cabeca. ALU, MUL/DIV ou LOAD inteiro.
                //
                // LOAD pode porque um load no M que recebe FlushW no PRIMEIRO
                // ciclo nao chega a' memoria: o `busfsm` bloqueia o HTRANS com
                // `Flush` e a D-cache aborta pelo `FlushStage` — o mesmo
                // caminho que o trap usa. Entao nem MMIO e' tocado pelo caminho
                // errado. Se o load FALTAR no caminho certo, o host faz replay
                // (ver `IFAReplayM` no wallypipelinedcore).
                //
                // STORE nao: escreve a memoria. Load de ponto flutuante e AMO
                // ficam de fora pelas outras classes.
                spec_ready_flags[i] = ~(stall | barr_nc) & (i_u != head_ptr) &
                                      !(window_data[i].data.is_mem &&
                                        !window_data[i].data.writes_rd) &&
                                      !window_data[i].data.is_branch &&
                                      !window_data[i].data.is_csr &&
                                      !window_data[i].data.is_syscall_atomic;
            end
        end // i
    end
endmodule
