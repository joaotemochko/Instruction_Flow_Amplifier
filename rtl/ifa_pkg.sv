package ifa_pkg;
    // JANELA DE 32 (o paper preve 16 a 32).
    //
    // Medido com 16: apenas 533 ciclos em 4,1M tiveram uma instrucao PRONTA
    // atras de uma travada — e ZERO foram barradas por regra do dispatcher
    // (UNSAFE=0, BARREIRA=0). Ou seja, o limitante nao e' o criterio de
    // despacho, e' a AUSENCIA de candidata dentro do alcance.
    //
    // Em codigo compilado com cadeia de dependencia longa, a proxima instrucao
    // independente pode estar alem da posicao 16. Dobrar a janela e' o unico
    // parametro que muda o CONJUNTO de candidatas, em vez de mudar quem e'
    // aceito — afrouxar mais o dispatcher nao teria efeito, ja' que ele nunca
    // recusou nada.
    //
    // Custo: o DAU e' O(n²) em comparacoes (32x32 = 1024 contra 256). Em
    // silicio isso pesa; aqui interessa medir se a oportunidade existe.
    // MEDIDO: 32 rendeu apenas 96 candidatas a mais (533 -> 629) e ZERO
    // reorders adicionais, ao custo de 4x em comparacoes no DAU (O(n^2)).
    // A oportunidade nao estava fora de alcance — ela nao existe no codigo.
    localparam WINDOW_SIZE     = 16;
    localparam REG_ADDR_WIDTH  = 5;

    // Pacote de controle: SO os campos usados pela analise de dependencia.
    // O payload real (fetch_entry_t do CVA6) NAO trafega aqui — ele vai pela
    // shadow memory parametrizada em ifa_top. Removido o antigo campo
    // payload[255:0] (era storage morto: 256b x 16 entradas).
    // ------------------------------------------------------------------
    // ESTADO DE UMA ENTRADA DO FEB
    //
    // O bit `dispatched` sozinho nao distingue os tres destinos possiveis de
    // uma instrucao que saiu do dispatcher, e cada um precisa de tratamento
    // diferente no flush:
    //
    //   FEB_PENDING  no buffer, ainda nao escolhida
    //                -> preservavel se anterior ao desvio
    //   FEB_IN_QUEUE escolhida e retida na fila do consumidor
    //                -> a fila decide; o FEB nao pode preservar nem
    //                   redespachar
    //   FEB_IN_HOST  entregue ao ID, executando
    //                -> NAO pode ser reexecutada em hipotese alguma
    //   FEB_DONE     concluida, aguardando commit
    //                -> so' ocupa espaco ate' o commit chegar
    //
    // As quatro tentativas de flush parcial falharam por tratar IN_QUEUE e
    // IN_HOST como se fossem a mesma coisa (`dispatched`), o que gerava
    // reexecucao (Stores acima do que o programa executa) ou preservacao de
    // caminho invalido (MULs despachadas = 0).
    // ------------------------------------------------------------------
    typedef enum logic [1:0] {
        FEB_PENDING  = 2'b00,
        FEB_IN_QUEUE = 2'b01,
        FEB_IN_HOST  = 2'b10,
        FEB_DONE     = 2'b11
    } feb_state_e;

    // A INSTRUCAO BRUTA NAO ESTA' AQUI, de proposito.
    //
    // Este pacote e' lido COMBINACIONALMENTE para as WINDOW_SIZE entradas de
    // uma vez, porque e' assim que o DAU compara todas contra todas. Cada bit
    // que entra aqui custa um flop por entrada MAIS o fan-out dessa leitura
    // total — e a analise de dependencia nao olha a instrucao bruta uma vez
    // sequer: ela usa os campos que o `ifa_pre_decode` extraiu ANTES do push.
    //
    // A instrucao vive no payload do host (`payload_mem` no ifa_top), que e'
    // lido para UMA entrada so', a despachada. Duplica-la aqui custava 32
    // flops por entrada e alargava a janela inteira em 32 bits, para servir
    // apenas a um `$display` de depuracao.
    typedef struct packed {
        logic [REG_ADDR_WIDTH-1:0] rs1;
        logic [REG_ADDR_WIDTH-1:0] rs2;
        logic [REG_ADDR_WIDTH-1:0] rd;
        logic                      uses_rs1;
        logic                      uses_rs2;
        logic                      writes_rd;
        logic                      is_branch;
        logic                      is_syscall_atomic;
        logic                      is_mem;
        logic                      is_csr;
        // MUL/DIV: ocupa recurso dedicado por varios ciclos. O IFA marca isso
        // NO DESPACHO (nao espera o sinal do host, que so' sobe depois de a
        // instrucao entrar no ID) para poder escolher uma ALU independente no
        // lugar da proxima que precisaria do mesmo recurso.
        logic                      is_multdiv;
        // IDEMPOTENTE: reexecutar produz o mesmo resultado.
        //
        // E' o que permite ultrapassar uma instrucao mais velha que possa
        // FALTAR. Se ela faltar depois, o `mepc` aponta para ela, o handler
        // roda, retorna, ela executa, e esta aqui e' buscada e executada de
        // NOVO. Sendo idempotente, o resultado e' o mesmo e nao ha' corrupcao.
        //
        // Condicao: ALU pura (sem memoria/desvio/CSR/syscall) e `rd` NAO e'
        // uma das fontes.
        //   add  x5, x6, x7  -> reexecutar da' o mesmo valor  (idempotente)
        //   addi x5, x5, 1   -> incrementa duas vezes          (NAO)
        logic                      is_idempotent;
        // NUMERO DE SEQUENCIA de busca (conta modulo 256 a cada instrucao que
        // entra no FEB). E' o que permite o FLUSH PARCIAL: quando um desvio
        // despachado fora de ordem erra a previsao, apaga-se so' o que e'
        // MAIS NOVO que ele e preserva-se o que e' mais velho. A posicao no
        // buffer nao serve para isso porque pode ser reaproveitada.
        logic [7:0]                seq;
        // A BUSCA CONSUMIU UMA PREVISAO DE RETORNO por esta instrucao (tirou um
        // endereco da pilha de retorno do host). Se ela morrer num flush ainda
        // dentro do FEB, esse consumo precisa ser devolvido — ver
        // `flush_ras_cnt_o` em front_end_buffer.sv. Quem marca e' o host, no
        // push; o pre-decode nao tem como saber.
        logic                      ras_pop;
    } ifa_ctrl_packet_t;

    typedef struct packed {
        ifa_ctrl_packet_t data;
        logic             valid;
        // `dispatched` MANTIDO: o DAU e o dispatcher usam para saber se a
        // entrada ainda e' candidata. Equivale a `state != FEB_PENDING`.
        logic             dispatched;
        // Estado detalhado, para o flush saber o que preservar (ver enum).
        feb_state_e       state;
    } ifa_feb_entry_t;
endpackage
