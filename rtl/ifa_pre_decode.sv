// =============================================================================
// ifa_pre_decode — AUTO-CONTIDO: decodifica dependencia de instrucoes
// RISC-V COMPRIMIDAS (16b) e NAO comprimidas (32b), SEM depender do
// decoder do core hospedeiro. Objetivo: IFA portavel entre cores RV.
//
// NAO descomprime (nao gera os 32b exatos) — so extrai os campos que a
// analise de dependencia precisa: rs1/rs2/rd, uses_rs1/2, writes_rd,
// is_mem, is_branch, is_syscall_atomic (barreira), is_csr.
//
// FAIL-SAFE: qualquer formato nao reconhecido com certeza -> BARREIRA.
// Assim, mesmo cobertura incompleta de RVC nunca gera reorder incorreto.
//
// Convencao: recebe `instruction[31:0]`.
//
// NOTA: com o IFA no Wally, a instrucao entra no FEB JA' EXPANDIDA (um
// `decompress` roda na entrada — ver `ifa_decomp` no ifu.sv). Entao o ramo
// RVC abaixo nao e' exercitado nessa integracao; fica para hosts que
// entreguem a forma comprimida (Ibex, CVA6).
//
// Se instruction[1:0]!=11 e' uma comprimida (nos 16 bits inferiores).
// =============================================================================
module ifa_pre_decode #(
    // 1 = host RV32. Quatro encodings COMPRIMIDOS mudam de significado entre
    // RV32 e RV64, e errar neles e' grave:
    //
    //   q1 f3=001   RV32 C.JAL    x  RV64 C.ADDIW
    //   q0 f3=011   RV32 C.FLW    x  RV64 C.LD
    //   q2 f3=011   RV32 C.FLWSP  x  RV64 C.LDSP
    //   q1 f3=100, [11:10]=11, [12]=1  RV32 reservado x RV64 C.SUBW/C.ADDW
    //
    // O primeiro e' o que quebra: um C.JAL decodificado como aritmetica deixa
    // o IFA sem barreira de controle num DESVIO. Medido no Embench/crc32 em
    // rv32gc: o despacho atravessava os `c.jal` e o PC caia para 0 no ciclo
    // 189.
    parameter bit RV32 = 1'b0
) (
    input  logic [31:0] instruction,
    output logic [4:0]  rs1, rs2, rd,
    output logic        uses_rs1, uses_rs2, writes_rd,
    output logic        is_branch, is_mem, is_syscall_atomic, is_csr,
    output logic        is_multdiv,
    output logic        is_idempotent
);
    logic [6:0] opcode;
    logic [1:0] cq;        // quadrante comprimido (instruction[1:0])
    logic [2:0] cf3;       // funct3 comprimido (instruction[15:13])
    logic       is_comp;

    // registradores "linha" (x8..x15) dos formatos comprimidos de 3 bits
    logic [4:0] rdp;       // {2'b01, instr[4:2]}
    logic [4:0] rs1p;      // {2'b01, instr[9:7]}
    logic [4:0] rfull_117; // instr[11:7]
    logic [4:0] rfull_62;  // instr[6:2]

    assign opcode    = instruction[6:0];
    assign cq        = instruction[1:0];
    assign cf3       = instruction[15:13];
    assign is_comp   = (cq != 2'b11);
    assign rdp       = {2'b01, instruction[4:2]};
    assign rs1p      = {2'b01, instruction[9:7]};
    assign rfull_117 = instruction[11:7];
    assign rfull_62  = instruction[6:2];

    always_comb begin
        rs1=5'd0; rs2=5'd0; rd=5'd0;
        uses_rs1=1'b0; uses_rs2=1'b0; writes_rd=1'b0;
        is_branch=1'b0; is_mem=1'b0; is_syscall_atomic=1'b0; is_csr=1'b0;
        is_multdiv=1'b0;

        if (!is_comp) begin
            // ================= 32 bits =================
            rd=instruction[11:7]; rs1=instruction[19:15]; rs2=instruction[24:20];
            case (opcode)
                7'b0110011, 7'b0111011: begin
                    uses_rs1=1; uses_rs2=1; writes_rd=1;             // OP / OP-32
                    is_multdiv = (instruction[31:25] == 7'b0000001); // extensao M
                end
                7'b0010011, 7'b0011011: begin uses_rs1=1;             writes_rd=1; end // OP-IMM / -32
                7'b0110111, 7'b0010111: begin                         writes_rd=1; end // LUI/AUIPC
                7'b0000011: begin uses_rs1=1; writes_rd=1; is_mem=1; end // LOAD
                7'b0100011: begin uses_rs1=1; uses_rs2=1;  is_mem=1; end // STORE
                7'b1100011: begin uses_rs1=1; uses_rs2=1; is_branch=1; end // BRANCH
                7'b1101111: begin             writes_rd=1; is_branch=1; end // JAL
                7'b1100111: begin uses_rs1=1; writes_rd=1; is_branch=1; end // JALR
                7'b0001111: begin is_syscall_atomic=1; end // FENCE
                7'b1110011: begin
                    is_csr            = (instruction[14:12] != 3'b000);
                    is_syscall_atomic = (instruction[14:12] == 3'b000);
                    if (is_csr) begin
                        uses_rs1  = (instruction[14:13]==2'b01) && (instruction[19:15]!=5'd0);
                        writes_rd = 1'b1;
                    end
                end
                7'b0101111: begin uses_rs1=1; uses_rs2=1; writes_rd=1; is_mem=1; is_syscall_atomic=1; end // AMO
                7'b0000111, 7'b0100111: begin uses_rs1=1; is_mem=1; end // FP load/store (end. GPR)
                7'b1010011, 7'b1000011, 7'b1000111,
                7'b1001011, 7'b1001111: begin is_syscall_atomic=1; end  // FP compute -> barreira
                default: is_syscall_atomic=1;                            // fail-safe
            endcase
        end else begin
            // ================= 16 bits (RVC) =================
            case (cq)
            // ---- Quadrante 0 ----
            2'b00: case (cf3)
                3'b000: begin // C.ADDI4SPN: rd'=sp+imm
                    if (instruction[12:5]!=8'd0) begin rd=rdp; uses_rs1=1; rs1=5'd2; writes_rd=1; end
                    else is_syscall_atomic=1; // reservado
                end
                3'b010: begin // C.LW : rd' = mem[rs1']
                    rd=rdp; rs1=rs1p; uses_rs1=1; writes_rd=1; is_mem=1;
                end
                3'b011: begin
                    // RV64 C.LD (escreve inteiro) x RV32 C.FLW (escreve FP).
                    rs1=rs1p; uses_rs1=1; is_mem=1;
                    if (!RV32) begin rd=rdp; writes_rd=1; end
                end
                3'b110, 3'b111: begin // C.SW / C.SD : mem[rs1'] = rs2'
                    rs1=rs1p; rs2=rdp; uses_rs1=1; uses_rs2=1; is_mem=1;
                end
                3'b001, 3'b101: begin // C.FLD / C.FSD : end. GPR rs1', dado FP
                    rs1=rs1p; uses_rs1=1; is_mem=1;
                end
                default: is_syscall_atomic=1;
            endcase
            // ---- Quadrante 1 ----
            2'b01: case (cf3)
                3'b000: begin // C.ADDI / C.NOP : rd=rs1=[11:7]
                    rd=rfull_117; rs1=rfull_117; uses_rs1=1; writes_rd=1;
                end
                3'b001: begin
                    // RV32 C.JAL (desvio, escreve ra) x RV64 C.ADDIW
                    // (aritmetica, rd=rs1=[11:7]). Ver a nota no parametro
                    // RV32: classificar C.JAL como aritmetica tira a barreira
                    // de controle de um desvio e o despacho passa por cima
                    // dele; classificar C.ADDIW como desvio poe barreira onde
                    // nao ha' (e em RV64 ele e' muito comum).
                    if (RV32) begin
                        is_branch=1; writes_rd=1; rd=5'd1;   // escreve ra
                    end else begin
                        rd=rfull_117; rs1=rfull_117; uses_rs1=1; writes_rd=1;
                    end
                end
                3'b010: begin // C.LI : rd=[11:7]
                    rd=rfull_117; writes_rd=1;
                end
                3'b011: begin // C.ADDI16SP (rd==2) / C.LUI
                    if (rfull_117==5'd2) begin rd=5'd2; rs1=5'd2; uses_rs1=1; writes_rd=1; end
                    else begin rd=rfull_117; writes_rd=1; end
                end
                3'b100: begin
                    // C.SRLI / C.SRAI / C.ANDI (rs1'=rd')
                    // C.SUB / C.XOR / C.OR / C.AND / C.SUBW / C.ADDW (reg-reg)
                    //
                    // Em RV64 o C.SRLI/SRAI usam shamt de 6 bits (o bit 12
                    // faz parte do shamt). Nao afeta a classificacao aqui —
                    // ambos sao aritmetica com rd=rs1=rs1p.
                    //
                    // `instruction[11:10]==2'b11` distingue o grupo reg-reg,
                    // que inclui C.SUBW/C.ADDW (RV64-only).
                    rd=rs1p; rs1=rs1p; uses_rs1=1; writes_rd=1;
                    if (instruction[11:10]==2'b11) begin
                        rs2=rdp; uses_rs2=1;
                        // [12]=1 e' C.SUBW/C.ADDW, que so' existem em RV64.
                        // Em RV32 o encoding e' RESERVADO: barreira, para nao
                        // reordenar em volta de algo que vai dar instrucao
                        // ilegal.
                        if (RV32 && instruction[12]) is_syscall_atomic=1;
                    end
                end
                3'b101: begin is_branch=1; end          // C.J
                3'b110, 3'b111: begin                   // C.BEQZ / C.BNEZ
                    rs1=rs1p; uses_rs1=1; is_branch=1;
                end
                default: is_syscall_atomic=1;
            endcase
            // ---- Quadrante 2 ----
            2'b10: case (cf3)
                3'b000: begin // C.SLLI : rd=rs1=[11:7]
                    rd=rfull_117; rs1=rfull_117; uses_rs1=1; writes_rd=1;
                end
                3'b010: begin // C.LWSP : rd = mem[sp]
                    rd=rfull_117; rs1=5'd2; uses_rs1=1; writes_rd=1; is_mem=1;
                end
                3'b011: begin
                    // RV64 C.LDSP (escreve inteiro) x RV32 C.FLWSP (FP).
                    rs1=5'd2; uses_rs1=1; is_mem=1;
                    if (!RV32) begin rd=rfull_117; writes_rd=1; end
                end
                3'b100: begin // C.JR / C.MV / C.EBREAK / C.JALR / C.ADD
                    if (instruction[12]==1'b0) begin
                        if (rfull_62==5'd0) begin rs1=rfull_117; uses_rs1=1; is_branch=1; end // C.JR
                        else begin rd=rfull_117; rs2=rfull_62; uses_rs2=1; writes_rd=1; end   // C.MV
                    end else begin
                        if (rfull_117==5'd0 && rfull_62==5'd0) is_syscall_atomic=1;            // C.EBREAK
                        else if (rfull_62==5'd0) begin rs1=rfull_117; uses_rs1=1; is_branch=1; writes_rd=1; rd=5'd1; end // C.JALR
                        else begin rd=rfull_117; rs1=rfull_117; rs2=rfull_62; uses_rs1=1; uses_rs2=1; writes_rd=1; end // C.ADD
                    end
                end
                3'b110, 3'b111: begin // C.SWSP / C.SDSP : mem[sp] = rs2
                    rs1=5'd2; rs2=rfull_62; uses_rs1=1; uses_rs2=1; is_mem=1;
                end
                3'b001, 3'b101: begin // C.FLDSP / C.FSDSP : end. sp, dado FP
                    rs1=5'd2; uses_rs1=1; is_mem=1;
                end
                default: is_syscall_atomic=1;
            endcase
            default: is_syscall_atomic=1;
            endcase
        end
    end

    // Idempotencia — ver nota em ifa_pkg.sv.
    //
    // Exige escrita em registrador (senao nao ha' efeito a repetir), destino
    // diferente das fontes, e nenhum efeito colateral fora do register file.
    // MUL/DIV entram: sao puras tambem, so' demoram mais.
    assign is_idempotent = writes_rd & (rd != 5'd0) &
                           ~(uses_rs1 & (rd == rs1)) &
                           ~(uses_rs2 & (rd == rs2)) &
                           ~is_mem & ~is_branch & ~is_csr & ~is_syscall_atomic;

endmodule
