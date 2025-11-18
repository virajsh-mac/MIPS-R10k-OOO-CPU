
// The decoder, copied from project 3

// This has a few changes, it now sets a "mult" flag for multiply instructions
// Pass these to the mult module with inst.r.funct3 as the MULT_FUNC

`include "sys_defs.svh"
`include "ISA.svh"

// Decode an instruction: generate useful datapath control signals by matching the RISC-V ISA
// This module is purely combinational
module decoder (
    input INST  inst,
    input logic valid, // when low, ignore inst. Output will look like a NOP

    output ALU_OPA_SELECT opa_select,
    output ALU_OPB_SELECT opb_select,
    output logic          has_dest,    // if there is a destination register
    output OP_TYPE        op_type,
    output logic          csr_op,      // used for CSR operations
    output logic          halt,        // non-zero on a halt
    output logic          illegal,     // non-zero on an illegal instruction

    // Enhanced outputs for packet construction
    output REG_IDX rs1_idx,     // Source register 1 index
    output REG_IDX rs2_idx,     // Source register 2 index
    output REG_IDX rd_idx,      // Destination register index
    output logic   uses_rd,     // Whether instruction actually uses destination
    output DATA    immediate    // Pre-extracted immediate value
);

    // Note: I recommend using an IDE's code folding feature on this block
    always_comb begin
        // Default control values (looks like a NOP)
        // See sys_defs.svh for the constants used here
        opa_select = OPA_IS_RS1;
        opb_select = OPB_IS_RS2;
        has_dest   = `FALSE;
        op_type    = '{category: CAT_ALU, func: 4'h0};
        csr_op     = `FALSE;
        halt       = `FALSE;
        illegal    = `FALSE;

        // Default register indices and immediate
        rs1_idx    = inst[19:15];
        rs2_idx    = inst[24:20];
        rd_idx     = inst[11:7];
        uses_rd    = has_dest && (rd_idx != `ZERO_REG);
        immediate  = '0;

        if (valid) begin
            casez (inst)
                `RV32_LUI: begin
                    has_dest   = `TRUE;
                    opa_select = OPA_IS_ZERO;
                    opb_select = OPB_IS_U_IMM;
                    op_type    = OP_ALU_ADD;
                end
                `RV32_AUIPC: begin
                    has_dest   = `TRUE;
                    opa_select = OPA_IS_PC;
                    opb_select = OPB_IS_U_IMM;
                    op_type    = OP_ALU_ADD;
                end
                `RV32_JAL: begin
                    has_dest   = `TRUE;
                    opa_select = OPA_IS_PC;
                    opb_select = OPB_IS_J_IMM;
                    op_type    = OP_JAL;
                end
                `RV32_JALR: begin
                    has_dest   = `TRUE;
                    opa_select = OPA_IS_RS1;
                    opb_select = OPB_IS_I_IMM;
                    op_type    = OP_JALR;
                end
                `RV32_BEQ: begin
                    opa_select = OPA_IS_PC;
                    opb_select = OPB_IS_B_IMM;
                    op_type    = OP_BR_EQ;
                end
                `RV32_BNE: begin
                    opa_select = OPA_IS_PC;
                    opb_select = OPB_IS_B_IMM;
                    op_type    = OP_BR_NE;
                end
                `RV32_BLT: begin
                    opa_select = OPA_IS_PC;
                    opb_select = OPB_IS_B_IMM;
                    op_type    = OP_BR_LT;
                end
                `RV32_BGE: begin
                    opa_select = OPA_IS_PC;
                    opb_select = OPB_IS_B_IMM;
                    op_type    = OP_BR_GE;
                end
                `RV32_BLTU: begin
                    opa_select = OPA_IS_PC;
                    opb_select = OPB_IS_B_IMM;
                    op_type    = OP_BR_LTU;
                end
                `RV32_BGEU: begin
                    opa_select = OPA_IS_PC;
                    opb_select = OPB_IS_B_IMM;
                    op_type    = OP_BR_GEU;
                end
                `RV32_MUL: begin
                    has_dest = `TRUE;
                    op_type  = OP_MULT_MUL;
                end
                `RV32_MULH: begin
                    has_dest = `TRUE;
                    op_type  = OP_MULT_MULH;
                end
                `RV32_MULHSU: begin
                    has_dest = `TRUE;
                    op_type  = OP_MULT_MULHSU;
                end
                `RV32_MULHU: begin
                    has_dest = `TRUE;
                    op_type  = OP_MULT_MULHU;
                end
                `RV32_LB: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    op_type    = OP_LOAD_BYTE;
                end
                `RV32_LH: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    op_type    = OP_LOAD_HALF;
                end
                `RV32_LW: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    op_type    = OP_LOAD_WORD;
                end
                `RV32_LBU: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    op_type    = OP_LOAD_BYTE_U;
                end
                `RV32_LHU: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    op_type    = OP_LOAD_HALF_U;
                end
                `RV32_SB: begin
                    opb_select = OPB_IS_S_IMM;
                    op_type    = OP_STORE_BYTE;
                end
                `RV32_SH: begin
                    opb_select = OPB_IS_S_IMM;
                    op_type    = OP_STORE_HALF;
                end
                `RV32_SW: begin
                    opb_select = OPB_IS_S_IMM;
                    op_type    = OP_STORE_WORD;
                end
                `RV32_ADDI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    op_type    = OP_ALU_ADD;
                end
                `RV32_SLTI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    op_type    = OP_ALU_SLT;
                end
                `RV32_SLTIU: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    op_type    = OP_ALU_SLTU;
                end
                `RV32_ANDI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    op_type    = OP_ALU_AND;
                end
                `RV32_ORI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    op_type    = OP_ALU_OR;
                end
                `RV32_XORI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    op_type    = OP_ALU_XOR;
                end
                `RV32_SLLI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    op_type    = OP_ALU_SLL;
                end
                `RV32_SRLI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    op_type    = OP_ALU_SRL;
                end
                `RV32_SRAI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    op_type    = OP_ALU_SRA;
                end
                `RV32_ADD: begin
                    has_dest = `TRUE;
                    op_type  = OP_ALU_ADD;
                end
                `RV32_SUB: begin
                    has_dest = `TRUE;
                    op_type  = OP_ALU_SUB;
                end
                `RV32_SLT: begin
                    has_dest = `TRUE;
                    op_type  = OP_ALU_SLT;
                end
                `RV32_SLTU: begin
                    has_dest = `TRUE;
                    op_type  = OP_ALU_SLTU;
                end
                `RV32_AND: begin
                    has_dest = `TRUE;
                    op_type  = OP_ALU_AND;
                end
                `RV32_OR: begin
                    has_dest = `TRUE;
                    op_type  = OP_ALU_OR;
                end
                `RV32_XOR: begin
                    has_dest = `TRUE;
                    op_type  = OP_ALU_XOR;
                end
                `RV32_SLL: begin
                    has_dest = `TRUE;
                    op_type  = OP_ALU_SLL;
                end
                `RV32_SRL: begin
                    has_dest = `TRUE;
                    op_type  = OP_ALU_SRL;
                end
                `RV32_SRA: begin
                    has_dest = `TRUE;
                    op_type  = OP_ALU_SRA;
                end
                `RV32_CSRRW, `RV32_CSRRS, `RV32_CSRRC, `RV32_CSRRWI, `RV32_CSRRSI, `RV32_CSRRCI: begin
                    // Ignore CSR instructions - treat as NOP
                    illegal = `TRUE;
                end
                `WFI: begin
                    op_type = OP_HALT;
                    halt = `TRUE;
                end
                default: begin
                    illegal = `TRUE;
                end
            endcase  // casez (inst)
        end  // if (valid)

        // Compute final uses_rd based on has_dest result
        uses_rd = has_dest && (rd_idx != `ZERO_REG);

        // Extract immediate based on opb_select
        case (opb_select)
            OPB_IS_I_IMM: immediate = `RV32_signext_Iimm(inst);
            OPB_IS_S_IMM: immediate = `RV32_signext_Simm(inst);
            OPB_IS_B_IMM: immediate = `RV32_signext_Bimm(inst);
            OPB_IS_U_IMM: immediate = `RV32_signext_Uimm(inst);
            OPB_IS_J_IMM: immediate = `RV32_signext_Jimm(inst);
            default:      immediate = '0;
        endcase
    end  // always

endmodule  // decoder
