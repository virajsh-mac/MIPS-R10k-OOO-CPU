`include "sys_defs.svh"

// Branch module: compute whether to take branches and target addresses
// This module is purely combinational
module branch (
    input DATA rs1,
    input DATA rs2,
    input BRANCH_FUNC func,  // Which branch condition to check
    input ADDR pc,           // Current PC
    input DATA offset,       // Branch offset (from immediate)

    output logic take,       // True/False condition result
    output ADDR target       // Target address (PC+4 if not taken, PC+offset if taken)
);

    always_comb begin
        case (func)
            EQ:   take = signed'(rs1) == signed'(rs2);  // BEQ
            NE:   take = signed'(rs1) != signed'(rs2);  // BNE
            LT:   take = signed'(rs1) < signed'(rs2);   // BLT
            GE:   take = signed'(rs1) >= signed'(rs2);  // BGE
            LTU:  take = rs1 < rs2;                     // BLTU
            GEU:  take = rs1 >= rs2;                    // BGEU
            JAL:  take = `TRUE;                         // JAL is always taken
            JALR: take = `TRUE;                         // JALR is always taken
            default: take = `FALSE;
        endcase

        // Compute target address: PC+4 if not taken, PC+offset if taken
        target = take ? (pc + offset) : (pc + 32'h4);
    end

endmodule  // branch
