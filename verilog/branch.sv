`include "sys_defs.svh"

// Conditional branch module: compute whether to take conditional branches
// This module is purely combinational
module conditional_branch (
    input DATA rs1,
    input DATA rs2,
    input [2:0] func,  // Which branch condition to check

    output logic take  // True/False condition result
);

    always_comb begin
        case (func)
            3'b000:  take = signed'(rs1) == signed'(rs2);  // BEQ
            3'b001:  take = signed'(rs1) != signed'(rs2);  // BNE
            3'b100:  take = signed'(rs1) < signed'(rs2);  // BLT
            3'b101:  take = signed'(rs1) >= signed'(rs2);  // BGE
            3'b110:  take = rs1 < rs2;  // BLTU
            3'b111:  take = rs1 >= rs2;  // BGEU
            default: take = `FALSE;
        endcase
    end

endmodule  // conditional_branch
