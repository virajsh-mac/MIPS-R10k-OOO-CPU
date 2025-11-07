`include "sys_defs.svh"

// ALU: computes the result of FUNC applied with operands A and B
// This module is purely combinational
module alu (
    input DATA     opa,
    input DATA     opb,
    input ALU_FUNC alu_func,

    output DATA result
);

    always_comb begin
        case (alu_func)
            ADD: result = opa + opb;
            SUB: result = opa - opb;
            AND: result = opa & opb;
            SLT: result = signed'(opa) < signed'(opb);
            SLTU: result = opa < opb;
            OR: result = opa | opb;
            XOR: result = opa ^ opb;
            SRL: result = opa >> opb[4:0];
            SLL: result = opa << opb[4:0];
            SRA: result = signed'(opa) >>> opb[4:0];  // arithmetic from logical shift
            HALT: result = 32'h0;  // HALT doesn't produce a meaningful result
            // here to prevent latches:
            default: result = 32'hfacebeec;
        endcase
    end

endmodule  // alu
