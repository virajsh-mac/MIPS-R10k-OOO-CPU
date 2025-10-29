`include "sys_defs.svh"

module stage_issue (
    input clock,
    input reset,

    // All RS entries (from RS module)
    input RS_ENTRY [`RS_SZ-1:0] entries,

    // signal not to take anything from the RS this cycle
    input logic mispredict,

    // Inputs from CBD arbiter indicating available FUs next cycle
    input logic [`NUM_FU_BRANCH-1:0] branch_grants,
    input logic [`NUM_FU_ALU-1:0] alu_grants,
    input logic [`NUM_FU_MEM-1:0] mem_grants,
    // Input from EX indicating availiable MULT next cycle
    input logic [`NUM_FU_MULT-1:0] mult_grants,

    // Outputs to RS for clearing issued entries on the next cycle
    output logic  [`NUM_FU_TOTAL-1:0] clear_valid,
    output RS_IDX [`NUM_FU_TOTAL-1:0] clear_idxs,

    // Outputs to issue-execute pipeline register
    output logic [`NUM_FU_TOTAL-1:0] issue_valid,
    output RS_ENTRY [`NUM_FU_TOTAL-1:0] issued_entries
);

    logic [`RS_SZ-1:0] ready;
    OP_CATEGORY [`RS_SZ-1:0] cat;

    psel_gen #(
        .WIDTH(`NUM_FU_TOTAL),
        .REQS(`NUM_FU_TOTAL)
    ) instruction_selector (
        // Priority on the request bus alternates between highest and lowest
        .req(),
        .gnt(),
        .gnt_bus()
    );

    // find all ready instructions in your respective RS
    // using a parking lot at each area of the pipeline register allocate instructions
    // by category select which ones will go into the S/E register
    // based on the amount of availiable FU's the next cycle

    // Combinational logic for issue selection
    always_comb begin
        clear_valid = '0;
        clear_idxs = '0;
        issue_valid = '0;
        issued_entries = '0;

        if (reset || mispredict) begin
            // No issue on reset or mispredict
        end else begin

        end
    end

endmodule
