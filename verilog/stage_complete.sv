`include "sys_defs.svh"

module complete #(
    parameter int N = `N
) (
    input logic clock,
    input logic reset,

    // From EX/COMP pipe reg
    input logic             ex_valid_in[N-1:0],
    input EX_COMPLETE_ENTRY ex_comp_in [N-1:0],

    // To ROB
    output ROB_UPDATE_PACKET rob_update_packet
);

    // ROB updates: mark complete
    always_comb begin
        rob_update_packet = '0;  // Initialize all fields to 0

        for (int i = 0; i < N; i++) begin  // Per-lane fan-out (not a dependent loop)
            if (ex_valid_in[i]) begin
                rob_update_packet.valid[i] = 1'b1;
                rob_update_packet.idx[i] = ex_comp_in[i].rob_idx;
                rob_update_packet.mispredicts[i] = ex_comp_in[i].mispredict;

                // Handle branch information if present
                if (ex_comp_in[i].branch_valid) begin
                    rob_update_packet.branch_taken[i]   = ex_comp_in[i].branch_taken;
                    rob_update_packet.branch_targets[i] = ex_comp_in[i].branch_target;
                end
            end
        end
    end

endmodule
