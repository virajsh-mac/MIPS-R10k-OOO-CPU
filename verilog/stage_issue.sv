`include "sys_defs.svh"

module stage_issue (
    input clock,
    input reset,

    // All RS entries (from RS module)
    input RS_ENTRY [`RS_SZ-1:0] entries,

    // signal not to take anything from the RS this cycle
    input logic mispredict,

    // Inputs from EX indicating available FUs this cycle
    input logic [`NUM_FU_ALU-1:0] alu_avail,
    input logic [`NUM_FU_MULT-1:0] mult_avail,
    input logic [`NUM_FU_BRANCH-1:0] branch_avail,
    input logic [`NUM_FU_ADDR-1:0] addr_avail,
    input logic [`NUM_FU_MEM-1:0] mem_avail,

    // Outputs to RS for clearing issued entries
    output logic [`N-1:0] clear_valid,
    output RS_IDX [`N-1:0] clear_idxs,

    // Outputs to issue-execute pipeline register
    output logic [`N-1:0] issue_valid,
    output RS_ENTRY [`N-1:0] issued_entries
);

    // Combinational logic for issue selection
    always_comb begin
        clear_valid = '0;
        clear_idxs = '0;
        issue_valid = '0;
        issued_entries = '0;

        if (reset || mispredict) begin
            // No issue on reset or mispredict
        end else begin
            // Step 1: Identify ready entries (valid && both srcs ready)
            logic [`RS_SZ-1:0] ready_mask = '0;
            for (int i = 0; i < `RS_SZ; i++) begin
                if (entries[i].valid && entries[i].src1_ready && entries[i].src2_ready) begin
                    ready_mask[i] = 1'b1;
                end
            end

            // Compute number of available FUs per type
            logic [$clog2(`NUM_FU_ALU+1)-1:0] num_alu_avail = $countones(alu_avail);
            logic [$clog2(`NUM_FU_MULT+1)-1:0] num_mult_avail = $countones(mult_avail);
            logic [$clog2(`NUM_FU_BRANCH+1)-1:0] num_branch_avail = $countones(branch_avail);
            logic [$clog2(`NUM_FU_MEM+1)-1:0] num_mem_avail = $countones(mem_avail);

            // Track selected entries
            logic [`RS_SZ-1:0] select_mask = '0;
            int num_issued = 0;

            // Step 2: Iteratively select up to `N oldest ready instructions that have an available FU
            for (int sel = 0; sel < `N; sel++) begin
                if (num_issued >= `N) break;

                // Find the oldest ready entry not yet selected (smallest rob_idx assumes no wrap handling)
                ROB_IDX min_rob = {`ROB_IDX_BITS{1'b1}};  // Max value
                RS_IDX min_idx = 0;
                logic found = 1'b0;

                for (int i = 0; i < `RS_SZ; i++) begin
                    // TODO: dependent for loop on min rob figure out how to select oldest non sequentially
                    // figure out our selection strategy
                    if (ready_mask[i] && !select_mask[i] && (entries[i].rob_idx < min_rob)) begin
                        min_rob = entries[i].rob_idx;
                        min_idx = i;
                        found = 1'b1;
                    end
                end

                if (found) begin
                    // Step 3: Check OP_TYPE category and availability
                    logic can_issue = 1'b0;
                    case (entries[min_idx].op_type.category)
                        CAT_ALU, CAT_CSR: begin  // Treat CSR as ALU
                            if (num_alu_avail > 0) begin
                                can_issue = 1'b1;
                                num_alu_avail--;
                            end
                        end
                        CAT_MULT: begin
                            if (num_mult_avail > 0) begin
                                can_issue = 1'b1;
                                num_mult_avail--;
                            end
                        end
                        CAT_BRANCH: begin
                            if (num_branch_avail > 0) begin
                                can_issue = 1'b1;
                                num_branch_avail--;
                            end
                        end
                        CAT_MEM: begin
                            // Assume MEM ops use mem_avail (or addr_avail if separate; adjust as needed)
                            if (num_mem_avail > 0) begin
                                can_issue = 1'b1;
                                num_mem_avail--;
                            end
                        end
                        default: can_issue = 1'b0;
                    endcase

                    if (can_issue) begin
                        select_mask[min_idx] = 1'b1;
                        num_issued++;
                    end
                end else begin
                    break;
                end
            end

            // Pack selected entries and clear signals
            int iss = 0;
            for (int i = 0; i < `RS_SZ; i++) begin
                if (select_mask[i]) begin
                    issued_entries[iss] = entries[i];
                    clear_idxs[iss] = i;
                    clear_valid[iss] = 1'b1;
                    issue_valid[iss] = 1'b1;
                    iss++;
                end
            end
        end
    end

endmodule
