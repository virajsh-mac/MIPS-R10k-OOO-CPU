`include "sys_defs.svh"

module stage_issue (
    input clock,
    input reset,

    // All RS entries (from RS module)
    input RS_ENTRY [`RS_SZ-1:0] entries,

    // signal not to take anything from the RS this cycle
    input logic mispredict,

    // From ROB: wrap bit of the current head entry (for age adjustment)
    input logic head_wrap,

    // Inputs from EX indicating available FUs this cycle
    input logic [`NUM_FU_ALU-1:0] alu_avail,
    input logic [`NUM_FU_MULT-1:0] mult_avail,
    input logic [`NUM_FU_BRANCH-1:0] branch_avail,
    input logic [`NUM_FU_MEM-1:0] mem_avail,

    // Outputs to RS for clearing issued entries
    output logic  [`N-1:0] clear_valid,
    output RS_IDX [`N-1:0] clear_idxs,

    // Outputs to issue-execute pipeline register
    output logic [`N-1:0] issue_valid,
    output RS_ENTRY [`N-1:0] issued_entries
);

    // Uses bit-vector presence maps indexed by age and population counts ($countones) for
    // oldest-first priority resolution with per-category FU limiting. Issues as many
    // instructions as FUs allow. Tested in synthesis to CP: 5ns.

    // Combinational logic for issue selection
    always_comb begin
        clear_valid = '0;
        clear_idxs = '0;
        issue_valid = '0;
        issued_entries = '0;

        if (reset || mispredict) begin
            // No issue on reset or mispredict
        end else begin
            // Local variables for issue selection logic
            logic [`RS_SZ-1:0] ready;
            logic [5:0] age[`RS_SZ-1:0];  // 6-bit age {rob_wrap, rob_idx}
            OP_CATEGORY cat[`RS_SZ-1:0];
            logic [3:0] num_fu[`RS_SZ-1:0];

            // Bit-matrix presence vectors for fast priority encoding
            logic [63:0] per_cat_presence[`NUM_CATS-1:0];  // Ready entries per category at each age (0-63)

            // Ranking counts and decisions
            logic [3:0] per_cat_count[`RS_SZ-1:0];  // Older ready entries in same category
            logic [`RS_SZ-1:0] cand;  // Candidates after per-category filtering

            // Loop variable and output packing
            int i;
            int out_idx;

            for (i = 0; i < `RS_SZ; i++) begin
                ready[i] = entries[i].valid && entries[i].src1_ready && entries[i].src2_ready;
                // Adjust age relative to current ROB head wrap bit to handle straddle cases
                // where the tail wraps multiple times (e.g., youngest age=0, oldest=33-63).
                // XOR flips the wrap bit, shifting old entries to low adjusted ages and
                // young ones to high, ensuring unsigned comparison selects oldest (smallest
                // adjusted age) correctly without duplicates (max in-flight=32 < 64).
                age[i]   = {entries[i].rob_wrap ^ head_wrap, entries[i].rob_idx};
                cat[i]   = entries[i].op_type.category;
            end

            // Count available FUs per category
            for (i = 0; i < `RS_SZ; i++) begin
                case (cat[i])
                    CAT_ALU:    num_fu[i] = $countones(alu_avail);
                    CAT_MULT:   num_fu[i] = $countones(mult_avail);
                    CAT_BRANCH: num_fu[i] = $countones(branch_avail);
                    CAT_MEM:    num_fu[i] = $countones(mem_avail);
                    default:    num_fu[i] = 0;
                endcase
            end

            // Build age presence vectors for per-category and global ranking
            per_cat_presence = '{default: 64'd0};
            for (i = 0; i < `RS_SZ; i++) begin
                if (ready[i]) begin
                    per_cat_presence[cat[i]][age[i]] = 1'b1;
                end
            end

            // Phase 1: Per-category ranking to respect FU limits
            for (i = 0; i < `RS_SZ; i++) begin
                // Count older ready entries in same category: mask lower ages (0 to age[i]-1) with category presence bits
                per_cat_count[i] = $countones(per_cat_presence[cat[i]] & ((64'd1 << age[i]) - 64'd1));
                // Candidate if ready and fewer than available FUs of this category have older entries
                cand[i] = ready[i] && (per_cat_count[i] < num_fu[i]);
            end

            // Pack issued entries into output arrays (program order not required for correctness)
            out_idx = 0;
            for (i = 0; i < `RS_SZ && out_idx < `N; i++) begin
                if (cand[i]) begin
                    issue_valid[out_idx] = 1'b1;
                    issued_entries[out_idx] = entries[i];
                    clear_valid[out_idx] = 1'b1;
                    clear_idxs[out_idx] = i;
                    out_idx++;
                end
            end
        end
    end

endmodule
