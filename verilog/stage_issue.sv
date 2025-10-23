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
    input logic [`NUM_FU_MEM-1:0] mem_avail,

    // Outputs to RS for clearing issued entries
    output logic  [`N-1:0] clear_valid,
    output RS_IDX [`N-1:0] clear_idxs,

    // Outputs to issue-execute pipeline register
    output logic [`N-1:0] issue_valid,
    output RS_ENTRY [`N-1:0] issued_entries
);
    // Uses bit-vector presence maps indexed by age and population counts ($countones) for 
    // oldest-first priority resolution in a two-phase process (per-category FU limiting 
    // followed by global bandwidth limiting) known as a compacted age matrix scheduler.

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
            logic [63:0] global_presence;  // Candidate entries at each age

            // Ranking counts and decisions
            logic [3:0] per_cat_count[`RS_SZ-1:0];  // Older ready entries in same category
            logic [`RS_SZ-1:0] cand;  // Candidates after per-category filtering
            logic [3:0] global_count[`RS_SZ-1:0];  // Older candidates globally
            logic [`RS_SZ-1:0] issue;  // Final issue decisions

            // Loop variable and output packing
            int i;
            int out_idx;

            for (i = 0; i < `RS_SZ; i++) begin
                ready[i] = entries[i].valid && entries[i].src1_ready && entries[i].src2_ready;
                age[i]   = {entries[i].rob_wrap, entries[i].rob_idx};
                cat[i]   = entries[i].op_type.category;
            end

            // Count available FUs per category
            for (i = 0; i < `RS_SZ; i++) begin
                case (cat[i])
                    CAT_ALU, CAT_CSR: num_fu[i] = $countones(alu_avail);
                    CAT_MULT:         num_fu[i] = $countones(mult_avail);
                    CAT_BRANCH:       num_fu[i] = $countones(branch_avail);
                    CAT_MEM:          num_fu[i] = $countones(mem_avail);
                    default:          num_fu[i] = 0;
                endcase
            end

            // Build age presence vectors for per-category and global ranking
            per_cat_presence = '{default: 64'd0};
            for (int i = 0; i < `RS_SZ; i++) begin
                if (ready[i]) begin
                    per_cat_presence[cat[i]][age[i]] = 1'b1;
                end
            end

            // Phase 1: Per-category ranking to respect FU limits
            for (i = 0; i < `RS_SZ; i++) begin
                logic [63:0] mask = (64'd1 << age[i]) - 64'd1;  // Lower bits: ages < age[i]
                logic [63:0] older_bits = per_cat_presence[cat[i]] & mask;
                per_cat_count[i] = $countones(older_bits);
                cand[i] = ready[i] && (per_cat_count[i] < num_fu[i]);
            end

            // Build global presence vector from candidates
            global_presence = 64'd0;
            for (i = 0; i < `RS_SZ; i++) begin
                if (cand[i]) begin
                    global_presence[age[i]] = 1'b1;
                end
            end

            // Phase 2: Global ranking among candidates to respect total issue width
            for (i = 0; i < `RS_SZ; i++) begin
                logic [63:0] mask = (64'd1 << age[i]) - 64'd1;
                logic [63:0] older_bits = global_presence & mask;
                global_count[i] = $countones(older_bits);
                issue[i] = cand[i] && (global_count[i] < `N);
            end

            // Pack issued entries into output arrays (program order not required for correctness)
            out_idx = 0;
            for (i = 0; i < `RS_SZ && out_idx < `N; i++) begin
                if (issue[i]) begin
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
