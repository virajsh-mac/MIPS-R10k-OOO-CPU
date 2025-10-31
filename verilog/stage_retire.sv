`include "sys_defs.svh"

module stage_retire #(
    parameter  int N          = `N,
    parameter  int ARCH_COUNT = `ARCH_REG_SZ,
    parameter  int PHYS_REGS  = `PHYS_REG_SZ_R10K,
    localparam int PRW        = (PHYS_REGS <= 2) ? 1 : $clog2(PHYS_REGS)
) (
    input logic clock,
    input logic reset,

    // From ROB: head window (N-1 = oldest, 0 = youngest)
    input ROB_ENTRY [N-1:0] head_entries,
    input logic     [N-1:0] head_valids,
    input ROB_IDX   [N-1:0] head_idxs,     // ROB index per head slot

    // To ROB: flush younger if head is a mispredicted branch
    output logic   rob_mispredict,
    output ROB_IDX rob_mispred_idx,

    // Global recovery pulse (tables react internally)
    output logic bp_recover_en,

    // To freelist: bitmap of PRs to free (all committed lanes' Told this cycle)
    output logic [PHYS_REGS-1:0] free_mask,

    // To archMapTable: N write ports (commit multiple per cycle)
    output logic    [N-1:0] arch_write_enables,
    output REG_IDX  [N-1:0] arch_write_addrs,
    output PHYS_TAG [N-1:0] arch_write_phys_regs
);

    always_comb begin
        // Locals
        ROB_ENTRY head_entry;
        ROB_ENTRY entry;
        logic recover, mispred_dir, mispred_tgt, mispred;

        // ---- Synth-friendly defaults ----
        {rob_mispredict, rob_mispred_idx, bp_recover_en, free_mask} = '0;
        arch_write_enables = '0;
        arch_write_addrs = '0;
        arch_write_phys_regs = '0;
        recover = 1'b0;
        mispred_dir = 1'b0;
        mispred_tgt = 1'b0;
        mispred = 1'b0;
        head_entry = '0;
        entry = '0;

        // -------- Mispredict detect on oldest visible head --------
        if (head_valids[N-1]) begin
            head_entry = head_entries[N-1];
            if (head_entry.branch && head_entry.complete) begin
                mispred_dir = (head_entry.pred_taken != head_entry.branch_taken);
                mispred_tgt = (head_entry.branch_taken && (head_entry.pred_target != head_entry.branch_target));
                mispred     = (mispred_dir || mispred_tgt);
                if (mispred) begin
                    rob_mispredict  = 1'b1;
                    rob_mispred_idx = head_idxs[N-1];  // provided by ROB
                    bp_recover_en   = 1'b1;  // one-cycle recover pulse
                    recover         = 1'b1;  // block normal retire this cycle
                end
            end
        end

        // -------- Normal retire: multi-commit (oldest→younger; stop at first incomplete) --------
        if (!recover) begin
            for (int w = N - 1; w >= 0; w--) begin
                if (!head_valids[w]) continue;

                entry = head_entries[w];
                if (!entry.complete) break;  // in-order boundary

                if (entry.arch_rd != '0) begin
                    // Commit this lane to the architected map
                    arch_write_enables[w]  = 1'b1;
                    arch_write_addrs[w]    = entry.arch_rd;
                    arch_write_phys_regs[w] = entry.phys_rd;

                    // Freelist: free the Told (previous phys)
                    if ((entry.prev_phys_rd != '0) && (entry.prev_phys_rd < PHYS_REGS)) free_mask[entry.prev_phys_rd] = 1'b1;
                end
                // No-dest instructions (e.g., branches) retire silently.
            end
        end
    end

endmodule
