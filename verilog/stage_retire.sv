`include "sys_defs.svh"

module stage_retire #(
    parameter  int N          = `N,
    parameter  int ARCH_COUNT = 32,
    parameter  int PHYS_REGS  = `PHYS_REG_SZ_R10K,
    localparam int PRW        = (PHYS_REGS <= 2) ? 1 : $clog2(PHYS_REGS)
) (
    input logic clock,
    input logic reset,

    // From ROB: head window (N-1 = oldest, 0 = youngest)
    input ROB_ENTRY [N-1:0] headEntries,
    input logic     [N-1:0] headValids,
    input ROB_IDX   [N-1:0] headIdxs,     // ROB index per head slot

    // To ROB: flush younger if head is a mispredicted branch
    output logic   robMispredict,
    output ROB_IDX robMispredIdx,

    // Global recovery pulse (tables react internally)
    output logic bpRecoverEn,

    // To freelist: bitmap of PRs to free (all committed lanes’ Told this cycle)
    output logic [PHYS_REGS-1:0] freeMask,

    // To archMapTable: N write ports (commit multiple per cycle)
    output logic    [N-1:0] archWriteEnables,
    output REG_IDX  [N-1:0] archWriteAddrs,
    output PHYS_TAG [N-1:0] archWritePhysRegs
);

    always_comb begin
        // Locals
        ROB_ENTRY headEntry;
        ROB_ENTRY entry;
        logic recover, mispredDir, mispredTgt, mispred;

        // ---- Synth-friendly defaults ----
        {robMispredict, robMispredIdx, bpRecoverEn, freeMask} = '0;
        archWriteEnables = '0;
        archWriteAddrs = '0;
        archWritePhysRegs = '0;
        recover = 1'b0;
        mispredDir = 1'b0;
        mispredTgt = 1'b0;
        mispred = 1'b0;
        headEntry = '0;
        entry = '0;

        // -------- Mispredict detect on oldest visible head --------
        if (headValids[N-1]) begin
            headEntry = headEntries[N-1];
            if (headEntry.branch && headEntry.complete) begin
                mispredDir = (headEntry.pred_taken  != headEntry.branch_taken);
                mispredTgt = (headEntry.branch_taken && (headEntry.pred_target != headEntry.branch_target));
                mispred    = (mispredDir || mispredTgt);
                if (mispred) begin
                    robMispredict = 1'b1;
                    robMispredIdx = headIdxs[N-1];  // provided by ROB
                    bpRecoverEn   = 1'b1;  // one-cycle recover pulse
                    recover       = 1'b1;  // block normal retire this cycle
                end
            end
        end

        // -------- Normal retire: multi-commit (oldest→younger; stop at first incomplete) --------
        if (!recover) begin
            for (int w = N - 1; w >= 0; w--) begin
                if (!headValids[w]) continue;

                entry = headEntries[w];
                if (!entry.complete) break;  // in-order boundary

                if (entry.arch_rd != '0) begin
                    // Commit this lane to the architected map
                    archWriteEnables[w]  = 1'b1;
                    archWriteAddrs[w]    = entry.arch_rd;
                    archWritePhysRegs[w] = entry.phys_rd;

                    // Freelist: free the Told (previous phys)
                    if ((entry.prev_phys_rd != '0) && (entry.prev_phys_rd < PHYS_REGS)) freeMask[entry.prev_phys_rd] = 1'b1;
                end
                // No-dest instructions (e.g., branches) retire silently.
            end
        end
    end

endmodule
