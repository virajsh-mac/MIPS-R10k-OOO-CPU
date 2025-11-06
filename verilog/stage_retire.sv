`include "sys_defs.svh"

module stage_retire #(
    parameter  int N          = `N,
    parameter  int ARCH_COUNT = `ARCH_REG_SZ,
    parameter  int PHYS_REGS  = `PHYS_REG_SZ_R10K,
    localparam int PRW        = (PHYS_REGS <= 2) ? 1 : $clog2(PHYS_REGS)
) (
    input logic clock,
    input logic reset,

    // From ROB: head window (0 = oldest, N-1 = youngest)
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
    output PHYS_TAG [N-1:0] arch_write_phys_regs,

    // debug output which is used to count committed instructions at retire
    output COMMIT_PACKET [N-1:0] retire_commits_dbg,

    // to Fake fetch for branching
    output logic branch_taken_o,
    output ADDR  branch_target_o,

    // to read committed data from PRF
    input DATA [`PHYS_REG_SZ_R10K-1:0] regfile_entries

);
    // debug output
    COMMIT_PACKET [N-1:0] retire_commits_dbg;

    ROB_ENTRY entry;
    logic recover;
    logic mispred_dir, mispred_tgt, mispred;

    always_comb begin
        {rob_mispredict, rob_mispred_idx, bp_recover_en, free_mask} = '0;
        arch_write_enables = '0;
        arch_write_addrs = '0;
        arch_write_phys_regs = '0;
        recover = 1'b0;
        mispred_dir = 1'b0;
        mispred_tgt = 1'b0;
        mispred = 1'b0;
        entry = '0;
        retire_commits_dbg = '0;

        // branch info for fake fetch
        branch_taken_o  = 1'b0;
        branch_target_o = '0;


        // Walk oldest -> youngest and commit until first incomplete
        for (int w = 0; w < N; w++) begin
            if (!head_valids[w]) begin
                // empty slot, skip
                continue;
            end

            entry = head_entries[w];

            // Stop at first incomplete instruction (in-order boundary)
            if (!entry.complete) begin
                break;
            end

            // Record debug info
            retire_commits_dbg[w].NPC    = entry.PC + 4;
            retire_commits_dbg[w].data   = regfile_entries[entry.phys_rd];
            retire_commits_dbg[w].reg_idx = entry.arch_rd;
            retire_commits_dbg[w].halt   = entry.halt;
            retire_commits_dbg[w].illegal = (entry.exception == ILLEGAL_INST);
            retire_commits_dbg[w].valid  = 1'b1;

            // Commit this entry (it's complete)
            if (entry.arch_rd != '0) begin
                arch_write_enables[w]  = 1'b1;
                arch_write_addrs[w]    = entry.arch_rd;
                arch_write_phys_regs[w] = entry.phys_rd;

                if ((entry.prev_phys_rd != '0) && (entry.prev_phys_rd < PHYS_REGS))
                    free_mask[entry.prev_phys_rd] = 1'b1;
            end

            // If this entry is a branch, check for mispredict (compare prediction vs actual)
            if (entry.branch) begin

                // to fake fetch (No EBR)
                if (entry.branch_taken) begin
                    branch_taken_o  = 1'b1;
                    branch_target_o = entry.branch_target;
                end

                // Only consider mispredict if branch had completed
                mispred_dir = (entry.pred_taken != entry.branch_taken);
                mispred_tgt = (entry.branch_taken && (entry.pred_target != entry.branch_target));
                mispred     = (mispred_dir || mispred_tgt);

                if (mispred) begin
                    // Commit this branch (we already did commits for this entry above),
                    rob_mispredict  = 1'b1;
                    rob_mispred_idx = head_idxs[w];   // ROB index of the mispredicted branch
                    bp_recover_en   = 1'b1;           
                    recover         = 1'b1;

                    // stop committing younger entries
                    break;
                end
            end

        end
    end


endmodule

