`include "sys_defs.svh"

module stage_retire #(
    parameter  int N          = `N,
    parameter  int ARCH_COUNT = `ARCH_REG_SZ,
    parameter  int PHYS_REGS  = `PHYS_REG_SZ_R10K,
    localparam int PRW        = (PHYS_REGS <= 2) ? 1 : $clog2(PHYS_REGS),
    localparam logic [PHYS_REGS-1:0] INITIAL_AVAIL_MASK = {{PHYS_REGS - `ARCH_REG_SZ{1'b1}}, {`ARCH_REG_SZ{1'b0}}}
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
    output logic branch_taken_out,
    output ADDR  branch_target_out,

    // to read committed data from PRF
    input DATA [`PHYS_REG_SZ_R10K-1:0] regfile_entries,

    // From arch map table for freelist restore on mispredict
    input MAP_ENTRY [`ARCH_REG_SZ-1:0] arch_table_snapshot,

    // To freelist: restore mask on mispredict
    output logic [PHYS_REGS-1:0] freelist_restore_mask

);
    // debug output
    COMMIT_PACKET [N-1:0] retire_commits_dbg;

    ROB_ENTRY entry;
    logic recover;
    logic mispred_dir, mispred_tgt, mispred;

    // Freelist checkpoint for mispredict restore
    logic [PHYS_REGS-1:0] freelist_checkpoint_mask, freelist_checkpoint_mask_next;

    always_comb begin
        freelist_checkpoint_mask_next = freelist_checkpoint_mask;
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
        branch_taken_out  = 1'b0;
        branch_target_out = '0;


        // Walk oldest -> youngest and commit until first incomplete
        for (int w = 0; w < N; w++) begin
            entry = head_entries[w];

            // Stop at first incomplete instruction (in-order boundary)
            if (!entry.complete) begin
                break;
            end

            // Record debug info
            retire_commits_dbg[w].NPC    = entry.PC + 4;
            retire_commits_dbg[w].data   = regfile_entries[entry.phys_rd];
            retire_commits_dbg[w].reg_idx = entry.branch ? `ZERO_REG : entry.arch_rd;
            retire_commits_dbg[w].halt   = entry.halt;
            retire_commits_dbg[w].illegal = (entry.exception == ILLEGAL_INST);
            retire_commits_dbg[w].valid  = 1'b1;

            // Commit this entry (it's complete)
            if (entry.arch_rd != '0 && !entry.branch) begin
                arch_write_enables[w]  = 1'b1;
                arch_write_addrs[w]    = entry.arch_rd;
                arch_write_phys_regs[w] = entry.phys_rd;

                freelist_checkpoint_mask_next[arch_write_phys_regs[w]] = 1'b0;

                if ((entry.prev_phys_rd != '0) && (entry.prev_phys_rd < PHYS_REGS)) begin
                    free_mask[entry.prev_phys_rd] = 1'b1;
                    freelist_checkpoint_mask_next[entry.prev_phys_rd] = 1'b1;
                end
            end

            // If this entry is a branch, check for mispredict (compare prediction vs actual)
            if (entry.branch) begin

                // to fake fetch (No EBR)
                if (entry.branch_taken) begin
                    branch_taken_out  = 1'b1;
                    branch_target_out = entry.branch_target;
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

    always_ff @(posedge clock) begin
        if (reset) begin
            freelist_checkpoint_mask <= INITIAL_AVAIL_MASK;  // Arch regs busy, extras free
        end else begin
            freelist_checkpoint_mask <= freelist_checkpoint_mask_next;
        end
    end

    assign freelist_restore_mask = freelist_checkpoint_mask_next;

endmodule

