`include "sys_defs.svh"

module stage_retire (
    input logic clock,
    input logic reset,

    // From ROB: head window (0 = oldest, N-1 = youngest)
    input ROB_ENTRY [`N-1:0] head_entries,
    input logic     [`N-1:0] head_valids,
    input ROB_IDX   [`N-1:0] head_idxs,     // ROB index per head slot

    // To ROB: flush younger if head is a mispredicted branch
    output logic   mispredict,      // Single consolidated mispredict signal
    output ROB_IDX rob_mispred_idx, // ROB index of mispredicted branch

    // To freelist: bitmap of PRs to free (all committed lanes' Told this cycle)
    output logic [`PHYS_REG_SZ_R10K-1:0] free_mask,

    // To archMapTable: N write ports (commit multiple per cycle)
    output logic    [`N-1:0] arch_write_enables,
    output REG_IDX  [`N-1:0] arch_write_addrs,
    output PHYS_TAG [`N-1:0] arch_write_phys_regs,

    // debug output which is used to count committed instructions at retire
    output COMMIT_PACKET [`N-1:0] committed_insts,

    // Fetch stage IOs
    output ADDR branch_target_out,
    output BP_TRAIN_REQUEST train_req_o,

    // to read committed data from PRF
    input DATA [`PHYS_REG_SZ_R10K-1:0] regfile_entries,

    // From arch map table for freelist restore on mispredict
    input MAP_ENTRY [`ARCH_REG_SZ-1:0] arch_table_snapshot,

    // To freelist: restore mask on mispredict
    output logic [`PHYS_REG_SZ_R10K-1:0] freelist_restore_mask,

    // To store queue: free count
    output logic [$clog2(`N+1)-1:0]  sq_free_count

);

    localparam int PRW = (`PHYS_REG_SZ_R10K <= 2) ? 1 : $clog2(`PHYS_REG_SZ_R10K);
    localparam logic [`PHYS_REG_SZ_R10K-1:0] INITIAL_AVAIL_MASK = {{`PHYS_REG_SZ_R10K - `ARCH_REG_SZ{1'b1}}, {`ARCH_REG_SZ{1'b0}}};

    ROB_ENTRY entry;
    logic trained;
    logic mispred_dir, mispred_tgt, mispred;

    // Freelist checkpoint for mispredict restore
    logic [`PHYS_REG_SZ_R10K-1:0] freelist_checkpoint_mask, freelist_checkpoint_mask_next;


    always_comb begin
        freelist_checkpoint_mask_next = freelist_checkpoint_mask;
        {mispredict, rob_mispred_idx, free_mask} = '0;
        train_req_o = '{default: 0};
        trained = 1'b0;
        arch_write_enables = '0;
        arch_write_addrs = '0;
        arch_write_phys_regs = '0;
        mispred_dir = 1'b0;
        mispred_tgt = 1'b0;
        mispred = 1'b0;
        entry = '0;
        committed_insts = '0;
        sq_free_count = '0;

        // branch info for fetch
        branch_target_out = '0;

        // Walk oldest -> youngest and commit until first incomplete
        for (int w = 0; w < `N; w++) begin
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
            committed_insts[w].NPC    = entry.PC + 4;
            committed_insts[w].data   = regfile_entries[entry.phys_rd];
            committed_insts[w].reg_idx = entry.branch ? `ZERO_REG : entry.arch_rd;
            committed_insts[w].halt   = entry.halt;
            committed_insts[w].illegal = (entry.exception == ILLEGAL_INST);
            committed_insts[w].valid  = 1'b1;

            // Commit this entry (it's complete)
            if (entry.arch_rd != '0 && !entry.branch) begin
                arch_write_enables[w]  = 1'b1;
                arch_write_addrs[w]    = entry.arch_rd;
                arch_write_phys_regs[w] = entry.phys_rd;

                freelist_checkpoint_mask_next[arch_write_phys_regs[w]] = 1'b0;

                if ((entry.prev_phys_rd != '0) && (entry.prev_phys_rd < `PHYS_REG_SZ_R10K)) begin
                    free_mask[entry.prev_phys_rd] = 1'b1;
                    freelist_checkpoint_mask_next[entry.prev_phys_rd] = 1'b1;
                end
            end

            // handles store instructions
            if (entry.store) begin
                sq_free_count = sq_free_count + 1;
            end

            // If this entry is a branch, check for mispredict (compare prediction vs actual)
            if (entry.branch) begin

                // to fetch stage
                branch_target_out = entry.branch_target;

                // Only consider mispredict if branch had completed
                mispred_dir               = (entry.pred_taken != entry.branch_taken);
                mispred_tgt               = (entry.branch_taken && (entry.pred_target != entry.branch_target));
                mispred                   = (mispred_dir || mispred_tgt);

                // Train on retired branches
                train_req_o.valid         = 1'b1;
                train_req_o.pc            = entry.PC;
                train_req_o.actual_taken  = entry.branch_taken;
                train_req_o.actual_target = entry.branch_target;
                train_req_o.ghr_snapshot  = entry.ghr_snapshot;

                if (mispred) begin
                    // Single consolidated mispredict signal
                    mispredict      = 1'b1;
                    rob_mispred_idx = head_idxs[w];  // ROB index of the mispredicted branch
                    train_req_o.mispredict = 1'b1;

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
