`include "sys_defs.svh"

module testbench;

    logic clock, reset;
    logic                             failed;

    // Inputs to stage_retire
    ROB_ENTRY [               `N-1:0] head_entries;
    logic     [               `N-1:0] head_valids;
    ROB_IDX   [               `N-1:0] head_idxs;

    // Outputs from stage_retire
    logic                             rob_mispredict;
    ROB_IDX                           rob_mispred_idx;
    logic                             bp_recover_en;
    logic     [`PHYS_REG_SZ_R10K-1:0] free_mask;
    logic     [               `N-1:0] arch_write_enables;
    REG_IDX   [               `N-1:0] arch_write_addrs;
    PHYS_TAG  [               `N-1:0] arch_write_phys_regs;

    stage_retire dut (
        .clock(clock),
        .reset(reset),
        .head_entries(head_entries),
        .head_valids(head_valids),
        .head_idxs(head_idxs),
        .rob_mispredict(rob_mispredict),
        .rob_mispred_idx(rob_mispred_idx),
        .bp_recover_en(bp_recover_en),
        .free_mask(free_mask),
        .arch_write_enables(arch_write_enables),
        .arch_write_addrs(arch_write_addrs),
        .arch_write_phys_regs(arch_write_phys_regs)
    );

    always begin
        #(`CLOCK_PERIOD / 2.0);
        clock = ~clock;
    end

    // Helper function to create a default empty ROB entry
    function ROB_ENTRY empty_entry;
        empty_entry = '{default: '0};
        empty_entry.valid = 1'b0;
        empty_entry.complete = 1'b0;
        empty_entry.exception = NO_ERROR;
    endfunction

    // Helper function to create a completed ALU entry with destination
    function ROB_ENTRY completed_alu_entry(input int rob_idx, input int arch_reg, input int phys_reg, input int prev_phys_reg);
        completed_alu_entry = empty_entry();
        completed_alu_entry.valid = 1'b1;
        completed_alu_entry.complete = 1'b1;
        completed_alu_entry.arch_rd = REG_IDX'(arch_reg);
        completed_alu_entry.phys_rd = PHYS_TAG'(phys_reg);
        completed_alu_entry.prev_phys_rd = PHYS_TAG'(prev_phys_reg);
        completed_alu_entry.branch = 1'b0;
    endfunction

    // Helper function to create a completed branch entry
    function ROB_ENTRY completed_branch_entry(input int rob_idx, input bit pred_taken, input int pred_target,
                                              input bit resolved_taken, input int resolved_target);
        completed_branch_entry = empty_entry();
        completed_branch_entry.valid = 1'b1;
        completed_branch_entry.complete = 1'b1;
        completed_branch_entry.branch = 1'b1;
        completed_branch_entry.pred_taken = pred_taken;
        completed_branch_entry.pred_target = ADDR'(pred_target);
        completed_branch_entry.branch_taken = resolved_taken;
        completed_branch_entry.branch_target = ADDR'(resolved_target);
    endfunction

    // Helper function to create an incomplete entry
    function ROB_ENTRY incomplete_entry(input int rob_idx, input int arch_reg, input int phys_reg, input int prev_phys_reg);
        incomplete_entry = completed_alu_entry(rob_idx, arch_reg, phys_reg, prev_phys_reg);
        incomplete_entry.complete = 1'b0;
    endfunction

    // Helper to reset and wait for proper timing
    task reset_dut;
        reset = 1;
        repeat (2) @(negedge clock);  // Hold reset over two negedges
        reset = 0;
        @(negedge clock);  // One more cycle for stability
    endtask

    // Helper to clear all inputs
    task clear_inputs;
        head_entries = '{default: '0};
        head_valids = '0;
        head_idxs = '0;
    endtask

    // Helper to check retire outputs
    function logic check_no_retire();
        return (rob_mispredict == 0 && bp_recover_en == 0 && arch_write_enables == 0 && free_mask == 0);
    endfunction

    initial begin
        int test_num = 1;
        clock  = 0;
        reset  = 1;
        failed = 0;

        // Validate N is large enough for our tests
        if (`N < 3) begin
            $display("ERROR: N must be at least 3 for this testbench to work properly");
            $finish;
        end

        // Initialize inputs
        clear_inputs();

        reset_dut();

        // Test 1: No valid entries should produce no retire activity
        $display("\nTest %0d: No valid entries", test_num++);
        reset_dut();
        clear_inputs();
        begin
            @(negedge clock);
            if (check_no_retire()) begin
                $display("  PASS: No retire activity with no valid entries");
            end else begin
                $display("  FAIL: Unexpected retire activity with no valid entries");
                failed = 1;
            end
        end

        // Test 2: Single completed ALU instruction with destination
        $display("\nTest %0d: Single completed ALU instruction with destination", test_num++);
        reset_dut();
        clear_inputs();
        begin
            // Set up a completed ALU instruction at oldest position (`N-1)
            head_valids[`N-1] = 1'b1;
            head_entries[`N-1] = completed_alu_entry(5, 10, 40, 15);  // rob_idx=5, arch_reg=10, phys=40, prev_phys=15
            head_idxs[`N-1] = ROB_IDX'(5);

            @(negedge clock);

            // Check that instruction was retired
            if (arch_write_enables[`N-1] && arch_write_addrs[`N-1] == 10 && arch_write_phys_regs[`N-1] == 40) begin
                $display("  PASS: ALU instruction committed to architectural map");
            end else begin
                $display("  FAIL: ALU instruction not properly committed (en=%b, addr=%0d, phys=%0d)", arch_write_enables[`N-1],
                         arch_write_addrs[`N-1], arch_write_phys_regs[`N-1]);
                failed = 1;
            end

            // Check that previous physical register was freed
            if (free_mask[15]) begin
                $display("  PASS: Previous physical register freed");
            end else begin
                $display("  FAIL: Previous physical register not freed");
                failed = 1;
            end

            // Check no mispredict
            if (!rob_mispredict && !bp_recover_en) begin
                $display("  PASS: No mispredict detected");
            end else begin
                $display("  FAIL: Unexpected mispredict signals");
                failed = 1;
            end
        end

        // Test 3: Multiple completed instructions (in-order retire)
        $display("\nTest %0d: Multiple completed instructions (%0d instructions)", test_num++, `N);
        reset_dut();
        clear_inputs();
        begin
            int   expected_commits = 0;
            int   actual_commits = 0;
            logic all_freed = 1;

            // Set up multiple completed instructions
            for (int i = 0; i < `N; i++) begin
                head_valids[i] = 1'b1;
                head_entries[i] = completed_alu_entry(i + 10, 20 + i, 50 + i, 5 + i);
                head_idxs[i] = ROB_IDX'(i + 10);
                expected_commits++;
            end

            @(negedge clock);

            // All should commit since all are complete
            for (int i = 0; i < `N; i++) begin
                if (arch_write_enables[i]) actual_commits++;
            end

            if (actual_commits == expected_commits) begin
                $display("  PASS: All %0d instructions committed", expected_commits);
            end else begin
                $display("  FAIL: Expected %0d commits, got %0d", expected_commits, actual_commits);
                failed = 1;
            end

            // Check that all previous registers were freed
            for (int i = 0; i < `N; i++) begin
                if (!free_mask[5+i]) all_freed = 0;
            end
            if (all_freed) begin
                $display("  PASS: All previous physical registers freed");
            end else begin
                $display("  FAIL: Not all previous physical registers freed");
                failed = 1;
            end
        end

        // Test 4: Stop at first incomplete instruction
        $display("\nTest %0d: Stop at first incomplete instruction (%0d complete + 1 incomplete)", test_num++, `N - 1);
        reset_dut();
        clear_inputs();
        begin
            int expected_commits = `N - 1;  // Should retire all but the last position
            int actual_commits = 0;

            // Set up N-1 complete instructions followed by one incomplete
            for (int i = 0; i < `N - 1; i++) begin
                head_valids[`N-1-i] = 1'b1;  // Start from oldest (N-1) down to youngest available
                head_entries[`N-1-i] = completed_alu_entry(i + 1, 5 + i, 30 + i, 10 + i);
                head_idxs[`N-1-i] = ROB_IDX'(i + 1);
            end

            // Make the youngest position incomplete (should stop here)
            head_valids[0] = 1'b1;
            head_entries[0] = incomplete_entry(`N, 5 + `N - 1, 30 + `N - 1, 10 + `N - 1);
            head_idxs[0] = ROB_IDX'(`N);

            @(negedge clock);

            // Should commit all complete instructions, but stop at the incomplete one
            for (int i = 0; i < `N; i++) begin
                if (arch_write_enables[i]) actual_commits++;
            end

            if (actual_commits == expected_commits) begin
                $display("  PASS: Retired %0d instructions, stopped at incomplete", expected_commits);
            end else begin
                $display("  FAIL: Expected %0d commits, got %0d", expected_commits, actual_commits);
                failed = 1;
            end
        end

        // Test 5: Branch mispredict detection (direction mispredict)
        $display("\nTest %0d: Branch mispredict detection (direction)", test_num++);
        reset_dut();
        clear_inputs();
        begin
            // Set up a completed branch with direction mispredict
            head_valids[`N-1] = 1'b1;
            head_entries[`N-1] = completed_branch_entry(1, 0, 100, 1, 200);  // pred not taken, resolved taken
            head_idxs[`N-1] = ROB_IDX'(1);

            @(negedge clock);

            // Should detect mispredict
            if (rob_mispredict && bp_recover_en && rob_mispred_idx == 1) begin
                $display("  PASS: Direction mispredict detected");
            end else begin
                $display("  FAIL: Direction mispredict not detected (mispred=%b, recover=%b, idx=%0d)", rob_mispredict,
                         bp_recover_en, rob_mispred_idx);
                failed = 1;
            end

            // Should not commit any instructions during recovery
            if (arch_write_enables == 0 && free_mask == 0) begin
                $display("  PASS: No commits during recovery cycle");
            end else begin
                $display("  FAIL: Unexpected commits during recovery");
                failed = 1;
            end
        end

        // Test 6: Branch mispredict detection (target mispredict)
        $display("\nTest %0d: Branch mispredict detection (target)", test_num++);
        reset_dut();
        clear_inputs();
        begin
            // Set up a completed branch with target mispredict
            head_valids[`N-1] = 1'b1;
            head_entries[`N-1] = completed_branch_entry(2, 1, 300, 1, 400);  // both taken, different targets
            head_idxs[`N-1] = ROB_IDX'(2);

            @(negedge clock);

            // Should detect mispredict
            if (rob_mispredict && bp_recover_en && rob_mispred_idx == 2) begin
                $display("  PASS: Target mispredict detected");
            end else begin
                $display("  FAIL: Target mispredict not detected (mispred=%b, recover=%b, idx=%0d)", rob_mispredict,
                         bp_recover_en, rob_mispred_idx);
                failed = 1;
            end
        end

        // Test 7: Correct branch prediction should not trigger mispredict
        $display("\nTest %0d: Correct branch prediction", test_num++);
        reset_dut();
        clear_inputs();
        begin
            // Set up a completed branch with correct prediction
            head_valids[`N-1] = 1'b1;
            head_entries[`N-1] = completed_branch_entry(3, 1, 500, 1, 500);  // both taken, same target
            head_idxs[`N-1] = ROB_IDX'(3);

            // Also set up a regular instruction to commit
            head_valids[`N-2] = 1'b1;
            head_entries[`N-2] = completed_alu_entry(4, 9, 35, 14);
            head_idxs[`N-2] = ROB_IDX'(4);

            @(negedge clock);

            // Should not detect mispredict
            if (!rob_mispredict && !bp_recover_en) begin
                $display("  PASS: No mispredict for correct branch prediction");
            end else begin
                $display("  FAIL: Unexpected mispredict for correct prediction");
                failed = 1;
            end

            // Should commit the ALU instruction
            if (arch_write_enables[`N-2] && arch_write_addrs[`N-2] == 9 && arch_write_phys_regs[`N-2] == 35) begin
                $display("  PASS: ALU instruction committed alongside correct branch");
            end else begin
                $display("  FAIL: ALU instruction not committed");
                failed = 1;
            end
        end

        // Test 8: Branch without destination doesn't write to arch map
        $display("\nTest %0d: Branch without destination", test_num++);
        reset_dut();
        clear_inputs();
        begin
            // Set up a completed branch without destination
            head_valids[`N-1] = 1'b1;
            head_entries[`N-1] = completed_branch_entry(5, 1, 600, 1, 600);
            head_entries[`N-1].arch_rd = '0;  // No destination
            head_idxs[`N-1] = ROB_IDX'(5);

            @(negedge clock);

            // Should not write to architectural map
            if (arch_write_enables == 0) begin
                $display("  PASS: Branch without destination doesn't write to arch map");
            end else begin
                $display("  FAIL: Unexpected arch map write for branch without destination");
                failed = 1;
            end

            // Should not free any registers (no prev_phys_rd to free)
            if (free_mask == 0) begin
                $display("  PASS: No registers freed for branch without destination");
            end else begin
                $display("  FAIL: Unexpected register freeing for branch without destination");
                failed = 1;
            end
        end

        // // this test is invalid as retire is fully combinational
        // // Test 9: Reset clears outputs
        // $display("\nTest %0d: Reset clears outputs", test_num++);
        // clear_inputs();
        // begin
        //     // Set up some inputs
        //     head_valids[`N-1] = 1'b1;
        //     head_entries[`N-1] = completed_alu_entry(6, 11, 36, 15);
        //     head_idxs[`N-1] = ROB_IDX'(6);

        //     @(negedge clock);

        //     // Apply reset
        //     reset = 1;
        //     @(negedge clock);

        //     // Check that outputs are cleared
        //     if (check_no_retire()) begin
        //         $display("  PASS: Reset clears all retire outputs");
        //     end else begin
        //         $display("  FAIL: Reset should clear all retire outputs");
        //         failed = 1;
        //     end
        // end

        // Test 10: Invalid entries are ignored
        $display("\nTest %0d: Invalid entries are ignored", test_num++);
        reset_dut();
        clear_inputs();
        begin
            int commit_count = 0;

            // Set up one valid and one invalid entry
            head_valids[`N-1] = 1'b1;
            head_entries[`N-1] = completed_alu_entry(7, 12, 37, 16);
            head_idxs[`N-1] = ROB_IDX'(7);

            head_valids[`N-2] = 1'b0;  // Invalid
            head_entries[`N-2] = completed_alu_entry(8, 13, 38, 17);
            head_idxs[`N-2] = ROB_IDX'(8);

            @(negedge clock);

            // Should only commit the valid entry
            for (int i = 0; i < `N; i++) begin
                if (arch_write_enables[i]) commit_count++;
            end

            if (commit_count == 1) begin
                $display("  PASS: Only valid entries committed");
            end else begin
                $display("  FAIL: Expected 1 commit, got %0d", commit_count);
                failed = 1;
            end
        end

        $display("\n");
        if (failed) begin
            $display("@@@ FAILED");
        end else begin
            $display("@@@ PASSED");
        end

        $finish;
    end

endmodule
