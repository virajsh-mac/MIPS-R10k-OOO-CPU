`include "sys_defs.svh"

// Comprehensive testbench for stage_issue module
// Tests edge cases for issue selection logic

`ifndef __SYS_DEFS_SVH__  // Override if needed for testbench
`define RS_SZ 16
`define N 3
`define CDB_SZ 3

typedef logic [5:0] PHYS_TAG;  // Assuming 64 physical regs, clog2(64)=6
typedef logic [$clog2(`RS_SZ)-1:0] RS_IDX;  // 4 bits for 16

typedef struct packed {
    logic [`CDB_SZ-1:0] valid;
    PHYS_TAG [`CDB_SZ-1:0] tags;
} CDB_PACKET;
`endif

module testbench;

    logic clock, reset;
    logic failed;

    // Inputs to stage_issue
    RS_ENTRY [`RS_SZ-1:0] entries;
    logic mispredict;
    logic head_wrap;
    logic [`NUM_FU_ALU-1:0] alu_avail;
    logic [`NUM_FU_MULT-1:0] mult_avail;
    logic [`NUM_FU_BRANCH-1:0] branch_avail;
    logic [`NUM_FU_MEM-1:0] mem_avail;

    // Outputs from stage_issue
    logic [`N-1:0] clear_valid;
    RS_IDX [`N-1:0] clear_idxs;
    logic [`N-1:0] issue_valid;
    RS_ENTRY [`N-1:0] issued_entries;

    stage_issue dut (
        .clock(clock),
        .reset(reset),
        .entries(entries),
        .mispredict(mispredict),
        .head_wrap(head_wrap),
        .alu_avail(alu_avail),
        .mult_avail(mult_avail),
        .branch_avail(branch_avail),
        .mem_avail(mem_avail),
        .clear_valid(clear_valid),
        .clear_idxs(clear_idxs),
        .issue_valid(issue_valid),
        .issued_entries(issued_entries)
    );

    always begin
        #50 clock = ~clock;  // Assume 10ns period, adjustable
    end

    // Helper task to set inputs
    task set_inputs(input RS_ENTRY [`RS_SZ-1:0] rs_entries, input logic mp, input logic hw, input logic [`NUM_FU_ALU-1:0] alu,
                    input logic [`NUM_FU_MULT-1:0] mult, input logic [`NUM_FU_BRANCH-1:0] branch,
                    input logic [`NUM_FU_MEM-1:0] mem);
        entries = rs_entries;
        mispredict = mp;
        head_wrap = hw;
        alu_avail = alu;
        mult_avail = mult;
        branch_avail = branch;
        mem_avail = mem;
        @(posedge clock);
        #10;  // Small delay to let combinational logic settle
    endtask

    // Helper function to create a default empty entry
    function RS_ENTRY empty_entry;
        empty_entry.valid = 0;
        empty_entry.opa_select = OPA_IS_RS1;
        empty_entry.opb_select = OPB_IS_RS2;
        empty_entry.op_type = OP_ALU_ADD;
        empty_entry.src1_tag = 0;
        empty_entry.src1_ready = 0;
        empty_entry.src1_value = 0;
        empty_entry.src2_tag = 0;
        empty_entry.src2_ready = 0;
        empty_entry.src2_value = 0;
        empty_entry.dest_tag = 0;
        empty_entry.rob_idx = 0;
        empty_entry.PC = 0;
        empty_entry.pred_taken = 0;
        empty_entry.pred_target = 0;
    endfunction

    // Helper function to create a ready ALU entry
    function RS_ENTRY ready_alu_entry(input int rob_idx_val, input int rob_wrap_val = 0);
        ready_alu_entry = empty_entry();
        ready_alu_entry.valid = 1;
        ready_alu_entry.op_type.category = CAT_ALU;
        ready_alu_entry.src1_ready = 1;
        ready_alu_entry.src2_ready = 1;
        ready_alu_entry.rob_idx = rob_idx_val;
        ready_alu_entry.rob_wrap = rob_wrap_val;
    endfunction

    // Helper function to create a ready MULT entry
    function RS_ENTRY ready_mult_entry(input int rob_idx_val, input int rob_wrap_val = 0);
        ready_mult_entry = empty_entry();
        ready_mult_entry.valid = 1;
        ready_mult_entry.op_type.category = CAT_MULT;
        ready_mult_entry.src1_ready = 1;
        ready_mult_entry.src2_ready = 1;
        ready_mult_entry.rob_idx = rob_idx_val;
        ready_mult_entry.rob_wrap = rob_wrap_val;
    endfunction

    // Helper function to create a ready BRANCH entry
    function RS_ENTRY ready_branch_entry(input int rob_idx_val, input int rob_wrap_val = 0);
        ready_branch_entry = empty_entry();
        ready_branch_entry.valid = 1;
        ready_branch_entry.op_type.category = CAT_BRANCH;
        ready_branch_entry.src1_ready = 1;
        ready_branch_entry.src2_ready = 1;
        ready_branch_entry.rob_idx = rob_idx_val;
        ready_branch_entry.rob_wrap = rob_wrap_val;
    endfunction

    // Helper function to create a ready MEM entry
    function RS_ENTRY ready_mem_entry(input int rob_idx_val, input int rob_wrap_val = 0);
        ready_mem_entry = empty_entry();
        ready_mem_entry.valid = 1;
        ready_mem_entry.op_type.category = CAT_MEM;
        ready_mem_entry.src1_ready = 1;
        ready_mem_entry.src2_ready = 1;
        ready_mem_entry.rob_idx = rob_idx_val;
        ready_mem_entry.rob_wrap = rob_wrap_val;
    endfunction

    // Helper function to create a not-ready entry
    function RS_ENTRY not_ready_entry(input OP_CATEGORY cat, input int rob_idx_val, input int rob_wrap_val = 0);
        not_ready_entry = empty_entry();
        not_ready_entry.valid = 1;
        not_ready_entry.op_type.category = cat;
        not_ready_entry.src1_ready = 0;
        not_ready_entry.src2_ready = 0;
        not_ready_entry.rob_idx = rob_idx_val;
        not_ready_entry.rob_wrap = rob_wrap_val;
    endfunction

    // Helper function to create an all-empty RS
    function RS_ENTRY [`RS_SZ-1:0] all_empty_rs;
        for (int i = 0; i < `RS_SZ; i++) begin
            all_empty_rs[i] = empty_entry();
        end
    endfunction

    // Helper to print RS state
    task print_rs_state(input RS_ENTRY [`RS_SZ-1:0] rs_entries, input string label);
        $display("\n=== %s ===", label);
        for (int i = 0; i < `RS_SZ; i++) begin
            if (rs_entries[i].valid) begin
                string fu_type;
                case (rs_entries[i].op_type.category)
                    CAT_ALU: fu_type = "ALU";
                    CAT_MULT: fu_type = "MULT";
                    CAT_BRANCH: fu_type = "BRANCH";
                    CAT_MEM: fu_type = "MEM";
                    default: fu_type = "UNKNOWN";
                endcase
                $display("  RS[%2d]: %s, ROB=%d, wrap=%d, ready=%b%b, age=7'b%b%b", i, fu_type, rs_entries[i].rob_idx,
                         rs_entries[i].rob_wrap, rs_entries[i].src1_ready, rs_entries[i].src2_ready,
                         rs_entries[i].rob_wrap ^ head_wrap, rs_entries[i].rob_idx);
            end
        end
        $display("");
    endtask

    // Helper to print issue results
    task print_issue_results(input string label);
        $display("=== %s ===", label);
        $display("Issue valid: %b", issue_valid);
        for (int i = 0; i < `N; i++) begin
            if (issue_valid[i]) begin
                string fu_type;
                case (issued_entries[i].op_type.category)
                    CAT_ALU: fu_type = "ALU";
                    CAT_MULT: fu_type = "MULT";
                    CAT_BRANCH: fu_type = "BRANCH";
                    CAT_MEM: fu_type = "MEM";
                    default: fu_type = "UNKNOWN";
                endcase
                $display("  Issue[%d]: RS[%d] (%s), ROB=%d, wrap=%d", i, clear_idxs[i], fu_type, issued_entries[i].rob_idx,
                         issued_entries[i].rob_wrap);
            end
        end
        $display("");
    endtask

    // Helper to check issue results
    task check_issue_results(input logic [`N-1:0] expected_valid, input RS_IDX [`N-1:0] expected_idxs,
                             input string test_name = "");
        if (issue_valid != expected_valid) begin
            $display("Issue valid mismatch in %s: expected %b, got %b", test_name, expected_valid, issue_valid);
            failed = 1;
        end
        for (int i = 0; i < `N; i++) begin
            if (expected_valid[i] && clear_idxs[i] != expected_idxs[i]) begin
                $display("Clear idx %d mismatch in %s: expected %d, got %d", i, test_name, expected_idxs[i], clear_idxs[i]);
                failed = 1;
            end
        end
    endtask

    // Helper to reset and wait for proper timing
    task reset_dut;
        reset = 1;
        @(negedge clock);
        @(negedge clock);
        reset = 0;
        @(negedge clock);
    endtask

    initial begin
        int test_num = 1;
        // $dumpfile("../stage_issue.vcd");
        // $dumpvars(0, testbench.dut);

        clock = 0;
        reset = 1;
        failed = 0;

        // Initialize inputs
        entries = all_empty_rs();
        mispredict = 0;
        head_wrap = 0;
        alu_avail = {`NUM_FU_ALU{1'b1}};  // All ALUs available
        mult_avail = {`NUM_FU_MULT{1'b1}};  // All MULTs available
        branch_avail = {`NUM_FU_BRANCH{1'b1}};  // All BRANCHs available
        mem_avail = {`NUM_FU_MEM{1'b1}};  // All MEMs available

        reset_dut();

        // Single Ready Instruction, Sufficient FU
        $display("\nTest %0d: Single Ready Instruction, Sufficient FU", test_num++);
        reset_dut();
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            test_rs[5] = ready_alu_entry(10);  // ROB idx 10, wrap 0

            // $display("BEFORE: head_wrap=0, mispredict=0, alu_avail=%b, mult_avail=%b, branch_avail=%b, mem_avail=%b",
            //          {`NUM_FU_ALU{1'b1}}, {`NUM_FU_MULT{1'b1}}, {`NUM_FU_BRANCH{1'b1}}, {`NUM_FU_MEM{1'b1}});
            // print_rs_state(test_rs, "RS State BEFORE");

            set_inputs(test_rs, 0, 0, {`NUM_FU_ALU{1'b1}}, {`NUM_FU_MULT{1'b1}}, {`NUM_FU_BRANCH{1'b1}}, {`NUM_FU_MEM{1'b1}});

            // print_issue_results("AFTER");
            check_issue_results(3'b001, '{0, 0, 5}, "Test 1");
        end

        $display("\nTest %0d: Multiple Ready Instructions, No FU Limits", test_num++);
        reset_dut();
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            test_rs[0] = ready_alu_entry(10);
            test_rs[1] = ready_mult_entry(15);
            test_rs[2] = ready_branch_entry(20);
            test_rs[3] = ready_mem_entry(25);

            // $display("BEFORE: head_wrap=0, mispredict=0, all FUs available");
            // print_rs_state(test_rs, "RS State BEFORE");

            set_inputs(test_rs, 0, 0, {`NUM_FU_ALU{1'b1}}, {`NUM_FU_MULT{1'b1}}, {`NUM_FU_BRANCH{1'b1}}, {`NUM_FU_MEM{1'b1}});

            // print_issue_results("AFTER");
            // Should issue all N=3 oldest: idx0(ALU), idx1(MULT), idx2(BRANCH)
            check_issue_results(3'b111, '{2, 1, 0}, "Test 2");
        end

        $display("\nTest %0d: Ready but Over-Issue Width", test_num++);
        reset_dut();
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            // Fill with more ready ALUs than N
            for (int i = 0; i < `RS_SZ; i++) begin
                if (i < 8) test_rs[i] = ready_alu_entry(i);  // Ages 0,1,2 (oldest (smallest) first)
            end

            set_inputs(test_rs, 0, 0, {`NUM_FU_ALU{1'b1}}, {`NUM_FU_MULT{1'b1}}, {`NUM_FU_BRANCH{1'b1}}, {`NUM_FU_MEM{1'b1}});

            // Should issue exactly N=3 oldest ALUs
            check_issue_results(3'b111, '{2, 1, 0}, "Test 3");
        end

        $display("\nTest %0d: Oldest-First Priority Within Category", test_num++);
        reset_dut();
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            test_rs[0] = ready_alu_entry(25);  // Age 25 (newer)
            test_rs[1] = ready_alu_entry(10);  // Age 10 (older)
            test_rs[2] = ready_alu_entry(20);  // Age 20 (middle)

            // $display("BEFORE: head_wrap=0, mispredict=0, alu_avail=2'b11 (2 ALUs), others=0");
            // print_rs_state(test_rs, "RS State BEFORE");

            set_inputs(test_rs, 0, 0, 2'b11, 0, 0, 0);  // Only 2 ALUs available

            // print_issue_results("AFTER");
            // Should issue 2 oldest ALUs: idx1 (age 10), idx2 (age 20)
            check_issue_results(3'b011, '{0, 2, 1}, "Test 4");
        end

        // Age Wrap-Around (Head Straddle)
        $display("\nTest %0d: Age Wrap-Around (Head Straddle)", test_num++);
        reset_dut();
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            test_rs[0] = ready_alu_entry(0, 1);  // ROB idx 0, wrap 1 (younger after XOR with head_wrap=0: age 32)
            test_rs[1] = ready_alu_entry(31, 0);  // ROB idx 31, wrap 0 (older after XOR: age 31)

            // $display("BEFORE: head_wrap=0, mispredict=0, alu_avail=2'b01 (1 ALUs), others=0");
            // print_rs_state(test_rs, "RS State BEFORE");

            set_inputs(test_rs, 0, 0, 2'b01, 0, 0, 0);  // head_wrap=0, 1 ALUs available

            // print_issue_results("AFTER");
            // Entry 1 should be older due to wrap: (0^0,31) = (0,31) adjusted age 31
            // Entry 0: (1^0,0) = (1,0) = 32 adjusted age
            check_issue_results(3'b001, '{0, 0, 1}, "Test 5");
        end

        // Physical Straddle (High Idx Old)
        $display("\nTest %0d: Physical Straddle (High Idx Old)", test_num++);
        reset_dut();
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            test_rs[0] = ready_alu_entry(31, 0);  // ROB idx 31, wrap 0 (old, age=31 with head_wrap=0)
            test_rs[1] = ready_alu_entry(0, 1);  // ROB idx 0, wrap 1 (young, age=32)

            set_inputs(test_rs, 0, 0, 2'b01, 0, 0, 0);  // head_wrap=0, 1 ALUs available

            check_issue_results(3'b001, '{0, 0, 0}, "Physical Straddle High");
        end

        // Multiple in Older Group
        $display("\nTest %0d: Multiple Older Entries", test_num++);
        reset_dut();
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            test_rs[0] = ready_alu_entry(29, 0);  // age=29
            test_rs[1] = ready_alu_entry(30, 0);  // age=30
            test_rs[2] = ready_alu_entry(31, 0);  // age=31
            test_rs[3] = ready_alu_entry(0, 1);  // age=32 young

            set_inputs(test_rs, 0, 0, 2'b11, 0, 0, 0);  // head_wrap=0, 2 ALUs available (assuming `NUM_FU_ALU >=2)

            // Issues the two oldest: rs[0] and rs[1]
            check_issue_results(3'b011, '{0, 1, 0}, "Multiple Older");
        end

        // Issue Across Boundary
        $display("\nTest %0d: Issue Across Wrap Boundary", test_num++);
        reset_dut();
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            test_rs[0] = ready_alu_entry(31, 0);  // age=31 old
            test_rs[1] = ready_alu_entry(0, 1);  // age=32 young
            test_rs[2] = ready_alu_entry(1, 1);  // age=33 younger

            set_inputs(test_rs, 0, 0, 2'b11, 0, 0, 0);  // head_wrap=0, 2 ALUs

            // Issues rs[0] and rs[1]
            check_issue_results(3'b011, '{0, 1, 0}, "Cross Boundary");
        end

        // Multiple in Young Group
        $display("\nTest %0d: Multiple Young Entries", test_num++);
        reset_dut();
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            test_rs[0] = ready_alu_entry(0, 1);  // age=32
            test_rs[1] = ready_alu_entry(1, 1);  // age=33
            test_rs[2] = ready_alu_entry(2, 1);  // age=34

            set_inputs(test_rs, 0, 0, 2'b01, 0, 0, 0);  // head_wrap=0, 1 ALU

            // Issues the oldest young: rs[0]
            check_issue_results(3'b001, '{0, 0, 0}, "Multiple Young");
        end

        // Youngest-Only RS (No Old Entries)
        $display("\nTest %0d: Youngest-Only RS (No Old Entries)", test_num++);
        reset_dut();
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            test_rs[0] = ready_alu_entry(1, 0);  // Young
            test_rs[1] = ready_alu_entry(2, 0);  // Younger
            test_rs[2] = ready_alu_entry(3, 0);  // Youngest

            set_inputs(test_rs, 0, 0, 2'b11, 0, 0, 0);  // 2 ALUs available

            // Should issue oldest among young: idx0 (age 1), then idx1 (age 2)
            check_issue_results(3'b011, '{0, 1, 0}, "Test 7");
        end

        // Per-Category FU Exhaustion
        $display("\nTest %0d: Per-Category FU Exhaustion", test_num++);
        reset_dut();
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            test_rs[0] = ready_alu_entry(10);
            test_rs[1] = ready_alu_entry(15);
            test_rs[2] = ready_alu_entry(20);  // 3rd ALU, but only 2 FUs
            test_rs[3] = ready_mem_entry(12);

            set_inputs(test_rs, 0, 0, 2'b11, 0, 0, 2'b11);  // 2 ALUs, 2 MEMs

            // Should issue 2 oldest ALUs + 1 MEM
            check_issue_results(3'b111, '{3, 1, 0}, "Test 8");
        end

        // Zero FUs Available Per Category
        $display("\nTest %0d: Zero FUs Available Per Category", test_num++);
        reset_dut();
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            test_rs[0] = ready_alu_entry(10);
            test_rs[1] = ready_mult_entry(15);
            test_rs[2] = ready_branch_entry(20);

            set_inputs(test_rs, 0, 0, 0, 0, 0, 0);  // No FUs available

            check_issue_results(3'b000, '{0, 0, 0}, "Test 9");
        end

        // Mixed Categories with Uneven Limits
        $display("\nTest %0d: Mixed Categories with Uneven Limits", test_num++);
        reset_dut();
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            test_rs[0] = ready_alu_entry(10);
            test_rs[1] = ready_branch_entry(12);  // No BRANCH FUs
            test_rs[2] = ready_mult_entry(15);
            test_rs[3] = ready_mem_entry(20);

            set_inputs(test_rs, 0, 0, 2'b11, 4'b1111, 0, 2'b11);  // 2 ALUs, 4 MULTs, 0 BRANCHs, 2 MEMs

            // Should issue ALU, MULT, MEM (skip BRANCH)
            check_issue_results(3'b111, '{3, 2, 0}, "Test 10");
        end

        // Reset During Issue
        $display("\nTest %0d: Reset During Issue", test_num++);
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            test_rs[0] = ready_alu_entry(10);

            // Apply inputs then immediately reset
            entries = test_rs;
            mispredict = 0;
            head_wrap = 0;
            alu_avail = {`NUM_FU_ALU{1'b1}};
            mult_avail = 0;
            branch_avail = 0;
            mem_avail = 0;
            @(posedge clock);
            #5;  // Partial cycle

            reset = 1;
            @(negedge clock);
            #10;

            check_issue_results(3'b000, '{0, 0, 0}, "Test 12");
        end

        // Mispredict Squashes Issue
        $display("\nTest %0d: Mispredict Squashes Issue", test_num++);
        reset_dut();
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            test_rs[0] = ready_alu_entry(10);

            set_inputs(test_rs, 1, 0, {`NUM_FU_ALU{1'b1}}, 0, 0, 0);  // mispredict=1

            check_issue_results(3'b000, '{0, 0, 0}, "Test 13");
        end

        // Invalid/Empty RS Entry
        $display("\nTest %0d: Invalid/Empty RS Entry", test_num++);
        reset_dut();
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            test_rs[0] = empty_entry();  // Invalid
            test_rs[1] = not_ready_entry(CAT_ALU, 10);  // Valid but not ready
            test_rs[2] = ready_alu_entry(15);  // Valid and ready

            set_inputs(test_rs, 0, 0, {`NUM_FU_ALU{1'b1}}, 0, 0, 0);

            //TODO double check this is the right clear index
            check_issue_results(3'b001, '{0, 0, 2}, "Test 14");
        end

        // head_wrap Impact Without Entries
        $display("\nTest %0d: head_wrap Impact Without Entries", test_num++);
        reset_dut();
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            test_rs[0] = ready_alu_entry(10, 0);

            // Test with head_wrap=1 but no wrap-around scenario
            set_inputs(test_rs, 0, 1, {`NUM_FU_ALU{1'b1}}, 0, 0, 0);

            check_issue_results(3'b001, '{0, 0, 0}, "Test 15");
        end

        // FU Availability Changes (simulate across cycles)
        $display("\nTest %0d: FU Availability Changes", test_num++);
        reset_dut();
        begin
            RS_ENTRY [`RS_SZ-1:0] test_rs = all_empty_rs();
            test_rs[0] = ready_alu_entry(10);
            test_rs[1] = ready_alu_entry(15);
            test_rs[2] = ready_alu_entry(20);
            test_rs[3] = ready_alu_entry(25);

            // First cycle: 2 ALUs available
            set_inputs(test_rs, 0, 0, 2'b11, 0, 0, 0);
            check_issue_results(3'b011, '{0, 1, 0}, "Test 17a");

            // Second cycle: 4 ALUs available, remaining entries
            reset_dut();
            test_rs = all_empty_rs();
            test_rs[0] = ready_alu_entry(20);
            test_rs[1] = ready_alu_entry(25);
            set_inputs(test_rs, 0, 0, 4'b1111, 0, 0, 0);
            check_issue_results(3'b011, '{0, 1, 0}, "Test 17b");
        end

        $display("");
        if (failed) $display("@@@ Failed\n");
        else $display("@@@ Passed\n");

        $finish;
    end

endmodule
