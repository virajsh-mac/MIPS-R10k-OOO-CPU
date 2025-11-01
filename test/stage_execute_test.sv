`include "sys_defs.svh"

// Basic testbench for stage_execute module
// Tests comprehensive functions implemented within execute stage

module testbench;

    logic clock, reset;
    logic mispredict;
    logic failed;

    // Inputs to stage_execute
    ISSUE_ENTRIES issue_entries;
    CDB_ENTRY [`N-1:0] cdb_data;
    PRF_READ_DATA prf_read_data_src1, prf_read_data_src2;

    // Outputs from stage_execute
    PRF_READ_EN prf_read_en_src1, prf_read_en_src2;
    PRF_READ_TAGS prf_read_tag_src1, prf_read_tag_src2;
    logic [`NUM_FU_MULT-1:0] mult_request;
    CDB_FU_OUTPUTS fu_outputs;

    // Signals for EX/COMP interface
    logic [`N-1:0] ex_valid;
    EX_COMPLETE_PACKET ex_comp;

    // From CDB for grant selection
    logic [`N-1:0][`NUM_FU_TOTAL-1:0] gnt_bus;

    // DUT instantiation (packed struct, no guards needed)
    stage_execute dut (
        .clock(clock),
        .reset(reset),
        .mispredict(mispredict),
        .issue_entries(issue_entries),
        .cdb_data(cdb_data),
        .prf_read_en_src1(prf_read_en_src1),
        .prf_read_en_src2(prf_read_en_src2),
        .prf_read_tag_src1(prf_read_tag_src1),
        .prf_read_tag_src2(prf_read_tag_src2),
        .prf_read_data_src1(prf_read_data_src1),
        .prf_read_data_src2(prf_read_data_src2),
        .mult_request(mult_request),
        .fu_outputs(fu_outputs),
        .ex_valid(ex_valid),
        .ex_comp(ex_comp),
        .gnt_bus(gnt_bus)
    );

    // // Monitor changes in key outputs
    // always @(fu_outputs) begin
    //     if (fu_outputs.alu[0].valid || fu_outputs.alu[1].valid || fu_outputs.alu[2].valid ||
    //         fu_outputs.mult[0].valid || fu_outputs.branch[0].valid || fu_outputs.mem[0].valid) begin
    //         $display(
    //             "[@%0t] FU_OUTPUTS CHANGED: alu[0].valid=%b (tag=%0d, data=%h), alu[1].valid=%b (tag=%0d, data=%h), alu[2].valid=%b (tag=%0d, data=%h), mult[0].valid=%b (tag=%0d, data=%h), branch[0].valid=%b (tag=%0d, data=%h), mem[0].valid=%b (tag=%0d, data=%h)",
    //             $time, fu_outputs.alu[0].valid, fu_outputs.alu[0].tag, fu_outputs.alu[0].data, fu_outputs.alu[1].valid,
    //             fu_outputs.alu[1].tag, fu_outputs.alu[1].data, fu_outputs.alu[2].valid, fu_outputs.alu[2].tag,
    //             fu_outputs.alu[2].data, fu_outputs.mult[0].valid, fu_outputs.mult[0].tag, fu_outputs.mult[0].data,
    //             fu_outputs.branch[0].valid, fu_outputs.branch[0].tag, fu_outputs.branch[0].data, fu_outputs.mem[0].valid,
    //             fu_outputs.mem[0].tag, fu_outputs.mem[0].data);
    //     end
    // end

    always begin
        #(`CLOCK_PERIOD / 2.0);
        clock = ~clock;
    end

    // Helper function to create a default empty RS entry
    function RS_ENTRY empty_rs_entry;
        empty_rs_entry.valid = 0;
        empty_rs_entry.opa_select = OPA_IS_RS1;
        empty_rs_entry.opb_select = OPB_IS_RS2;
        empty_rs_entry.op_type = OP_ALU_ADD;
        empty_rs_entry.src1_tag = 0;
        empty_rs_entry.src1_ready = 1;  // Always ready in execute stage
        empty_rs_entry.src2_tag = 0;
        empty_rs_entry.src2_ready = 1;  // Always ready in execute stage
        empty_rs_entry.src2_immediate = 0;
        empty_rs_entry.dest_tag = 0;
        empty_rs_entry.rob_idx = 0;
        empty_rs_entry.PC = 0;
        empty_rs_entry.pred_taken = 0;
        empty_rs_entry.pred_target = 0;
    endfunction

    // Helper to initialize all inputs
    task init_inputs;
        int i;
        issue_entries = '0;
        cdb_data = '0;
        prf_read_data_src1 = '0;
        prf_read_data_src2 = '0;
        gnt_bus = '0;
        mispredict = 0;
        // Explicitly clear fu related signals if needed, but combinational
    endtask

    // Helper function to create a default empty ISSUE_ENTRIES
    function ISSUE_ENTRIES empty_issue_entries;
        int i;
        for (i = 0; i < `NUM_FU_ALU; i++) begin
            empty_issue_entries.alu[i] = empty_rs_entry();
        end
        for (i = 0; i < `NUM_FU_MULT; i++) begin
            empty_issue_entries.mult[i] = empty_rs_entry();
        end
        for (i = 0; i < `NUM_FU_BRANCH; i++) begin
            empty_issue_entries.branch[i] = empty_rs_entry();
        end
        for (i = 0; i < `NUM_FU_MEM; i++) begin
            empty_issue_entries.mem[i] = empty_rs_entry();
        end
    endfunction

    // Helper function to create a ready ALU entry for execute
    function RS_ENTRY ready_alu_entry(input int rob_idx_val, input PHYS_TAG dest_tag_val = 1);
        ready_alu_entry = empty_rs_entry();
        ready_alu_entry.valid = 1;
        ready_alu_entry.op_type.category = CAT_ALU;
        ready_alu_entry.op_type.func = ADD;
        ready_alu_entry.opa_select = OPA_IS_RS1;
        ready_alu_entry.opb_select = OPB_IS_RS2;
        ready_alu_entry.src1_tag = 0;
        ready_alu_entry.src2_tag = 0;
        ready_alu_entry.dest_tag = dest_tag_val;
        ready_alu_entry.rob_idx = rob_idx_val;
        ready_alu_entry.PC = 32'h1000;
    endfunction

    // Helper function to create a ready MULT entry for execute
    function RS_ENTRY ready_mult_entry(input int rob_idx_val, input PHYS_TAG dest_tag_val = 2);
        ready_mult_entry = empty_rs_entry();
        ready_mult_entry.valid = 1;
        ready_mult_entry.op_type.category = CAT_MULT;
        ready_mult_entry.op_type.func = MUL;
        ready_mult_entry.src1_tag = 0;
        ready_mult_entry.src2_tag = 0;
        ready_mult_entry.dest_tag = dest_tag_val;
        ready_mult_entry.rob_idx = rob_idx_val;
        ready_mult_entry.PC = 32'h1004;
    endfunction

    // Helper function to create a ready BRANCH entry for execute
    function RS_ENTRY ready_branch_entry(input int rob_idx_val, input PHYS_TAG dest_tag_val = 3);
        ready_branch_entry = empty_rs_entry();
        ready_branch_entry.valid = 1;
        ready_branch_entry.op_type.category = CAT_BRANCH;
        ready_branch_entry.op_type.func = EQ;
        ready_branch_entry.opa_select = OPA_IS_RS1;
        ready_branch_entry.opb_select = OPB_IS_RS2;
        ready_branch_entry.src1_tag = 0;
        ready_branch_entry.src2_tag = 0;
        ready_branch_entry.dest_tag = dest_tag_val;
        ready_branch_entry.rob_idx = rob_idx_val;
        ready_branch_entry.PC = 32'h1008;
    endfunction

    // Helper function to create an entry with specific tags (operands always ready in execute)
    function RS_ENTRY tagged_entry(input OP_CATEGORY cat, input int rob_idx_val, input PHYS_TAG src1_tag_val = 10,
                                   input PHYS_TAG src2_tag_val = 11, input PHYS_TAG dest_tag_val = 4);
        tagged_entry = empty_rs_entry();
        tagged_entry.valid = 1;
        tagged_entry.op_type.category = cat;
        tagged_entry.opa_select = OPA_IS_RS1;
        tagged_entry.opb_select = OPB_IS_RS2;  // Assume register operands for now
        tagged_entry.src1_tag = src1_tag_val;
        tagged_entry.src2_tag = src2_tag_val;
        tagged_entry.dest_tag = dest_tag_val;
        tagged_entry.rob_idx = rob_idx_val;
        tagged_entry.PC = 32'h100C;
    endfunction

    // Helper to initialize PRF read data
    task init_prf_data;
        int i;
        for (i = 0; i < `NUM_FU_ALU; i++) begin
            prf_read_data_src1.alu[i] = 32'hAA000000 + i;
            prf_read_data_src2.alu[i] = 32'hBB000000 + i;
        end
        for (i = 0; i < `NUM_FU_MULT; i++) begin
            prf_read_data_src1.mult[i] = 32'hCC000000 + i;
            prf_read_data_src2.mult[i] = 32'hDD000000 + i;
        end
        for (i = 0; i < `NUM_FU_BRANCH; i++) begin
            prf_read_data_src1.branch[i] = 32'hEE000000 + i;
            prf_read_data_src2.branch[i] = 32'hFF000000 + i;
        end
        for (i = 0; i < `NUM_FU_MEM; i++) begin
            prf_read_data_src1.mem[i] = 32'h11000000 + i;
            prf_read_data_src2.mem[i] = 32'h22000000 + i;
        end
    endtask

    // Helper to initialize CDB data (empty)
    task init_cdb_data;
        int i;
        for (i = 0; i < `N; i++) begin
            cdb_data[i].valid = 0;
            cdb_data[i].tag   = 0;
            cdb_data[i].data  = 0;
        end
    endtask

    // Helper to print execute results
    task print_execute_results(input string label);
        $display("\n=== %s ===", label);
        $display("PRF Read Requests:");
        $display("  ALU: en_src1=%p, en_src2=%p", prf_read_en_src1.alu, prf_read_en_src2.alu);
        $display("  MULT: en_src1=%p, en_src2=%p", prf_read_en_src1.mult, prf_read_en_src2.mult);
        $display("  BRANCH: en_src1=%p, en_src2=%p", prf_read_en_src1.branch, prf_read_en_src2.branch);
        $display("  MEM: en_src1=%p, en_src2=%p", prf_read_en_src1.mem, prf_read_en_src2.mem);
        $display("");
        $display("FU Outputs:");
        $display("  ALU valid=%p, tags=%p", {fu_outputs.alu[0].valid, fu_outputs.alu[1].valid, fu_outputs.alu[2].valid}, {
                 fu_outputs.alu[0].tag, fu_outputs.alu[1].tag, fu_outputs.alu[2].tag});
        $display("  MULT valid=%p, tag=%p", fu_outputs.mult[0].valid, fu_outputs.mult[0].tag);
        $display("  BRANCH valid=%p, tag=%p", fu_outputs.branch[0].valid, fu_outputs.branch[0].tag);
        $display("  MEM valid=%p, tag=%p", fu_outputs.mem[0].valid, fu_outputs.mem[0].tag);
        $display("MULT Request: %p", mult_request);
        $display("");
    endtask

    // Helper to reset and wait for proper timing
    task reset_dut;
        reset = 1;
        init_inputs();
        @(negedge clock);
        @(negedge clock);
        reset = 0;
        @(negedge clock);
    endtask

    initial begin
        int test_num = 1;
        clock  = 0;
        reset  = 1;
        failed = 0;

        // Initialize all inputs
        init_inputs();

        // Test 2: Single ready ALU instruction should execute
        $display("\nTest %0d: Single ready ALU instruction should execute", test_num++);
        reset_dut();
        begin
            logic alu_executed;
            issue_entries = empty_issue_entries();
            issue_entries.alu[0] = ready_alu_entry(10);
            // Set PRF data for register operands (since value ignored in entry for registers)
            prf_read_data_src1.alu[0] = 32'h10;
            prf_read_data_src2.alu[0] = 32'h20;

            @(negedge clock);

            alu_executed = fu_outputs.alu[0].valid;
            if (alu_executed && (fu_outputs.alu[0].data == 32'h30)) begin  // 0x10 + 0x20 = 0x30
                $display("  PASS: ALU executed correctly (0x10 + 0x20 = 0x%h)", fu_outputs.alu[0].data);
            end else begin
                $display("  FAIL: ALU should execute and produce correct result (got 0x%h, expected 0x30)",
                         fu_outputs.alu[0].data);
                failed = 1;
            end
        end

        // Test 3: Multiple ALU instructions should all execute
        $display("\nTest %0d: Multiple ALU instructions should all execute", test_num++);
        reset_dut();
        begin
            int valid_count;
            issue_entries = empty_issue_entries();
            issue_entries.alu[0] = ready_alu_entry(10, 1);
            prf_read_data_src1.alu[0] = 32'h10;
            prf_read_data_src2.alu[0] = 32'h20;
            issue_entries.alu[1] = ready_alu_entry(11, 2);
            prf_read_data_src1.alu[1] = 32'h10;
            prf_read_data_src2.alu[1] = 32'h20;
            issue_entries.alu[2] = ready_alu_entry(12, 3);
            prf_read_data_src1.alu[2] = 32'h10;
            prf_read_data_src2.alu[2] = 32'h20;

            @(negedge clock);

            valid_count = fu_outputs.alu[0].valid + fu_outputs.alu[1].valid + fu_outputs.alu[2].valid;
            if (valid_count == 3) begin
                $display("  PASS: All %0d ALU instructions executed", valid_count);
            end else begin
                $display("  FAIL: Should execute all 3 ALU instructions, got %0d", valid_count);
                failed = 1;
            end
        end

        // Test 4: PRF read requests for register operands
        $display("\nTest %0d: PRF read requests for register operands", test_num++);
        reset_dut();
        begin
            logic read_requested;
            issue_entries = empty_issue_entries();
            issue_entries.alu[0] = tagged_entry(CAT_ALU, 10, 5, 6, 1);

            @(negedge clock);

            read_requested = prf_read_en_src1.alu[0] && prf_read_en_src2.alu[0];
            if (read_requested) begin
                $display("  PASS: PRF read requests generated for register operands");
            end else begin
                $display("  FAIL: Should request PRF reads for register operands");
                failed = 1;
            end
        end

        // Fix for CDB forwarding test (test 4 in numbering, but 5 in display)
        $display("\nTest %0d: CDB forwarding should override PRF data", test_num++);
        reset_dut();
        begin
            issue_entries = empty_issue_entries();
            issue_entries.alu[0] = tagged_entry(CAT_ALU, 10, 5, 6, 1);  // src tags 5 and 6

            // Set PRF data explicitly for this test
            prf_read_data_src1.alu[0] = 32'h00000000;  // Will be overridden by CDB for SRC1
            prf_read_data_src2.alu[0] = 32'hBB000000;  // SRC2 from PRF

            // Set up CDB with forwarding data for tag 5 (SRC1)
            cdb_data[0].valid = 1;
            cdb_data[0].tag = 5;
            cdb_data[0].data = 32'h12345678;

            @(negedge clock);

            // Expected: 0x12345678 (SRC1 from CDB) + 0xBB000000 (SRC2 from PRF) = 0xCD345678
            if (fu_outputs.alu[0].valid && (fu_outputs.alu[0].data == 32'hCD345678)) begin
                $display("  PASS: CDB forwarding worked (result = 0x%h)", fu_outputs.alu[0].data);
            end else begin
                $display("  FAIL: CDB forwarding failed (got 0x%h, expected 0x%h)", fu_outputs.alu[0].data, 32'hCD345678);
                failed = 1;
            end
        end

        // Test 6a: MULT instruction should execute after pipeline delay
        $display("\nTest %0d: MULT instruction should execute after pipeline delay", test_num++);
        reset_dut();
        begin
            logic mult_done;
            issue_entries = empty_issue_entries();
            issue_entries.mult[0] = ready_mult_entry(15);
            // Set PRF data for register operands
            prf_read_data_src1.mult[0] = 32'h3;
            prf_read_data_src2.mult[0] = 32'h4;

            // MULT takes multiple cycles
            repeat (`MULT_STAGES + 1) begin
                @(posedge clock);
                #10;
            end

            if (fu_outputs.mult[0].data == 32'hC) begin  // 3 * 4 = 12
                $display("  PASS: MULT executed correctly (3 * 4 = %0d)", fu_outputs.mult[0].data);
            end else begin
                $display("  FAIL: MULT should produce correct result (got %0d, expected 12)", fu_outputs.mult[0].data);
                failed = 1;
            end
        end

        // // Test 6b: MULT instruction should execute after pipeline delay and check metadata
        // $display("\nTest %0d: MULT instruction should execute after pipeline delay", test_num++);
        // reset_dut();
        // begin
        //     int cycles = 0;
        //     // Hold PRF data constant across cycles
        //     prf_read_data_src1.mult[0] = 32'h3;
        //     prf_read_data_src2.mult[0] = 32'h4;
        //     gnt_bus = '0;

        //     // Set issue_entries and keep valid throughout pipeline (metadata needs to flow through)
        //     @(negedge clock);
        //     issue_entries = empty_issue_entries();
        //     issue_entries.mult[0] = ready_mult_entry(15);

        //     // Wait for MULT pipeline to complete (keep issue_entries valid)
        //     while (!fu_outputs.mult[0].valid && cycles < 100) begin
        //         @(posedge clock);
        //         cycles++;
        //     end
        //     if (cycles >= 100) begin
        //         $display("  FAIL: MULT timeout after %0d cycles", cycles);
        //         failed = 1;
        //     end else begin
        //         $display("  MULT completed after %0d cycles, data=%h", cycles, fu_outputs.mult[0].data);
        //     end

        //     // Set grant for completion (one cycle)
        //     @(negedge clock);
        //     gnt_bus[0][0] = 1'b1;
        //     @(posedge clock);  // End of grant cycle
        //     gnt_bus = '0;

        //     // Check results after grant
        //     if (fu_outputs.mult[0].data == 32'hC) begin
        //         $display("  PASS: MULT executed correctly (3 * 4 = %0d)", fu_outputs.mult[0].data);
        //     end else begin
        //         $display("  FAIL: MULT should produce correct result (got %0d, expected 12)", fu_outputs.mult[0].data);
        //         failed = 1;
        //     end

        //     // Check metadata passing through to completion
        //     if (ex_valid[0] && ex_comp.rob_idx[0] == 15 && ex_comp.result[0] == 32'hC && ex_comp.dest_pr[0] == 2) begin
        //         $display("  PASS: MULT metadata passed correctly to completion (rob_idx=%0d, result=0x%h, dest_pr=%0d)",
        //                  ex_comp.rob_idx[0], ex_comp.result[0], ex_comp.dest_pr[0]);
        //     end else begin
        //         $display("  FAIL: MULT metadata not passed correctly (ex_valid[0]=%b, rob_idx=%0d, result=0x%h, dest_pr=%0d)",
        //                  ex_valid[0], ex_comp.rob_idx[0], ex_comp.result[0], ex_comp.dest_pr[0]);
        //         failed = 1;
        //     end
        // end

        // Test 7: BRANCH instruction should evaluate condition
        $display("\nTest %0d: BRANCH instruction should evaluate condition", test_num++);
        reset_dut();
        begin
            issue_entries = empty_issue_entries();
            issue_entries.branch[0] = ready_branch_entry(20);
            // Set PRF data for register operands
            prf_read_data_src1.branch[0] = 32'h50;
            prf_read_data_src2.branch[0] = 32'h50;

            @(negedge clock);

            // For EQ branch with equal operands, should produce some result
            // The actual branch logic is in conditional_branch module, just check it produces output
            if (fu_outputs.branch[0].valid) begin
                $display("  PASS: BRANCH executed (data = 0x%h)", fu_outputs.branch[0].data);
            end else begin
                $display("  FAIL: BRANCH should execute");
                failed = 1;
            end
        end

        // Fix for mispredict test (test 9)
        $display("\nTest %0d: Mispredict should clear outputs", test_num++);
        reset_dut();
        begin
            logic any_output_before, any_output_after;
            issue_entries = empty_issue_entries();
            issue_entries.alu[0] = ready_alu_entry(10);
            prf_read_data_src1.alu[0] = 32'h10;
            prf_read_data_src2.alu[0] = 32'h20;
            gnt_bus = '0;  // Ensure no grants

            @(negedge clock);

            any_output_before = |{fu_outputs.alu[0].valid, fu_outputs.alu[1].valid, fu_outputs.alu[2].valid,
                                 fu_outputs.mult[0].valid, fu_outputs.branch[0].valid, fu_outputs.mem[0].valid};

            // Set clear conditions without resetting mispredict
            mispredict = 1;
            issue_entries = '0;
            prf_read_data_src1 = '0;
            prf_read_data_src2 = '0;
            gnt_bus = '0;
            cdb_data = '0;

            @(negedge clock);

            any_output_after = |{fu_outputs.alu[0].valid, fu_outputs.alu[1].valid, fu_outputs.alu[2].valid,
                                fu_outputs.mult[0].valid, fu_outputs.branch[0].valid, fu_outputs.mem[0].valid};

            mispredict = 0;  // Reset after evaluation

            if (any_output_before && !any_output_after) begin
                $display("  PASS: Mispredict clears all outputs");
            end else begin
                $display("  FAIL: Mispredict should clear outputs (before=%b, after=%b)", any_output_before, any_output_after);
                failed = 1;
            end
        end

        // Test 10: ALU operand selection (different immediate types)
        $display("\nTest %0d: ALU operand selection with immediates", test_num++);
        reset_dut();
        begin
            RS_ENTRY test_entry;
            test_entry = ready_alu_entry(10);
            test_entry.opa_select = OPA_IS_PC;
            test_entry.opb_select = OPB_IS_I_IMM;
            test_entry.src2_immediate = 32'h100;  // I-immediate value
            issue_entries = empty_issue_entries();
            issue_entries.alu[0] = test_entry;

            @(negedge clock);

            // PC (0x1000) + I-imm (0x100) = 0x1100
            if (fu_outputs.alu[0].valid && (fu_outputs.alu[0].data == 32'h1100)) begin
                $display("  PASS: ALU operand selection worked (PC + I-imm = 0x%h)", fu_outputs.alu[0].data);
            end else begin
                $display("  FAIL: ALU operand selection failed (got 0x%h, expected 0x%h)", fu_outputs.alu[0].data, 32'h1100);
                failed = 1;
            end
        end

        // Test 11: PRF read requests for immediate operands
        $display("\nTest %0d: PRF read requests for immediate operands", test_num++);
        reset_dut();
        begin
            RS_ENTRY test_entry;
            test_entry = ready_alu_entry(10);
            test_entry.opa_select = OPA_IS_RS1;
            test_entry.opb_select = OPB_IS_I_IMM;
            test_entry.src2_immediate = 32'h100;  // I-immediate value
            issue_entries = empty_issue_entries();
            issue_entries.alu[0] = test_entry;

            @(negedge clock);

            // SRC1 should read from PRF, SRC2 should NOT read from PRF (it's an immediate)
            if (prf_read_en_src1.alu[0] && !prf_read_en_src2.alu[0]) begin
                $display("  PASS: Correct PRF read pattern for immediate operands");
            end else begin
                $display("  FAIL: Incorrect PRF read pattern (src1_en=%b, src2_en=%b)", prf_read_en_src1.alu[0],
                         prf_read_en_src2.alu[0]);
                failed = 1;
            end
        end

        // Similar fix for reset test (test 8), but manual clear
        $display("\nTest %0d: Reset should clear all outputs", test_num++);
        begin
            logic any_output_before, any_output_after;
            reset = 1;
            issue_entries = '0;
            prf_read_data_src1 = '0;
            prf_read_data_src2 = '0;
            gnt_bus = '0;
            cdb_data = '0;
            mispredict = 0;
            @(negedge clock);

            // Set input to produce output
            reset = 0;
            issue_entries.alu[0] = ready_alu_entry(10);
            prf_read_data_src1.alu[0] = 32'h10;
            prf_read_data_src2.alu[0] = 32'h20;
            gnt_bus = '0;  // No grant yet

            @(negedge clock);

            any_output_before = |{fu_outputs.alu[0].valid, fu_outputs.alu[1].valid, fu_outputs.alu[2].valid,
                                 fu_outputs.mult[0].valid, fu_outputs.branch[0].valid, fu_outputs.mem[0].valid};

            // Trigger reset
            reset = 1;
            issue_entries = '0;
            prf_read_data_src1 = '0;
            prf_read_data_src2 = '0;
            gnt_bus = '0;
            cdb_data = '0;

            @(negedge clock);

            any_output_after = |{fu_outputs.alu[0].valid, fu_outputs.alu[1].valid, fu_outputs.alu[2].valid,
                                fu_outputs.mult[0].valid, fu_outputs.branch[0].valid, fu_outputs.mem[0].valid};

            reset = 0;

            if (any_output_before && !any_output_after) begin
                $display("  PASS: Reset clears all outputs");
            end else begin
                $display("  FAIL: Reset should clear outputs (before=%b, after=%b)", any_output_before, any_output_after);
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
