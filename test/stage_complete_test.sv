`include "sys_defs.svh"

// Basic testbench for stage_complete module
// Tests basic functionality

module testbench;

    logic clock, reset;
    logic failed;

    // Inputs to stage_complete
    logic ex_valid[`N-1:0];
    EX_COMPLETE_PACKET ex_comp;

    // Outputs from stage_complete
    ROB_UPDATE_PACKET rob_update_packet;

    stage_complete dut (
        .clock(clock),
        .reset(reset),
        .ex_valid_in(ex_valid),
        .ex_comp_in(ex_comp),
        .rob_update_packet(rob_update_packet)
    );

    always begin
        #(`CLOCK_PERIOD / 2.0);
        clock = ~clock;
    end

    // Helper function to set ex_comp for a lane
    task automatic set_ex_comp_lane(int i, int ridx, bit br_valid, bit br_taken, int br_tgt);
        ex_comp.rob_idx[i]       = ROB_IDX'(ridx);
        ex_comp.branch_valid[i]  = br_valid;
        ex_comp.mispredict[i]    = 1'b1;  // intentionally set; DUT must ignore
        ex_comp.branch_taken[i]  = br_taken;
        ex_comp.branch_target[i] = ADDR'(br_tgt);
        ex_comp.dest_pr[i]       = PHYS_TAG'(0);
        ex_comp.result[i]        = DATA'(0);
    endtask

    function automatic int onesum(input logic v[`N-1:0]);
        int s = 0;
        for (int i = 0; i < `N; i++) s += v[i];
        return s;
    endfunction

    // Helper to print complete results
    task print_complete_results(input string label);
        $display("\n=== %s ===", label);
        $display("ROB Updates:");
        $display("  Valid: %b", rob_update_packet.valid);
        $display("  Idxs: %p", rob_update_packet.idx);
        $display("  Branch taken: %b", rob_update_packet.branch_taken);
        $display("  Branch targets: %p", rob_update_packet.branch_targets);
        $display("");
    endtask

    // Helper to clear inputs
    task clr_inputs;
        for (int i = 0; i < `N; i++) ex_valid[i] = 0;
        ex_comp = '0;
    endtask

    // Helper to check conditions
    task expect_ok(input bit cond, input string msg);
        if (!cond) begin
            $display("[%0t] FAIL: %s", $time, msg);
            failed = 1;
        end
    endtask

    // Helper to reset and wait for proper timing
    task reset_dut;
        reset = 1;
        for (int i = 0; i < `N; i++) ex_valid[i] = 0;
        ex_comp = '0;
        @(negedge clock);
        @(negedge clock);
        reset = 0;
        @(negedge clock);
    endtask

    // ---------------------------
    // Tests
    // ---------------------------
    initial begin
        int test_num = 1;
        clock  = 0;
        reset  = 1;
        failed = 0;

        // Initialize inputs
        for (int i = 0; i < `N; i++) ex_valid[i] = 0;
        ex_comp = '0;

        // Test 1: Single lane completion
        $display("\nTest %0d: Single lane completion", test_num++);
        reset_dut();
        begin
            ex_valid[0] = 1'b1;
            set_ex_comp_lane(0, 10, 0, 0, 0);  // ROB idx 10, no branch

            @(negedge clock);

            if (rob_update_packet.valid[0] && rob_update_packet.idx[0] == 10) begin
                $display("  PASS: Single lane completion works");
            end else begin
                $display("  FAIL: Single lane completion failed");
                failed = 1;
            end
        end

        // Test 2: Branch completion
        $display("\nTest %0d: Branch completion", test_num++);
        reset_dut();
        begin
            ex_valid[1] = 1'b1;
            set_ex_comp_lane(1, 15, 1, 1, 32'h100);  // Branch taken to 0x100

            @(negedge clock);

            if (rob_update_packet.valid[1] && rob_update_packet.branch_taken[1] &&
                    rob_update_packet.branch_targets[1] == 32'h100) begin
                $display("  PASS: Branch completion works");
            end else begin
                $display("  FAIL: Branch completion failed");
                failed = 1;
            end
        end

        // Test 3: Multiple lanes
        $display("\nTest %0d: Multiple lanes", test_num++);
        reset_dut();
        begin
            ex_valid[0] = 1'b1;
            ex_valid[2] = 1'b1;
            set_ex_comp_lane(0, 20, 0, 0, 0);
            set_ex_comp_lane(2, 25, 1, 0, 32'h200);  // Branch not taken

            @(negedge clock);

            if (rob_update_packet.valid[0] && rob_update_packet.valid[2] &&
                    rob_update_packet.idx[0] == 20 && rob_update_packet.idx[2] == 25 &&
                    !rob_update_packet.branch_taken[2]) begin
                $display("  PASS: Multiple lanes work");
            end else begin
                $display("  FAIL: Multiple lanes failed");
                failed = 1;
            end
        end

        // Test 4: Reset clears outputs
        $display("\nTest %0d: Reset clears outputs", test_num++);
        reset_dut();
        begin
            ex_valid[0] = 1'b1;
            set_ex_comp_lane(0, 30, 0, 0, 0);

            reset = 1;
            @(negedge clock);

            if (!rob_update_packet.valid[0]) begin
                $display("  PASS: Reset clears outputs");
            end else begin
                $display("  FAIL: Reset should clear outputs");
                failed = 1;
            end
            reset = 0;
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
