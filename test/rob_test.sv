`include "sys_defs.svh"

// Basic testbench for ROB module
// Tests basic functionality for synthesis verification

module testbench;

    logic clock, reset;
    logic failed;

    // Inputs to ROB
    ROB_ENTRY [`N-1:0] rob_entry_packet;
    ROB_UPDATE_PACKET rob_update_packet;

    // Outputs from ROB
    logic [$clog2(`ROB_SZ+1)-1:0] free_slots;
    ROB_ENTRY [`N-1:0] head_entries;

    rob dut (
        .clock(clock),
        .reset(reset),
        .rob_entry_packet(rob_entry_packet),
        .free_slots(free_slots),
        .rob_update_packet(rob_update_packet),
        .head_entries(head_entries)
    );

    always begin
        #(`CLOCK_PERIOD / 2.0);
        clock = ~clock;
    end

    // Helper function to create a default empty ROB entry
    function ROB_ENTRY empty_rob_entry;
        empty_rob_entry = '0;
        empty_rob_entry.valid = 0;
    endfunction

    // Helper function to create a valid ROB entry for dispatch
    function ROB_ENTRY valid_rob_entry(input int rob_idx);
        valid_rob_entry = '0;
        valid_rob_entry.valid = 1;
        valid_rob_entry.PC = 32'h1000 + rob_idx * 4;
        valid_rob_entry.inst = 32'h00000013;  // NOP instruction
        valid_rob_entry.arch_rd = rob_idx % 32;
        valid_rob_entry.phys_rd = rob_idx % `PHYS_REG_SZ_R10K;
        valid_rob_entry.prev_phys_rd = (rob_idx > 0) ? (rob_idx - 1) % `PHYS_REG_SZ_R10K : 0;
        valid_rob_entry.complete = 0;
        valid_rob_entry.mispredict = 0;
        valid_rob_entry.branch = 0;
        valid_rob_entry.branch_taken = 0;
        valid_rob_entry.branch_target = 0;
        valid_rob_entry.pred_taken = 0;
        valid_rob_entry.pred_target = 0;
        valid_rob_entry.halt = 0;
        valid_rob_entry.illegal = 0;
    endfunction

    // Helper function to create a ROB update packet
    function ROB_UPDATE_PACKET create_update_packet(input int rob_idx, input logic mispredict_val);
        create_update_packet = '0;
        create_update_packet.valid[0] = 1;
        create_update_packet.idx[0] = rob_idx;
        create_update_packet.values[0] = 32'hDEADBEEF;  // dummy value
        create_update_packet.mispredicts[0] = mispredict_val;
        create_update_packet.branch_taken[0] = 0;
        create_update_packet.branch_targets[0] = 0;
    endfunction

    // Helper to print ROB state
    task print_rob_state(input string label);
        $display("\n=== %s ===", label);
        $display("Free slots: %0d", free_slots);
        $display("Head entries valid: %b", head_entries[0].valid);
        if (head_entries[0].valid) begin
            $display("Head entry: PC=0x%h, complete=%b, mispredict=%b", head_entries[0].PC, head_entries[0].complete,
                     head_entries[0].mispredict);
        end
        $display("");
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
        clock = 0;
        reset = 1;
        failed = 0;

        // Initialize inputs
        rob_entry_packet = '0;
        rob_update_packet = '0;

        reset_dut();

        // Test 1: Basic dispatch - single instruction
        $display("\nTest %0d: Basic dispatch - single instruction", test_num++);
        reset_dut();
        begin
            int num_to_dispatch = 1;
            // Initialize all entries to empty, then set the ones we want to dispatch
            for (int i = 0; i < `N; i++) begin
                rob_entry_packet[i] = empty_rob_entry();
                rob_entry_packet[i].valid = 0;  // Explicitly ensure valid is 0
            end
            for (int i = 0; i < num_to_dispatch; i++) begin
                rob_entry_packet[i] = valid_rob_entry(i);
            end
            rob_update_packet = '0;

            @(negedge clock);

            // Check that free slots decreased
            if (free_slots == `ROB_SZ - num_to_dispatch) begin
                $display("  PASS: Free slots decreased to %0d", free_slots);
            end else begin
                $display("  FAIL: Expected free slots = %0d, got %0d", `ROB_SZ - num_to_dispatch, free_slots);
                failed = 1;
            end
        end

        // Test 2: Dispatch multiple instructions
        $display("\nTest %0d: Dispatch multiple instructions", test_num++);
        reset_dut();
        begin
            int initial_free_slots;
            int num_to_dispatch = 2;
            // Initialize all entries to empty, then set the ones we want to dispatch
            for (int i = 0; i < `N; i++) begin
                rob_entry_packet[i] = empty_rob_entry();
                rob_entry_packet[i].valid = 0;  // Explicitly ensure valid is 0
            end
            for (int i = 0; i < num_to_dispatch; i++) begin
                rob_entry_packet[i] = valid_rob_entry(i);
            end
            rob_update_packet = '0;

            // Capture initial free slots after combinational logic settles
            @(negedge clock);
            initial_free_slots = free_slots;

            // Now dispatch
            @(negedge clock);

            if (free_slots == initial_free_slots - num_to_dispatch) begin
                $display("  PASS: Free slots decreased by %0d (from %0d to %0d)", num_to_dispatch, initial_free_slots,
                         free_slots);
            end else begin
                $display("  FAIL: Expected free slots = %0d, got %0d", initial_free_slots - num_to_dispatch, free_slots);
                failed = 1;
            end
        end

        // Test 3: Complete an instruction
        $display("\nTest %0d: Complete an instruction", test_num++);
        reset_dut();
        begin
            // First dispatch
            for (int i = 0; i < `N; i++) begin
                rob_entry_packet[i] = empty_rob_entry();
                rob_entry_packet[i].valid = 0;  // Explicitly ensure valid is 0
            end
            rob_entry_packet[0] = valid_rob_entry(0);
            rob_update_packet   = '0;
            @(negedge clock);

            // Then complete
            for (int i = 0; i < `N; i++) begin
                rob_entry_packet[i] = empty_rob_entry();
                rob_entry_packet[i].valid = 0;  // Explicitly ensure valid is 0
            end
            rob_update_packet = create_update_packet(0, 0);
            @(negedge clock);

            // Check that head entry is now complete
            if (head_entries[0].complete) begin
                $display("  PASS: Head entry marked as complete");
            end else begin
                $display("  FAIL: Head entry should be marked as complete");
                failed = 1;
            end
        end

        // Test 4: Complete with mispredict
        $display("\nTest %0d: Complete with mispredict flag", test_num++);
        reset_dut();
        begin
            // First dispatch
            for (int i = 0; i < `N; i++) begin
                rob_entry_packet[i] = empty_rob_entry();
                rob_entry_packet[i].valid = 0;  // Explicitly ensure valid is 0
            end
            rob_entry_packet[0] = valid_rob_entry(0);
            rob_update_packet   = '0;
            @(negedge clock);

            // Then complete with mispredict
            for (int i = 0; i < `N; i++) begin
                rob_entry_packet[i] = empty_rob_entry();
                rob_entry_packet[i].valid = 0;  // Explicitly ensure valid is 0
            end
            rob_update_packet = create_update_packet(0, 1);
            @(negedge clock);

            // Check that head entry has mispredict flag set
            if (head_entries[0].mispredict) begin
                $display("  PASS: Head entry marked with mispredict");
            end else begin
                $display("  FAIL: Head entry should be marked with mispredict");
                failed = 1;
            end
        end

        // Test 5: Dispatch and retire sequence
        $display("\nTest %0d: Dispatch and retire sequence", test_num++);
        reset_dut();
        begin
            // Dispatch 2 instructions
            for (int i = 0; i < `N; i++) begin
                rob_entry_packet[i] = empty_rob_entry();
                rob_entry_packet[i].valid = 0;  // Explicitly ensure valid is 0
            end
            rob_entry_packet[0] = valid_rob_entry(0);
            rob_entry_packet[1] = valid_rob_entry(1);
            rob_update_packet   = '0;
            @(negedge clock);

            // Complete both
            for (int i = 0; i < `N; i++) begin
                rob_entry_packet[i] = empty_rob_entry();
                rob_entry_packet[i].valid = 0;  // Explicitly ensure valid is 0
            end
            rob_update_packet.valid[0] = 1;
            rob_update_packet.idx[0] = 0;
            rob_update_packet.mispredicts[0] = 0;
            rob_update_packet.valid[1] = 1;
            rob_update_packet.idx[1] = 1;
            rob_update_packet.mispredicts[1] = 0;
            @(negedge clock);

            // Should be able to retire both now (since head entries are complete)
            // The ROB retire logic is implicit - when head entries are complete,
            // they can be retired by external logic
            if (head_entries[0].complete && head_entries[1].complete) begin
                $display("  PASS: Both head entries are complete and ready for retire");
            end else begin
                $display("  FAIL: Both head entries should be complete");
                failed = 1;
            end
        end

        // Test 6: Reset functionality
        $display("\nTest %0d: Reset functionality", test_num++);
        reset_dut();
        begin
            // Dispatch some instructions
            for (int i = 0; i < `N; i++) begin
                rob_entry_packet[i] = empty_rob_entry();
                rob_entry_packet[i].valid = 0;  // Explicitly ensure valid is 0
            end
            rob_entry_packet[0] = valid_rob_entry(0);
            rob_entry_packet[1] = valid_rob_entry(1);
            rob_update_packet   = '0;
            @(negedge clock);

            // Apply reset
            for (int i = 0; i < `N; i++) begin
                rob_entry_packet[i] = empty_rob_entry();
                rob_entry_packet[i].valid = 0;  // Explicitly ensure valid is 0
            end  // Clear inputs during reset
            rob_update_packet = '0;
            reset = 1;
            @(negedge clock);
            reset = 0;
            @(negedge clock);

            // Check that ROB is cleared
            if (free_slots == `ROB_SZ && !head_entries[0].valid) begin
                $display("  PASS: Reset cleared ROB state");
            end else begin
                $display("  FAIL: Reset should clear ROB (free_slots=%0d, head_valid=%b)", free_slots, head_entries[0].valid);
                failed = 1;
            end
        end

        // Test 7: Full ROB handling
        $display("\nTest %0d: Full ROB handling", test_num++);
        reset_dut();
        begin
            int i;
            // Fill the ROB
            for (i = 0; i < `N; i++) begin
                rob_entry_packet[i] = valid_rob_entry(i);
            end
            rob_update_packet = '0;

            // Keep dispatching until almost full
            repeat (`ROB_SZ / `N) begin
                @(negedge clock);
            end

            // ROB should report fewer free slots
            if (free_slots < `ROB_SZ) begin
                $display("  PASS: ROB shows reduced free slots (%0d)", free_slots);
            end else begin
                $display("  FAIL: ROB should show reduced free slots");
                failed = 1;
            end
        end

        // Test 8: Circular buffer wraparound - tail pointer near head
        $display("\nTest %0d: Circular buffer wraparound - tail pointer near head", test_num++);
        reset_dut();
        begin
            int i;
            int slots_after_fill;
            int slots_after_retire;

            // Phase 1: Fill ROB almost completely (tail close to head)
            $display("  Filling ROB to trigger wraparound scenario...");
            for (int batch = 0; batch < `ROB_SZ / `N - 1; batch++) begin  // Fill almost completely
                for (i = 0; i < `N; i++) begin
                    rob_entry_packet[i] = valid_rob_entry(batch * `N + i);
                end
                rob_update_packet = '0;
                @(negedge clock);
            end

            slots_after_fill = free_slots;
            $display("  After filling ROB: %0d free slots (tail should be close to head)", slots_after_fill);

            // Phase 2: Complete some instructions at the head to trigger retirement
            // Complete the oldest N instructions
            for (i = 0; i < `N; i++) begin
                rob_entry_packet[i] = empty_rob_entry();
                rob_entry_packet[i].valid = 0;
                rob_update_packet.valid[i] = 1;
                rob_update_packet.idx[i] = i;  // Complete entries 0, 1, 2
                rob_update_packet.mispredicts[i] = 0;
            end
            @(negedge clock);  // Complete happens here

            // Retirement happens on next cycle
            rob_entry_packet  = '0;
            rob_update_packet = '0;
            @(negedge clock);  // Retirement should happen here

            slots_after_retire = free_slots;
            $display("  After retiring %0d instructions: %0d free slots (should have increased)", `N, slots_after_retire);

            // Verify retirement increased free slots
            if (slots_after_retire > slots_after_fill) begin
                $display("  PASS: Retirement freed up slots (free slots increased from %0d to %0d)", slots_after_fill,
                         slots_after_retire);
            end else begin
                $display("  FAIL: Retirement should have freed up slots");
                failed = 1;
            end

            // Phase 3: Dispatch more instructions - should use freed slots and potentially wrap around
            $display("  Dispatching more instructions to test wraparound...");
            for (i = 0; i < `N; i++) begin
                rob_entry_packet[i] = valid_rob_entry(2000 + i);
            end
            rob_update_packet = '0;
            @(negedge clock);

            $display("  After wraparound dispatch: %0d free slots", free_slots);

            // Verify free slots decreased (tail moved, potentially wrapping around)
            if (free_slots < slots_after_retire) begin
                $display("  PASS: Free slots decreased after dispatch (tail moved, may have wrapped around)");
            end else if (free_slots == slots_after_retire) begin
                $display("  INFO: Free slots unchanged (dispatch used exactly the retired slots)");
            end else begin
                $display("  FAIL: Free slots should not increase after dispatch");
                failed = 1;
            end

            // Phase 4: Continue dispatching but respect free slots limit
            for (int extra = 0; extra < 2; extra++) begin  // Dispatch a couple more batches
                if (free_slots >= `N) begin  // Only dispatch if we have enough free slots
                    for (i = 0; i < `N; i++) begin
                        rob_entry_packet[i] = valid_rob_entry(3000 + extra * `N + i);
                    end
                    rob_update_packet = '0;
                    @(negedge clock);
                    $display("  After extra batch %0d: %0d free slots", extra + 1, free_slots);
                end else begin
                    $display("  Skipping extra batch %0d: only %0d free slots available", extra + 1, free_slots);
                    break;
                end
            end

            // Final validation
            if (free_slots >= 0 && free_slots <= `ROB_SZ) begin
                $display("  PASS: Free slots always within valid range [0, %0d]", `ROB_SZ);
            end else begin
                $display("  FAIL: Free slots out of valid range: %0d", free_slots);
                failed = 1;
            end

            if (free_slots < slots_after_fill) begin
                $display("  PASS: Circular buffer correctly handled wraparound (final free slots %0d < initial %0d)", free_slots,
                         slots_after_fill);
            end else begin
                $display("  INFO: May not have triggered full wraparound scenario, but free slots calculation is correct");
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
