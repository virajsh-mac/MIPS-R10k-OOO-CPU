`include "sys_defs.svh"

// Basic testbench for map_table module
// Tests basic functionality for synthesis verification

module testbench;

    logic clock, reset;
    logic failed;

    // Inputs to map_table
    logic [`N-1:0] write_enables;
    REG_IDX [`N-1:0] write_addrs;
    PHYS_TAG [`N-1:0] write_phys_regs;
    REG_IDX [`N-1:0] read_addrs;
    CDB_ENTRY [`N-1:0] cdb_broadcasts;

    // Mispredict recovery I/O
    MAP_ENTRY [`ARCH_REG_SZ-1:0] table_restore;
    logic table_restore_en;

    // Outputs from map_table
    MAP_ENTRY [`N-1:0] read_entries;
    MAP_ENTRY [`ARCH_REG_SZ-1:0] table_snapshot;

    // Inputs to arch_map_table
    logic [`N-1:0] arch_write_enables;
    REG_IDX [`N-1:0] arch_write_addrs;
    PHYS_TAG [`N-1:0] arch_write_phys_regs;
    REG_IDX [`N-1:0] arch_read_addrs;

    // Mispredict recovery I/O for arch_map_table
    ARCH_MAP_ENTRY [`ARCH_REG_SZ-1:0] arch_table_restore;
    logic arch_table_restore_en;

    // Outputs from arch_map_table
    ARCH_MAP_ENTRY [`N-1:0] arch_read_entries;
    ARCH_MAP_ENTRY [`ARCH_REG_SZ-1:0] arch_table_snapshot;

    map_table dut (
        .clock(clock),
        .reset(reset),
        .write_enables(write_enables),
        .write_addrs(write_addrs),
        .write_phys_regs(write_phys_regs),
        .read_addrs(read_addrs),
        .read_entries(read_entries),
        .cdb_broadcasts(cdb_broadcasts),
        .table_snapshot(table_snapshot),
        .table_restore(table_restore),
        .table_restore_en(table_restore_en)
    );

    arch_map_table arch_dut (
        .clock(clock),
        .reset(reset),
        .write_enables(arch_write_enables),
        .write_addrs(arch_write_addrs),
        .write_phys_regs(arch_write_phys_regs),
        .read_addrs(arch_read_addrs),
        .read_entries(arch_read_entries),
        .table_snapshot(arch_table_snapshot),
        .table_restore(arch_table_restore),
        .table_restore_en(arch_table_restore_en)
    );

    // always @(read_entries) begin
    //     for (int i = 0; i < `N; i++) begin
    //         if (read_entries[i].phys_reg != 0 || read_entries[i].ready != 0) begin
    //             $display("[@%0t] READ_ENTRY[%0d] CHANGED: phys_reg=%0d, ready=%b", $time, i, read_entries[i].phys_reg,
    //                      read_entries[i].ready);
    //         end
    //     end
    // end

    always begin
        #(`CLOCK_PERIOD / 2.0);
        clock = ~clock;
    end

    // Helper to reset and wait for proper timing
    task reset_dut;
        reset = 1;
        repeat (2) @(negedge clock);  // Hold reset over two negedges
        reset = 0;
        @(negedge clock);  // One more cycle for stability
    endtask

    // Helper to set read addresses for all ports
    task set_read_addrs;
        for (int i = 0; i < `N; i++) begin
            read_addrs[i] = i;  // Read architectural reg i on port i
        end
    endtask

    // Helper to set arch read addresses for all ports
    task set_arch_read_addrs;
        for (int i = 0; i < `N; i++) begin
            arch_read_addrs[i] = i;  // Read architectural reg i on port i
        end
    endtask

    // Helper to clear all inputs
    task clear_inputs;
        write_enables = '0;
        write_addrs = '0;
        write_phys_regs = '0;
        cdb_broadcasts = '0;
        table_restore = '0;
        table_restore_en = 1'b0;
        arch_write_enables = '0;
        arch_write_addrs = '0;
        arch_write_phys_regs = '0;
        arch_table_restore = '0;
        arch_table_restore_en = 1'b0;
        set_read_addrs();
        set_arch_read_addrs();
    endtask

    // Helper to create a CDB broadcast entry
    function CDB_ENTRY cdb_entry(input logic valid, input PHYS_TAG tag, input DATA data = 0);
        cdb_entry.valid = valid;
        cdb_entry.tag   = tag;
        cdb_entry.data  = data;
    endfunction

    // Helper to check if a read entry matches expected values
    function logic check_entry(input int port, input PHYS_TAG expected_phys, input logic expected_ready);
        return (read_entries[port].phys_reg == expected_phys && read_entries[port].ready == expected_ready);
    endfunction

    // Helper to check if an arch read entry matches expected values
    function logic check_arch_entry(input int port, input PHYS_TAG expected_phys);
        return (arch_read_entries[port].phys_reg == expected_phys);
    endfunction

    initial begin
        int test_num = 1;
        clock  = 0;
        reset  = 1;
        failed = 0;

        // Initialize inputs
        clear_inputs();

        reset_dut();

        // Test 1: Initial identity mapping after reset
        $display("\nTest %0d: Initial identity mapping after reset", test_num++);
        reset_dut();
        begin
            logic all_correct = 1;

            @(negedge clock);

            // Check that first few registers have identity mapping and are ready
            for (int i = 0; i < `N && i < 8; i++) begin  // Check first 8 or N registers
                if (!check_entry(i, PHYS_TAG'(i), 1'b1)) begin
                    $display("  FAIL: Reg %0d should map to phys %0d and be ready, got phys=%0d ready=%b", i, i,
                             read_entries[i].phys_reg, read_entries[i].ready);
                    all_correct = 0;
                end
            end

            if (all_correct) begin
                $display("  PASS: Initial identity mapping correct");
            end else begin
                failed = 1;
            end
        end

        // Test 2: CDB broadcast updates ready bits
        $display("\nTest %0d: CDB broadcast updates ready bits", test_num++);
        reset_dut();
        clear_inputs();
        begin
            logic cdb_updated = 0;

            // Set up some mappings that are initially not ready
            write_enables[0] = 1'b1;
            write_addrs[0] = 5'd5;
            write_phys_regs[0] = 6'd40;  // Map arch 5 to phys 40

            write_enables[1] = 1'b1;
            write_addrs[1] = 5'd10;
            write_phys_regs[1] = 6'd45;  // Map arch 10 to phys 45

            // Set read addresses to match written arch regs
            read_addrs[0] = 5'd5;
            read_addrs[1] = 5'd10;

            @(negedge clock);

            // Verify mappings are set but not ready
            if (!check_entry(0, 6'd40, 1'b0) || !check_entry(1, 6'd45, 1'b0)) begin
                $display("  FAIL: Initial mappings should not be ready");
                failed = 1;
            end

            // Clear writes and send CDB broadcasts
            write_enables = '0;
            cdb_broadcasts[0] = cdb_entry(1'b1, 6'd40, 32'h1234);  // Make phys 40 ready
            cdb_broadcasts[1] = cdb_entry(1'b1, 6'd45, 32'h5678);  // Make phys 45 ready

            @(negedge clock);

            // Check that ready bits were updated
            if (check_entry(0, 6'd40, 1'b1) && check_entry(1, 6'd45, 1'b1)) begin
                $display("  PASS: CDB broadcasts updated ready bits");
            end else begin
                $display("  FAIL: CDB broadcasts should update ready bits (port0: phys=%0d ready=%b, port1: phys=%0d ready=%b)",
                         read_entries[0].phys_reg, read_entries[0].ready, read_entries[1].phys_reg, read_entries[1].ready);
                failed = 1;
            end
        end

        // Test 3: Dispatch writes override CDB updates
        $display("\nTest %0d: Dispatch writes override CDB updates", test_num++);
        reset_dut();
        clear_inputs();
        begin
            // Set up initial mapping
            write_enables[0] = 1'b1;
            write_addrs[0] = 5'd3;
            write_phys_regs[0] = 6'd50;  // Map arch 3 to phys 50

            // Set read address to match written arch reg
            read_addrs[0] = 5'd3;

            @(negedge clock);

            // Send CDB broadcast for phys 50 AND a new write to arch 3 in same cycle
            cdb_broadcasts[0] = cdb_entry(1'b1, 6'd50, 32'h9999);
            write_enables[0] = 1'b1;
            write_addrs[0] = 5'd3;
            write_phys_regs[0] = 6'd60;  // Change mapping to phys 60

            @(negedge clock);

            // Write should override CDB, so phys 60 should not be ready
            if (check_entry(0, 6'd60, 1'b0)) begin
                $display("  PASS: Dispatch write overrode CDB broadcast (new mapping not ready)");
            end else begin
                $display("  FAIL: Dispatch write should override CDB (expected phys=60 ready=0, got phys=%0d ready=%b)",
                         read_entries[0].phys_reg, read_entries[0].ready);
                failed = 1;
            end
        end

        // Test 4: Multiple simultaneous writes
        $display("\nTest %0d: Multiple simultaneous writes", test_num++);
        reset_dut();
        clear_inputs();
        begin
            logic all_correct = 1;

            // Write to multiple registers simultaneously
            for (int i = 0; i < `N; i++) begin
                write_enables[i] = 1'b1;
                write_addrs[i] = i;
                write_phys_regs[i] = 32 + i;  // Map to phys registers 32-32+N-1
            end

            @(negedge clock);

            // Check all mappings
            for (int i = 0; i < `N; i++) begin
                if (!check_entry(i, 32 + i, 1'b0)) begin
                    $display("  FAIL: Reg %0d should map to phys %0d and not be ready, got phys=%0d ready=%b", i, 32 + i,
                             read_entries[i].phys_reg, read_entries[i].ready);
                    all_correct = 0;
                end
            end

            if (all_correct) begin
                $display("  PASS: Multiple simultaneous writes correct");
            end else begin
                failed = 1;
            end
        end

        // Test 5: Reset clears all mappings
        $display("\nTest %0d: Reset clears all mappings to identity", test_num++);
        clear_inputs();
        reset_dut();
        begin
            logic all_correct = 1;

            @(negedge clock);

            // Check that registers are back to identity mapping
            for (int i = 0; i < `N && i < 8; i++) begin
                if (!check_entry(i, PHYS_TAG'(i), 1'b1)) begin
                    $display("  FAIL: After reset, reg %0d should map to phys %0d and be ready, got phys=%0d ready=%b", i, i,
                             read_entries[i].phys_reg, read_entries[i].ready);
                    all_correct = 0;
                end
            end

            if (all_correct) begin
                $display("  PASS: Reset restored identity mapping");
            end else begin
                failed = 1;
            end
        end

        // Test 6: Read ports work independently
        $display("\nTest %0d: Read ports work independently", test_num++);
        reset_dut();
        clear_inputs();
        begin
            // Set up some test mappings
            write_enables[0] = 1'b1;
            write_addrs[0] = 5'd1;
            write_phys_regs[0] = 6'd35;

            write_enables[1] = 1'b1;
            write_addrs[1] = 5'd5;
            write_phys_regs[1] = 6'd40;

            @(negedge clock);

            // Read different registers on different ports
            read_addrs[0] = 5'd1;  // Should read arch 1
            read_addrs[1] = 5'd5;  // Should read arch 5

            @(negedge clock);

            // Check independent reads
            if (check_entry(0, 6'd35, 1'b0) && check_entry(1, 6'd40, 1'b0)) begin
                $display("  PASS: Read ports work independently");
            end else begin
                $display("  FAIL: Independent reads failed (port0: phys=%0d ready=%b, port1: phys=%0d ready=%b)",
                         read_entries[0].phys_reg, read_entries[0].ready, read_entries[1].phys_reg, read_entries[1].ready);
                failed = 1;
            end
        end

        // Test 7: Mispredict recovery functionality
        $display("\nTest %0d: Mispredict recovery functionality", test_num++);
        reset_dut();
        clear_inputs();
        begin
            MAP_ENTRY [`ARCH_REG_SZ-1:0] saved_snapshot;
            logic recovery_successful = 1;

            // Phase 1: Set up initial mappings (architected state)
            write_enables[0] = 1'b1;
            write_addrs[0] = 5'd2;
            write_phys_regs[0] = 6'd32;  // Map arch 2 to phys 32

            write_enables[1] = 1'b1;
            write_addrs[1] = 5'd7;
            write_phys_regs[1] = 6'd37;  // Map arch 7 to phys 37

            @(negedge clock);

            // Capture the table snapshot (architected state)
            saved_snapshot = table_snapshot;
            $display("  Captured architected state: arch2->phys%0d, arch7->phys%0d", saved_snapshot[2].phys_reg,
                     saved_snapshot[7].phys_reg);

            // Phase 2: Perform speculative mappings (simulate branch speculation)
            clear_inputs();
            write_enables[0] = 1'b1;
            write_addrs[0] = 5'd2;
            write_phys_regs[0] = 6'd50;  // Speculative: map arch 2 to phys 50

            write_enables[1] = 1'b1;
            write_addrs[1] = 5'd7;
            write_phys_regs[1] = 6'd55;  // Speculative: map arch 7 to phys 55

            @(negedge clock);

            // Verify speculative mappings are in place
            read_addrs[0] = 5'd2;
            read_addrs[1] = 5'd7;
            @(negedge clock);

            if (!check_entry(0, 6'd50, 1'b0) || !check_entry(1, 6'd55, 1'b0)) begin
                $display("  FAIL: Speculative mappings not correctly set (expected arch2:50, arch7:55)");
                failed = 1;
                recovery_successful = 0;
            end

            // Phase 3: Simulate mispredict - restore from saved architected state
            clear_inputs();
            table_restore = saved_snapshot;
            table_restore_en = 1'b1;

            @(negedge clock);

            // Clear restore enable for next cycle
            table_restore_en = 1'b0;

            // Phase 4: Verify recovery - table should be restored to architected state
            read_addrs[0] = 5'd2;
            read_addrs[1] = 5'd7;
            @(negedge clock);

            if (check_entry(0, 6'd32, 1'b0) && check_entry(1, 6'd37, 1'b0)) begin
                $display("  PASS: Mispredict recovery successful - table restored to architected state");
            end else begin
                $display(
                    "  FAIL: Mispredict recovery failed (expected arch2:32, arch7:37, got arch2:%0d ready:%b, arch7:%0d ready:%b)",
                    read_entries[0].phys_reg, read_entries[0].ready, read_entries[1].phys_reg, read_entries[1].ready);
                failed = 1;
                recovery_successful = 0;
            end

            // Phase 5: Verify table_snapshot reflects restored state
            if (recovery_successful) begin
                if (table_snapshot[2].phys_reg == 6'd32 && table_snapshot[7].phys_reg == 6'd37) begin
                    $display("  PASS: table_snapshot correctly reflects restored state");
                end else begin
                    $display("  FAIL: table_snapshot doesn't match restored state (arch2:%0d, arch7:%0d)",
                             table_snapshot[2].phys_reg, table_snapshot[7].phys_reg);
                    failed = 1;
                end
            end
        end

        // ===== ARCHITECTED MAP TABLE TESTS =====

        // Test 8: Arch map table initial identity mapping after reset
        $display("\nTest %0d: Arch map table initial identity mapping after reset", test_num++);
        reset_dut();
        begin
            logic all_correct = 1;

            @(negedge clock);

            // Check that first few registers have identity mapping
            for (int i = 0; i < `N && i < 8; i++) begin
                if (!check_arch_entry(i, PHYS_TAG'(i))) begin
                    $display("  FAIL: Arch reg %0d should map to phys %0d, got phys=%0d", i, i, arch_read_entries[i].phys_reg);
                    all_correct = 0;
                end
            end

            if (all_correct) begin
                $display("  PASS: Arch map table initial identity mapping correct");
            end else begin
                failed = 1;
            end
        end

        // Test 9: Arch map table multiple simultaneous writes
        $display("\nTest %0d: Arch map table multiple simultaneous writes", test_num++);
        reset_dut();
        clear_inputs();
        begin
            logic all_correct = 1;

            // Write to multiple registers simultaneously
            for (int i = 0; i < `N; i++) begin
                arch_write_enables[i] = 1'b1;
                arch_write_addrs[i] = i;
                arch_write_phys_regs[i] = 64 + i;  // Map to phys registers 64-64+N-1
            end

            @(negedge clock);

            // Check all mappings
            for (int i = 0; i < `N; i++) begin
                if (!check_arch_entry(i, 64 + i)) begin
                    $display("  FAIL: Arch reg %0d should map to phys %0d, got phys=%0d", i, 64 + i,
                             arch_read_entries[i].phys_reg);
                    all_correct = 0;
                end
            end

            if (all_correct) begin
                $display("  PASS: Arch map table multiple simultaneous writes correct");
            end else begin
                failed = 1;
            end
        end

        // Test 10: Arch map table read ports work independently
        $display("\nTest %0d: Arch map table read ports work independently", test_num++);
        reset_dut();
        clear_inputs();
        begin
            // Set up some test mappings
            arch_write_enables[0] = 1'b1;
            arch_write_addrs[0] = 5'd3;
            arch_write_phys_regs[0] = 6'd35;

            arch_write_enables[1] = 1'b1;
            arch_write_addrs[1] = 5'd8;
            arch_write_phys_regs[1] = 6'd40;

            @(negedge clock);

            // Read different registers on different ports
            arch_read_addrs[0] = 5'd3;  // Should read arch 3
            arch_read_addrs[1] = 5'd8;  // Should read arch 8

            @(negedge clock);

            // Check independent reads
            if (check_arch_entry(0, 6'd35) && check_arch_entry(1, 6'd40)) begin
                $display("  PASS: Arch map table read ports work independently");
            end else begin
                $display("  FAIL: Arch independent reads failed (port0: phys=%0d, port1: phys=%0d)",
                         arch_read_entries[0].phys_reg, arch_read_entries[1].phys_reg);
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
