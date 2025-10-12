`include "sys_defs.svh"

// Note: Since sys_defs.svh has placeholders like `define RS_SZ xx, we define them here for the testbench.
// Assume RS_SZ=16, N=3, CDB_SZ=3 as per project description.
// Also define necessary types if not fully in provided sys_defs.svh.

`ifndef __SYS_DEFS_SVH__  // Override if needed for testbench
`define RS_SZ 16
`define N 3
`define CDB_SZ 3

typedef logic [5:0] PHYS_TAG;  // Assuming 64 physical regs, clog2(64)=6
typedef logic [$clog2(`RS_SZ)-1:0] RS_IDX;  // 4 bits for 16

typedef struct packed {
    PHYS_TAG [`CDB_SZ-1:0] tags;
} CDB_PACKET;
`endif

module testbench;

    logic clock, reset;
    logic failed;

    // Inputs
    logic [`N-1:0] alloc_valid;
    RS_ENTRY [`N-1:0] alloc_entries;
    CDB_PACKET cdb_broadcast;
    logic [`N-1:0] clear_valid;
    RS_IDX [`N-1:0] clear_idxs;
    logic mispredict;

    // Outputs
    RS_ENTRY [`RS_SZ-1:0] entries;
    logic [$clog2(`RS_SZ+1)-1:0] free_count;

    rs dut (
        .clock(clock),
        .reset(reset),
        .alloc_valid(alloc_valid),
        .alloc_entries(alloc_entries),
        .cdb_broadcast(cdb_broadcast),
        .clear_valid(clear_valid),
        .clear_idxs(clear_idxs),
        .mispredict(mispredict),
        .entries(entries),
        .free_count(free_count)
    );

    always begin
        #5 clock = ~clock;  // Assume 10ns period, adjustable
    end

    // Helper task to advance one cycle with inputs
    task apply_inputs(
        input logic [`N-1:0] a_v,
        input RS_ENTRY [`N-1:0] a_e,
        input CDB_PACKET cdb,
        input logic [`N-1:0] c_v,
        input RS_IDX [`N-1:0] c_i,
        input logic misp
    );
        alloc_valid = a_v;
        alloc_entries = a_e;
        cdb_broadcast = cdb;
        clear_valid = c_v;
        clear_idxs = c_i;
        mispredict = misp;
        @(posedge clock);
        #1;  // Small delay to let combinational logic settle
    endtask

    // Helper task to check free_count
    task check_free_count(
        input logic [$clog2(`RS_SZ+1)-1:0] expected
    );
        if (free_count != expected) begin
            $display("Free count mismatch: expected %d, got %d", expected, free_count);
            failed = 1;
        end
    endtask

    // Helper task to print full expected and actual RS arrays
    task print_rs_arrays(
        input RS_ENTRY [`RS_SZ-1:0] expected
    );
        $display("EXPECTED RS:");
        for (int i = 0; i < `RS_SZ; i++) begin
            $display("  Entry %2d: valid=%b, src1_tag=%h, src1_ready=%b, src2_tag=%h, src2_ready=%b",
                     i, expected[i].valid, expected[i].src1_tag, expected[i].src1_ready, expected[i].src2_tag, expected[i].src2_ready);
        end
        $display("ACTUAL RS:");
        for (int i = 0; i < `RS_SZ; i++) begin
            $display("  Entry %2d: valid=%b, src1_tag=%h, src1_ready=%b, src2_tag=%h, src2_ready=%b",
                     i, entries[i].valid, entries[i].src1_tag, entries[i].src1_ready, entries[i].src2_tag, entries[i].src2_ready);
        end
    endtask

    // Helper task to check all entries against expected
    task check_entries(
        input RS_ENTRY [`RS_SZ-1:0] expected
    );
        logic mismatch;
        mismatch = 0;
        for (int i = 0; i < `RS_SZ; i++) begin
            if (entries[i].valid != expected[i].valid ||
                entries[i].src1_tag != expected[i].src1_tag ||
                entries[i].src1_ready != expected[i].src1_ready ||
                entries[i].src2_tag != expected[i].src2_tag ||
                entries[i].src2_ready != expected[i].src2_ready) begin
                $display("Entry %d mismatch: expected valid=%b, src1_tag=%h, src1_ready=%b, src2_tag=%h, src2_ready=%b",
                         i, expected[i].valid, expected[i].src1_tag, expected[i].src1_ready, expected[i].src2_tag, expected[i].src2_ready);
                $display("               got valid=%b, src1_tag=%h, src1_ready=%b, src2_tag=%h, src2_ready=%b",
                         entries[i].valid, entries[i].src1_tag, entries[i].src1_ready, entries[i].src2_tag, entries[i].src2_ready);
                mismatch = 1;
            end
        end
        if (mismatch) failed = 1;
    endtask

    // Helper to create a default empty entry
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

    // Helper to create a default CDB packet (no broadcasts)
    function CDB_PACKET empty_cdb;
        empty_cdb.valid = 3'b000;
        empty_cdb.tags = '{6'h0, 6'h0, 6'h0};
    endfunction

    // Helper to create an expected array with all empty
    function RS_ENTRY [`RS_SZ-1:0] all_empty;
        for (int i = 0; i < `RS_SZ; i++) begin
            all_empty[i] = empty_entry();
        end
    endfunction

    initial begin
        $dumpfile("../rs.vcd");
        $dumpvars(0, testbench.dut);

        clock = 0;
        reset = 1;
        failed = 0;
        alloc_valid = 0;
        clear_valid = 0;
        mispredict = 0;
        cdb_broadcast.tags = '{0,0,0};
        @(negedge clock);
        @(negedge clock);
        reset = 0;
        @(negedge clock);

        // Test 1: Reset state
        $display("\nTest 1: Reset state");
        reset = 1;
        @(negedge clock);
        reset = 0;
        @(negedge clock);
        check_free_count(`N);
        check_entries(all_empty());

        // Test 2: Allocate single entry
        $display("\nTest 2: Allocate single entry");
        begin
            RS_ENTRY [`N-1:0] a_e;
            a_e[0] = empty_entry();
            a_e[0].valid = 1;
            a_e[0].src1_tag = 6'h01;
            a_e[0].src1_ready = 0;
            a_e[0].src2_tag = 6'h02;
            a_e[0].src2_ready = 0;
            apply_inputs(3'b001, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);
        end
        begin
            RS_ENTRY [`RS_SZ-1:0] exp = all_empty();
            exp[`RS_SZ-1] = alloc_entries[0];  // priority selector inserts top-bottom alternating
            // print_rs_arrays(exp);  // Debug: print full expected and actual RS
            check_entries(exp);
            check_free_count(`N);  // Still >N free
        end

        // // Test 3: Allocate multiple (3) entries to lowest indices
        // $display("\nTest 3: Allocate multiple (3) entries to lowest indices");
        // reset = 1; @(negedge clock); reset = 0; @(negedge clock);  // Reset for clean start
        // begin
        //     RS_ENTRY [`N-1:0] a_e;
        //     for (int i = 0; i < `N; i++) begin
        //         a_e[i] = empty_entry();
        //         a_e[i].valid = 1;
        //         a_e[i].src1_tag = 6'h10 + i;
        //         a_e[i].src1_ready = 0;
        //         a_e[i].src2_tag = 6'h20 + i;
        //         a_e[i].src2_ready = 0;
        //     end
        //     apply_inputs(3'b111, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);
        // end
        // begin
        //     RS_ENTRY [`RS_SZ-1:0] exp = all_empty();
        //     for (int i = 0; i < `N; i++) exp[i] = alloc_entries[i];
        //     check_entries(exp);
        //     check_free_count(3);
        // end

        // // Test 4: Allocate when some occupied, check next lowest free
        // $display("\nTest 4: Allocate when some occupied, check next lowest free");
        // // Continue from Test 3 state (indices 0-2 occupied)
        // begin
        //     RS_ENTRY [`N-1:0] a_e;
        //     a_e[0] = empty_entry();
        //     a_e[0].valid = 1;
        //     a_e[0].src1_tag = 6'h03;
        //     a_e[0].src1_ready = 0;
        //     a_e[0].src2_tag = 6'h04;
        //     a_e[0].src2_ready = 0;
        //     apply_inputs(3'b001, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);
        // end
        // begin
        //     RS_ENTRY [`RS_SZ-1:0] exp;
        //     for (int i = 0; i < `RS_SZ; i++) exp[i] = entries[i];  // Current state
        //     exp[3] = alloc_entries[0];
        //     check_entries(exp);
        //     check_free_count(3);
        // end

        // // Test 5: Fill up the RS and attempt over-allocation
        // $display("\nTest 5: Fill up the RS and attempt over-allocation");
        // reset = 1; @(negedge clock); reset = 0; @(negedge clock);
        // begin
        //     RS_ENTRY [`N-1:0] a_e_base;
        //     for (int i = 0; i < `N; i++) begin
        //         a_e_base[i] = empty_entry();
        //         a_e_base[i].valid = 1;
        //         a_e_base[i].src1_tag = 6'h20 + i;
        //         a_e_base[i].src1_ready = 0;
        //         a_e_base[i].src2_tag = 6'h30 + i;
        //         a_e_base[i].src2_ready = 0;
        //     end
        //     // Allocate 3 per cycle until full
        //     for (int c = 0; c < 5; c++) begin  // 5*3=15
        //         apply_inputs(3'b111, a_e_base, empty_cdb(), 3'b000, '{0,0,0}, 0);
        //     end
        //     // One more to make 16
        //     apply_inputs(3'b001, a_e_base, empty_cdb(), 3'b000, '{0,0,0}, 0);
        //     check_free_count(0);
        //     // Attempt to allocate 3 more, should not happen
        //     apply_inputs(3'b111, a_e_base, empty_cdb(), 3'b000, '{0,0,0}, 0);
        //     check_free_count(0);  // No change
        // end

        // // Test 6: Wakeup single operand
        // $display("\nTest 6: Wakeup single operand");
        // reset = 1; @(negedge clock); reset = 0; @(negedge clock);
        // begin
        //     RS_ENTRY [`N-1:0] a_e;
        //     CDB_PACKET cdb;  // Moved to top
        //     a_e[0] = empty_entry();
        //     a_e[0].valid = 1;
        //     a_e[0].src1_tag = 6'h05;
        //     a_e[0].src1_ready = 0;
        //     a_e[0].src2_tag = 6'h06;
        //     a_e[0].src2_ready = 1;  // src2 already ready
        //     apply_inputs(3'b001, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);
        //     // Broadcast src1 tag
        //     cdb.tags = '{6'h05, 0, 0};
        //     apply_inputs(3'b000, a_e, cdb, 3'b000, '{0,0,0}, 0);
        // end
        // begin
        //     RS_ENTRY [`RS_SZ-1:0] exp = all_empty();
        //     exp[0].valid = 1;
        //     exp[0].src1_tag = 6'h05;
        //     exp[0].src1_ready = 1;  // Woken
        //     exp[0].src2_tag = 6'h06;
        //     exp[0].src2_ready = 1;
        //     check_entries(exp);
        // end

        // // Test 7: Wakeup multiple sources in one cycle
        // $display("\nTest 7: Wakeup multiple sources in one cycle");
        // reset = 1; @(negedge clock); reset = 0; @(negedge clock);
        // begin
        //     RS_ENTRY [`N-1:0] a_e;
        //     CDB_PACKET cdb;  // Moved to top
        //     a_e[0] = empty_entry();
        //     a_e[0].valid = 1;
        //     a_e[0].src1_tag = 6'h07;
        //     a_e[0].src1_ready = 0;
        //     a_e[0].src2_tag = 6'h08;
        //     a_e[0].src2_ready = 0;
        //     a_e[1] = empty_entry();
        //     a_e[1].valid = 1;
        //     a_e[1].src1_tag = 6'h09;
        //     a_e[1].src1_ready = 0;
        //     a_e[1].src2_tag = 6'h0A;
        //     a_e[1].src2_ready = 0;
        //     apply_inputs(3'b011, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);
        //     // Broadcast multiple tags
        //     cdb.tags = '{6'h07, 6'h08, 6'h09};
        //     apply_inputs(3'b000, a_e, cdb, 3'b000, '{0,0,0}, 0);
        // end
        // begin
        //     RS_ENTRY [`RS_SZ-1:0] exp = all_empty();
        //     exp[0].valid = 1;
        //     exp[0].src1_ready = 1;
        //     exp[0].src2_ready = 1;
        //     exp[0].src1_tag = 6'h07;
        //     exp[0].src2_tag = 6'h08;
        //     exp[1].valid = 1;
        //     exp[1].src1_ready = 1;
        //     exp[1].src2_ready = 0;  // 0x0A not broadcast
        //     exp[1].src1_tag = 6'h09;
        //     exp[1].src2_tag = 6'h0A;
        //     check_entries(exp);
        // end

        // // Test 8: Wakeup same tag for src1 and src2 in one entry
        // $display("\nTest 8: Wakeup same tag for src1 and src2 in one entry");
        // reset = 1; @(negedge clock); reset = 0; @(negedge clock);
        // begin
        //     RS_ENTRY [`N-1:0] a_e;
        //     CDB_PACKET cdb;  // Moved to top
        //     a_e[0] = empty_entry();
        //     a_e[0].valid = 1;
        //     a_e[0].src1_tag = 6'h0B;
        //     a_e[0].src1_ready = 0;
        //     a_e[0].src2_tag = 6'h0B;  // Same tag
        //     a_e[0].src2_ready = 0;
        //     apply_inputs(3'b001, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);
        //     // Broadcast the tag
        //     cdb.tags = '{6'h0B, 0, 0};
        //     apply_inputs(3'b000, a_e, cdb, 3'b000, '{0,0,0}, 0);
        // end
        // begin
        //     RS_ENTRY [`RS_SZ-1:0] exp = all_empty();
        //     exp[0].valid = 1;
        //     exp[0].src1_ready = 1;
        //     exp[0].src2_ready = 1;
        //     exp[0].src1_tag = 6'h0B;
        //     exp[0].src2_tag = 6'h0B;
        //     check_entries(exp);
        // end

        // // Test 9: Broadcast tag not matching any
        // $display("\nTest 9: Broadcast tag not matching any");
        // // Continue from Test 8 state
        // begin
        //     CDB_PACKET cdb;  // Already at top
        //     RS_ENTRY [`RS_SZ-1:0] exp;  // Moved to top for declaration order
        //     cdb.tags = '{6'h3F, 6'h3E, 6'h3D};  // Valid 6-bit, no match
        //     apply_inputs(3'b000, '{empty_entry(), empty_entry(), empty_entry()}, cdb, 3'b000, '{0,0,0}, 0);
        //     // State should unchanged
        //     exp = all_empty();
        //     exp[0].valid = 1;
        //     exp[0].src1_ready = 1;
        //     exp[0].src2_ready = 1;
        //     exp[0].src1_tag = 6'h0B;
        //     exp[0].src2_tag = 6'h0B;
        //     check_entries(exp);
        // end

        // // Test 10: Clear single entry
        // $display("\nTest 10: Clear single entry");
        // reset = 1; @(negedge clock); reset = 0; @(negedge clock);
        // begin
        //     RS_ENTRY [`N-1:0] a_e;
        //     a_e[0] = empty_entry();
        //     a_e[0].valid = 1;
        //     a_e[0].src1_tag = 6'h0C;
        //     a_e[0].src1_ready = 1;
        //     a_e[0].src2_tag = 6'h0D;
        //     a_e[0].src2_ready = 1;
        //     apply_inputs(3'b001, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);
        //     // Clear index 0
        //     apply_inputs(3'b000, a_e, empty_cdb(), 3'b001, '{4'd0,0,0}, 0);
        // end
        // check_entries(all_empty());
        // check_free_count(3);

        // // Test 11: Clear and allocate in same cycle (check no reuse in same cycle)
        // $display("\nTest 11: Clear and allocate in same cycle (check no reuse in same cycle)");
        // reset = 1; @(negedge clock); reset = 0; @(negedge clock);
        // begin
        //     RS_ENTRY [`N-1:0] a_e;
        //     a_e[0] = empty_entry();
        //     a_e[0].valid = 1;
        //     a_e[0].src1_tag = 6'h0E;
        //     a_e[0].src1_ready = 0;
        //     a_e[0].src2_tag = 6'h0F;
        //     a_e[0].src2_ready = 0;
        //     apply_inputs(3'b001, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);  // Allocate to 0
        // end
        // begin
        //     RS_ENTRY [`N-1:0] a_e_new;
        //     a_e_new[0] = empty_entry();
        //     a_e_new[0].valid = 1;
        //     a_e_new[0].src1_tag = 6'h10;
        //     a_e_new[0].src1_ready = 0;
        //     a_e_new[0].src2_tag = 6'h11;
        //     a_e_new[0].src2_ready = 0;
        //     // Clear 0 and allocate 1 new (should go to 1, not reuse 0 yet)
        //     apply_inputs(3'b001, a_e_new, empty_cdb(), 3'b001, '{4'd0,0,0}, 0);
        // end
        // begin
        //     RS_ENTRY [`RS_SZ-1:0] exp = all_empty();
        //     exp[1] = alloc_entries[0];  // New alloc to next free (1)
        //     check_entries(exp);
        //     check_free_count(3);
        // end

        // // Test 12: Mispredict flush
        // $display("\nTest 12: Mispredict flush");
        // reset = 1; @(negedge clock); reset = 0; @(negedge clock);
        // begin
        //     RS_ENTRY [`N-1:0] a_e;
        //     for (int i = 0; i < `N; i++) begin
        //         a_e[i] = empty_entry();
        //         a_e[i].valid = 1;
        //     end
        //     apply_inputs(3'b111, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);  // Allocate 3
        //     apply_inputs(3'b000, a_e, empty_cdb(), 3'b000, '{0,0,0}, 1);  // Mispredict (now in scope)
        // end
        // check_entries(all_empty());
        // check_free_count(3);

        // // Test 13: Mispredict with simultaneous alloc (should ignore alloc)
        // $display("\nTest 13: Mispredict with simultaneous alloc (should ignore alloc)");
        // reset = 1; @(negedge clock); reset = 0; @(negedge clock);
        // begin
        //     RS_ENTRY [`N-1:0] a_e;
        //     for (int i = 0; i < `N; i++) begin
        //         a_e[i] = empty_entry();
        //         a_e[i].valid = 1;
        //     end
        //     apply_inputs(3'b111, a_e, empty_cdb(), 3'b000, '{0,0,0}, 1);  // Alloc + mispredict
        // end
        // check_entries(all_empty());
        // check_free_count(3);

        // // Test 14: Stress test - mixed operations over multiple cycles
        // $display("\nTest 14: Stress test - mixed alloc, wakeup, clear over multiple cycles");
        // reset = 1; @(negedge clock); reset = 0; @(negedge clock);
        // begin
        //     RS_ENTRY [`N-1:0] a_e;
        //     // Cycle 1: Alloc 2 entries
        //     a_e[0] = empty_entry(); a_e[0].valid = 1; a_e[0].src1_tag = 6'h12; a_e[0].src1_ready = 0; a_e[0].src2_tag = 6'h13; a_e[0].src2_ready = 0;
        //     a_e[1] = empty_entry(); a_e[1].valid = 1; a_e[1].src1_tag = 6'h14; a_e[1].src1_ready = 0; a_e[1].src2_tag = 6'h15; a_e[1].src2_ready = 0;
        //     apply_inputs(3'b011, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);
        // end
        // begin
        //     RS_ENTRY [`N-1:0] a_e;  // Local declaration for cycle 2
        //     CDB_PACKET cdb;  // Already at top
        //     // Cycle 2: Wakeup some, alloc 1 more, clear 1
        //     cdb.tags = '{6'h12, 6'h14, 0};
        //     a_e[0] = empty_entry(); a_e[0].valid = 1; a_e[0].src1_tag = 6'h16; a_e[0].src1_ready = 0; a_e[0].src2_tag = 6'h17; a_e[0].src2_ready = 0;
        //     apply_inputs(3'b001, a_e, cdb, 3'b001, '{4'd0,0,0}, 0);  // Clear 0, alloc to 2 (next free)
        // end
        // begin
        //     RS_ENTRY [`RS_SZ-1:0] exp = all_empty();
        //     exp[1].valid = 1; exp[1].src1_tag = 6'h14; exp[1].src1_ready = 1; exp[1].src2_tag = 6'h15; exp[1].src2_ready = 0;
        //     exp[2].valid = 1; exp[2].src1_tag = 6'h16; exp[2].src1_ready = 0; exp[2].src2_tag = 6'h17; exp[2].src2_ready = 0;
        //     check_entries(exp);
        // end
        // begin
        //     CDB_PACKET cdb;  // Moved to top
        //     // Cycle 3: More wakeup, clear 2, no alloc
        //     cdb.tags = '{6'h15, 6'h17, 6'h16};
        //     apply_inputs(3'b000, '{empty_entry(), empty_entry(), empty_entry()}, cdb, 3'b011, '{4'd1,4'd2,0}, 0);  // Clear 1 and 2 (no alloc)
        // end
        // check_entries(all_empty());

        // // Test 15: Allocate with non-consecutive alloc_valid (stress priority assignment)
        // $display("\nTest 15: Allocate with non-consecutive alloc_valid (stress priority)");
        // reset = 1; @(negedge clock); reset = 0; @(negedge clock);
        // begin
        //     RS_ENTRY [`N-1:0] a_e;
        //     a_e[0] = empty_entry(); a_e[0].valid = 1;  // Will be skipped since alloc_valid[0]=0
        //     a_e[1] = empty_entry(); a_e[1].valid = 1;
        //     a_e[2] = empty_entry(); a_e[2].valid = 1;
        //     apply_inputs(3'b110, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);  // valid[2]=1, [1]=1, [0]=0 -> assign to 0 (for i=1), 1 (for i=2)
        // end
        // begin
        //     RS_ENTRY [`RS_SZ-1:0] exp = all_empty();
        //     exp[0].valid = 1;  // a_e[1] to lowest
        //     exp[1].valid = 1;  // a_e[2] to next
        //     check_entries(exp);
        // end

        // // Test 16: Become ready and clear in same cycle (wakeup then clear)
        // $display("\nTest 16: Become ready and clear in same cycle (wakeup then clear)");
        // reset = 1; @(negedge clock); reset = 0; @(negedge clock);
        // begin
        //     RS_ENTRY [`N-1:0] a_e;
        //     a_e[0] = empty_entry();
        //     a_e[0].valid = 1;
        //     a_e[0].src1_tag = 6'h18;
        //     a_e[0].src1_ready = 0;
        //     a_e[0].src2_tag = 6'h19;
        //     a_e[0].src2_ready = 0;
        //     apply_inputs(3'b001, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);
        // end
        // begin
        //     RS_ENTRY [`N-1:0] a_e;  // Local for wakeup block (unused but for consistency)
        //     CDB_PACKET cdb;
        //     cdb.tags = '{6'h18, 6'h19, 0};
        //     // Wakeup and clear same cycle
        //     apply_inputs(3'b000, a_e, cdb, 3'b001, '{4'd0,0,0}, 0);
        // end
        // check_entries(all_empty());  // Cleared after wakeup, but since cleared, valid=0

        $display("");
        if (failed)
            $display("@@@ Failed\n");
        else
            $display("@@@ Passed\n");

        $finish;
    end

endmodule
