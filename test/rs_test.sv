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
    logic [`CDB_SZ-1:0] valid;
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
    // logic [`N-1:0][`RS_SZ-1:0] available_entries;  // Removed debug wire

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
        // .available_entries_out(available_entries)  // Removed debug port
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
        // $display("EXPECTED RS:");
        // for (int i = 0; i < `RS_SZ; i++) begin
        //     $display("  Entry %2d: valid=%b, src1_tag=%h, src1_ready=%b, src2_tag=%h, src2_ready=%b",
        //              i, expected[i].valid, expected[i].src1_tag, expected[i].src1_ready, expected[i].src2_tag, expected[i].src2_ready);
        // end
        $display("ACTUAL RS:");
        for (int i = 0; i < `RS_SZ; i++) begin
            $display("  Entry %2d: valid=%b, src1_tag=%h, src1_ready=%b, src2_tag=%h, src2_ready=%b",
                     i, entries[i].valid, entries[i].src1_tag, entries[i].src1_ready, entries[i].src2_tag, entries[i].src2_ready);
        end
    endtask

    // Helper task to print the free mask
    task print_free_mask;
        $display("FREE MASK:");
        $write("  ");
        for (int i = 0; i < `RS_SZ; i++) begin
            $write("%b", !entries[i].valid);
        end
        $display("");
        $display("  (1=free, 0=occupied, LSB=index 0, MSB=index %0d)", `RS_SZ-1);
    endtask

    // Helper task to print effective free count from RS
    task print_effective_free;
        $display("EFFECTIVE FREE COUNT: %0d", free_count);
    endtask

    // Helper task to check all entries against expected
    task check_entries(
        input RS_ENTRY [`RS_SZ-1:0] expected
    );
        logic mismatch;
        mismatch = 0;
        for (int i = 0; i < `RS_SZ; i++) begin
            // Only check valid bit for invalid expected entries
            if (!expected[i].valid) begin
                if (entries[i].valid != expected[i].valid) begin
                    $display("Entry %d mismatch: expected valid=%b, got valid=%b",
                             i, expected[i].valid, entries[i].valid);
                    mismatch = 1;
                end
            end else begin
                compare_rs_entry(i, expected[i], entries[i], mismatch);
            end
        end
        if (mismatch) failed = 1;
    endtask

    // Helper function to compare two RS_ENTRY and print specific mismatches
    function void compare_rs_entry(int idx, RS_ENTRY exp, RS_ENTRY act, ref logic mismatch);
        mismatch = 0;
        if (act.valid != exp.valid) begin
            $display("Entry %0d valid: expected %0b, got %0b", idx, exp.valid, act.valid);
            mismatch = 1;
        end
        if (act.opa_select != exp.opa_select) begin
            $display("Entry %0d opa_select: expected %0d, got %0d", idx, exp.opa_select, act.opa_select);
            mismatch = 1;
        end
        if (act.opb_select != exp.opb_select) begin
            $display("Entry %0d opb_select: expected %0d, got %0d", idx, exp.opb_select, act.opb_select);
            mismatch = 1;
        end
        if (act.op_type != exp.op_type) begin
            if (act.op_type.category != exp.op_type.category) begin
                $display("Entry %0d op_type.category: expected %0d, got %0d", idx, exp.op_type.category, act.op_type.category);
            end
            if (act.op_type.func != exp.op_type.func) begin
                $display("Entry %0d op_type.func: expected %0h, got %0h", idx, exp.op_type.func, act.op_type.func);
            end
            mismatch = 1;
        end
        if (act.src1_tag != exp.src1_tag) begin
            $display("Entry %0d src1_tag: expected %0h, got %0h", idx, exp.src1_tag, act.src1_tag);
            mismatch = 1;
        end
        if (act.src1_ready != exp.src1_ready) begin
            $display("Entry %0d src1_ready: expected %0b, got %0b", idx, exp.src1_ready, act.src1_ready);
            mismatch = 1;
        end
        if (act.src1_value != exp.src1_value) begin
            $display("Entry %0d src1_value: expected %0h, got %0h", idx, exp.src1_value, act.src1_value);
            mismatch = 1;
        end
        if (act.src2_tag != exp.src2_tag) begin
            $display("Entry %0d src2_tag: expected %0h, got %0h", idx, exp.src2_tag, act.src2_tag);
            mismatch = 1;
        end
        if (act.src2_ready != exp.src2_ready) begin
            $display("Entry %0d src2_ready: expected %0b, got %0b", idx, exp.src2_ready, act.src2_ready);
            mismatch = 1;
        end
        if (act.src2_value != exp.src2_value) begin
            $display("Entry %0d src2_value: expected %0h, got %0h", idx, exp.src2_value, act.src2_value);
            mismatch = 1;
        end
        if (act.dest_tag != exp.dest_tag) begin
            $display("Entry %0d dest_tag: expected %0h, got %0h", idx, exp.dest_tag, act.dest_tag);
            mismatch = 1;
        end
        if (act.rob_idx != exp.rob_idx) begin
            $display("Entry %0d rob_idx: expected %0d, got %0d", idx, exp.rob_idx, act.rob_idx);
            mismatch = 1;
        end
        if (act.PC != exp.PC) begin
            $display("Entry %0d PC: expected %0h, got %0h", idx, exp.PC, act.PC);
            mismatch = 1;
        end
        if (act.pred_taken != exp.pred_taken) begin
            $display("Entry %0d pred_taken: expected %0b, got %0b", idx, exp.pred_taken, act.pred_taken);
            mismatch = 1;
        end
        if (act.pred_target != exp.pred_target) begin
            $display("Entry %0d pred_target: expected %0h, got %0h", idx, exp.pred_target, act.pred_target);
            mismatch = 1;
        end
    endfunction

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
        // $dumpfile("../rs.vcd");
        // $dumpvars(0, testbench.dut);

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

        $display("\nTest 3: Allocate multiple (3) entries");
        reset = 1; #10; reset = 0;
        begin
            RS_ENTRY [`N-1:0] a_e;
            for (int i = 0; i < `N; i++) begin
                a_e[i] = empty_entry();
                a_e[i].valid = 1;
                a_e[i].src1_tag = 6'h10 + i;
                a_e[i].src1_ready = 0;
                a_e[i].src2_tag = 6'h20 + i;
                a_e[i].src2_ready = 0;
            end
            apply_inputs(3'b111, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);
        end
        begin
            RS_ENTRY [`RS_SZ-1:0] exp = all_empty();
            // Priority selector alternates: MSB (15), LSB (0), next MSB (14)
            exp[`RS_SZ-1] = alloc_entries[0];  // Index 15
            exp[0] = alloc_entries[1];         // Index 0
            exp[`RS_SZ-2] = alloc_entries[2];  // Index 14
            check_entries(exp);
            check_free_count(`N);
        end

        $display("\nTest 4: Allocate when some occupied, check next highest free");
        // Continue from Test 3 state (indices 15,0,14 occupied)
        begin
            RS_ENTRY [`N-1:0] a_e;
            a_e[0] = empty_entry();
            a_e[0].valid = 1;
            a_e[0].src1_tag = 6'h03;
            a_e[0].src1_ready = 0;
            a_e[0].src2_tag = 6'h04;
            a_e[0].src2_ready = 0;
            apply_inputs(3'b001, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);
        end
        begin
            RS_ENTRY [`RS_SZ-1:0] exp = all_empty();
            // Previous from Test 3
            exp[`RS_SZ-1].valid = 1; exp[`RS_SZ-1].src1_tag = 6'h10; exp[`RS_SZ-1].src1_ready = 0; exp[`RS_SZ-1].src2_tag = 6'h20; exp[`RS_SZ-1].src2_ready = 0;
            exp[0].valid = 1; exp[0].src1_tag = 6'h11; exp[0].src1_ready = 0; exp[0].src2_tag = 6'h21; exp[0].src2_ready = 0;
            exp[`RS_SZ-2].valid = 1; exp[`RS_SZ-2].src1_tag = 6'h12; exp[`RS_SZ-2].src1_ready = 0; exp[`RS_SZ-2].src2_tag = 6'h22; exp[`RS_SZ-2].src2_ready = 0;
            // New allocation to next highest free (13)
            exp[`RS_SZ-3].valid = 1; exp[`RS_SZ-3].src1_tag = 6'h03; exp[`RS_SZ-3].src1_ready = 0; exp[`RS_SZ-3].src2_tag = 6'h04; exp[`RS_SZ-3].src2_ready = 0;
            check_entries(exp);
            check_free_count(`N);
        end

        $display("\nTest 5: Fill up the RS");
        reset = 1; #10; reset = 0;
        begin
            RS_ENTRY [`N-1:0] a_e_base;
            for (int i = 0; i < `N; i++) begin
                a_e_base[i] = empty_entry();
                a_e_base[i].valid = 1;
                a_e_base[i].src1_tag = 6'h20 + i;
                a_e_base[i].src1_ready = 0;
                a_e_base[i].src2_tag = 6'h30 + i;
                a_e_base[i].src2_ready = 0;
            end
            // Allocate N per cycle until only partial batch left
            for (int c = 0; c < (`RS_SZ / `N); c++) begin
                apply_inputs({`N{1'b1}}, a_e_base, empty_cdb(), 3'b000, '{0,0,0}, 0);
            end
            check_free_count(`RS_SZ % `N);  // Remaining free before partial allocation
            // Allocate remaining entries
            if (`RS_SZ % `N > 0) begin
                logic [`N-1:0] alloc_valid = {`N{1'b1}} >> (`N - (`RS_SZ % `N));
                apply_inputs(alloc_valid, a_e_base, empty_cdb(), 3'b000, '{0,0,0}, 0);
            end
            check_free_count(0);  // Now fully occupied
        end

        $display("\nTest 6: Simultaneous allocation and clear of 3 entries with reuse when full");
        begin
            RS_ENTRY [`N-1:0] new_alloc_entries;
            CDB_PACKET cdb;
            RS_IDX [`N-1:0] clear_idxs_new;
            cdb = empty_cdb();
            clear_idxs_new = '{4'd15, 4'd0, 4'd14};
            for (int i = 0; i < `N; i++) begin
                new_alloc_entries[i] = empty_entry();
                new_alloc_entries[i].valid = 1'b1;
                new_alloc_entries[i].src1_tag = 6'h20 + i;
                new_alloc_entries[i].src1_ready = 1'b0;
                new_alloc_entries[i].src2_tag = 6'h30 + i;
                new_alloc_entries[i].src2_ready = 1'b0;
            end
            
            apply_inputs({`N{1'b1}}, new_alloc_entries, cdb, {`N{1'b1}}, clear_idxs_new, 1'b0);
            check_free_count(0);
            if (entries[`RS_SZ-1] != new_alloc_entries[0] ||
                entries[0] != new_alloc_entries[1] ||
                entries[`RS_SZ-2] != new_alloc_entries[2]) begin
                $display("ERROR: Positions 15,0,14 do not match new allocations");
                failed = 1'b1;
            end
            for (int i = 0; i < `RS_SZ; i++) begin
                if (i == `RS_SZ-1 || i == 0 || i == `RS_SZ-2) continue;
                if (!entries[i].valid) begin
                    $display("ERROR: Entry %0d invalid after operation", i);
                    failed = 1'b1;
                end
            end
        end

        $display("\nTest 7: Wakeup single operand");
        reset = 1; #10; reset = 0;
        begin
            RS_ENTRY [`N-1:0] a_e;
            CDB_PACKET cdb;
            a_e[0] = empty_entry();
            a_e[0].valid = 1;
            a_e[0].src1_tag = 6'h05;
            a_e[0].src1_ready = 0;
            a_e[0].src2_tag = 6'h06;
            a_e[0].src2_ready = 1;  // src2 already ready
            apply_inputs(3'b001, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);
            // Broadcast src1 tag
            cdb.tags = '{0, 0, 6'h05};
            cdb.valid = 3'b001;
            apply_inputs(3'b000, a_e, cdb, 3'b000, '{0,0,0}, 0);
        end
        begin
            RS_ENTRY [`RS_SZ-1:0] exp = all_empty();
            exp[`RS_SZ-1].valid = 1;
            exp[`RS_SZ-1].src1_tag = 6'h05;
            exp[`RS_SZ-1].src1_ready = 1;  // Woken
            exp[`RS_SZ-1].src2_tag = 6'h06;
            exp[`RS_SZ-1].src2_ready = 1;
            check_entries(exp);
        end

        $display("\nTest 8: Wakeup multiple sources in one cycle");
        reset = 1; #10; reset = 0;
        begin
            RS_ENTRY [`N-1:0] a_e;
            CDB_PACKET cdb;
            a_e[0] = empty_entry();
            a_e[0].valid = 1;
            a_e[0].src1_tag = 6'h07;
            a_e[0].src1_ready = 0;
            a_e[0].src2_tag = 6'h08;
            a_e[0].src2_ready = 0;
            a_e[1] = empty_entry();
            a_e[1].valid = 1;
            a_e[1].src1_tag = 6'h09;
            a_e[1].src1_ready = 0;
            a_e[1].src2_tag = 6'h0A;
            a_e[1].src2_ready = 0;
            apply_inputs(3'b011, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);
            // Broadcast multiple tags
            cdb.tags = '{6'h07, 6'h08, 6'h09};
            cdb.valid = 3'b111;
            apply_inputs(3'b000, a_e, cdb, 3'b000, '{0,0,0}, 0);
        end
        begin
            RS_ENTRY [`RS_SZ-1:0] exp = all_empty();
            exp[`RS_SZ-1].valid = 1;
            exp[`RS_SZ-1].src1_ready = 1;
            exp[`RS_SZ-1].src2_ready = 1;
            exp[`RS_SZ-1].src1_tag = 6'h07;
            exp[`RS_SZ-1].src2_tag = 6'h08;
            exp[0].valid = 1;
            exp[0].src1_ready = 1;
            exp[0].src2_ready = 0;  // 0x0A not broadcast
            exp[0].src1_tag = 6'h09;
            exp[0].src2_tag = 6'h0A;
            check_entries(exp);
        end

        $display("\nTest 9: Wakeup same tag for src1 and src2 in one entry");
        reset = 1; #10; reset = 0;
        begin
            RS_ENTRY [`N-1:0] a_e;
            CDB_PACKET cdb;
            a_e[0] = empty_entry();
            a_e[0].valid = 1;
            a_e[0].src1_tag = 6'h0B;
            a_e[0].src1_ready = 0;
            a_e[0].src2_tag = 6'h0B;  // Same tag
            a_e[0].src2_ready = 0;
            apply_inputs(3'b001, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);
            // Broadcast the tag
            cdb.tags = '{0, 0, 6'h0B};
            cdb.valid = 3'b001;
            apply_inputs(3'b000, a_e, cdb, 3'b000, '{0,0,0}, 0);
        end
        begin
            RS_ENTRY [`RS_SZ-1:0] exp = all_empty();
            exp[`RS_SZ-1].valid = 1;
            exp[`RS_SZ-1].src1_ready = 1;
            exp[`RS_SZ-1].src2_ready = 1;
            exp[`RS_SZ-1].src1_tag = 6'h0B;
            exp[`RS_SZ-1].src2_tag = 6'h0B;
            check_entries(exp);
        end

        // Test 10: Broadcast tag not matching any
        // Continue from Test 8 state
        begin
            CDB_PACKET cdb;
            RS_ENTRY [`RS_SZ-1:0] exp;
            cdb.tags = '{6'h3F, 6'h3E, 6'h3D};  // Valid 6-bit, no match
            cdb.valid = 3'b111;
            apply_inputs(3'b000, '{empty_entry(), empty_entry(), empty_entry()}, cdb, 3'b000, '{0,0,0}, 0);
            // State should unchanged
            exp = all_empty();
            exp[`RS_SZ-1].valid = 1;
            exp[`RS_SZ-1].src1_ready = 1;
            exp[`RS_SZ-1].src2_ready = 1;
            exp[`RS_SZ-1].src1_tag = 6'h0B;
            exp[`RS_SZ-1].src2_tag = 6'h0B;
            check_entries(exp);
        end

        $display("\nTest 11: Clear single entry");
        reset = 1; #10; reset = 0;
        begin
            RS_ENTRY [`N-1:0] a_e;
            a_e[0] = empty_entry();
            a_e[0].valid = 1;
            a_e[0].src1_tag = 6'h0C;
            a_e[0].src1_ready = 1;
            a_e[0].src2_tag = 6'h0D;
            a_e[0].src2_ready = 1;
            apply_inputs(3'b001, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);
            begin
                RS_ENTRY [`RS_SZ-1:0] exp_after_alloc = all_empty();
                exp_after_alloc[`RS_SZ-1] = a_e[0];
            end
            // Clear index 15
            apply_inputs(3'b000, a_e, empty_cdb(), 3'b001, '{0,0,4'd15}, 0);
        end
        check_entries(all_empty());
        check_free_count(3);

        $display("\nTest 12: Clear and allocate one entry in same cycle");
        reset = 1; #10; reset = 0;
        begin
            RS_ENTRY [`N-1:0] a_e;
            a_e[0] = empty_entry();
            a_e[0].valid = 1;
            a_e[0].src1_tag = 6'h0E;
            a_e[0].src1_ready = 0;
            a_e[0].src2_tag = 6'h0F;
            a_e[0].src2_ready = 0;
            apply_inputs(3'b001, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);  // Allocate to 15
        end
        begin
            RS_ENTRY [`N-1:0] a_e_new;
            a_e_new[0] = empty_entry();
            a_e_new[0].valid = 1;
            a_e_new[0].src1_tag = 6'h10;
            a_e_new[0].src1_ready = 0;
            a_e_new[0].src2_tag = 6'h11;
            a_e_new[0].src2_ready = 0;
            // Clear 15 and allocate 1 new (should go to 15)
            apply_inputs(3'b001, a_e_new, empty_cdb(), 'b001, '{0,0,4'd15}, 0);
        end
        begin
            RS_ENTRY [`RS_SZ-1:0] exp = all_empty();
            exp[`RS_SZ-1] = alloc_entries[0];
            check_entries(exp);
            check_free_count(`N);
        end

        $display("\nTest 13: Mispredict flush");
        reset = 1; #10; reset = 0;
        begin
            RS_ENTRY [`N-1:0] a_e;
            for (int i = 0; i < `N; i++) begin
                a_e[i] = empty_entry();
                a_e[i].valid = 1;
            end
            apply_inputs(3'b111, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);  // Allocate 3
            apply_inputs(3'b000, a_e, empty_cdb(), 3'b000, '{0,0,0}, 1);  // Mispredict
        end
        check_entries(all_empty());
        check_free_count(`N);

        $display("\nTest 14: Mispredict with simultaneous alloc (should ignore alloc)");
        reset = 1; #10; reset = 0;
        begin
            RS_ENTRY [`N-1:0] a_e;
            for (int i = 0; i < `N; i++) begin
                a_e[i] = empty_entry();
                a_e[i].valid = 1;
            end
            apply_inputs(3'b111, a_e, empty_cdb(), 3'b000, '{0,0,0}, 1);  // Alloc + mispredict
        end
        check_entries(all_empty());
        check_free_count(`N);

        $display("\nTest 15: Stress test - mixed operations over multiple cycles");
        reset = 1; #10; reset = 0;
        begin
            RS_ENTRY [`N-1:0] a_e;
            // Cycle 1: Alloc 2 entries
            a_e[0] = empty_entry(); a_e[0].valid = 1; a_e[0].src1_tag = 6'h12; a_e[0].src1_ready = 0; a_e[0].src2_tag = 6'h13; a_e[0].src2_ready = 0;
            a_e[1] = empty_entry(); a_e[1].valid = 1; a_e[1].src1_tag = 6'h14; a_e[1].src1_ready = 0; a_e[1].src2_tag = 6'h15; a_e[1].src2_ready = 0;
            apply_inputs(3'b011, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);
        end
        begin
            RS_ENTRY [`N-1:0] a_e;  // Local declaration for cycle 2
            CDB_PACKET cdb;
            // Cycle 2: Wakeup some, alloc 1 more, clear 1
            cdb.tags = '{6'h12, 6'h14, 0};
            cdb.valid = 3'b011;
            a_e[0] = empty_entry(); a_e[0].valid = 1; a_e[0].src1_tag = 6'h16; a_e[0].src1_ready = 0; a_e[0].src2_tag = 6'h17; a_e[0].src2_ready = 0;
            apply_inputs(3'b001, a_e, cdb, 3'b001, '{0,0,4'd15}, 0);  // Clear 15, alloc to 15 (highest free)
        end
        begin
            RS_ENTRY [`RS_SZ-1:0] exp = all_empty();
            exp[0].valid = 1; exp[0].src1_tag = 6'h14; exp[0].src1_ready = 1; exp[0].src2_tag = 6'h15; exp[0].src2_ready = 0;
            exp[15].valid = 1; exp[15].src1_tag = 6'h16; exp[15].src1_ready = 0; exp[15].src2_tag = 6'h17; exp[15].src2_ready = 0;
            check_entries(exp);
        end
        begin
            CDB_PACKET cdb;
            // Cycle 3: More wakeup, clear 2, no alloc
            cdb.tags = '{6'h15, 6'h17, 6'h16};
            cdb.valid = 3'b111;
            apply_inputs(3'b000, '{empty_entry(), empty_entry(), empty_entry()}, cdb, 3'b011, '{0,4'd15,4'd0}, 0);  // Clear 0 and 15
        end
        check_entries(all_empty());

        $display("\nTest 16: Allocate with non-consecutive alloc_valid (stress priority)");
        reset = 1; #10; reset = 0;
        begin
            RS_ENTRY [`N-1:0] a_e;
            a_e[0] = empty_entry(); a_e[0].valid = 1;  // Will be skipped since alloc_valid[0]=0
            a_e[1] = empty_entry(); a_e[1].valid = 1;
            a_e[2] = empty_entry(); a_e[2].valid = 1;
            apply_inputs(3'b110, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);  // valid[2]=1, [1]=1, [0]=0 -> i=1 to 0, i=2 to 14
        end
        begin
            RS_ENTRY [`RS_SZ-1:0] exp = all_empty();
            exp[`RS_SZ-2].valid = 1;  // a_e[2] to next top
            exp[0].valid = 1;  // a_e[1] to bottom
            check_entries(exp);
        end

        // Test 17: Become ready and clear in same cycle (wakeup then clear)
        $display("\nTest 17: Become ready and clear in same cycle (wakeup then clear)");
        reset = 1; #10; reset = 0;
        begin
            RS_ENTRY [`N-1:0] a_e;
            a_e[0] = empty_entry();
            a_e[0].valid = 1;
            a_e[0].src1_tag = 6'h18;
            a_e[0].src1_ready = 0;
            a_e[0].src2_tag = 6'h19;
            a_e[0].src2_ready = 0;
            apply_inputs(3'b001, a_e, empty_cdb(), 3'b000, '{0,0,0}, 0);
        end
        begin
            RS_ENTRY [`N-1:0] a_e;  // Local for wakeup block (unused but for consistency)
            CDB_PACKET cdb;
            cdb.tags = '{6'h18, 6'h19, 0};
            cdb.valid = 3'b011;
            // Wakeup and clear same cycle
            apply_inputs(3'b000, a_e, cdb, 3'b001, '{0,0,4'd15}, 0);
        end
        check_entries(all_empty());  // Cleared after wakeup, but since cleared, valid=0

        $display("");
        if (failed)
            $display("@@@ Failed\n");
        else
            $display("@@@ Passed\n");

        $finish;
    end

endmodule
