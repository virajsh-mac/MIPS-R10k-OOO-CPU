`include "sys_defs.svh"

// -----------------------------------------------------------------------------
// Basic testbench for Store Queue module
// -----------------------------------------------------------------------------
module testbench;

    logic clock, reset;
    logic failed;

    STOREQ_ENTRY [`N-1:0] sq_dispatch_packet;
    logic [$clog2(`LSQ_SZ+1)-1:0] free_slots;
    STOREQ_IDX [`N-1:0] sq_alloc_idxs;

    logic mispredict;
    logic [$clog2(`N+1)-1:0] free_count;

    // Instantiate DUT
    store_queue dut (
        .clock(clock),
        .reset(reset),

        .sq_dispatch_packet(sq_dispatch_packet),
        .free_slots(free_slots),
        .sq_alloc_idxs(sq_alloc_idxs),

        .mispredict(mispredict),
        .free_count(free_count)
    );

    // Clock generation
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    // -------------------------------------------------------------------------
    // Helper constructors
    // -------------------------------------------------------------------------
    function STOREQ_ENTRY make_entry(input int idx);
        make_entry = '0;
        make_entry.valid = 1;
        make_entry.address  = 32'h1000 + idx;
        make_entry.data     = 32'hABCD0000 + idx;
        make_entry.rob_idx  = ROB_IDX'(idx); 
    endfunction

    function STOREQ_ENTRY empty_entry();
        empty_entry = '0;
        empty_entry.valid = 0;
    endfunction

    // -------------------------------------------------------------------------
    // Print Store Queue state
    // -------------------------------------------------------------------------
    // task print_sq_state(input string label);
    //     $display("\n=== %s ===", label);
    //     $display("Free slots: %0d", free_slots);
    //     for (int i = 0; i < `LSQ_SZ; i++) begin
    //         $display("SQ[%0d]: valid=%b addr=%h data=%h",
    //                  i,
    //                  dut.sq_entries_next[i].valid,
    //                  dut.sq_entries_next[i].address,
    //                  dut.sq_entries_next[i].data);
    //     end
    //     $display("");
    // endtask

    // -------------------------------------------------------------------------
    // Reset task
    // -------------------------------------------------------------------------
    task reset_dut;
        reset = 0;
        @(negedge clock);
        reset = 1;
        @(negedge clock);
        reset = 0;
        @(posedge clock);  // allow sequential update after reset
    endtask

    int test_num;
    int cycles;
    int leftover;
    int free_before;

    initial begin
        test_num = 1;
        failed = 0;
        clock  = 0;
        reset  = 0;

        mispredict = 0;
        free_count = 0;

        sq_dispatch_packet = '{default:empty_entry()};

        // ----------------------------------------------------
        // Test 1: Reset clears store queue
        // ----------------------------------------------------
        $display("\nTest %0d: Reset initializes state", test_num++);
        reset_dut();
        //print_sq_state("After reset");

        if (free_slots == `LSQ_SZ)
            $display("  PASS: All slots free");
        else begin
            $display("  FAIL: Free slots=%0d expected=%0d",
                     free_slots, `LSQ_SZ);
            failed = 1;
        end

        // ----------------------------------------------------
        // Test 2: Dispatch one store entry
        // ----------------------------------------------------
        $display("\nTest %0d: Dispatch one store", test_num++);
        reset_dut();

        sq_dispatch_packet = '{default:empty_entry()};
        sq_dispatch_packet[0] = make_entry(0);

        @(posedge clock);  // wait for sequential update
        sq_dispatch_packet = '{default:empty_entry()};

        //print_sq_state("After dispatching 1 store");
        if (free_slots == `LSQ_SZ - 1)
            $display("  PASS: Free slots decreased");
        else begin
            $display("  FAIL: free_slots=%0d expected=%0d",
                    free_slots, `LSQ_SZ - 1);
            failed = 1;
        end

        // ----------------------------------------------------
        // Test 3: Dispatch N stores (superscalar)
        // ----------------------------------------------------
        $display("\nTest %0d: Dispatch N stores", test_num++);
        reset_dut();

        sq_dispatch_packet = '{default:empty_entry()};
        for (int i = 0; i < `N; i++)
            sq_dispatch_packet[i] = make_entry(i);

        @(posedge clock);  // wait for sequential update
        sq_dispatch_packet = '{default:empty_entry()};

        //print_sq_state("After dispatching N stores");

        if (free_slots == (`LSQ_SZ - `N))
            $display("  PASS: N entries dispatched");
        else begin
            $display("  FAIL: free_slots=%0d expected=%0d",
                     free_slots, (`LSQ_SZ - `N));
            failed = 1;
        end

        // ----------------------------------------------------
        // Test 4: Retire frees entries (free_count)
        // ----------------------------------------------------
        $display("\nTest %0d: Retire/free_count", test_num++);
        free_count = 2;   // free two entries
        @(posedge clock); // wait for sequential update
        free_count = 0;

        //print_sq_state("After retiring 2 entries");

        if (free_slots == (`LSQ_SZ - `N + 2))
            $display("  PASS: 2 entries retired correctly");
        else begin
            $display("  FAIL: free_slots=%0d expected=%0d",
                     free_slots, (`LSQ_SZ - `N + 2));
            failed = 1;
        end

        // ----------------------------------------------------
        // Test 5: Mispredict flush clears entire queue
        // ----------------------------------------------------
        $display("\nTest %0d: Mispredict flush", test_num++);
        @(negedge clock);
        mispredict = 1;
        @(negedge clock); // wait for sequential update
        mispredict = 0;

        //print_sq_state("After mispredict flush");
        if (free_slots == `LSQ_SZ)
            $display("  PASS: Store queue fully cleared");
        else begin
            $display("  FAIL: free_slots=%0d expected=%0d",
                     free_slots, `LSQ_SZ);
            failed = 1;
        end

        // ----------------------------------------------------
        // Test 6: Fill SQ completely, verify free_slots == 0
        // ----------------------------------------------------
        $display("\nTest %0d: Fill store queue completely", test_num++);
        reset_dut();

        cycles = `LSQ_SZ / `N;
        leftover = `LSQ_SZ % `N;

        for (int c = 0; c < cycles; c++) begin
            for (int i = 0; i < `N; i++)
                sq_dispatch_packet[i] = make_entry(c * `N + i);
            @(posedge clock);
            sq_dispatch_packet = '{default:empty_entry()};
        end

        if (leftover > 0) begin
            for (int i = 0; i < leftover; i++)
                sq_dispatch_packet[i] = make_entry(cycles * `N + i);
            @(posedge clock);
            sq_dispatch_packet = '{default:empty_entry()};
        end

        //print_sq_state("After filling entire store queue");

        if (free_slots == 0)
            $display("  PASS: Store queue filled completely (free_slots = 0)");
        else begin
            $display("  FAIL: free_slots=%0d expected=0", free_slots);
            failed = 1;
        end

        // ----------------------------------------------------
        // Test 7: Retire 2 and dispatch 2 in same cycle (from full SQ)
        // ----------------------------------------------------
        $display("\nTest %0d: Retire 2 and dispatch 2 in same cycle (full SQ)", test_num++);
        free_count = 2;
        sq_dispatch_packet[0] = make_entry(200);
        sq_dispatch_packet[1] = make_entry(201);

        @(posedge clock); // wait for sequential update

        // Clear signals
        free_count = 0;
        sq_dispatch_packet = '{default:empty_entry()};

        //print_sq_state("After retiring 2 and dispatching 2 from full SQ");

        if (free_slots == 0)
            $display("  PASS: free_slots correctly updated (%0d)", free_slots);
        else begin
            $display("  FAIL: free_slots=%0d expected=0", free_slots);
            failed = 1;
        end

        // ----------------------------------------------------
        // Final
        // ----------------------------------------------------
        if (!failed)
            $display("\nALL SQ TESTS PASSED\n");
        else
            $display("\nSOME SQ TESTS FAILED\n");

        $finish;
    end

endmodule