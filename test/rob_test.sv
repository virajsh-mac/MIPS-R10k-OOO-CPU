`include "sys_defs.svh"

module rob_test;

  // -------------------------------------------------------------
  // DUT signals
  // -------------------------------------------------------------
  logic clock, reset;
  logic [`N-1:0] alloc_valid;
  ROB_ENTRY [`N-1:0] rob_entry_packet;
  ROB_IDX [`N-1:0] alloc_idxs;
  logic [$clog2(`ROB_SZ+1)-1:0] free_slots;
  ROB_UPDATE_PACKET rob_update_packet;
  ROB_ENTRY [`N-1:0] head_entries;
  logic [`N-1:0] head_valids;
  logic mispredict;
  ROB_IDX mispred_idx;

  // -------------------------------------------------------------
  // DUT instantiation
  // -------------------------------------------------------------
  rob dut (
      .clock(clock),
      .reset(reset),
      .alloc_valid(alloc_valid),
      .rob_entry_packet(rob_entry_packet),
      .alloc_idxs(alloc_idxs),
      .free_slots(free_slots),
      .rob_update_packet(rob_update_packet),
      .head_entries(head_entries),
      .head_valids(head_valids),
      .mispredict(mispredict),
      .mispred_idx(mispred_idx)
  );

  // -------------------------------------------------------------
  // Clock generation
  // -------------------------------------------------------------
  always #5 clock = ~clock;

  // -------------------------------------------------------------
  // Testbench state and helper variables
  // -------------------------------------------------------------
  bit                            failed = 0;
  ADDR                           pc_val = 32'h1000;
  REG_IDX                        arch_val = 5'd1;
  PHYS_TAG                       phys_val = 6'd10;
  DATA                           data_val = 32'd1000;

  logic    [$clog2(`ROB_SZ)-1:0] start_idx;
  logic    [$clog2(`ROB_SZ)-1:0] i;
  logic    [$clog2(`ROB_SZ)-1:0] rob_idx;

  // -------------------------------------------------------------
  // Helper functions
  // -------------------------------------------------------------
  function automatic ROB_ENTRY make_rob_entry(
      input ADDR pc, input INST inst, input REG_IDX arch_rd, input PHYS_TAG phys_rd,
      input PHYS_TAG prev_phys_rd, input DATA value, input logic branch = 0,
      input ADDR branch_target = '0, input logic branch_taken = 0, input ADDR pred_target = '0,
      input logic pred_taken = 0, input logic halt = 0, input logic illegal = 0);
    ROB_ENTRY entry;
    entry.valid         = 1'b1;
    entry.PC            = pc;
    entry.inst          = inst;
    entry.arch_rd       = arch_rd;
    entry.phys_rd       = phys_rd;
    entry.prev_phys_rd  = prev_phys_rd;
    entry.value         = value;
    entry.complete      = 1'b0;
    entry.exception     = NO_ERROR;
    entry.branch        = branch;
    entry.branch_target = branch_target;
    entry.branch_taken  = branch_taken;
    entry.pred_target   = pred_target;
    entry.pred_taken    = pred_taken;
    entry.halt          = halt;
    entry.illegal       = illegal;
    return entry;
  endfunction

  function automatic void fill_rob_packet(inout ROB_ENTRY [`N-1:0] packet, input ADDR base_pc,
                                          input REG_IDX base_arch, input PHYS_TAG base_phys,
                                          input DATA base_value);
    for (int i = 0; i < `N; i++) begin
      packet[i] = make_rob_entry(
          base_pc + i,  // PC
          `NOP,  // Instruction
          base_arch + i,  // arch_rd
          base_phys + i,  // phys_rd
          base_phys + i - 1,  // prev_phys_rd
          base_value + i  // value
      );
    end
  endfunction

  // -------------------------------------------------------------
  // Main test sequence
  // -------------------------------------------------------------
  initial begin
    // -------------------------------
    
    automatic ROB_IDX expected_tail;
    automatic int k;
    automatic int retire_cnt;
    automatic ROB_IDX idx;
    automatic int actual_allocs;
    automatic int ran_num_alloc;
    automatic int ran_num_retire;
    automatic int alloc_counter;
    automatic int complete_counter;
    logic do_alloc, do_complete;

    
    // Initialization & Reset
    // -------------------------------
    clock = 0;
    reset = 1;
    alloc_valid = 0;
    rob_update_packet = '{default: 0};
    mispredict = 0;
    mispred_idx = '0;

    @(negedge clock);
    @(negedge clock);
    reset = 0;
    @(posedge clock);  // allow one cycle after reset

    // use below monitior statement for debugging

    $monitor("Time %0t | head=%0d tail=%0d free_slots=%0d valid=%0d",
              $time, dut.head, dut.tail, dut.free_slots, dut.rob_array[0].valid);

    // -------------------------------
    // Test 1: Check Empty ROB after Reset
    // -------------------------------
    $display("\nTest 1: Checking if ROB is empty after reset...\n");
    if (dut.head !== dut.tail) begin
      $display("FAIL: head (%0d) != tail (%0d)", dut.head, dut.tail);
      failed = 1;
    end

    for (int i = 0; i < `ROB_SZ; i++) begin
      if (dut.rob_array[i].valid !== 0) begin
        $display("FAIL: rob_array[%0d].valid = %b (expected 0)", i, dut.rob_array[i].valid);
        failed = 1;
      end
    end

    if (!failed) $display("PASS: ROB is empty and head == tail after reset.\n");
    else $display("FAIL: ROB initial empty-state check.\n");

    // -------------------------------
    // Test 2: Fill ROB
    // -------------------------------
    $display("Test 2: Filling the ROB and checking if its full\n");
    alloc_valid = '1;
    for (int i = 0; i < (`ROB_SZ / `N); i++) begin
      fill_rob_packet(rob_entry_packet, pc_val + i * `N, arch_val + i * `N, phys_val + i * `N,
                      data_val + i * `N);
      @(posedge clock);
    end

    alloc_valid = '0;
    for (int i = 0; i < 2; i++) begin
      rob_entry_packet[i] = make_rob_entry(pc_val + i, `NOP, arch_val + i, phys_val + i,
                                           phys_val + i - 1, data_val + i);
      alloc_valid[i] = 1'b1;
    end
    @(posedge clock);
    alloc_valid = '0;
    @(posedge clock);

    if (free_slots !== 0) begin
      $display("FAIL: Expected free_slots = 0, got %0d", free_slots);
      failed = 1;
    end else $display("PASS: ROB full condition detected.\n");

    // -------------------------------
    // Test 3: Complete and Retire All Instructions
    // -------------------------------
    $display("Test 3: Completing and retiring all instructions...\n");
    for (start_idx = 0; start_idx < `ROB_SZ - 3; start_idx += `N) begin
      for (i = 0; i < `N; i++) begin
        rob_idx                             = (start_idx + i) % `ROB_SZ;
        rob_update_packet.valid[i]          = 1'b1;
        rob_update_packet.idx[i]            = rob_idx;
        rob_update_packet.values[i]         = 0;
        rob_update_packet.branch_taken[i]   = 1'b0;
        rob_update_packet.branch_targets[i] = '0;
      end
      @(posedge clock);
      rob_update_packet.valid = '0;
      @(posedge clock);
      @(posedge clock);
    end

    // Final batch
    for (i = 0; i < `N; i++) begin
      rob_idx                             = (start_idx + i) % `ROB_SZ;
      rob_update_packet.valid[i]          = 1'b1;
      rob_update_packet.idx[i]            = rob_idx;
      rob_update_packet.values[i]         = 0;
      rob_update_packet.branch_taken[i]   = 1'b0;
      rob_update_packet.branch_targets[i] = '0;
    end
    rob_update_packet.valid[2] = 1'b0;
    @(posedge clock);
    rob_update_packet.valid = '0;
    @(posedge clock);
    @(posedge clock);

    if (free_slots !== `ROB_SZ) begin
      $display("FAIL: ROB did not retire all entries, free_slots = %0d", free_slots);
      failed = 1;
    end else $display("PASS: All instructions completed and retired correctly.\n");

    // -------------------------------
    // Test 4: Simultaneous Retirement & Dispatch
    // -------------------------------
    $display("Test 4: Testing simultaneous retirement and dispatch...\n");
    alloc_valid = '1;
    for (int i = 0; i < (`ROB_SZ / `N); i++) begin
      fill_rob_packet(rob_entry_packet, pc_val + i * `N, arch_val + i * `N, phys_val + i * `N,
                      data_val + i * `N);
      @(posedge clock);
    end
    alloc_valid = '0;
    for (int i = 0; i < 2; i++) begin
      rob_entry_packet[i] = make_rob_entry(pc_val + i, `NOP, arch_val + i, phys_val + i,
                                           phys_val + i - 1, data_val + i);
      alloc_valid[i] = 1'b1;
    end
    @(posedge clock);
    alloc_valid = '0;
    @(posedge clock);

    // Complete first N instructions
    rob_update_packet.valid = '0;
    for (i = 0; i < `N; i++) begin
      rob_update_packet.valid[i]  = 1'b1;
      rob_update_packet.idx[i]    = i;
      rob_update_packet.values[i] = 0;
      rob_update_packet.branch_taken[i] = 1'b0;
      rob_update_packet.branch_targets[i] = '0;
    end
    @(posedge clock);

    // Dispatch new instructions while retiring
    rob_update_packet.valid = '0;
    for (i = 0; i < `N; i++) begin
      rob_entry_packet[i] = make_rob_entry(
          pc_val + `ROB_SZ + i,
          `NOP,
          arch_val + `ROB_SZ + i,
          phys_val + `ROB_SZ + i,
          phys_val + `ROB_SZ + i - 1,
          data_val + `ROB_SZ + i
      );
      alloc_valid[i] = 1'b1;
    end
    @(posedge clock);
    alloc_valid = '0;
    @(posedge clock);
    @(posedge clock);

    if (free_slots !== 0) begin
      $display("FAIL: ROB free_slots incorrect after retire+dispatch, got %0d", free_slots);
      failed = 1;
    end else $display("PASS: Simultaneous retirement and dispatch successful.\n");

    // -------------------------------
    // Test 5: OOO Complete and In Order Retire
    // -------------------------------

    $display("\nTest 5: Out of Order Complete and In Order Retire...\n");
    reset = 1;
    @(negedge clock);
    @(negedge clock);
    reset = 0;
    @(posedge clock);

    // 1. Fill the ROB completely
    alloc_valid = '1;
    for (int i = 0; i < (`ROB_SZ / `N); i++) begin
      fill_rob_packet(rob_entry_packet, pc_val + i * `N, arch_val + i * `N, phys_val + i * `N, data_val + i * `N);
      @(posedge clock);
    end
    if (`ROB_SZ % `N != 0) begin
        alloc_valid = '0; for (int i = 0; i < (`ROB_SZ % `N); i++) alloc_valid[i] = 1'b1;
        @(posedge clock);
    end
    alloc_valid = '0;
    @(posedge clock);

    // 2. Complete several instructions out of order (but not the first few)
    $display("Completing entries at indices 5, 2, 8 out of order...");
    rob_update_packet.valid = '0;
    rob_update_packet.valid[0] = 1'b1; rob_update_packet.idx[0] = 5;
    rob_update_packet.valid[1] = 1'b1; rob_update_packet.idx[1] = 2;
    rob_update_packet.valid[2] = 1'b1; rob_update_packet.idx[2] = 4;
    @(posedge clock);
    rob_update_packet.valid = '0;
    
    // 3. Verify that the head has NOT moved, because instruction 0 is not complete
    repeat(3) @(posedge clock);
    if (dut.head !== 0) begin
      $display("FAIL: Head advanced on out-of-order complete. Head is %0d, should be 0.", dut.head);
      failed = 1;
    end else begin
      $display("PASS: Head correctly stalled while waiting for in-order instruction.\n");
    end

    // 4. Now, complete the first block of instructions to fill the gap
    $display("Completing first block of instructions (0, 1, 3, 4) to un-stall retirement...");
    rob_update_packet.valid = '0;
    rob_update_packet.valid[0] = 1'b1; rob_update_packet.idx[0] = 0;
    rob_update_packet.valid[1] = 1'b1; rob_update_packet.idx[1] = 1;
    rob_update_packet.valid[2] = 1'b1; rob_update_packet.idx[2] = 3;
    @(posedge clock);
    rob_update_packet.valid = '0;

    // 5. Verify that the head has advanced past the entire contiguous completed block
    repeat(3) @(posedge clock);
    // Instructions 0, 1, 2, 3, 4, 5 are all now complete. Head should be at 6.
    if (dut.head !== 6) begin
      $display("FAIL: Head did not correctly batch-retire. Expected 6, got %0d", dut.head);
      failed = 1;
    end else begin
      $display("PASS: Instructions correctly retired in-order after out-of-order completion.\n");
    end

    // -------------------------------
    // Test 6: Partial Completions
    // -------------------------------

    $display("Test 6: Partial completions");
    reset=1; @(negedge clock); @(negedge clock); reset=0; @(posedge clock);

    // allocate 2*N entries
    for (int i = 0; i < 2; i++) begin
      alloc_valid='1; 
      fill_rob_packet(rob_entry_packet, pc_val+i*`N, arch_val+i*`N, phys_val+i*`N, data_val+i*`N);
      @(posedge clock);
    end
    alloc_valid='0; @(posedge clock);

    // set first k complete, next one incomplete
    k = (`N >= 3) ? (`N-1) : 1;
    rob_update_packet.valid='0;
    for (int i = 0; i < k; i++) begin
      rob_update_packet.valid[i] = 1'b1; 
      rob_update_packet.idx[i] = (dut.head + i) % `ROB_SZ;
    end
    @(posedge clock); 
    rob_update_packet.valid='0; 
    repeat(2) @(posedge clock);

    // Expect head advanced by k, and exactly those k entries invalidated
    if (dut.head != k % `ROB_SZ) begin 
      $display("FAIL: Head advanced by %0d, expected %0d", dut.head, k%`ROB_SZ); 
      failed=1; 
    end
    
    for (int i = 0; i < k; i++) begin
        if (dut.rob_array[i].valid) begin 
            $display("FAIL: Retired entry %0d still valid", i); 
            failed=1; 
        end
    end

    if (!dut.rob_array[k].valid) begin 
      $display("FAIL: Entry %0d (next after k) should still be valid", k); 
      failed=1; 
    end
    if (!failed) $display("PASS: Partial block of %0d instructions retired correctly.\n", k);


    // -------------------------------
    // Test 7: Pointer Wrap-Around and Boundary Conditions
    // -------------------------------
    $display("\nTest 7: Pointer Wrap-Around and Boundary Conditions...\n");
    // Reset and fill all but the last N slots
    reset = 1; @(negedge clock); @(negedge clock); reset = 0; @(posedge clock);
    alloc_valid = '1;
    for (int i = 0; i < (`ROB_SZ / `N); i++) begin 
      fill_rob_packet(rob_entry_packet, i*`N, i*`N, i*`N, i*`N); 
      @(posedge clock); 
    end
    alloc_valid = '0; @(posedge clock);

    // Complete and retire the first N*2 instructions to move the head up
    for (int i = 0; i < (`N * 2); i++) begin
      rob_update_packet.valid[i % `N] = 1'b1;
      rob_update_packet.idx[i % `N]   = i;
      if ((i % `N) == (`N - 1) || i == (`N*2 - 1) ) begin 
        @(posedge clock); rob_update_packet.valid = '0; 
      end
    end
    repeat (3) 
    @(posedge clock); 
    // Wait for retirement

    // At this point, tail is at `ROB_SZ - N`.
    $display("Allocating instructions to wrap tail pointer...");
    alloc_valid = '1;
    fill_rob_packet(rob_entry_packet, pc_val, arch_val, phys_val, data_val);
    @(posedge clock); // Allocates N instructions, tail becomes (`ROB_SZ-N+N)%ROB_SZ = 0
    fill_rob_packet(rob_entry_packet, pc_val, arch_val, phys_val, data_val);
    @(posedge clock); // Allocates N more, tail becomes (0+N)%ROB_SZ = N
    alloc_valid = '0;
    @(posedge clock);

    expected_tail = 4;
    if (dut.tail !== expected_tail) begin
      $display("FAIL: Tail did not wrap correctly. Expected %0d, got %0d", expected_tail, dut.tail);
      failed = 1;
    end else $display("PASS: Tail pointer wrapped around correctly.\n");

    // -------------------------------
    // Test 8: Partial Allocation and Retirement
    // -------------------------------
    $display("\nTest 8: Partial Allocation and Retirement...\n");
    // -- 8.1: Partial Allocation
    reset = 1; @(negedge clock); @(negedge clock); reset = 0; @(posedge clock); // Reset ROB
    alloc_valid = '0;
    alloc_valid[0] = 1'b1;
    alloc_valid[2] = 1'b1;
    fill_rob_packet(rob_entry_packet, pc_val, arch_val, phys_val, data_val);
    @(posedge clock);
    alloc_valid = '0;
    @(posedge clock);

    if (dut.tail !== 2) begin
      $display("FAIL: Tail advanced incorrectly on partial alloc. Expected 2, got %0d", dut.tail);
      failed = 1;
    end else if (!dut.rob_array[0].valid || dut.rob_array[1].valid) begin
      $display("FAIL: ROB entries written incorrectly on partial allocation.");
      failed = 1;
    end else $display("PASS: Partial allocation handled correctly.");

    // -- 8.2: Partial Retirement
    reset = 1; @(negedge clock); @(negedge clock); reset = 0; @(posedge clock);
    alloc_valid = '1; // Refill ROB
    for (int i = 0; i < (`ROB_SZ / `N); i++) begin fill_rob_packet(rob_entry_packet, i, i, i, i); @(posedge clock); end
    if (`ROB_SZ % `N != 0) begin alloc_valid = '0; for (int i = 0; i < (`ROB_SZ % `N); i++) alloc_valid[i] = 1'b1; @(posedge clock); end
    alloc_valid = '0; @(posedge clock);

    rob_update_packet.valid[0] = 1'b1;
    rob_update_packet.idx[0] = dut.head; // Complete only the instruction at the head
    @(posedge clock);
    rob_update_packet.valid = '0;
    repeat(2) @(posedge clock);

    if (dut.head !== 1) begin
      $display("FAIL: Head did not advance by 1 on single retirement. Expected 1, got %0d", dut.head);
      failed = 1;
    end else $display("PASS: Partial retirement of one instruction successful.\n");


    // -------------------------------
    // Test 9: Retire 3 Instructions and Check head_entries
    // -------------------------------
    $display("Test 9: Retiring 3 instructions and checking head_entries...\n");

    // Reset DUT again for a clean state
    reset = 1;
    @(negedge clock);   // change reset between clock edges
    reset = 0;
    @(posedge clock);   // allow one cycle for reset to propagate

    // Step 1: Allocate 3 instructions
    @(posedge clock);   // set inputs before next posedge
    alloc_valid = '0;
    for (int i = 0; i < 3; i++) begin
      rob_entry_packet[i] = make_rob_entry(
        pc_val + i,
        `NOP,
        arch_val + i,
        phys_val + i,
        phys_val + i - 1,
        data_val + i
      );
      alloc_valid[i] = 1'b1;
    end
    @(posedge clock);   // DUT samples alloc_valid + packet
    alloc_valid = '0;
    @(posedge clock);
    //alloc_valid = '0;   // clear after DUT sampled

    // Step 2: Mark all 3 instructions as complete
    @(negedge clock);   // set updates before next posedge
    rob_update_packet = '{default:0};
    for (int i = 0; i < 3; i++) begin
      
      rob_update_packet.valid[i]  = 1'b1;
      rob_update_packet.idx[i]    = i;
      rob_update_packet.values[i] = data_val + i;
    end
    @(posedge clock);   // DUT sees completions
    @(negedge clock);
    rob_update_packet.valid = '0;

    // Step 3: Wait one cycle for completion logic to execute
    repeat (1) @(posedge clock);

    // Step 4: Check head_entries (after DUT update)
    for (int i = 0; i < 3; i++) begin
      if (head_valids[i] !== 1'b1) begin
        $display("FAIL: head_valids[%0d] = %b (expected 1)", i, head_valids[i]);
        failed = 1;
      end else if (head_entries[i].PC !== pc_val + i) begin
        $display("FAIL: head_entries[%0d].PC = 0x%0h (expected 0x%0h)",
                  i, head_entries[i].PC, pc_val + i);
        failed = 1;
      end else begin
        $display("PASS: head_entries[%0d] retired instruction PC=0x%0h",
                  i, head_entries[i].PC);
      end
    end

    if (!failed)
      $display("\033[1;32mPASS: All 3 retired instructions match expected head_entries.\033[0m\n");
    else
      $display("\033[1;31mFAIL: Retired instructions do not match expected head_entries.\033[0m\n");
    
begin
      // -------------------------------
      // Test #10 - Simplified Randomized Stress Test
      // -------------------------------
      $display("\nTest 10: Simplified Randomized Stress Test...\n");
      begin
        localparam int STRESS_CYCLES = 20;
        // Use a fixed-size array and counter to track allocated instructions
        ROB_IDX allocated_indices_arr[`ROB_SZ];
        int num_allocated_not_completed = 0;
        int ran_num_alloc, ran_num_complete;
        ROB_IDX temp_idx;

        // --- Phase 1: Stress the ROB with random allocations and completions ---
        for (int i = 0; i < STRESS_CYCLES; i++) begin
          // Default to no operations in this cycle
          alloc_valid = '0;
          rob_update_packet.valid = '0;


          // Randomly decide whether to allocate, complete, both, or neither in this cycle
          // Ensure the action is possible (e.g., free slots exist, or instructions are available to complete)
          do_alloc = ($urandom_range(0, 1) == 1) && (dut.free_slots > 0);
          do_complete = ($urandom_range(0, 1) == 1) && (num_allocated_not_completed >= 0);

          // Prepare allocation packet if we are allocating this cycle
          if (do_alloc) begin
            ran_num_alloc = $urandom_range(1, `N);
            if (ran_num_alloc > dut.free_slots) ran_num_alloc = dut.free_slots;

            for (int j = 0; j < ran_num_alloc; j++) alloc_valid[j] = 1'b1;
            fill_rob_packet(rob_entry_packet, pc_val, arch_val, phys_val, data_val);
          end

          $display("do_complete: %0d", do_complete);

          // Prepare completion packet if we are completing this cycle
          if (do_complete) begin
            ran_num_complete = $urandom_range(1, `N);
            if (ran_num_complete > num_allocated_not_completed) ran_num_complete = num_allocated_not_completed;

            for (int j = 0; j < ran_num_complete; j++) begin
              // Pick a random instruction from the array to complete (simulates out-of-order completion)
              int arr_idx = $urandom_range(0, num_allocated_not_completed - 1);
              temp_idx = allocated_indices_arr[arr_idx];

              // Remove from array by swapping with the last element and decrementing the count
              allocated_indices_arr[arr_idx] = allocated_indices_arr[num_allocated_not_completed - 1];
              num_allocated_not_completed--;

              rob_update_packet.valid[j] = 1'b1;
              rob_update_packet.idx[j] = temp_idx;
            end
          end

          // Advance simulation by one clock cycle
          @(posedge clock);

          // After the clock edge, capture the results of any allocation
          if (do_alloc) begin
            for (int j = 0; j < ran_num_alloc; j++) begin
              allocated_indices_arr[num_allocated_not_completed] = dut.alloc_idxs[j];
              num_allocated_not_completed++;
            end
            // Update base values to ensure unique instructions for the next allocation
            pc_val += ran_num_alloc;
            arch_val += ran_num_alloc;
            phys_val += ran_num_alloc;
          end
        end

        // --- Phase 2: Wind-down. Stop allocating and complete all remaining instructions ---
        $display("Stress phase finished. Completing %0d remaining instructions...", num_allocated_not_completed);
        alloc_valid = '0; // Stop new allocations


        while (num_allocated_not_completed > 0) begin
          ran_num_complete = $urandom_range(1, `N);
          if (ran_num_complete > num_allocated_not_completed) ran_num_complete = num_allocated_not_completed;

          rob_update_packet.valid = '0;
          for (int j = 0; j < ran_num_complete; j++) begin
            // Complete from the end of the array
            num_allocated_not_completed--;
            temp_idx = allocated_indices_arr[num_allocated_not_completed];
            rob_update_packet.valid[j] = 1'b1;
            rob_update_packet.idx[j] = temp_idx;
          end
          @(posedge clock);
        end
        rob_update_packet.valid = '0;

        // --- Phase 3: Verification. Wait for the ROB to become empty through retirement ---
        $display("Waiting for ROB to drain...");
        // Wait for retirement to clear the ROB. Max wait time is ROB size / N instructions per cycle + buffer.
        repeat (`ROB_SZ / `N + 20) @(posedge clock);

        $display("Final check...");
        if ((dut.head + `ROB_SZ - dut.tail) == dut.free_slots) begin
          $display("PASS: ROB is empty after stress test.\n");
        end else begin
          $display("FAIL: ROB is not empty. free_slots=%0d, head=%0d, tail=%0d", dut.free_slots, dut.head, dut.tail);
          failed = 1;
        end
      end
    end

    if (!failed)
      $display("\033[1;32mAll tests passed.\033[0m\n");
    else
      $display("\033[1;31mOne or more tests failed.\033[0m\n");



    

    // -------------------------------
    // Test Summary
    // -------------------------------
    if (failed) begin
      $display("\033[1;31m@@@ Failed\033[0m\n");
    end else begin
      $display("\033[1;32m@@@ Passed\033[0m\n");
    end


    $finish;
  end

endmodule

/// Test cases:
// 
// 

// input to check if it's ready to issue
// all the inputs and output signals

// stress test
// one test case




