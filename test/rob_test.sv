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
    bit failed = 0;
    ADDR pc_val       = 32'h1000;
    REG_IDX arch_val  = 5'd1;
    PHYS_TAG phys_val = 6'd10;
    DATA data_val     = 32'd1000;

    logic [$clog2(`ROB_SZ)-1:0] start_idx;
    logic [$clog2(`ROB_SZ)-1:0] i;
    logic [$clog2(`ROB_SZ)-1:0] rob_idx;

    // -------------------------------------------------------------
    // Helper functions
    // -------------------------------------------------------------
    function automatic ROB_ENTRY make_rob_entry(
        input ADDR pc,
        input INST inst,
        input REG_IDX arch_rd,
        input PHYS_TAG phys_rd,
        input PHYS_TAG prev_phys_rd,
        input DATA value,
        input logic branch = 0,
        input ADDR branch_target = '0,
        input logic branch_taken = 0,
        input ADDR pred_target = '0,
        input logic pred_taken = 0,
        input logic halt = 0,
        input logic illegal = 0
    );
        ROB_ENTRY entry;
        entry.valid        = 1'b1;
        entry.PC           = pc;
        entry.inst         = inst;
        entry.arch_rd      = arch_rd;
        entry.phys_rd      = phys_rd;
        entry.prev_phys_rd = prev_phys_rd;
        entry.value        = value;
        entry.complete     = 1'b0;
        entry.exception    = NO_ERROR;
        entry.branch       = branch;
        entry.branch_target= branch_target;
        entry.branch_taken = branch_taken;
        entry.pred_target  = pred_target;
        entry.pred_taken   = pred_taken;
        entry.halt         = halt;
        entry.illegal      = illegal;
        return entry;
    endfunction

    function automatic void fill_rob_packet(
        inout ROB_ENTRY [`N-1:0] packet,
        input ADDR base_pc,
        input REG_IDX base_arch,
        input PHYS_TAG base_phys,
        input DATA base_value
    );
        for (int i = 0; i < `N; i++) begin
            packet[i] = make_rob_entry(
                base_pc + i,       // PC
                `NOP,              // Instruction
                base_arch + i,     // arch_rd
                base_phys + i,     // phys_rd
                base_phys + i - 1, // prev_phys_rd
                base_value + i     // value
            );
        end
    endfunction

    // -------------------------------------------------------------
    // Main test sequence
    // -------------------------------------------------------------
    initial begin
        // -------------------------------
        // Initialization & Reset
        // -------------------------------
        clock = 0;
        reset = 1;
        alloc_valid = 0;
        rob_update_packet = '{default:0};
        mispredict = 0;
        mispred_idx = '0;

        @(negedge clock);
        @(negedge clock);
        reset = 0;
        @(posedge clock);  // allow one cycle after reset

        // use below monitior statement for debugging

        // $monitor("Time %0t | head=%0d tail=%0d free_slots=%0d valid=%0d",
        //           $time, dut.head, dut.tail, dut.free_slots, dut.rob_array[0].valid);

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
            fill_rob_packet(rob_entry_packet, pc_val + i*`N, arch_val + i*`N, phys_val + i*`N, data_val + i*`N);
            @(posedge clock);
        end

        alloc_valid = '0;
        for (int i = 0; i < 2; i++) begin
            rob_entry_packet[i] = make_rob_entry(pc_val + i, `NOP, arch_val + i, phys_val + i, phys_val + i - 1, data_val + i);
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
                rob_idx = (start_idx + i) % `ROB_SZ;
                rob_update_packet.valid[i]        = 1'b1;
                rob_update_packet.idx[i]          = rob_idx;
                rob_update_packet.values[i]       = 0;
                rob_update_packet.branch_taken[i] = 1'b0;
                rob_update_packet.branch_targets[i] = '0;
            end
            @(posedge clock);
            rob_update_packet.valid = '0;
            @(posedge clock);
            @(posedge clock);
        end

        // Final batch
        for (i = 0; i < `N; i++) begin
            rob_idx = (start_idx + i) % `ROB_SZ;
            rob_update_packet.valid[i]        = 1'b1;
            rob_update_packet.idx[i]          = rob_idx;
            rob_update_packet.values[i]       = 0;
            rob_update_packet.branch_taken[i] = 1'b0;
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
            fill_rob_packet(rob_entry_packet, pc_val + i*`N, arch_val + i*`N, phys_val + i*`N, data_val + i*`N);
            @(posedge clock);
        end
        alloc_valid = '0;
        for (int i = 0; i < 2; i++) begin
            rob_entry_packet[i] = make_rob_entry(pc_val + i, `NOP, arch_val + i, phys_val + i, phys_val + i - 1, data_val + i);
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
            rob_entry_packet[i] = make_rob_entry(pc_val + `ROB_SZ + i, `NOP, arch_val + `ROB_SZ + i, phys_val + `ROB_SZ + i, phys_val + `ROB_SZ + i - 1, data_val + `ROB_SZ + i);
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
        // Test 5: WIP (something OoO??)
        // -------------------------------



        // -------------------------------
        // Test Summary
        // -------------------------------
        if (failed) begin
            $display("\033[1;31m@@@ Failed\033[0m\n");
        end
        else begin
            $display("\033[1;32m@@@ Passed\033[0m\n");
        end


        $finish;
    end

endmodule



//ignore below just used as reference (can delete)

// // Compute the correct mult output similar to project 3
// module correct_mult (
//     input DATA rs1,
//     input DATA rs2,
//     MULT_FUNC   func,

//     output DATA result
// );

//     logic signed [63:0] signed_mul, mixed_mul;
//     logic        [63:0] unsigned_mul;

//     assign signed_mul = signed'(rs1) * signed'(rs2);
//     assign unsigned_mul = rs1 * rs2;
//     // Verilog only does signed multiplication if both arguments are signed :/
//     assign mixed_mul = signed'(rs1) * signed'({1'b0, rs2});

//     always_comb begin
//         case (func)
//             M_MUL:    result = signed_mul[31:0];
//             M_MULH:   result = signed_mul[63:32];
//             M_MULHU:  result = unsigned_mul[63:32];
//             M_MULHSU: result = mixed_mul[63:32];
//             default:  result = 0;
//         endcase
//     end

// endmodule // correct_mult


// module testbench;

//     logic clock, start, reset, done, failed;
//     DATA r1, r2, correct_r, mul_r;
//     MULT_FUNC f;

//     string fmt;

//     mult dut(
//         .clock(clock),
//         .reset(reset),
//         .start(start),
//         .rs1(r1),
//         .rs2(r2),
//         .func(f),
//         .result(mul_r),
//         .done(done)
//     );

//     correct_mult not_dut(
//         .rs1(r1),
//         .rs2(r2),
//         .func(f),
//         .result(correct_r)
//     );


//     always begin
//         #(`CLOCK_PERIOD/2.0);
//         clock = ~clock;
//     end


//     task wait_until_done;
//         forever begin : wait_loop
//             @(posedge done);
//             @(negedge clock);
//             if (done) begin
//                 disable wait_until_done;
//             end
//         end
//     endtask


//     task test;
//         input MULT_FUNC func;
//         input DATA reg_1, reg_2;
//         begin
//             @(negedge clock);
//             start = 1;
//             r1 = reg_1;
//             r2 = reg_2;
//             f = func;
//             @(negedge clock);
//             start = 0;
//             wait_until_done();
//             $display(fmt, f.name(), r1, r2, correct_r, mul_r);
//             if (correct_r != mul_r) begin
//                 $display("NOT EQUAL");
//                 failed = 1;
//             end
//             @(negedge clock);
//         end
//     endtask


//     initial begin
//         clock = 0;
//         reset = 1;
//         failed = 0;
//         @(negedge clock);
//         @(negedge clock);
//         reset = 0;
//         @(negedge clock);

//         fmt = "%-8s | %3d * %3d = correct: %3d | mul: %3d";
//         $display("");
//         test(M_MUL, 0, 0);
//         test(M_MUL, 1, 0);
//         test(M_MUL, 0, 1);
//         test(M_MUL, 3, 4);
//         test(M_MUL, 2, 15);
//         test(M_MUL, 15, 2);
//         test(M_MUL, 30, 30);

//         fmt = "%-8s | %h * %h = correct: %h | mul: %h";
//         $display("");
//         test(M_MUL,    32'hff12_3456, 32'hfffff888);
//         test(M_MULH,   32'hff12_3456, 32'hfffff888);
//         test(M_MULHU,  32'hff12_3456, 32'hfffff888);
//         test(M_MULHSU, 32'hff12_3456, 32'hfffff888);

//         fmt = "%-8s | %d * %d = correct: %d | mul: %d";
//         $display("");
//         test(M_MUL,    32'h3 << 30, 4);
//         test(M_MULH,   32'h3 << 30, 4);
//         test(M_MULHU,  32'h3 << 30, 4);
//         test(M_MULHSU, 32'h3 << 30, 4);
//         test(M_MUL,    4, 32'h3 << 30);
//         test(M_MULH,   4, 32'h3 << 30);
//         test(M_MULHU,  4, 32'h3 << 30);
//         test(M_MULHSU, 4, 32'h3 << 30);

//         fmt = "%-8s | %d * %d = correct: %d | mul: %d";
//         $display(""); repeat (10) test(M_MUL,    $random, $random);
//         $display(""); repeat (10) test(M_MULH,   $random, $random);
//         $display(""); repeat (10) test(M_MULHU,  $random, $random);
//         $display(""); repeat (10) test(M_MULHSU, $random, $random);

//         $display("");

//         if (failed)
//             $display("@@@ Failed\n");
//         else
//             $display("@@@ Passed\n");

//         $finish;
//     end

// endmodule
