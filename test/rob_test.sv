
`include "sys_defs.svh"
// sys_defs.svh



module rob_test;

    // DUT signals
    logic clock, reset;
    logic [`N-1:0] alloc_valid;
    ROB_ENTRY [`N-1:0] rob_entry_packet;
    ROB_IDX [`N-1:0] alloc_idxs;
    logic [$clog2(`ROB_SZ+1)-1:0] free_slots;
    ROB_UPDATE_PACKET rob_update_packet;
    ROB_ENTRY [`N-1:0] head_entries;
    logic [`N-1:0] head_valids;
    //logic [`N:0] retire_count;
    logic mispredict;
    ROB_IDX mispred_idx;

    // Instantiate DUT
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

    // Clock
    always #5 clock = ~clock;

    // Simple monitor signal for success/failure
    bit failed = 0;

    initial begin
        integer remainder;
        // --- Initialization ---
        clock = 0;
        reset = 1;
        alloc_valid = 0;
        //retire_count = 0;
        rob_update_packet = '{default:0};
        mispredict = 0;
        mispred_idx = '0;

        // --- Apply reset ---
        @(negedge clock);
        @(negedge clock);
        reset = 0;
        @(posedge clock);  // Allow one cycle after reset
    
        // monitor the following signals (can add more later):
        $monitor("Time %0t | head=%0d tail=%0d free_slots=%0d",
                  $time, dut.head, dut.tail, dut.free_slots);

        // --- Check conditions ---
        $display("Checking if ROB is empty after reset...");

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

        if (!failed)
            $display("PASS: ROB is empty and head == tail after reset.");
        else
            $display("ROB failed initial empty-state check.");

        // --- Fill the ROB ---
    $display("Filling the ROB...");

    // Mark all allocations as valid
    alloc_valid = '1;  // all bits 1
    for (int i = 0; i < `ROB_SZ / `N; i++) begin
        rob_entry_packet = '{default:'0};  // you can set some dummy values
        @(posedge clock);
    end

    // Allocate any remaining entries if `ROB_SZ` is not a multiple of `N`
    remainder = `ROB_SZ % `N;
    if (remainder > 0) begin
        alloc_valid = '0;
        for (int i = 0; i < remainder; i++) alloc_valid[i] = 1'b1;
        rob_entry_packet = '{default:'0};
        @(posedge clock);
    end

    // --- Check if ROB is full ---
    $display("Checking if ROB is full...");
    if (free_slots !== 0) begin
        $display("FAIL: Expected free_slots = 0, got %0d", free_slots);
        failed = 1;
    end

    if (!failed)
        $display("PASS: ROB full condition detected correctly.");
    else
        $display("ROB full test failed.");
    
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
