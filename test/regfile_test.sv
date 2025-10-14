`timescale 1ns/1ps
`include "sys_defs.svh"

module regfile_test;

  localparam int  N    = `N;
  localparam time TCK  = 10ns;

  // DUT I/O
  logic                         clock;
  logic                         reset_n;
  PHYS_TAG [2*N-1:0]            read_idx;
  DATA     [2*N-1:0]            read_out;
  logic     [N-1:0]             write_en;
  PHYS_TAG  [N-1:0]             write_idx;
  DATA      [N-1:0]             write_data;

  // DUT
  regfile dut (
    .clock     (clock),
    .reset_n   (reset_n),
    .read_idx  (read_idx),
    .read_out  (read_out),
    .write_en  (write_en),
    .write_idx (write_idx),
    .write_data(write_data)
  );

  // Clock
  initial clock = 0;
  always #(TCK/2) clock = ~clock;

  // Helpers
  task automatic clear_bus();
    write_en   = '0;
    write_idx  = '{default: PHYS_TAG'(0)};
    write_data = '{default: '0};
    read_idx   = '{default: PHYS_TAG'(0)};
  endtask

  task automatic expect_eq(string msg, DATA got, DATA exp);
    if (got !== exp) begin
      $display("[%0t] FAIL: %s (got=%h exp=%h)", $time, msg, got, exp);
      $display("@@@ Failed");   // for Makefile grep
      $fatal(1);
    end
  endtask

  // Known patterns + tags
  localparam DATA     A   = 32'hAAAA_AAAA;
  localparam DATA     B   = 32'hBBBB_BBBB;
  localparam DATA     C   = 32'hCCCC_CCCC;
  localparam DATA     D1  = 32'hDEAD_BEEF;     // used for enable=0 test
  localparam DATA     ZW  = 32'hFFFF_FFFF;     // attempted write to phys-0
  localparam PHYS_TAG T5  = PHYS_TAG'(5);
  localparam PHYS_TAG T10 = PHYS_TAG'(10);
  localparam PHYS_TAG T20 = PHYS_TAG'(20);
  localparam PHYS_TAG T7  = PHYS_TAG'(7);      // fresh tag for enable=0 test

  initial begin
    $display("=== tb_regfile_multi_rw: start ===");
    clear_bus();

    // ----- reset pulse -----
    reset_n = 1'b0;
    repeat (2) @(posedge clock);
    reset_n = 1'b1;            // mem[] now guaranteed zeroed
    @(posedge clock);

    // ===== Test 1: N writes in one cycle, read back on 2N ports =====
    if (N >= 1) begin write_en[0]=1; write_idx[0]=T5;  write_data[0]=A; end
    if (N >= 2) begin write_en[1]=1; write_idx[1]=T10; write_data[1]=B; end
    if (N >= 3) begin write_en[2]=1; write_idx[2]=T20; write_data[2]=C; end

    @(posedge clock);  // commit writes
    write_en = '0;

    // Read back over 2N ports (duplicates allowed)
    read_idx = '{default: PHYS_TAG'(0)};
    if (2*N >= 1) read_idx[0] = T5;
    if (2*N >= 2) read_idx[1] = T10;
    if (2*N >= 3) read_idx[2] = T20;
    if (2*N >= 4) read_idx[3] = T5;
    if (2*N >= 5) read_idx[4] = PHYS_TAG'(0); // must be 0
    if (2*N >= 6) read_idx[5] = T10;

    #1; // let combinational reads settle

    if (2*N >= 1) expect_eq("R0=T5",  read_out[0], A);
    if (2*N >= 2) expect_eq("R1=T10", read_out[1], B);
    if (2*N >= 3) expect_eq("R2=T20", read_out[2], C);
    if (2*N >= 4) expect_eq("R3=T5",  read_out[3], A);
    if (2*N >= 5) expect_eq("R4=T0",  read_out[4], '0);
    if (2*N >= 6) expect_eq("R5=T10", read_out[5], B);

    // ===== Test 2: Enable=0 blocks write =====
    clear_bus();
    // drive address/data but keep enable=0
    write_idx[0]  = T7;
    write_data[0] = D1;
    @(posedge clock);           // no write should occur

    // read back T7 -> must still be 0
    clear_bus();
    read_idx[0] = T7;
    #1;
    expect_eq("enable=0 blocks write", read_out[0], '0);

    // ===== Test 3: Writes to phys-0 are ignored =====
    clear_bus();
    write_en[0]  = 1'b1;
    write_idx[0] = PHYS_TAG'(0);
    write_data[0]= ZW;
    @(posedge clock);           // attempted write to zero-reg

    // read phys-0 -> must be 0
    clear_bus();
    read_idx[0] = PHYS_TAG'(0);
    #1;
    expect_eq("writes to phys-0 ignored", read_out[0], '0);

    // ===== Test 4: Same-cycle RAW forwarding (single writer) =====
    clear_bus();
    read_idx[0]  = T5;
    write_en[0]  = 1'b1;
    write_idx[0] = T5;
    write_data[0]= 32'h1234_5678;

    // With bypass, new data should appear immediately (same cycle)
    #1;
    expect_eq("same-cycle RAW (single port)", read_out[0], 32'h1234_5678);

    // After the clock edge, value should be committed as well
    @(posedge clock);
    #1;
    expect_eq("post-edge holds new value", read_out[0], 32'h1234_5678);

    // ===== Test 5: Same-cycle RAW with two writers to same tag (last-wins) =====
    if (N >= 2) begin
      clear_bus();
      read_idx[0]  = T10;

      // Two ports target same tag this cycle; higher w (1) should win
      write_en[0]  = 1'b1; write_idx[0] = T10; write_data[0] = 32'hAAAA_0001;
      write_en[1]  = 1'b1; write_idx[1] = T10; write_data[1] = 32'hBBBB_0002;

      // Bypass should pick the higher-index writer immediately
      #1;
      expect_eq("same-cycle RAW (two ports, last-wins)", read_out[0], 32'hBBBB_0002);

      // After the clock, mem[T10] should also hold the winner
      @(posedge clock);
      #1;
      expect_eq("post-edge holds last-wins value", read_out[0], 32'hBBBB_0002);
    end

    $display("=== PASS: multi-write/multi-read + enable + zero-reg + RAW fwd ===");
    $display("@@@ Passed");
    $finish;
  end

endmodule
