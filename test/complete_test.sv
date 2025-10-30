// test/complete_test.sv
`timescale 1ns/1ps
`include "verilog/sys_defs.svh"

module complete_test;

  localparam int  N   = `N;
  localparam time TCK = 10ns;
  localparam int  EXW = $bits(EX_COMPLETE_ENTRY); // width of one EX_COMPLETE_ENTRY

  // Clock/reset
  logic clock, reset;
  initial clock = 0;
  always #(TCK/2) clock = ~clock;

  // DUT I/O (TB-native forms)
  logic            ex_valid [N-1:0];
  EX_COMPLETE_ENTRY  ex_comp  [N-1:0];
  ROB_UPDATE_PACKET rob_update_packet;

  // ---------------------------
  // SYNTH build: packed mirrors
  // ---------------------------
`ifdef SYNTH
  // Synth netlists flatten arrays/structs on ports. Present packed equivalents:
  logic [N-1:0]     ex_valid_p;
  logic [EXW*N-1:0] ex_comp_bus;

  function automatic [EXW*N-1:0] pack_ex_arr(input EX_COMPLETE_ENTRY a [N-1:0]);
    automatic logic [EXW*N-1:0] b; b = '0;
    for (int k=0; k<N; k++) b[k*EXW +: EXW] = a[k];
    return b;
  endfunction

  always_comb begin
    for (int k=0; k<N; k++) ex_valid_p[k] = ex_valid[k];
    ex_comp_bus = pack_ex_arr(ex_comp);
  end
`endif

  // ---------------------------
  // DUT
  // ---------------------------
`ifndef SYNTH
  // Behavioral RTL build — module has an array-of-structs interface
  complete #(.N(N)) dut (
    .clock,
    .reset,
    .ex_valid(ex_valid),
    .ex_comp (ex_comp),
    .rob_update_packet
  );
`else
  // Synth netlist — no parameters on ports; use packed equivalents
  complete dut (
    .clock,
    .reset,
    .ex_valid(ex_valid_p),
    .ex_comp (ex_comp_bus),
    .rob_update_packet(rob_update_packet)
  );
`endif

  // ---------------------------
  // Helpers
  // ---------------------------
  task automatic clr_inputs();
    for (int i=0;i<N;i++) begin
      ex_valid[i] = 1'b0;
      ex_comp[i]  = '0;
    end
  endtask

  task automatic expect_ok(input bit cond, input string msg);
    if (!cond) begin
      $display("[%0t] FAIL: %s", $time, msg);
      $display("@@@ Failed");
      $fatal(1);
    end
  endtask

  function automatic EX_COMPLETE_ENTRY mk_ex(input int ridx,
                                           input bit br_valid,
                                           input bit br_taken,
                                           input int br_tgt);
    EX_COMPLETE_ENTRY p;
    p.rob_idx       = ROB_IDX'(ridx);
    p.branch_valid  = br_valid;
    p.mispredict    = 1'b1;                 // intentionally set; DUT must ignore
    p.branch_taken  = br_taken;
    p.branch_target = ADDR'(br_tgt);
    p.dest_pr       = PHYS_TAG'(0);
    p.result        = DATA'(0);
    return p;
  endfunction

  function automatic int onesum(input logic [N-1:0] v);
    int s=0; for (int i=0;i<N;i++) s+=v[i]; return s;
  endfunction

  // Temps (avoid mid-block decls)
  int ridx2;
  int ridx0;
  EX_COMPLETE_ENTRY tmp;

  // ---------------------------
  // The tests
  // ---------------------------
  initial begin
    // default init
    reset = 1'b1; clr_inputs();
    repeat (2) @(posedge clock);
    reset = 1'b0; @(posedge clock);

    // Test 0: all lanes invalid -> all zeros out
    clr_inputs(); #1;
    expect_ok(rob_update_packet.valid == '0, "T0: all valid bits should be 0");

    // elementwise zero checks (no assignment patterns in comparisons)
    for (int i=0;i<N;i++) begin
      expect_ok(rob_update_packet.idx[i] == ROB_IDX'(0),  $sformatf("T0: idx[%0d]==0", i));
      expect_ok(rob_update_packet.branch_targets[i] == ADDR'(0), $sformatf("T0: br_tgt[%0d]==0", i));
    end
    expect_ok(rob_update_packet.branch_taken == '0, "T0: branch_taken cleared");
    $display("[%0t] PASS: T0 all-zero passthrough", $time);

    // Test 1: single lane, no branch info
    clr_inputs();
    ex_valid[0] = 1'b1;
    ex_comp[0]  = mk_ex(/*ridx*/ 42, /*br_valid*/0, /*taken*/0, /*tgt*/'0);
    #1;
    expect_ok(rob_update_packet.valid[0] == 1'b1, "T1: valid[0]");
    expect_ok(rob_update_packet.idx[0]   == ROB_IDX'(42), "T1: idx[0]=42");
    expect_ok(rob_update_packet.branch_taken[0]   == 1'b0, "T1: branch_taken[0]=0");
    expect_ok(rob_update_packet.branch_targets[0] == ADDR'(0), "T1: branch_target[0]=0");
    for (int i=1;i<N;i++) expect_ok(rob_update_packet.valid[i]==0, $sformatf("T1: valid[%0d]==0", i));
    $display("[%0t] PASS: T1 single-lane no-branch", $time);

    // Test 2: single lane with branch info (taken to 0x100)
    clr_inputs();
    ridx2 = 7;
    ex_valid[1] = 1'b1;
    ex_comp[1]  = mk_ex(/*ridx*/ ridx2, /*br_valid*/1, /*taken*/1, /*tgt*/32'h100);
    #1;
    expect_ok(rob_update_packet.valid[1] == 1'b1, "T2: valid[1]");
    expect_ok(rob_update_packet.idx[1]   == ROB_IDX'(ridx2), "T2: idx[1]");
    expect_ok(rob_update_packet.branch_taken[1]==1'b1, "T2: branch_taken[1]=1");
    expect_ok(rob_update_packet.branch_targets[1]==ADDR'(32'h100), "T2: br_target[1]=0x100");
    for (int i=0;i<N;i++) if (i!=1) begin
      expect_ok(rob_update_packet.valid[i]==0, $sformatf("T2: valid[%0d]==0", i));
      expect_ok(rob_update_packet.branch_taken[i]==0, $sformatf("T2: br_taken[%0d]==0", i));
      expect_ok(rob_update_packet.branch_targets[i]==ADDR'(0), $sformatf("T2: br_tgt[%0d]==0", i));
    end
    $display("[%0t] PASS: T2 single-lane with branch info", $time);

    // Test 3: multi-lane sparse: lanes 0 and 2 valid, with lane2 carrying branch info
    clr_inputs();
    ridx0 = 3; ridx2 = 19;
    ex_valid[0] = 1'b1; ex_comp[0] = mk_ex(ridx0, /*br_valid*/0, 0, 0);
    if (N>2) begin
      ex_valid[2] = 1'b1; ex_comp[2] = mk_ex(ridx2, /*br_valid*/1, 1, 32'hABC);
    end
    #1;
    expect_ok(onesum(rob_update_packet.valid) == ((N>2)?2:1), "T3: valid popcount");
    expect_ok(rob_update_packet.idx[0] == ROB_IDX'(ridx0), "T3: idx[0]");
    if (N>2) begin
      expect_ok(rob_update_packet.idx[2] == ROB_IDX'(ridx2), "T3: idx[2]");
      expect_ok(rob_update_packet.branch_taken[2]==1'b1, "T3: br_taken[2]=1");
      expect_ok(rob_update_packet.branch_targets[2]==ADDR'(32'hABC), "T3: br_tgt[2]=ABC");
    end
    $display("[%0t] PASS: T3 multi-lane sparse", $time);

    // Test 4: “mispredict” flag is ignored by DUT
    clr_inputs();
    ex_valid[0] = 1'b1;
    tmp = mk_ex(55, /*br_valid*/1, /*taken*/0, /*tgt*/32'h4444_0000);
    tmp.mispredict = 1'b1; // should not affect outputs
    ex_comp[0] = tmp;
    #1;
    expect_ok(rob_update_packet.valid[0]==1, "T4: valid[0]");
    expect_ok(rob_update_packet.idx[0]==ROB_IDX'(55), "T4: idx[0]=55");
    expect_ok(rob_update_packet.branch_taken[0]==1'b0, "T4: br_taken[0]=0");
    expect_ok(rob_update_packet.branch_targets[0]==ADDR'(32'h4444_0000), "T4: br_tgt[0]=44440000");
    $display("[%0t] PASS: T4 mispredict bit ignored by complete", $time);

    $display("=== All complete.sv unit tests passed ===");
    $display("@@@ Passed");
    $finish;
  end

endmodule