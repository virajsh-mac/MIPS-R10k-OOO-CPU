// test/map_table_test.sv
`timescale 1ns/100ps
`ifndef SYNTH
`define TEST_MODE
`endif

// ----------------------------------------------------------
// Gate-level convenience: RTL stub for arch_maptable
// (Only compiled when SYNTH is defined.)
// ----------------------------------------------------------
`ifdef SYNTH
module arch_maptable #(
  parameter int ARCH_COUNT = 32,
  parameter int PHYS_REGS  = 64,
  parameter int N          = 3,
  localparam int PRW       = (PHYS_REGS <= 2) ? 1 : $clog2(PHYS_REGS)
)(
  input  logic                                  clock,
  input  logic                                  reset,
  input  logic          [N-1:0]                 Retire_EN,
  input  logic [N-1:0][PRW-1:0]                 Tnew_in,
  input  logic [N-1:0][$clog2(ARCH_COUNT)-1:0]  Retire_AR,
  output logic [ARCH_COUNT-1:0][PRW-1:0]        archi_maptable
);
  typedef logic [PRW-1:0] PR_TAG;
  PR_TAG archi_maptable_next [ARCH_COUNT];

  always_ff @(posedge clock) begin
    if (reset) begin
      for (int i = 0; i < ARCH_COUNT; i++)
        archi_maptable[i] <= PR_TAG'(i[PRW-1:0]); // identity
    end else begin
      for (int i = 0; i < ARCH_COUNT; i++)
        archi_maptable_next[i] = archi_maptable[i];
      for (int lane = N-1; lane >= 0; lane--) begin
        if (Retire_EN[lane])
          archi_maptable_next[ Retire_AR[lane] ] = Tnew_in[lane];
      end
      for (int i = 0; i < ARCH_COUNT; i++)
        archi_maptable[i] <= archi_maptable_next[i];
    end
  end
endmodule
`endif

module map_table_test;
  // ---- DUT params ----
  localparam int ARCH_COUNT = 32;
  localparam int PHYS_REGS  = 64;
  localparam int N          = 3;
  localparam int PRW        = (PHYS_REGS <= 2) ? 1 : $clog2(PHYS_REGS);

  // ---- Clock/Reset ----
  logic clock, reset;
  initial begin clock = 0; forever #5 clock = ~clock; end
  initial begin reset = 1; repeat (2) @(posedge clock); reset = 0; end

  // ---- Wires between DUTs ----
  logic [ARCH_COUNT-1:0][PRW-1:0] archi_maptable;

  // map_table inputs
  logic                           BPRecoverEN;
  logic         [N-1:0]           cdb_valid;
  logic [N-1:0][PRW-1:0]          cdb_tag;
  logic [N-1:0][PRW-1:0]          maptable_new_pr;
  logic [N-1:0][$clog2(ARCH_COUNT)-1:0] maptable_new_ar;
  logic [N-1:0][$clog2(ARCH_COUNT)-1:0] reg1_ar, reg2_ar;

  // map_table outputs
  logic [N-1:0][PRW-1:0]          reg1_tag, reg2_tag;
  logic [N-1:0]                   reg1_ready, reg2_ready;
  logic [N-1:0][PRW-1:0]          Told_out;

  // ===== TEST_MODE taps + helpers =====
`ifdef TEST_MODE
  // RTL: internal display taps exposed by map_table when TEST_MODE
  logic [ARCH_COUNT-1:0][PRW-1:0] map_array_disp;
  logic [ARCH_COUNT-1:0]          ready_array_disp;

  function automatic int get_map(input int ar);   return map_array_disp[ar]; endfunction
  function automatic bit get_ready(input int ar); return ready_array_disp[ar]; endfunction
`else
  // Gate-level: no internal taps; safe stubs (won't be used by guarded checks)
  function automatic int get_map(input int ar);   return 0;    endfunction
  function automatic bit get_ready(input int ar); return 1'b1; endfunction
`endif

  // arch_maptable (retire) inputs
  logic [N-1:0]                   Retire_EN;
  logic [N-1:0][PRW-1:0]          Tnew_in;
  logic [N-1:0][$clog2(ARCH_COUNT)-1:0] Retire_AR;

  // ---- DUTs ----
`ifdef SYNTH
  map_table u_mt (
`else
  map_table #(.ARCH_COUNT(ARCH_COUNT), .PHYS_REGS(PHYS_REGS), .N(N)) u_mt (
`endif
    .clock, .reset,
    .archi_maptable(archi_maptable),
    .BPRecoverEN(BPRecoverEN),
    .cdb_valid, .cdb_tag,
    .maptable_new_pr, .maptable_new_ar,
    .reg1_ar, .reg2_ar,
    .reg1_tag, .reg2_tag, .reg1_ready, .reg2_ready,
    .Told_out
`ifdef TEST_MODE
   , .map_array_disp(map_array_disp), .ready_array_disp(ready_array_disp)
`endif
  );

  arch_maptable #(.ARCH_COUNT(ARCH_COUNT), .PHYS_REGS(PHYS_REGS), .N(N)) u_amt (
    .clock, .reset,
    .Retire_EN, .Tnew_in, .Retire_AR,
    .archi_maptable
  );

  // ================= Helpers =================
  task automatic clear_dispatch();
    for (int i = 0; i < N; ++i) begin
      maptable_new_ar[i] = '0;
      maptable_new_pr[i] = '0;
      reg1_ar[i]         = '0;
      reg2_ar[i]         = '0;
    end
  endtask

  task automatic clear_cdb();    cdb_valid = '0; cdb_tag = '0; endtask
  task automatic clear_retire(); Retire_EN = '0; Tnew_in = '0; Retire_AR = '0; endtask

  task automatic begin_cycle();
    @(negedge clock);
    clear_dispatch(); clear_cdb(); clear_retire();
    BPRecoverEN = 1'b0;
  endtask

  task automatic end_cycle();
    @(posedge clock); #1;
  endtask

  task automatic set_lane(
    input int lane,
    input int ar_dst,
    input int pr_new,
    input int ar_src1,
    input int ar_src2
  );
    maptable_new_ar[lane] = ar_dst[$clog2(ARCH_COUNT)-1:0];
    maptable_new_pr[lane] = pr_new[PRW-1:0];
    reg1_ar[lane]         = ar_src1[$clog2(ARCH_COUNT)-1:0];
    reg2_ar[lane]         = ar_src2[$clog2(ARCH_COUNT)-1:0];
  endtask

  task automatic set_cdb(input int slot, input bit v, input int tag);
    cdb_valid[slot] = v;
    cdb_tag  [slot] = tag[PRW-1:0];
  endtask

  // tiny assert wrappers
  int pass_ct = 0, fail_ct = 0;
  task automatic expect_eq_int(string name, int got, int exp);
    if (got !== exp) begin
      $display("FAIL: %s  got=%0d exp=%0d", name, got, exp); fail_ct++;
    end else begin
      pass_ct++;
    end
  endtask
  task automatic expect_eq_bit(string name, bit got, bit exp);
    if (got !== exp) begin
      $display("FAIL: %s  got=%0b exp=%0b", name, got, exp); fail_ct++;
    end else begin
      pass_ct++;
    end
  endtask

  // ================= Testcases =================

  // 0) Reset: identity mapping, all ready=1
  task automatic tc_reset();
`ifdef TEST_MODE
    // Internal-state version (RTL)
    @(negedge reset); @(posedge clock); #1;
    for (int ar = 0; ar < ARCH_COUNT; ++ar) begin
      expect_eq_int($sformatf("reset map[%0d]", ar), get_map(ar), ar);
      expect_eq_bit($sformatf("reset ready[%0d]", ar), get_ready(ar), 1'b1);
    end
`else
    // I/O-only version (SYNTH): probe via lookup path
    @(negedge reset); @(posedge clock); #1;
    for (int ar = 0; ar < 8; ++ar) begin
      @(negedge clock);
        reg1_ar[0] = ar[$clog2(ARCH_COUNT)-1:0];
      @(posedge clock); #1;
      expect_eq_int($sformatf("reset lookup tag[%0d]", ar), reg1_tag[0], ar);
      expect_eq_bit($sformatf("reset lookup ready[%0d]", ar), reg1_ready[0], 1);
    end
`endif
  endtask

  // 1) Register renaming (multi-lane) + Told correctness
  task automatic tc_rename_and_told();
    begin_cycle();
      // oldest..youngest: AR1->33, AR2->34, AR3->35
      set_lane(2, 1, 33, 0, 0);
      set_lane(1, 2, 34, 0, 0);
      set_lane(0, 3, 35, 0, 0);
    end_cycle();
    
  `ifdef SYNTH
    // Gate-level: allow one cycle for told_q to be observable
    @(posedge clock); #1;
  `endif

    // Told must be old mappings (1,2,3)
    expect_eq_int("Told lane2 (AR1)", Told_out[2], 1);
    expect_eq_int("Told lane1 (AR2)", Told_out[1], 2);
    expect_eq_int("Told lane0 (AR3)", Told_out[0], 3);

`ifdef TEST_MODE
    // Internal view (RTL)
    expect_eq_int("map[1] C1", get_map(1), 33);
    expect_eq_int("map[2] C1", get_map(2), 34);
    expect_eq_int("map[3] C1", get_map(3), 35);
    expect_eq_bit("ready[1] C1", get_ready(1), 0);
    expect_eq_bit("ready[2] C1", get_ready(2), 0);
    expect_eq_bit("ready[3] C1", get_ready(3), 0);
`else
    // I/O check: younger lane reading AR just renamed by older lane sees new PR
    @(negedge clock);
      reg1_ar[0] = 1; // AR1
      reg2_ar[0] = 2; // AR2
    @(posedge clock); #1;
    expect_eq_int("C1 lookup reg1_tag==33", reg1_tag[0], 33);
    expect_eq_int("C1 lookup reg2_tag==34", reg2_tag[0], 34);
`endif
  endtask

  // 2) CDB match → mark ready
  task automatic tc_cdb_ready();
    // complete PR33 and PR34
    begin_cycle();
      set_cdb(0, 1, 33);
      set_cdb(1, 1, 34);
      set_cdb(2, 0, 0);
    end_cycle();
`ifdef TEST_MODE
    expect_eq_bit("CDB: AR1 ready", get_ready(1), 1);
    expect_eq_bit("CDB: AR2 ready", get_ready(2), 1);
    expect_eq_bit("CDB: AR3 still 0", get_ready(3), 0);
`else
    // I/O check: probe readiness via source lookup (reg*_ready)
    @(negedge clock);
      reg1_ar[0] = 1; // AR1
      reg2_ar[0] = 2; // AR2
    @(posedge clock); #1;
    expect_eq_bit("CDB/IO AR1 ready", reg1_ready[0], 1);
    expect_eq_bit("CDB/IO AR2 ready", reg2_ready[0], 1);
`endif
  endtask

  // 3) Rename again + CDB for a different PR
  task automatic tc_rename_again_and_cdb();
    begin_cycle();
      set_lane(2, 1, 36, 1, 2); // AR1->36, AR1 not-ready again
      set_cdb(0, 1, 35);        // complete PR35 (AR3)
    end_cycle();
`ifdef TEST_MODE
    expect_eq_int("C3 map[1]=36", get_map(1), 36);
    expect_eq_bit("C3 AR1 not-ready", get_ready(1), 0);
    expect_eq_bit("C3 AR3 now ready", get_ready(3), 1);
`else
    // I/O: AR3 should now be ready; AR1 not ready
    @(negedge clock);
      reg1_ar[0] = 3; // AR3
      reg2_ar[0] = 1; // AR1
    @(posedge clock); #1;
    expect_eq_bit("C3/IO AR3 ready", reg1_ready[0], 1);
    expect_eq_bit("C3/IO AR1 not-ready", reg2_ready[0], 0);
`endif
  endtask

  // 4) AMT updates on retire
  task automatic tc_retire_updates_amt();
    begin_cycle();
      Retire_EN[N-1] = 1; Retire_AR[N-1] = 1; Tnew_in[N-1] = 33;
      Retire_EN[N-2] = 1; Retire_AR[N-2] = 2; Tnew_in[N-2] = 34;
    end_cycle();

    expect_eq_int("AMT AR1=33", archi_maptable[1], 33);
    expect_eq_int("AMT AR2=34", archi_maptable[2], 34);
  endtask

  // 5) Branch recovery: copy AMT -> MT and set ready=1
  task automatic tc_recovery_from_amt();
`ifdef TEST_MODE
    begin_cycle(); BPRecoverEN = 1; end_cycle(); BPRecoverEN = 0;
    for (int ar = 0; ar < ARCH_COUNT; ++ar) begin
      expect_eq_int($sformatf("REC map[%0d]=AMT", ar), get_map(ar), archi_maptable[ar]);
      expect_eq_bit($sformatf("REC ready[%0d]=1", ar), get_ready(ar), 1);
    end
`else
    // I/O-only: tweak AMT first, then recover and observe via lookups
    begin_cycle();
      Retire_EN[N-1]=1; Retire_AR[N-1]=5; Tnew_in[N-1]=42; // AMT[5]=42
    end_cycle();

    begin_cycle(); BPRecoverEN = 1; end_cycle(); BPRecoverEN = 0;

    @(negedge clock) reg1_ar[0]=5;
    @(posedge clock); #1;
    expect_eq_int("REC/IO lookup AR5==42", reg1_tag[0], 42);
    expect_eq_bit("REC/IO ready AR5==1",  reg1_ready[0], 1);
`endif
  endtask

  // 6) Same-cycle bypass: younger sees older-lane rename
  task automatic tc_same_cycle_bypass();
    begin_cycle();
      set_lane(2, 10, 50, 0, 0);   // oldest
      set_lane(1, 11, 51, 10, 0);  // reads AR10
      set_lane(0, 12, 52, 11, 10); // reads AR11 & AR10
    end_cycle();

`ifdef TEST_MODE
    expect_eq_int("bypass map[10]=50", get_map(10), 50);
    expect_eq_int("bypass map[11]=51", get_map(11), 51);
    expect_eq_int("bypass map[12]=52", get_map(12), 52);
`endif
    expect_eq_int("lane1 reg1_tag == 50", reg1_tag[1], 50);
    expect_eq_int("lane0 reg1_tag == 51", reg1_tag[0], 51);
    expect_eq_int("lane0 reg2_tag == 50", reg2_tag[0], 50);
  endtask

  // 7) AR0 behavior: stays ready
  task automatic tc_ar0_always_ready();
    begin_cycle();
      set_lane(2, 0, 63, 0, 0); // write to AR0 shouldn't drop ready
    end_cycle();
`ifdef TEST_MODE
    expect_eq_bit("AR0 ready stays 1", get_ready(0), 1);
`else
    // I/O: probe via reg1_ready on AR0
    @(negedge clock) reg1_ar[0]=0;
    @(posedge clock); #1;
    expect_eq_bit("AR0 ready stays 1 (IO)", reg1_ready[0], 1);
`endif
  endtask

  // ================= Test Runner =================
  initial begin
    BPRecoverEN = 0; clear_dispatch(); clear_cdb(); clear_retire();

    tc_reset();
    tc_rename_and_told();
    tc_cdb_ready();
    tc_rename_again_and_cdb();
    tc_retire_updates_amt();
    tc_recovery_from_amt();
    tc_same_cycle_bypass();
    tc_ar0_always_ready();

    $display("\n===== SUMMARY =====");
    $display("PASS: %0d", pass_ct);
    $display("FAIL: %0d", fail_ct);
    if (fail_ct == 0) $display("\n@@@PASS: rename_tables_unit_tb completed successfully\n");
    else              $display("\n@@@FAIL: rename_tables_unit_tb had failures\n");
    $finish;
  end
endmodule
