// map_table.sv
`ifndef MAP_TABLE_SV
`define MAP_TABLE_SV
`timescale 1ns/100ps

// Map Table (rename) + Architectural Map Table (commit)

module map_table #(
  parameter int ARCH_COUNT = 32,
  parameter int PHYS_REGS  = 64,
  parameter int N          = 3,
  localparam int PRW       = (PHYS_REGS <= 2) ? 1 : $clog2(PHYS_REGS)
) (
  input  logic                                  clock,
  input  logic                                  reset,

  // precise snapshot from commit side
  input  logic [ARCH_COUNT-1:0][PRW-1:0]        archi_maptable,

  // recovery: copy precise state next cycle
  input  logic                                  BPRecoverEN,

  // CDB: up to N broadcasts (valid + tag)
  input  logic          [N-1:0]                 cdb_valid,
  input  logic [N-1:0][PRW-1:0]                 cdb_tag,

  // per-lane renames (lane N-1 oldest .. lane 0 youngest)
  input  logic [N-1:0][PRW-1:0]                 maptable_new_pr,
  input  logic [N-1:0][$clog2(ARCH_COUNT)-1:0]  maptable_new_ar,

  // source arch regs for this cycle’s N instructions
  input  logic [N-1:0][$clog2(ARCH_COUNT)-1:0]  reg1_ar,
  input  logic [N-1:0][$clog2(ARCH_COUNT)-1:0]  reg2_ar,

  // lookups (after considering older lanes + CDB)
  output logic [N-1:0][PRW-1:0]                 reg1_tag,
  output logic [N-1:0][PRW-1:0]                 reg2_tag,
  output logic [N-1:0]                          reg1_ready,
  output logic [N-1:0]                          reg2_ready,

  // Told = old physical mapping of each renamed AR
  output logic [N-1:0][PRW-1:0]                 Told_out

`ifdef TEST_MODE
 ,output logic [ARCH_COUNT-1:0][PRW-1:0]        map_array_disp,
  output logic [ARCH_COUNT-1:0]                 ready_array_disp
`endif
);
  typedef logic [PRW-1:0] PR_TAG;

  // live state
  PR_TAG map_array        [ARCH_COUNT];   // AR -> PR
  logic  ready_array      [ARCH_COUNT];   // AR ready?

  // next-state pipeline (stage N = current, stage 0 = after all updates)
  PR_TAG map_array_next   [N:0][ARCH_COUNT];
  logic  ready_array_next [N:0][ARCH_COUNT];

  // reset / precise images
  PR_TAG map_array_reset  [ARCH_COUNT];
  PR_TAG map_array_PS     [ARCH_COUNT];
  logic  ready_array_reset[ARCH_COUNT];
  logic  ready_array_PS   [ARCH_COUNT];

  // Told (hold start-of-cycle values)
  logic [N-1:0][PRW-1:0] told_next, told_q;

  // true if any valid CDB slot matches 'tag' (ignores tag==0)
  function automatic bit cdb_hit(input PR_TAG tag,
                                 input logic [N-1:0] v,
                                 input logic [N-1:0][PRW-1:0] t);
    bit hit = 1'b0;
    if (tag != '0) begin
      for (int c = 0; c < N; c++) begin
        if (v[c] && (t[c] == tag)) begin
          hit = 1'b1;
`ifdef TEST_MODE
          $display("CDB-HIT: tag=%0d (slot %0d) @ %0t", tag, c, $time);
`endif
        end
      end
    end
    return hit;
  endfunction

`ifdef TEST_MODE
  for (genvar gi = 0; gi < ARCH_COUNT; gi++) begin : G_DBG
    assign map_array_disp [gi] = map_array[gi];
    assign ready_array_disp[gi] = ready_array[gi];
  end
`endif

  // reset/precise images
  always_comb begin
    for (int i = 0; i < ARCH_COUNT; i++) begin
      map_array_reset[i]   = PR_TAG'(i[PRW-1:0]); // identity
      ready_array_reset[i] = 1'b1;
      map_array_PS[i]      = archi_maptable[i];
      ready_array_PS[i]    = 1'b1;
    end
  end

  // state regs
  always_ff @(posedge clock) begin
    if (reset) begin
      for (int i = 0; i < ARCH_COUNT; i++) begin
        map_array[i]   <= map_array_reset[i];
        ready_array[i] <= ready_array_reset[i];
      end
    end else if (BPRecoverEN) begin
      for (int i = 0; i < ARCH_COUNT; i++) begin
        map_array[i]   <= map_array_PS[i];
        ready_array[i] <= ready_array_PS[i];
      end
    end else begin
      for (int i = 0; i < ARCH_COUNT; i++) begin
        map_array[i]   <= map_array_next[0][i];
        ready_array[i] <= ready_array_next[0][i];
      end
    end
  end
  // Build per cycle next state
  // next-state: first apply CDB to current state (stage N), then fold renames
  always_comb begin
    // start from current
    int unsigned ar_idx;
    for (int r = 0; r < ARCH_COUNT; r++) begin
      map_array_next[N][r]   = map_array[r];
      ready_array_next[N][r] = ready_array[r];
    end

    // CDB readiness (on current state only)
    for (int r = 0; r < ARCH_COUNT; r++) begin
      if (cdb_hit(map_array_next[N][r], cdb_valid, cdb_tag))
        ready_array_next[N][r] = 1'b1;
    end

    // fold N renames: oldest -> youngest
    for (int lane = N-1; lane >= 0; lane--) begin
      for (int r = 0; r < ARCH_COUNT; r++) begin
        map_array_next  [lane][r] = map_array_next  [lane+1][r];
        ready_array_next[lane][r] = ready_array_next[lane+1][r];
      end

      ar_idx = int'(maptable_new_ar[lane]);
      if (ar_idx < ARCH_COUNT) begin
        map_array_next[lane][ar_idx] = maptable_new_pr[lane];
        // x0 stays ready; others drop until completed
        if (ar_idx != 0) ready_array_next[lane][ar_idx] = 1'b0;
      end
    end
  end

  // Told = start-of-cycle map value for each renamed AR
  always_comb begin
    for (int lane = N-1; lane >= 0; lane--)
      told_next[lane] = map_array[ maptable_new_ar[lane] ];
  end

  always_ff @(posedge clock) begin
    if (reset) begin
      for (int lane = 0; lane < N; lane++) told_q[lane] <= '0;
    end else begin
      told_q <= told_next;
    end
  end

  assign Told_out = told_q;

  // operand lookups (see effects of older lanes + CDB)
  always_comb begin
    for (int lane = N-1; lane >= 0; lane--) begin
      reg1_tag  [lane] = map_array_next[lane+1][ reg1_ar[lane] ];
      reg2_tag  [lane] = map_array_next[lane+1][ reg2_ar[lane] ];
      reg1_ready[lane] = ready_array_next[lane+1][ reg1_ar[lane] ];
      reg2_ready[lane] = ready_array_next[lane+1][ reg2_ar[lane] ];
    end
  end
endmodule


module arch_maptable #(
  parameter int ARCH_COUNT = 32,
  parameter int PHYS_REGS  = 64,
  parameter int N          = 3,
  localparam int PRW       = (PHYS_REGS <= 2) ? 1 : $clog2(PHYS_REGS)
) (
  input  logic                                  clock,
  input  logic                                  reset,

  // retire side: program order (N-1 oldest)
  input  logic          [N-1:0]                 Retire_EN,
  input  logic [N-1:0][PRW-1:0]                 Tnew_in,
  input  logic [N-1:0][$clog2(ARCH_COUNT)-1:0]  Retire_AR,

  output logic [ARCH_COUNT-1:0][PRW-1:0]        archi_maptable
);
  typedef logic [PRW-1:0] PR_TAG;

  PR_TAG archi_maptable_next  [ARCH_COUNT];
  PR_TAG archi_maptable_reset [ARCH_COUNT];

  // reset to identity
  always_comb begin
    for (int i = 0; i < ARCH_COUNT; i++)
      archi_maptable_reset[i] = PR_TAG'(i[PRW-1:0]);
  end

  // register precise state
  always_ff @(posedge clock) begin
    if (reset) begin
      for (int i = 0; i < ARCH_COUNT; i++)
        archi_maptable[i] <= archi_maptable_reset[i];
    end else begin
      for (int i = 0; i < ARCH_COUNT; i++)
        archi_maptable[i] <= archi_maptable_next[i];
    end
  end

  // apply up to N commits (oldest first)
  always_comb begin
    for (int i = 0; i < ARCH_COUNT; i++)
      archi_maptable_next[i] = archi_maptable[i];

    for (int lane = N-1; lane >= 0; lane--) begin
      if (Retire_EN[lane])
        archi_maptable_next[ Retire_AR[lane] ] = Tnew_in[lane];
    end
  end
endmodule

`endif // MAP_TABLE_SV
