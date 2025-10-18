// rename_tables.sv
`ifndef __RENAME_TABLES_V__
`define __RENAME_TABLES_V__
`timescale 1ns/100ps

// ============================================================
// N-wide Map Table (rename) + Architectural Map Table (commit)
// ============================================================

module map_table #(
  parameter int ARCH_COUNT = 32,                 // # architectural regs
  parameter int PHYS_REGS  = 64,                 // # physical regs
  parameter int N          = 3,                  // superscalar width (override as needed)
  localparam int PRW       = (PHYS_REGS <= 2) ? 1 : $clog2(PHYS_REGS)
) (
  input  logic                                  clock,
  input  logic                                  reset,

  // Precise architectural snapshot (from arch_maptable)
  input  logic [ARCH_COUNT-1:0][PRW-1:0]        archi_maptable,

  // Branch recovery: restore precise state on next cycle
  input  logic                                  BPRecoverEN,

  // CDB: up to N broadcasts this cycle (valid + tag)
  input  logic          [N-1:0]                 cdb_valid,
  input  logic [N-1:0][PRW-1:0]                 cdb_tag,

  // Per-lane new rename (AR -> PR) for dispatch lanes [N-1:0]
  // Convention: lane N-1 = oldest, lane 0 = youngest
  input  logic [N-1:0][PRW-1:0]                 maptable_new_pr,
  input  logic [N-1:0][$clog2(ARCH_COUNT)-1:0]  maptable_new_ar,

  // Operand architectural indices for this cycle’s N instructions
  input  logic [N-1:0][$clog2(ARCH_COUNT)-1:0]  reg1_ar,
  input  logic [N-1:0][$clog2(ARCH_COUNT)-1:0]  reg2_ar,

  // Lookups + readiness (after considering same-cycle renames & CDB forwarding)
  output logic [N-1:0][PRW-1:0]                 reg1_tag,
  output logic [N-1:0][PRW-1:0]                 reg2_tag,
  output logic [N-1:0]                          reg1_ready,
  output logic [N-1:0]                          reg2_ready,

  // Old physical mapping (for freeing at retire)
  output logic [N-1:0][PRW-1:0]                 Told_out

`ifdef TEST_MODE
 ,output logic [ARCH_COUNT-1:0][PRW-1:0]        map_array_disp,
  output logic [ARCH_COUNT-1:0]                 ready_array_disp
`endif
);
  typedef logic [PRW-1:0] PR_TAG;

  // Live state
  PR_TAG map_array          [ARCH_COUNT];        // AR -> PR
  logic  ready_array        [ARCH_COUNT];        // AR ready?

  // Next-state pipeline (stage N = current; stage 0 = after all updates)
  PR_TAG map_array_next     [N:0][ARCH_COUNT];
  logic  ready_array_next   [N:0][ARCH_COUNT];

  // Reset / precise images
  PR_TAG map_array_reset    [ARCH_COUNT];
  PR_TAG map_array_PS       [ARCH_COUNT];
  logic  ready_array_reset  [ARCH_COUNT];
  logic  ready_array_PS     [ARCH_COUNT];
  // Hold Told (start-of-cycle snapshot) stable for the whole cycle
  logic [N-1:0][PRW-1:0] told_next;
  logic [N-1:0][PRW-1:0] told_q;

    // Return 1 if any valid CDB tag equals 'tag'
  function automatic bit cdb_hits_tag(
    input PR_TAG                         tag,
    input logic [N-1:0]                  v,
    input logic [N-1:0][PRW-1:0]         t
  );
    bit h = 1'b0;
    for (int c = 0; c < N; ++c) begin
      if (v[c] && (t[c] == tag)) begin
        h = 1'b0 | 1'b1; // force 2-state result
        h = 1'b1;
`ifdef TEST_MODE
        $display("CDB-HIT: tag=%0d (slot %0d) @ %0t", tag, c, $time);
`endif
      end
    end
    return h;
  endfunction



`ifdef TEST_MODE
  for (genvar gi = 0; gi < ARCH_COUNT; ++gi) begin : G_DBG
    assign map_array_disp[gi]  = map_array[gi];
    assign ready_array_disp[gi] = ready_array[gi];
  end
`endif

  // --------- Reset and precise (architectural) images ----------
  always_comb begin
    for (int i = 0; i < ARCH_COUNT; ++i) begin
      map_array_reset[i]   = PR_TAG'(i[PRW-1:0]); // identity AR->PR
      ready_array_reset[i] = 1'b1;                // ready at reset
    end
  end

  always_comb begin
    for (int i = 0; i < ARCH_COUNT; ++i) begin
      map_array_PS[i]   = archi_maptable[i];      // precise AR->PR
      ready_array_PS[i] = 1'b1;                   // architectural state is ready
    end
  end

  // -------------------- State registers -----------------------
  always_ff @(posedge clock) begin
    if (reset) begin
      for (int i = 0; i < ARCH_COUNT; ++i) begin
        map_array[i]   <= map_array_reset[i];
        ready_array[i] <= ready_array_reset[i];
      end
    end else if (BPRecoverEN) begin
      for (int i = 0; i < ARCH_COUNT; ++i) begin
        map_array[i]   <= map_array_PS[i];
        ready_array[i] <= ready_array_PS[i];
      end
    end else begin
      for (int i = 0; i < ARCH_COUNT; ++i) begin
        map_array[i]   <= map_array_next[0][i];
        ready_array[i] <= ready_array_next[0][i];
      end
    end
  end

  // -------------- Compute next state (CDB + renames) ----------
  always_comb begin
    int unsigned ar_idx;
    // Stage N starts from current state
    for (int r = 0; r < ARCH_COUNT; ++r) begin
      map_array_next[N][r]   = map_array[r];
      ready_array_next[N][r] = ready_array[r];
    end

    // CDB readiness forwarding on current state
        // CDB readiness forwarding on current state (robust form)
    for (int r = 0; r < ARCH_COUNT; ++r) begin
      if (cdb_hits_tag(map_array_next[N][r], cdb_valid, cdb_tag)) begin
        ready_array_next[N][r] = 1'b1;
      end
    end


    // Apply N renames in priority order: oldest (N-1) -> youngest (0)
    for (int lane = N-1; lane >= 0; --lane) begin
      // Default propagate from the next stage
      for (int r = 0; r < ARCH_COUNT; ++r) begin
        map_array_next[lane][r]   = map_array_next[lane+1][r];
        ready_array_next[lane][r] = ready_array_next[lane+1][r];
      end

      // This lane’s rename
      ar_idx = int'(maptable_new_ar[lane]);
      if (ar_idx < ARCH_COUNT) begin
        map_array_next[lane][ar_idx] = maptable_new_pr[lane];
        // AR0 (x0) stays ready; all other renamed ARs become not ready
        if (ar_idx != 0) ready_array_next[lane][ar_idx] = 1'b0;
      end
    end
  end

    // Compute Told from the registered start-of-cycle map
  always_comb begin
    for (int lane = N-1; lane >= 0; --lane) begin
      told_next[lane] = map_array[ maptable_new_ar[lane] ];
    end
  end

    // Register Told so it’s stable across the cycle
  always_ff @(posedge clock) begin
    if (reset) begin
      for (int lane = 0; lane < N; ++lane) told_q[lane] <= '0;
    end else begin
      told_q <= told_next;
    end
  end

  // Drive the output port
  assign Told_out = told_q;



  // -------------- Operand lookups & Told production ----------
  // -------------- Operand lookups & Told production ----------
  // -------------- Operand lookups (no Told here) --------------
  always_comb begin
    for (int lane = N-1; lane >= 0; --lane) begin
      // Sources see all older lanes and CDB effects
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

  // From retire: per-lane architectural commit (program order: N-1 oldest)
  input  logic          [N-1:0]                 Retire_EN,
  input  logic [N-1:0][PRW-1:0]                 Tnew_in,     // committed PR for AR
  input  logic [N-1:0][$clog2(ARCH_COUNT)-1:0]  Retire_AR,   // architectural reg

  // Precise architectural mapping (feed back to map_table for recovery)
  output logic [ARCH_COUNT-1:0][PRW-1:0]        archi_maptable
);
  typedef logic [PRW-1:0] PR_TAG;
  PR_TAG archi_maptable_next  [ARCH_COUNT];
  PR_TAG archi_maptable_reset [ARCH_COUNT];

  // Reset to identity AR->PR
  always_comb begin
    for (int i = 0; i < ARCH_COUNT; ++i) begin
      archi_maptable_reset[i] = PR_TAG'(i[PRW-1:0]);
    end
  end

  // Register the precise architectural state
  always_ff @(posedge clock) begin
    if (reset) begin
      for (int i = 0; i < ARCH_COUNT; ++i) begin
        archi_maptable[i] <= archi_maptable_reset[i];
      end
    end else begin
      for (int i = 0; i < ARCH_COUNT; ++i) begin
        archi_maptable[i] <= archi_maptable_next[i];
      end
    end
  end

  // Apply up to N commits per cycle, in program order (oldest first)
  always_comb begin
    for (int i = 0; i < ARCH_COUNT; ++i) begin
      archi_maptable_next[i] = archi_maptable[i];
    end
    for (int lane = N-1; lane >= 0; --lane) begin
      if (Retire_EN[lane]) begin
        archi_maptable_next[ Retire_AR[lane] ] = Tnew_in[lane];
      end
    end
  end
endmodule

`endif // __RENAME_TABLES_V__
