`include "sys_defs.svh"

// Pipelined 64x64 multiplier (returns low 32 for MUL, high 32 for MULH/HSU)
module mult (
    input  logic             clock,
    input  logic             reset,
    input  logic             start,
    input  DATA              rs1,
    input  DATA              rs2,
    input  MULT_FUNC         func,
    input  EX_COMPLETE_ENTRY meta_in,

    output DATA              result,   // combinational slice of product
    output logic             request,  // CDB request (S-1 cycle)
    output logic             done,     // tail stage done (S cycle)
    output EX_COMPLETE_ENTRY meta_out  // completion packet with stamped .result
);

    // ---------- Internal pipeline wiring ----------
    MULT_FUNC                             func_out;
    MULT_FUNC           [`MULT_STAGES-2:0] internal_funcs;

    logic [(64*(`MULT_STAGES-1))-1:0]     internal_sums;
    logic [(64*(`MULT_STAGES-1))-1:0]     internal_mcands;
    logic [(64*(`MULT_STAGES-1))-1:0]     internal_mpliers;
    logic            [`MULT_STAGES-1:0]   dones;

    logic [63:0] mcand, mplier, product;
    logic [63:0] mcand_out, mplier_out;  // exposed only for wiring
    logic busy;
    logic start_accept;

    EX_COMPLETE_ENTRY [`MULT_STAGES-2:0] internal_meta;
    EX_COMPLETE_ENTRY                    meta_tail;     // meta at pipeline tail

    // ---------- Stage array ----------
    mult_stage mstage[`MULT_STAGES-1:0] (
        .clock       (clock),
        .reset       (reset),
        .func        ({internal_funcs, func}),
        .start ({dones[`MULT_STAGES-2:0], start_accept}),

        //.start       ({dones[`MULT_STAGES-2:0], start}), // chain done -> next start
        .prev_sum    ({internal_sums, 64'h0}),
        .mplier      ({internal_mpliers, mplier}),
        .mcand       ({internal_mcands,  mcand }),
        .product_sum ({product, internal_sums}),
        .next_mplier ({mplier_out, internal_mpliers}),
        .next_mcand  ({mcand_out,  internal_mcands }),
        .next_func   ({func_out,   internal_funcs  }),
        .meta_in     ({internal_meta, meta_in}),
        .meta_out    ({meta_tail,     internal_meta}),
        .done        (dones)
    );

    // Busy/accept logic
    always_ff @(posedge clock) begin
        if (reset) begin
            busy <= 1'b0;
        end else begin
            // Accept a new op only if not busy
            if (!busy && start)
                busy <= 1'b1;
            // Clear when tail completes
            else if (dones[`MULT_STAGES-1])
                busy <= 1'b0;
        end
    end

assign start_accept = start & ~busy;

    // ---------- Input sign-extension ----------
    always_comb begin
        unique case (func)
            MUL, MULH, MULHSU: mcand = {{32{rs1[31]}}, rs1};
            default:           mcand = {32'b0, rs1};
        endcase
        unique case (func)
            MUL, MULH: mplier = {{32{rs2[31]}}, rs2};
            default:   mplier = {32'b0, rs2};
        endcase
    end

    // ---------- Result slice (combinational view) ----------
    assign result = (func_out == MUL) ? product[31:0] : product[63:32];

    // ---------- Completion stamping (aligned with tail) ----------
    EX_COMPLETE_ENTRY meta_out_r;
    always_comb begin
        meta_out_r         = meta_tail;               // keep rob_idx, dest_pr, etc.
        meta_out_r.result  = (func_out == MUL) ? product[31:0]
                                               : product[63:32];
        // If your struct tracks func/FU, you can set fields here as needed.
        // e.g., meta_out_r.func = func_out;  (only if that field exists)
    end
    assign meta_out = meta_out_r;

    // CDB timing (request one cycle before done; change if you want alignment)
    assign request = dones[`MULT_STAGES-2];
    assign done    = dones[`MULT_STAGES-1];

endmodule  // mult


module mult_stage (
    input  logic             clock,
    input  logic             reset,
    input  logic             start,
    input  logic [63:0]      prev_sum,
    input  logic [63:0]      mplier,
    input  logic [63:0]      mcand,
    input  MULT_FUNC         func,
    input  EX_COMPLETE_ENTRY meta_in,

    output logic [63:0]      product_sum,
    output logic [63:0]      next_mplier,
    output logic [63:0]      next_mcand,
    output MULT_FUNC         next_func,
    output EX_COMPLETE_ENTRY meta_out,
    output logic             done
);
    // Each stage processes SHIFT LSBs of the current multiplier
    localparam int SHIFT = 64 / `MULT_STAGES;

    logic [63:0] partial_product;
    logic [63:0] shifted_mplier, shifted_mcand;

    assign partial_product = mplier[SHIFT-1:0] * mcand;

    // Safer zero padding syntax
    assign shifted_mplier = {{SHIFT{1'b0}}, mplier[63:SHIFT]};
    assign shifted_mcand  = {mcand[63-SHIFT:0], {SHIFT{1'b0}}};

    // Drop this in mult.sv (TEMP DEBUG)
    always_ff @(posedge clock) begin
        if (start) begin
            $display("MULT start @%0t : rob_idx=%0d dest_pr=%0d (meta_in)", $time, meta_in.rob_idx, meta_in.dest_pr);
            end
        end


    // Gate stage registers with 'start' (and reset) to avoid X-smear pre-token
    always_ff @(posedge clock) begin
        if (reset) begin
            product_sum <= '0;
            next_mplier <= '0;
            next_mcand  <= '0;
            next_func   <= '0;
            meta_out    <= '0;
        end else if (start) begin
            product_sum <= prev_sum + partial_product;
            next_mplier <= shifted_mplier;
            next_mcand  <= shifted_mcand;
            next_func   <= func;
            meta_out    <= meta_in;
        end
    end

    // Done token advances one stage per cycle when start is pulsed
    always_ff @(posedge clock) begin
        if (reset)  done <= 1'b0;
        else        done <= start;
    end
endmodule  // mult_stage
