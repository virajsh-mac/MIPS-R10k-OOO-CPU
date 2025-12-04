
`include "sys_defs.svh"


// This is a pipelined multiplier that multiplies two 64-bit integers and
// returns the low 64 bits of the result.
// This is not an ideal multiplier but is sufficient to allow a faster clock
// period than straight multiplication.
//
// ETB (Early Tag Broadcast) Architecture:
// - Request goes out at stage MULT_STAGES-2 (early tag broadcast)
// - If granted, instruction proceeds to final stage, result broadcast next cycle
// - If NOT granted, hold at request stage until granted
// - We should NEVER hold at the final stage

module mult (
    input clock,
    reset,
    start,
    grant,  // CDB grant - allows instruction to proceed to final stage
    input DATA rs1,
    rs2,
    input MULT_FUNC func,
    input EX_COMPLETE_ENTRY meta_in,

    output DATA result,
    output logic request,
    output logic done,
    output EX_COMPLETE_ENTRY meta_out
);

    MULT_FUNC [`MULT_STAGES-2:0] internal_funcs;
    MULT_FUNC func_out;

    logic [(64*(`MULT_STAGES-1))-1:0] internal_sums, internal_mcands, internal_mpliers;
    logic [`MULT_STAGES-1:0] dones;

    logic [63:0] mcand, mplier, product;
    logic [63:0] mcand_out, mplier_out;  // unused, just for wiring

    EX_COMPLETE_ENTRY [`MULT_STAGES-2:0] internal_meta;
    EX_COMPLETE_ENTRY meta_out_pipe;  // Direct output from pipeline

    // ==========================================================================
    // Holding state at the REQUEST stage (stage MULT_STAGES-2)
    // ==========================================================================
    logic req_pending;                    // Request was made but not yet granted
    logic [63:0] sum_held;                // Accumulated sum at request stage
    logic [63:0] mcand_held;              // Multiplicand at request stage
    logic [63:0] mplier_held;             // Multiplier at request stage
    MULT_FUNC func_held;                  // Function at request stage
    EX_COMPLETE_ENTRY meta_held;          // Metadata at request stage

    // ==========================================================================
    // Final stage input/control signals
    // ==========================================================================
    logic [63:0] final_prev_sum;
    logic [63:0] final_mplier;
    logic [63:0] final_mcand;
    MULT_FUNC final_func;
    EX_COMPLETE_ENTRY final_meta;
    logic final_start;

    // ==========================================================================
    // Early pipeline stages (0 to MULT_STAGES-2)
    // ==========================================================================
    mult_stage mstage[`MULT_STAGES-2:0] (
        .clock      (clock),
        .reset      (reset),
        .func       ({internal_funcs[`MULT_STAGES-3:0], func}),
        .start      ({dones[`MULT_STAGES-3:0], start}),
        .prev_sum   ({internal_sums[64*(`MULT_STAGES-2)-1:0], 64'h0}),
        .mplier     ({internal_mpliers[64*(`MULT_STAGES-2)-1:0], mplier}),
        .mcand      ({internal_mcands[64*(`MULT_STAGES-2)-1:0], mcand}),
        .product_sum(internal_sums),
        .next_mplier(internal_mpliers),
        .next_mcand (internal_mcands),
        .next_func  (internal_funcs),
        .meta_in    ({internal_meta[`MULT_STAGES-3:0], meta_in}),
        .meta_out   (internal_meta),
        .done       (dones[`MULT_STAGES-2:0])
    );

    // ==========================================================================
    // Final pipeline stage (MULT_STAGES-1) - gated by grant
    // ==========================================================================
    mult_stage mstage_final (
        .clock      (clock),
        .reset      (reset),
        .func       (final_func),
        .start      (final_start),
        .prev_sum   (final_prev_sum),
        .mplier     (final_mplier),
        .mcand      (final_mcand),
        .product_sum(product),
        .next_mplier(mplier_out),
        .next_mcand (mcand_out),
        .next_func  (func_out),
        .meta_in    (final_meta),
        .meta_out   (meta_out_pipe),
        .done       (dones[`MULT_STAGES-1])
    );

    // Sign-extend the multiplier inputs based on the operation
    always_comb begin
        case (func)
            MUL, MULH, MULHSU: mcand = {{(32) {rs1[31]}}, rs1};
            default:           mcand = {32'b0, rs1};
        endcase
        case (func)
            MUL, MULH: mplier = {{(32) {rs2[31]}}, rs2};
            default:   mplier = {32'b0, rs2};
        endcase
    end

    // ==========================================================================
    // Request stage holding logic
    // ==========================================================================
    always_ff @(posedge clock) begin
        if (reset) begin
            req_pending <= 1'b0;
            sum_held <= '0;
            mcand_held <= '0;
            mplier_held <= '0;
            func_held <= MUL;
            meta_held <= '0;
        end else if (req_pending && grant) begin
            // Grant received while pending, clear pending state
            req_pending <= 1'b0;
        end else if (dones[`MULT_STAGES-2] && !grant) begin
            // Request stage completed but no grant, capture and hold
            req_pending <= 1'b1;
            sum_held <= internal_sums[64*(`MULT_STAGES-1)-1 -: 64];
            mcand_held <= internal_mcands[64*(`MULT_STAGES-1)-1 -: 64];
            mplier_held <= internal_mpliers[64*(`MULT_STAGES-1)-1 -: 64];
            func_held <= internal_funcs[`MULT_STAGES-2];
            meta_held <= internal_meta[`MULT_STAGES-2];
        end
        // If dones[MULT_STAGES-2] && grant: don't capture, instruction proceeds
    end

    // ==========================================================================
    // Final stage input muxing - only proceed when granted
    // ==========================================================================
    always_comb begin
        if (req_pending && grant) begin
            // Use held values from captured request stage
            final_prev_sum = sum_held;
            final_mcand = mcand_held;
            final_mplier = mplier_held;
            final_func = func_held;
            final_meta = meta_held;
            final_start = 1'b1;
        end else if (dones[`MULT_STAGES-2] && grant) begin
            // Fresh request granted, use current pipeline values
            final_prev_sum = internal_sums[64*(`MULT_STAGES-1)-1 -: 64];
            final_mcand = internal_mcands[64*(`MULT_STAGES-1)-1 -: 64];
            final_mplier = internal_mpliers[64*(`MULT_STAGES-1)-1 -: 64];
            final_func = internal_funcs[`MULT_STAGES-2];
            final_meta = internal_meta[`MULT_STAGES-2];
            final_start = 1'b1;
        end else begin
            // No grant, don't start final stage
            final_prev_sum = '0;
            final_mcand = '0;
            final_mplier = '0;
            final_func = MUL;
            final_meta = '0;
            final_start = 1'b0;
        end
    end

    // ==========================================================================
    // Outputs
    // ==========================================================================
    
    // Result from final stage
    assign result = (func_out == MUL) ? product[31:0] : product[63:32];

    // Metadata from final stage
    assign meta_out = meta_out_pipe;

    // Request CDB when request stage completes OR we have a pending request
    assign request = dones[`MULT_STAGES-2] || req_pending;

    // Done when final stage completes (only happens after grant)
    assign done = dones[`MULT_STAGES-1];

    // ============================================================
    // Multiplier Pipeline Debug Display
    // ============================================================
    always_ff @(posedge clock) begin
        if (!reset) begin
            $display("========================================");
            $display("=== MULT PIPELINE STATE (Cycle %0t) ===", $time);
            $display("========================================");
            
            // Inputs
            $display("--- Inputs ---");
            $display("  start=%b grant=%b rs1=%h rs2=%h func=%s", start, grant, rs1, rs2, func);
            $display("  mcand=%h mplier=%h", mcand, mplier);
            
            // Early pipeline stages (0 to MULT_STAGES-2)
            $display("--- Early Pipeline Stages (0 to %0d) ---", `MULT_STAGES-2);
            $display("  [0] (input)  mcand=%h mplier=%h func=%s done=%b", mcand, mplier, func, dones[0]);
            
            for (int i = 0; i < `MULT_STAGES-1; i++) begin
                $display("  [%0d] sum=%h mcand=%h mplier=%h func=%s done=%b meta=%p", 
                         i+1, 
                         internal_sums[64*i +: 64],
                         internal_mcands[64*i +: 64],
                         internal_mpliers[64*i +: 64],
                         internal_funcs[i],
                         dones[i+1],
                         internal_meta[i]);
            end
            
            // Request stage holding state
            $display("--- Request Stage Holding ---");
            $display("  req_pending=%b", req_pending);
            if (req_pending) begin
                $display("  sum_held=%h mcand_held=%h mplier_held=%h", sum_held, mcand_held, mplier_held);
                $display("  func_held=%s meta_held=%p", func_held, meta_held);
            end
            
            // Final stage inputs
            $display("--- Final Stage (Stage %0d) ---", `MULT_STAGES-1);
            $display("  final_start=%b final_func=%s", final_start, final_func);
            $display("  final_prev_sum=%h final_mcand=%h final_mplier=%h", final_prev_sum, final_mcand, final_mplier);
            $display("  product=%h func_out=%s done=%b", product, func_out, dones[`MULT_STAGES-1]);
            
            // Outputs
            $display("--- Outputs ---");
            $display("  result=%h request=%b done=%b", result, request, done);
            $display("  meta_out=%p", meta_out);
            
            $display("");
        end
    end

endmodule  // mult


module mult_stage (
    input clock,
    reset,
    start,
    input [63:0] prev_sum,
    mplier,
    mcand,
    input MULT_FUNC func,
    input EX_COMPLETE_ENTRY meta_in,

    output logic [63:0] product_sum,
    next_mplier,
    next_mcand,
    output MULT_FUNC next_func,
    output EX_COMPLETE_ENTRY meta_out,
    output logic done
);

    parameter SHIFT = 64 / `MULT_STAGES;

    logic [63:0] partial_product, shifted_mplier, shifted_mcand;

    assign partial_product = mplier[SHIFT-1:0] * mcand;

    assign shifted_mplier  = {SHIFT'('b0), mplier[63:SHIFT]};
    assign shifted_mcand   = {mcand[63-SHIFT:0], SHIFT'('b0)};

    always_ff @(posedge clock) begin
        product_sum <= prev_sum + partial_product;
        next_mplier <= shifted_mplier;
        next_mcand  <= shifted_mcand;
        next_func   <= func;
        meta_out    <= meta_in;
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            done <= 1'b0;
        end else begin
            done <= start;
        end
    end

endmodule  // mult_stage
