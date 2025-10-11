/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_fetch.sv                                      //
//                                                                     //
//  Description :  Fetch stage of the pipeline; fetches up to 3        //
//                 instructions per cycle from the I-cache,            //
//                 integrates branch prediction and prefetching.       //
//                 Handles redirects from branch mispredictions.       //
//                 Demonstrates out-of-order capability by allowing    //
//                 fetch beyond predicted branches (speculatively).    //
//                 Includes partial decode to extract arch regs and    //
//                 control signals for dispatch.                       //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"
`include "ISA.svh"

// Parameters are now centrally defined in sys_defs.svh

// Packet from Fetch to Dispatch (includes partial decode)
typedef struct packed {
    logic [`N-1:0] valid;  // Valid bits for each instruction in the bundle

    INST [`N-1:0] inst;    // Up to 3 instructions
    ADDR [`N-1:0] PC;      // PC for each instruction

    // Decoded architectural register indices
    REG_IDX [`N-1:0] rs1_idx;
    REG_IDX [`N-1:0] rs2_idx;
    REG_IDX [`N-1:0] rd_idx;

    // Flags indicating if registers are used
    logic [`N-1:0] uses_rs1;
    logic [`N-1:0] uses_rs2;
    logic [`N-1:0] uses_rd;

    // Control signals from decoder (for dispatch to use in FU selection, etc.)
    ALU_OPA_SELECT [`N-1:0] opa_select;
    ALU_OPB_SELECT [`N-1:0] opb_select;
    OP_TYPE [`N-1:0] op_type;  // Consolidated operation type
    logic [`N-1:0] csr_op;
    logic [`N-1:0] halt;
    logic [`N-1:0] illegal;
    
    // Branch prediction fields (from branch predictor)
    logic [`N-1:0] pred_taken;
    ADDR [`N-1:0] pred_target;
} FETCH_DISP_PACKET;

// Packet for branch predictor update (from Retire/Execute)
typedef struct packed {
    logic valid;       // Update valid
    ADDR pc;           // PC of the branch
    logic taken;       // Actual taken/not taken
    ADDR target;       // Actual target address
} BP_UPDATE_PACKET;

module stage_fetch (
    input              clock,           // system clock
    input              reset,           // system reset

    // Misprediction handling (from Execute/Complete/Retire)
    input logic        mispredict,      // Signal to flush and redirect
    input ADDR         branch_target,   // Correct branch target on mispredict

    // Dispatch feedback
    input logic [`N-1:0] dispatch_ready,  // Per-instruction ready signals from dispatch (space in ROB/RS)

    // Memory interface (shared with I-cache)
    input MEM_BLOCK    mem2icache_data,   // Data from main memory
    input MEM_TAG      mem2icache_transaction_tag,  // Transaction tag for response
    input MEM_TAG      mem2icache_data_tag,         // Data tag

    // Branch predictor update (from Retire)
    input BP_UPDATE_PACKET bp_update,     // Update packet for BTB and predictor

    // Outputs to memory (via I-cache)
    output MEM_COMMAND icache2mem_command,  // Command to main memory
    output ADDR        icache2mem_addr,     // Address to main memory
    output MEM_SIZE    icache2mem_size,     // Size (though for cache lines, typically DOUBLE)

    // Output to Dispatch
    output FETCH_DISP_PACKET fetch_packet,  // Bundle of up to 3 decoded instructions
    output logic       fetch_valid,         // Overall valid signal for the bundle

    // Prefetch trigger (for next predicted line)
    output logic       prefetch_request,    // Signal to request prefetch
    output ADDR        prefetch_addr        // Predicted next cache line address
);

    // Internal signals
    ADDR PC_reg;  // Current PC (word-aligned)
    logic stall;  // Stall fetch due to dispatch not ready or cache miss

    // I-cache outputs
    logic [(`ICACHE_LINE_BYTES*8)-1:0] icache_data_out;  // Wide data out (e.g., 128 bits for 16 bytes)
    logic icache_valid_out;  // Hit/miss indicator
    logic icache_prefetch_ready;  // Prefetch buffer status

    // Extracted instructions (up to 4 from line, but fetch max 3)
    INST [3:0] raw_insts;
    logic [3:0] raw_valids;

    // Decoded bundle (before branch handling)
    logic [`N-1:0] pre_branch_valid;
    ADDR [`N-1:0] inst_pcs;
    logic first_taken_idx;  // Index of first predicted taken branch (0-2), or 3 if none

    // Predicted next PC
    ADDR next_pc;

    // Branch prediction outputs (per potential instruction)
    logic [`N-1:0] pred_taken;
    ADDR [`N-1:0] pred_targets;

    // Instantiate I-cache (interface only, full impl separate)
    icache #(
        .ASSOC(`ICACHE_ASSOC),
        .LINES(`ICACHE_LINES),
        .LINE_BYTES(`ICACHE_LINE_BYTES),
        .VICTIM_SZ(`VICTIM_CACHE_SZ)
    ) icache_0 (
        .clock(clock),
        .reset(reset),
        .proc2Icache_addr(PC_reg),  // Fetch starting at PC
        .Imem2proc_transaction_tag(mem2icache_transaction_tag),
        .Imem2proc_data(mem2icache_data),
        .Imem2proc_data_tag(mem2icache_data_tag),
        .prefetch_addr(prefetch_addr),  // For prefetching next line
        .prefetch_request(prefetch_request),

        .proc2Imem_command(icache2mem_command),
        .proc2Imem_addr(icache2mem_addr),
        .proc2Imem_size(icache2mem_size),  // Typically DOUBLE for 64-bit, but adjust for line size
        .Icache_data_out(icache_data_out),  // Wide output
        .Icache_valid_out(icache_valid_out),
        .Icache_prefetch_ready(icache_prefetch_ready)  // Can accept prefetch
    );

    // Instantiate Branch Predictor (BTB + bimodal, interface only)
    branch_predictor #(
        .PRED_SZ(`BRANCH_PRED_SZ)
    ) bp_0 (
        .clock(clock),
        .reset(reset),
        .query_pcs(inst_pcs),  // PCs of the fetched instructions
        .update(bp_update),    // Update from retire

        .pred_taken(pred_taken),    // Predicted direction per inst
        .pred_targets(pred_targets) // Predicted targets per inst
    );

    // PC update logic
    always_ff @(posedge clock) begin
        if (reset) begin
            PC_reg <= 0;
        end else if (mispredict) begin
            PC_reg <= branch_target;  // Redirect on mispredict
        end else if (!stall) begin
            PC_reg <= next_pc;  // Predicted or sequential next PC
        end
    end

    // Alignment and extraction logic (extract up to 3 insts from wide cache data)
    always_comb begin
        // Assume PC[1:0]==00 (aligned), extract based on PC[3:2] (offset in line)
        // For 16-byte line: 4 insts possible
        raw_insts = '{default: `NOP};  // Default to NOP
        raw_valids = 4'b0;
        // TODO: Shift/extract logic based on PC offset, e.g.:
        // raw_insts[0] = icache_data_out[31:0] if offset 0, etc.
        // Limit to max 3: if offset allows only 2, raw_valids[2]=0;
    end

    // Assign inst PCs: sequential PC, PC+4, PC+8
    always_comb begin
        inst_pcs[0] = PC_reg;
        inst_pcs[1] = PC_reg + 4;
        inst_pcs[2] = PC_reg + 8;
    end

    // Partial decode for each potential inst (instantiate 3 decoders)
    generate
        for (genvar i = 0; i < `N; i++) begin : decode_gen
            decoder decoder_i (
                .inst(raw_insts[i]),
                .valid(raw_valids[i]),

                .opa_select(fetch_packet.opa_select[i]),
                .opb_select(fetch_packet.opb_select[i]),
                .has_dest(fetch_packet.uses_rd[i]),
                .op_type(fetch_packet.op_type[i]),
                .csr_op(fetch_packet.csr_op[i]),
                .halt(fetch_packet.halt[i]),
                .illegal(fetch_packet.illegal[i])
            );

            // Extract arch reg indices (as in stage_id.sv)
            assign fetch_packet.rs1_idx[i] = raw_insts[i].r.rs1;
            assign fetch_packet.rs2_idx[i] = raw_insts[i].r.rs2;
            assign fetch_packet.rd_idx[i]  = (fetch_packet.uses_rd[i]) ? raw_insts[i].r.rd : `ZERO_REG;
            assign fetch_packet.uses_rs1[i] = (fetch_packet.opa_select[i] == OPA_IS_RS1);
            assign fetch_packet.uses_rs2[i] = (fetch_packet.opb_select[i] == OPB_IS_RS2);

            // Assign inst and PC
            assign fetch_packet.inst[i] = raw_insts[i];
            assign fetch_packet.PC[i]   = inst_pcs[i];
            
            // Assign branch prediction fields
            assign fetch_packet.pred_taken[i] = pred_taken[i];
            assign fetch_packet.pred_target[i] = pred_targets[i];
        end
    endgenerate

    // Branch handling: find first predicted taken branch, invalidate after it
    always_comb begin
        first_taken_idx = 3;  // None
        for (int i = 0; i < `N; i++) begin
            if (raw_valids[i] && (fetch_packet.op_type[i].category == CAT_BRANCH) &&
                pred_taken[i]) begin
                first_taken_idx = i;
                break;
            end
        end

        // Set valids: all before first taken, invalidate after
        for (int i = 0; i < `N; i++) begin
            pre_branch_valid[i] = (i <= first_taken_idx) && raw_valids[i];
        end
    end

    // Next PC calculation
    always_comb begin
        if (first_taken_idx < 3) begin
            // For JALR (uncond_branch and opa_select==OPA_IS_RS1), target prediction may be inaccurate; assume BTB handles
            next_pc = pred_targets[first_taken_idx];
        end else begin
            next_pc = PC_reg + (4 * `N);  // Sequential +12
        end
    end

    // Prefetch logic: predict next line after next_pc
    always_comb begin
        prefetch_addr = {next_pc[31:$clog2(`ICACHE_LINE_BYTES)], {($clog2(`ICACHE_LINE_BYTES)){1'b0}}} + `ICACHE_LINE_BYTES;
        prefetch_request = icache_prefetch_ready && !stall;  // Simple next-line prefetch if buffer ready
    end

    // Stall logic
    assign stall = !icache_valid_out || (|dispatch_ready == 0);  // Miss or no space in dispatch (simplified; could check partial)

    // Final packet valids and overall valid
    assign fetch_packet.valid = pre_branch_valid & {`N{!stall}};
    assign fetch_valid = |fetch_packet.valid;

endmodule // stage_fetch