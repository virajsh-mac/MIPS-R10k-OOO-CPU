/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_execute.sv                                    //
//                                                                     //
//  Description :  Execute stage of the pipeline; executes up to 3     //
//                 instructions in parallel across functional units,   //
//                 computes results, handles branches (resolves        //
//                 mispredictions), and manages memory accesses via    //
//                 the D-cache (associative with victim cache).        //
//                 Demonstrates out-of-order execution by processing   //
//                 ready instructions issued from the RS, bypassing   //
//                 if necessary (though base assumes values ready at  //
//                 issue; advanced may add intra-stage bypass).        //
//                 For loads/stores, computes address and accesses     //
//                 D-cache; on miss, forwards to memory (blocking in  //
//                 base, stalls pipeline until resolved). Multi-cycle //
//                 ops (e.g., mul) are pipelined. No LSQ, so memory    //
//                 ops serialized (issue ensures <=1 mem op/cycle).   //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"
`include "ISA.svh"

// Parameters and typedefs are now centrally defined in sys_defs.svh

// Extend RS_ENTRY from dispatch for execute (add memory-specific fields)
typedef struct packed {
    RS_ENTRY base;             // Base RS entry from sys_defs.svh
    // For mem ops: size and sign (extracted from OP_TYPE.func)
    MEM_SIZE mem_size;
    logic mem_unsigned;
} EX_RS_ENTRY;

// Packet from Issue to Execute (up to N ready instructions from RS)
typedef struct packed {
    logic [`N-1:0] valid;      // Valid bits for issued bundle
    EX_RS_ENTRY [`N-1:0] entries;  // Issued RS entries (with values ready)
} ISS_EX_PACKET;

// Packet from Execute to Complete (results for CDB broadcast)
typedef struct packed {
    logic [`N-1:0] valid;      // Valid results this cycle
    DATA [`N-1:0] results;     // Computed values (ALU/mult/load result)
    PHYS_TAG [`N-1:0] dest_tags;  // Physical dest tags for broadcast
    ROB_IDX [`N-1:0] rob_idxs; // ROB indices for complete marking
} EX_COMP_PACKET;

// Mispred recovery packet (from Execute to Dispatch/Fetch for flush)
typedef struct packed {
    logic valid;               // Mispredict detected
    ROB_IDX rob_idx;           // ROB idx of mispredicted branch
    ADDR correct_target;       // Correct target (or PC+4 if not taken)
} MISPRED_RECOVERY_PACKET;

// Pending mem request info (for blocking base; holds on miss)
typedef struct packed {
    logic valid;               // Pending load/store
    MEM_TAG mem_tag;           // Outstanding mem tag
    PHYS_TAG dest_tag;         // For load: dest phys tag
    ROB_IDX rob_idx;           // For load/store: ROB idx
    ADDR addr;                 // Computed address (for extraction on response)
    MEM_SIZE mem_size;         // Byte/half/word/double
    logic mem_unsigned;        // Unsigned load?
    logic is_store;            // Store? (no result, just complete)
    DATA store_data;           // Store data (if store)
} PENDING_MEM;

module stage_execute (
    input              clock,           // system clock
    input              reset,           // system reset

    // From Issue: issued bundle
    input ISS_EX_PACKET issue_packet,
    input logic [`N-1:0] issued_count,  // Number issued this cycle (for debug)

    // From memory: response data/tag
    input MEM_BLOCK    mem2dcache_data,
    input MEM_TAG      mem2dcache_tag,

    // From Complete: CDB for potential intra-cycle bypass (advanced)
    // (Base assumes no need; values ready at issue)
    input CDB_PACKET   cdb_broadcast,

    // To Complete: computed results
    output EX_COMP_PACKET execute_packet,

    // To Dispatch/Fetch: mispredict recovery
    output MISPRED_RECOVERY_PACKET mispred_recovery,

    // To memory: via D-cache
    output MEM_COMMAND dcache2mem_command,
    output ADDR        dcache2mem_addr,
    output MEM_SIZE    dcache2mem_size,
    output DATA        dcache2mem_data,  // For stores

    // To Issue/Dispatch/Fetch: stall on blocking miss
    output logic       stall_pipeline
);

    // Internal signals
    DATA [`N-1:0] opa_values;  // Per-inst OPA (src1_value, potentially bypassed)
    DATA [`N-1:0] opb_values;  // Per-inst OPB (src2_value, potentially bypassed)
    DATA [`N-1:0] alu_results; // ALU outputs
    DATA [`N-1:0] mult_results;// Mult outputs (from pipeline)
    logic [`N-1:0] take_conds; // Conditional branch outcomes
    ADDR [`N-1:0] branch_targets; // Computed branch targets
    logic [`N-1:0] per_inst_mispred; // Per-inst mispredict flag
    ADDR mem_addr;             // Computed mem address (for single mem op)
    logic mem_is_load, mem_is_store; // Flags for mem type
    logic dcache_hit;          // D-cache hit indicator
    logic [(`DCACHE_LINE_BYTES*8)-1:0] dcache_data_out; // D-cache data

    // Pending mem (for blocking miss handling)
    PENDING_MEM pending_mem_reg, pending_mem_next;

    // D-cache outputs to mem (interface)
    MEM_COMMAND proc2Dcache_command;
    ADDR proc2Dcache_addr;
    MEM_SIZE proc2Dcache_size;
    DATA proc2Dcache_data;

    // Instantiate D-cache (interface only, full impl separate; single-ported, blocking base)
    dcache #(
        .ASSOC(`DCACHE_ASSOC),
        .LINES(`DCACHE_LINES),
        .LINE_BYTES(`DCACHE_LINE_BYTES),
        .VICTIM_SZ(`DCACHE_VICTIM_SZ)
    ) dcache_0 (
        .clock(clock),
        .reset(reset),
        .proc2Dcache_command(proc2Dcache_command),  // LOAD/STORE/NONE
        .proc2Dcache_addr(proc2Dcache_addr),        // Address
        .proc2Dcache_size(proc2Dcache_size),        // BYTE/HALF/WORD/DOUBLE
        .proc2Dcache_data(proc2Dcache_data),        // Store data
        .Dmem2proc_data(mem2dcache_data),           // From main mem
        .Dmem2proc_tag(mem2dcache_tag),             // Response tag

        .dcache2mem_command(dcache2mem_command),    // To main mem
        .dcache2mem_addr(dcache2mem_addr),
        .dcache2mem_size(dcache2mem_size),
        .dcache2mem_data(dcache2mem_data),          // For write-through/allocate
        .dcache_data_out(dcache_data_out),          // Hit data
        .dcache_hit(dcache_hit)                     // Hit/miss
    );

    // Instantiate ALUs (combinational; one per potential inst)
    generate
        for (genvar i = 0; i < `N; i++) begin : alu_gen
            alu alu_i (
                .opa(opa_values[i]),
                .opb(opb_values[i]),
                .alu_func(issue_packet.entries[i].base.op_type.func),

                .result(alu_results[i])
            );
        end
    endgenerate

    // Instantiate pipelined multiplier (clocked; assume fully pipelined for throughput 1/cycle)
    // (Interface only; full impl would add regs between stages)
    mult_pipelined #(.STAGES(4)) mult_0 (  // Example 4-stage pipeline
        .clock(clock),
        .reset(reset),
        .start(mult_start),  // From comb logic
        .rs1(mult_opa),
        .rs2(mult_opb),
        .func(mult_func),    // From inst funct3

        .result(mult_results[0]),  // To comb (assume single mult)
        .done(mult_done)
    );

    // Instantiate conditional branch units (combinational)
    generate
        for (genvar i = 0; i < `N; i++) begin : branch_gen
            if (issue_packet.entries[i].cond_branch) begin
                conditional_branch cond_branch_i (
                    .rs1(opa_values[i]),
                    .rs2(opb_values[i]),
                    .func(issue_packet.entries[i].inst.b.funct3),  // Assume inst available or extracted

                    .take(take_conds[i])
                );
            end
        end
    endgenerate

    // Stall logic (blocking on mem miss; advanced non-blocking would remove)
    assign stall_pipeline = pending_mem_reg.valid;

    // Pending mem FF
    always_ff @(posedge clock) begin
        if (reset) begin
            pending_mem_reg <= '0;
        end else begin
            pending_mem_reg <= pending_mem_next;
        end
    end

    // Main execute logic (parallel for FUs)
    always_comb begin
        // Defaults
        execute_packet = '0;
        mispred_recovery = '0;
        per_inst_mispred = '0;
        opa_values = '0;
        opb_values = '0;
        proc2Dcache_command = MEM_NONE;
        proc2Dcache_addr = '0;
        proc2Dcache_size = WORD;  // Default
        proc2Dcache_data = '0;
        mem_addr = '0;
        mem_is_load = '0;
        mem_is_store = '0;
        pending_mem_next = pending_mem_reg;
        mult_start = '0;
        mult_opa = '0;
        mult_opb = '0;
        mult_func = M_MUL;  // Default

        // Handle pending mem response first (priority over new insts for blocking)
        if (pending_mem_reg.valid && mem2dcache_tag == pending_mem_reg.mem_tag && mem2dcache_tag != 0) begin
            if (!pending_mem_reg.is_store) begin  // Load: extract data
                DATA load_result;
                case (pending_mem_reg.mem_size)
                    BYTE:   load_result = {{24{pending_mem_reg.mem_unsigned ? 1'b0 : mem2dcache_data.byte_level[pending_mem_reg.addr[2:0]] [7]}}, mem2dcache_data.byte_level[pending_mem_reg.addr[2:0]]};
                    HALF:   load_result = {{16{pending_mem_reg.mem_unsigned ? 1'b0 : mem2dcache_data.half_level[pending_mem_reg.addr[2:1]] [15]}}, mem2dcache_data.half_level[pending_mem_reg.addr[2:1]]};
                    WORD:   load_result = mem2dcache_data.word_level[pending_mem_reg.addr[2]];
                    default: load_result = '0;
                endcase
                execute_packet.valid[0] = 1'b1;
                execute_packet.results[0] = load_result;
                execute_packet.dest_tags[0] = pending_mem_reg.dest_tag;
                execute_packet.rob_idxs[0] = pending_mem_reg.rob_idx;
            end else begin  // Store: just complete
                execute_packet.valid[0] = 1'b1;
                execute_packet.results[0] = '0;  // No value
                execute_packet.dest_tags[0] = pending_mem_reg.dest_tag;  // May be 0 for no dest
                execute_packet.rob_idxs[0] = pending_mem_reg.rob_idx;
            end
            pending_mem_next.valid = 1'b0;  // Clear pending
        end

        // Process new issued insts (if not stalled)
        if (!stall_pipeline) begin
            integer fu_idx = 0;  // Track assigned FUs
            mem_is_load = 0;
            mem_is_store = 0;
            for (int i = 0; i < `N; i++) begin
                if (issue_packet.valid[i]) begin
                    // Potential bypass from concurrent CDB (advanced; base skips)
                    opa_values[i] = issue_packet.entries[i].base.src1_value;
                    opb_values[i] = issue_packet.entries[i].base.src2_value;
                    // Check for CDB match (if tag not ready somehow; rare)
                    for (int c = 0; c < `CDB_SZ; c++) begin
                        if (cdb_broadcast.valid[c]) begin
                            if (cdb_broadcast.tags[c] == issue_packet.entries[i].base.src1_tag) opa_values[i] = cdb_broadcast.values[c];
                            if (cdb_broadcast.tags[c] == issue_packet.entries[i].base.src2_tag) opb_values[i] = cdb_broadcast.values[c];
                        end
                    end

                    // Route to FU based on control
                    if (issue_packet.entries[i].base.op_type.category == CAT_MULT) begin
                        // Start mult (assume available; issue checks)
                        mult_start = 1'b1;
                        mult_opa = opa_values[i];
                        mult_opb = opb_values[i];
                        mult_func = issue_packet.entries[i].base.op_type.func;  // Use OP_TYPE func
                        // If done this cycle (placeholder; multi-cycle outputs later)
                        if (mult_done) begin
                            execute_packet.valid[fu_idx] = 1'b1;
                            execute_packet.results[fu_idx] = mult_results[0];
                            execute_packet.dest_tags[fu_idx] = issue_packet.entries[i].base.dest_tag;
                            execute_packet.rob_idxs[fu_idx] = issue_packet.entries[i].base.rob_idx;
                            fu_idx++;
                        end  // Else, handled in pipeline output to execute_packet
                    end else if (issue_packet.entries[i].base.op_type.category == CAT_BRANCH) begin
                        // Compute branch outcome/target
                        logic take = (issue_packet.entries[i].base.op_type == OP_JAL || issue_packet.entries[i].base.op_type == OP_JALR) ? 1'b1 : take_conds[i];
                        ADDR target;
                        if (issue_packet.entries[i].base.op_type == OP_JALR) begin
                            target = (opa_values[i] + opb_values[i]) & ~32'h1;  // Clear LSB
                        end else begin
                            target = issue_packet.entries[i].base.PC + opb_values[i];  // IMM
                        end
                        branch_targets[i] = target;
                        per_inst_mispred[i] = (take != issue_packet.entries[i].base.pred_taken) ||
                                              (take && target != issue_packet.entries[i].base.pred_target);
                        if (per_inst_mispred[i]) begin
                            mispred_recovery.valid = 1'b1;
                            mispred_recovery.rob_idx = issue_packet.entries[i].base.rob_idx;
                            mispred_recovery.correct_target = take ? target : issue_packet.entries[i].base.PC + 4;
                        end
                        // Branch has no dest value unless JAL/JALR (rd link)
                        if (issue_packet.entries[i].base.op_type == OP_JAL || issue_packet.entries[i].base.op_type == OP_JALR) begin  // Link reg
                            execute_packet.valid[fu_idx] = 1'b1;
                            execute_packet.results[fu_idx] = issue_packet.entries[i].base.PC + 4;
                            execute_packet.dest_tags[fu_idx] = issue_packet.entries[i].base.dest_tag;
                            execute_packet.rob_idxs[fu_idx] = issue_packet.entries[i].base.rob_idx;
                            fu_idx++;
                        end else begin
                            // Just complete (no value)
                            execute_packet.valid[fu_idx] = 1'b1;
                            execute_packet.results[fu_idx] = '0;
                            execute_packet.dest_tags[fu_idx] = '0;
                            execute_packet.rob_idxs[fu_idx] = issue_packet.entries[i].base.rob_idx;
                            fu_idx++;
                        end
                    end else if (issue_packet.entries[i].base.op_type.category == CAT_MEM) begin
                        // Mem op (assume only one; issue ensures)
                        mem_addr = opa_values[i] + opb_values[i];
                        proc2Dcache_addr = mem_addr;
                        proc2Dcache_size = issue_packet.entries[i].mem_size;
                        mem_is_load = (issue_packet.entries[i].base.op_type.category == CAT_MEM && issue_packet.entries[i].base.op_type.func[3] == 0);
                        mem_is_store = (issue_packet.entries[i].base.op_type.category == CAT_MEM && issue_packet.entries[i].base.op_type.func[3] == 1);
                        if (mem_is_store) begin
                            proc2Dcache_command = MEM_STORE;
                            proc2Dcache_data = opb_values[i];  // rs2_value
                        end else begin
                            proc2Dcache_command = MEM_LOAD;
                        end
                        if (dcache_hit) begin
                            if (mem_is_load) begin
                                // Extract load data
                                DATA load_result;
                                case (proc2Dcache_size)
                                    BYTE:   load_result = {{24{issue_packet.entries[i].mem_unsigned ? 1'b0 : dcache_data_out[7:0]}}, dcache_data_out[7:0]};
                                    HALF:   load_result = {{16{issue_packet.entries[i].mem_unsigned ? 1'b0 : dcache_data_out[15:0] [15]}}, dcache_data_out[15:0]};
                                    WORD:   load_result = dcache_data_out[31:0];
                                    default: load_result = '0;
                                endcase
                                execute_packet.valid[fu_idx] = 1'b1;
                                execute_packet.results[fu_idx] = load_result;
                                execute_packet.dest_tags[fu_idx] = issue_packet.entries[i].base.dest_tag;
                                execute_packet.rob_idxs[fu_idx] = issue_packet.entries[i].base.rob_idx;
                                fu_idx++;
                            end else begin  // Store complete
                                execute_packet.valid[fu_idx] = 1'b1;
                                execute_packet.results[fu_idx] = '0;
                                execute_packet.dest_tags[fu_idx] = '0;
                                execute_packet.rob_idxs[fu_idx] = issue_packet.entries[i].base.rob_idx;
                                fu_idx++;
                            end
                        end else begin
                            // Miss: set pending (blocking stall)
                            pending_mem_next.valid = 1'b1;
                            pending_mem_next.mem_tag = 1;  // Placeholder; actual from dcache request
                            pending_mem_next.dest_tag = issue_packet.entries[i].base.dest_tag;
                            pending_mem_next.rob_idx = issue_packet.entries[i].base.rob_idx;
                            pending_mem_next.addr = mem_addr;
                            pending_mem_next.mem_size = proc2Dcache_size;
                            pending_mem_next.mem_unsigned = issue_packet.entries[i].mem_unsigned;
                            pending_mem_next.is_store = mem_is_store;
                            pending_mem_next.store_data = opb_values[i];
                        end
                    end else begin
                        // Standard ALU op
                        execute_packet.valid[fu_idx] = 1'b1;
                        execute_packet.results[fu_idx] = alu_results[i];
                        execute_packet.dest_tags[fu_idx] = issue_packet.entries[i].base.dest_tag;
                        execute_packet.rob_idxs[fu_idx] = issue_packet.entries[i].base.rob_idx;
                        fu_idx++;
                    end
                end
            end
        end
    end

endmodule // stage_execute