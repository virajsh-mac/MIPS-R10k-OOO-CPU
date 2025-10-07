/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_dispatch.sv                                   //
//                                                                     //
//  Description :  Dispatch stage of the pipeline; handles register    //
//                 renaming, allocates entries in the ROB and RS,      //
//                 checks for structural hazards (space in ROB/RS/     //
//                 free list). Inserts instructions into RS with       //
//                 priority for oldest (using compacting shifter or    //
//                 priority encoder from top). Demonstrates OoO by     //
//                 allowing dispatch of non-dependent instructions.    //
//                 Builds on partial decode from Fetch.                //
//                 Supports recovery on branch mispredictions via      //
//                 map table checkpoints or ROB walkback (simplified). //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"
`include "ISA.svh"

// Parameters and typedefs are now centrally defined in sys_defs.svh

// Map table entry: maps arch reg to phys reg
typedef struct packed {
    PHYS_TAG phys_tag;  // Current physical register mapping
} MAP_ENTRY;

// ROB entry is now defined in sys_defs.svh

// RS entry is now defined in sys_defs.svh

// Packet from Dispatch to Issue (minimal, since Issue reads from RS directly; this could signal new entries)
typedef struct packed {
    logic [`N-1:0] valid;      // Valid bits for dispatched bundle
    RS_IDX [`N-1:0] rs_idxs;   // Indices of newly allocated RS entries
} DISP_ISS_PACKET;

// Packet for mispredict recovery (from Execute/Complete to Dispatch for map/free recovery)
typedef struct packed {
    logic valid;               // Mispredict occurred
    ROB_IDX rob_idx;           // ROB index of mispredicted branch (to truncate from)
    ADDR correct_target;       // Correct target for fetch redirect
} MISPRED_RECOVERY_PACKET;

// Packet from Retire to Dispatch (for committed map updates and free list additions)
typedef struct packed {
    logic [`N-1:0] valid;      // Valid commits this cycle
    REG_IDX [`N-1:0] arch_rds; // Architectural destinations
    PHYS_TAG [`N-1:0] phys_rds; // Committed physical regs
    PHYS_TAG [`N-1:0] prev_phys_rds; // Previous phys to free
} RETIRE_DISP_PACKET;

module stage_dispatch (
    input              clock,           // system clock
    input              reset,           // system reset

    // From Fetch: partially decoded bundle
    input FETCH_DISP_PACKET fetch_packet,
    input logic        fetch_valid,     // Overall valid for bundle

    // From Retire: committed updates for map table and free list
    input RETIRE_DISP_PACKET retire_packet,

    // From Complete: CDB for wake-up (though RS handles wake-up, Dispatch may use for values during rename)
    input CDB_PACKET   cdb_broadcast,

    // From Execute/Complete: mispredict for recovery
    input MISPRED_RECOVERY_PACKET mispred_recovery,

    // To Fetch: stall signal if no space
    output logic       stall_fetch,

    // To Issue: signal new dispatches (Issue selects from RS)
    output DISP_ISS_PACKET dispatch_packet,
    output logic       dispatch_valid,

    // To ROB: allocation signals (interface; full ROB module separate)
    output logic [`N-1:0] rob_alloc_valid,  // Allocate these
    output ROB_ENTRY [`N-1:0] rob_alloc_entries,  // Data to write
    output ROB_IDX   rob_tail_update,     // New tail after alloc

    // To RS: allocation signals (interface; full RS module separate)
    output logic [`N-1:0] rs_alloc_valid,   // Allocate these
    output RS_ENTRY [`N-1:0] rs_alloc_entries,  // Data to write
    output logic     rs_compact,          // Signal to compact/shift for oldest-first

    // To Free List: allocations and frees (interface; full free list module separate)
    output logic [`N-1:0] free_alloc_valid, // Request new phys regs
    input PHYS_TAG [`N-1:0] allocated_phys, // Granted phys tags from free list
    output logic [`N-1:0] free_add_valid,   // Add freed phys (from retire)
    output PHYS_TAG [`N-1:0] freed_phys     // Phys to add back
);

    // Internal structures
    MAP_ENTRY [31:0] map_table;          // Arch reg -> phys tag (32 entries)
    MAP_ENTRY [31:0] checkpoint_map;     // Checkpoint for branches (simplified: one checkpoint)
    logic            checkpoint_valid;   // Active checkpoint?

    // ROB interface signals (assume ROB is separate module, here we generate writes)
    ROB_IDX rob_head, rob_tail;          // Maintained here or from ROB module
    logic [`ROB_IDX_BITS:0] rob_free_count;  // Available entries (ROB_SZ - occupied)

    // RS interface signals
    logic [`RS_IDX_BITS:0] rs_free_count;    // Available entries

    // Free list interface (assume FIFO-like with head/tail)
    logic free_list_empty;               // No free phys regs

    // Internal signals for dispatch bundle
    logic [`N-1:0] disp_valid;           // Per-inst valid after checks
    PHYS_TAG [`N-1:0] rs1_phys;          // Renamed rs1
    PHYS_TAG [`N-1:0] rs2_phys;          // Renamed rs2
    PHYS_TAG [`N-1:0] rd_phys;           // New phys for rd
    PHYS_TAG [`N-1:0] prev_rd_phys;      // Prev mapping for ROB
    logic [`N-1:0] src1_ready;           // Src1 ready at dispatch (from map/phys reg file?)
    logic [`N-1:0] src2_ready;           // Src2 ready
    DATA [`N-1:0] src1_value;            // If ready, value from phys reg file
    DATA [`N-1:0] src2_value;            // If ready

    // Phys reg file interface (separate module; for values if ready)
    // Assume read ports: up to 2*`N for srcs
    output logic [2*`N-1:0] prf_read_valid;
    output PHYS_TAG [2*`N-1:0] prf_read_tags;
    input DATA [2*`N-1:0] prf_read_values;

    // Instantiate submodules (interfaces only, no full impl)
    // ROB module interface (would be instantiated here if full)
    // rob #(.SZ(`ROB_SZ)) rob_0 ( .clock, .reset, .alloc_valid(rob_alloc_valid), .alloc_entries(rob_alloc_entries), ... );

    // RS module interface (compacting array or priority select)
    // rs #(.SZ(`RS_SZ)) rs_0 ( .clock, .reset, .alloc_valid(rs_alloc_valid), .alloc_entries(rs_alloc_entries), .compact(rs_compact), ... );

    // Free list module interface (FIFO of free phys tags)
    // free_list #(.SZ(`PHYS_REG_SZ_R10K - 32)) fl_0 ( .clock, .reset, .alloc_req(free_alloc_valid), .alloc_grant(allocated_phys), .free_add(free_add_valid), .free_phys(freed_phys), .empty(free_list_empty), ... );

    // Phys reg file interface (for reading ready values during dispatch)
    // phys_reg_file #(.SZ(`PHYS_REG_SZ_R10K)) prf_0 ( .clock, .reset, .read_valid(prf_read_valid), .read_tags(prf_read_tags), .read_data(prf_read_values), ... );

    // Stall logic: insufficient space for ALL `N insts (atomic dispatch)
    always_comb begin
        stall_fetch = fetch_valid && (rob_free_count < `N || rs_free_count < `N || |free_alloc_valid && free_list_empty);
    end

    // Dispatch logic (parallel for rename, sequential for allocation)
    always_comb begin
        // Default outputs
        disp_valid = fetch_packet.valid & {`N{!stall_fetch}};
        dispatch_valid = |disp_valid;
        rob_alloc_valid = '0;
        rs_alloc_valid = '0;
        free_alloc_valid = '0;
        free_add_valid = '0;
        prf_read_valid = '0;

        // Parallel: Rename srcs/dest for each inst
        for (int i = 0; i < `N; i++) begin
            if (disp_valid[i]) begin
                // Lookup srcs
                rs1_phys[i] = map_table[fetch_packet.rs1_idx[i]].phys_tag;
                rs2_phys[i] = map_table[fetch_packet.rs2_idx[i]].phys_tag;
                prev_rd_phys[i] = map_table[fetch_packet.rd_idx[i]].phys_tag;

                // Request new phys for rd if uses_rd
                if (fetch_packet.uses_rd[i]) begin
                    free_alloc_valid[i] = 1'b1;
                    rd_phys[i] = allocated_phys[i];  // Assume granted instantly (combo)
                end else begin
                    rd_phys[i] = '0;  // No dest
                end

                // Check readiness and read values if ready (assume busy table or from CDB)
                // For simplicity: assume ready if not waiting on tag (but actual: check if producer complete)
                // Here, placeholder: read from PRF always, set ready if value avail (but need busy bits)
                prf_read_valid[2*i] = fetch_packet.uses_rs1[i];
                prf_read_tags[2*i] = rs1_phys[i];
                src1_value[i] = prf_read_values[2*i];
                src1_ready[i] = 1'b1;  // Placeholder; actual: if !busy[rs1_phys[i]]

                prf_read_valid[2*i+1] = fetch_packet.uses_rs2[i];
                prf_read_tags[2*i+1] = rs2_phys[i];
                src2_value[i] = prf_read_values[2*i+1];
                src2_ready[i] = 1'b1;  // Placeholder
            end
        end

        // Sequential/atomic: if no stall, allocate all
        if (!stall_fetch && fetch_valid) begin
            ROB_IDX new_tail = rob_tail;
            for (int i = 0; i < `N; i++) begin
                if (disp_valid[i]) begin
                    // Allocate ROB
                    rob_alloc_valid[i] = 1'b1;
                    rob_alloc_entries[i].valid = 1'b1;
                    rob_alloc_entries[i].PC = fetch_packet.PC[i];
                    rob_alloc_entries[i].inst = fetch_packet.inst[i];
                    rob_alloc_entries[i].arch_rd = fetch_packet.rd_idx[i];
                    rob_alloc_entries[i].phys_rd = rd_phys[i];
                    rob_alloc_entries[i].prev_phys_rd = prev_rd_phys[i];
                    rob_alloc_entries[i].complete = 1'b0;
                    rob_alloc_entries[i].exception = NO_ERROR;
                    rob_alloc_entries[i].branch = (fetch_packet.op_type[i].category == CAT_BRANCH);

                    // Allocate RS (priority insert from top for oldest)
                    rs_alloc_valid[i] = 1'b1;
                    rs_alloc_entries[i].valid = 1'b1;
                    rs_alloc_entries[i].opa_select = fetch_packet.opa_select[i];
                    rs_alloc_entries[i].opb_select = fetch_packet.opb_select[i];
                    rs_alloc_entries[i].op_type = fetch_packet.op_type[i];
                    rs_alloc_entries[i].src1_tag = rs1_phys[i];
                    rs_alloc_entries[i].src1_ready = src1_ready[i];
                    rs_alloc_entries[i].src1_value = src1_value[i];
                    rs_alloc_entries[i].src2_tag = rs2_phys[i];
                    rs_alloc_entries[i].src2_ready = src2_ready[i];
                    rs_alloc_entries[i].src2_value = src2_value[i];
                    rs_alloc_entries[i].dest_tag = rd_phys[i];
                    rs_alloc_entries[i].rob_idx = new_tail;  // Current tail as idx
                    rs_alloc_entries[i].PC = fetch_packet.PC[i];
                    rs_alloc_entries[i].pred_taken = fetch_packet.pred_taken[i];
                    rs_alloc_entries[i].pred_target = fetch_packet.pred_target[i];

                    // Update map table (after all prev in bundle)
                    if (fetch_packet.uses_rd[i]) begin
                        map_table[fetch_packet.rd_idx[i]].phys_tag = rd_phys[i];
                    end

                    // Advance tail
                    new_tail = new_tail + 1;
                end
            end
            rob_tail_update = new_tail;
            rs_compact = 1'b1;  // Compact RS after alloc for priority
        end
    end

    // Retire updates: free prev phys, update committed map (but map is speculative; retire uses arch map?)
    // In R10K, there's a separate committed map table updated on retire
    MAP_ENTRY [31:0] committed_map;  // Separate for recovery
    always_ff @(posedge clock) begin
        if (reset) begin
            // Init maps, etc.
        end else begin
            for (int i = 0; i < `N; i++) begin
                if (retire_packet.valid[i]) begin
                    free_add_valid[i] = 1'b1;
                    freed_phys[i] = retire_packet.prev_phys_rds[i];
                    committed_map[retire_packet.arch_rds[i]].phys_tag = retire_packet.phys_rds[i];
                end
            end
        end
    end

    // Mispredict recovery: truncate ROB/RS from mispred rob_idx, restore map/free list
    always_comb begin
        if (mispred_recovery.valid) begin
            // Walk ROB backwards from mispred.rob_idx to restore map and free list (add back allocated phys)
            // Placeholder logic: for each entry from tail to mispred+1, undo map, free phys_rd
            // Actual impl would loop over ROB entries
            // If checkpoint: restore from checkpoint_map if branch
        end
    end

    // Checkpoint map on branches (simplified: one checkpoint)
    always_ff @(posedge clock) begin
        // On dispatch of branch, checkpoint = map_table
        // On mispredict, map_table = checkpoint
    end

    // Update free counts (from ROB/RS modules)
    // assume inputs rob_free_count, rs_free_count from those modules

endmodule // stage_dispatch