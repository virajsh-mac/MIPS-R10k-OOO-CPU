/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  cpu.sv                                              //
//                                                                     //
//  Description :  Top-level module of the verisimple out-of-order      //
//                 processor; This instantiates and connects the OOO   //
//                 pipeline stages together.                            //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"
`include "ISA.svh"

module cpu (
    input clock,  // System clock
    input reset,  // System reset

    // Memory interface (data only - instruction fetch is fake)
    input MEM_TAG   mem2proc_transaction_tag,  // Memory tag for current transaction
    input MEM_BLOCK mem2proc_data,             // Data coming back from memory
    input MEM_TAG   mem2proc_data_tag,         // Tag for which transaction data is for

    output MEM_COMMAND proc2mem_command,  // Command sent to memory
    output ADDR        proc2mem_addr,     // Address sent to memory
    output MEM_BLOCK   proc2mem_data,     // Data sent to memory
`ifndef CACHE_MODE
    output MEM_SIZE    proc2mem_size,     // Data size sent to memory
`endif

    // Retire interface
    output COMMIT_PACKET [`N-1:0] committed_insts,

    // Additional debug outputs for OOO processor debugging
    output logic [`N-1:0] rob_head_valids_dbg,
    output ROB_ENTRY [`N-1:0] rob_head_entries_dbg,
    output ROB_IDX [`N-1:0] rob_head_idxs_dbg,
    output logic [$clog2(`N+1)-1:0] dispatch_count_dbg,
    output RS_GRANTED_BANKS rs_granted_dbg,
    output logic [`RS_ALU_SZ-1:0] rs_alu_ready_dbg,
    output ISSUE_ENTRIES issue_entries_dbg,
    output logic [`RS_ALU_SZ-1:0] rs_alu_requests_dbg,
    output logic [`RS_MULT_SZ-1:0] rs_mult_requests_dbg,
    output logic [`RS_BRANCH_SZ-1:0] rs_branch_requests_dbg,
    output logic [`RS_MEM_SZ-1:0] rs_mem_requests_dbg,
    output logic [`NUM_FU_ALU-1:0] alu_clear_signals_dbg,  // TEMP: ALU clear signals
    output logic [`RS_ALU_SZ-1:0][`NUM_FU_ALU-1:0] grants_alu_dbg,  // TEMP: allocator grants

    // Execute stage debug outputs
    output logic [`N-1:0] ex_valid_dbg,
    output EX_COMPLETE_PACKET ex_comp_dbg,

    // Complete stage debug outputs
    output ROB_UPDATE_PACKET rob_update_packet_dbg,

    // PRF Debug output
    output DATA [`PHYS_REG_SZ_R10K-1:0] regfile_entries_dbg,

    // architecture map table Debug output
    output MAP_ENTRY [`ARCH_REG_SZ-1:0] arch_table_snapshot_dbg,

    // rs_alu Debug output
    output RS_ENTRY [`RS_ALU_SZ-1:0] rs_alu_entries_dbg,

    // Additional RS debug outputs
    output RS_ENTRY [  `RS_MULT_SZ-1:0] rs_mult_entries_dbg,
    output RS_ENTRY [`RS_BRANCH_SZ-1:0] rs_branch_entries_dbg,
    output RS_ENTRY [   `RS_MEM_SZ-1:0] rs_mem_entries_dbg,

    // Map table debug output
    output MAP_ENTRY [`ARCH_REG_SZ-1:0] map_table_snapshot_dbg,

    // Freelist debug output (available physical registers)
    output logic [`PHYS_REG_SZ_R10K-1:0] freelist_available_dbg,
    output logic [`PHYS_REG_SZ_R10K-1:0] freelist_restore_mask_dbg,

    // CDB debug outputs
    output CDB_ENTRY [`N-1:0] cdb_output_dbg,
    output logic [`N-1:0][`NUM_FU_TOTAL-1:0] cdb_gnt_bus_dbg,
    output FU_REQUESTS cdb_requests_dbg,
    output CDB_FU_OUTPUTS cdb_fu_outputs_dbg,
    output logic [`NUM_FU_TOTAL-1:0] cdb_grants_flat_dbg,
    output CDB_EARLY_TAG_ENTRY [`N-1:0] cdb_early_tags_dbg,

    // Instruction Buffer debug output
    output FETCH_PACKET [`N-1:0] ib_output_disp_wdn_dbg,

    // Store Queue entry packet output
    output STOREQ_ENTRY [`N-1:0] sq_entry_packet_dbg,
    output logic [$clog2(`LSQ_SZ+1)-1:0] sq_free_slots_dbg,
    output logic [$clog2(`LSQ_SZ)-1:0] head_idx_dbg,
    output logic [$clog2(`LSQ_SZ)-1:0] tail_idx_dbg,



    // Issue clear signals debug output
    output RS_CLEAR_SIGNALS rs_clear_signals_dbg,

    // Fetch stage debug outputs
    output FETCH_PACKET [3:0]   fetch_packet_dbg,

    // Debug output, exposes DCache so memory is written back to .out file
    output D_CACHE_LINE [`DCACHE_LINES-1:0]      cache_lines_dbg
);

    //////////////////////////////////////////////////
    //                                              //
    //                Pipeline Wires                //
    //                                              //
    //////////////////////////////////////////////////

    // Outputs from ID stage (decode)
    FETCH_DISP_PACKET                                                                fetch_disp_packet;

    // From arch map table
    MAP_ENTRY [`ARCH_REG_SZ-1:0]                                                     arch_table_snapshot;

    // Outputs from Dispatch stage
    logic                   [               $clog2(`N+1)-1:0]                        dispatch_count;
    logic                   [                         `N-1:0]                        fetch_valid_mask;
    RS_ALLOC_BANKS                                                                   rs_alloc_from_dispatch;

    // ROB allocation signals
    ROB_ENTRY               [                         `N-1:0]                        rob_entry_packet;

    // ROB update signals
    ROB_UPDATE_PACKET                                                                rob_update_packet;
    logic                   [          $clog2(`ROB_SZ+1)-1:0]                        rob_free_slots;
    ROB_IDX                 [                         `N-1:0]                        rob_alloc_idxs;

    // Store Queue wires
    STOREQ_ENTRY [`N-1:0] sq_dispatch_packet;  // from dispatch
    STOREQ_IDX   [`N-1:0] sq_alloc_idxs;       // indices assigned by SQ
    STOREQ_IDX            sq_tail_idx;         // current SQ tail (for load forwarding)
    logic [$clog2(`LSQ_SZ+1)-1:0] sq_free_slots;

    // From execute: address/data updates for stores
    EXECUTE_STOREQ_ENTRY [`NUM_FU_MEM-1:0] mem_storeq_entries;

    // From ROB: how many SQ entries to free this cycle
    logic [$clog2(`N+1)-1:0] sq_free_count;

    // From Store Queue
    logic sq_unexecuted_store;

    // Store queue forwarding interface (execute <-> store queue)
    logic      [`NUM_FU_MEM-1:0] sq_lookup_valid;
    ADDR       [`NUM_FU_MEM-1:0] sq_lookup_addr;
    STOREQ_IDX [`NUM_FU_MEM-1:0] sq_lookup_sq_tail;
    logic      [`NUM_FU_MEM-1:0] sq_forward_valid;
    DATA       [`NUM_FU_MEM-1:0] sq_forward_data;
    logic      [`NUM_FU_MEM-1:0] sq_forward_stall;

    // From Store Queue to D-Cache (Processor Store)
    logic sq_to_dcache_valid;
    ADDR  sq_to_dcache_addr;
    DATA  sq_to_dcache_data;

    // Retire <-> D-Cache Handshake
    logic retire_store_request;
    logic dcache_store_response;
    
    // Retire count from stage_retire to ROB
    logic [$clog2(`N+1)-1:0] retire_count;

    // Free list allocation signals
    logic                   [                         `N-1:0]                        free_alloc_valid;
    PHYS_TAG                [                         `N-1:0]                        allocated_phys;
    logic                   [                         `N-1:0]                        freelist_alloc_req;
    logic                   [          `PHYS_REG_SZ_R10K-1:0]                        freelist_free_mask;
    logic                   [          `PHYS_REG_SZ_R10K-1:0]                        freelist_restore_mask;
    logic                   [                         `N-1:0][`PHYS_REG_SZ_R10K-1:0] freelist_granted_regs;
    logic                   [$clog2(`PHYS_REG_SZ_R10K+1)-1:0]                        freelist_free_slots;

    // Map table communication packets
    MAP_TABLE_WRITE_REQUEST [                         `N-1:0]                        maptable_write_reqs;
    MAP_TABLE_READ_REQUEST                                                           maptable_read_req;
    MAP_TABLE_READ_RESPONSE                                                          maptable_read_resp;

    // RS wires - structured by functional unit category
    RS_ALLOC_BANKS                                                                   rs_alloc;
    RS_GRANTED_BANKS                                                                 rs_granted;
    RS_BANKS                                                                         rs_banks;
    ISSUE_CLEAR                                                                      issue_clear;

    // Connect rs_alloc from dispatch stage
    assign rs_alloc = rs_alloc_from_dispatch;

    // RS clear signals (structured)
    RS_CLEAR_SIGNALS                        rs_clear_signals;

    // Individual RS entries outputs (needed for rs_banks)
    RS_ENTRY            [  `RS_MULT_SZ-1:0] rs_mult_entries;
    RS_ENTRY            [`RS_BRANCH_SZ-1:0] rs_branch_entries;
    RS_ENTRY            [   `RS_MEM_SZ-1:0] rs_mem_entries;

    // CDB wires (structured)
    FU_REQUESTS                             cdb_requests;
    FU_GRANTS                               cdb_grants;
    CDB_FU_OUTPUTS                          cdb_fu_outputs;
    CDB_EARLY_TAG_ENTRY [           `N-1:0] early_tag_broadcast;
    CDB_ENTRY           [           `N-1:0] cdb_output;

    // Execute stage signals
    logic               [           `N-1:0] ex_valid;
    EX_COMPLETE_PACKET                      ex_comp;
    PRF_READ_EN prf_read_en_src1, prf_read_en_src2;
    PRF_READ_TAGS prf_read_tag_src1, prf_read_tag_src2;
    PRF_READ_DATA prf_read_data_src1, prf_read_data_src2;
    logic [`NUM_FU_MULT-1:0] mult_request;
    logic [`NUM_FU_MEM-1:0] execute_mem_cdb_requests;

    // Retire stage signals
    ROB_ENTRY [`N-1:0] rob_head_entries;
    logic     [`N-1:0] rob_head_valids;
    ROB_IDX   [`N-1:0] rob_head_idxs;
    ROB_IDX            rob_mispred_idx;
    logic     [`N-1:0] arch_write_enables;
    REG_IDX   [`N-1:0] arch_write_addrs;
    PHYS_TAG  [`N-1:0] arch_write_phys_regs;

    // Global mispredict signal
    logic              mispredict;

        // Memory interface placeholders (TODO: implement proper data memory stages)
    logic                        Dmem_command_filtered = MEM_NONE;
    MEM_SIZE                     Dmem_size = DOUBLE;
    ADDR                         Dmem_addr = '0;
    MEM_BLOCK                    Dmem_store_data = '0;
    // Data memory request (from execute/store queue)
    logic        dmem_req_valid;
    MEM_COMMAND  dmem_req_command;
    ADDR         dmem_req_addr;
    MEM_BLOCK    dmem_req_data;
`ifndef CACHE_MODE
    MEM_SIZE     dmem_req_size;
`endif

    // CDB requests: single-cycle FUs request during issue, multi-cycle during execute
    assign cdb_requests.alu    = issue_cdb_requests.alu;  // From issue stage
    assign cdb_requests.mult   = mult_request;  // From execute stage (when completing)
    assign cdb_requests.branch = issue_cdb_requests.branch;  // From issue stage
    assign cdb_requests.mem    = execute_mem_cdb_requests;  // From execute stage (late requests for cache hits)

    // Connect dispatch outputs to freelist inputs
    assign freelist_alloc_req  = free_alloc_valid;

    // Get free slots from freelist module
    assign freelist_free_slots = freelist_0.free_slots;

    // debug for architecture map table
    MAP_ENTRY [`ARCH_REG_SZ-1:0] arch_table_snapshot_dbg_next;

    //////////////////////////////////////////////////
    //                                              //
    //                icache                        //
    //                                              //
    //////////////////////////////////////////////////


    // I-cache <-> fetch
    I_ADDR_PACKET          [1:0] i_cache_read_addrs;
    I_ADDR_PACKET           icache_mem_req_addr;
    CACHE_DATA             [1:0] icache_data;
    logic                        icache_mem_req_accepted;

    icache_subsystem icache_subsystem_inst (
        .clock            (clock),
        .reset            (reset),
        // Fetch
        .read_addrs       (i_cache_read_addrs),
        .cache_outs       (icache_data),
        // Mem.sv IOs
        .current_req_tag  (mem2proc_transaction_tag),
        .mem_data         (mem2proc_data),
        .mem_data_tag     (mem2proc_data_tag),

        // Arbitor IOs
        .mem_req_addr     (icache_mem_req_addr),
        .mem_req_accepted (icache_mem_req_accepted)
    );

    //////////////////////////////////////////////////
    //                                              //
    //                dcache                        //
    //                                              //
    //////////////////////////////////////////////////

    // D-cache <-> memory execution
    // TODO: Connect these to actual memory operation addresses from execute stage
    D_ADDR_PACKET          [1:0] d_cache_read_addrs;
    CACHE_DATA             [1:0] dcache_data;
    D_ADDR_PACKET                dcache_mem_req_addr;
    D_ADDR_PACKET                dcache_mem_write_addr;
    MEM_BLOCK                    dcache_mem_write_data;
    logic                        dcache_mem_write_valid;
    logic                        dcache_mem_req_accepted;


    // Dcache read addresses come from execute stage (for loads)
    // Note: This is connected below in the execute stage instantiation

    dcache_subsystem dcache_subsystem_inst (
        .clock            (clock),
        .reset            (reset),
        // Memory operations read
        .read_addrs       (d_cache_read_addrs),
        .cache_outs       (dcache_data),
        // Mem.sv IOs - Read requests
        .current_req_tag  (mem2proc_transaction_tag),
        .mem_data         (mem2proc_data),
        .mem_data_tag     (mem2proc_data_tag),
        // Arbitor IOs - Read requests
        .mem_req_addr     (dcache_mem_req_addr),
        .mem_req_accepted (dcache_mem_req_accepted),
        // Arbitor IOs - Write requests (dirty writebacks)
        .mem_write_addr   (dcache_mem_write_addr),
        .mem_write_data   (dcache_mem_write_data),
        .mem_write_valid  (dcache_mem_write_valid),
        // Processor Stores
        .proc_store_valid (retire_store_request), // Driven by Retire, using data from SQ
        .proc_store_addr  (sq_to_dcache_addr),
        .proc_store_data  (sq_to_dcache_data),
        .proc_store_response (dcache_store_response),
        // debug to expose DCache to testbench
        .cache_lines_debug (cache_lines_dbg)
    );
    //////////////////////////////////////////////////
    //                                              //
    //          Memory Arbiter Logic                //
    //                                              //
    //////////////////////////////////////////////////

    // Arbitration: Data requests (dcache) prioritized over instruction requests (icache)
    // Writes: Only dcache writes, no conflicts
    // 
    // D_ADDR to 32-bit address conversion:
    // D_ADDR stores: tag = addr[31:3] (29 bits), block_offset = addr[2:0] (3 bits)
    // To reconstruct 8-byte-aligned address: {tag, 3'b0}
    always_comb begin
        // Default values
        proc2mem_command = MEM_NONE;
        proc2mem_addr    = '0;
        proc2mem_data    = '0;
        icache_mem_req_accepted = 1'b0;
        dcache_mem_req_accepted = 1'b0;

        // Priority 1: Dcache writes (dirty writebacks)
        if (dcache_mem_write_valid) begin
            proc2mem_command = MEM_STORE;
            // Convert D_ADDR to 32-bit 8-byte-aligned address
            proc2mem_addr    = {dcache_mem_write_addr.addr.tag, 3'b0};
            proc2mem_data    = dcache_mem_write_data;
            // Writes are always accepted if transaction tag is available
            // Note: mem_req_accepted logic is for reads only
        end
        // Priority 2: Dcache reads (data memory operations)
        else if (dcache_mem_req_addr.valid) begin
            proc2mem_command = MEM_LOAD;
            // Convert D_ADDR to 32-bit 8-byte-aligned address
            proc2mem_addr    = {dcache_mem_req_addr.addr.tag, 3'b0};
            dcache_mem_req_accepted = (mem2proc_transaction_tag != 0);
        end
        // Priority 3: Icache reads (instruction fetch)
        else if (icache_mem_req_addr.valid) begin
            proc2mem_command = MEM_LOAD;
            proc2mem_addr    = icache_mem_req_addr.addr;
            icache_mem_req_accepted = (mem2proc_transaction_tag != 0);
        end

`ifndef CACHE_MODE
        proc2mem_size = DOUBLE; // Always 64-bit blocks in cache mode
`endif
    end
    //////////////////////////////////////////////////
    //                                              //
    //                  Fetch-Stage                 //
    //                                              //
    //////////////////////////////////////////////////

    // fetch stage signals
    // Note: I'm going to add all the single here and then move it to the pipeline wire stage
    BP_PREDICT_REQUEST    bp_predict_request;
    BP_PREDICT_RESPONSE   bp_predict_response;

    FETCH_PACKET    [3:0]     fetch_packet;
    ADDR correct_branch_target;
    logic [`IB_IDX_BITS:0] ib_free_slots;

    stage_fetch stage_fetch_0 (
        .clock        (clock),
        .reset        (reset),

        .read_addrs   (i_cache_read_addrs),
        .cache_data   (icache_data),

        .fetch_packet (fetch_packet),

        .bp_request   (bp_predict_request),
        .bp_response  (bp_predict_response),

        .correct_branch_target ({mispredict, correct_branch_target}),
        .ib_free_slots          (ib_free_slots)
    );

    // Branch Predictor singals
   BP_TRAIN_REQUEST train_req;
   BP_RECOVER_REQUEST recover_req;

    bp bp_0 (
        .clock(clock),
        .reset(reset),

        // predict request IF -> BP
        .predict_req_i(bp_predict_request),

        // predict response BP -> IF
        .predict_resp_o(bp_predict_response),

        // training request (from ROB retire stage)
        .train_req_i(train_req)
    );

    //////////////////////////////////////////////////
    //                                              //
    //           Instruction Buffer (IB)             //
    //                                              //
    //////////////////////////////////////////////////

    // Instruction Buffer signals
    FETCH_PACKET [2:0] ib_dispatch_window;  // Window of instructions for decode/dispatch
    logic [1:0]        ib_window_valid_count; // Number of valid instructions in window

    instr_buffer instr_buffer_0 (
        .clock(clock),
        .reset(reset | mispredict),

        // Fetch
        .new_ib_entries(fetch_packet),
        .available_slots(ib_free_slots),

        // Dispatch
        .num_pops(dispatch_count),  // Dispatch count (0-3)
        .dispatch_window(ib_dispatch_window),
        .window_valid_count(ib_window_valid_count)
    );

    //////////////////////////////////////////////////
    //                                              //
    //                Decode Stage                  //
    //                                              //
    //////////////////////////////////////////////////

    // Decoder outputs for each instruction in the bundle
    ALU_OPA_SELECT [`N-1:0] decode_opa_select;
    ALU_OPB_SELECT [`N-1:0] decode_opb_select;
    logic          [`N-1:0] decode_has_dest;
    OP_TYPE        [`N-1:0] decode_op_type;
    logic          [`N-1:0] decode_csr_op;
    logic          [`N-1:0] decode_halt;
    logic          [`N-1:0] decode_illegal;

    // Enhanced decoder outputs for packet construction
    REG_IDX [`N-1:0] decode_rs1_idx;
    REG_IDX [`N-1:0] decode_rs2_idx;
    REG_IDX [`N-1:0] decode_rd_idx;
    logic   [`N-1:0] decode_uses_rd;
    DATA    [`N-1:0] decode_immediate;

    // Instantiate decoders for each instruction in the bundle
    for (genvar i = 0; i < `N; i++) begin
        decoder decoder_i (
            .inst      (ib_dispatch_window[i].inst),
            .valid     (i < ib_window_valid_count),
            .opa_select(decode_opa_select[i]),
            .opb_select(decode_opb_select[i]),
            .has_dest  (decode_has_dest[i]),
            .op_type   (decode_op_type[i]),
            .csr_op    (decode_csr_op[i]),
            .halt      (decode_halt[i]),
            .illegal   (decode_illegal[i]),
            // Enhanced outputs
            .rs1_idx   (decode_rs1_idx[i]),
            .rs2_idx   (decode_rs2_idx[i]),
            .rd_idx    (decode_rd_idx[i]),
            .uses_rd   (decode_uses_rd[i]),
            .immediate (decode_immediate[i])
        );
    end

    //////////////////////////////////////////////////
    //                                              //
    //                Dispatch-Stage                //
    //                                              //
    //////////////////////////////////////////////////

    // Dispatch stage
    stage_dispatch stage_dispatch_0 (
        .clock(clock),
        .reset(reset),

        // From decode: individual signals
        .decode_rs1_idx   (decode_rs1_idx),
        .decode_rs2_idx   (decode_rs2_idx),
        .decode_rd_idx    (decode_rd_idx),
        .decode_uses_rd   (decode_uses_rd),
        .decode_op_type   (decode_op_type),
        .decode_opa_select(decode_opa_select),
        .decode_opb_select(decode_opb_select),
        .decode_immediate (decode_immediate),
        .decode_halt      (decode_halt),
        .dispatch_window  (ib_dispatch_window),  // Full instruction packets from IB
        .window_valid_count(ib_window_valid_count), // Number of valid instructions

        // From ROB/Freelist
        .free_slots_rob    (rob_free_slots),
        .rob_alloc_idxs    (rob_alloc_idxs),
        .freelist_free_slots(freelist_free_slots),

        // From RS Banks: free slot counts
        .rs_alu_free_slots   (rs_alu.free_slots),
        .rs_mult_free_slots  (rs_mult.free_slots),
        .rs_branch_free_slots(rs_branch.free_slots),
        .rs_mem_free_slots   (rs_mem.free_slots),

        // From Store Queue
        .store_queue_has_pending_store(sq_unexecuted_store),

        // To Instruction Buffer
        .dispatch_count(dispatch_count),

        // TO ROB
        .rob_entry_packet(rob_entry_packet),

        // TO Store queue free slots
       .store_queue_free_slots(sq_free_slots),

        // To Store Queue
        .store_queue_alloc_idxs   (sq_alloc_idxs),      // indices from SQ
        .store_queue_tail_idx     (sq_tail_idx),        // SQ tail for load forwarding
        .store_queue_entry_packet (sq_dispatch_packet), // entries to SQ

        // TO RS (structured allocation requests)
        .rs_alloc(rs_alloc_from_dispatch),

        // TO FREE LIST
        .free_alloc_valid(free_alloc_valid),
        .granted_regs    (freelist_granted_regs),

        // TO/FROM MAP TABLE
        .maptable_write_reqs(maptable_write_reqs),
        .maptable_read_req  (maptable_read_req),
        .maptable_read_resp (maptable_read_resp)
    );

    //////////////////////////////////////////////////
    //                                              //
    //             Reorder Buffer (ROB)             //
    //                                              //
    //////////////////////////////////////////////////

    rob rob_0 (
        .clock(clock),
        .reset(reset | mispredict), // Reset on mispredict

        // Dispatch
        .rob_entry_packet(rob_entry_packet),
        .free_slots(rob_free_slots),
        .alloc_idxs(rob_alloc_idxs),

        // Complete
        .rob_update_packet(rob_update_packet),

        // Retire
        .retire_count_in(retire_count),
        .head_entries(rob_head_entries),
        .head_idxs(rob_head_idxs),
        .head_valids(rob_head_valids)
    );

    //////////////////////////////////////////////////
    //                                              //
    //                 Store Queue                  //
    //                                              //
    //////////////////////////////////////////////////

    store_queue store_queue_0 (
        .clock(clock),
        .reset(reset),

        // Dispatch side
        .sq_dispatch_packet(sq_dispatch_packet),
        .free_slots        (sq_free_slots),
        .sq_alloc_idxs     (sq_alloc_idxs),
        .sq_tail_idx       (sq_tail_idx),

        // Execute side
        .mem_storeq_entries(mem_storeq_entries),

        // Load forwarding interface (from execute stage MEM FUs)
        .load_lookup_valid (sq_lookup_valid),
        .load_lookup_addr  (sq_lookup_addr),
        .load_lookup_sq_tail(sq_lookup_sq_tail),
        .forward_valid     (sq_forward_valid),
        .forward_data      (sq_forward_data),
        .forward_stall     (sq_forward_stall),

        // Retire / flush side
        .mispredict(mispredict),
        .free_count(sq_free_count),

        // Outputs
        .unexecuted_store(sq_unexecuted_store),
        
        // To D-Cache
        .dcache_store_valid(sq_to_dcache_valid),
        .dcache_store_addr (sq_to_dcache_addr),
        .dcache_store_data (sq_to_dcache_data),

        // Dbg Signals
        .head_idx_dbg (head_idx_dbg),
        .tail_idx_dbg (tail_idx_dbg)
    );

    //////////////////////////////////////////////////
    //                                              //
    //           Reservation Stations (RS)          //
    //                                              //
    //////////////////////////////////////////////////

    // RS for ALU operations (6 entries, 3 clear ports)
    rs #(
        .ALLOC_WIDTH(`N),
        .RS_SIZE(`RS_ALU_SZ),
        .CLEAR_WIDTH(`NUM_FU_ALU),
        .CDB_WIDTH(`CDB_SZ)
    ) rs_alu (
        // Inputs
        .clock(clock),
        .reset(reset),

        // From dispatch: allocation signals (structured)
        .alloc_valid  (rs_alloc.alu.valid),
        .alloc_entries(rs_alloc.alu.entries),

        // From complete: CDB broadcasts for operand wakeup
        .early_tag_broadcast(early_tag_broadcast),

        // From issue: clear signals for issued entries
        .clear_valid(rs_clear_signals.valid_alu),
        .clear_idxs (rs_clear_signals.idxs_alu),

        // From execute: mispredict flush signal
        .mispredict(mispredict),

        // Outputs to issue/dispatch
        .entries        (rs_alu_entries_dbg),
        .granted_entries(rs_granted.alu)
    );

    // RS for MULT operations (2 entries, 1 clear port)
    rs #(
        .ALLOC_WIDTH(`N),
        .RS_SIZE(`RS_MULT_SZ),
        .CLEAR_WIDTH(`NUM_FU_MULT),
        .CDB_WIDTH(`CDB_SZ)
    ) rs_mult (
        // Inputs
        .clock(clock),
        .reset(reset),

        // From dispatch: allocation signals (structured)
        .alloc_valid  (rs_alloc.mult.valid),
        .alloc_entries(rs_alloc.mult.entries),

        // From complete: CDB broadcasts for operand wakeup
        .early_tag_broadcast(early_tag_broadcast),

        // From issue: clear signals for issued entries
        .clear_valid(rs_clear_signals.valid_mult),
        .clear_idxs (rs_clear_signals.idxs_mult),

        // From execute: mispredict flush signal
        .mispredict(mispredict),

        // Outputs to issue/dispatch
        .entries        (rs_mult_entries),
        .granted_entries(rs_granted.mult)
    );

    // RS for BRANCH operations (2 entries, 1 clear port)
    rs #(
        .ALLOC_WIDTH(`N),
        .RS_SIZE(`RS_BRANCH_SZ),
        .CLEAR_WIDTH(`NUM_FU_BRANCH),
        .CDB_WIDTH(`CDB_SZ)
    ) rs_branch (
        // Inputs
        .clock(clock),
        .reset(reset),

        // From dispatch: allocation signals (structured)
        .alloc_valid  (rs_alloc.branch.valid),
        .alloc_entries(rs_alloc.branch.entries),

        // From complete: CDB broadcasts for operand wakeup
        .early_tag_broadcast(early_tag_broadcast),

        // From issue: clear signals for issued entries
        .clear_valid(rs_clear_signals.valid_branch),
        .clear_idxs (rs_clear_signals.idxs_branch),

        // From execute: mispredict flush signal
        .mispredict(mispredict),

        // Outputs to issue/dispatch
        .entries        (rs_branch_entries),
        .granted_entries(rs_granted.branch)
    );

    // RS for MEM operations (2 entries, 1 clear port)
    rs #(
        .ALLOC_WIDTH(`N),
        .RS_SIZE(`RS_MEM_SZ),
        .CLEAR_WIDTH(`NUM_FU_MEM),
        .CDB_WIDTH(`CDB_SZ)
    ) rs_mem (
        // Inputs
        .clock(clock),
        .reset(reset),

        // From dispatch: allocation signals (structured)
        .alloc_valid  (rs_alloc.mem.valid),
        .alloc_entries(rs_alloc.mem.entries),

        // From complete: CDB broadcasts for operand wakeup
        .early_tag_broadcast(early_tag_broadcast),

        // From issue: clear signals for issued entries
        .clear_valid(rs_clear_signals.valid_mem),
        .clear_idxs (rs_clear_signals.idxs_mem),

        // From execute: mispredict flush signal
        .mispredict(mispredict),

        // Outputs to issue/dispatch
        .entries        (rs_mem_entries),
        .granted_entries(rs_granted.mem)
    );

    //////////////////////////////////////////////////
    //                                              //
    //                 Issue Stage                  //
    //                                              //
    //////////////////////////////////////////////////

    // Issue stage structured inputs/outputs
    ISSUE_ENTRIES issue_entries;
    FU_REQUESTS issue_cdb_requests;

    // Debug signals from issue stage
    logic [`RS_ALU_SZ-1:0] rs_alu_ready;
    ISSUE_ENTRIES issue_entries_debug;

    // Create structured RS banks from individual RS module outputs
    assign rs_banks.alu    = rs_alu_entries_dbg;
    assign rs_banks.mult   = rs_mult_entries;
    assign rs_banks.branch = rs_branch_entries;
    assign rs_banks.mem    = rs_mem_entries;

    stage_issue stage_issue_0 (
        .clock(clock),
        .reset(reset),
        .mispredict(mispredict),

        // RS entries (structured)
        .rs_banks(rs_banks),

        // FU availability grants (structured) - comes from CDB
        .fu_grants(cdb_grants),

        // Clear signals (structured)
        .issue_clear(issue_clear),

        // Issue outputs (structured)
        .issue_entries(issue_entries),

        // CDB requests for single-cycle FUs
        .cdb_requests(issue_cdb_requests),

        // Debug outputs
        .rs_alu_ready_dbg(rs_alu_ready),
        .issue_entries_dbg(issue_entries_debug),
        .rs_alu_requests_dbg(rs_alu_requests_dbg),
        .rs_mult_requests_dbg(rs_mult_requests_dbg),
        .rs_branch_requests_dbg(rs_branch_requests_dbg),
        .rs_mem_requests_dbg(rs_mem_requests_dbg),
        .alu_clear_signals_dbg(alu_clear_signals_dbg),
        .grants_alu_dbg(grants_alu_dbg)
    );

    // Extract clear signals from structured output for RS modules
    assign rs_clear_signals = issue_clear;

    //////////////////////////////////////////////////
    //                                              //
    //                Execute Stage                  //
    //                                              //
    //////////////////////////////////////////////////

    stage_execute stage_execute_0 (
        .clock(clock),
        .reset(reset),

        .mispredict(mispredict),

        // Inputs from issue stage (structured)
        .issue_entries(issue_entries),

        // Input from CDB for data forwarding
        .cdb_data(cdb_output),

        // To PRF for operand reads (structured)
        .prf_read_en_src1  (prf_read_en_src1),
        .prf_read_en_src2  (prf_read_en_src2),
        .prf_read_tag_src1 (prf_read_tag_src1),
        .prf_read_tag_src2 (prf_read_tag_src2),
        .prf_read_data_src1(prf_read_data_src1),
        .prf_read_data_src2(prf_read_data_src2),

        // Outputs
        .mult_request(mult_request),
        .fu_outputs  (cdb_fu_outputs),

        // To complete stage
        .ex_valid(ex_valid),
        .ex_comp (ex_comp),

        // From CDB for grant selection
        .gnt_bus(cdb_0.grant_bus_out),

        // to/from dcache
        .dcache_read_addrs(d_cache_read_addrs),
        .dcache_read_data(dcache_data),

        // To Store Queue
        .mem_storeq_entries(mem_storeq_entries),

        // Late CDB requests from MEM FUs
        .mem_cdb_requests_out(execute_mem_cdb_requests),

        // Store queue forwarding interface
        .sq_lookup_valid(sq_lookup_valid),
        .sq_lookup_addr(sq_lookup_addr),
        .sq_lookup_sq_tail(sq_lookup_sq_tail),
        .sq_forward_valid(sq_forward_valid),
        .sq_forward_data(sq_forward_data),
        .sq_forward_stall(sq_forward_stall)
    );

    //////////////////////////////////////////////////
    //                                              //
    //       Execute/Complete Pipeline Register      //
    //                                              //
    //////////////////////////////////////////////////

    logic              [`N-1:0] ex_comp_reg_valid;
    EX_COMPLETE_PACKET          ex_comp_reg;

    always_ff @(posedge clock) begin
        if (reset | mispredict) begin
            ex_comp_reg_valid <= '0;
            ex_comp_reg       <= '0;
        end else begin
            ex_comp_reg_valid <= ex_valid;
            ex_comp_reg       <= ex_comp;
        end
    end

    //////////////////////////////////////////////////
    //                                              //
    //                 Map Table                     //
    //                                              //
    //////////////////////////////////////////////////

    map_table map_table_0 (
        .clock(clock),
        .reset(reset),

        // From dispatch: new register mappings
        .write_reqs(maptable_write_reqs),

        // From dispatch: read requests
        .read_req(maptable_read_req),

        // To dispatch: read responses
        .read_resp(maptable_read_resp),

        // From CDB: broadcasts that update ready bits
        .cdb_broadcasts(cdb_output),

        // Mispredict recovery
        .table_snapshot(),
        .table_restore(arch_table_snapshot_dbg),
        .table_restore_en(mispredict)
    );

    //////////////////////////////////////////////////
    //                                              //
    //               Architected Map Table           //
    //                                              //
    //////////////////////////////////////////////////

    arch_map_table arch_map_table_0 (
        .clock(clock),
        .reset(reset),

        // From retire: update architected register mappings
        .write_enables (arch_write_enables),
        .write_addrs   (arch_write_addrs),
        .write_phys_regs(arch_write_phys_regs),

        // Read ports for selective access (not used in current design)
        .read_addrs  ('0),
        .read_entries(),

        // Mispredict recovery: output snapshot for map_table restoration
        .table_snapshot(arch_table_snapshot_dbg),
        .table_restore('0),  // Not used - arch table doesn't restore
        .table_restore_en(1'b0),  // Arch table never restores

        // debug output
        .table_snapshot_next(arch_table_snapshot_dbg_next)
    );

    //////////////////////////////////////////////////
    //                                              //
    //                  Freelist                     //
    //                                              //
    //////////////////////////////////////////////////

    freelist freelist_0 (
        .clock(clock),
        .reset(reset),
        .mispredict(mispredict),
        .restore_mask(freelist_restore_mask),

        // From dispatch: allocation requests
        .alloc_req(freelist_alloc_req),

        // From retire: deallocation requests
        .free_mask(freelist_free_mask),

        // To dispatch: granted physical registers
        .granted_regs(freelist_granted_regs)
    );

    //////////////////////////////////////////////////
    //                                              //
    //            Physical Register File            //
    //                                              //
    //////////////////////////////////////////////////

    // Instantiate Physical Register File
    regfile prf (
        .clock(clock),
        .reset(reset),

        // Read interface - directly connect structured interfaces
        .read_tags({prf_read_tag_src2, prf_read_tag_src1}),
        .read_data({prf_read_data_src2, prf_read_data_src1}),

        // Write interface - directly from CDB
        .cdb_writes(cdb_output),

        .regfile_entries(regfile_entries_dbg)
    );

    //////////////////////////////////////////////////
    //                                              //
    //                    CDB                       //
    //                                              //
    //////////////////////////////////////////////////


    cdb cdb_0 (
        .clock(clock),
        .reset(reset || mispredict),

        // Arbiter inputs (structured)
        .requests(cdb_requests),

        // Arbiter outputs indicating which requests are granted (structured)
        .grants(cdb_grants),

        // CDB inputs from functional units (structured)
        .fu_outputs(cdb_fu_outputs),

        // CDB output indicating which tags should be awoken a cycle early
        .early_tags(early_tag_broadcast),

        // CDB register outputs broadcasting to PRF, EX stage, and Map Table
        .cdb_output(cdb_output),

        // Debug outputs
        .requests_dbg(cdb_requests_dbg),
        .fu_outputs_dbg(cdb_fu_outputs_dbg),
        .grants_flat_dbg(cdb_grants_flat_dbg),
        .gnt_bus_dbg(cdb_gnt_bus_dbg),
        .early_tags_dbg(cdb_early_tags_dbg)
    );

    //////////////////////////////////////////////////
    //                                              //
    //              Complete Stage                  //
    //                                              //
    //////////////////////////////////////////////////

    stage_complete stage_complete_0 (
        .clock(clock),
        .reset(reset),

        // From EX/COMP pipe reg
        .ex_valid_in(ex_comp_reg_valid),
        .ex_comp_in (ex_comp_reg),

        // To ROB
        .rob_update_packet(rob_update_packet),

        // Debug output
        .rob_update_packet_dbg(rob_update_packet_dbg)
    );

    //////////////////////////////////////////////////
    //                                              //
    //                Retire Stage                  //
    //                                              //
    //////////////////////////////////////////////////

    logic v;

    logic bp_enabled;
    logic bp_enabled_dbg;
    logic branch_retired_dbg;
    logic branch_taken_dbg;
    logic is_branch_target_unknown_dbg;
    logic train_triggered_dbg;
    logic retire_valid_dbg;

    stage_retire stage_retire_0 (
        .clock(clock),
        .reset(reset),

        // From ROB: head window (N-1 = oldest, 0 = youngest)
        .head_entries(rob_head_entries),
        .head_valids (rob_head_valids),
        .head_idxs   (rob_head_idxs),

        // To ROB: flush younger if head is a mispredicted branch
        .mispredict (mispredict),
        .rob_mispred_idx(rob_mispred_idx),

        // To freelist: bitmap of PRs to free (all committed lanes' Told this cycle)
        .free_mask(freelist_free_mask),

        // To archMapTable: N write ports (commit multiple per cycle)
        .arch_write_enables  (arch_write_enables),
        .arch_write_addrs    (arch_write_addrs),
        .arch_write_phys_regs(arch_write_phys_regs),

        .committed_insts(committed_insts),


        // To fetch
        .branch_target_out(correct_branch_target),

        // Branch predictor
        .train_req_o            (train_req),

        // From PRF for committed data
        .regfile_entries(regfile_entries_dbg),

        // From arch map table for freelist restore on mispredict
        .arch_table_snapshot(arch_table_snapshot),

        // To freelist: restore mask on mispredict
        .freelist_restore_mask(freelist_restore_mask),

        // to store queue: free entries and writes to DCache
        .sq_free_count(sq_free_count),

        // From store queue
        .sq_head_valid(sq_to_dcache_valid),

        // To D-Cache
        .dcache_store_request(retire_store_request),
        .dcache_store_response(dcache_store_response),
        
        // To ROB: how many to actually retire
        .retire_count_out(retire_count)
    );

    //////////////////////////////////////////////////
    //                                              //
    //               Pipeline Outputs               //
    //                                              //
    //////////////////////////////////////////////////

    // Output the committed instructions to the testbench for counting
    // For superscalar, show the oldest ready instruction (whether retired or not)


    // Fake-fetch outputs
  //  assign ff_consumed            = dispatch_count;  // Number of instructions consumed by dispatch


    // Additional debug outputs
    assign rob_head_valids_dbg    = rob_head_valids;
    assign rob_head_entries_dbg   = rob_head_entries;
    assign rob_head_idxs_dbg      = rob_head_idxs;
    assign dispatch_count_dbg     = dispatch_count;
    assign rs_granted_dbg         = rs_granted;
    assign rs_alu_ready_dbg       = rs_alu_ready;
    assign issue_entries_dbg      = issue_entries_debug;

    // Execute stage debug outputs
    assign ex_valid_dbg           = ex_valid;
    assign ex_comp_dbg            = ex_comp;

    // Additional RS debug outputs
    assign rs_mult_entries_dbg    = rs_mult_entries;
    assign rs_branch_entries_dbg  = rs_branch_entries;
    assign rs_mem_entries_dbg     = rs_mem_entries;

    // Map table debug output
    assign map_table_snapshot_dbg = map_table_0.map_table_reg;

    // Freelist debug output (available physical registers)
    assign freelist_available_dbg = freelist_0.available_regs;
    assign freelist_restore_mask_dbg = freelist_restore_mask;

    // CDB debug outputs
    assign cdb_output_dbg         = cdb_output;

    // Issue clear signals debug output
    assign rs_clear_signals_dbg   = rs_clear_signals;

    // Fetch stage debug outputs
    assign fetch_packet_dbg      = fetch_packet;

    // Instruction Buffer debug outputs
    assign ib_output_disp_wdn_dbg = ib_dispatch_window;

    // Store Queue entry packet output
    assign sq_entry_packet_dbg = sq_dispatch_packet;
    assign sq_free_slots_dbg = sq_free_slots;

endmodule  // cpu
