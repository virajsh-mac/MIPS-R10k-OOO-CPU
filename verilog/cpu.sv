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

    // Dispatch packet debug output
    output FETCH_DISP_PACKET fetch_disp_packet_dbg,

    // Issue clear signals debug output
    output RS_CLEAR_SIGNALS rs_clear_signals_dbg,

    // Execute stage debug outputs
    output FU_RESULTS fu_results_dbg,
    output PRF_READ_EN prf_read_en_src1_dbg,
    output PRF_READ_EN prf_read_en_src2_dbg,
    output PRF_READ_TAGS prf_read_tag_src1_dbg,
    output PRF_READ_TAGS prf_read_tag_src2_dbg,
    output PRF_READ_DATA resolved_src1_dbg,
    output PRF_READ_DATA resolved_src2_dbg,
    output logic [`N-1:0][`NUM_FU_TOTAL-1:0] fu_gnt_bus_dbg,
    output logic [`NUM_FU_MULT-1:0] mult_request_dbg,
    output logic [`NUM_FU_MULT-1:0] mult_start_dbg,
    output logic [`NUM_FU_MULT-1:0] mult_done_dbg,
    output logic [`NUM_FU_BRANCH-1:0] branch_take_dbg,
    output ADDR [`NUM_FU_BRANCH-1:0] branch_target_dbg,
    output logic [`NUM_FU_ALU-1:0] alu_executing_dbg,
    output ALU_FUNC [`NUM_FU_ALU-1:0] alu_func_dbg,
    output logic [`NUM_FU_MULT-1:0] mult_executing_dbg,
    output logic [`NUM_FU_BRANCH-1:0] branch_executing_dbg,
    output logic [`NUM_FU_MEM-1:0] mem_executing_dbg,

    // ICache debug outputs
    output I_ADDR_PACKET [1:0]  read_addrs_dbg,
    output CACHE_DATA [1:0]     cache_outs_dbg,
    output logic [1:0]          icache_hits_dbg,
    output logic [1:0]          icache_misses_dbg,
    output logic                icache_full_dbg,
    output I_ADDR_PACKET        prefetch_addr_dbg,
    output I_ADDR_PACKET        oldest_miss_addr_dbg,
    output logic                mshr_addr_found_dbg,
    output logic [$clog2(`NUM_MEM_TAGS + `N)-1:0] mshr_head_dbg,
    output logic [$clog2(`NUM_MEM_TAGS + `N)-1:0] mshr_tail_dbg,
    output MSHR_PACKET [`NUM_MEM_TAGS + `N-1:0]   mshr_entries_dbg,
    output logic [$clog2(`NUM_MEM_TAGS + `N)-1:0] mshr_next_head_dbg,
    output logic [$clog2(`NUM_MEM_TAGS + `N)-1:0] mshr_next_tail_dbg,
    output logic                             mshr_pop_condition_dbg,
    output logic                             mshr_push_condition_dbg,
    output logic                             mshr_pop_cond_has_data_dbg,
    output logic                             mshr_pop_cond_head_valid_dbg,
    output logic                             mshr_pop_cond_tag_match_dbg,
    output logic                mem_write_icache_dbg,
    output I_ADDR_PACKET        mem_write_addr_dbg,
    output MEM_BLOCK            mem_data_dbg,
    output MEM_TAG              mem_data_tag_dbg,
    output I_ADDR_PACKET        icache_write_addr_dbg,
    output MEM_BLOCK            icache_write_data_dbg,
    output I_CACHE_LINE         icache_line_write_dbg,
    output logic [(`ICACHE_LINES + `PREFETCH_STREAM_BUFFER_SIZE)-1:0] icache_write_enable_mask_dbg,
    // Prefetcher debug outputs
    output I_ADDR_PACKET        prefetcher_last_icache_miss_mem_req_dbg,
    output I_ADDR_PACKET        prefetcher_next_last_icache_miss_mem_req_dbg,
    output I_ADDR               prefetcher_addr_incrementor_dbg,
    output I_ADDR               prefetcher_next_addr_incrementor_dbg,
    // Logic block debug outputs
    output I_ADDR_PACKET        mem_req_addr_dbg,
    output logic                snooping_found_icache_dbg,
    output MSHR_PACKET          new_mshr_entry_dbg,

    // Fetch stage debug outputs
    output FETCH_PACKET [3:0]   fetch_packet_dbg,
    output logic                ib_bundle_valid_dbg,

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

    // Retire stage signals
    ROB_ENTRY [`N-1:0] rob_head_entries;
    logic     [`N-1:0] rob_head_valids;
    ROB_IDX   [`N-1:0] rob_head_idxs;
    ROB_IDX            rob_mispred_idx;
    logic     [`N-1:0] arch_write_enables;
    REG_IDX   [`N-1:0] arch_write_addrs;
    PHYS_TAG  [`N-1:0] arch_write_phys_regs;

    // DEBUG signal for committed instructions:
    COMMIT_PACKET [`N-1:0] retire_commits_dbg;

    // Global mispredict signal
    logic              mispredict;

    // CDB requests: single-cycle FUs request during issue, multi-cycle during execute
    assign cdb_requests.alu    = issue_cdb_requests.alu;  // From issue stage
    assign cdb_requests.mult   = mult_request;  // From execute stage (when completing)
    assign cdb_requests.branch = issue_cdb_requests.branch;  // From issue stage
    assign cdb_requests.mem    = issue_cdb_requests.mem;  // From issue stage

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
    I_ADDR_PACKET          [1:0] i_cache_read_addrs ;
    I_ADDR_PACKET           mem_req_addr;
    CACHE_DATA             [1:0] icache_data ;

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
        .mem_req_addr     (mem_req_addr),
        .mem_req_accepted (mem_req_accepted),

        // Debug outputs
        .read_addrs_dbg       (read_addrs_dbg),
        .cache_outs_dbg       (cache_outs_dbg),
        .icache_hits_dbg      (icache_hits_dbg),
        .icache_misses_dbg    (icache_misses_dbg),
        .icache_full_dbg      (icache_full_dbg),
        .prefetch_addr_dbg    (prefetch_addr_dbg),
        .oldest_miss_addr_dbg (oldest_miss_addr_dbg),
        .mshr_addr_found_dbg  (mshr_addr_found_dbg),
        .mshr_head_dbg        (mshr_head_dbg),
        .mshr_tail_dbg        (mshr_tail_dbg),
        .mshr_entries_dbg     (mshr_entries_dbg),
        .mshr_next_head_dbg   (mshr_next_head_dbg),
        .mshr_next_tail_dbg   (mshr_next_tail_dbg),
        .mshr_pop_condition_dbg(mshr_pop_condition_dbg),
        .mshr_push_condition_dbg(mshr_push_condition_dbg),
        .mshr_pop_cond_has_data_dbg(mshr_pop_cond_has_data_dbg),
        .mshr_pop_cond_head_valid_dbg(mshr_pop_cond_head_valid_dbg),
        .mshr_pop_cond_tag_match_dbg(mshr_pop_cond_tag_match_dbg),
        .mem_write_icache_dbg (mem_write_icache_dbg),
        .mem_write_addr_dbg   (mem_write_addr_dbg),
        .mem_data_dbg         (mem_data_dbg),
        .mem_data_tag_dbg     (mem_data_tag_dbg),
        .icache_write_addr_dbg(icache_write_addr_dbg),
        .icache_write_data_dbg(icache_write_data_dbg),
        .icache_line_write_dbg(icache_line_write_dbg),
        .icache_write_enable_mask_dbg(icache_write_enable_mask_dbg),
        // Prefetcher debug outputs
        .prefetcher_last_icache_miss_mem_req_dbg(prefetcher_last_icache_miss_mem_req_dbg),
        .prefetcher_next_last_icache_miss_mem_req_dbg(prefetcher_next_last_icache_miss_mem_req_dbg),
        .prefetcher_addr_incrementor_dbg(prefetcher_addr_incrementor_dbg),
        .prefetcher_next_addr_incrementor_dbg(prefetcher_next_addr_incrementor_dbg),
        // Logic block debug outputs
        .mem_req_addr_dbg(mem_req_addr_dbg),
        .snooping_found_icache_dbg(snooping_found_icache_dbg),
        .new_mshr_entry_dbg(new_mshr_entry_dbg)
    );

    // icache access memory only
    // needs to be decided by arbitrator later when dcache is done
    always_comb begin
        // Using fake fetch - only handle data memory operations
        if (mem_req_addr.valid) begin
            proc2mem_command = MEM_LOAD;
            proc2mem_addr    = mem_req_addr.addr;
        end else begin
            proc2mem_command = MEM_NONE;
            proc2mem_addr    = '0;
        end
`ifndef CACHE_MODE
        proc2mem_size    = '0; // data size sent to memory
`endif
        proc2mem_data    = '0; // data sent to memory, no memory write for instruciotn
    end

    // In this simplified model, memory always accepts valid requests
    assign mem_req_accepted = (proc2mem_command == MEM_LOAD) && (mem2proc_transaction_tag != 0);

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
    logic [`IB_IDX_BITs-1:0] ib_free_slots;

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
        .train_req_i(train_req),

        // Recover request (from ROB retire stage)
        .recover_req_i(recover_req)
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
        .reset(reset),
        .flush(mispredict),  // Flush on branch mispredict

        // Fetch interface (temporary fake fetch)
        .new_ib_entry(fetch_packet),
        .num_pushes(),
        .available_slots(ib_free_slots),

        // Dispatch interface
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
    generate
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
    endgenerate

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

        // To Fetch
        .dispatch_count(dispatch_count),

        // TO ROB
        .rob_entry_packet(rob_entry_packet),

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
        .head_entries(rob_head_entries),
        .head_idxs(rob_head_idxs),
        .head_valids(rob_head_valids)
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

        // Debug outputs
        .fu_results_dbg(fu_results_dbg),
        .prf_read_en_src1_dbg(prf_read_en_src1_dbg),
        .prf_read_en_src2_dbg(prf_read_en_src2_dbg),
        .prf_read_tag_src1_dbg(prf_read_tag_src1_dbg),
        .prf_read_tag_src2_dbg(prf_read_tag_src2_dbg),
        .resolved_src1_dbg(resolved_src1_dbg),
        .resolved_src2_dbg(resolved_src2_dbg),
        .mult_start_dbg(mult_start_dbg),
        .mult_done_dbg(mult_done_dbg),
        .branch_take_dbg(branch_take_dbg),
        .branch_target_dbg(branch_target_dbg),
        .alu_executing_dbg(alu_executing_dbg),
        .alu_func_dbg(alu_func_dbg),
        .mult_executing_dbg(mult_executing_dbg),
        .branch_executing_dbg(branch_executing_dbg),
        .mem_executing_dbg(mem_executing_dbg)
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

        .retire_commits_dbg(retire_commits_dbg),


        // To fetch
        .branch_target_out(correct_branch_target),

        // Branch predictor
        .train_req_o            (train_req),
        .recover_req_o          (recover_req),

        // From PRF for committed data
        .regfile_entries(regfile_entries_dbg),

        // From arch map table for freelist restore on mispredict
        .arch_table_snapshot(arch_table_snapshot),

        // To freelist: restore mask on mispredict
        .freelist_restore_mask(freelist_restore_mask)
    );

    //////////////////////////////////////////////////
    //                                              //
    //               Pipeline Outputs               //
    //                                              //
    //////////////////////////////////////////////////

    // Output the committed instructions to the testbench for counting
    // For superscalar, show the oldest ready instruction (whether retired or not)
    assign committed_insts = retire_commits_dbg;


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
    assign fu_gnt_bus_dbg         = cdb_0.grant_bus_out;
    assign mult_request_dbg       = mult_request;

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
    assign ib_bundle_valid_dbg   = ib_bundle_valid;

endmodule  // cpu
