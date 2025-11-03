/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  cpu.sv                                              //
//                                                                     //
//  Description :  Top-level module of the verisimple processor;       //
//                 This instantiates and connects the 5 stages of the  //
//                 Verisimple pipeline together.                       //
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
    output MEM_SIZE    proc2mem_size,     // Data size sent to memory

    // Retire interface
    output COMMIT_PACKET [`N-1:0] committed_insts,

    // Fake-fetch interface
    input  DATA                     ff_instr       [`N-1:0],  // Instruction bundle from testbench
    input  ADDR                     ff_pc,                    // Current PC from testbench
    input  logic [$clog2(`N+1)-1:0] ff_nvalid,                // Number of valid instructions from testbench
    output logic [$clog2(`N+1)-1:0] ff_consumed,              // Number consumed by CPU
    output logic                    branch_taken_o,           // Branch taken signal to testbench
    output ADDR                     branch_target_o,          // Branch target to testbench

    // Debug outputs: these signals are solely used for debugging in testbenches
    // Adapted for OOO pipeline: fetch, dispatch, issue, execute, complete, retire
    output ADDR  fetch_NPC_dbg,
    output DATA  fetch_inst_dbg,
    output logic fetch_valid_dbg,
    output ADDR  dispatch_NPC_dbg,
    output DATA  dispatch_inst_dbg,
    output logic dispatch_valid_dbg,
    output ADDR  issue_NPC_dbg,
    output DATA  issue_inst_dbg,
    output logic issue_valid_dbg,
    output ADDR  execute_NPC_dbg,
    output DATA  execute_inst_dbg,
    output logic execute_valid_dbg,
    output ADDR  complete_NPC_dbg,
    output DATA  complete_inst_dbg,
    output logic complete_valid_dbg,
    output ADDR  retire_NPC_dbg,
    output DATA  retire_inst_dbg,
    output logic retire_valid_dbg,

    // Additional debug outputs for OOO processor debugging
    output logic [`N-1:0] rob_head_valids_dbg,
    output ROB_ENTRY [`N-1:0] rob_head_entries_dbg,
    output logic [$clog2(`N+1)-1:0] dispatch_count_dbg,
    output RS_GRANTED_BANKS rs_granted_dbg,
    output logic [`RS_ALU_SZ-1:0] rs_alu_ready_dbg,
    output ISSUE_ENTRIES issue_entries_dbg,

    // Execute stage debug outputs
    output logic [`N-1:0] ex_valid_dbg,
    output EX_COMPLETE_PACKET ex_comp_dbg,

    // Complete stage debug outputs
    output ROB_UPDATE_PACKET rob_update_packet_dbg
);

    //////////////////////////////////////////////////
    //                                              //
    //                Pipeline Wires                //
    //                                              //
    //////////////////////////////////////////////////
    // NOTE: organize this section by the module that outputs referenced wires

    // Pipeline register enables
    logic                                                                            issue_execute_enable;

    // Outputs from ID stage (decode)
    FETCH_DISP_PACKET                                                                fetch_disp_packet;

    // Outputs from Dispatch stage
    logic                   [                 $clog2(`N)-1:0]                        dispatch_count;
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
    RS_ENTRY            [   `RS_ALU_SZ-1:0] rs_alu_entries;
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

    // PRF read data now comes from the regfile instantiation below
    // For now, assume src2 data comes from CDB forwarding or immediates
    assign prf_read_data_src2 = '0;  // TODO: Implement proper src2 reading if needed

    // Retire stage signals
    ROB_ENTRY [`N-1:0] rob_head_entries;
    logic     [`N-1:0] rob_head_valids;
    ROB_IDX   [`N-1:0] rob_head_idxs;
    logic              rob_mispredict;
    ROB_IDX            rob_mispred_idx;
    logic              bp_recover_en;
    logic     [`N-1:0] arch_write_enables;
    REG_IDX   [`N-1:0] arch_write_addrs;
    PHYS_TAG  [`N-1:0] arch_write_phys_regs;

    // Global mispredict signal
    logic              mispredict;
    assign mispredict = rob_mispredict;

    // Memory interface placeholders (TODO: implement proper data memory stages)
    logic                        Dmem_command_filtered = MEM_NONE;
    MEM_SIZE                     Dmem_size = DOUBLE;
    ADDR                         Dmem_addr = '0;
    MEM_BLOCK                    Dmem_store_data = '0;

    // Arch map table signals
    MAP_ENTRY [`ARCH_REG_SZ-1:0] arch_table_snapshot;

    // CDB requests: single-cycle FUs request during issue, multi-cycle during execute
    assign cdb_requests.alu    = issue_cdb_requests.alu;  // From issue stage
    // TODO fix mult (for now its 0 to make things work)
    assign cdb_requests.mult   = mult_request;  // From execute stage (when completing)
    assign cdb_requests.branch = issue_cdb_requests.branch;  // From issue stage
    assign cdb_requests.mem    = issue_cdb_requests.mem;  // From issue stage

    // cdb_fu_outputs connected from execute stage via fu_outputs

    // Connect dispatch outputs to freelist inputs
    assign freelist_alloc_req  = free_alloc_valid;

    // Calculate freelist free slots (placeholder - TODO: get from freelist module)
    assign freelist_free_slots = 32;  // Placeholder

    // TODO Debug for PRF remove when synthesizing/ not needed anymore
    DATA [`PHYS_REG_SZ_R10K-1:0] regfile_entries;


    //////////////////////////////////////////////////
    //                                              //
    //                Memory Outputs                //
    //                                              //
    //////////////////////////////////////////////////

    // these signals go to and from the processor and memory
    // we give precedence to the mem stage over instruction fetch
    // there is a 100ns latency to memory 

    always_comb begin
        // Using fake fetch - only handle data memory operations
        proc2mem_command = Dmem_command_filtered;
        proc2mem_size    = Dmem_size;
        proc2mem_addr    = Dmem_addr;
        proc2mem_data    = Dmem_store_data;
    end

    //////////////////////////////////////////////////
    //                                              //
    //                  Valid Bit                   //
    //                                              //
    //////////////////////////////////////////////////

    // This state controls the stall signal that artificially forces IF
    // to stall until the previous instruction has completed.
    // For project 3, start by assigning if_valid to always be 1

    logic if_valid, start_valid_on_reset, wb_valid;


    always_ff @(posedge clock) begin
        // Start valid on reset. Other stages (ID,EX,MEM,WB) start as invalid
        // Using a separate always_ff is necessary since if_valid is combinational
        // Assigning if_valid = reset doesn't work as you'd hope :/
        start_valid_on_reset <= reset;
    end

    // valid bit will cycle through the pipeline and come back from the wb stage
    // For OOO, stall fetch when dispatch count is zero
    assign if_valid = (start_valid_on_reset || wb_valid) && (dispatch_count != 0);

    //////////////////////////////////////////////////
    //                                              //
    //                  Fetch-Stage                 //
    //                                              //
    //////////////////////////////////////////////////

    // Fetch stage removed - using fake fetch directly

    //////////////////////////////////////////////////
    //                                              //
    //       Fetch/Dispatch Pipeline Register       //
    //                                              //
    //////////////////////////////////////////////////

    // Pipeline register removed - using fake fetch directly

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

    // Instantiate decoders for each instruction in the bundle
    generate
        for (genvar i = 0; i < `N; i++) begin
            decoder decoder_i (
                .inst      (ff_instr[i]),
                .valid     (i < ff_nvalid),
                .opa_select(decode_opa_select[i]),
                .opb_select(decode_opb_select[i]),
                .has_dest  (decode_has_dest[i]),
                .op_type   (decode_op_type[i]),
                .csr_op    (decode_csr_op[i]),
                .halt      (decode_halt[i]),
                .illegal   (decode_illegal[i])
            );
        end
    endgenerate

    //////////////////////////////////////////////////
    //                                              //
    //                Dispatch-Stage                //
    //                                              //
    //////////////////////////////////////////////////


    // Convert ff_nvalid count to bit mask for dispatch stage
    always_comb begin
        fetch_valid_mask = '0;
        for (int i = 0; i < `N; i++) begin
            if (i < ff_nvalid) fetch_valid_mask[i] = 1'b1;
        end
    end

    // Build FETCH_DISP_PACKET from fake fetch inputs and decoder outputs
    always_comb begin
        for (int i = 0; i < `N; i++) begin
            // Register indices from instruction (RISC-V bit fields)
            fetch_disp_packet.rs1_idx[i] = ff_instr[i][19:15];  // rs1: bits 19-15
            fetch_disp_packet.rs2_idx[i] = ff_instr[i][24:20];  // rs2: bits 24-20
            fetch_disp_packet.rd_idx[i]  = ff_instr[i][11:7];   // rd: bits 11-7

            // Use destination flag from decoder
            fetch_disp_packet.uses_rd[i] = decode_has_dest[i] && (ff_instr[i][11:7] != `ZERO_REG);

            // Operation info from decoder
            fetch_disp_packet.op_type[i]    = decode_op_type[i];
            fetch_disp_packet.opa_select[i] = decode_opa_select[i];
            fetch_disp_packet.opb_select[i] = decode_opb_select[i];

            // Extract immediate based on operand B select
            case (decode_opb_select[i])
                OPB_IS_I_IMM: fetch_disp_packet.rs2_immediate[i] = `RV32_signext_Iimm(ff_instr[i]);
                OPB_IS_S_IMM: fetch_disp_packet.rs2_immediate[i] = `RV32_signext_Simm(ff_instr[i]);
                OPB_IS_B_IMM: fetch_disp_packet.rs2_immediate[i] = `RV32_signext_Bimm(ff_instr[i]);
                OPB_IS_U_IMM: fetch_disp_packet.rs2_immediate[i] = `RV32_signext_Uimm(ff_instr[i]);
                OPB_IS_J_IMM: fetch_disp_packet.rs2_immediate[i] = `RV32_signext_Jimm(ff_instr[i]);
                default:      fetch_disp_packet.rs2_immediate[i] = '0;
            endcase

            // PC and instruction info from fake fetch
            fetch_disp_packet.PC[i]          = ff_pc + 32'(4 * i);
            fetch_disp_packet.inst[i]        = ff_instr[i];

            // No branch prediction for now
            fetch_disp_packet.pred_taken[i]  = 1'b0;
            fetch_disp_packet.pred_target[i] = '0;
            fetch_disp_packet.halt[i]        = decode_halt[i];
        end
    end

    // Dispatch stage
    stage_dispatch stage_dispatch_0 (
        .clock(clock),
        .reset(reset),

        // From decode
        .fetch_packet(fetch_disp_packet),
        .fetch_valid (fetch_valid_mask),   // Convert count to bit mask

        // From ROB/Freelist
        .free_slots_rob    (rob_free_slots),
        .free_slots_freelst(freelist_free_slots),
        .rob_alloc_idxs    (rob_alloc_idxs),

        // From RS Banks: granted allocations (from current cycle)
        .rs_alu_granted(rs_granted.alu),
        .rs_mult_granted(rs_granted.mult),
        .rs_branch_granted(rs_granted.branch),
        .rs_mem_granted(rs_granted.mem),

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
        .reset(reset | rob_mispredict), // Reset on mispredict

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
        .entries        (rs_alu_entries),
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
    assign rs_banks.alu    = rs_alu_entries;
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
        .rs_alu_ready_dbg (rs_alu_ready),
        .issue_entries_dbg(issue_entries_debug)
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
        .gnt_bus(cdb_0.gnt_bus_out)
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
        .table_restore(arch_table_snapshot),
        .table_restore_en(bp_recover_en)
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
        .table_snapshot(arch_table_snapshot),
        .table_restore('0),  // Not used - arch table doesn't restore
        .table_restore_en(1'b0)  // Arch table never restores
    );

    //////////////////////////////////////////////////
    //                                              //
    //                  Freelist                     //
    //                                              //
    //////////////////////////////////////////////////

    freelist freelist_0 (
        .clock(clock),
        .reset(reset),

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
        .read_tags(prf_read_tag_src1),
        .read_data(prf_read_data_src1),

        // Write interface - directly from CDB
        .cdb_writes(cdb_output),

        .regfile_entries(regfile_entries)
    );

    //////////////////////////////////////////////////
    //                                              //
    //                    CDB                       //
    //                                              //
    //////////////////////////////////////////////////


    cdb cdb_0 (
        .clock(clock),
        .reset(reset),

        // Arbiter inputs (structured)
        .requests(cdb_requests),

        // Arbiter outputs indicating which requests are granted (structured)
        .grants(cdb_grants),

        // CDB inputs from functional units (structured)
        .fu_outputs(cdb_fu_outputs),

        // CDB output indicating which tags should be awoken a cycle early
        .early_tags(early_tag_broadcast),

        // CDB register outputs broadcasting to PRF, EX stage, and Map Table
        .cdb_output(cdb_output)
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

    stage_retire stage_retire_0 (
        .clock(clock),
        .reset(reset),

        // From ROB: head window (N-1 = oldest, 0 = youngest)
        .head_entries(rob_head_entries),
        .head_valids (rob_head_valids),
        .head_idxs   (rob_head_idxs),

        // To ROB: flush younger if head is a mispredicted branch
        .rob_mispredict (rob_mispredict),
        .rob_mispred_idx(rob_mispred_idx),

        // Global recovery pulse (tables react internally)
        .bp_recover_en(bp_recover_en),

        // To freelist: bitmap of PRs to free (all committed lanes' Told this cycle)
        .free_mask(freelist_free_mask),

        // To archMapTable: N write ports (commit multiple per cycle)
        .arch_write_enables  (arch_write_enables),
        .arch_write_addrs    (arch_write_addrs),
        .arch_write_phys_regs(arch_write_phys_regs)
    );

    //////////////////////////////////////////////////
    //                                              //
    //               Pipeline Outputs               //
    //                                              //
    //////////////////////////////////////////////////

    // Output the committed instructions to the testbench for counting
    // For superscalar, show the oldest ready instruction (whether retired or not)
   always_comb begin
        committed_insts = '0;
        for (int i = 0; i < `N; i++) begin
            if (rob_head_valids[i]) begin  // Instruction i is valid
                committed_insts[i] = '{
                    NPC: rob_head_entries[i].PC + 4,
                    data: regfile_entries[arch_table_snapshot[rob_head_entries[i].arch_rd]],  // TODO: Get actual result data
                    reg_idx: rob_head_entries[i].arch_rd,
                    halt: rob_head_entries[i].halt,
                    illegal: rob_head_entries[i].exception == ILLEGAL_INST,
                    valid: rob_head_entries[i].complete
                };
            end
        end
    end


    // Fake-fetch outputs
    assign ff_consumed          = dispatch_count;  // Number of instructions consumed by dispatch
    assign branch_taken_o       = 1'b0;  // TODO: implement branch resolution
    assign branch_target_o      = '0;  // TODO: implement branch target

    // Debug signal assignments for OOO pipeline stages

    // Fetch: fake-fetch inputs (instructions available to dispatch)
    assign fetch_NPC_dbg        = ff_pc;
    assign fetch_inst_dbg       = ff_instr[0];
    assign fetch_valid_dbg      = (ff_nvalid > 0);

    // Dispatch: instructions dispatched to ROB/RS (first dispatched instruction)
    assign dispatch_NPC_dbg     = fetch_disp_packet.PC[0];
    assign dispatch_inst_dbg    = fetch_disp_packet.inst[0];
    assign dispatch_valid_dbg   = fetch_valid_mask[0] && (dispatch_count > 0);

    // Issue: instructions issued from RS to execute (first issued instruction)
    assign issue_NPC_dbg        = issue_entries.alu[0].PC;  // RS_ENTRY has PC
    assign issue_inst_dbg       = '0;  // RS_ENTRY doesn't have inst
    assign issue_valid_dbg      = |rs_granted.alu;  // Any ALU RS granted

    // Execute: instructions currently executing (first executing instruction)
    assign execute_NPC_dbg      = '0;  // EX_COMPLETE_PACKET doesn't have PC
    assign execute_inst_dbg     = '0;  // EX_COMPLETE_PACKET doesn't have inst
    assign execute_valid_dbg    = ex_valid[0];  // First execution slot valid

    // Complete: instructions that just completed (from completion register)
    assign complete_NPC_dbg     = '0;  // EX_COMPLETE_PACKET doesn't have PC
    assign complete_inst_dbg    = '0;  // EX_COMPLETE_PACKET doesn't have inst
    assign complete_valid_dbg   = ex_comp_reg_valid[0];

    // Retire: instructions being retired (ROB head)
    assign retire_NPC_dbg       = rob_head_entries[`N-1].PC;
    assign retire_inst_dbg      = rob_head_entries[`N-1].inst;
    assign retire_valid_dbg     = rob_head_valids[`N-1] && rob_head_entries[`N-1].complete;

    // Additional debug outputs
    assign rob_head_valids_dbg  = rob_head_valids;
    assign rob_head_entries_dbg = rob_head_entries;
    assign dispatch_count_dbg   = dispatch_count;
    assign rs_granted_dbg       = rs_granted;
    assign rs_alu_ready_dbg     = rs_alu_ready;
    assign issue_entries_dbg    = issue_entries_debug;

    // Execute stage debug outputs
    assign ex_valid_dbg         = ex_valid;
    assign ex_comp_dbg          = ex_comp;

endmodule  // cpu
