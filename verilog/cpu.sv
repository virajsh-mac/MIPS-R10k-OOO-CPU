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

module cpu (
    input clock,  // System clock
    input reset,  // System reset

    input MEM_TAG   mem2proc_transaction_tag,  // Memory tag for current transaction
    input MEM_BLOCK mem2proc_data,             // Data coming back from memory
    input MEM_TAG   mem2proc_data_tag,         // Tag for which transaction data is for

    output MEM_COMMAND proc2mem_command,  // Command sent to memory
    output ADDR        proc2mem_addr,     // Address sent to memory
    output MEM_BLOCK   proc2mem_data,     // Data sent to memory
    output MEM_SIZE    proc2mem_size,     // Data size sent to memory

    // Note: these are assigned at the very bottom of the module
    output COMMIT_PACKET [`N-1:0] committed_insts,

    // Debug outputs: these signals are solely used for debugging in testbenches
    // Do not change for project 3
    // You should definitely change these for project 4
    output ADDR  if_NPC_dbg,
    output DATA  if_inst_dbg,
    output logic if_valid_dbg,
    output ADDR  if_id_NPC_dbg,
    output DATA  if_id_inst_dbg,
    output logic if_id_valid_dbg,
    output ADDR  id_ex_NPC_dbg,
    output DATA  id_ex_inst_dbg,
    output logic id_ex_valid_dbg,
    output ADDR  ex_mem_NPC_dbg,
    output DATA  ex_mem_inst_dbg,
    output logic ex_mem_valid_dbg,
    output ADDR  mem_wb_NPC_dbg,
    output DATA  mem_wb_inst_dbg,
    output logic mem_wb_valid_dbg
);

    //////////////////////////////////////////////////
    //                                              //
    //                Pipeline Wires                //
    //                                              //
    //////////////////////////////////////////////////
    // NOTE: organize this section by the module that outputs referenced wires

    logic                                    mispredict;

    // Pipeline register enables
    logic                                    issue_execute_enable;

    // Outputs from ID stage (decode)
    FETCH_DISP_PACKET                        fetch_disp_packet;

    // Outputs from Dispatch stage
    logic                   [$clog2(`N)-1:0] dispatch_count;
    RS_ALLOC_BANKS                           rs_alloc_from_dispatch;

    // ROB allocation signals
    ROB_ENTRY               [        `N-1:0] rob_entry_packet;

    // Free list allocation signals
    logic                   [        `N-1:0] free_alloc_valid;
    PHYS_TAG                [        `N-1:0] allocated_phys;

    // Map table communication packets
    MAP_TABLE_WRITE_REQUEST [        `N-1:0] maptable_write_reqs;
    MAP_TABLE_READ_REQUEST                   maptable_read_req;
    MAP_TABLE_READ_RESPONSE                  maptable_read_resp;

    // RS wires - structured by functional unit category
    RS_ALLOC_BANKS                           rs_alloc;
    RS_GRANTED_BANKS                         rs_granted;
    RS_BANKS                                 rs_banks;
    ISSUE_CLEAR                              issue_clear;

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

    // TODO: Connect these to actual functional unit outputs when FUs are implemented
    // Placeholder assignments for now
    assign cdb_requests   = '0;
    assign cdb_fu_outputs = '0;

    //////////////////////////////////////////////////
    //                                              //
    //                Memory Outputs                //
    //                                              //
    //////////////////////////////////////////////////

    // these signals go to and from the processor and memory
    // we give precedence to the mem stage over instruction fetch
    // note that there is no latency in project 3
    // but there will be a 100ns latency in project 4

    always_comb begin
        if (Dmem_command != MEM_NONE) begin  // read or write DATA from memory
            proc2mem_command = Dmem_command_filtered;
            proc2mem_size    = Dmem_size;
            proc2mem_addr    = Dmem_addr;
        end else begin  // read an INSTRUCTION from memory
            proc2mem_command = Imem_command;
            proc2mem_addr    = Imem_addr;
            proc2mem_size    = DOUBLE;  // instructions load a full memory line (64 bits)
        end
        proc2mem_data = Dmem_store_data;
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

    stage_if stage_if_0 (
        // Inputs
        .clock        (clock),
        .reset        (reset),
        .if_valid     (if_valid),
        .take_branch  (ex_mem_reg.take_branch),
        .branch_target(ex_mem_reg.alu_result),
        .Imem_data    (mem2proc_data),

        .Imem2proc_transaction_tag(mem2proc_transaction_tag),
        .Imem2proc_data_tag       (mem2proc_data_tag),

        // Outputs
        .Imem_command(Imem_command),
        .if_packet   (if_packet),
        .Imem_addr   (Imem_addr)
    );

    // debug outputs
    assign if_NPC_dbg   = if_packet.NPC;
    assign if_inst_dbg  = if_packet.inst;
    assign if_valid_dbg = if_packet.valid;

    //////////////////////////////////////////////////
    //                                              //
    //       Fetch/Dispatch Pipeline Register       //
    //                                              //
    //////////////////////////////////////////////////

    assign if_id_enable = !load_stall;

    always_ff @(posedge clock) begin
        if (reset) begin
            if_id_reg.inst  <= `NOP;
            if_id_reg.valid <= `FALSE;
            if_id_reg.NPC   <= 0;
            if_id_reg.PC    <= 0;
        end else if (if_id_enable) begin
            if_id_reg <= if_packet;
        end
    end

    // debug outputs
    assign if_id_NPC_dbg   = if_id_reg.NPC;
    assign if_id_inst_dbg  = if_id_reg.inst;
    assign if_id_valid_dbg = if_id_reg.valid;

    //////////////////////////////////////////////////
    //                                              //
    //                Dispatch-Stage                //
    //                                              //
    //////////////////////////////////////////////////


    // For now, convert id_packet to fetch_disp_packet format
    // TODO: Update stage_id to output FETCH_DISP_PACKET directly
    always_comb begin
        for (int i = 0; i < `N; i++) begin
            fetch_disp_packet.rs1_idx[i]       = id_packet.inst.r.rs1;
            fetch_disp_packet.rs2_idx[i]       = id_packet.inst.r.rs2;
            fetch_disp_packet.rd_idx[i]        = id_packet.dest_reg_idx;
            fetch_disp_packet.uses_rd[i]       = (id_packet.dest_reg_idx != `ZERO_REG);
            // TODO: Set op_type, opa_select, opb_select based on decode logic
            fetch_disp_packet.op_type[i]       = OP_ALU_ADD;  // Placeholder
            fetch_disp_packet.opa_select[i]    = id_packet.opa_select;
            fetch_disp_packet.opb_select[i]    = id_packet.opb_select;
            fetch_disp_packet.rs2_immediate[i] = '0;  // TODO: Extract immediate from instruction
            fetch_disp_packet.PC[i]            = id_packet.PC;
            fetch_disp_packet.inst[i]          = id_packet.inst;
            fetch_disp_packet.pred_taken[i]    = 1'b0;  // TODO: Add branch prediction
            fetch_disp_packet.pred_target[i]   = '0;
        end
    end

    // Dispatch stage
    stage_dispatch stage_dispatch_0 (
        .clock(clock),
        .reset(reset),

        // From decode
        .fetch_packet(fetch_disp_packet),
        .fetch_valid ({`N{id_packet.valid}}), // Replicate valid bit

        // From ROB/Freelist (TODO: connect these)
        .free_slots_rob    (16),  // Placeholder
        .free_slots_freelst(32),  // Placeholder
        .rob_alloc_idxs    ('0),  // Placeholder

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

        // TO FREE LIST (TODO: connect)
        .free_alloc_valid(free_alloc_valid),
        .allocated_phys  (allocated_phys),

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

    assign issue_execute_enable = '1;

    always_ff @(posedge clock) begin
        if (reset) begin
            // TODO make sure this is the correct way to reset the register
            issue_execute_register <= '0;
        end else if (issue_execute_enable) begin
            issue_execute_register <= issue_execute_packet;
        end
    end

    // debug outputs
    assign id_ex_NPC_dbg   = id_ex_reg.NPC;
    assign id_ex_inst_dbg  = id_ex_reg.inst;
    assign id_ex_valid_dbg = id_ex_reg.valid;

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

    stage_issue stage_issue_0 (
        .clock(clock),
        .reset(reset),
        .mispredict(mispredict),

        // RS entries (structured)
        .rs_banks(rs_banks),

        // FU availability grants (structured) - comes from CDB
        .fu_avails(cdb_grants),

        // Clear signals (structured)
        .issue_clear(issue_clear),

        // Issue outputs (structured)
        .issue_entries(issue_entries)
    );

    //////////////////////////////////////////////////
    //                                              //
    //                 Issue Stage                  //
    //                                              //
    //////////////////////////////////////////////////

    // Issue stage structured inputs/outputs
    ISSUE_ENTRIES issue_entries;
    ISSUE_CLEAR   issue_clear;

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
        .issue_entries(issue_entries)
    );

    // Extract clear signals from structured output for RS modules
    assign rs_clear_signals = issue_clear;

    //////////////////////////////////////////////////
    //                                              //
    //            Physical Register File            //
    //                                              //
    //////////////////////////////////////////////////

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


    //////////////////////////////////////////////////
    //                                              //
    //                Retire Stage                  //
    //                                              //
    //////////////////////////////////////////////////

    //////////////////////////////////////////////////
    //                                              //
    //               Pipeline Outputs               //
    //                                              //
    //////////////////////////////////////////////////

    // Output the committed instruction to the testbench for counting
    assign committed_insts[0] = wb_packet;

endmodule  // pipeline
