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
    input clock, // System clock
    input reset, // System reset

    input MEM_TAG   mem2proc_transaction_tag, // Memory tag for current transaction
    input MEM_BLOCK mem2proc_data,            // Data coming back from memory
    input MEM_TAG   mem2proc_data_tag,        // Tag for which transaction data is for

    output MEM_COMMAND proc2mem_command, // Command sent to memory
    output ADDR        proc2mem_addr,    // Address sent to memory
    output MEM_BLOCK   proc2mem_data,    // Data sent to memory
    output MEM_SIZE    proc2mem_size,    // Data size sent to memory

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

    // Pipeline register enables
    logic issue_execute_enable;

    // Outputs from ID stage and ID/EX Pipeline Register
    ISSUE_EXECUTE_PACKET issue_execute_packet, issue_execute_register;

    // CDB wires
    logic [`NUM_FU_BRANCH-1:0] cdb_branch_requests;
    logic [`NUM_FU_ALU-1:0]    cdb_alu_requests;
    logic [`NUM_FU_MEM-1:0]    cdb_mem_requests;
    logic [`NUM_FU_MULT-1:0]   cdb_mult_requests;

    logic [`NUM_FU_BRANCH-1:0] cdb_branch_grants;
    logic [`NUM_FU_ALU-1:0]    cdb_alu_grants;
    logic [`NUM_FU_MEM-1:0]    cdb_mem_grants;
    logic [`NUM_FU_MULT-1:0]   cdb_mult_grants;

    CDB_ENTRY [`NUM_FU_TOTAL-1:0] cdb_fu_outputs;
    CDB_EARLY_TAG_ENTRY [`N-1:0]  cdb_early_tags;
    CDB_ENTRY [`N-1:0]             cdb_output;

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
        end else begin                      // read an INSTRUCTION from memory
            proc2mem_command = Imem_command;
            proc2mem_addr    = Imem_addr;
            proc2mem_size    = DOUBLE;      // instructions load a full memory line (64 bits)
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
    assign if_valid = start_valid_on_reset || wb_valid;

    //////////////////////////////////////////////////
    //                                              //
    //                  Fetch-Stage                 //
    //                                              //
    //////////////////////////////////////////////////

    stage_if stage_if_0 (
        // Inputs
        .clock (clock),
        .reset (reset),
        .if_valid      (if_valid),
        .take_branch   (ex_mem_reg.take_branch),
        .branch_target (ex_mem_reg.alu_result),
        .Imem_data     (mem2proc_data),

        .Imem2proc_transaction_tag(mem2proc_transaction_tag),
        .Imem2proc_data_tag       (mem2proc_data_tag),

        // Outputs
        .Imem_command  (Imem_command),
        .if_packet     (if_packet),
        .Imem_addr     (Imem_addr)
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

    stage_id stage_id_0 (
        // Inputs
        .clock (clock),
        .reset (reset),
        .if_id_reg       (if_id_reg),
        .wb_regfile_en   (wb_packet.valid),
        .wb_regfile_idx  (wb_packet.reg_idx),
        .wb_regfile_data (wb_packet.data),

        // Output
        .id_packet (id_packet)
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
    //           Reservation Station (RS)           //
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
    //                 Issue Stage                  //
    //                                              //
    //////////////////////////////////////////////////

    stage_ex stage_ex_0 (
        // Input
        .id_ex_reg (id_ex_reg),

        // Output
        .ex_packet (ex_packet)
    );

    //////////////////////////////////////////////////
    //                                              //
    //              Execute Stage                    //
    //                                              //
    //////////////////////////////////////////////////

    stage_ex stage_ex_0 (
        // Input
        .id_ex_reg (id_ex_reg),

        // Output
        .ex_packet (ex_packet)
    );

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
        .clock (clock),
        .reset (reset),

        // Arbiter inputs
        .branch_requests (cdb_branch_requests),
        .alu_requests    (cdb_alu_requests),
        .mem_requests    (cdb_mem_requests),
        .mult_requests   (cdb_mult_requests),

        // Arbiter outputs indicating which requests are granted
        .branch_grants (cdb_branch_grants),
        .alu_grants    (cdb_alu_grants),
        .mem_grants    (cdb_mem_grants),
        .mult_grants   (cdb_mult_grants),

        // CDB inputs from functional units
        .fu_outputs (cdb_fu_outputs),

        // CDB output indicating which tags should be awoken a cycle early
        .early_tags (cdb_early_tags),

        // CDB register outputs broadcasting to PRF, EX stage, and Map Table
        .cdb_output (cdb_output)
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

endmodule // pipeline
