/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  cpu_test.sv                                         //
//                                                                     //
//  Description :  Testbench module for the VeriSimpleV processor.     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"

// P4 TODO: Add your own debugging framework. Basic printing of data structures
//          is an absolute necessity for the project. You can use C functions
//          like in test/pipeline_print.c or just do everything in verilog.
//          Be careful about running out of space on CAEN printing lots of state
//          for longer programs (alexnet, outer_product, etc.)

// These link to the pipeline_print.c file in this directory, and are used below to print
// detailed output to the pipeline_output_file, initialized by open_pipeline_output_file()
import "DPI-C" function string decode_inst(int inst);
// Pipeline printing disabled for OOO processor
// import "DPI-C" function void open_pipeline_output_file(string file_name);
// import "DPI-C" function void print_header();
// import "DPI-C" function void print_cycles(int clock_count);
// import "DPI-C" function void print_stage(
//     int inst,
//     int npc,
//     int valid_inst
// );
// import "DPI-C" function void print_reg(
//     int wb_data,
//     int wb_idx,
//     int wb_en
// );
// import "DPI-C" function void print_membus(int proc2mem_command, int proc2mem_addr,
//                                           int proc2mem_data_hi, int proc2mem_data_lo);
// import "DPI-C" function void close_pipeline_output_file();


`define TB_MAX_CYCLES 10000


module testbench;
    // string inputs for loading memory and output files
    // run like: cd build && ./simv +MEMORY=../programs/mem/<my_program>.mem +OUTPUT=../output/<my_program>
    // this testbench will generate 4 output files based on the output
    // named OUTPUT.{out cpi, wb, ppln} for the memory, cpi, writeback, and pipeline outputs.
    string program_memory_file, output_name;
    string out_outfile, cpi_outfile, writeback_outfile;  //, pipeline_outfile;
    int out_fileno, cpi_fileno, wb_fileno;  // verilog uses integer file handles with $fopen and $fclose

    // variables used in the testbench
    logic clock;
    logic reset;
    logic [31:0] clock_count;  // also used for terminating infinite loops
    logic [31:0] instr_count;

    // Disconnected: proc2mem/mem2proc portions (fake-fetch)
    // MEM_COMMAND             proc2mem_command;
    // ADDR                    proc2mem_addr;
    // MEM_BLOCK               proc2mem_data;
    // MEM_TAG                 mem2proc_transaction_tag;
    // MEM_BLOCK               mem2proc_data;
    // MEM_TAG                 mem2proc_data_tag;
    // MEM_SIZE                proc2mem_size;

    COMMIT_PACKET [`N-1:0] committed_insts;
    EXCEPTION_CODE error_status = NO_ERROR;

    // OOO debug signals
    logic [`N-1:0] rob_head_valids;
    ROB_ENTRY [`N-1:0] rob_head_entries;
    ROB_IDX [`N-1:0] rob_head_idxs;
    logic [$clog2(`N+1)-1:0] dispatch_count;
    RS_GRANTED_BANKS rs_granted;

    // Additional debug for RS and issue state
    logic [`RS_ALU_SZ-1:0] rs_alu_ready;
    ISSUE_ENTRIES issue_entries;

    // RS request debug signals
    logic [`RS_ALU_SZ-1:0] rs_alu_requests;
    logic [`RS_MULT_SZ-1:0] rs_mult_requests;
    logic [`RS_BRANCH_SZ-1:0] rs_branch_requests;
    logic [`RS_MEM_SZ-1:0] rs_mem_requests;

    // TEMP: ALU clear signals
    logic [`NUM_FU_ALU-1:0] alu_clear_signals;

    // TEMP: allocator grants
    logic [`RS_ALU_SZ-1:0][`NUM_FU_ALU-1:0] grants_alu;

    // Execute stage debug signals
    logic [`N-1:0] ex_valid;
    EX_COMPLETE_PACKET ex_comp;

    // Complete stage debug signals
    ROB_UPDATE_PACKET rob_update_packet;

    // Debug Output PRF
    DATA [`PHYS_REG_SZ_R10K-1:0] regfile_entries;

    // Debug output Architecture map table
    MAP_ENTRY [`ARCH_REG_SZ-1:0] arch_table_snapshot;

    // debug output rs_alu
    RS_ENTRY [`RS_ALU_SZ-1:0] rs_alu_entries;

    // Additional RS debug outputs
    RS_ENTRY [`RS_MULT_SZ-1:0] rs_mult_entries;
    RS_ENTRY [`RS_BRANCH_SZ-1:0] rs_branch_entries;
    RS_ENTRY [`RS_MEM_SZ-1:0] rs_mem_entries;

    // Map table debug output
    MAP_ENTRY [`ARCH_REG_SZ-1:0] map_table_snapshot;

    // Freelist debug output (available physical registers)
    logic [`PHYS_REG_SZ_R10K-1:0] freelist_available;

    // Freelist restore mask debug
    logic [`PHYS_REG_SZ_R10K-1:0] freelist_restore_mask;

    // CDB debug outputs
    CDB_ENTRY [`N-1:0] cdb_output;
    logic [`N-1:0][`NUM_FU_TOTAL-1:0] cdb_gnt_bus;
    FU_REQUESTS cdb_requests;
    CDB_FU_OUTPUTS cdb_fu_outputs;
    logic [`NUM_FU_TOTAL-1:0] cdb_grants_flat;
    CDB_EARLY_TAG_ENTRY [`N-1:0] cdb_early_tags;

    // Instruction Buffer debug output
    FETCH_PACKET [`N-1:0] ib_output_disp_wdn;

    // Store Queue entry packet output
    STOREQ_ENTRY [`N-1:0] sq_entry_packet;

    // Store -> DCache
    logic sq_to_dcache_valid;
    ADDR  sq_to_dcache_addr;
    DATA  sq_to_dcache_data;

    // store forwarding
    logic      [`NUM_FU_MEM-1:0] sq_forward_valid;
    DATA       [`NUM_FU_MEM-1:0] sq_forward_data;
    logic      [`NUM_FU_MEM-1:0] sq_forward_stall;

    // Issue clear signals debug output
    RS_CLEAR_SIGNALS rs_clear_signals;

    // Fetch stage debug signals
    FETCH_PACKET [3:0] fetch_packet_out;
    logic ib_bundle_valid;


    MEM_TAG mem2proc_transaction_tag;  // Memory tag for current transaction
    MEM_BLOCK mem2proc_data;  // Data coming back from memory
    MEM_TAG mem2proc_data_tag;

    MEM_COMMAND proc2mem_command;  // Command sent to memory
    ADDR proc2mem_addr;  // Address sent to memory
    MEM_BLOCK proc2mem_data;  // Data sent to memory
    MEM_SIZE proc2mem_size;  // Data size sent to memory

    // debug to expose DCache to testbench
    D_CACHE_LINE [`DCACHE_LINES-1:0]      cache_lines_debug;

    // Instantiate the Pipeline
    cpu verisimpleV (
        // Inputs
        .clock(clock),
        .reset(reset),

        .mem2proc_transaction_tag(mem2proc_transaction_tag),
        .mem2proc_data           (mem2proc_data),
        .mem2proc_data_tag       (mem2proc_data_tag),

        .proc2mem_command(proc2mem_command),
        .proc2mem_addr   (proc2mem_addr),
        .proc2mem_data   (proc2mem_data),
`ifndef CACHE_MODE
        .proc2mem_size   (proc2mem_size),
`endif

        .committed_insts(committed_insts),

        // Additional debug outputs
        .rob_head_valids_dbg(rob_head_valids),
        .rob_head_entries_dbg(rob_head_entries),
        .rob_head_idxs_dbg(rob_head_idxs),
        .dispatch_count_dbg(dispatch_count),
        .rs_granted_dbg(rs_granted),
        .rs_alu_ready_dbg(rs_alu_ready),
        .issue_entries_dbg(issue_entries),
        .rs_alu_requests_dbg(rs_alu_requests),
        .rs_mult_requests_dbg(rs_mult_requests),
        .rs_branch_requests_dbg(rs_branch_requests),
        .rs_mem_requests_dbg(rs_mem_requests),
        .alu_clear_signals_dbg(alu_clear_signals),
        .grants_alu_dbg(grants_alu),

        // Execute stage debug outputs
        .ex_valid_dbg(ex_valid),
        .ex_comp_dbg (ex_comp),

        // Complete stage debug outputs
        .rob_update_packet_dbg(rob_update_packet),

        // Debug output from PRF
        .regfile_entries_dbg(regfile_entries),

        // Debug output from architecture map table
        .arch_table_snapshot_dbg(arch_table_snapshot),

        // Debug output from RS_ALU
        .rs_alu_entries_dbg(rs_alu_entries),

        // Additional RS debug outputs
        .rs_mult_entries_dbg(rs_mult_entries),
        .rs_branch_entries_dbg(rs_branch_entries),
        .rs_mem_entries_dbg(rs_mem_entries),

        // Map table debug output
        .map_table_snapshot_dbg(map_table_snapshot),

        // Freelist debug output
        .freelist_available_dbg(freelist_available),
        .freelist_restore_mask_dbg(freelist_restore_mask),

        // CDB debug outputs
        .cdb_output_dbg(cdb_output),
        .cdb_gnt_bus_dbg(cdb_gnt_bus),
        .cdb_requests_dbg(cdb_requests),
        .cdb_fu_outputs_dbg(cdb_fu_outputs),
        .cdb_grants_flat_dbg(cdb_grants_flat),
        .cdb_early_tags_dbg(cdb_early_tags),

        // Instruction Buffer outputs
        .ib_output_disp_wdn_dbg(ib_output_disp_wdn),

        // store queue outputs
        .sq_entry_packet_dbg(sq_entry_packet),

        // store queue -> dcache
        .sq_to_dcache_valid_dbg(sq_to_dcache_valid),
        .sq_to_dcache_addr_dbg(sq_to_dcache_addr),
        .sq_to_dcache_data_dbg(sq_to_dcache_data),

        // store forwarding
        .sq_forward_valid_dbg(sq_forward_valid),
        .sq_forward_data_dbg(sq_forward_data),
        .sq_forward_stall_dbg(sq_forward_stall),

        // Issue clear signals debug output
        .rs_clear_signals_dbg(rs_clear_signals),

        // Fetch stage debug outputs
        .fetch_packet_dbg(fetch_packet_out),

        // debug to expose DCache to testbench
        .cache_lines_dbg(cache_lines_debug)
    );

    // Instruction Memory (for fake-fetch only - data operations disconnected)
    mem memory (
        // Only connect clock for initialization
        .clock           (clock),
        // Data operations disconnected
        .proc2mem_command(proc2mem_command),
        .proc2mem_addr   (proc2mem_addr),
        .proc2mem_data   (proc2mem_data),
`ifndef CACHE_MODE
        .proc2mem_size   (proc2mem_size),
`endif

        .mem2proc_transaction_tag(mem2proc_transaction_tag),
        .mem2proc_data           (mem2proc_data),
        .mem2proc_data_tag       (mem2proc_data_tag)
    );


    // Generate System Clock
    always begin
        #(`CLOCK_PERIOD / 2.0);
        clock = ~clock;
    end

    // ----------------------------------------------------------------
    // Read a 32b instruction from unified memory at byte address 'addr'
    // ----------------------------------------------------------------
    function DATA get_inst32(input ADDR addr);
        MEM_BLOCK blk;
        begin
            blk = memory.unified_memory[addr[31:3]];  // 8B-aligned line
            get_inst32 = blk.word_level[addr[2]];  // 0: low word, 1: high word
        end
    endfunction

    initial begin
        $dumpfile("cpu_test.vcd");
        $dumpvars(0, testbench);

        $display("\n---- Starting CPU Testbench (Fake-Fetch) ----\n");

        // set paramterized strings, see comment at start of module
        if ($value$plusargs("MEMORY=%s", program_memory_file)) begin
            $display("Using memory file  : %s", program_memory_file);
        end else begin
            $display("Did not receive '+MEMORY=' argument. Exiting.\n");
            $finish;
        end
        if ($value$plusargs("OUTPUT=%s", output_name)) begin
            $display("Using output files : %s.{out, cpi, wb}", output_name);
            out_outfile       = {output_name, ".out"};  // this is how you concatenate strings in verilog
            cpi_outfile       = {output_name, ".cpi"};
            writeback_outfile = {output_name, ".wb"};
            // pipeline_outfile  = {output_name, ".ppln"};
        end else begin
            $display("\nDid not receive '+OUTPUT=' argument. Exiting.\n");
            $finish;
        end

        clock = 1'b0;
        reset = 1'b0;

        $display("\n  %16t : Asserting Reset", $realtime);
        reset = 1'b1;

        @(posedge clock);
        @(posedge clock);

        $display("  %16t : Loading Unified Memory", $realtime);
        // load the compiled program's hex data into the memory module
        $readmemh(program_memory_file, memory.unified_memory);

        @(posedge clock);
        @(posedge clock);
        #1;  // This reset is at an odd time to avoid the pos & neg clock edges
        $display("  %16t : Deasserting Reset", $realtime);
        reset = 1'b0;

        wb_fileno = $fopen(writeback_outfile);
        $fdisplay(wb_fileno, "Register writeback output (hexadecimal)");

        // Pipeline output disabled for OOO processor
        // open_pipeline_output_file(pipeline_outfile);
        // print_header();

        out_fileno = $fopen(out_outfile);

        $display("  %16t : Running Processor", $realtime);
    end


    always @(negedge clock) begin
        if (reset) begin
            // Count the number of cycles and number of instructions committed
            clock_count = 0;
            instr_count = 0;
        end else begin
            #2;  // wait a short time to avoid a clock edge


            clock_count = clock_count + 1;

            if ((clock_count % 10000) == 0) $display("  %16t : %0d cycles", $realtime, clock_count);

            // Optional: peek at fake-fetch behavior
            // $display("%0t [FF] pc=%h consumed=%0d br=%0d tgt=%h", $time, fake_pc, fake_consumed, ff_branch_taken,
            //          ff_branch_target);

            // Pipeline printing disabled for OOO processor
            // print_cycles(clock_count - 1);
            // print_stage(if_inst_dbg, if_NPC_dbg, {31'b0, if_valid_dbg});  // Fetch
            // print_stage(if_id_inst_dbg, if_id_NPC_dbg, {31'b0, if_id_valid_dbg});  // Dispatch
            // print_stage(id_ex_inst_dbg, id_ex_NPC_dbg, {31'b0, id_ex_valid_dbg});  // Issue
            // print_stage(ex_mem_inst_dbg, ex_mem_NPC_dbg, {31'b0, ex_mem_valid_dbg});  // Execute
            // print_stage(mem_wb_inst_dbg, mem_wb_NPC_dbg, {31'b0, mem_wb_valid_dbg});  // Complete/Retire
            // print_reg(committed_insts[0].data, {27'b0, committed_insts[0].reg_idx}, {31'b0, committed_insts[0].valid});
            // print_membus({30'b0,proc2mem_command}, proc2mem_addr[31:0],
            //              proc2mem_data[63:32], proc2mem_data[31:0]);

            print_custom_data();

            output_reg_writeback_and_maybe_halt();


            // Optional: print CDB broadcasts
            // for (integer i = 0; i < `N; i++) begin
            //     $display("  CDB[%0d]: valid=%b, tag=%0d, data=%h", i, cdb_output[i].valid,
            //              cdb_output[i].tag, cdb_output[i].data);
            // end


            // stop the processor
            if (error_status != NO_ERROR || clock_count > `TB_MAX_CYCLES) begin

                $display("  %16t : Processor Finished", $realtime);

                // close the writeback output file (pipeline output disabled)
                // close_pipeline_output_file();
                $fclose(wb_fileno);

                // display the final memory and status
                show_final_mem_and_status(error_status);
                // output the final CPI
                output_cpi_file();

                $display("\n---- Finished CPU Testbench (Fake-Fetch) ----\n");

                #100 $finish;
            end
        end  // if(reset)
    end


    // Task to output register writeback data and potentially halt the processor.
    task output_reg_writeback_and_maybe_halt;
        ADDR pc;
        DATA inst;
        MEM_BLOCK block;
        for (int n = 0; n < `N; n++) begin
            if (committed_insts[n].valid) begin
                // update the count for every committed instruction
                instr_count = instr_count + 1;

                pc = committed_insts[n].NPC - 4;
                block = memory.unified_memory[pc[31:3]];
                inst = block.word_level[pc[2]];
                // print the committed instructions to the writeback output file
                if (committed_insts[n].reg_idx == `ZERO_REG) begin
                    $fdisplay(wb_fileno, "PC %4x:%-8s| ---", pc, decode_inst(inst));
                end else begin
                    $fdisplay(wb_fileno, "PC %4x:%-8s| r%02d=%-8x", pc, decode_inst(inst), committed_insts[n].reg_idx,
                              committed_insts[n].data);
                end

                // if (committed_insts[n].reg_idx == `ZERO_REG) begin
                //     $fdisplay(wb_fileno, "@%-8d PC %4x:%-8s| ---", clock_count, pc, decode_inst(inst));
                // end else begin
                //     $fdisplay(wb_fileno, "@%-8d PC %4x:%-8s| r%02d=%-8x", clock_count, pc, decode_inst(inst), committed_insts[n].reg_idx,
                //               committed_insts[n].data);
                // end

                // exit if we have an illegal instruction or a halt
                if (committed_insts[n].illegal) begin
                    error_status = ILLEGAL_INST;
                    break;
                end else if (committed_insts[n].halt) begin
                    error_status = HALTED_ON_WFI;
                    break;
                end
            end  // if valid
        end
    endtask  // task output_reg_writeback_and_maybe_halt


    // Task to output the final CPI and # of elapsed clock edges
    task output_cpi_file;
        real cpi;
        begin
            cpi = $itor(clock_count) / instr_count;  // must convert int to real
            cpi_fileno = $fopen(cpi_outfile);
            $fdisplay(cpi_fileno, "@@@  %0d cycles / %0d instrs = %f CPI", clock_count, instr_count, cpi);
            $fdisplay(cpi_fileno, "@@@  %4.2f ns total time to execute", clock_count * `CLOCK_PERIOD);
            $fclose(cpi_fileno);
        end
    endtask  // task output_cpi_file


    // Show contents of Unified Memory in both hex and decimal
    // Also output the final processor status
    task show_final_mem_and_status;
        input EXCEPTION_CODE final_status;
        int showing_data;
        logic [63:0] value;
        logic [`DTAG_BITS-1:0] addr_tag;
        int offset;
        int i;
        begin
            $fdisplay(out_fileno, "\nFinal memory state and exit status:\n");
            $fdisplay(out_fileno, "@@@ Unified Memory contents hex on left, decimal on right: ");
            $fdisplay(out_fileno, "@@@");
            showing_data = 0;

            for (int k = 0; k <= `MEM_64BIT_LINES - 1; k = k + 1) begin
                // compute tag & offset for fully associative cache
                addr_tag = k; // for simplicity, assume 1:1 mapping to cache line index for 64-bit words
                value = memory.unified_memory[k]; // default to memory

                // search all cache lines
                for (i = 0; i < `DCACHE_LINES; i++) begin
                    if (cache_lines_debug[i].valid && cache_lines_debug[i].dirty && cache_lines_debug[i].tag == addr_tag) begin
                        value = cache_lines_debug[i].data; // override with cache content
                        break;
                    end
                end

                if (value != 0) begin
                    $fdisplay(out_fileno, "@@@ mem[%5d] = %x : %0d", k * 8, value, value);
                    showing_data = 1;
                end else if (showing_data != 0) begin
                    $fdisplay(out_fileno, "@@@");
                    showing_data = 0;
                end
            end

            $fdisplay(out_fileno, "@@@");

            case (final_status)
                LOAD_ACCESS_FAULT: $fdisplay(out_fileno, "@@@ System halted on memory error");
                HALTED_ON_WFI:     $fdisplay(out_fileno, "@@@ System halted on WFI instruction");
                ILLEGAL_INST:      $fdisplay(out_fileno, "@@@ System halted on illegal instruction");
                default:           $fdisplay(out_fileno, "@@@ System halted on unknown error code %x", final_status);
            endcase
            $fdisplay(out_fileno, "@@@");
            $fclose(out_fileno);
        end
    endtask




    // OPTIONAL: Print our your data here
    // It will go to the $program.log file
    // task print_custom_data;
    //     $display(
    //         "%3d: ROB head valid=%b | Dispatch count=%0d | RS ready=%b | Issue ALU valid=%b | Execute valid=%b | ROB update valid=%b | Committed halt=%b valid=%b",
    //         clock_count - 1, rob_head_valids, dispatch_count, rs_alu_ready, issue_entries.alu[0].valid, ex_valid,
    //         rob_update_packet.valid, committed_insts[0].halt, committed_insts[0].valid);
    // endtask

    task print_custom_data;
        integer prf_count;
        integer i;
        int fu;
        int line;

        $display("\n=================================================================================");
        $display("CPU STATE SNAPSHOT - Cycle %0d", clock_count - 1);
        $display("=================================================================================");

        // ========================================
        // Fetch Packet
        // ========================================

        $display("\nFetch Packet Output (4-wide bundle):");
        // INV SIG
        // $display("  IB Bundle Valid: %b", ib_bundle_valid);
        for (integer fp_idx = 0; fp_idx < 4; fp_idx++) begin
            $display("  Packet[%0d]:", fp_idx);
            $display("    Valid:          %b", fetch_packet_out[fp_idx].valid);
            if (fetch_packet_out[fp_idx].valid) begin
                $display("    PC:             0x%08h", fetch_packet_out[fp_idx].pc);
                $display("    Instruction:    0x%08h", fetch_packet_out[fp_idx].inst);
                $display("    Is Branch:      %b", fetch_packet_out[fp_idx].is_branch);
                if (fetch_packet_out[fp_idx].is_branch) begin
                    $display("    BP Pred Taken:  %b", fetch_packet_out[fp_idx].bp_pred_taken);
                    $display("    BP Pred Target: 0x%08h", fetch_packet_out[fp_idx].bp_pred_target);
                    $display("    BP GHR Snapshot: 0b%07b", fetch_packet_out[fp_idx].bp_ghr_snapshot);
                end
            end
        end


        // ========================================
        // Instruction Buffer Debug (output to Dispatch)
        // ========================================
        $display("\n--- INSTRUCTION BUFFER OUTPUT TO DISPATCH ---");
        for (i = 0; i < `N; i++) begin
            if (ib_output_disp_wdn[i].valid) begin
                $display(
                    "  IB_OUT[%0d]: PC=0x%08h | Inst=0x%08h | Branch=%b | PredTaken=%b | PredTarget=0x%08h | GHR=%0b",
                    i,
                    ib_output_disp_wdn[i].pc,
                    ib_output_disp_wdn[i].inst,
                    ib_output_disp_wdn[i].is_branch,
                    ib_output_disp_wdn[i].bp_pred_taken,
                    ib_output_disp_wdn[i].bp_pred_target,
                    ib_output_disp_wdn[i].bp_ghr_snapshot
                );
            end else begin
                $display("  IB_OUT[%0d]: INVALID", i);
            end
        end


        // ========================================
        // Store Queue Debug (entries coming from Dispatch)
        // ========================================
        $display("\n--- STORE QUEUE DISPATCH PACKET ---");
        for (i = 0; i < `N; i++) begin
            if (sq_entry_packet[i].valid) begin
                $display(
                    "  SQ_IN[%0d]: addr=0x%08h | data=0x%08h | rob_idx=%0d",
                    i,
                    sq_entry_packet[i].address,
                    sq_entry_packet[i].data,
                    sq_entry_packet[i].rob_idx
                );
            end else begin
                $display("  SQ_IN[%0d]: INVALID", i);
            end
        end

        // =====================================
        // 1) Store Queue -> DCache (retire)
        // =====================================
        $display("--- SQ -> DCache ---");
        $display("  sq_to_dcache_valid = %0d", sq_to_dcache_valid);
        if (sq_to_dcache_valid) begin
            $display("  sq_to_dcache_addr  = 0x%08h", sq_to_dcache_addr);
            $display("  sq_to_dcache_data  = 0x%08h", sq_to_dcache_data);
        end

        // =====================================
        // 2) Store Forwarding (per MEM FU)
        // =====================================
        $display("--- Store Forwarding (SQ -> MEM FUs) ---");
        for (fu = 0; fu < `NUM_FU_MEM; fu++) begin
            if (sq_forward_valid[fu] || sq_forward_stall[fu]) begin
                $display("  FU%0d: fwd_valid=%0d fwd_stall=%0d fwd_data=0x%08h",
                            fu,
                            sq_forward_valid[fu],
                            sq_forward_stall[fu],
                            sq_forward_data[fu]);
            end
        end
        

        // RESERVATION STATIONS
        $display("\n--- RESERVATION STATIONS ---");

        // Show RS requests (ready entries)
        $display("RS Requests (ready entries):");
        $display("  ALU requests: %b", rs_alu_requests);
        $display("  MULT requests: %b", rs_mult_requests);
        $display("  BRANCH requests: %b", rs_branch_requests);
        $display("  MEM requests: %b", rs_mem_requests);

        // ALU RS
        $display("\nALU RS (%0d entries):", `RS_ALU_SZ);
        for (integer i = 0; i < `RS_ALU_SZ; i++) begin
            if (rs_alu_entries[i].valid) begin
                $display("  ALU_RS[%0d]: PC=0x%08h | OP=%0d | Dest=P%0d | Src1=P%0d(%b) | Src2=P%0d(%b) | ROB=%0d", i,
                         rs_alu_entries[i].PC, rs_alu_entries[i].op_type.category, rs_alu_entries[i].dest_tag,
                         rs_alu_entries[i].src1_tag, rs_alu_entries[i].src1_ready, rs_alu_entries[i].src2_tag,
                         rs_alu_entries[i].src2_ready, rs_alu_entries[i].rob_idx);
            end else begin
                $display("  ALU_RS[%0d]: EMPTY", i);
            end
        end

        // MULT RS
        $display("\nMULT RS (%0d entries):", `RS_MULT_SZ);
        for (integer i = 0; i < `RS_MULT_SZ; i++) begin
            if (rs_mult_entries[i].valid) begin
                $display("  MULT_RS[%0d]: PC=0x%08h | OP=%0d | Dest=P%0d | Src1=P%0d(%b) | Src2=P%0d(%b) | ROB=%0d", i,
                         rs_mult_entries[i].PC, rs_mult_entries[i].op_type.category, rs_mult_entries[i].dest_tag,
                         rs_mult_entries[i].src1_tag, rs_mult_entries[i].src1_ready, rs_mult_entries[i].src2_tag,
                         rs_mult_entries[i].src2_ready, rs_mult_entries[i].rob_idx);
            end else begin
                $display("  MULT_RS[%0d]: EMPTY", i);
            end
        end

        // BRANCH RS
        $display("\nBRANCH RS (%0d entries):", `RS_BRANCH_SZ);
        for (integer i = 0; i < `RS_BRANCH_SZ; i++) begin
            if (rs_branch_entries[i].valid) begin
                $display("  BR_RS[%0d]: PC=0x%08h | OP=%0d | Dest=P%0d | Src1=P%0d(%b) | Src2=P%0d(%b) | ROB=%0d", i,
                         rs_branch_entries[i].PC, rs_branch_entries[i].op_type.category, rs_branch_entries[i].dest_tag,
                         rs_branch_entries[i].src1_tag, rs_branch_entries[i].src1_ready, rs_branch_entries[i].src2_tag,
                         rs_branch_entries[i].src2_ready, rs_branch_entries[i].rob_idx);
            end else begin
                $display("  BR_RS[%0d]: EMPTY", i);
            end
        end

        // MEM RS
        $display("\nMEM RS (%0d entries):", `RS_MEM_SZ);
        for (integer i = 0; i < `RS_MEM_SZ; i++) begin
            if (rs_mem_entries[i].valid) begin
                $display("  MEM_RS[%0d]: PC=0x%08h | OP=%0d | Dest=P%0d | Src1=P%0d(%b) | Src2=P%0d(%b) | ROB=%0d", i,
                         rs_mem_entries[i].PC, rs_mem_entries[i].op_type.category, rs_mem_entries[i].dest_tag,
                         rs_mem_entries[i].src1_tag, rs_mem_entries[i].src1_ready, rs_mem_entries[i].src2_tag,
                         rs_mem_entries[i].src2_ready, rs_mem_entries[i].rob_idx);
            end else begin
                $display("  MEM_RS[%0d]: EMPTY", i);
            end
        end

        // ISSUE STAGE
        $display("\n--- ISSUE STAGE ---");
        $display("RS Grants - ALU: %b | MULT: %b | BRANCH: %b | MEM: %b", rs_granted.alu, rs_granted.mult, rs_granted.branch,
                 rs_granted.mem);

        $display("ISSUE REGISTER (currently issued to execute):");
        $display("  ALU: valid[0]=%b valid[1]=%b valid[2]=%b", issue_entries.alu[0].valid, issue_entries.alu[1].valid,
                 issue_entries.alu[2].valid);
        $display("  MULT: valid[0]=%b", issue_entries.mult[0].valid);
        $display("  BRANCH: valid[0]=%b", issue_entries.branch[0].valid);
        $display("  MEM: valid[0]=%b", issue_entries.mem[0].valid);

        $display("RS CLEARS (immediate, follow grants):");
        $display("  ALU: %b | MULT: %b | BRANCH: %b | MEM: %b", rs_clear_signals.valid_alu, rs_clear_signals.valid_mult,
                 rs_clear_signals.valid_branch, rs_clear_signals.valid_mem);

        $display("TEMP DEBUG - ALU Clear Signals from CDB: %b", alu_clear_signals);
        $display("TEMP DEBUG - Grants ALU[0]: %b", grants_alu[0]);
        $display("TEMP DEBUG - Grants ALU[1]: %b", grants_alu[1]);
        $display("TEMP DEBUG - Grants ALU[2]: %b", grants_alu[2]);
        $display("TEMP DEBUG - Grants ALU[3]: %b", grants_alu[3]);
        $display("TEMP DEBUG - Grants ALU[4]: %b", grants_alu[4]);
        $display("TEMP DEBUG - Grants ALU[5]: %b", grants_alu[5]);

        // EXECUTE STAGE
        // INV SIG
        // $display("\n--- EXECUTE STAGE ---");
        // $display("FUNCTIONAL UNITS EXECUTING:");
        // $display("  ALU: valid[0]=%b valid[1]=%b valid[2]=%b", alu_executing[0], alu_executing[1], alu_executing[2]);
        // $display("  MULT: valid[0]=%b", mult_executing[0]);
        // $display("  BRANCH: valid[0]=%b", branch_executing[0]);
        // $display("  MEM: valid[0]=%b", mem_executing[0]);

        // FU Results
        // INV SIG
        // $display("\nFunctional Unit Results:");
        // for (integer i = 0; i < `NUM_FU_ALU; i++) begin
        //     $display("  ALU[%0d]: func=%0d | result=0x%08h", i, alu_func[i], fu_results.alu[i]);
        // end
        // for (integer i = 0; i < `NUM_FU_MULT; i++) begin
        //     $display("  MULT[%0d]: 0x%08h (start=%b done=%b req=%b)", i, fu_results.mult[i], mult_start[i], mult_done[i],
        //              mult_request[i]);
        // end
        // for (integer i = 0; i < `NUM_FU_BRANCH; i++) begin
        //     $display("  BRANCH[%0d]: take=%b target=0x%08h", i, branch_take[i], branch_target[i]);
        // end

        // PRF Reads
        // INV SIG
        // $display("\nPRF Read Operations:");
        // $display("  ALU reads - src1_en=%b src2_en=%b", prf_read_en_src1.alu, prf_read_en_src2.alu);
        // $display("  MULT reads - src1_en=%b src2_en=%b", prf_read_en_src1.mult, prf_read_en_src2.mult);
        // $display("  BRANCH reads - src1_en=%b src2_en=%b", prf_read_en_src1.branch, prf_read_en_src2.branch);

        // Resolved Operands (after forwarding)
        // INV SIG
        // $display("\nResolved Operands (after CDB forwarding):");
        // for (integer i = 0; i < `NUM_FU_ALU; i++) begin
        //     if (prf_read_en_src1.alu[i] || prf_read_en_src2.alu[i]) begin
        //         $display("  ALU[%0d]: src1=0x%08h (P%0d) src2=0x%08h (P%0d)", i, resolved_src1.alu[i], prf_read_tag_src1.alu[i],
        //                  resolved_src2.alu[i], prf_read_tag_src2.alu[i]);
        //     end
        // end
        // for (integer i = 0; i < `NUM_FU_MULT; i++) begin
        //     if (prf_read_en_src1.mult[i] || prf_read_en_src2.mult[i]) begin
        //         $display("  MULT[%0d]: src1=0x%08h (P%0d) src2=0x%08h (P%0d)", i, resolved_src1.mult[i],
        //                  prf_read_tag_src1.mult[i], resolved_src2.mult[i], prf_read_tag_src2.mult[i]);
        //     end
        // end
        // for (integer i = 0; i < `NUM_FU_BRANCH; i++) begin
        //     if (prf_read_en_src1.branch[i] || prf_read_en_src2.branch[i]) begin
        //         $display("  BRANCH[%0d]: src1=0x%08h (P%0d) src2=0x%08h (P%0d)", i, resolved_src1.branch[i],
        //                  prf_read_tag_src1.branch[i], resolved_src2.branch[i], prf_read_tag_src2.branch[i]);
        //     end
        // end

        // COMPLETE STAGE
        $display("\n--- COMPLETE STAGE ---");
        $display("ROB Update: valid=%b", rob_update_packet.valid);
        if (rob_update_packet.valid) begin
            for (integer i = 0; i < `N; i++) begin
                if (rob_update_packet.valid[i]) begin
                    $display("  ROB[%0d]: idx=%0d | value=%h | taken=%b | target=%h", i, rob_update_packet.idx[i],
                             rob_update_packet.values[i], rob_update_packet.branch_taken[i], rob_update_packet.branch_targets[i]);
                end
            end
        end

        // CDB
        $display("\n--- COMMON DATA BUS (CDB) ---");

        // FU Requests to CDB
        $display("FU REQUESTS to CDB Arbiter:");
        $display("  ALU requests: %b", cdb_requests.alu);
        $display("  MULT requests: %b", cdb_requests.mult);
        $display("  BRANCH requests: %b", cdb_requests.branch);
        $display("  MEM requests: %b", cdb_requests.mem);

        // FU Outputs (data ready to broadcast)
        $display("\nFU OUTPUTS (data ready for CDB):");
        for (integer i = 0; i < `NUM_FU_ALU; i++) begin
            if (cdb_fu_outputs.alu[i].valid) begin
                $display("  ALU[%0d]: tag=P%0d | data=0x%08h", i, cdb_fu_outputs.alu[i].tag, cdb_fu_outputs.alu[i].data);
            end
        end
        for (integer i = 0; i < `NUM_FU_MULT; i++) begin
            if (cdb_fu_outputs.mult[i].valid) begin
                $display("  MULT[%0d]: tag=P%0d | data=0x%08h", i, cdb_fu_outputs.mult[i].tag, cdb_fu_outputs.mult[i].data);
            end
        end
        for (integer i = 0; i < `NUM_FU_BRANCH; i++) begin
            if (cdb_fu_outputs.branch[i].valid) begin
                $display("  BRANCH[%0d]: tag=P%0d | data=0x%08h", i, cdb_fu_outputs.branch[i].tag, cdb_fu_outputs.branch[i].data);
            end
        end
        for (integer i = 0; i < `NUM_FU_MEM; i++) begin
            if (cdb_fu_outputs.mem[i].valid) begin
                $display("  MEM[%0d]: tag=P%0d | data=0x%08h", i, cdb_fu_outputs.mem[i].tag, cdb_fu_outputs.mem[i].data);
            end
        end

        // // CDB Arbitration Results
        // $display("\nCDB ARBITRATION RESULTS:");
        // $display("  Grants (flat): %b", cdb_grants_flat);
        // $display("  Priority order: BRANCH(highest) -> ALU -> MEM -> MULT(lowest)");

        // // CDB Broadcasts
        // $display("\nCDB BROADCASTS:");
        // for (integer i = 0; i < `N; i++) begin
        //     if (cdb_output[i].valid) begin
        //         $display("  CDB[%0d]: tag=P%0d | data=0x%08h | grant_bus=%b", i, cdb_output[i].tag, cdb_output[i].data,
        //                  cdb_gnt_bus[i]);
        //     end else begin
        //         $display("  CDB[%0d]: INVALID", i);
        //     end
        // end

        // // Early Tag Broadcasts
        // $display("\nEARLY TAG BROADCASTS (for wakeup):");
        // for (integer i = 0; i < `N; i++) begin
        //     if (cdb_early_tags[i].valid) begin
        //         $display("  Early[%0d]: tag=P%0d", i, cdb_early_tags[i].tag);
        //     end
        // end

        // // ROB Head Window (oldest N instructions, candidates for retirement)
        // $display("\n--- REORDER BUFFER HEAD WINDOW ---");
        // for (integer i = 0; i < `N; i++) begin
        //     if (rob_head_valids[i]) begin
        //         $display("HEAD[%0d] (ROB idx=%0d): PC=0x%08h | Inst=0x%08h | RD=r%0d->P%0d | Complete=%b | Branch=%b", i,
        //                  rob_head_idxs[i], rob_head_entries[i].PC, rob_head_entries[i].inst, rob_head_entries[i].arch_rd,
        //                  rob_head_entries[i].phys_rd, rob_head_entries[i].complete, rob_head_entries[i].branch);
        //     end else begin
        //         $display("HEAD[%0d]: INVALID", i);
        //     end
        // end

        // RETIRE/COMMIT
        $display("\n--- RETIRE/COMMIT ---");
        for (integer i = 0; i < `N; i++) begin
            if (committed_insts[i].valid) begin
                if (committed_insts[i].reg_idx == `ZERO_REG)
                    $display("COMMIT[%0d]: PC=0x%08h | ---", i, committed_insts[i].NPC - 4);
                else
                    $display(
                        "COMMIT[%0d]: PC=0x%08h | r%0d = 0x%08h",
                        i,
                        committed_insts[i].NPC - 4,
                        committed_insts[i].reg_idx,
                        committed_insts[i].data
                    );
            end
        end

        // // MAP TABLES
        // $display("\n--- MAP TABLES ---");
        // $display("Architected Map Table (speculative):");
        // for (integer i = 0; i < `ARCH_REG_SZ; i += 4) begin
        //     $display("  r%0d->P%0d  r%0d->P%0d  r%0d->P%0d  r%0d->P%0d", i, arch_table_snapshot[i].phys_reg, i + 1,
        //              arch_table_snapshot[i+1].phys_reg, i + 2, arch_table_snapshot[i+2].phys_reg, i + 3,
        //              arch_table_snapshot[i+3].phys_reg);
        // end

        // $display("Working Map Table (speculative):");
        // for (integer i = 0; i < `ARCH_REG_SZ; i += 4) begin
        //     $display("  r%0d->P%0d%s  r%0d->P%0d%s  r%0d->P%0d%s  r%0d->P%0d%s", i, map_table_snapshot[i].phys_reg,
        //              map_table_snapshot[i].ready ? "+" : "", i + 1, map_table_snapshot[i+1].phys_reg,
        //              map_table_snapshot[i+1].ready ? "+" : "", i + 2, map_table_snapshot[i+2].phys_reg,
        //              map_table_snapshot[i+2].ready ? "+" : "", i + 3, map_table_snapshot[i+3].phys_reg,
        //              map_table_snapshot[i+3].ready ? "+" : "");
        // end

        // // FREE LIST
        // $display("\n--- FREE LIST ---");
        // $display("Available Physical Registers: %0d", $countones(freelist_available));
        // $display("Available Register Ranges:");

        // begin
        //     integer start_range = -1;
        //     integer in_range = 0;
        //     integer first_available = 1;

        //     // Find and display contiguous ranges of available registers
        //     for (i = 0; i < `PHYS_REG_SZ_R10K; i = i + 1) begin
        //         if (freelist_available[i]) begin
        //             if (!in_range) begin
        //                 start_range = i;
        //                 in_range = 1;
        //             end
        //         end else begin
        //             if (in_range) begin
        //                 // End of range
        //                 if (first_available) begin
        //                     first_available = 0;
        //                     $write("  ");
        //                 end else begin
        //                     $write(", ");
        //                 end

        //                 if (start_range == i - 1) begin
        //                     $write("P%0d", start_range);
        //                 end else begin
        //                     $write("P%0d-P%0d", start_range, i - 1);
        //                 end
        //                 in_range = 0;
        //             end
        //         end
        //     end

        //     // Handle case where last register is available
        //     if (in_range) begin
        //         if (first_available) begin
        //             $write("  ");
        //         end else begin
        //             $write(", ");
        //         end

        //         if (start_range == `PHYS_REG_SZ_R10K - 1) begin
        //             $write("P%0d", start_range);
        //         end else begin
        //             $write("P%0d-P%0d", start_range, `PHYS_REG_SZ_R10K - 1);
        //         end
        //     end

        //     if (first_available) begin
        //         $display("  (None)");
        //     end else begin
        //         $display("");
        //     end
        // end

        // $display("Freelist Restore Mask: %h", freelist_restore_mask);

        // // PHYSICAL REGISTER FILE (show non-zero values)
        // $display("\n--- PHYSICAL REGISTER FILE (non-zero values) ---");
        // prf_count = 0;
        // for (i = 0; i < `PHYS_REG_SZ_R10K; i = i + 1) begin
        //     if (regfile_entries[i] != 0 && prf_count < 16) begin  // Limit output
        //         $display("  P%0d = 0x%08h", i, regfile_entries[i]);
        //         prf_count = prf_count + 1;
        //     end
        // end
        // if (prf_count == 0) $display("  (All registers are zero)");

        $display("=================================================================================\n");
    endtask


endmodule  // module testbench