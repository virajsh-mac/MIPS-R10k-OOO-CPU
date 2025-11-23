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


`define TB_MAX_CYCLES 1000


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

    // Dispatch packet debug output
    FETCH_DISP_PACKET fetch_disp_packet;

    // Issue clear signals debug output
    RS_CLEAR_SIGNALS rs_clear_signals;

    // Execute stage debug outputs
    FU_RESULTS fu_results;
    PRF_READ_EN prf_read_en_src1, prf_read_en_src2;
    PRF_READ_TAGS prf_read_tag_src1, prf_read_tag_src2;
    PRF_READ_DATA resolved_src1, resolved_src2;
    logic [`NUM_FU_MULT-1:0] mult_request, mult_start, mult_done;
    logic [`NUM_FU_BRANCH-1:0] branch_take;
    ADDR [`NUM_FU_BRANCH-1:0] branch_target;

    // Execute stage functional unit validity
    logic [`NUM_FU_ALU-1:0] alu_executing;
    ALU_FUNC [`NUM_FU_ALU-1:0] alu_func;
    logic [`NUM_FU_MULT-1:0] mult_executing;
    logic [`NUM_FU_BRANCH-1:0] branch_executing;
    logic [`NUM_FU_MEM-1:0] mem_executing;

    // ICache debug signals
    I_ADDR_PACKET [1:0] icache_read_addrs;
    CACHE_DATA [1:0] icache_cache_outs;
    logic [1:0] icache_hits;
    logic [1:0] icache_misses;
    logic icache_full;
    I_ADDR_PACKET prefetch_addr;
    I_ADDR_PACKET oldest_miss_addr;
    logic mshr_addr_found;
    logic [$clog2(`NUM_MEM_TAGS + `N)-1:0] mshr_head;
    logic [$clog2(`NUM_MEM_TAGS + `N)-1:0] mshr_tail;
    MSHR_PACKET [`NUM_MEM_TAGS + `N-1:0] mshr_entries;
    logic [$clog2(`NUM_MEM_TAGS + `N)-1:0] mshr_next_head;
    logic [$clog2(`NUM_MEM_TAGS + `N)-1:0] mshr_next_tail;
    logic mshr_pop_condition;
    logic mshr_push_condition;
    logic mshr_pop_cond_has_data;
    logic mshr_pop_cond_head_valid;
    logic mshr_pop_cond_tag_match;
    logic mem_write_icache;
    I_ADDR_PACKET mem_write_addr;
    MEM_BLOCK mem_data_from_mem;
    MEM_TAG mem_data_tag_from_mem;
    I_ADDR_PACKET icache_write_addr;
    MEM_BLOCK icache_write_data;
    I_CACHE_LINE icache_line_write;
    logic [(`ICACHE_LINES + `PREFETCH_STREAM_BUFFER_SIZE)-1:0] icache_write_enable_mask;

    // Prefetcher debug signals
    I_ADDR_PACKET prefetcher_last_icache_miss_mem_req;
    I_ADDR_PACKET prefetcher_next_last_icache_miss_mem_req;
    I_ADDR prefetcher_addr_incrementor;
    I_ADDR prefetcher_next_addr_incrementor;

    // Logic block debug signals
    I_ADDR_PACKET mem_req_addr_dbg;
    logic snooping_found_icache_dbg;
    MSHR_PACKET new_mshr_entry_dbg;

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

        // Dispatch packet debug output
        .fetch_disp_packet_dbg(fetch_disp_packet),

        // Issue clear signals debug output
        .rs_clear_signals_dbg(rs_clear_signals),

        // Execute stage debug outputs
        .fu_results_dbg(fu_results),
        .prf_read_en_src1_dbg(prf_read_en_src1),
        .prf_read_en_src2_dbg(prf_read_en_src2),
        .prf_read_tag_src1_dbg(prf_read_tag_src1),
        .prf_read_tag_src2_dbg(prf_read_tag_src2),
        .resolved_src1_dbg(resolved_src1),
        .resolved_src2_dbg(resolved_src2),
        .fu_gnt_bus_dbg(),
        .mult_request_dbg(mult_request),
        .mult_start_dbg(mult_start),
        .mult_done_dbg(mult_done),
        .branch_take_dbg(branch_take),
        .branch_target_dbg(branch_target),
        .alu_executing_dbg(alu_executing),
        .alu_func_dbg(alu_func),
        .mult_executing_dbg(mult_executing),
        .branch_executing_dbg(branch_executing),
        .mem_executing_dbg(mem_executing),

        // ICache debug outputs
        .read_addrs_dbg(icache_read_addrs),
        .cache_outs_dbg(icache_cache_outs),
        .icache_hits_dbg(icache_hits),
        .icache_misses_dbg(icache_misses),
        .icache_full_dbg(icache_full),
        .prefetch_addr_dbg(prefetch_addr),
        .oldest_miss_addr_dbg(oldest_miss_addr),
        .mshr_addr_found_dbg(mshr_addr_found),
        .mshr_head_dbg(mshr_head),
        .mshr_tail_dbg(mshr_tail),
        .mshr_entries_dbg(mshr_entries),
        .mshr_next_head_dbg(mshr_next_head),
        .mshr_next_tail_dbg(mshr_next_tail),
        .mshr_pop_condition_dbg(mshr_pop_condition),
        .mshr_push_condition_dbg(mshr_push_condition),
        .mshr_pop_cond_has_data_dbg(mshr_pop_cond_has_data),
        .mshr_pop_cond_head_valid_dbg(mshr_pop_cond_head_valid),
        .mshr_pop_cond_tag_match_dbg(mshr_pop_cond_tag_match),
        .mem_write_icache_dbg(mem_write_icache),
        .mem_write_addr_dbg(mem_write_addr),
        .mem_data_dbg(mem_data_from_mem),
        .mem_data_tag_dbg(mem_data_tag_from_mem),
        .icache_write_addr_dbg(icache_write_addr),
        .icache_write_data_dbg(icache_write_data),
        .icache_line_write_dbg(icache_line_write),
        .icache_write_enable_mask_dbg(icache_write_enable_mask),
        // Prefetcher debug outputs
        .prefetcher_last_icache_miss_mem_req_dbg(prefetcher_last_icache_miss_mem_req),
        .prefetcher_next_last_icache_miss_mem_req_dbg(prefetcher_next_last_icache_miss_mem_req),
        .prefetcher_addr_incrementor_dbg(prefetcher_addr_incrementor),
        .prefetcher_next_addr_incrementor_dbg(prefetcher_next_addr_incrementor),
        // Logic block debug outputs
        .mem_req_addr_dbg(mem_req_addr_dbg),
        .snooping_found_icache_dbg(snooping_found_icache_dbg),
        .new_mshr_entry_dbg(new_mshr_entry_dbg),

        // Fetch stage debug outputs
        .fetch_packet_dbg(fetch_packet_out),
        .ib_bundle_valid_dbg(ib_bundle_valid)
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
        begin
            $fdisplay(out_fileno, "\nFinal memory state and exit status:\n");
            $fdisplay(out_fileno, "@@@ Unified Memory contents hex on left, decimal on right: ");
            $fdisplay(out_fileno, "@@@");
            showing_data = 0;
            for (int k = 0; k <= `MEM_64BIT_LINES - 1; k = k + 1) begin
                if (memory.unified_memory[k] != 0) begin
                    $fdisplay(out_fileno, "@@@ mem[%5d] = %x : %0d", k * 8, memory.unified_memory[k], memory.unified_memory[k]);
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
    endtask  // task show_final_mem_and_status



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

        $display("\n=================================================================================");
        $display("CPU STATE SNAPSHOT - Cycle %0d", clock_count - 1);
        $display("=================================================================================");

        // ICACHE SUBSYSTEM DEBUG
        // $display("\n--- ICACHE SUBSYSTEM ---");

        // $display("\nRead Addresses from Fetch:");
        // for (integer k = 0; k < 2; k++) begin
        //     $display("  Port[%0d]: Valid=%b", k, icache_read_addrs[k].valid);
        //     if (icache_read_addrs[k].valid) begin
        //         $display("    Full PC: 0x%04h", {icache_read_addrs[k].addr.zeros, icache_read_addrs[k].addr.tag,
        //                                          icache_read_addrs[k].addr.block_offset});
        //         $display("    Addr breakdown - zeros: 0x%04h, tag: 0x%02h, block_offset: 0x%01h",
        //                  icache_read_addrs[k].addr.zeros, icache_read_addrs[k].addr.tag, icache_read_addrs[k].addr.block_offset);
        //     end
        // end

        // $display("\nICache Status:");
        // $display("  Port[0]: %s | Port[1]: %s | Full: %b", icache_hits[0] ? "HIT " : (icache_misses[0] ? "MISS" : "IDLE"),
        //          icache_hits[1] ? "HIT " : (icache_misses[1] ? "MISS" : "IDLE"), icache_full);

        // $display("\nCache Outputs to Fetch:");
        // for (integer co_idx = 0; co_idx < 2; co_idx++) begin
        //     $display("  Port[%0d]: Valid=%b", co_idx, icache_cache_outs[co_idx].valid);
        //     if (icache_cache_outs[co_idx].valid) begin
        //         $display("    Data (MEM_BLOCK):");
        //         $display("      Word[0]: 0x%08h | Word[1]: 0x%08h", icache_cache_outs[co_idx].data.word_level[0],
        //                  icache_cache_outs[co_idx].data.word_level[1]);
        //         $display("      Full 64-bit: 0x%016h", icache_cache_outs[co_idx].data.dbbl_level);
        //     end
        // end

        // Oldest miss address logic
        $display("\n--- OLDEST MISS ADDRESS LOGIC ---");
        $display("read_addrs[0]:");
        $display("  Valid: %b", icache_read_addrs[0].valid);
        if (icache_read_addrs[0].valid) begin
            $display("  Full PC: 0x%04h", {icache_read_addrs[0].addr.zeros, icache_read_addrs[0].addr.tag,
                                           icache_read_addrs[0].addr.block_offset});
            $display("  Addr breakdown - zeros: 0x%04h, tag: 0x%02h, block_offset: 0x%01h",
                     icache_read_addrs[0].addr.zeros, icache_read_addrs[0].addr.tag, icache_read_addrs[0].addr.block_offset);
        end
        $display("read_addrs[1]:");
        $display("  Valid: %b", icache_read_addrs[1].valid);
        if (icache_read_addrs[1].valid) begin
            $display("  Full PC: 0x%04h", {icache_read_addrs[1].addr.zeros, icache_read_addrs[1].addr.tag,
                                           icache_read_addrs[1].addr.block_offset});
            $display("  Addr breakdown - zeros: 0x%04h, tag: 0x%02h, block_offset: 0x%01h",
                     icache_read_addrs[1].addr.zeros, icache_read_addrs[1].addr.tag, icache_read_addrs[1].addr.block_offset);
        end
        $display("cache_outs[0]:");
        $display("  Valid: %b", icache_cache_outs[0].valid);
        if (icache_cache_outs[0].valid) begin
            $display("    Data (MEM_BLOCK):");
            $display("      Word[0]: 0x%08h | Word[1]: 0x%08h", icache_cache_outs[0].data.word_level[0],
                     icache_cache_outs[0].data.word_level[1]);
            $display("      Full 64-bit: 0x%016h", icache_cache_outs[0].data.dbbl_level);
        end
        $display("cache_outs[1]:");
        $display("  Valid: %b", icache_cache_outs[1].valid);
        if (icache_cache_outs[1].valid) begin
            $display("    Data (MEM_BLOCK):");
            $display("      Word[0]: 0x%08h | Word[1]: 0x%08h", icache_cache_outs[1].data.word_level[0],
                     icache_cache_outs[1].data.word_level[1]);
            $display("      Full 64-bit: 0x%016h", icache_cache_outs[1].data.dbbl_level);
        end
        $display("oldest_miss_addr:");
        $display("  Valid: %b", oldest_miss_addr.valid);
        if (oldest_miss_addr.valid) begin
            $display("  Full PC: 0x%04h", {oldest_miss_addr.addr.zeros, oldest_miss_addr.addr.tag,
                                           oldest_miss_addr.addr.block_offset});
            $display("  Addr breakdown - zeros: 0x%04h, tag: 0x%02h, block_offset: 0x%01h", oldest_miss_addr.addr.zeros,
                     oldest_miss_addr.addr.tag, oldest_miss_addr.addr.block_offset);
        end

        $display("\nPrefetcher:");
        $display("  prefetcher_snooping_addr (prefetch_addr_dbg):");
        $display("    Valid: %b", prefetch_addr.valid);
        if (prefetch_addr.valid) begin
            $display("    Full PC: 0x%04h", {prefetch_addr.addr.zeros, prefetch_addr.addr.tag, prefetch_addr.addr.block_offset});
            $display("    Addr breakdown - zeros: 0x%04h, tag: 0x%02h, block_offset: 0x%01h", prefetch_addr.addr.zeros,
                     prefetch_addr.addr.tag, prefetch_addr.addr.block_offset);
        end
        $display("  last_icache_miss_mem_req:");
        $display("    Valid: %b", prefetcher_last_icache_miss_mem_req.valid);
        if (prefetcher_last_icache_miss_mem_req.valid) begin
            $display("    Full PC: 0x%04h", {prefetcher_last_icache_miss_mem_req.addr.zeros, prefetcher_last_icache_miss_mem_req.addr.tag,
                                             prefetcher_last_icache_miss_mem_req.addr.block_offset});
            $display("    Addr breakdown - zeros: 0x%04h, tag: 0x%02h, block_offset: 0x%01h",
                     prefetcher_last_icache_miss_mem_req.addr.zeros, prefetcher_last_icache_miss_mem_req.addr.tag,
                     prefetcher_last_icache_miss_mem_req.addr.block_offset);
        end
        $display("  next_last_icache_miss_mem_req:");
        $display("    Valid: %b", prefetcher_next_last_icache_miss_mem_req.valid);
        if (prefetcher_next_last_icache_miss_mem_req.valid) begin
            $display("    Full PC: 0x%04h", {prefetcher_next_last_icache_miss_mem_req.addr.zeros, prefetcher_next_last_icache_miss_mem_req.addr.tag,
                                             prefetcher_next_last_icache_miss_mem_req.addr.block_offset});
            $display("    Addr breakdown - zeros: 0x%04h, tag: 0x%02h, block_offset: 0x%01h",
                     prefetcher_next_last_icache_miss_mem_req.addr.zeros, prefetcher_next_last_icache_miss_mem_req.addr.tag,
                     prefetcher_next_last_icache_miss_mem_req.addr.block_offset);
        end
        $display("  addr_incrementor:");
        $display("    Full I_ADDR: 0x%04h", {prefetcher_addr_incrementor.zeros, prefetcher_addr_incrementor.tag,
                                             prefetcher_addr_incrementor.block_offset});
        $display("    Addr breakdown - zeros: 0x%04h, tag: 0x%02h, block_offset: 0x%01h",
                 prefetcher_addr_incrementor.zeros, prefetcher_addr_incrementor.tag, prefetcher_addr_incrementor.block_offset);
        $display("  next_addr_incrementor:");
        $display("    Full I_ADDR: 0x%04h", {prefetcher_next_addr_incrementor.zeros, prefetcher_next_addr_incrementor.tag,
                                             prefetcher_next_addr_incrementor.block_offset});
        $display("    Addr breakdown - zeros: 0x%04h, tag: 0x%02h, block_offset: 0x%01h",
                 prefetcher_next_addr_incrementor.zeros, prefetcher_next_addr_incrementor.tag, prefetcher_next_addr_incrementor.block_offset);

        $display("\nMSHR (Miss Status Holding Register):");
        $display("  Head: %0d (next=%0d) | Tail: %0d (next=%0d)", mshr_head, mshr_next_head, mshr_tail, mshr_next_tail);
        $display("  Push Condition: %b", mshr_push_condition);
        $display("  Pop Condition: %b (BREAKDOWN BELOW)", mshr_pop_condition);
        $display("    [1] mem_data_tag != 0      : %b (mem_data_tag=%0d)", mshr_pop_cond_has_data, mem_data_tag_from_mem);
        $display("    [2] mshr[head].valid       : %b", mshr_pop_cond_head_valid);
        $display("    [3] tag match              : %b (mem_tag=%0d vs mshr[%0d].mem_tag=%0d)", mshr_pop_cond_tag_match,
                 mem_data_tag_from_mem, mshr_head, mshr_entries[mshr_head].mem_tag);
        $display("  MSHR[head=%0d]: valid=%b | mem_tag=%0d | i_tag=0x%02h", mshr_head, mshr_entries[mshr_head].valid,
                 mshr_entries[mshr_head].mem_tag, mshr_entries[mshr_head].i_tag);

        $display("  All MSHR Entries:");
        for (integer j = 0; j < (`NUM_MEM_TAGS + `N); j++) begin
            if (mshr_entries[j].valid) begin
                $display("  MSHR[%0d]: valid=1 | mem_tag=%0d | i_tag=0x%02h", j, mshr_entries[j].mem_tag, mshr_entries[j].i_tag);
            end
        end

        // Mem request address logic
        $display("\n--- MEM REQUEST ADDRESS LOGIC ---");
        $display("snooping_found_icache: %b", snooping_found_icache_dbg);
        $display("snooping_found_mshr: %b", mshr_addr_found);
        $display("prefetcher_snooping_addr (prefetch_addr_dbg):");
        $display("  Valid: %b", prefetch_addr.valid);
        if (prefetch_addr.valid) begin
            $display("  Full PC: 0x%04h", {prefetch_addr.addr.zeros, prefetch_addr.addr.tag, prefetch_addr.addr.block_offset});
            $display("  Addr breakdown - zeros: 0x%04h, tag: 0x%02h, block_offset: 0x%01h", prefetch_addr.addr.zeros,
                     prefetch_addr.addr.tag, prefetch_addr.addr.block_offset);
        end
        $display("mem_req_addr:");
        $display("  Valid: %b", mem_req_addr_dbg.valid);
        if (mem_req_addr_dbg.valid) begin
            $display("  Full PC: 0x%04h", {mem_req_addr_dbg.addr.zeros, mem_req_addr_dbg.addr.tag, mem_req_addr_dbg.addr.block_offset});
            $display("  Addr breakdown - zeros: 0x%04h, tag: 0x%02h, block_offset: 0x%01h", mem_req_addr_dbg.addr.zeros,
                     mem_req_addr_dbg.addr.tag, mem_req_addr_dbg.addr.block_offset);
        end

        // MSHR entry logic
        $display("\n--- MSHR ENTRY LOGIC ---");
        begin
            logic mem_req_accepted_calc;
            mem_req_accepted_calc = (proc2mem_command == 2'h1) && (mem2proc_transaction_tag != 0);  // MEM_LOAD == 2'h1
            $display("mem_req_accepted: %b (computed as: (proc2mem_command == MEM_LOAD) && (mem2proc_transaction_tag != 0))", mem_req_accepted_calc);
            $display("  proc2mem_command: %b (0=MEM_NONE, 1=MEM_LOAD, 2=MEM_STORE)", proc2mem_command);
            $display("  mem2proc_transaction_tag: %0d", mem2proc_transaction_tag);
        end
        $display("current_req_tag (mem2proc_transaction_tag): %0d", mem2proc_transaction_tag);
        $display("mem_req_addr.addr.tag: 0x%02h", mem_req_addr_dbg.addr.tag);
        $display("new_mshr_entry:");
        $display("  valid: %b", new_mshr_entry_dbg.valid);
        $display("  mem_tag: %0d", new_mshr_entry_dbg.mem_tag);
        $display("  i_tag: 0x%02h", new_mshr_entry_dbg.i_tag);

        // $display("\nMemory Interface:");
        // $display("  Memory Data Tag: %0d (0 = no data)", mem_data_tag_from_mem);
        // if (mem_data_tag_from_mem != 0) begin
        //     $display("  Memory Data Returned:");
        //     $display("    Word[0]: 0x%08h | Word[1]: 0x%08h", mem_data_from_mem.word_level[0], mem_data_from_mem.word_level[1]);
        //     $display("    Full 64-bit: 0x%016h", mem_data_from_mem.dbbl_level);
        //     $display("    Instruction check - Word[0] is WFI? %b (WFI = 0x10500073)",
        //              (mem_data_from_mem.word_level[0] == 32'h10500073));
        //     $display("    Instruction check - Word[1] is WFI? %b (WFI = 0x10500073)",
        //              (mem_data_from_mem.word_level[1] == 32'h10500073));
        // end

        // $display("\nICache Write Operations:");
        // $display("  Write Addr Valid: %b | Tag: 0x%02h", icache_write_addr.valid, icache_write_addr.addr.tag);

        // $display("\nFetch Packet Output (4-wide bundle):");
        // $display("  IB Bundle Valid: %b", ib_bundle_valid);
        // for (integer fp_idx = 0; fp_idx < 4; fp_idx++) begin
        //     $display("  Packet[%0d]:", fp_idx);
        //     $display("    Valid:          %b", fetch_packet_out[fp_idx].valid);
        //     if (fetch_packet_out[fp_idx].valid) begin
        //         $display("    PC:             0x%08h", fetch_packet_out[fp_idx].pc);
        //         $display("    Instruction:    0x%08h", fetch_packet_out[fp_idx].inst);
        //         $display("    Is Branch:      %b", fetch_packet_out[fp_idx].is_branch);
        //         if (fetch_packet_out[fp_idx].is_branch) begin
        //             $display("    BP Pred Taken:  %b", fetch_packet_out[fp_idx].bp_pred_taken);
        //             $display("    BP Pred Target: 0x%08h", fetch_packet_out[fp_idx].bp_pred_target);
        //             $display("    BP GHR Snapshot: 0b%07b", fetch_packet_out[fp_idx].bp_ghr_snapshot);
        //         end
        //     end
        // end

        // // INSTRUCTION BUFFER DEBUG
        // $display("\n--- INSTRUCTION BUFFER ---");
        // $display("Buffer State:");
        // $display("  Head Pointer: %0d | Tail Pointer: %0d | Count: %0d | Free Slots: %0d", ib_head_ptr, ib_tail_ptr, ib_count,
        //          ib_free_slots);
        // $display("  Full: %b | Empty: %b", (ib_free_slots == 0), (ib_count == 0));

        // $display("\nPush Operations (this cycle):");
        // $display("  Num Pushes: %0d", ib_num_pushes);
        // for (integer push_idx = 0; push_idx < 4; push_idx++) begin
        //     if (push_idx < ib_num_pushes && ib_new_entries[push_idx].valid) begin
        //         $display("  Push[%0d]: PC=0x%08h | Inst=0x%08h | Branch=%b", push_idx, ib_new_entries[push_idx].pc,
        //                  ib_new_entries[push_idx].inst, ib_new_entries[push_idx].is_branch);
        //     end
        // end

        // $display("\nPop Operations (this cycle):");
        // $display("  Num Pops: %0d", ib_num_pops);
        // for (integer pop_idx = 0; pop_idx < 3; pop_idx++) begin
        //     if (pop_idx < ib_num_pops && ib_popped_entries[pop_idx].valid) begin
        //         $display("  Pop[%0d]: PC=0x%08h | Inst=0x%08h | Branch=%b", pop_idx, ib_popped_entries[pop_idx].pc,
        //                  ib_popped_entries[pop_idx].inst, ib_popped_entries[pop_idx].is_branch);
        //     end
        // end

        // $display("\nBuffer Contents (valid entries only):");
        // begin
        //     integer valid_count = 0;
        //     for (integer buf_idx = 0; buf_idx < 32 && valid_count < 8; buf_idx++) begin  // Limit output to first 8 entries
        //         if (ib_buffer_entries[buf_idx].valid) begin
        //             $display("  Slot[%0d]: PC=0x%08h | Inst=0x%08h | Branch=%b", buf_idx, ib_buffer_entries[buf_idx].pc,
        //                      ib_buffer_entries[buf_idx].inst, ib_buffer_entries[buf_idx].is_branch);
        //             valid_count++;
        //         end
        //     end
        //     if (valid_count == 0) begin
        //         $display("  (Buffer is empty)");
        //     end else if (valid_count >= 8) begin
        //         $display("  ... (showing first 8 of %0d total entries)", ib_count);
        //     end
        // end

        // // FETCH/DISPATCH STAGE
        // $display("\n--- FETCH/DISPATCH STAGE ---");
        // $display("Dispatch count: %0d", dispatch_count);
        // // $display("Fake PC: 0x%08h | Consumed: %0d", fake_pc, fake_consumed);

        // for (integer i = 0; i < `N; i++) begin
        //     // if (i < dispatch_count) begin
        //     $display("DISP[%0d]: PC=0x%08h | Inst=0x%08h | Uses_RD=%b | RD=%0d", i, fetch_disp_packet.entries[i].PC,
        //              fetch_disp_packet.entries[i].inst, fetch_disp_packet.entries[i].uses_rd,
        //              fetch_disp_packet.entries[i].rd_idx);
        //     // end
        // end

        // // RESERVATION STATIONS
        // $display("\n--- RESERVATION STATIONS ---");

        // // Show RS requests (ready entries)
        // $display("RS Requests (ready entries):");
        // $display("  ALU requests: %b", rs_alu_requests);
        // $display("  MULT requests: %b", rs_mult_requests);
        // $display("  BRANCH requests: %b", rs_branch_requests);
        // $display("  MEM requests: %b", rs_mem_requests);

        // // ALU RS
        // $display("\nALU RS (%0d entries):", `RS_ALU_SZ);
        // for (integer i = 0; i < `RS_ALU_SZ; i++) begin
        //     if (rs_alu_entries[i].valid) begin
        //         $display("  ALU_RS[%0d]: PC=0x%08h | OP=%0d | Dest=P%0d | Src1=P%0d(%b) | Src2=P%0d(%b) | ROB=%0d", i,
        //                  rs_alu_entries[i].PC, rs_alu_entries[i].op_type.category, rs_alu_entries[i].dest_tag,
        //                  rs_alu_entries[i].src1_tag, rs_alu_entries[i].src1_ready, rs_alu_entries[i].src2_tag,
        //                  rs_alu_entries[i].src2_ready, rs_alu_entries[i].rob_idx);
        //     end else begin
        //         $display("  ALU_RS[%0d]: EMPTY", i);
        //     end
        // end

        // // MULT RS
        // $display("\nMULT RS (%0d entries):", `RS_MULT_SZ);
        // for (integer i = 0; i < `RS_MULT_SZ; i++) begin
        //     if (rs_mult_entries[i].valid) begin
        //         $display("  MULT_RS[%0d]: PC=0x%08h | OP=%0d | Dest=P%0d | Src1=P%0d(%b) | Src2=P%0d(%b) | ROB=%0d", i,
        //                  rs_mult_entries[i].PC, rs_mult_entries[i].op_type.category, rs_mult_entries[i].dest_tag,
        //                  rs_mult_entries[i].src1_tag, rs_mult_entries[i].src1_ready, rs_mult_entries[i].src2_tag,
        //                  rs_mult_entries[i].src2_ready, rs_mult_entries[i].rob_idx);
        //     end else begin
        //         $display("  MULT_RS[%0d]: EMPTY", i);
        //     end
        // end

        // // BRANCH RS
        // $display("\nBRANCH RS (%0d entries):", `RS_BRANCH_SZ);
        // for (integer i = 0; i < `RS_BRANCH_SZ; i++) begin
        //     if (rs_branch_entries[i].valid) begin
        //         $display("  BR_RS[%0d]: PC=0x%08h | OP=%0d | Dest=P%0d | Src1=P%0d(%b) | Src2=P%0d(%b) | ROB=%0d", i,
        //                  rs_branch_entries[i].PC, rs_branch_entries[i].op_type.category, rs_branch_entries[i].dest_tag,
        //                  rs_branch_entries[i].src1_tag, rs_branch_entries[i].src1_ready, rs_branch_entries[i].src2_tag,
        //                  rs_branch_entries[i].src2_ready, rs_branch_entries[i].rob_idx);
        //     end else begin
        //         $display("  BR_RS[%0d]: EMPTY", i);
        //     end
        // end

        // // MEM RS
        // $display("\nMEM RS (%0d entries):", `RS_MEM_SZ);
        // for (integer i = 0; i < `RS_MEM_SZ; i++) begin
        //     if (rs_mem_entries[i].valid) begin
        //         $display("  MEM_RS[%0d]: PC=0x%08h | OP=%0d | Dest=P%0d | Src1=P%0d(%b) | Src2=P%0d(%b) | ROB=%0d", i,
        //                  rs_mem_entries[i].PC, rs_mem_entries[i].op_type.category, rs_mem_entries[i].dest_tag,
        //                  rs_mem_entries[i].src1_tag, rs_mem_entries[i].src1_ready, rs_mem_entries[i].src2_tag,
        //                  rs_mem_entries[i].src2_ready, rs_mem_entries[i].rob_idx);
        //     end else begin
        //         $display("  MEM_RS[%0d]: EMPTY", i);
        //     end
        // end

        // // ISSUE STAGE
        // $display("\n--- ISSUE STAGE ---");
        // $display("RS Grants - ALU: %b | MULT: %b | BRANCH: %b | MEM: %b", rs_granted.alu, rs_granted.mult, rs_granted.branch,
        //          rs_granted.mem);

        // $display("ISSUE REGISTER (currently issued to execute):");
        // $display("  ALU: valid[0]=%b valid[1]=%b valid[2]=%b", issue_entries.alu[0].valid, issue_entries.alu[1].valid,
        //          issue_entries.alu[2].valid);
        // $display("  MULT: valid[0]=%b", issue_entries.mult[0].valid);
        // $display("  BRANCH: valid[0]=%b", issue_entries.branch[0].valid);
        // $display("  MEM: valid[0]=%b", issue_entries.mem[0].valid);

        // $display("RS CLEARS (immediate, follow grants):");
        // $display("  ALU: %b | MULT: %b | BRANCH: %b | MEM: %b", rs_clear_signals.valid_alu, rs_clear_signals.valid_mult,
        //          rs_clear_signals.valid_branch, rs_clear_signals.valid_mem);

        // $display("TEMP DEBUG - ALU Clear Signals from CDB: %b", alu_clear_signals);
        // $display("TEMP DEBUG - Grants ALU[0]: %b", grants_alu[0]);
        // $display("TEMP DEBUG - Grants ALU[1]: %b", grants_alu[1]);
        // $display("TEMP DEBUG - Grants ALU[2]: %b", grants_alu[2]);
        // $display("TEMP DEBUG - Grants ALU[3]: %b", grants_alu[3]);
        // $display("TEMP DEBUG - Grants ALU[4]: %b", grants_alu[4]);
        // $display("TEMP DEBUG - Grants ALU[5]: %b", grants_alu[5]);

        // // EXECUTE STAGE
        // $display("\n--- EXECUTE STAGE ---");
        // $display("FUNCTIONAL UNITS EXECUTING:");
        // $display("  ALU: valid[0]=%b valid[1]=%b valid[2]=%b", alu_executing[0], alu_executing[1], alu_executing[2]);
        // $display("  MULT: valid[0]=%b", mult_executing[0]);
        // $display("  BRANCH: valid[0]=%b", branch_executing[0]);
        // $display("  MEM: valid[0]=%b", mem_executing[0]);

        // // FU Results
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

        // // PRF Reads
        // $display("\nPRF Read Operations:");
        // $display("  ALU reads - src1_en=%b src2_en=%b", prf_read_en_src1.alu, prf_read_en_src2.alu);
        // $display("  MULT reads - src1_en=%b src2_en=%b", prf_read_en_src1.mult, prf_read_en_src2.mult);
        // $display("  BRANCH reads - src1_en=%b src2_en=%b", prf_read_en_src1.branch, prf_read_en_src2.branch);

        // // Resolved Operands (after forwarding)
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

        // // COMPLETE STAGE
        // $display("\n--- COMPLETE STAGE ---");
        // $display("ROB Update: valid=%b", rob_update_packet.valid);
        // if (rob_update_packet.valid) begin
        //     for (integer i = 0; i < `N; i++) begin
        //         if (rob_update_packet.valid[i]) begin
        //             $display("  ROB[%0d]: idx=%0d | value=%h | taken=%b | target=%h", i, rob_update_packet.idx[i],
        //                      rob_update_packet.values[i], rob_update_packet.branch_taken[i], rob_update_packet.branch_targets[i]);
        //         end
        //     end
        // end

        // // CDB
        // $display("\n--- COMMON DATA BUS (CDB) ---");

        // // FU Requests to CDB
        // $display("FU REQUESTS to CDB Arbiter:");
        // $display("  ALU requests: %b", cdb_requests.alu);
        // $display("  MULT requests: %b", cdb_requests.mult);
        // $display("  BRANCH requests: %b", cdb_requests.branch);
        // $display("  MEM requests: %b", cdb_requests.mem);

        // // FU Outputs (data ready to broadcast)
        // $display("\nFU OUTPUTS (data ready for CDB):");
        // for (integer i = 0; i < `NUM_FU_ALU; i++) begin
        //     if (cdb_fu_outputs.alu[i].valid) begin
        //         $display("  ALU[%0d]: tag=P%0d | data=0x%08h", i, cdb_fu_outputs.alu[i].tag, cdb_fu_outputs.alu[i].data);
        //     end
        // end
        // for (integer i = 0; i < `NUM_FU_MULT; i++) begin
        //     if (cdb_fu_outputs.mult[i].valid) begin
        //         $display("  MULT[%0d]: tag=P%0d | data=0x%08h", i, cdb_fu_outputs.mult[i].tag, cdb_fu_outputs.mult[i].data);
        //     end
        // end
        // for (integer i = 0; i < `NUM_FU_BRANCH; i++) begin
        //     if (cdb_fu_outputs.branch[i].valid) begin
        //         $display("  BRANCH[%0d]: tag=P%0d | data=0x%08h", i, cdb_fu_outputs.branch[i].tag, cdb_fu_outputs.branch[i].data);
        //     end
        // end
        // for (integer i = 0; i < `NUM_FU_MEM; i++) begin
        //     if (cdb_fu_outputs.mem[i].valid) begin
        //         $display("  MEM[%0d]: tag=P%0d | data=0x%08h", i, cdb_fu_outputs.mem[i].tag, cdb_fu_outputs.mem[i].data);
        //     end
        // end

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

        // // RETIRE/COMMIT
        // $display("\n--- RETIRE/COMMIT ---");
        // for (integer i = 0; i < `N; i++) begin
        //     if (committed_insts[i].valid) begin
        //         if (committed_insts[i].reg_idx == `ZERO_REG)
        //             $display("COMMIT[%0d]: PC=0x%08h | ---", i, committed_insts[i].NPC - 4);
        //         else
        //             $display(
        //                 "COMMIT[%0d]: PC=0x%08h | r%0d = 0x%08h",
        //                 i,
        //                 committed_insts[i].NPC - 4,
        //                 committed_insts[i].reg_idx,
        //                 committed_insts[i].data
        //             );
        //     end
        // end

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
