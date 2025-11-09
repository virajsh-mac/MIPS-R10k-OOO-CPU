/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  icache_subsystem.sv                                 //
//                                                                     //
//  Description :  Non-blocking instruction cache subsystem with MSHR, //
//                 prefetcher (with integrated stream buffer), and     //
//                 2-way banked icache.                                //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

// ============================================================================
// Main ICache Subsystem Module
// ============================================================================
module icache_subsystem (
    input clock,
    input reset,

    // From memory (via arbiter)
    input MEM_TAG   Imem2proc_transaction_tag,  // Memory accepted request with this tag (0 = rejected)
    input MEM_BLOCK Imem2proc_data,             // Data returning from memory
    input MEM_TAG   Imem2proc_data_tag,         // Tag for returned data (0 = no data)

    // From arbiter
    input logic arbiter_accept,                 // Arbiter accepted our memory request this cycle

    // From victim cache (external module)
    input logic      victim_cache_hit,
    input MEM_BLOCK  victim_cache_data,

    // From fetch stage
    input ADDR proc2Icache_addr,
    
    // To arbiter (for memory requests)
    output logic        mem_req_valid,          // Request to send to memory
    output ADDR         mem_req_addr,           // Address for memory request
    output MEM_COMMAND  mem_req_command,        // Memory command (MEM_LOAD)

    // To victim cache (for lookup)
    output ADDR victim_cache_lookup_addr,

    // To fetch stage
    output MEM_BLOCK Icache_data_out,           // Instruction data output
    output logic     Icache_valid_out           // Data is valid
);

    // Internal signals between submodules
    // TODO: Wire up connections between icache, prefetcher, MSHR

endmodule


// ============================================================================
// ICache (2-way banked, fully associative per bank)
// ============================================================================
// Two memDP modules for odd/even banking to support 2 simultaneous reads
// Each bank is fully associative (16 lines per bank = 32 total lines)
// Uses LFSR for pseudo-random replacement policy within each bank
module icache (
    input clock,
    input reset,

    // Lookup interface from fetch stage
    input I_ADDR       read_addr,
    output CACHE_DATA  cache_out, // cache hit, if cache_out.valid == 1

    // Fill interface from MSHR (when data returns from memory)
    // Instruction fetch never have to wrtie back
    input logic      write_en,
    input I_ADDR     write_addr,
    input MEM_BLOCK  write_data,

    // Eviction interface to victim cache
    output ADDR      evict_addr,
    output CACHE_DATA evict_data
);

    MEM_BLOCK bank0_data_out, bank1_data_out, cache_in;
    logic [1:0][(`ICACHE_LINES/`ICACHE_ASSOC)-1:0] valids;
    logic [1:0][(`ICACHE_LINES/`ICACHE_ASSOC)-1:0] tags [`ITAG_BITS-1:0];
    
    // Bank 0: 16 lines, fully associative (addr[3] = 0, even lines)
    memDP #(
        .WIDTH     ($bits(MEM_BLOCK)),
        .DEPTH     (`ICACHE_LINES / `ICACHE_ASSOC),
        .READ_PORTS(1),
        .BYPASS_EN (0)
    ) bank0 (
        .clock(clock),
        .reset(reset),
        .re   (1'b1),
        .raddr(read_addr.index),
        .rdata(bank0_data_out),
        .we   (write_en && ~write_addr.bank),
        .waddr(), // fully associative so write can't depend on index
        .wdata(cache_in)
    );

    // Bank 1: 16 lines, fully associative (addr[3] = 1, odd lines)
    memDP #(
        .WIDTH     ($bits(MEM_BLOCK)),
        .DEPTH     (`ICACHE_LINES / `ICACHE_ASSOC),
        .READ_PORTS(1),
        .BYPASS_EN (0)
    ) bank1 (
        .clock(clock),
        .reset(reset),
        .re   (1'b1),
        .raddr(read_addr.index),
        .rdata(bank1_data_out),
        .we   (write_en && write_addr.bank),
        .waddr(), // fully associative so write can't depend on index
        .wdata(cache_in)
    );

    // Read logic
    // Address break down [31:16] 0s, [15:9] tag, [8:4] index, [3] bank, [2:0] one mem_block
    assign cache_out.valid = read_addr.tag == tags[read_addr.bank][read_addr.index] && valids[read_addr.bank][read_addr.index];
    assign cache_out.cache_line = read_addr.bank ? bank1_data_out : bank0_data_out;

endmodule


// ============================================================================
// Miss Status Handling Register (MSHR)
// ============================================================================
// Uses psel_gen for efficient allocation of MSHR entries
module mshr (
    input clock,
    input reset,

    // Allocation requests (from cache miss path and prefetcher)
    input logic [1:0] alloc_req,                // [1]=prefetch, [0]=demand
    input ADDR        alloc_addr_demand,
    input ADDR        alloc_addr_prefetch,

    // From arbiter
    input logic       arbiter_accept,           // Arbiter accepted our request

    // From memory
    input MEM_TAG     Imem2proc_transaction_tag,
    input MEM_TAG     Imem2proc_data_tag,
    input MEM_BLOCK   Imem2proc_data,

    // To arbiter (memory request)
    output logic      mem_req_valid,
    output ADDR       mem_req_addr,

    // MSHR status
    output logic      mshr_full,
    output logic [3:0] mshr_occupancy,

    // Lookup interface (check if address already pending)
    input ADDR        lookup_addr,
    output logic      lookup_hit,               // Address already in MSHR

    // Data output when ready
    output logic      data_valid,
    output ADDR       data_addr,
    output MEM_BLOCK  data_block,
    output logic      data_is_prefetch
);

    // Internal: psel_gen for MSHR entry allocation
    // TODO: Instantiate psel_gen for allocating free MSHR entries

endmodule


// ============================================================================
// Prefetcher (with integrated stream buffer)
// ============================================================================
// Simple next-line sequential prefetcher with 4-entry stream buffer
// Stream buffer holds prefetched data before promotion to main cache
module prefetcher (
    input clock,
    input reset,

    // From fetch stage (monitor access pattern)
    input ADDR  fetch_addr,

    // Lookup interface (checked on cache miss)
    input ADDR       lookup_addr,
    output logic     prefetch_hit,              // Address found in stream buffer
    output MEM_BLOCK prefetch_data,             // Data from stream buffer

    // Fill interface from MSHR (when prefetch data returns)
    input logic      fill_en,
    input ADDR       fill_addr,
    input MEM_BLOCK  fill_data,

    // Prefetch request output (to MSHR)
    output logic     prefetch_req_valid,
    output ADDR      prefetch_req_addr,

    // Status
    output logic     stream_buffer_full
);

    // Internal: 4-entry stream buffer (holds prefetched lines)
    // TODO: Implement stream buffer storage and prefetch generation logic

endmodule

module LFSR #(parameter NUM_BITS)
  (
   input i_Clk,
   input i_Enable,
 
   // Optional Seed Value
   input i_Seed_DV,
   input [NUM_BITS-1:0] i_Seed_Data,
 
   output [NUM_BITS-1:0] o_LFSR_Data,
   output o_LFSR_Done
   );
 
  reg [NUM_BITS:1] r_LFSR = 0;
  reg              r_XNOR;
 
 
  // Purpose: Load up LFSR with Seed if Data Valid (DV) pulse is detected.
  // Othewise just run LFSR when enabled.
  always @(posedge i_Clk)
    begin
      if (i_Enable == 1'b1)
        begin
          if (i_Seed_DV == 1'b1)
            r_LFSR <= i_Seed_Data;
          else
            r_LFSR <= ;
        end
    end
 
  // Create Feedback Polynomials.  Based on Application Note:
  // http://www.xilinx.com/support/documentation/application_notes/xapp052.pdf
  always @(*)
    begin
      case (NUM_BITS)
        3: begin
          r_XNOR = r_LFSR[3] ^~ r_LFSR[2];
        end
        4: begin
          r_XNOR = r_LFSR[4] ^~ r_LFSR[3];
        end
        5: begin
          r_XNOR = r_LFSR[5] ^~ r_LFSR[3];
        end
        6: begin
          r_XNOR = r_LFSR[6] ^~ r_LFSR[5];
        end
        7: begin
          r_XNOR = r_LFSR[7] ^~ r_LFSR[6];
        end
        8: begin
          r_XNOR = r_LFSR[8] ^~ r_LFSR[6] ^~ r_LFSR[5] ^~ r_LFSR[4];
        end
        9: begin
          r_XNOR = r_LFSR[9] ^~ r_LFSR[5];
        end
        10: begin
          r_XNOR = r_LFSR[10] ^~ r_LFSR[7];
        end
        11: begin
          r_XNOR = r_LFSR[11] ^~ r_LFSR[9];
        end
        12: begin
          r_XNOR = r_LFSR[12] ^~ r_LFSR[6] ^~ r_LFSR[4] ^~ r_LFSR[1];
        end
        13: begin
          r_XNOR = r_LFSR[13] ^~ r_LFSR[4] ^~ r_LFSR[3] ^~ r_LFSR[1];
        end
        14: begin
          r_XNOR = r_LFSR[14] ^~ r_LFSR[5] ^~ r_LFSR[3] ^~ r_LFSR[1];
        end
        15: begin
          r_XNOR = r_LFSR[15] ^~ r_LFSR[14];
        end
        16: begin
          r_XNOR = r_LFSR[16] ^~ r_LFSR[15] ^~ r_LFSR[13] ^~ r_LFSR[4];
          end
        17: begin
          r_XNOR = r_LFSR[17] ^~ r_LFSR[14];
        end
        18: begin
          r_XNOR = r_LFSR[18] ^~ r_LFSR[11];
        end
        19: begin
          r_XNOR = r_LFSR[19] ^~ r_LFSR[6] ^~ r_LFSR[2] ^~ r_LFSR[1];
        end
        20: begin
          r_XNOR = r_LFSR[20] ^~ r_LFSR[17];
        end
        21: begin
          r_XNOR = r_LFSR[21] ^~ r_LFSR[19];
        end
        22: begin
          r_XNOR = r_LFSR[22] ^~ r_LFSR[21];
        end
        23: begin
          r_XNOR = r_LFSR[23] ^~ r_LFSR[18];
        end
        24: begin
          r_XNOR = r_LFSR[24] ^~ r_LFSR[23] ^~ r_LFSR[22] ^~ r_LFSR[17];
        end
        25: begin
          r_XNOR = r_LFSR[25] ^~ r_LFSR[22];
        end
        26: begin
          r_XNOR = r_LFSR[26] ^~ r_LFSR[6] ^~ r_LFSR[2] ^~ r_LFSR[1];
        end
        27: begin
          r_XNOR = r_LFSR[27] ^~ r_LFSR[5] ^~ r_LFSR[2] ^~ r_LFSR[1];
        end
        28: begin
          r_XNOR = r_LFSR[28] ^~ r_LFSR[25];
        end
        29: begin
          r_XNOR = r_LFSR[29] ^~ r_LFSR[27];
        end
        30: begin
          r_XNOR = r_LFSR[30] ^~ r_LFSR[6] ^~ r_LFSR[4] ^~ r_LFSR[1];
        end
        31: begin
          r_XNOR = r_LFSR[31] ^~ r_LFSR[28];
        end
        32: begin
          r_XNOR = r_LFSR[32] ^~ r_LFSR[22] ^~ r_LFSR[2] ^~ r_LFSR[1];
        end
 
      endcase // case (NUM_BITS)
    end // always @ (*)
 
 
  assign o_LFSR_Data = r_LFSR[NUM_BITS:1];
 
  // Conditional Assignment (?)
  assign o_LFSR_Done = (r_LFSR[NUM_BITS:1] == i_Seed_Data) ? 1'b1 : 1'b0;
 
endmodule // LFSR