/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rob.sv                                              //
//                                                                     //
//  Description :  Reorder Buffer module; manages up to ROB_SZ         //
//                 in-flight instructions for in-order commit,         //
//                 branch misprediction recovery, and exception        //
//                 handling. Acts as a circular buffer with head       //
//                 (oldest) and tail (next alloc) pointers.            //
//                 Allocates entries from Dispatch, updates from       //
//                 Complete (complete bit, value, branch info),        //
//                 provides head entries to Retire, advances head      //
//                 on retire, and truncates on mispredict flush.       //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"

// Parameters and typedefs are now centrally defined in sys_defs.svh

// ROB entry structure is now defined in sys_defs.svh

// ROB update packet (extended for branch info) now in SYS_DEFS


module rob (
    input clock,  // system clock
    input reset,  // system reset

    // Allocation from Dispatch
    input logic [`N-1:0] alloc_valid,  // Valid allocations this cycle
    input ROB_ENTRY [`N-1:0] rob_entry_packet,  // New entries (partial fields set by Dispatch)
    
    output ROB_IDX [`N-1:0] alloc_idxs,  // Assigned ROB indices for new entries (which slots in ROB has been reserved)
    output logic [$clog2(`ROB_SZ+1)-1:0] free_slots,  // Number of free slots (for stall check)

    // Updates from Complete
    input ROB_UPDATE_PACKET rob_update_packet,  // Updates for complete bit, value, and branch info

    // Retire interface
    output ROB_ENTRY [`N-1:0] head_entries,  // Up to N consecutive head entries for Retire
    output logic [`N-1:0] head_valids,  // Valid bits for each head entry


    // Flush on mispredict (from Execute or Retire)
    input logic   mispredict,  // Flush signal
    input ROB_IDX mispred_idx  // ROB index of mispredicted branch (truncate after this)
);

  localparam ALLOC_CNT_WIDTH = $clog2(`N);
  // Internal storage: circular buffer of entries
  ROB_ENTRY [`ROB_SZ-1:0] rob_array;

  // Head (oldest) and tail (next allocation) pointers
  ROB_IDX head, tail;
  ROB_IDX head_next, tail_next;
  
  
  // Pointer used to indicate current retire location
  ROB_IDX current_retire_idx;
  
  // Signal from rob_update_packet that used to indicate the idx that is ready to complete (EX -> C)
  ROB_IDX rob_complete_update_idx;
  
  // Pointer that indicating current allocation idx
  ROB_IDX current_alloc_idx;

  


  // Combinational OUTPUT (free_slots): compute free slots
  always_comb begin
    logic [$clog2(`ROB_SZ):0] output_valid_count = 0;

    for (int i = 0; i < `ROB_SZ; i++) begin
      if (rob_array[i].valid) begin
        output_valid_count = output_valid_count + 1;
      end
    end

    free_slots = `ROB_SZ - output_valid_count;
  end


  // Combinational OUTPUT (alloc_idxs): assign allocation indices starting from tail
  always_comb begin
    logic [$clog2(`ROB_SZ):0] output_current_alloc_cnt = 0; // used to move the tail pointer for allocation
    
    for (int i = 0; i < `N; i++) begin
      alloc_idxs[i] = (tail + output_current_alloc_cnt) % `ROB_SZ;
      if (alloc_valid[i]) begin
        output_current_alloc_cnt++;
      end
    end
  end

  // Combinational OUTPUT (head_entries & head_valids): head entries and valids
  always_comb begin
    ROB_IDX output_current_output_idx;

    for (int i = 0; i < `N; i++) begin
      output_current_output_idx = (head + i) % `ROB_SZ;
      
      // output current head_entries based on current output idx
      head_entries[i] = rob_array[output_current_output_idx];
      
      // Valid if within committed range and entry is valid
      head_valids[i] = rob_array[output_current_output_idx].valid && (head != tail || free_slots == 0);
    end
  end

  // Next state logic (combinational)
  ROB_ENTRY [`ROB_SZ-1:0] rob_next;
  logic [(ALLOC_CNT_WIDTH-1):0] alloc_cnt;
  logic [(ALLOC_CNT_WIDTH-1):0] retire_cnt;
  
  always_comb begin
    // default vals
    rob_next  = rob_array;
    head_next = head;
    tail_next = tail;

    // Priority: handle mispredict flush (WIP)
    if (mispredict) begin
      tail_next = (mispred_idx + 1) % `ROB_SZ;
      // No need to invalidate entries explicitly; overwriting on future alloc suffices
    end else begin

      // RETIRE STAGE
      for (int i = 0; i < `N; i++) begin
        current_retire_idx = (head + i) % `ROB_SZ;
        if (rob_array[current_retire_idx].valid && rob_array[current_retire_idx].complete) begin
          retire_cnt++;
        end else begin
          break; // will stop at 1st non-complete, in-order inst. The retire pointer will stop here.
        end
      end

      // update header pointer, invalidate entry
      head_next = (head + retire_cnt) % `ROB_SZ;
      for (int i = 0; i < retire_cnt; i++) begin
        rob_next[(head + 1) % `ROB_SZ].valid = 1'b0;
      end

      // Updates from Complete: set complete, value, and branch info
      for (int i = 0; i < `N; i++) begin
        // when it's complete
        if (rob_update_packet.valid[i]) begin
          rob_complete_update_idx = rob_update_packet.idx[i];
          rob_next[rob_complete_update_idx].complete = 1'b1;
          
          // TODO: not sure if we need to store value inside the rob_update_packet
          // rob_next[idx].value = rob_update_packet.values[i];

          // Branching WIP
          if (rob_next[rob_complete_update_idx].branch) begin
            rob_next[rob_complete_update_idx].branch_taken  = rob_update_packet.branch_taken[i];
            rob_next[rob_complete_update_idx].branch_target = rob_update_packet.branch_targets[i];
          end
        end
      end


      // ### ALLOCATION (Dispatch entries in order): write new entries at alloc_idxs
      for (int i = 0; i < `N; i++) begin
        if (alloc_valid[i]) begin
          current_alloc_idx = (tail + alloc_cnt) % `ROB_SZ;
          rob_next[current_alloc_idx] = rob_entry_packet[i];
          rob_next[current_alloc_idx].valid = 1'b1;
          rob_next[current_alloc_idx].complete = 1'b0;
          rob_next[current_alloc_idx].exception = NO_ERROR;
          alloc_cnt++;
        end
      end
      tail_next = (tail + alloc_cnt) % `ROB_SZ;
    end
  end

  // Clocked update
  always_ff @(posedge clock) begin
    if (reset) begin
      head <= 0;
      tail <= 0;
      for (int i = 0; i < `ROB_SZ; i++) begin
        rob_array[i].valid <= 1'b0;
      end
    end else begin
      head <= head_next;
      tail <= tail_next;
      rob_array <= rob_next;
    end
  end

endmodule  // rob
