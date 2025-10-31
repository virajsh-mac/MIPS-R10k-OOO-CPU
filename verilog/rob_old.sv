/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rob.sv                                              //
//                                                                     //
//  Description :  Reorder Buffer                                      //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"

module rob (
    input  logic                    clock,
    input  logic                    reset,

    // Allocation from Dispatch
    input  logic      [`N-1:0]      alloc_valid,
    input  ROB_ENTRY  [`N-1:0]      rob_entry_packet,
    output ROB_IDX    [`N-1:0]      alloc_idxs,
    output logic [$clog2(`ROB_SZ+1)-1:0] free_slots,

    // Updates from Complete
    input  ROB_UPDATE_PACKET        rob_update_packet,

    // Retire interface
    // [N-1] = oldest .. [0] = youngest
    output ROB_ENTRY [`N-1:0]       head_entries,
    output logic     [`N-1:0]       head_valids,

    // Flush on mispredict (from Execute or Retire)
    input  logic                    mispredict,
    input  ROB_IDX                  mispred_idx
);

  localparam ALLOC_CNT_WIDTH = $clog2(`N+1);

  // ------------------------
  // Storage and pointers
  // ------------------------
  ROB_ENTRY [`ROB_SZ-1:0] rob_array;

  ROB_IDX head, tail;
  ROB_IDX head_next, tail_next;

  // temps (module-scope, one driver each)
  ROB_IDX idx_hw;
  ROB_IDX rob_complete_update_idx;
  ROB_IDX rp;
  ROB_IDX idx_alloc;

  int     g_grant;     // grant calc block only
  int     g_idx;       // alloc_idxs block only

  // Disambiguate head==tail (empty vs full)
  logic full, full_next;

  // Peek how many we can retire this cycle (for capacity-aware grants)
  logic [$clog2(`N+1)-1:0]           retire_cnt_peek;
  logic [$clog2(`ROB_SZ+1)-1:0]      free_slots_for_grant;

  // ----------------------------
  // In-flight & free_slots (O(1))
  // ----------------------------
  logic [$clog2(`ROB_SZ+1)-1:0] inflight;
  always_comb begin
    if (tail == head)           inflight = (full ? `ROB_SZ : '0);
    else if (tail > head)       inflight = tail - head;
    else                        inflight = `ROB_SZ - (head - tail);
    free_slots = `ROB_SZ - inflight;
  end

  // -----------------------------------------
  // Head window view for Retire (combinational)
  // [N-1] = oldest, [0] = youngest
  // -----------------------------------------
  always_comb begin
    for (int unsigned li = 0; li < `N; li++) begin
      idx_hw = (head + li) % `ROB_SZ;
      head_entries[`N-1 - li] = rob_array[idx_hw];
      head_valids [`N-1 - li] = (li < inflight) && rob_array[idx_hw].valid;
    end
  end

  // ---------------------------------------------------------
  // Peek: how many entries can retire this cycle (no mutation)
  // ---------------------------------------------------------
  always_comb begin
    retire_cnt_peek = '0;
    rp = head;
    for (int unsigned li = 0; li < `N; li++) begin
      if (li >= inflight) break;
      if (rob_array[rp].valid && rob_array[rp].complete) begin
        retire_cnt_peek++;
        rp = (rp + 1) % `ROB_SZ;
      end else begin
        break;
      end
    end
  end

  // Capacity that dispatch can consume this cycle = current free + soon-to-be freed
  always_comb begin
    free_slots_for_grant = free_slots + retire_cnt_peek;
    if (free_slots_for_grant > `ROB_SZ) free_slots_for_grant = `ROB_SZ; // saturate
  end

  // ==========================================================
  // Grant calculation for allocation (capacity-throttled)
  // ==========================================================
  logic [ALLOC_CNT_WIDTH-1:0] req_cnt, grant_cnt;
  logic [`N-1:0]              grant_mask;

  always_comb begin
    req_cnt    = '0;
    grant_cnt  = '0;
    grant_mask = '0;
    g_grant    = 0;

    for (int unsigned li = 0; li < `N; li++) begin
      req_cnt += alloc_valid[li];
    end

    // Use capacity after same-cycle retire
    grant_cnt = (req_cnt <= free_slots_for_grant) ? req_cnt : free_slots_for_grant;

    for (int unsigned li = 0; li < `N; li++) begin
      if (alloc_valid[li] && (g_grant < grant_cnt)) begin
        grant_mask[li] = 1'b1;
        g_grant++;
      end
    end
  end

  // -----------------------------------------
  // alloc_idxs reflect the granted positions (sparse by lane)
  // -----------------------------------------
  always_comb begin
    for (int unsigned li = 0; li < `N; li++) begin
      if (grant_mask[li]) begin
        alloc_idxs[li] = (tail + li) % `ROB_SZ;  // per-lane position
      end else begin
        alloc_idxs[li] = tail; // benign for ungranted lanes
      end
    end
  end

  // --------------------------
  // Next-state (combinational)
  // --------------------------
  ROB_ENTRY [`ROB_SZ-1:0] rob_next;

  always_comb begin
    // defaults
    rob_next  = rob_array;
    head_next = head;
    tail_next = tail;
    full_next = full;

    // Flush on mispredict: truncate after mispred_idx
    if (mispredict) begin
      tail_next = (mispred_idx + 1) % `ROB_SZ;
      full_next = 1'b0;

    end else begin
      // -----------------------------
      // In-order retire advance (no skips)
      // -----------------------------
      ROB_IDX retire_ptr; bit retired_any;
      retire_ptr  = head_next;
      retired_any = 1'b0;

      for (int unsigned li = 0; li < `N; li++) begin
        if (li >= inflight) break;

        if (rob_array[retire_ptr].valid && rob_array[retire_ptr].complete) begin
          rob_next[retire_ptr].valid    = 1'b0;
          rob_next[retire_ptr].complete = 1'b0; // optional clear
          retire_ptr  = (retire_ptr + 1) % `ROB_SZ;
          retired_any = 1'b1;
        end else begin
          break; // stop at first incomplete
        end
      end

      head_next = retire_ptr;
      if (retired_any) full_next = 1'b0;

      // -----------------------------
      // COMPLETE updates (per-lane)
      // -----------------------------
      for (int unsigned li = 0; li < `N; li++) begin
        if (rob_update_packet.valid[li]) begin
          rob_complete_update_idx = rob_update_packet.idx[li];

          // mark complete
          rob_next[rob_complete_update_idx].complete = 1'b1;

          // branch metadata (if this entry is a branch)
          if (rob_next[rob_complete_update_idx].branch) begin
            rob_next[rob_complete_update_idx].branch_taken  = rob_update_packet.branch_taken[li];
            rob_next[rob_complete_update_idx].branch_target = rob_update_packet.branch_targets[li];
          end
        end
      end

      // ----------------------------------------
      // ALLOCATION (Dispatch) — capacity throttled (sparse by lane)
      // ----------------------------------------
      for (int unsigned li = 0; li < `N; li++) begin
        if (grant_mask[li]) begin
          idx_alloc = (tail + li) % `ROB_SZ;  // per-lane slot

          rob_next[idx_alloc]               = rob_entry_packet[li];
          rob_next[idx_alloc].valid         = 1'b1;
          rob_next[idx_alloc].complete      = 1'b0;
          rob_next[idx_alloc].exception     = NO_ERROR;
          rob_next[idx_alloc].rob_idx       = idx_alloc;
        end
      end

      tail_next = (tail + grant_cnt) % `ROB_SZ;

      // If we allocated something and collided with head, we're full
      if (grant_cnt != 0 && (tail_next == head_next)) begin
        full_next = 1'b1;
      end
    end
  end

  // -------------
  // Clocked state
  // -------------
  always_ff @(posedge clock) begin
    if (reset) begin
      head <= '0;
      tail <= '0;
      full <= 1'b0;
      for (int unsigned li = 0; li < `ROB_SZ; li++) begin
        rob_array[li].valid    <= 1'b0;
        rob_array[li].complete <= 1'b0;
      end
    end else begin
      head      <= head_next;
      tail      <= tail_next;
      full      <= full_next;
      rob_array <= rob_next;
    end
  end
`ifdef DEBUG_ROB
// Fires each cycle and logs what the COMPLETE block would do.
always_ff @(posedge clock) begin
  if (!reset) begin
    $display("ROB DBG @%0t: head=%0d tail=%0d full=%0b inflight=%0d free=%0d mispredict=%0b",
             $time, head, tail, full, inflight, free_slots, mispredict);
    // show incoming complete packet
    for (int unsigned li=0; li<`N; li++) begin
      if (rob_update_packet.valid[li]) begin
        $display("  COMPLETE REQ lane%0d: idx=%0d br_taken=%0b br_tgt=%h",
                 li, rob_update_packet.idx[li],
                 rob_update_packet.branch_taken[li],
                 rob_update_packet.branch_targets[li]);
      end
    end
  end
end

// After state updates: show the head entry view used by retire.sv
always_ff @(negedge clock) begin
  if (!reset) begin
    ROB_IDX hw = head;
    $display("ROB DBG POST-STATE @%0t: head=%0d tail=%0d",
             $time, head, tail);
    $display("  HEAD SLOT %0d : valid=%0b complete=%0b branch=%0b pred=%0b br_tkn=%0b",
             hw, rob_array[hw].valid, rob_array[hw].complete, rob_array[hw].branch,
             rob_array[hw].pred_taken, rob_array[hw].branch_taken);
  end
end
`endif

endmodule
