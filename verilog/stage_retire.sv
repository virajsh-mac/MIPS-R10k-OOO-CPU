/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_retire.sv                                     //
//                                                                     //
//  Description :  Retire stage of the pipeline; retires up to 3       //
//                 instructions from the head of the ROB if complete,  //
//                 commits to architectural state by updating the      //
//                 committed map table and freeing previous physical  //
//                 registers, handles branch predictor updates with   //
//                 actual outcomes. Demonstrates in-order commit by    //
//                 only retiring consecutive completed instructions   //
//                 from the ROB head. Handles misprediction recovery  //
//                 at retire for base design (flush ROB after         //
//                 mispredicted branch, rollback speculative map      //
//                 table to committed state, redirect Fetch). No       //
//                 decode here (handled in Fetch).                     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"

// Parameters and typedefs are now centrally defined in sys_defs.svh

// ROB entry is now defined in sys_defs.svh

// Packet to Dispatch for committed updates (free list and committed map)
typedef struct packed {
    logic [`N-1:0] valid;      // Valid commits this cycle
    REG_IDX [`N-1:0] arch_rds; // Architectural destinations
    PHYS_TAG [`N-1:0] phys_rds; // Committed physical regs
    PHYS_TAG [`N-1:0] prev_phys_rds; // Previous phys to free
} RETIRE_DISP_PACKET;

// Packet for branch predictor update (from Retire, single for simplicity)
typedef struct packed {
    logic valid;       // Update valid
    ADDR pc;           // PC of the branch
    logic taken;       // Actual taken/not taken
    ADDR target;       // Actual target address
} BP_UPDATE_PACKET;

// No direct packet from Complete to Retire (Retire polls ROB head)
// If needed, could use COMP_RET_PACKET from complete for signaling, but polled here

module stage_retire (
    input              clock,           // system clock
    input              reset,           // system reset

    // From ROB: up to 3 consecutive head entries (interface; full ROB module separate)
    input ROB_ENTRY [`N-1:0] rob_heads, // Head, head+1, head+2 entries
    input logic [`N-1:0]     rob_head_valids, // Valid bits for each (consecutive)

    // From Physical Reg File: values for commit packet (interface; full PRF module separate)
    input DATA [`N-1:0]      prf_read_values, // Read data for committed values

    // To ROB: retire count to advance head and free entries
    output logic [`ROB_IDX_BITS:0] retire_count, // Number retired this cycle (0 to `N)

    // To Dispatch: packet for committed map updates and free list additions
    output RETIRE_DISP_PACKET retire_packet,

    // To Branch Predictor: update with actual outcome (single update; prioritize first branch)
    output BP_UPDATE_PACKET  bp_update,

    // To Fetch/Dispatch/RS/ROB: mispredict flush signals for recovery
    output logic             mispredict_flush,  // Flush speculative path
    output ADDR              correct_target,    // Correct target for redirect
    output logic             rollback_map,      // Signal Dispatch to rollback map_table to committed_map
    output logic             flush_rob,         // Signal ROB to invalidate all entries (or set head = tail)
    output logic             flush_rs,          // Signal RS to invalidate all entries

    // To Physical Reg File: read requests for committed values
    output logic [`N-1:0]    prf_read_valid,   // Valid reads this cycle
    output PHYS_TAG [`N-1:0] prf_read_tags,    // Physical tags to read

    // To testbench: committed instructions for verification
    output COMMIT_PACKET [`N-1:0] committed_insts
);

    // Internal signals
    logic mispred_detected;  // Mispredict found this cycle
    ADDR mispred_correct_target;  // Computed correct target
    integer branch_idx;  // Index of mispredicted branch (if any)

    // Main retire logic (combinational; check consecutive completes, atomic commit)
    always_comb begin
        // Defaults
        retire_packet = '0;
        bp_update = '0;
        committed_insts = '0;
        prf_read_valid = '0;
        retire_count = 0;
        mispredict_flush = 1'b0;
        correct_target = '0;
        rollback_map = 1'b0;
        flush_rob = 1'b0;
        flush_rs = 1'b0;
        mispred_detected = 1'b0;
        mispred_correct_target = '0;
        branch_idx = `N;

        for (int i = 0; i < `N; i++) begin
            if (rob_head_valids[i] && rob_heads[i].valid && rob_heads[i].complete && rob_heads[i].exception == NO_ERROR) begin
                // Commit this instruction
                retire_packet.valid[i] = 1'b1;
                retire_packet.arch_rds[i] = rob_heads[i].arch_rd;
                retire_packet.phys_rds[i] = rob_heads[i].phys_rd;
                retire_packet.prev_phys_rds[i] = rob_heads[i].prev_phys_rd;

                // Read committed value from PRF for commit packet (if dest)
                if (rob_heads[i].arch_rd != `ZERO_REG) begin
                    prf_read_valid[i] = 1'b1;
                    prf_read_tags[i] = rob_heads[i].phys_rd;
                    committed_insts[i].data = prf_read_values[i];
                end

                // Fill commit packet
                committed_insts[i].valid = 1'b1;
                committed_insts[i].NPC = rob_heads[i].PC + 4;
                committed_insts[i].reg_idx = rob_heads[i].arch_rd;
                committed_insts[i].halt = rob_heads[i].halt;
                committed_insts[i].illegal = rob_heads[i].illegal;

                // If branch, check for mispredict and update predictor
                if (rob_heads[i].branch) begin
                    logic actual_taken = rob_heads[i].branch_taken;
                    ADDR actual_target = rob_heads[i].branch_target;
                    logic pred_taken = rob_heads[i].pred_taken;
                    ADDR pred_target = rob_heads[i].pred_target;
                    logic is_mispred = (actual_taken != pred_taken) || (actual_taken && (actual_target != pred_target));

                    // Update branch predictor with actual (only one per cycle; take first)
                    if (!bp_update.valid) begin
                        bp_update.valid = 1'b1;
                        bp_update.pc = rob_heads[i].PC;
                        bp_update.taken = actual_taken;
                        bp_update.target = actual_taken ? actual_target : (rob_heads[i].PC + 4);
                    end

                    // Handle mispredict (base: late at retire)
                    if (is_mispred) begin
                        mispred_detected = 1'b1;
                        mispred_correct_target = actual_taken ? actual_target : (rob_heads[i].PC + 4);
                        branch_idx = i;
                        break;  // Stop retiring further this cycle
                    end
                end

                retire_count = retire_count + 1;
            end else begin
                break;  // Stop at first non-complete/invalid/exception
            end
        end

        // Post-loop: if mispredict detected, initiate flush after committing up to the branch
        if (mispred_detected) begin
            mispredict_flush = 1'b1;
            correct_target = mispred_correct_target;
            rollback_map = 1'b1;  // Signal dispatch to set map_table = committed_map (after update)
            flush_rob = 1'b1;     // Flush ROB (invalidate all or reset head/tail)
            flush_rs = 1'b1;      // Flush RS (invalidate all speculative entries)
            retire_count = branch_idx + 1;  // Commit up to and including the branch
        end
    end

    // Note: ROB module would subscribe to retire_count to advance head
    // Dispatch would update committed_map and free list on retire_packet, and rollback on rollback_map
    // Fetch would redirect on mispredict_flush and correct_target
    // RS would clear on flush_rs
    // No clocked logic needed (combinational stage; updates combo or FF in other modules)

endmodule // stage_retire