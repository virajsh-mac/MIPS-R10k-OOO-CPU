/////////////////////////////////////////////////////////////////////////
//                                                                     //
//  Modulename :  bp.sv                                                //
//                                                                     //
//  Description :  Branch Predictor module; implements a hybrid       //
//                 gshare + BTB predictor with global history.        //
//                 Handles prediction requests from fetch, training   //
//                 from retire, and mispredict recovery.              //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

module bp (
    input logic clock,
    input logic reset,

    // ---------- Predict request (Inputs from Instruction Fetch) ----------
    input BP_PREDICT_REQUEST predict_req_i,

    // ---------- Predict response (Outputs to Instruction Fetch) ----------
    output BP_PREDICT_RESPONSE predict_resp_o,

    // ---------- Training request (Inputs from Rob/Retire Stage) ----------
    input BP_TRAIN_REQUEST train_req_i,

    // ---------- Recovery request (Inputs from Rob/Retire Stage) ----------
    input BP_RECOVER_REQUEST recover_req_i,

    // ---------- Debug outputs ----------
    output logic [`BP_GH-1:0] ghr_dbg,
    output logic [`BP_PHT_BITS-1:0] pht_idx_dbg,
    output BP_COUNTER_STATE pht_entry_dbg,
    output logic btb_hit_dbg,
    output logic train_actual_taken_dbg,
    output logic [`BP_PHT_BITS-1:0] train_pht_idx_dbg,
    output BP_COUNTER_STATE train_pht_old_entry_dbg,
    output BP_COUNTER_STATE train_pht_new_entry_dbg,
    output logic train_valid_dbg,
    output logic [`BP_GH-1:0] next_ghr_dbg,
    output logic [`BP_GH-1:0] recovery_next_dbg,
    output logic [`BP_GH-1:0] recover_snapshot_dbg,
    output logic [`BP_GH-1:0] train_snapshot_dbg,
    output logic recovery_active_dbg,
    output logic [`BP_GH-1:0] ghr_next_ff_dbg,
    output logic recovery_active_comb_dbg,  // Combinational version for current cycle
    output logic recover_pulse_comb_dbg,
    output logic train_valid_comb_dbg,
    output logic [`BP_GH-1:0] ghr_next_value_comb_dbg  // Combinational ghr_next_value
);

    // Local parameters derived from macros
    localparam int unsigned PHT_ENTRY_COUNT = (1 << `BP_PHT_BITS);
    localparam int unsigned BTB_ENTRY_COUNT = (1 << `BP_BTB_BITS);

    // Parameter constraints
    initial begin
        assert (`BP_GH <= `BP_PHT_BITS)
        else $fatal("Global history bits (GH=%0d) must be <= PHT index bits (PHT_BITS=%0d)", `BP_GH, `BP_PHT_BITS);
    end

    // Internal storage
    logic [`BP_GH-1:0] global_history_reg, global_history_next;
    logic            [`BP_GH-1:0] ghr_next_value;
    logic                         recovery_active;
    BP_COUNTER_STATE              pattern_history_table[PHT_ENTRY_COUNT];
    BP_BTB_ENTRY                  btb_array            [BTB_ENTRY_COUNT];

    // Index calculations (combinational)
    BP_INDICES prediction_indices, training_indices;

    // BTB hit signal for debug
    logic btb_hit;

    // Helper functions
    function automatic logic is_btb_hit(logic request_valid, logic prediction_taken, BP_BTB_ENTRY btb_entry,
                                        logic [`BP_BTB_TAG_BITS-1:0] btb_tag);
        return request_valid && prediction_taken && btb_entry.valid && (btb_entry.tag == btb_tag);
    endfunction

    function automatic BP_COUNTER_STATE update_counter(BP_COUNTER_STATE current_counter, logic branch_taken);
        case (current_counter)
            STRONGLY_NOT_TAKEN: return branch_taken ? WEAKLY_NOT_TAKEN : STRONGLY_NOT_TAKEN;
            WEAKLY_NOT_TAKEN: return branch_taken ? WEAKLY_TAKEN : STRONGLY_NOT_TAKEN;
            WEAKLY_TAKEN: return branch_taken ? STRONGLY_TAKEN : WEAKLY_NOT_TAKEN;
            STRONGLY_TAKEN: return branch_taken ? STRONGLY_TAKEN : WEAKLY_TAKEN;
        endcase
    endfunction

    function automatic BP_INDICES compute_indices(logic [31:0] pc, logic [`BP_GH-1:0] global_history);
        BP_INDICES computed_indices;
        computed_indices.pht_idx = pc[`BP_PC_WORD_ALIGN_BITS+:`BP_PHT_BITS] ^ {{(`BP_PHT_BITS - `BP_GH) {1'b0}}, global_history};
        computed_indices.btb_idx = pc[`BP_PC_WORD_ALIGN_BITS+:`BP_BTB_BITS];
        computed_indices.btb_tag = pc[`BP_PC_WORD_ALIGN_BITS+`BP_BTB_BITS+:`BP_BTB_TAG_BITS];
        return computed_indices;
    endfunction

    // Index calculations
    always_comb begin
        prediction_indices = compute_indices(predict_req_i.pc, global_history_reg);
        training_indices   = compute_indices(train_req_i.pc, train_req_i.ghr_snapshot);
    end

    // Prediction logic
    always_comb begin
        predict_resp_o.ghr_snapshot = global_history_reg;  // Snapshot the precise history used for this prediction

        // Determine taken prediction from PHT counter MSB
        predict_resp_o.taken = predict_req_i.valid ? pattern_history_table[prediction_indices.pht_idx][1] : 1'b0;  // MSB of counter is the taken bit

        // Check BTB hit
        btb_hit = is_btb_hit(predict_req_i.valid, predict_resp_o.taken, btb_array[prediction_indices.btb_idx],
                             prediction_indices.btb_tag);
        predict_resp_o.target = btb_hit ? btb_array[prediction_indices.btb_idx].target : 32'h0; // Provide target only on a definite BTB hit

        // Update global history register (don't update speculatively during recovery)
        if (recover_req_i.pulse) begin
            global_history_next = global_history_reg;
        end else if (predict_req_i.valid) begin
            global_history_next = {global_history_reg[`BP_GH-2:0], predict_resp_o.taken};
        end else begin
            global_history_next = global_history_reg;
        end
    end

    // Debug outputs (moved outside always_comb)
    assign ghr_dbg = global_history_reg;
    assign pht_idx_dbg = prediction_indices.pht_idx;
    assign pht_entry_dbg = pattern_history_table[prediction_indices.pht_idx];
    assign btb_hit_dbg = btb_hit;

    // Training debug
    assign train_actual_taken_dbg = train_req_i.valid ? train_req_i.actual_taken : 1'b0;
    assign train_pht_idx_dbg = training_indices.pht_idx;
    assign train_pht_old_entry_dbg = train_req_i.valid ? pattern_history_table[training_indices.pht_idx] : '0;
    assign train_pht_new_entry_dbg = train_req_i.valid ? update_counter(
        pattern_history_table[training_indices.pht_idx], train_req_i.actual_taken
    ) : '0;
    assign train_valid_dbg = train_req_i.valid;

    // Add to debug ports
    assign next_ghr_dbg = global_history_next;
    assign recovery_next_dbg = recover_req_i.pulse ? {recover_req_i.ghr_snapshot[`BP_GH-2:0], train_req_i.actual_taken} : global_history_reg;
    assign recover_snapshot_dbg = recover_req_i.ghr_snapshot;
    assign train_snapshot_dbg = train_req_i.ghr_snapshot;
    assign recovery_active_comb_dbg = recovery_active;  // Combinational version
    assign recover_pulse_comb_dbg = recover_req_i.pulse;
    assign train_valid_comb_dbg = train_req_i.valid;
    assign ghr_next_value_comb_dbg = ghr_next_value;  // Combinational ghr_next_value

    // Compute next GHR value combinational-logic-side
    always_comb begin
        recovery_active = recover_req_i.pulse && train_req_i.valid;
        if (recovery_active) begin
            ghr_next_value = {recover_req_i.ghr_snapshot[`BP_GH-2:0], train_req_i.actual_taken};
        end else if (recover_req_i.pulse) begin
            ghr_next_value = {recover_req_i.ghr_snapshot[`BP_GH-2:0], 1'b0};
        end else begin
            ghr_next_value = global_history_next;
        end
    end

    // Sequential logic: register updates and training
    always_ff @(posedge clock) begin
        if (reset) begin
            if (reset) begin
                // Initialize all structures on reset
                global_history_reg      <= '0;
                pattern_history_table   <= '{default: WEAKLY_NOT_TAKEN};  // Initialize to weakly not-taken
                btb_array               <= '{default: '0};                // Every BTB entry all zeros
            end

        end else begin
            // Handle mispredict recovery or normal GHR update
            // Use pre-computed ghr_next_value (handles recovery priority)
            global_history_reg <= ghr_next_value;

            // Debug outputs (registered)
            recovery_active_dbg <= recovery_active;
            ghr_next_ff_dbg <= ghr_next_value;

            // Handle training updates (independent of GHR update)
            if (train_req_i.valid) begin
                // Update PHT counter
                pattern_history_table[training_indices.pht_idx] <= update_counter(
                    pattern_history_table[training_indices.pht_idx], train_req_i.actual_taken
                );

                // Update BTB on taken branches
                if (train_req_i.actual_taken) begin
                    btb_array[training_indices.btb_idx].valid <= 1'b1;
                    btb_array[training_indices.btb_idx].tag <= training_indices.btb_tag;
                    btb_array[training_indices.btb_idx].target <= train_req_i.actual_target;
                end
            end
        end
    end
endmodule
