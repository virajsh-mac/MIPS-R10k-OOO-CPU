/////////////////////////////////////////////////////////////////////////
//                                                                     //
//  Modulename :  bp.sv                                                //
//                                                                     //
//  Description :  Branch Predictor module; implements a hybrid        //
//                 gshare + BTB predictor with global history.         //
//                 Handles prediction requests from fetch, training    //
//                 from retire, and mispredict recovery.               //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

module bp (
    input logic clock,
    input logic reset,

    // Fetch stage IOs
    input BP_PREDICT_REQUEST predict_req_i,
    output BP_PREDICT_RESPONSE predict_resp_o,

    // Retire stage IOs
    input BP_TRAIN_REQUEST train_req_i,
);

    // Local parameters derived from macros
    localparam PHT_ENTRY_COUNT = (1 << `BP_PHT_BITS);
    localparam BTB_ENTRY_COUNT = (1 << `BP_BTB_BITS);

    // Internal storage
    logic                     [`BP_GHR_WIDTH-1:0] ghr, ghr_next;
    logic                                  recovery_active;
    BP_COUNTER_STATE [PHT_ENTRY_COUNT-1:0] pattern_history_table;
    BP_BTB_ENTRY     [PHT_ENTRY_COUNT-1:0] btb_array;

    BP_INDICES prediction_indices, training_indices;
    logic btb_hit;
    logic predict_taken;

    // Helper functions
    function automatic BP_COUNTER_STATE update_counter(BP_COUNTER_STATE current_counter, logic branch_taken);
        case (current_counter)
            STRONGLY_NOT_TAKEN: return branch_taken ? WEAKLY_NOT_TAKEN : STRONGLY_NOT_TAKEN;
            WEAKLY_NOT_TAKEN: return branch_taken ? WEAKLY_TAKEN : STRONGLY_NOT_TAKEN;
            WEAKLY_TAKEN: return branch_taken ? STRONGLY_TAKEN : WEAKLY_NOT_TAKEN;
            STRONGLY_TAKEN: return branch_taken ? STRONGLY_TAKEN : WEAKLY_TAKEN;
        endcase
    endfunction

    function automatic BP_INDICES compute_indices(logic [31:0] pc, logic [`BP_GHR_WIDTH-1:0] global_history);
        BP_INDICES computed_indices;
        computed_indices.pht_idx = pc[`BP_PC_WORD_ALIGN_BITS+:`BP_PHT_BITS] ^ {{(`BP_PHT_BITS - `BP_GHR_WIDTH) {1'b0}}, global_history};
        computed_indices.btb_idx = pc[`BP_PC_WORD_ALIGN_BITS+:`BP_BTB_BITS];
        computed_indices.btb_tag = pc[`BP_PC_WORD_ALIGN_BITS+`BP_BTB_BITS+:`BP_BTB_TAG_BITS];
        return computed_indices;
    endfunction

    // Index calculations
    always_comb begin
        prediction_indices = compute_indices(predict_req_i.pc, ghr);
        training_indices   = compute_indices(train_req_i.pc, train_req_i.ghr_snapshot);
    end

    // Prediction logic
    always_comb begin
        predict_resp_o.valid = predict_req_i.valid;
        predict_resp_o.ghr_snapshot = ghr;
        predict_taken = pattern_history_table[prediction_indices.pht_idx][1];  // MSB of counter is the taken bit
        ghr_next = ghr;

        btb_hit = btb_array[prediction_indices.btb_idx].valid && btb_array[prediction_indices.btb_idx].tag == prediction_indices.btb_tag;

        predict_resp_o.target = (predict_taken && btb_hit) ? btb_array[prediction_indices.btb_idx].target : (predict_req_i.pc + 32'h4);
    end

    // ghr_next logic
    always_comb begin
        ghr_next = ghr;
        if (train_req_i.valid && train_req_i.mispredict) begin // a retring branch instruction was mispredicted
            ghr_next = {train_req_i.ghr_snapshot[`BP_GHR_WIDTH-2:0], actual_taken};
        end else if ()begin // from fetch
            ghr_next = {ghr[`BP_GHR_WIDTH-2:0], predict_taken};
        end 
    end 

    always_ff @(posedge clock) begin
        if (reset) begin
            ghr      <= '0;
            pattern_history_table   <= '0;  // Initialize to weakly not-taken
            btb_array               <= '0;  // Every BTB entry all zeros
        end else begin
            ghr <= ghr_next;

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
