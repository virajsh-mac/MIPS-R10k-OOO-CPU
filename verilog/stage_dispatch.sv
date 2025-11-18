`include "sys_defs.svh"

// Dispatch Stage: Register renaming and resource allocation
module stage_dispatch (
    input logic clock,
    reset,

    // From decode: individual signals
    input REG_IDX [`N-1:0]        decode_rs1_idx,
    input REG_IDX [`N-1:0]        decode_rs2_idx,
    input REG_IDX [`N-1:0]        decode_rd_idx,
    input logic [`N-1:0]          decode_uses_rd,
    input OP_TYPE [`N-1:0]        decode_op_type,
    input ALU_OPA_SELECT [`N-1:0] decode_opa_select,
    input ALU_OPB_SELECT [`N-1:0] decode_opb_select,
    input DATA [`N-1:0]           decode_immediate,
    input logic [`N-1:0]          decode_halt,
    input DATA                    ff_instr       [`N-1:0],
    input ADDR                    ff_pc,
    input logic [`N-1:0]          fetch_valid,

    // Structural hazard inputs
    input logic   [          $clog2(`ROB_SZ+1)-1:0] free_slots_rob,
    input ROB_IDX [                         `N-1:0] rob_alloc_idxs,
    input logic   [$clog2(`PHYS_REG_SZ_R10K+1)-1:0] freelist_free_slots,
    input logic   [       $clog2(`RS_ALU_SZ+1)-1:0] rs_alu_free_slots,
    input logic   [     $clog2(`RS_MULT_SZ+1)-1:0] rs_mult_free_slots,
    input logic   [   $clog2(`RS_BRANCH_SZ+1)-1:0] rs_branch_free_slots,
    input logic   [      $clog2(`RS_MEM_SZ+1)-1:0] rs_mem_free_slots,


    // To fetch: dispatch count (0 = stall)
    output logic [$clog2(`N)-1:0] dispatch_count,

    // To ROB: allocation entries
    output ROB_ENTRY [`N-1:0] rob_entry_packet,

    // To RS: allocation requests
    output RS_ALLOC_BANKS rs_alloc,

    // To freelist: allocation requests
    output logic [`N-1:0]                        free_alloc_valid,
    input  logic [`N-1:0][`PHYS_REG_SZ_R10K-1:0] granted_regs,

    // To/from map table: register mapping
    output MAP_TABLE_WRITE_REQUEST [`N-1:0] maptable_write_reqs,
    output MAP_TABLE_READ_REQUEST           maptable_read_req,
    input  MAP_TABLE_READ_RESPONSE          maptable_read_resp
);

    // Dispatch control
    logic [$clog2(`N+1)-1:0] num_to_dispatch;
    int num_valid_from_fetch;
    int rob_slots_used, freelist_slots_used;
    logic rs_bank_available;

    // RS allocation counters
    int alu_count, mult_count, branch_count, mem_count;

    // Track RS usage dynamically as we go
    int alu_used;
    int mult_used;
    int branch_used;
    int mem_used;

    // Map table read results
    logic [`PHYS_TAG_BITS-1:0] local_reg1_tag[`N-1:0];
    logic [`PHYS_TAG_BITS-1:0] local_reg2_tag[`N-1:0];
    logic local_reg1_ready[`N-1:0];
    logic local_reg2_ready[`N-1:0];
    logic [`PHYS_TAG_BITS-1:0] local_Told[`N-1:0];

    // Extracted physical register allocations from freelist grants
    PHYS_TAG [`N-1:0] allocated_phys;

    // Create RS entry from instruction and map table data
    function RS_ENTRY create_rs_entry(int idx);
        create_rs_entry.valid          = 1'b1;
        create_rs_entry.opa_select     = decode_opa_select[idx];
        create_rs_entry.opb_select     = decode_opb_select[idx];
        create_rs_entry.op_type        = decode_op_type[idx];
        create_rs_entry.src1_tag       = local_reg1_tag[idx];
        create_rs_entry.src1_ready     = local_reg1_ready[idx];
        create_rs_entry.src2_tag       = local_reg2_tag[idx];
        create_rs_entry.src2_ready     = local_reg2_ready[idx];
        create_rs_entry.src2_immediate = decode_immediate[idx];
        create_rs_entry.dest_tag       = allocated_phys[idx];
        create_rs_entry.rob_idx        = rob_alloc_idxs[idx];
        create_rs_entry.PC             = ff_pc + 32'(4 * idx);
        create_rs_entry.pred_taken     = 1'b0;  // No branch prediction
        create_rs_entry.pred_target    = '0;    // No prediction target
    endfunction

    always_comb begin
        // Count valid instructions and check for contiguous validity (ordered dispatch)
        num_valid_from_fetch = 0;
        for (int i = 0; i < `N; i++) begin
            if (fetch_valid[i]) begin
                num_valid_from_fetch = i + 1;  // Must be contiguous from 0
            end else if (num_valid_from_fetch > 0) begin
                break;  // Gap found, stop counting
            end
        end

        // Calculate dispatch count by checking each instruction individually
        // Stop at the first instruction that hits a structural hazard
        num_to_dispatch = 0;
        rob_slots_used = 0;
        freelist_slots_used = 0;

        alu_used = 0;
        mult_used = 0;
        branch_used = 0;
        mem_used = 0;

        for (int i = 0; i < `N; i++) begin
            if (i >= num_valid_from_fetch) break;  // No more valid instructions

            // Check ROB space
            if (rob_slots_used >= free_slots_rob) break;

            // Check freelist space (only if instruction uses a destination register)
            if (decode_uses_rd[i] && freelist_slots_used >= freelist_free_slots) break;

            // Check RS bank space for this instruction's functional unit
            case (decode_op_type[i].category)
                CAT_ALU:    if (alu_used   >= rs_alu_free_slots)    break;
                CAT_MULT:   if (mult_used  >= rs_mult_free_slots)   break;
                CAT_BRANCH: if (branch_used>= rs_branch_free_slots) break;
                CAT_MEM:    if (mem_used   >= rs_mem_free_slots)    break;
                default:    break;
            endcase

            // This instruction can be dispatched
            num_to_dispatch++;
            rob_slots_used++;
            if (decode_uses_rd[i]) freelist_slots_used++;

            // Reserve RS slot for this instruction type
            case (decode_op_type[i].category)
                CAT_ALU:    alu_used++;
                CAT_MULT:   mult_used++;
                CAT_BRANCH: branch_used++;
                CAT_MEM:    mem_used++;
            endcase
        end

        // Set dispatch count (0 = stall)
        dispatch_count = num_to_dispatch;

        // Initialize outputs
        rs_alloc = '0;
        free_alloc_valid = '0;
        rob_entry_packet = '0;

        // Read register mappings from map table
        for (int i = 0; i < `N; i++) begin
            maptable_read_req.rs1_addrs[i]  = decode_rs1_idx[i];
            maptable_read_req.rs2_addrs[i]  = decode_rs2_idx[i];
            maptable_read_req.told_addrs[i] = decode_rd_idx[i];
        end

        // Store map table responses locally
        for (int i = 0; i < `N; i++) begin
            local_reg1_tag[i]   = maptable_read_resp.rs1_entries[i].phys_reg;
            local_reg2_tag[i]   = maptable_read_resp.rs2_entries[i].phys_reg;
            local_reg1_ready[i] = maptable_read_resp.rs1_entries[i].ready;
            local_reg2_ready[i] = maptable_read_resp.rs2_entries[i].ready;

            // Halt instructions don't use source registers, so mark them ready
            if (decode_halt[i]) begin
                local_reg1_ready[i] = 1'b1;
                local_reg2_ready[i] = 1'b1;
            end

            if ((decode_opb_select[i] != OPB_IS_RS2) && !decode_halt[i]) begin
                local_reg2_ready[i] = 1'b1;
            end
            local_Told[i] = maptable_read_resp.told_entries[i].phys_reg;
        end

        // Extract physical register allocations from freelist grants
        // granted_regs[i][j] is one-hot: exactly one bit set, index j = allocated phys reg
        for (int i = 0; i < `N; i++) begin
            allocated_phys[i] = '0;  // Default value
            for (int j = 0; j < `PHYS_REG_SZ_R10K; j++) begin
                if (granted_regs[i][j]) begin
                    allocated_phys[i] = PHYS_TAG'(j);
                end
            end
        end

        // Forward register renaming within dispatch group
        // For each instruction, check if any earlier instruction renamed its source registers
        begin
            PHYS_TAG [`ARCH_REG_SZ-1:0] dispatch_renames;
            logic [`ARCH_REG_SZ-1:0] has_rename;

            // Initialize rename map
            dispatch_renames = '0;
            has_rename = '0;

            // Build rename map incrementally as we process each instruction
            for (int i = 0; i < dispatch_count; i++) begin
                if (fetch_valid[i]) begin
                    // First apply forwarding from previous renames to this instruction
                    if (has_rename[decode_rs1_idx[i]]) begin
                        local_reg1_tag[i] = dispatch_renames[decode_rs1_idx[i]];
                        local_reg1_ready[i] = 1'b0;
                    end
                    if (has_rename[decode_rs2_idx[i]] && decode_opb_select[i] == OPB_IS_RS2) begin
                        local_reg2_tag[i] = dispatch_renames[decode_rs2_idx[i]];
                        local_reg2_ready[i] = 1'b0;
                    end

                    // Then add this instruction's rename to the map for future instructions
                    if (decode_uses_rd[i]) begin
                        dispatch_renames[decode_rd_idx[i]] = allocated_phys[i];
                        has_rename[decode_rd_idx[i]] = 1'b1;
                    end
                end
            end
        end

        // Setup register remapping writes
        for (int i = 0; i < `N; i++) begin
            if (i < dispatch_count && fetch_valid[i] && decode_uses_rd[i])  begin
                maptable_write_reqs[i].valid    = 1'b1;
                maptable_write_reqs[i].addr     = decode_rd_idx[i];
                maptable_write_reqs[i].phys_reg = allocated_phys[i];
            end else begin
                maptable_write_reqs[i] = '0;
            end
        end

        // Build ROB and RS entries for dispatched instructions
        alu_count = 0;
        mult_count = 0;
        branch_count = 0;
        mem_count = 0;

        for (int i = 0; i < dispatch_count; i++) begin
            if (fetch_valid[i]) begin
                // ROB entry
                rob_entry_packet[i] = '{
                    valid: 1'b1,
                    PC: ff_pc + 32'(4 * i),
                    inst: ff_instr[i],
                    arch_rd: decode_rd_idx[i],
                    phys_rd: allocated_phys[i],
                    prev_phys_rd: local_Told[i],
                    complete: 1'b0,
                    exception: NO_ERROR,
                    branch: (decode_op_type[i].category == CAT_BRANCH),
                    pred_target: '0,    // No prediction target
                    pred_taken: 1'b0,   // No branch prediction
                    halt: decode_halt[i],
                    default: '0
                };

                // Route to appropriate RS bank
                case (decode_op_type[i].category)
                    CAT_ALU: begin
                        rs_alloc.alu.valid[alu_count]   = 1'b1;
                        rs_alloc.alu.entries[alu_count] = create_rs_entry(i);
                        alu_count++;
                    end
                    CAT_MULT: begin
                        rs_alloc.mult.valid[mult_count]   = 1'b1;
                        rs_alloc.mult.entries[mult_count] = create_rs_entry(i);
                        mult_count++;
                    end
                    CAT_BRANCH: begin
                        rs_alloc.branch.valid[branch_count]   = 1'b1;
                        rs_alloc.branch.entries[branch_count] = create_rs_entry(i);
                        branch_count++;
                    end
                    CAT_MEM: begin
                        rs_alloc.mem.valid[mem_count]   = 1'b1;
                        rs_alloc.mem.entries[mem_count] = create_rs_entry(i);
                        mem_count++;
                    end
                endcase

                // Request physical register allocation
                free_alloc_valid[i] = decode_uses_rd[i];
            end
        end
    end

endmodule
