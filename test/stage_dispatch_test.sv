`include "sys_defs.svh"

// Basic testbench for stage_dispatch module
// Tests basic functionality for synthesis verification

module testbench;

    logic clock, reset;
    logic                                                                        mispredict;
    logic                                                                        failed;

    // Inputs to stage_dispatch
    FETCH_DISP_PACKET                                                            fetch_packet;
    logic                   [                         `N-1:0]                    fetch_valid;
    logic                   [          $clog2(`ROB_SZ+1)-1:0]                    free_slots_rob;
    logic                   [$clog2(`PHYS_REG_SZ_R10K+1)-1:0]                    free_slots_freelst;
    ROB_IDX                 [                         `N-1:0]                    rob_alloc_idxs;
    logic                   [                         `N-1:0][   `RS_ALU_SZ-1:0] rs_alu_granted;
    logic                   [                         `N-1:0][  `RS_MULT_SZ-1:0] rs_mult_granted;
    logic                   [                         `N-1:0][`RS_BRANCH_SZ-1:0] rs_branch_granted;
    logic                   [                         `N-1:0][   `RS_MEM_SZ-1:0] rs_mem_granted;

    // Outputs from stage_dispatch
    logic                   [                 $clog2(`N)-1:0]                    dispatch_count;
    ROB_ENTRY               [                         `N-1:0]                    rob_entry_packet;
    RS_ALLOC_BANKS                                                               rs_alloc;
    logic                   [                         `N-1:0]                    free_alloc_valid;
    PHYS_TAG                [                         `N-1:0]                    allocated_phys;
    MAP_TABLE_WRITE_REQUEST [                         `N-1:0]                    maptable_write_reqs;
    MAP_TABLE_READ_REQUEST                                                       maptable_read_req;

    // For map table responses (we'll drive these)
    MAP_TABLE_READ_RESPONSE                                                      maptable_read_resp;

    // Test helper variables
    logic                                                                        expected_count;
    logic                                                                        expected_rob_valid;
    logic                                                                        expected_free_valid;

    stage_dispatch dut (
        .clock(clock),
        .reset(reset),
        .fetch_packet(fetch_packet),
        .fetch_valid(fetch_valid),
        .free_slots_rob(free_slots_rob),
        .free_slots_freelst(free_slots_freelst),
        .rob_alloc_idxs(rob_alloc_idxs),
        .rs_alu_granted(rs_alu_granted),
        .rs_mult_granted(rs_mult_granted),
        .rs_branch_granted(rs_branch_granted),
        .rs_mem_granted(rs_mem_granted),
        .dispatch_count(dispatch_count),
        .rob_entry_packet(rob_entry_packet),
        .rs_alloc(rs_alloc),
        .free_alloc_valid(free_alloc_valid),
        .allocated_phys(allocated_phys),
        .maptable_write_reqs(maptable_write_reqs),
        .maptable_read_req(maptable_read_req),
        .maptable_read_resp(maptable_read_resp)
    );

    always begin
        #(`CLOCK_PERIOD / 2.0);
        clock = ~clock;
    end

    // Helper function to create a default empty fetch packet
    function FETCH_DISP_PACKET empty_fetch_packet;
        empty_fetch_packet = '0;
        for (int i = 0; i < `N; i++) begin
            empty_fetch_packet.uses_rd[i] = 1'b0;
            empty_fetch_packet.op_type[i] = OP_ALU_ADD;
        end
    endfunction

    // Helper function to create an ALU instruction
    function void set_alu_inst(ref FETCH_DISP_PACKET pkt, input int idx, input int rs1, rs2, rd);
        pkt.rs1_idx[idx] = rs1;
        pkt.rs2_idx[idx] = rs2;
        pkt.rd_idx[idx] = rd;
        pkt.uses_rd[idx] = (rd != 0);
        pkt.op_type[idx] = '{category: CAT_ALU, func: ADD};
        pkt.opa_select[idx] = OPA_IS_RS1;
        pkt.opb_select[idx] = OPB_IS_RS2;
        pkt.PC[idx] = idx * 4;
        pkt.inst[idx] = 32'h00000000;  // dummy
    endfunction

    // Helper function to create a MULT instruction
    function void set_mult_inst(ref FETCH_DISP_PACKET pkt, input int idx, input int rs1, rs2, rd);
        pkt.rs1_idx[idx] = rs1;
        pkt.rs2_idx[idx] = rs2;
        pkt.rd_idx[idx] = rd;
        pkt.uses_rd[idx] = (rd != 0);
        pkt.op_type[idx] = '{category: CAT_MULT, func: MUL};
        pkt.opa_select[idx] = OPA_IS_RS1;
        pkt.opb_select[idx] = OPB_IS_RS2;
        pkt.PC[idx] = idx * 4;
        pkt.inst[idx] = 32'h00000000;  // dummy
    endfunction

    // Helper function to create a BRANCH instruction
    function void set_branch_inst(ref FETCH_DISP_PACKET pkt, input int idx, input int rs1, rs2);
        pkt.rs1_idx[idx] = rs1;
        pkt.rs2_idx[idx] = rs2;
        pkt.rd_idx[idx] = 0;
        pkt.uses_rd[idx] = 1'b0;
        pkt.op_type[idx] = '{category: CAT_BRANCH, func: EQ};
        pkt.opa_select[idx] = OPA_IS_RS1;
        pkt.opb_select[idx] = OPB_IS_RS2;
        pkt.PC[idx] = idx * 4;
        pkt.inst[idx] = 32'h00000000;  // dummy
    endfunction

    // Helper function to create a MEM instruction
    function void set_mem_inst(ref FETCH_DISP_PACKET pkt, input int idx, input int rs1, rs2, rd);
        pkt.rs1_idx[idx] = rs1;
        pkt.rs2_idx[idx] = rs2;
        pkt.rd_idx[idx] = rd;
        pkt.uses_rd[idx] = (rd != 0);
        pkt.op_type[idx] = '{category: CAT_MEM, func: LOAD_WORD};
        pkt.opa_select[idx] = OPA_IS_RS1;
        pkt.opb_select[idx] = OPB_IS_RS2;
        pkt.PC[idx] = idx * 4;
        pkt.inst[idx] = 32'h00000000;  // dummy
    endfunction

    // Helper to create map table read responses (ready entries)
    function MAP_TABLE_READ_RESPONSE ready_map_response;
        for (int i = 0; i < `N; i++) begin
            ready_map_response.rs1_entries[i]  = '{phys_reg: PHYS_TAG'(i * 4), ready: 1'b1};
            ready_map_response.rs2_entries[i]  = '{phys_reg: PHYS_TAG'(i * 4 + 1), ready: 1'b1};
            ready_map_response.told_entries[i] = '{phys_reg: PHYS_TAG'(i * 4 + 2), ready: 1'b1};
        end
    endfunction

    // Helper to print dispatch results
    task print_dispatch_results(input string label);
        $display("\n=== %s ===", label);
        $display("Dispatch count: %0d", dispatch_count);
        $display("ROB entries: %p", rob_entry_packet);
        $display("RS alloc ALU: valid=%b", rs_alloc.alu.valid);
        $display("RS alloc MULT: valid=%b", rs_alloc.mult.valid);
        $display("RS alloc BRANCH: valid=%b", rs_alloc.branch.valid);
        $display("RS alloc MEM: valid=%b", rs_alloc.mem.valid);
        $display("Free alloc valid: %b", free_alloc_valid);
        $display("Map table writes: %p", maptable_write_reqs);
        $display("");
    endtask

    // Helper to reset and wait for proper timing
    task reset_dut;
        reset = 1;
        @(negedge clock);
        @(negedge clock);
        reset = 0;
        @(negedge clock);
    endtask

    initial begin
        int test_num = 1;
        clock = 0;
        reset = 1;
        failed = 0;
        mispredict = 0;

        // Initialize inputs
        fetch_packet = empty_fetch_packet();
        fetch_valid = '0;
        free_slots_rob = `ROB_SZ;
        free_slots_freelst = 32;
        rob_alloc_idxs = '0;
        rs_alu_granted = '1;  // Assume grants available
        rs_mult_granted = '1;
        rs_branch_granted = '1;
        rs_mem_granted = '1;
        allocated_phys = '0;
        maptable_read_resp = ready_map_response();

        reset_dut();

        // Test 1: Single ALU instruction dispatch
        $display("\nTest %0d: Single ALU instruction dispatch", test_num++);
        reset_dut();
        begin
            fetch_packet = empty_fetch_packet();
            set_alu_inst(fetch_packet, 0, 1, 2, 3);
            fetch_valid[0] = 1'b1;

            @(negedge clock);

            if (dispatch_count == 1 && rob_entry_packet[0].valid && rs_alloc.alu.valid[0] && free_alloc_valid[0]) begin
                $display("  PASS: ALU instruction dispatched correctly");
            end else begin
                $display("  FAIL: ALU instruction not dispatched (count=%0d, rob_valid=%b, rs_valid=%b, free_valid=%b)",
                         dispatch_count, rob_entry_packet[0].valid, rs_alloc.alu.valid[0], free_alloc_valid[0]);
                failed = 1;
            end
        end

        // Test 2: ROB full should prevent dispatch
        $display("\nTest %0d: ROB full prevents dispatch", test_num++);
        reset_dut();
        free_slots_rob = `ROB_SZ;  // Reset to default
        free_slots_freelst = 32;  // Reset to default
        begin
            fetch_packet = empty_fetch_packet();
            set_alu_inst(fetch_packet, 0, 1, 2, 3);
            fetch_valid[0] = 1'b1;
            free_slots_rob = 0;  // ROB full

            @(negedge clock);

            if (dispatch_count == 0) begin
                $display("  PASS: Dispatch stalled when ROB full");
            end else begin
                $display("  FAIL: Should not dispatch when ROB full (count=%0d)", dispatch_count);
                failed = 1;
            end
        end

        // Test 3: Freelist empty should prevent dispatch
        $display("\nTest %0d: Freelist empty prevents dispatch", test_num++);
        reset_dut();
        free_slots_rob = `ROB_SZ;  // Reset to default
        free_slots_freelst = 32;  // Reset to default
        begin
            fetch_packet = empty_fetch_packet();
            set_alu_inst(fetch_packet, 0, 1, 2, 3);
            fetch_valid[0] = 1'b1;
            free_slots_freelst = 0;  // Freelist empty

            @(negedge clock);

            if (dispatch_count == 0) begin
                $display("  PASS: Dispatch stalled when freelist empty");
            end else begin
                $display("  FAIL: Should not dispatch when freelist empty (count=%0d)", dispatch_count);
                failed = 1;
            end
        end

        // Test 4: Multiple instruction categories dispatch
        $display("\nTest %0d: Multiple instruction categories dispatch", test_num++);
        reset_dut();
        free_slots_rob = `ROB_SZ;  // Reset to default
        free_slots_freelst = 32;  // Reset to default
        begin
            fetch_packet = empty_fetch_packet();
            set_alu_inst(fetch_packet, 0, 1, 2, 3);
            set_mult_inst(fetch_packet, 1, 4, 5, 6);
            set_branch_inst(fetch_packet, 2, 7, 8);
            fetch_valid = 3'b111;

            @(negedge clock);

            if (dispatch_count == 3 && rs_alloc.alu.valid[0] && rs_alloc.mult.valid[0] && rs_alloc.branch.valid[0]) begin
                $display("  PASS: Multiple categories dispatched correctly");
            end else begin
                $display("  FAIL: Multiple categories not dispatched (count=%0d, alu=%b, mult=%b, branch=%b)", dispatch_count,
                         rs_alloc.alu.valid[0], rs_alloc.mult.valid[0], rs_alloc.branch.valid[0]);
                failed = 1;
            end
        end

        // Test 5: No destination register (branch) doesn't allocate from freelist
        $display("\nTest %0d: Branch instruction doesn't allocate from freelist", test_num++);
        reset_dut();
        free_slots_rob     = `ROB_SZ;  // Reset to default
        free_slots_freelst = 32;  // Reset to default
        fetch_valid        = '0;  // Reset fetch_valid
        begin
            fetch_packet = empty_fetch_packet();
            set_branch_inst(fetch_packet, 0, 1, 2);
            fetch_valid[0] = 1'b1;

            @(negedge clock);

            if (dispatch_count == 1 && !free_alloc_valid[0]) begin
                $display("  PASS: Branch doesn't allocate from freelist");
            end else begin
                $display("  FAIL: Branch should not allocate from freelist (count=%0d, free_valid=%b)", dispatch_count,
                         free_alloc_valid[0]);
                failed = 1;
            end
        end

        // Test 6: Map table write requests generated
        $display("\nTest %0d: Map table write requests generated", test_num++);
        reset_dut();
        free_slots_rob = `ROB_SZ;  // Reset to default
        free_slots_freelst = 32;  // Reset to default
        begin
            fetch_packet = empty_fetch_packet();
            set_alu_inst(fetch_packet, 0, 1, 2, 3);
            fetch_valid[0] = 1'b1;

            @(negedge clock);

            if (maptable_write_reqs[0].valid && maptable_write_reqs[0].addr == 3) begin
                $display("  PASS: Map table write request generated for rd=3");
            end else begin
                $display("  FAIL: Map table write request not generated correctly (valid=%b, addr=%0d)",
                         maptable_write_reqs[0].valid, maptable_write_reqs[0].addr);
                failed = 1;
            end
        end

        // Test 7: Combinational behavior - no valid inputs = no outputs
        $display("\nTest %0d: Combinational behavior verified", test_num++);
        free_slots_rob = `ROB_SZ;  // Reset to default
        free_slots_freelst = 32;  // Reset to default
        begin
            // Test that with no valid instructions, outputs are inactive
            fetch_packet = empty_fetch_packet();
            fetch_valid  = '0;

            @(negedge clock);

            if (dispatch_count == 0 && !rob_entry_packet[0].valid && !free_alloc_valid[0]) begin
                $display("  PASS: No valid instructions produce no outputs");
            end else begin
                $display("  FAIL: Should have no outputs with no valid instructions (count=%0d, rob_valid=%b, free_valid=%b)",
                         dispatch_count, rob_entry_packet[0].valid, free_alloc_valid[0]);
                failed = 1;
            end
        end

        // Test 8: Mixed valid/invalid instruction bundle
        $display("\nTest %0d: Mixed valid/invalid instructions", test_num++);
        reset_dut();
        free_slots_rob = `ROB_SZ;  // Reset to default
        free_slots_freelst = 32;  // Reset to default
        begin
            fetch_packet = empty_fetch_packet();
            set_alu_inst(fetch_packet, 0, 1, 2, 3);  // Valid ALU
            set_mult_inst(fetch_packet, 1, 4, 5, 6);  // Valid MULT
            // Index 2: Invalid (default empty)
            fetch_valid = 3'b011;  // Only first two valid

            @(negedge clock);

            if (dispatch_count == 2 && rs_alloc.alu.valid[0] && rs_alloc.mult.valid[0] &&
                !rs_alloc.alu.valid[1] && !rs_alloc.mult.valid[1]) begin
                $display("  PASS: Only valid instructions dispatched, invalid ones ignored");
            end else begin
                $display("  FAIL: Mixed valid/invalid dispatch incorrect (count=%0d, alu0=%b, mult0=%b, alu1=%b, mult1=%b)",
                         dispatch_count, rs_alloc.alu.valid[0], rs_alloc.mult.valid[0], rs_alloc.alu.valid[1],
                         rs_alloc.mult.valid[1]);
                failed = 1;
            end
        end

        // Test 9: Partial dispatch due to freelist constraints
        $display("\nTest %0d: Partial dispatch due to freelist", test_num++);
        reset_dut();
        free_slots_rob = `ROB_SZ;  // Reset to default
        begin
            fetch_packet = empty_fetch_packet();
            set_alu_inst(fetch_packet, 0, 1, 2, 3);  // Needs dest reg
            set_alu_inst(fetch_packet, 1, 4, 5, 6);  // Needs dest reg
            set_alu_inst(fetch_packet, 2, 7, 8, 9);  // Needs dest reg
            fetch_valid = 3'b111;
            free_slots_freelst = 2;  // Only enough for 2 instructions

            @(negedge clock);

            if (dispatch_count == 2 && free_alloc_valid[0] && free_alloc_valid[1] && !free_alloc_valid[2]) begin
                $display("  PASS: Partial dispatch when freelist constrained");
            end else begin
                $display("  FAIL: Freelist constraint handling incorrect (count=%0d, free_valid=%b%b%b)", dispatch_count,
                         free_alloc_valid[0], free_alloc_valid[1], free_alloc_valid[2]);
                failed = 1;
            end
        end

        // Test 10: Partial dispatch due to ROB constraints
        $display("\nTest %0d: Partial dispatch due to ROB", test_num++);
        reset_dut();
        free_slots_freelst = 32;  // Reset to default
        begin
            fetch_packet = empty_fetch_packet();
            set_alu_inst(fetch_packet, 0, 1, 2, 3);
            set_alu_inst(fetch_packet, 1, 4, 5, 6);
            set_alu_inst(fetch_packet, 2, 7, 8, 9);
            fetch_valid = 3'b111;
            free_slots_rob = 1;  // Only enough for 1 instruction

            @(negedge clock);

            if (dispatch_count == 1 && rob_entry_packet[0].valid && !rob_entry_packet[1].valid && !rob_entry_packet[2].valid) begin
                $display("  PASS: Partial dispatch when ROB constrained");
            end else begin
                $display("  FAIL: ROB constraint handling incorrect (count=%0d, rob_valid=%b%b%b)", dispatch_count,
                         rob_entry_packet[0].valid, rob_entry_packet[1].valid, rob_entry_packet[2].valid);
                failed = 1;
            end
        end

        // Test 11: Mixed instructions with/without destination registers
        $display("\nTest %0d: Mixed dest/no-dest instructions", test_num++);
        reset_dut();
        free_slots_rob = `ROB_SZ;  // Reset to default
        free_slots_freelst = 32;  // Reset to default
        begin
            fetch_packet = empty_fetch_packet();
            set_alu_inst(fetch_packet, 0, 1, 2, 3);  // Has dest reg
            set_branch_inst(fetch_packet, 1, 4, 5);  // No dest reg
            set_alu_inst(fetch_packet, 2, 6, 7, 8);  // Has dest reg
            fetch_valid = 3'b111;

            @(negedge clock);

            if (dispatch_count == 3 && free_alloc_valid[0] && !free_alloc_valid[1] && free_alloc_valid[2]) begin
                $display("  PASS: Freelist allocation only for instructions with dest regs");
            end else begin
                $display("  FAIL: Dest reg allocation incorrect (count=%0d, free_valid=%b%b%b)", dispatch_count,
                         free_alloc_valid[0], free_alloc_valid[1], free_alloc_valid[2]);
                failed = 1;
            end
        end

        // Test 12: Map table writes only for dispatched instructions with dest regs
        $display("\nTest %0d: Map table writes for partial dispatch", test_num++);
        reset_dut();
        free_slots_rob = `ROB_SZ;  // Reset to default
        free_slots_freelst = 1;  // Only enough for 1 instruction
        begin
            fetch_packet = empty_fetch_packet();
            set_alu_inst(fetch_packet, 0, 1, 2, 10);  // Has dest reg -> should dispatch and write map
            set_alu_inst(fetch_packet, 1, 4, 5, 11);  // Has dest reg -> should not dispatch due to freelist
            set_branch_inst(fetch_packet, 2, 6, 7);  // No dest reg -> should dispatch but not write map
            fetch_valid = 3'b111;

            @(negedge clock);

            if (maptable_write_reqs[0].valid && maptable_write_reqs[0].addr == 10 &&
                !maptable_write_reqs[1].valid && !maptable_write_reqs[2].valid) begin
                $display("  PASS: Map table writes only for dispatched instructions with dest regs");
            end else begin
                $display("  FAIL: Map table write logic incorrect (write0=%b addr0=%0d, write1=%b, write2=%b)",
                         maptable_write_reqs[0].valid, maptable_write_reqs[0].addr, maptable_write_reqs[1].valid,
                         maptable_write_reqs[2].valid);
                failed = 1;
            end
        end

        // Test 13: RS allocation counters work correctly
        $display("\nTest %0d: RS allocation counters", test_num++);
        reset_dut();
        free_slots_rob = `ROB_SZ;  // Reset to default
        free_slots_freelst = 32;  // Reset to default
        begin
            fetch_packet = empty_fetch_packet();
            set_alu_inst(fetch_packet, 0, 1, 2, 3);  // ALU -> index 0
            set_mult_inst(fetch_packet, 1, 4, 5, 6);  // MULT -> index 0
            set_alu_inst(fetch_packet, 2, 7, 8, 9);  // ALU -> index 1
            fetch_valid = 3'b111;

            @(negedge clock);

            if (rs_alloc.alu.valid[0] && rs_alloc.alu.valid[1] && !rs_alloc.alu.valid[2] &&
                rs_alloc.mult.valid[0] && !rs_alloc.mult.valid[1]) begin
                $display("  PASS: RS allocation counters increment correctly");
            end else begin
                $display("  FAIL: RS counters incorrect (alu_valid=%b%b%b, mult_valid=%b%b)", rs_alloc.alu.valid[0],
                         rs_alloc.alu.valid[1], rs_alloc.alu.valid[2], rs_alloc.mult.valid[0], rs_alloc.mult.valid[1]);
                failed = 1;
            end
        end

        // Test 14: Boundary condition - exactly enough resources
        $display("\nTest %0d: Boundary condition - exact resources", test_num++);
        reset_dut();
        begin
            fetch_packet = empty_fetch_packet();
            set_alu_inst(fetch_packet, 0, 1, 2, 3);
            set_alu_inst(fetch_packet, 1, 4, 5, 6);
            fetch_valid        = 2'b11;
            free_slots_rob     = 2;  // Exactly enough ROB slots
            free_slots_freelst = 2;  // Exactly enough freelist slots

            @(negedge clock);

            if (dispatch_count == 2 && rob_entry_packet[0].valid && rob_entry_packet[1].valid &&
                free_alloc_valid[0] && free_alloc_valid[1]) begin
                $display("  PASS: Dispatches all when resources exactly match");
            end else begin
                $display("  FAIL: Boundary condition failed (count=%0d, rob_valid=%b%b, free_valid=%b%b)", dispatch_count,
                         rob_entry_packet[0].valid, rob_entry_packet[1].valid, free_alloc_valid[0], free_alloc_valid[1]);
                failed = 1;
            end
        end

        // Test 15: All instruction categories in one bundle (limited by N=3)
        $display("\nTest %0d: All instruction categories", test_num++);
        reset_dut();
        free_slots_rob = `ROB_SZ;  // Reset to default
        free_slots_freelst = 32;  // Reset to default
        begin
            fetch_packet = empty_fetch_packet();
            set_alu_inst(fetch_packet, 0, 1, 2, 3);  // ALU
            set_mult_inst(fetch_packet, 1, 4, 5, 6);  // MULT
            set_branch_inst(fetch_packet, 2, 7, 8);  // BRANCH
            fetch_valid = 3'b111;  // Only 3 instructions since N=3

            @(negedge clock);

            if (dispatch_count == 3 && rs_alloc.alu.valid[0] && rs_alloc.mult.valid[0] && rs_alloc.branch.valid[0]) begin
                $display("  PASS: All instruction categories dispatched correctly");
            end else begin
                $display("  FAIL: Category dispatch failed (count=%0d, alu=%b, mult=%b, branch=%b)", dispatch_count,
                         rs_alloc.alu.valid[0], rs_alloc.mult.valid[0], rs_alloc.branch.valid[0]);
                failed = 1;
            end
        end

        // Test 16: Instruction ordering in dispatch
        $display("\nTest %0d: Instruction ordering in dispatch", test_num++);
        reset_dut();
        free_slots_rob = `ROB_SZ;  // Reset to default
        free_slots_freelst = 32;  // Reset to default
        begin
            fetch_packet = empty_fetch_packet();
            set_alu_inst(fetch_packet, 0, 1, 2, 10);  // rd=10
            set_alu_inst(fetch_packet, 1, 3, 4, 11);  // rd=11
            set_branch_inst(fetch_packet, 2, 5, 6);  // no rd
            fetch_valid = 3'b111;

            @(negedge clock);

            // Should dispatch all 3, with map table writes for first two only
            if (dispatch_count == 3 &&
                maptable_write_reqs[0].valid && maptable_write_reqs[0].addr == 10 &&
                maptable_write_reqs[1].valid && maptable_write_reqs[1].addr == 11 &&
                !maptable_write_reqs[2].valid) begin
                $display("  PASS: Instructions dispatched in order with correct map table writes");
            end else begin
                $display("  FAIL: Ordering issue (count=%0d, mt0=%b addr0=%0d, mt1=%b addr1=%0d, mt2=%b)", dispatch_count,
                         maptable_write_reqs[0].valid, maptable_write_reqs[0].addr, maptable_write_reqs[1].valid,
                         maptable_write_reqs[1].addr, maptable_write_reqs[2].valid);
                failed = 1;
            end
        end

        // Test 17: Map table read requests always generated
        $display("\nTest %0d: Map table read requests", test_num++);
        reset_dut();
        free_slots_rob = `ROB_SZ;  // Reset to default
        free_slots_freelst = 32;  // Reset to default
        begin
            fetch_packet = empty_fetch_packet();
            set_alu_inst(fetch_packet, 0, 1, 2, 3);
            fetch_valid[0] = 1'b1;

            @(negedge clock);

            if (maptable_read_req.rs1_addrs[0] == 1 && maptable_read_req.rs2_addrs[0] == 2 &&
                maptable_read_req.told_addrs[0] == 3) begin
                $display("  PASS: Map table read requests generated correctly");
            end else begin
                $display("  FAIL: Map table read requests incorrect (rs1=%0d, rs2=%0d, told=%0d)",
                         maptable_read_req.rs1_addrs[0], maptable_read_req.rs2_addrs[0], maptable_read_req.told_addrs[0]);
                failed = 1;
            end
        end

        $display("\n");
        if (failed) begin
            $display("@@@ FAILED");
        end else begin
            $display("@@@ PASSED");
        end

        $finish;
    end

endmodule
