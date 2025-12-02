`timescale 1ns / 1ns

module final_tb;

    // Inputs
    reg clock_proc;
    reg clock_mem;
    reg rst;

    // Outputs
    wire halt;

    // Instantiate the Unit Under Test (UUT)
    Processor uut (
        .clock_proc(clock_proc), 
        .clock_mem(clock_mem), 
        .rst(rst), 
        .halt(halt)
    );

    initial begin
        clock_proc = 0;
        clock_mem = 0;
        forever begin
            #5 clock_proc = 1;
            #5 clock_mem = 1;
            #5 clock_proc = 0;
            #5 clock_mem = 0;
        end
    end

    // Task to write instructions directly into memory
    task write_inst;
        input [31:0] byte_addr;
        input [31:0] instruction;
        begin
            uut.memory.mem_array[byte_addr >> 2] = instruction;
        end
    endtask

    // Task to check register values and print PASS/FAIL
    task check_reg;
        input [4:0]  reg_idx;
        input [31:0] expected_val;
        input [255:0] test_name; 
        reg [31:0] actual_val;
        begin
            actual_val = uut.datapath.rf.regs[reg_idx];
            if (actual_val === expected_val) begin
                $display("[PASS] %0s | x%0d = %0d (Hex: %h)", test_name, reg_idx, $signed(actual_val), actual_val);
            end else begin
                $display("[FAIL] %0s | x%0d = %0d (Exp: %0d) (Hex Act: %h vs Exp: %h)", 
                         test_name, reg_idx, $signed(actual_val), $signed(expected_val), actual_val, expected_val);
            end
        end
    endtask

    initial begin
        rst = 1;
        #1; 

        // --- 1. SETUP PHASE (x1, x2, x3) ---
        write_inst(0,  32'h00A00093); // addi x1, x0, 10
        write_inst(4,  32'h00300113); // addi x2, x0, 3
        write_inst(8,  32'hFFB00193); // addi x3, x0, -5

        // --- 2. ALU TESTS (x4 - x12) ---
        write_inst(12, 32'h00211213); // slli x4, x2, 2
        write_inst(16, 32'h0011D293); // srli x5, x3, 1
        write_inst(20, 32'h4011D313); // srai x6, x3, 1
        write_inst(24, 32'h002083B3); // add x7, x1, x2
        write_inst(28, 32'h40208433); // sub x8, x1, x2
        write_inst(32, 32'h002114B3); // sll x9, x2, x2
        write_inst(36, 32'h0020F533); // and x10, x1, x2
        write_inst(40, 32'h0020E5B3); // or x11, x1, x2
        write_inst(44, 32'h0020C633); // xor x12, x1, x2

        // --- 3. RV32M EXTENSION TESTS (x13 - x18) ---
        write_inst(48, 32'h022086B3); // mul x13, x1, x2
        
        // MULH Setup
        write_inst(52, 32'h40000F37); // lui x30, 0x40000
        write_inst(56, 32'h01000F93); // addi x31, x0, 16
        write_inst(60, 32'h03FF1733); // mulh x14, x30, x31

        write_inst(64, 32'h0220C7B3); // div x15, x1, x2
        write_inst(68, 32'h0211D833); // divu x16, x3, x1
        write_inst(72, 32'h0220E8B3); // rem x17, x1, x2
        write_inst(76, 32'h0211F933); // remu x18, x3, x1

        // --- 4. BRANCH TESTS (x19 - x24) ---
        write_inst(80, 32'h00108463); // beq x1, x1, +8 (Taken)
        write_inst(84, 32'hFFF00993); // addi x19, x0, -1 (Fail)
        write_inst(88, 32'h00100993); // addi x19, x0, 1 (Pass)

        write_inst(92, 32'h00209463); // bne x1, x2, +8 (Taken)
        write_inst(96, 32'hFFF00A13); // addi x20, x0, -1 (Fail)
        write_inst(100, 32'h00100A13); // addi x20, x0, 1 (Pass)

        write_inst(104, 32'h0011C463); // blt x3, x1, +8 (Taken)
        write_inst(108, 32'hFFF00A93); // addi x21, x0, -1 (Fail)
        write_inst(112, 32'h00100A93); // addi x21, x0, 1 (Pass)

        write_inst(116, 32'h0030D463); // bge x1, x3, +8 (Taken)
        write_inst(120, 32'hFFF00B13); // addi x22, x0, -1 (Fail)
        write_inst(124, 32'h00100B13); // addi x22, x0, 1 (Pass)

        write_inst(128, 32'h0030E463); // bltu x1, x3, +8 (Taken)
        write_inst(132, 32'hFFF00B93); // addi x23, x0, -1 (Fail)
        write_inst(136, 32'h00100B93); // addi x23, x0, 1 (Pass)

        // --- BRANCH NOT TAKEN + FAKE JUMP FIX ---
        // 140: beq x1, x2, +12    (Not Taken)
        write_inst(140, 32'h00C08663); 
        
        // 144: addi x24, x0, 1    (Pass)
        write_inst(144, 32'h00100C13); 
        
        // 148: beq x0, x0, +8     (ALWAYS TAKEN "JUMP" -> Target 156)
        write_inst(148, 32'h00000463); 
        
        // 152: addi x24, x0, -1   (Fail)
        write_inst(152, 32'hFFF00C13);

        // --- 5. LOAD/STORE TESTS (x25 - x29) ---
        write_inst(156, 32'h40000F13); // addi x30, x0, 1024
        write_inst(160, 32'h001F2023); // sw x1, 0(x30)
        write_inst(164, 32'h003F1223); // sh x3, 4(x30)
        write_inst(168, 32'h002F0423); // sb x2, 8(x30)

        write_inst(172, 32'h000F2C83); // lw x25, 0(x30)
        write_inst(176, 32'h004F1D03); // lh x26, 4(x30)
        write_inst(180, 32'h004F5D83); // lhu x27, 4(x30)
        write_inst(184, 32'h008F0E03); // lb x28, 8(x30)
        write_inst(188, 32'h008F4E83); // lbu x29, 8(x30)

        // --- 6. JAL TESTS (x10, x11) ---
        // We reuse x10 and x11 since previous tests are already done.
        
        // 192: jal x10, +12  (Target = 192 + 12 = 204)
        // x10 should become 192 + 4 = 196
        write_inst(192, 32'h00C0056F); 

        // 196: addi x11, x0, -1 (Skipped)
        write_inst(196, 32'hFFF00593);

        // 200: addi x11, x0, -2 (Skipped)
        write_inst(200, 32'hFFE00593);

        // 204: addi x11, x0, 314 (Target landed!)
        write_inst(204, 32'h13A00593); // 314 = 0x13A

        // 208: ecall
        write_inst(208, 32'h00000073);

        // --- RUN SIMULATION ---
        #20;
        rst = 0;
        wait(halt);
        #20; 

        // --- VERIFICATION ---
        $display("\n--- 1. Setup Tests ---");
        check_reg(1,  10,            "ADDI x1");
        check_reg(2,  3,             "ADDI x2");
        check_reg(3,  -5,            "ADDI x3");

        $display("\n--- 2. ALU Tests ---");
        check_reg(4,  12,            "SLLI (3 << 2)");
        check_reg(5,  32'h7FFFFFFD, "SRLI (-5 >> 1 Log)");
        check_reg(6,  -3,            "SRAI (-5 >>> 1 Arith)");
        check_reg(7,  13,            "ADD  (10 + 3)");
        check_reg(8,  7,             "SUB  (10 - 3)");
        check_reg(9,  24,            "SLL  (3 << 3)");
        check_reg(10, 196,           "JAL Link Address (x10)"); // Updated check for JAL
        check_reg(11, 314,           "JAL Target Value (x11)"); // Updated check for JAL
        check_reg(12, 9,             "XOR  (10 ^ 3)");

        $display("\n--- 3. M-Extension Tests ---");
        check_reg(13, 30,            "MUL  (10 * 3)");
        check_reg(14, 4,             "MULH (High 2^30*16)");
        check_reg(15, 3,             "DIV  (10 / 3)");
        check_reg(16, 429496729,     "DIVU (0xFFFFFFFB/10)");
        check_reg(17, 1,             "REM  (10 % 3)");
        check_reg(18, 1,             "REMU (0xFFFFFFFB%10)");

        $display("\n--- 4. Branch Tests (1=Pass) ---");
        check_reg(19, 1,             "BEQ  (Taken)");
        check_reg(20, 1,             "BNE  (Taken)");
        check_reg(21, 1,             "BLT  (Taken)");
        check_reg(22, 1,             "BGE  (Taken)");
        check_reg(23, 1,             "BLTU (Taken)");
        check_reg(24, 1,             "BEQ  (Not Taken)");

        $display("\n--- 5. Load/Store Tests ---");
        check_reg(25, 10,            "LW   (Word)");
        check_reg(26, -5,            "LH   (Half Signed)");
        check_reg(27, 65531,         "LHU  (Half Unsigned)");
        check_reg(28, 3,             "LB   (Byte Signed)");
        check_reg(29, 3,             "LBU  (Byte Unsigned)");

        $finish;
    end

endmodule