`timescale 1ns / 1ns

module hazard_tb;

    // Inputs
    reg clk;
    reg rst;

    // Outputs
    wire halt;
    wire [31:0] trace_writeback_pc;
    wire [31:0] trace_writeback_inst;

    // Instantiate the Unit Under Test (UUT)
    Processor uut (
        .clk(clk), 
        .rst(rst), 
        .halt(halt),
        .trace_writeback_pc(trace_writeback_pc),
        .trace_writeback_inst(trace_writeback_inst)
    );

    // Clock Generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Helper: Write 32-bit instruction to memory
    task write_inst;
        input [31:0] byte_addr;
        input [31:0] instruction;
        begin
            uut.memory.mem_array[byte_addr >> 2] = instruction;
        end
    endtask

    // Helper: Check register value
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
        // Initialize Inputs
        rst = 1;

        // --- INSTRUCTION SEQUENCE SETUP ---
        // 1. Setup Data
        // x1 = 100
        write_inst(0,  32'h06400093); // addi x1, x0, 100
        // x2 = 10
        write_inst(4,  32'h00A00113); // addi x2, x0, 10
        // x5 = 5
        write_inst(8,  32'h00500293); // addi x5, x0, 5

        // 2. Back-to-Back Independent Divides
        // div x3, x1, x2  -> 100 / 10 = 10
        write_inst(12, 32'h0220C1B3); 
        // div x4, x1, x5  -> 100 / 5  = 20
        write_inst(16, 32'h0250C233); 

        // 3. Dependent Divides
        // div x6, x1, x2  -> 100 / 10 = 10
        write_inst(20, 32'h0220C333); 
        
        // div x7, x6, x5  -> x6 / 5 = 10 / 5 = 2  (Must wait for x6)
        // PREVIOUS INCORRECT HEX: 32'h0253C3B3 (decoded to div x7, x7, x5)
        // CORRECT HEX: 32'h025343B3 (div x7, x6, x5)
        write_inst(24, 32'h025343B3); 

        // 4. Divide followed by Dependent ALU (RAW Hazard)
        // div x8, x1, x5  -> 100 / 5 = 20
        write_inst(28, 32'h0250C433);
        // addi x9, x8, 1  -> 20 + 1 = 21  (Must wait/forward from x8)
        write_inst(32, 32'h00140493);

        // 5. Halt
        write_inst(36, 32'h00000073); // ecall

        // --- RUN SIMULATION ---
        #10;
        rst = 0;
        
        // Wait for halt or timeout
        fork : wait_loop
            wait(halt);
            #2000 disable wait_loop; // Timeout after 2000ns if stalled indefinitely
        join

        #20; // Allow final writebacks

        // --- VERIFY RESULTS ---
        $display("\n--- Hazard Test Results ---");
        
        // Setup checks
        check_reg(1, 100, "Setup x1");
        check_reg(2, 10,  "Setup x2");
        check_reg(5, 5,   "Setup x5");

        $display("\n--- Independent Divides ---");
        check_reg(3, 10, "Div 1 (100/10)");
        check_reg(4, 20, "Div 2 (100/5) - Back-to-back");

        $display("\n--- Dependent Divides ---");
        check_reg(6, 10, "Div 3 (100/10) - Dependency Source");
        check_reg(7, 2,  "Div 4 (x6/5) - Dependent Consumer");

        $display("\n--- Divide -> ALU RAW Hazard ---");
        check_reg(8, 20, "Div 5 (100/5) - Dependency Source");
        check_reg(9, 21, "Addi (x8 + 1) - Consumer (MX Bypass test)");

        $finish;
    end

endmodule