`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/19/2025 06:01:12 PM
// Design Name: 
// Module Name: Mul_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module Mul_tb(

    );
    

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

    // -----------------------------------------------------------
    // Clock Generation
    // -----------------------------------------------------------
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

    // -----------------------------------------------------------
    // Helper Task: Write Instruction to Memory
    // -----------------------------------------------------------
    task write_inst;
        input [31:0] byte_addr;
        input [31:0] instruction;
        begin
            uut.memory.mem_array[byte_addr >> 2] = instruction;
        end
    endtask

    // -----------------------------------------------------------
    // Test Procedure
    // -----------------------------------------------------------
    initial begin
        // 1. Initialize Inputs
        rst = 1;

        // 2. Load Program into Memory
        #1; 
        
        // --- PROGAM ASSEMBLY ---
        // 1. addi x1, x0, 10      (x1 = 10)
        // 2. addi x2, x0, 5       (x2 = 5)
        // 3. mul  x3, x1, x2      (x3 = 10 * 5 = 50)
        
        // FIX: Immediate for -20 is 0xFEC (not 0xCEC)
        // 4. addi x4, x0, -20     (x4 = -20)  
        
        // 5. addi x5, x0, 4       (x5 = 4)
        // 6. div  x6, x4, x5      (x6 = -20 / 4 = -5)
        // 7. addi x7, x0, -23     (x7 = -23)
        // 8. rem  x8, x7, x5      (x8 = -23 % 4 = -3)
        // 9. ecall                (Halt)

        // Machine Code (Hex):
        write_inst(32'd0,  32'h00a00093); // addi x1, x0, 10
        write_inst(32'd4,  32'h00500113); // addi x2, x0, 5
        write_inst(32'd8,  32'h022081b3); // mul  x3, x1, x2
        
        // --- FIXED INSTRUCTION HERE ---
        write_inst(32'd12, 32'hfec00213); // addi x4, x0, -20 (0xFEC = -20)
        // ------------------------------

        write_inst(32'd16, 32'h00400293); // addi x5, x0, 4
        write_inst(32'd20, 32'h02524333); // div  x6, x4, x5
        write_inst(32'd24, 32'hFE900393); // addi x7, x0, -23
        write_inst(32'd28, 32'h0253e433); // rem  x8, x7, x5
        write_inst(32'd32, 32'h00000073); // ecall

        // 3. Release Reset
        #20;
        rst = 0;
        $display("--- Simulation Start ---");

        // 4. Wait for Halt
        wait(halt);
        
        // 5. Verify Results
        #10; 
        $display("--- Simulation Halted. Checking Results ---");

        // Check MUL Result
        if (uut.datapath.rf.regs[3] === 32'd50) 
            $display("[PASS] MUL: x3 = %d", $signed(uut.datapath.rf.regs[3]));
        else 
            $display("[FAIL] MUL: x3 = %d (Expected 50)", $signed(uut.datapath.rf.regs[3]));

        // Check DIV Result
        if ($signed(uut.datapath.rf.regs[6]) === -5) 
            $display("[PASS] DIV: x6 = %d", $signed(uut.datapath.rf.regs[6]));
        else 
            $display("[FAIL] DIV: x6 = %d (Expected -5)", $signed(uut.datapath.rf.regs[6]));

        // Check REM Result
        if ($signed(uut.datapath.rf.regs[8]) === -3) 
            $display("[PASS] REM: x8 = %d", $signed(uut.datapath.rf.regs[8]));
        else 
            $display("[FAIL] REM: x8 = %d (Expected -3)", $signed(uut.datapath.rf.regs[8]));

        $finish;
    end
endmodule
