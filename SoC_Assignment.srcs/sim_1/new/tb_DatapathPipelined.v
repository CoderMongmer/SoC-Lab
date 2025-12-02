`timescale 1ns / 1ps

module tb_DatapathPipelined;

    reg clk;
    reg rst;
    wire halt;
    wire [31:0] trace_pc;
    wire [31:0] trace_inst;

    // Instantiate the Processor (which contains your Datapath and Memory)
    Processor uut (
        .clk(clk),
        .rst(rst),
        .halt(halt),
        .trace_writeback_pc(trace_pc),
        .trace_writeback_inst(trace_inst)
    );

    // Clock Generation (10ns period)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Test Sequence
    initial begin
        // 1. Initialize
        rst = 1;
        #20;
        
        // 2. Release Reset
        rst = 0;
        
        // 3. Run Simulation
        // We run for enough cycles to let the divider finish (approx 50 cycles is plenty for this small test)
        #500;
        
        // 4. Force Stop if it runs too long
        $display("Simulation finished by timeout.");
        $finish;
    end

    // Monitor Output
    // This prints the PC and Instruction every time a valid instruction completes Writeback
    always @(posedge clk) begin
        if (!rst && (trace_pc != 0)) begin
            $display("Time: %t | Writeback PC: %h | Inst: %h", $time, trace_pc, trace_inst);
            if (halt) begin
                $display("HALT detected.");
                $finish;
            end
        end
    end

endmodule