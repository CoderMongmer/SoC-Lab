`timescale 1ns / 1ns

module ALUv_tb;

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

    // Clock Generation
    // clock_proc period = 10ns
    initial begin
        clock_proc = 0;
        forever #5 clock_proc = ~clock_proc;
    end

    // clock_mem generation (Phase shifted 90 degrees as per your diagram)
    // If proc rises at 5, mem should rise at 7.5 (or similar delay)
    // Here we define it to lag clock_proc by 2.5ns
    initial begin
        clock_mem = 0;
        #2.5; // Initial phase shift
        forever #5 clock_mem = ~clock_mem;
    end

    initial begin
        // Initialize Inputs
        rst = 1;

        // Wait 100 ns for global reset to finish
        #20;
        rst = 0;
        
        $display("--- Starting Simulation ---");

        // Wait for the halt signal
        wait(halt);
        
        // Give it one extra cycle to let writes settle
        @(posedge clock_proc);
        #1;

        $display("--- Execution Halted ---");
        
        // Verify Results by peeking into RegFile
        // Note: Hierarchy path depends on your module instance names.
        // Based on your code: uut -> datapath -> rf -> regs
        
        // Check ADD (x10 should be 13)
        if (uut.datapath.rf.regs[10] === 32'd13) 
            $display("PASS: ADD (10 + 3 = 13)");
        else 
            $display("FAIL: ADD. Expected 13, Got %d", uut.datapath.rf.regs[10]);

        // Check SUB (x11 should be 7)
        if (uut.datapath.rf.regs[11] === 32'd7) 
            $display("PASS: SUB (10 - 3 = 7)");
        else 
            $display("FAIL: SUB. Expected 7, Got %d", uut.datapath.rf.regs[11]);

        // Check SLL (x12 should be 80)
        if (uut.datapath.rf.regs[12] === 32'd80) 
            $display("PASS: SLL (10 << 3 = 80)");
        else 
            $display("FAIL: SLL. Expected 80, Got %d", uut.datapath.rf.regs[12]);

        // Check SLT (x13 should be 1) because -5 < 10
        if (uut.datapath.rf.regs[13] === 32'd1) 
            $display("PASS: SLT (-5 < 10 = True)");
        else 
            $display("FAIL: SLT. Expected 1, Got %d", uut.datapath.rf.regs[13]);

        // Check SRA (x14 should be -1 / 0xFFFFFFFF) because -5 >>> 3
        if (uut.datapath.rf.regs[14] === 32'hFFFFFFFF) 
            $display("PASS: SRA (-5 >>> 3 = -1)");
        else 
            $display("FAIL: SRA. Expected -1 (All Fs), Got %h", uut.datapath.rf.regs[14]);

        // Check OR (x15 should be 255)
        if (uut.datapath.rf.regs[15] === 32'd255) 
            $display("PASS: OR (0x0F | 0xF0 = 0xFF)");
        else 
            $display("FAIL: OR. Expected 255, Got %d", uut.datapath.rf.regs[15]);

        // Check AND (x16 should be 0)
        if (uut.datapath.rf.regs[16] === 32'd0) 
            $display("PASS: AND (0x0F & 0xF0 = 0x00)");
        else 
            $display("FAIL: AND. Expected 0, Got %d", uut.datapath.rf.regs[16]);

        $finish;
    end
      
endmodule