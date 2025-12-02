`timescale 1ns / 1ns

module ALUv_tb; // Renamed to match your file if necessary

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
    initial begin
        clock_proc = 0;
        forever #5 clock_proc = ~clock_proc;
    end

    initial begin
        clock_mem = 0;
        #2.5; 
        forever #5 clock_mem = ~clock_mem;
    end

    initial begin
        // Initialize Inputs
        rst = 1;
        #20;
        rst = 0;
        
        $display("--- Starting Simulation ---");

        // Wait for the halt signal
        wait(halt);
        @(posedge clock_proc);
        #1;

        $display("--- Execution Halted ---");
        
        // --- VERIFICATION ---

        // ADD
        if (uut.datapath.rf.regs[10] === 32'd13) $display("PASS: ADD");
        else $display("FAIL: ADD. Expected 13, Got %d", uut.datapath.rf.regs[10]);

        // SUB
        if (uut.datapath.rf.regs[11] === 32'd7) $display("PASS: SUB");
        else $display("FAIL: SUB. Expected 7, Got %d", uut.datapath.rf.regs[11]);

        // SLL
        if (uut.datapath.rf.regs[12] === 32'd80) $display("PASS: SLL");
        else $display("FAIL: SLL. Expected 80, Got %d", uut.datapath.rf.regs[12]);

        // SLT (Signed comparison)
        if (uut.datapath.rf.regs[13] === 32'd1) $display("PASS: SLT (-5 < 10 is True)");
        else $display("FAIL: SLT. Expected 1, Got %d", uut.datapath.rf.regs[13]);

        // SRA
        if (uut.datapath.rf.regs[14] === 32'hFFFFFFFF) $display("PASS: SRA");
        else $display("FAIL: SRA. Expected -1, Got %h", uut.datapath.rf.regs[14]);

        // OR
        if (uut.datapath.rf.regs[15] === 32'd255) $display("PASS: OR");
        else $display("FAIL: OR. Expected 255, Got %d", uut.datapath.rf.regs[15]);

        // AND
        if (uut.datapath.rf.regs[16] === 32'd0) $display("PASS: AND");
        else $display("FAIL: AND. Expected 0, Got %d", uut.datapath.rf.regs[16]);

        // NEW: SLTU (Unsigned comparison)
        // We expect 0 because 0xFFFFFFFB (Big Number) is NOT < 10
        if (uut.datapath.rf.regs[17] === 32'd0) 
            $display("PASS: SLTU (Unsigned -5 < 10 is False)");
        else 
            $display("FAIL: SLTU. Expected 0 (False), Got %d", uut.datapath.rf.regs[17]);

        $finish;
    end
      
endmodule