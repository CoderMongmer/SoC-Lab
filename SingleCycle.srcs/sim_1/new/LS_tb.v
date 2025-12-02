`timescale 1ns / 1ns

module LS_tb;

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

    // 1. Processor Clock (Period 10ns)
    initial begin
        clock_proc = 0;
        forever #5 clock_proc = ~clock_proc;
    end

    // 2. Memory Clock (Phase Shifted ~90 deg)
    initial begin
        clock_mem = 0;
        #2.5; 
        forever #5 clock_mem = ~clock_mem;
    end

    initial begin
        // Initialize Inputs
        rst = 1;

        // Reset Pulse
        #20;
        rst = 0;
        
        $display("--- Starting Simple Load Test ---");

        // Wait for processor to finish
        wait(halt);
        
        // Stabilize
        @(posedge clock_proc);
        #1;

        $display("--- Execution Halted ---");
        
        // --- VERIFICATION ---

        // 1. Check LW (Load Word)
        // Should be the exact value we wrote: 0x1234ABCD
        if (uut.datapath.rf.regs[10] === 32'h1234ABCD) 
            $display("PASS: LW  (Got 0x1234ABCD)");
        else 
            $display("FAIL: LW. Expected 0x1234ABCD, Got %h", uut.datapath.rf.regs[10]);

        // 2. Check LB (Load Byte Signed)
        // Byte is 0xCD. MSB is 1. Should be 0xFFFFFFCD.
        if (uut.datapath.rf.regs[11] === 32'hFFFFFFCD) 
            $display("PASS: LB  (Got 0xFFFFFFCD - Sign Ext correctly)");
        else 
            $display("FAIL: LB. Expected 0xFFFFFFCD, Got %h", uut.datapath.rf.regs[11]);

        // 3. Check LBU (Load Byte Unsigned)
        // Byte is 0xCD. Should be 0x000000CD.
        if (uut.datapath.rf.regs[12] === 32'h000000CD) 
            $display("PASS: LBU (Got 0x000000CD - Zero Ext correctly)");
        else 
            $display("FAIL: LBU. Expected 0x000000CD, Got %h", uut.datapath.rf.regs[12]);

        // 4. Check LH (Load Halfword Signed)
        // Half is 0xABCD. MSB is 1 (A=1010). Should be 0xFFFFABCD.
        if (uut.datapath.rf.regs[13] === 32'hFFFFABCD) 
            $display("PASS: LH  (Got 0xFFFFABCD - Sign Ext correctly)");
        else 
            $display("FAIL: LH. Expected 0xFFFFABCD, Got %h", uut.datapath.rf.regs[13]);

        // 5. Check LHU (Load Halfword Unsigned)
        // Half is 0xABCD. Should be 0x0000ABCD.
        if (uut.datapath.rf.regs[14] === 32'h0000ABCD) 
            $display("PASS: LHU (Got 0x0000ABCD - Zero Ext correctly)");
        else 
            $display("FAIL: LHU. Expected 0x0000ABCD, Got %h", uut.datapath.rf.regs[14]);
            
        if (uut.datapath.rf.regs[10] === 32'hFFFFFFD8) 
        $display("PASS: MUL (10 * -4 = -40)");
        else 
            $display("FAIL: MUL. Expected 0xFFFFFFD8, Got %h", uut.datapath.rf.regs[10]);
    
        // MULH: 0x40000000
        if (uut.datapath.rf.regs[11] === 32'h40000000) 
            $display("PASS: MULH (MinInt^2 Upper)");
        else 
            $display("FAIL: MULH. Expected 0x40000000, Got %h", uut.datapath.rf.regs[11]);
    
        // MULHU: 0xFFFFFFFE
        if (uut.datapath.rf.regs[12] === 32'hFFFFFFFE) 
            $display("PASS: MULHU (MaxUint^2 Upper)");
        else 
            $display("FAIL: MULHU. Expected 0xFFFFFFFE, Got %h", uut.datapath.rf.regs[12]);
    
        // MULHSU: 0xFFFFFFFF
        if (uut.datapath.rf.regs[13] === 32'hFFFFFFFF) 
            $display("PASS: MULHSU (-1 * 2 Upper)");
        else 
            $display("FAIL: MULHSU. Expected 0xFFFFFFFF, Got %h", uut.datapath.rf.regs[13]);

        $finish;
    end
      
endmodule