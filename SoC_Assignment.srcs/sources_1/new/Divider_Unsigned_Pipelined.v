`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/01/2025 05:29:43 PM
// Design Name: 
// Module Name: Divider_Unsigned_Pipelined
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


module Divider_Unsigned_Pipelined (
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    output wire [31:0] o_quotient,
    output wire [31:0] o_remainder
);

    // ============================================================
    // Internal Registers (The Pipeline State)
    // We need 9 "boundaries" for 8 stages (Input -> 1 -> 2 ... -> 8 -> Output)
    // ============================================================
    
    // 1. Remainder: Holds the current remainder calculation
    reg [31:0] pipe_rem      [0:8];
    
    // 2. Quotient: Accumulates the result bits
    reg [31:0] pipe_quot     [0:8];
    
    // 3. Dividend: Holds the bits we haven't processed yet
    reg [31:0] pipe_dividend [0:8];
    
    // 4. Divisor: Must travel down the pipe to match the data at that stage
    reg [31:0] pipe_divisor  [0:8];

    // ============================================================
    // Stage 0: Input Logic (Loading the Pipeline)
    // ============================================================
    always @(*) begin
        // At the very start (Index 0), we just wire the inputs.
        // We initialize remainder to 0.
        pipe_rem[0]      = 32'b0;
        pipe_quot[0]     = 32'b0;
        pipe_dividend[0] = i_dividend;
        pipe_divisor[0]  = i_divisor;
    end

    // ============================================================
    // Pipeline Stages Generation
    // ============================================================
    genvar i; // Stage index (0 to 7)
    genvar j; // Iteration index inside a stage (0 to 3)

    generate
        for (i = 0; i < 8; i = i + 1) begin : stage
            
            // Temporary wires to connect the 4 iterations *within* this stage
            // We need 5 connection points for 4 blocks (0=Input, 4=Output)
            wire [31:0] temp_rem  [0:4];
            wire [31:0] temp_quot [0:4];
            
            // 1. Setup Input to this chain of 4
            assign temp_rem[0]  = pipe_rem[i];
            assign temp_quot[0] = pipe_quot[i];

            // 2. Instantiate 4 Iterations logic
            for (j = 0; j < 4; j = j + 1) begin : iter
                wire quot_bit;
                wire [31:0] next_rem_val;
                
                // The bit we are processing is the MSB of the current dividend.
                // Since we shift pipe_dividend left by 4 every stage, 
                // the bits we need are at [31], [30], [29], [28].
                // But wait! We can just act like we are processing the top bit,
                // then shift the dividend variable for the next sub-step?
                // Actually, let's keep it simple: 
                // In this loop `j`, we grab bit `31-j` of the dividend registered for this stage.
                
                divu_1iter u_logic (
                    .i_rem          (temp_rem[j]),
                    .i_dividend_bit (pipe_dividend[i][31 - j]), 
                    .i_divisor      (pipe_divisor[i]), // Use divisor stored for this stage
                    .o_rem          (next_rem_val),
                    .o_quot_bit     (quot_bit)
                );

                assign temp_rem[j+1] = next_rem_val;
                // Shift quotient left and add the new bit
                assign temp_quot[j+1] = {temp_quot[j][30:0], quot_bit}; 
            end

            // 3. Register Logic (The Flip-Flops)
            always @(posedge clk) begin
                if (rst) begin
                    pipe_rem[i+1]      <= 0;
                    pipe_quot[i+1]     <= 0;
                    pipe_dividend[i+1] <= 0;
                    pipe_divisor[i+1]  <= 0;
                end else begin
                    // Latch the results from the combinational chain
                    pipe_rem[i+1]      <= temp_rem[4];
                    pipe_quot[i+1]     <= temp_quot[4];
                    
                    // Shift the dividend left by 4 bits so the next stage 
                    // sees the "new" MSBs at [31:28]
                    pipe_dividend[i+1] <= pipe_dividend[i] << 4;
                    
                    // Pass the divisor down unchanged
                    pipe_divisor[i+1]  <= pipe_divisor[i];
                end
            end
        end
    endgenerate

    // ============================================================
    // Output Logic
    // ============================================================
    // The result appears at the end of the pipeline (Index 8)
    assign o_remainder = pipe_rem[8];
    assign o_quotient  = pipe_quot[8];

endmodule
