`timescale 1ns/1ps

module divu_1iter (
    input  wire [31:0] i_rem,          // Remainder from previous stage
    input  wire        i_dividend_bit, // The specific bit from the dividend (31 down to 0)
    input  wire [31:0] i_divisor,      // The divisor (constant across all stages)
    output wire [31:0] o_rem,          // Remainder passing to next stage
    output wire        o_quot_bit      // The bit calculated for the quotient at this stage
);

    wire [31:0] shifted_rem;
    wire [31:0] difference;

    // 1. Shift remainder left by 1 and tack on the dividend bit
    // We discard the MSB of i_rem because it's shifted out.
    assign shifted_rem = {i_rem[30:0], i_dividend_bit};

    // 2. Calculate the difference (Check if we can subtract)
    assign difference = shifted_rem - i_divisor;

    // 3. Determine the logic
    // If shifted_rem >= i_divisor, we subtract and the quotient bit is 1.
    // If shifted_rem < i_divisor, we keep the shifted_rem and quotient bit is 0.
    
    // Note: In hardware, checking ">= divisor" is the same as checking if the 
    // subtraction does NOT underflow (or simply checking the comparison).
    assign o_quot_bit = (shifted_rem >= i_divisor);

    // MUX to choose the next remainder
    assign o_rem = (o_quot_bit) ? difference : shifted_rem;

endmodule
