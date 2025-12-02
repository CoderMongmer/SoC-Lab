`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/23/2025 11:06:32 PM
// Design Name: 
// Module Name: divider_unsigned
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



module divider_unsigned (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    output wire [31:0] o_quotient,
    output wire [31:0] o_remainder
);

    // We need wires to hold the remainder between stages.
    // We need 33 wires: 1 for the initial input (0), and 32 for the outputs of the stages.
    wire [31:0] remainder_chain [32:0];

    // The first remainder input is always 0 (as per the C code: int remainder = 0)
    assign remainder_chain[0] = 32'b0;

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : div_stage
            divu_1iter u_div_stage (
                .i_rem         (remainder_chain[i]),      // Input from previous stage
                .i_dividend_bit(i_dividend[31 - i]),      // Take MSB first (31 down to 0)
                .i_divisor     (i_divisor),               // Divisor is same for all
                .o_rem         (remainder_chain[i+1]),    // Output to next stage
                .o_quot_bit    (o_quotient[31 - i])       // Fill quotient from MSB to LSB
            );
        end
    endgenerate

    // The final remainder is the output of the last stage
    assign o_remainder = remainder_chain[32];

endmodule