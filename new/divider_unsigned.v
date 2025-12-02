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
    input iClk,             // Not used in combinational version
    input iRst,             // Not used in combinational version
    input [31:0] dividend,
    input [31:0] divisor,
    output [31:0] quotient,  // Removed 'reg', now wire
    output [31:0] remainder  // Removed 'reg', now wire
);

    integer i;
    reg [31:0] rem;
    reg [31:0] div;
    reg [31:0] q;

    // Combinational calculation
    always @(*) begin
        rem = 0;
        div = dividend;
        q = 0;

        for (i = 0; i < 32; i = i + 1) begin
            rem = (rem << 1) | ((div >> 31) & 32'h1);  

            if (rem < divisor) begin
                q = (q << 1);
            end else begin
                rem = rem - divisor;
                q = (q << 1) | 1'b1;
            end
            div = div << 1;
        end
    end
    
    // DIRECT OUTPUT ASSIGNMENT (Combinational)
    assign quotient = q;
    assign remainder = rem;
    
    // Removed the sequential always @(posedge iClk) block
    
endmodule

