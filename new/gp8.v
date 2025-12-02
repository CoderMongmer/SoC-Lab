`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/29/2025 07:18:57 AM
// Design Name: 
// Module Name: gp8
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


module gp8(
    input [7:0] gin, pin,
    input cin,
    output gout, pout,
    output [6:0] cout);
    
    wire gout_low, pout_low;
    wire [2:0] cout_low; // carry C1, C2, C3
    wire gout_high, pout_high;
    wire [2:0] cout_high; // carry C5, C6, C7

    wire c4;
    
    gp4 low_4b (
        .gin(gin[3:0]), 
        .pin(pin[3:0]), 
        .cin(cin),
        .gout(gout_low), 
        .pout(pout_low),
        .cout(cout_low)     
    ); 
    
    assign c4 = gout_low | (pout_low & cin);
    
    gp4 high_4b (
        .gin(gin[7:4]),
        .pin(pin[7:4]),
        .cin(c4),          
        .gout(gout_high),
        .pout(pout_high),
        .cout(cout_high)    
    );
    
    assign cout[2:0] = cout_low;  
    assign cout[3]   = c4;        
    assign cout[6:4] = cout_high; 
   
    assign pout = pout_high & pout_low;
    assign gout = gout_high | (pout_high & gout_low);
     
endmodule
