`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly
// Engineer: Binh To
// 
// Create Date: 10/04/2023 01:33:02 AM
// Design Name: Program Counter
// Module Name: Program_Counter
// Project Name: RISC-V Otter MCU
// Target Devices: Basys3
// Description: A program that executes instructions 
// stored in memory to produce a result
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module Program_Counter (    
    input logic PC_WRITE,
    input logic  PC_RST,
    input logic [31:0] PC_DIN,
    input logic PC_CLK,
    output logic [31:0] PC_COUNT
    );
    
    always_ff @ (posedge PC_CLK) 
    
    begin
    if (PC_RST == 1) 
        PC_COUNT <= 32'd0;        
    else if (PC_WRITE == 1)
        PC_COUNT <= PC_DIN;
    else
        PC_COUNT <= PC_COUNT;
    end        
    
endmodule
