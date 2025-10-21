`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly    
// Engineer: Binh To
// 
// Create Date: 10/19/2023 09:58:33 PM
// Design Name: Arithmetic Logic Unit
// Module Name: ALU_MODULE
// Project Name: RISC-V Otter MCU
// Target Devices: Basys3
// Tool Versions: 
// Description: A module that performs math operations such as addition, subtraction, shifting, inverting, etc. 
// It can perform up to 11 operations. If any other instruction were to be used, it would not work and output to 0 as a result.
// It takes two inputs and perform the operation based on whatever the ALU_FUN input signal control is. 
//////////////////////////////////////////////////////////////////////////////////


module ALU_MODULE(
    input logic [31:0] ALU_scra,
    input logic [31:0] ALU_scrb,
    input logic [3:0] ALU_fun,
    output logic [31:0] ALU_result
    );  
    
    // Cases for 11 OTTER operations
    always_comb begin
        case (ALU_fun)
            4'b0000 : ALU_result = ALU_scra + ALU_scrb;    // add
            4'b1000 : ALU_result = $signed(ALU_scra) - $signed(ALU_scrb);    // sub
            4'b0110 : ALU_result = ALU_scra | ALU_scrb;    // or
            4'b0111 : ALU_result = ALU_scra & ALU_scrb;    // and
            4'b0100 : ALU_result = ALU_scra ^ ALU_scrb;    // xor
            4'b0101 : ALU_result = ALU_scra >> ALU_scrb[4:0];   // srl
            4'b0001 : ALU_result = ALU_scra << ALU_scrb[4:0];   // sll
            4'b1101 : ALU_result = $signed(ALU_scra) >>> ALU_scrb[4:0];    // sra
            4'b0010 : ALU_result = $signed(ALU_scra) < $signed(ALU_scrb) ? 32'b1 : 32'b0;    // slt
            4'b0011 : ALU_result = ALU_scra < ALU_scrb ? 32'b1 : 32'b0;      // sltu 
            4'b1001 : ALU_result = ALU_scra; // lui
            default : ALU_result = 32'b0; // Default to 0 for any other invalid/non-related AluFun
        endcase
    end
    
    
    
endmodule
