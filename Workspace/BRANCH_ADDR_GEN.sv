`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly
// Engineer: Binh To
// 
// Create Date: 10/26/2023 11:11:04 PM
// Module Name: BRANCH_ADDR_GEN
// Project Name: RISC-V Otter MCU
// Target Devices: Basys3
// Description: A branch adder generator that adds two intended values to route the result to its designated place. 
//              This module takes in two inputs and add them under these conditions: 
//              If one value is from PC and the other is a J-type, the output will route to Jal.
//              If one value is from PC and the other is a B-type, the output will route to Branch.
//              If one value is from rs1 and the other is an I-type, the output will route to Jalr.
//////////////////////////////////////////////////////////////////////////////////


module BRANCH_ADDR_GEN(
    input logic [31:0] BAG_pc,
    input logic [31:0] BAG_jtype,
    input logic [31:0] BAG_btype,
    input logic [31:0] BAG_rs1,
    input logic [31:0] BAG_itype,
    output logic [31:0] BAG_jalr,
    output logic [31:0] BAG_branch,
    output logic [31:0] BAG_jal
    );
    
    assign BAG_branch = BAG_pc + BAG_btype;
    assign BAG_jal = BAG_pc + BAG_jtype;
    assign BAG_jalr = BAG_rs1 + BAG_itype;
    
endmodule
