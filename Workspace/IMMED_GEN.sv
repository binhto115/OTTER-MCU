`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly
// Engineer: Binh To
// 
// Create Date: 10/19/2023 11:42:11 PM
// Module Name: IMMED_GEN
// Project Name: RISV-v Otter MCU
// Target Devices: Basys3
// Description: A module that routes give types of instruction to their designated destination. 
//////////////////////////////////////////////////////////////////////////////////


module IMMED_GEN(
    input logic [31:0] IG_instruction,
    output logic [31:0] IG_utype,
    output logic [31:0] IG_itype,
    output logic [31:0] IG_stype,
    output logic [31:0] IG_btype,
    output logic [31:0] IG_jtype
    );
    
    assign IG_utype = {IG_instruction[31:12], {12'b0}};
    assign IG_itype = {{21{IG_instruction[31]}}, IG_instruction[30:20]};
    assign IG_stype = {{21{IG_instruction[31]}}, IG_instruction[30:25], IG_instruction[11:7]};
    assign IG_btype = {{20{IG_instruction[31]}}, IG_instruction[7], IG_instruction[30:25], IG_instruction[11:8], 1'b0};
    assign IG_jtype = {{12{IG_instruction[31]}}, IG_instruction[19:12], IG_instruction[20], IG_instruction[30:21], 1'b0};

endmodule

