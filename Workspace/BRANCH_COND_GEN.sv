`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly
// Engineer: Binh To
// 
// Create Date: 10/26/2023 11:11:04 PM
// Module Name: BRANCH_COND_GEN
// Project Name: RISC-v OTTER MCU
// Target Devices: Basys3
// Description: This is a branch conditional generator. It takes two inputs in 
//              and compares them through three instructions (technically six but we only represent six because 
//              if one is true, then the other one must be false as they are mutually exclusive). 
//              Specifically, beq vs. bne, bge vs blt, and bgeu vs bltu. The first pair deals with equality. It checks if the two   
//              values are equal or not. The second pair checks if the first value is 
//              less than the second value or not when treated as signed numbers. The third and last pair also checks if the first 
//              value is less than the second value or not but they are treated as unsigned values.
//////////////////////////////////////////////////////////////////////////////////

module BRANCH_COND_GEN(
    input logic [31:0] BCG_condRs1,
    input logic [31:0] BCG_condRs2,
    output logic BCG_br_eq,
    output logic BCG_br_lt,
    output logic BCG_br_ltu
    );
    assign BCG_br_eq = (BCG_condRs1 == BCG_condRs2);
    assign BCG_br_lt = ($signed(BCG_condRs1) < $signed(BCG_condRs2)); 
    assign BCG_br_ltu = ($unsigned(BCG_condRs1) < $unsigned(BCG_condRs2));
//    always @(*) begin

//    if (BCG_condRs1 == BCG_condRs2) begin
//        BCG_br_eq = 1'b1;
//    end else begin
//        BCG_br_eq = 1'b0;
//    end
    
//    if ($signed(BCG_condRs1) < $signed(BCG_condRs2)) begin
//        BCG_br_lt = 1;
//    end else begin
//        BCG_br_lt = 0;
//    end
    
//    if ($unsigned(BCG_condRs1) < $unsigned(BCG_condRs2)) begin
//        BCG_br_ltu = 1;
//    end else begin
//        BCG_br_ltu = 0;
//    end
    
//    end
endmodule 