`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly
// Engineer: Binh To
// 
// Create Date: 12/01/2023 12:05:21 PM
// Design Name: CSR
// Module Name: OTTER_CSR
// Project Name: OTTER_WRAPPER
// Target Devices: Basys3
// Description: Program to handle interrupts
// 
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module OTTER_CSR(
    input logic CSR_reset,
    input logic CSR_mret_exec,
    input logic int_taken,
    input logic [11:0] CSR_ir,
    input logic CSR_csr_WE,
    input logic [31:0] CSR_pc,
    input logic [31:0] CSR_WD,
    input logic CSR_clk,
    output logic CSR_mstatus,
    output logic [31:0] CSR_mepc,
    output logic [31:0] CSR_mtvec,
    output logic [31:0] CSR_RD
    );
    
    logic [31:0] mstatus;
  
    always_ff @(posedge CSR_clk) begin
        if (CSR_reset) begin // Set outputs to 0 if reset
            CSR_mepc <= 32'b0;
            CSR_mtvec <= 32'b0;
            mstatus <=  32'b0;
        end
        
        // Enabling the interrupt
        else if (int_taken) begin
            mstatus[7] <= mstatus[3];
            CSR_mepc <= CSR_pc; //Save pc to mepc
            mstatus[3] <= 1'b0;
       end 
       
        else if (CSR_csr_WE) begin
            case (CSR_ir)
                12'h300: mstatus <= CSR_WD;
                12'h305: CSR_mtvec <= CSR_WD;
                12'h341: CSR_mepc <= CSR_WD;
            endcase
        end
        
            
        // When the execution of the ISR completes with MRET instruction, it then: 
        else if (CSR_mret_exec) begin
            mstatus[3] <= mstatus[7];
            mstatus[7] <= 1'b0;
        end
    end
       
        assign CSR_mstatus = mstatus[3]; // Assigning MIE
        
    // Read from registers
    always_comb begin
        case (CSR_ir)
            12'h300: CSR_RD = mstatus;
            12'h305: CSR_RD = CSR_mtvec;
            12'h341: CSR_RD = CSR_mepc;
            default: CSR_RD = 32'b0;
        endcase 
    end   
endmodule
