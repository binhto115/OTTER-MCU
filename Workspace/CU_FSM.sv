`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Ca Poly
// Engineer: Binh To
// 
// Create Date: 11/05/2023 01:19:41 AM
// Design Name: Control Unit Module
// Module Name: CU_FSM
// Project Name: RISC-V OTTER
// Target Devices: basys3
// Description: This module is responsible for all control signals. The module will fetch must instructions at
//              FETCH state, except for loading instructions. The EXEC state will set all of the 
//              intended control signals to read from memory, and the data will output on the next
//              rising edge of clock. The WRITE BACK state will set the register to save data and change
//              the program counter.
//////////////////////////////////////////////////////////////////////////////////


module CU_FSM(
    input logic FSM_RST,
    input logic FSM_INTR,
    input logic [6:0] FSM_opcode,
    input logic [2:0] FSM_funct3,
    input logic FSM_clk,
    output logic FSM_pcWrite,
    output logic FSM_regWrite,
    output logic FSM_memWE2,
    output logic FSM_memRDEN1,
    output logic FSM_memRDEN2,
    output logic FSM_reset,
    output logic csr_WE,
    output logic int_taken,
    output logic mret_exec    
    );
    
    
    typedef enum {INIT_ST, FETCH_ST, EXEC_ST, WRITE_BACK_ST, INTR_ST} STATE_TYPES;
    STATE_TYPES NS, PS; // Define signals for state registers
    
    // State Register
    always_ff @(posedge FSM_clk) begin
        if (FSM_RST == 1'b1)
            PS <= INIT_ST;
        else 
            PS <= NS;
        end    
    
    // Combinational Logic
    // Input/output logic
    always_comb begin
        // Set every signal to 0's 
        FSM_pcWrite = 1'b0;
        FSM_regWrite = 1'b0;
        FSM_memWE2 = 1'b0;
        FSM_memRDEN1 = 1'b0;
        FSM_memRDEN2 = 1'b0;
        FSM_reset = 1'b0;
        csr_WE = 1'b0;
        int_taken = 1'b0;
        mret_exec = 1'b0;
        
        // Cases for states
        case (PS)
            // Initial State
            INIT_ST : begin
                NS = FETCH_ST;
                FSM_reset = 1'b1;
            end
            // Fetch State
            FETCH_ST : begin
                FSM_memRDEN1 = 1'b1;
                NS = EXEC_ST;
            end
            // Execution State
            EXEC_ST: begin
                case (FSM_opcode)
                    7'b1110011: // csr instructions
                        begin
                        case(FSM_funct3)
                            3'b000: begin //CSR-type mret
                                FSM_pcWrite = 1'b1;
                                mret_exec = 1'b1;
                                if (FSM_INTR)
                                    NS = INTR_ST;
                                else
                                    NS = FETCH_ST;
                            end
                            
                            3'b011: begin //csrrc
                                FSM_regWrite = 1'b1;
                                csr_WE = 1'b1;
                                FSM_pcWrite = 1'b1;
                                if (FSM_INTR)
                                    NS = INTR_ST;
                                else
                                    NS = FETCH_ST;
                            end
                            3'b010: begin //csrrs
                                FSM_regWrite = 1'b1;
                                FSM_pcWrite = 1'b1;
                                csr_WE = 1'b1;
                                if (FSM_INTR)
                                    NS = INTR_ST;
                                else
                                    NS = FETCH_ST;
                            end
                            3'b001: begin //csrrw
                                FSM_regWrite = 1'b1;
                                FSM_pcWrite = 1'b1;
                                csr_WE = 1'b1;
                                if (FSM_INTR)
                                    NS = INTR_ST;
                                else
                                    NS = FETCH_ST;
                            end
                            default: begin
                            NS = INIT_ST;
                            end
                        endcase
                    end
                    7'b0110011: begin // R-type instruction (add, and or, sll, slt, sltu, sra, srl, sub, xor)
                        FSM_pcWrite = 1'b1;
                        FSM_regWrite = 1'b1;
                        if (FSM_INTR) 
                            NS = INTR_ST;
                        else
                            NS = FETCH_ST;                        
                    end
                    7'b0010011: begin // I-type (addi, ori, slli, slti, sltiu, srai, srli, xori)
                        FSM_pcWrite = 1'b1;
                        FSM_regWrite = 1'b1;
                        if (FSM_INTR) 
                            NS = INTR_ST;
                        else
                            NS = FETCH_ST;                        
                    end
                    7'b1100111: begin // I-type (jalr case)
                        FSM_pcWrite = 1'b1;
                        FSM_regWrite = 1'b1;
                        if (FSM_INTR) 
                            NS = INTR_ST;
                        else
                            NS = FETCH_ST;                    
                    end
                    7'b0000011: begin // I-type (lb, lbu, lh, lhu, lw)
                        FSM_pcWrite = 1'b0; // 0 bc you're loading, not writing to the next address
                        FSM_regWrite = 1'b0;
                        FSM_memRDEN2 = 1'b1;
                        if (FSM_INTR) 
                            NS = INTR_ST;
                        else
                            NS = FETCH_ST;                        
                    end
                    7'b0100011: begin // S-type (sb, sh, sw)
                        FSM_pcWrite = 1'b1;
                        FSM_memWE2 = 1'b1; // Since we're storing, its write enable. Cannot read and write at the same time
                        if (FSM_INTR) 
                            NS = INTR_ST;
                        else
                            NS = FETCH_ST;                    
                    end
                    7'b1100011: begin // B-type (beq, bne, bge, bgeu, blt, bltu)
                        FSM_pcWrite = 1'b1;
                        if (FSM_INTR) 
                            NS = INTR_ST;
                        else
                            NS = FETCH_ST;                        
                    end
                    7'b0110111: begin // U-type (lui)
                        FSM_pcWrite = 1'b1;
                        FSM_regWrite = 1'b1;
                        if (FSM_INTR) 
                            NS = INTR_ST;
                        else
                            NS = FETCH_ST;                        
                    end
                    7'b0010111: begin // U-type (auipc)
                        FSM_pcWrite = 1'b1;
                        FSM_regWrite = 1'b1;
                        if (FSM_INTR) 
                            NS = INTR_ST;
                        else
                            NS = FETCH_ST;
                    end
                    7'b1101111: begin // J-type (jal)
                        FSM_pcWrite = 1'b1;
                        FSM_regWrite = 1'b1;
                        if (FSM_INTR) 
                            NS = INTR_ST;
                        else
                            NS = FETCH_ST;                    
                    end
                    default: begin
                        NS = INIT_ST;
                        FSM_reset = 1'b1; // reset if the program somehow gets to this stage.
                        
                    end
                endcase
                if (FSM_opcode == 7'b0000011)
                    NS = WRITE_BACK_ST;
//                else begin
//                    if (FSM_INTR)
//                        NS = INTR_ST;
//                    else
//                        NS = FETCH_ST;
//                    end
            end
            
            // Write Back State
            WRITE_BACK_ST: begin
                FSM_pcWrite = 1'b1;
                FSM_regWrite = 1'b1;
                if (FSM_INTR)
                    NS = INTR_ST;
                else
                    NS = FETCH_ST;
            end
            
            INTR_ST: begin
                int_taken = 1'b1;
                FSM_pcWrite = 1'b1;
                FSM_regWrite = 1'b0;
                csr_WE = 1'b0;              
                NS = FETCH_ST;
            end
            default: begin // should not occur
                NS = INIT_ST;
            end
        endcase
    end             
endmodule
