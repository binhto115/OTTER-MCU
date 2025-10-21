`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly
// Engineer: Binh To
// 
// Create Date: 11/08/2023 03:10:16 AM
// Design Name: RISC-V Otter MCU
// Module Name: MCU
// Project Name: RISC-V Otter MCU
// Target Devices: Basys3
// Description: The micro-controller consisted  of Program Counter, Memory, register File, ALU, 
//              Immediate Generator, Branch Condition Generator, and Control Unit as subcomponents.
//              Note: csr module is excluded as it iw out of scope within the bound of this assignemnt. 
//////////////////////////////////////////////////////////////////////////////////


module MCU(

    input logic MCU_RST,
    input logic MCU_INTR,
    input logic [31:0] MCU_IOBUS_IN,
    input logic MCU_clk,
    output logic MCU_IOBUS_WR,
    output logic [31:0] MCU_IOBUS_OUT,
    output logic [31:0] MCU_IOBUS_ADDR
    );
    

    // Instruction wire AKA Dout1
    logic [31:0] ir_wire;
    
    
    //FINITE STATE MACHINE
    ////////////////////////////////////////////////////////////////////////
    //Inputs: MCU_RST, MCU_INTR, ir_wire
    logic mstatus;
    // FSM outputs 
    logic fsm_pc_write;
    logic fsm_reg_write;
    logic fsm_memWE2;
    logic fsm_memRDEN1;
    logic fsm_memRDEN2;
    logic fsm_reset;
    logic fsm_csr_we;           
    logic fsm_int_taken;        
    logic fsm_mret_exec;        
    
    // Control Unit Finite State Machine
    CU_FSM OTTER_CU_FSM (
    .FSM_RST            (MCU_RST),
    .FSM_INTR           (MCU_INTR & mstatus),
    .FSM_opcode         (ir_wire[6:0]),
    .FSM_funct3         (ir_wire[14:12]),
    .FSM_clk            (MCU_clk),
    .FSM_pcWrite        (fsm_pc_write),
    .FSM_regWrite       (fsm_reg_write),
    .FSM_memWE2         (fsm_memWE2),
    .FSM_memRDEN1       (fsm_memRDEN1),
    .FSM_memRDEN2       (fsm_memRDEN2),
    .FSM_reset          (fsm_reset),
    .csr_WE             (fsm_csr_we),
    .int_taken          (fsm_int_taken),
    .mret_exec          (fsm_mret_exec)   );
    ////////////////////////////////////////////////////////////////////////    
    
    
     //CONTROL UNIT DECODER
    ////////////////////////////////////////////////////////////////////////
    //Inputs: ir_wire
    logic br_eq_wire;
    logic br_lt_wire;
    logic br_ltu_wire;
    // Outputs:
    logic [3:0] alu_fun_wire;
    logic [1:0] alu_srca_wire;
    logic [2:0] alu_srcb_wire;
    logic [2:0] pc_source_wire;
    logic [1:0] rf_wr_sel_wire;
    
    // FSM outputs    
    CU_DCDR OTTER_CU_DCDR (
    .DCDR_ir_opcode     (ir_wire[6:0]),
    .DCDR_ir_funct      (ir_wire[14:12]),
    .DCDR_ir_30         (ir_wire[30]),
    .DCDR_int_taken     (fsm_int_taken),
    .DCDR_br_eq         (br_eq_wire),
    .DCDR_br_lt         (br_lt_wire),
    .DCDR_br_ltu        (br_ltu_wire),
    
    .DCDR_alu_fun       (alu_fun_wire),
    .DCDR_alu_scra      (alu_srca_wire),
    .DCDR_alu_scrb      (alu_srcb_wire),
    .DCDR_pc_source     (pc_source_wire),
    .DCDR_rf_wr_sel     (rf_wr_sel_wire)  );
    //////////////////////////////////////////////////////////////////////// 
    
    
    // Program Counter Mux
    //////////////////////////////////////////////////////////////////////// 
    // Inputs: pc_source_wire
    logic [31:0] NextInstructionAddress; // goes to 0
    logic [31:0] BAG_to_jalr_wire; // goes to 1
    logic [31:0] BAG_to_branch_wire; // goes to 2
    logic [31:0] BAG_to_jal_wire; // goes to 3
    logic [31:0] mtvec_to_4_wire; // goes to 4  UNUSED
    logic [31:0] mepc_to_5_wire; // goes to 5   UNUSED
    logic [31:0] din_wire;
    
    always_comb begin
        case (pc_source_wire)
            3'b000: din_wire = NextInstructionAddress;
            3'b001: din_wire = BAG_to_jalr_wire;
            3'b010: din_wire = BAG_to_branch_wire;
            3'b011: din_wire = BAG_to_jal_wire;
            3'b100: din_wire = mtvec_to_4_wire; 
            3'b101: din_wire = mepc_to_5_wire;
            default: din_wire = 32'h0;
        endcase
    end
    ////////////////////////////////////////////////////////////////////////     
    
    
    // Program Counter
    //////////////////////////////////////////////////////////////////////// 
    // Inputs: fsm_reset, fsm_pc_write, din_wire, MCU_clk
    // Outputs:
    logic [31:0] pc_wire;
    
    // Program Counter
    Program_Counter OTTER_PC (
    .PC_WRITE   (fsm_pc_write),
    .PC_RST     (fsm_reset),
    .PC_DIN     (din_wire),
    .PC_CLK     (MCU_clk),
    .PC_COUNT   (pc_wire)  );      
    
    // Plus4Adder
    logic [31:0] InstructionAddress;
    assign InstructionAddress = pc_wire;
    assign NextInstructionAddress = InstructionAddress + 4; // Increment by 4 to PC MUX and RF MUX
    //////////////////////////////////////////////////////////////////////// 


    // Memory    
    //////////////////////////////////////////////////////////////////////// 
    // Inputs:
    logic [31:0] aluResult_wire;
    logic [31:0] Din2_memory_wire;
    logic [13:0] pc_to_addr1_wire;
    assign pc_to_addr1_wire = pc_wire[15:2];
    // fsm_memRDEN1, fsm_memRDEN2, fsm_memWE2, ir_wire[13:12], ir_wire[14], MCU_IOBUS_IN, MCU_clk
    // Outputs
    logic [31:0] Dout2_wire;
    logic [31:0] Dout1_instruction_wire; 
    // ir_wire
    // MCU_IOBUS_WR
    
    //Otter Memory 
     Memory OTTER_MEMORY (
    .MEM_CLK   (MCU_clk),
    .MEM_RDEN1 (fsm_memRDEN1),
    .MEM_RDEN2 (fsm_memRDEN2),
    .MEM_WE2   (fsm_memWE2),
    .MEM_ADDR1 (pc_to_addr1_wire),
    .MEM_ADDR2 (aluResult_wire),
    .MEM_DIN2  (Din2_memory_wire),
    .MEM_SIZE  (ir_wire[13:12]),
    .MEM_SIGN  (ir_wire[14]),
    .IO_IN     (MCU_IOBUS_IN),
    .IO_WR     (MCU_IOBUS_WR),
    .MEM_DOUT1 (Dout1_instruction_wire),
    .MEM_DOUT2 (Dout2_wire)  );    
    
    assign ir_wire = Dout1_instruction_wire;
    ////////////////////////////////////////////////////////////////////////   
    
    
    // Reg File Mux
    ////////////////////////////////////,//////////////////////////////////// 
    //Inputs: fsm_rf_wr_sel, NextInstructionAddress
    logic [31:0] csr_RD_wire;
    // Dout2_wire, aluResult_wire
    // Outputs: 
    logic [31:0] wd_wire;  
    
    always_comb begin
        case(rf_wr_sel_wire)
            2'b00: wd_wire = NextInstructionAddress;
            2'b01: wd_wire = csr_RD_wire;
            2'b10: wd_wire = Dout2_wire;
            2'b11: wd_wire = aluResult_wire;
        endcase
    end
    //////////////////////////////////////////////////////////////////////// 
    ////////
    
    
      // Reg File
    ////////////////////////////////////,//////////////////////////////////// 
    //Inputs: wd_wire, fsm_reg_write,
    logic [4:0] addr1_instruction_to_rf;
    assign addr1_instruction_to_rf = ir_wire[19:15];
    logic [4:0] addr2_instruction_to_rf;
    assign addr2_instruction_to_rf = ir_wire[24:20];
    logic [4:0] wa_instruction_to_rf;
    assign wa_instruction_to_rf = ir_wire[31:7];
    // MCU_clk
    // Outputs: 
    logic [31:0] rf_rs1_wire;
    logic [31:0] rf_rs2_wire;
    assign Din2_memory_wire = rf_rs2_wire;
    
    REG_FILE OTTER_REG_FILE (
    .RF_ADR1    (addr1_instruction_to_rf),
    .RF_ADR2    (addr2_instruction_to_rf),
    .RF_WA      (wa_instruction_to_rf),
    .RF_WD      (wd_wire),
    .RF_CLK     (MCU_clk),
    .RF_EN      (fsm_reg_write),
    .RF_RS1     (rf_rs1_wire),
    .RF_RS2     (rf_rs2_wire)  );    
    
    assign MCU_IOBUS_OUT = Din2_memory_wire;
    //////////////////////////////////////////////////////////////////////// 


    // CSR
    ////////////////////////////////////////////////////////////////////////
    // Inputs: reset, mret_exec, int_taken, ir[31:20], csr_WE, pc, aluResult_wire, MCU_clk
    //Outputs mstatus[3], mepc, mtvec, RD

    OTTER_CSR OTTER_csr(
    .CSR_reset      (MCU_RST),
    .CSR_mret_exec  (fsm_mret_exec),
    .int_taken      (fsm_int_taken),
    .CSR_ir         (ir_wire[31:20]),
    .CSR_csr_WE     (fsm_csr_we),
    .CSR_pc         (pc_wire),
    .CSR_WD         (aluResult_wire),
    .CSR_clk        (MCU_clk),
    .CSR_mstatus    (mstatus),
    .CSR_mepc       (mepc_to_5_wire),
    .CSR_mtvec      (mtvec_to_4_wire),
    .CSR_RD         (csr_RD_wire) ); 
    ////////////////////////////////////////////////////////////////////////
    
        
    // Immediate Generator
    //////////////////////////////////////////////////////////////////////// 
    // Inputs: ir_wire
    // outputs:
    logic [31:0] utype_wire;
    logic [31:0] itype_wire;
    logic [31:0] stype_wire;
    logic [31:0] btype_wire;
    logic [31:0] jtype_wire;    

    // Immediate Generator
    IMMED_GEN OTTER_IMMED_GEN (
    .IG_instruction     (ir_wire),
    .IG_utype           (utype_wire),
    .IG_itype           (itype_wire),
    .IG_stype           (stype_wire),
    .IG_btype           (btype_wire),
    .IG_jtype           (jtype_wire)  );
    //////////////////////////////////////////////////////////////////////// 
    
    
    // BRANCH_ADDR_GENERATOR
    //////////////////////////////////////////////////////////////////////// 
    // Inputs: rf_rs1_wire, itype_wire, btype_wire, jtype_wire, InstructionAddress,
    // Outputs: BAG_to_jalr_wire, BAG_to_branch_wire, BAG_to_jal_wire
    
    BRANCH_ADDR_GEN OTTER_BAG (
    .BAG_pc         (InstructionAddress),
    .BAG_jtype      (jtype_wire),
    .BAG_btype      (btype_wire),
    .BAG_rs1        (rf_rs1_wire),
    .BAG_itype      (itype_wire),
    .BAG_jalr       (BAG_to_jalr_wire),
    .BAG_branch     (BAG_to_branch_wire),
    .BAG_jal        (BAG_to_jal_wire)  );
    //////////////////////////////////////////////////////////////////////// 
    
    
    // Branch_Cond_Generator
    //////////////////////////////////////////////////////////////////////// 
    // Inputs: rf_rs1_wire, rf_rs2_wire
    // Outputs: br_eq_wire, br_lt_wire, br_ltu_wire;
    // Branch Condition Generator
    BRANCH_COND_GEN OTTER_BCG (
    .BCG_condRs1    (rf_rs1_wire),
    .BCG_condRs2    (rf_rs2_wire),
    .BCG_br_eq      (br_eq_wire),
    .BCG_br_lt      (br_lt_wire),
    .BCG_br_ltu     (br_ltu_wire)  );
    //////////////////////////////////////////////////////////////////////// 
    
    // ALU SRCA MUX
    //////////////////////////////////////////////////////////////////////// 
    // Inputs: alu_srca_wire, rf_rs1_wire, utype_wire, ~rf_rs1_wire
    // Outputs: 
    logic [31:0] aluMUXA_to_alu_wire;
    
    always_comb begin
        case (alu_srca_wire)
            2'b00: aluMUXA_to_alu_wire = rf_rs1_wire;
            2'b01: aluMUXA_to_alu_wire = utype_wire;
            2'b10: aluMUXA_to_alu_wire = ~(rf_rs1_wire);
            default: aluMUXA_to_alu_wire = 32'b0;
        endcase
    end  
    ////////////////////////////////////////////////////////////////////////     
   
   
   
    // ALU SRCB MUX
    //////////////////////////////////////////////////////////////////////// 
    // Inputs: alu_srcb_wire, rf_rs2_wire, itype_wire, stype_wire, InstructionAddress, csr_RD_wire
    // Outputs: 
    logic [31:0] aluMUXB_to_alu_wire;
    
    always_comb begin
        case (alu_srcb_wire)
            3'b000: aluMUXB_to_alu_wire = Din2_memory_wire;
            3'b001: aluMUXB_to_alu_wire = itype_wire;
            3'b010: aluMUXB_to_alu_wire = stype_wire;
            3'b011: aluMUXB_to_alu_wire = InstructionAddress;
            3'b100: aluMUXB_to_alu_wire = csr_RD_wire;
            default: aluMUXB_to_alu_wire = 32'b0;
        endcase
    end  
    ////////////////////////////////////////////////////////////////////////     
    

    // ALU
    //////////////////////////////////////////////////////////////////////// 
    // Inputs: aluMUXA_to_alu_wire, aluMUXB_to_alu_wire, alu_fun_wire
    // Output: aluResult_wire
    ALU_MODULE OTTER_ALU(
    .ALU_scra   (aluMUXA_to_alu_wire),
    .ALU_scrb   (aluMUXB_to_alu_wire),
    .ALU_fun    (alu_fun_wire),
    .ALU_result (aluResult_wire)  ); 
    
    assign MCU_IOBUS_ADDR = aluResult_wire;
    //////////////////////////////////////////////////////////////////////// 

endmodule
