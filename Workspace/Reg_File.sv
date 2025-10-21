`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly   
// Engineer: Binh To
// 
// Create Date: 10/12/2023 04:28:28 PM
// Module Name: REG_FILE
// Project Name: Register File
// Target Devices: Basys3
// Tool Versions: 
// Description:  The machine takes in an input and stores it in a register     // indicated by the user. ADR1 and ADR2 are the two address inputs
// needed to access two registers simultaneously. The output is then saved and // outputted through RS1 and RS2. Write address tells you where to save
// the data and WD is the data to be saved.
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module REG_FILE(
    input logic [4:0] RF_ADR1, 
    input logic [4:0] RF_ADR2,
    input logic [4:0] RF_WA,
    input logic [31:0] RF_WD,
    input logic RF_CLK,
    input logic RF_EN,
    output logic [31:0] RF_RS1, 
    output logic [31:0] RF_RS2    
    );
    
    // Create a memory module with 32-bit width and 32 addresses
    logic [31:0] ram [0:31];
    
    // Initialize every value at all 32 addresses to be 0
    initial begin
    int i;
    for (i=0; i<32; i=i+1) begin
        // Access the ram memory by address
        ram[i] = 0;
        end
    end

    always_ff @ (posedge RF_CLK) begin
        // Write data to RAM specified by the register 
        // Store WD value to the ram memory specified by WA (rd)
        // Also prevents non-zero values to be outputted through register x0
        if ((RF_EN == 1) && (RF_WA != 5'h0)) begin
            ram[RF_WA] <= RF_WD;
        end
    end

    always_comb begin      
        // Output data from specified register, including zeroth register
        RF_RS1 = ram[RF_ADR1];
        RF_RS2 = ram[RF_ADR2];

    end
endmodule