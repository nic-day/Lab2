`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/15/2021 06:40:11 PM
// Design Name: 
// Module Name: top_demo
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


module top_demo
(
  // input
  input  logic [7:0] sw,
  input  logic [3:0] btn,
  input  logic       sysclk_125mhz,
  input  logic       rst,
  // output  
  output logic [7:0] led,
  output logic sseg_ca,
  output logic sseg_cb,
  output logic sseg_cc,
  output logic sseg_cd,
  output logic sseg_ce,
  output logic sseg_cf,
  output logic sseg_cg,
  output logic sseg_dp,
  output logic [3:0] sseg_an
 
);

  logic [16:0] CURRENT_COUNT;
  logic [16:0] NEXT_COUNT;
  logic        smol_clk;
  logic [63:0] key, plaintext;
  logic [63:0] cyphertext;
  logic [63:0] sel;
  logic [15:0] display;
  assign key = 64'h433E4529462A4A62;
  assign plaintext = 64'h2579DB866C0F528C;
  
  // Place des_dut instantiation here
  DES dut(key, plaintext, sw[0], cyphertext, sw[1]);
  
  //case block for seven segment output
  always @ (*) begin
    case(sw[7:6])
        2'b00    :   sel = cyphertext;
        2'b01    :   sel = plaintext;
        2'b10    :   sel = key;
        default  :   sel = 64'h0;
    endcase
    case(sw[5:4])
        2'b00    :  display = sel[15:0];
        2'b01    :  display = sel[31:16];
        2'b10    :  display = sel[47:32];
        2'b11    :  display = sel[63:48];
    endcase
  end
        
  
  // 7-segment display
  segment_driver driver(
  .clk(smol_clk),
  .rst(btn[3]),
  .digit0(display[3:0]),
  .digit1(display[7:4]),
  .digit2(display[11:8]),
  .digit3(display[15:12]),
  .decimals({1'b0, btn[2:0]}),
  .segment_cathodes({sseg_dp, sseg_cg, sseg_cf, sseg_ce, sseg_cd, sseg_cc, sseg_cb, sseg_ca}),
  .digit_anodes(sseg_an)
  );

// Register logic storing clock counts
  always@(posedge sysclk_125mhz)
  begin
    if(btn[3])
      CURRENT_COUNT = 17'h00000;
    else
      CURRENT_COUNT = NEXT_COUNT;
  end
  
  // Increment logic
  assign NEXT_COUNT = CURRENT_COUNT == 17'd100000 ? 17'h00000 : CURRENT_COUNT + 1;

  // Creation of smaller clock signal from counters
  assign smol_clk = CURRENT_COUNT == 17'd100000 ? 1'b1 : 1'b0;

endmodule
