`timescale 1ns/1ps
`include "CU_Extended.v"

module CU_Extended_tb2;
  // Clock & reset
  reg clk; reg rst_b;
  localparam integer CLK_PERIOD = 100; // ns

  // Inputs
  reg  [3:0] s;
  reg        start;
  reg        q0, q_1, a_16, cmp_cnt_m4;
  reg  [3:0] cnt;

  // Outputs
  wire [18:0] c;
  wire        finish;

  // DUT
  Control_Unit dut (
    .clk(clk), .rst_b(rst_b), .s(s), .start(start), .q0(q0), .q_1(q_1),
    .a_16(a_16), .cmp_cnt_m4(cmp_cnt_m4), .cnt(cnt), .c(c), .finish(finish)
  );

  // Clock
  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  // Helpers
  task automatic step; begin @(posedge clk); #0; end endtask
  task automatic stepn(input integer n);
    integer i; begin for (i=0;i<n;i=i+1) step(); end endtask
  task automatic reset_n;
    begin rst_b=1'b0; stepn(2); rst_b=1'b1; step(); end
  endtask

  // Waves
  initial begin
    $dumpfile("dump2.vcd");
    $dumpvars(0, CU_Extended_tb2);
    $display("t rst s start q0 q_1 a_16 cmp cnt | c finish");
    $monitor("%5t %b %4b  %b   %b  %b   %b    %b  %2d | %019b %b",
             $time, rst_b, s, start, q0, q_1, a_16, cmp_cnt_m4, cnt, c, finish);
  end

  // Stimulus
  initial begin
    // Defaults
    s=4'd0; start=0; q0=0; q_1=0; a_16=0; cmp_cnt_m4=0; cnt=4'd0; rst_b=1'b1;

    // Reset
    reset_n();

    // Scenario A: start pulse with s=0000, cnt sweep
    start=1; step(); start=0;
    repeat (4) begin cnt=cnt+1; step(); end

    // Scenario B: exercise cmp_cnt_m4 path with s=0010
    s=4'b0010; cmp_cnt_m4=0; step(); cmp_cnt_m4=1; stepn(3);

    // Scenario C: a_16 gating with max cnt
    cnt=4'd15; a_16=1; stepn(2); a_16=0; stepn(2);

    // Scenario D: toggle q0/q_1 to hit (q0 ~^ q_1)
    s=4'b0101; q0=0; q_1=1; step(); q0=1; q_1=0; step(); q0=1; q_1=1; step();

    // Scenario E: different opcode
    s=4'b1110; start=1; step(); start=0; stepn(5);

    $display("CU_Extended_tb2 finished");
    $finish;
  end
endmodule
