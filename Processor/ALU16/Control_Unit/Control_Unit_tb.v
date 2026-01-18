`timescale 1ns/1ps
`include "CU_Extended2.v"

module CU_Extended_tb;

  // Clocking and reset
  reg clk;
  reg rst_b;
  localparam integer CLK_PERIOD = 100; // ns

  // Inputs
  reg  [3:0] s;
  reg        start;
  reg        q0, q_1, a_16, cmp_cnt_m4;
  reg  [3:0] cnt;

  // Outputs
  wire [18:0] c;
  wire        finish;

  // dut
  Control_Unit dut (
    .clk(clk),
    .rst_b(rst_b),
    .s(s),
    .start(start),
    .q0(q0),
    .q_1(q_1),
    .a_16(a_16),
    .cmp_cnt_m4(cmp_cnt_m4),
    .cnt(cnt),
    .c(c),
    .finish(finish)
  );

  // Clock generation
  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  // Utilities
  task automatic do_reset(input integer cycles);
    begin
      rst_b = 1'b0;
      repeat (cycles) @(posedge clk);
      rst_b = 1'b1;
      @(posedge clk);
    end
  endtask

  task automatic drive_inputs(
    input [3:0] s_i,
    input       start_i,
    input       q0_i,
    input       q_1_i,
    input       a_16_i,
    input       cmp_cnt_m4_i,
    input [3:0] cnt_i
  );
    begin
      s          = s_i;
      start      = start_i;
      q0         = q0_i;
      q_1        = q_1_i;
      a_16       = a_16_i;
      cmp_cnt_m4 = cmp_cnt_m4_i;
      cnt        = cnt_i;
    end
  endtask

  // Advance one clock, then wait a delta (#0) so NBAs settle before sampling
  task automatic step; begin @(posedge clk); #0; end endtask
  task automatic stepn(input integer n);
    integer i;
    begin
      for (i = 0; i < n; i = i + 1) step();
    end
  endtask

  // Simple checker (fill expected values when known)
  task automatic check_outputs(
    input [18:0] exp_c,
    input        exp_finish,
    input [8*64:1] name
  );
    begin
      if (!((c === exp_c) && (finish === exp_finish)))
      begin
        //$display("[%0t] PASS %-20s c=%b finish=%b", $time, name, c, finish);
        $error  ("[%0t] FAIL %-20s", $time, name);
        $display("        exp c=%b got c=%b | exp fin=%b got fin=%b", exp_c, c, exp_finish, finish);
      end
    end
  endtask

  function [4:0] dec5(
    input f4, f3, f2, f1, f0
  );
    begin
      dec5 = {f4, f3, f2, f1, f0};
    end
  endfunction

  // Wave dumps and live monitor
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(0, CU_Extended_tb);
  end

  wire [4:0] state_bits     = {dut.f4.q, dut.f3.q, dut.f2.q, dut.f1.q, dut.f0.q};
  wire [4:0] next_state_bits= {dut.f4.d, dut.f3.d, dut.f2.d, dut.f1.d, dut.f0.d};

  initial begin
    $monitor("%8t  %b  %4b   %b    %b  %b   %b    %b  %2d %3d | %3d     %019b   %b",
             $time, rst_b, s, start, q0, q_1, a_16, cmp_cnt_m4, cnt,
             state_bits, next_state_bits, c, finish);
  end

  // Main stimulus
  initial begin
    // Defaults
    s = 4'd0; start = 1'b0; q0 = 1'b0; q_1 = 1'b0; a_16 = 1'b0; cmp_cnt_m4 = 1'b0; cnt = 4'd0;

    // ADD          sel   start q0    q_1   a_16  ccm4  cnt
    $display("ADD:");
    $display("Time    rst sel  start q0 q_1 a_16 cmp cnt st | st_next       c[18:0]      finish");
    do_reset(2);
     
    /* S0 */ drive_inputs(4'b0000, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step(); // S1
    check_outputs(19'd1, 1'b0, "check c");
    drive_inputs(4'b0000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step(); // S2
    stepn(4);
    $display();
    
    // SUB          sel   start q0    q_1   a_16  ccm4  cnt
    $display("SUB:");
    $display("Time    rst sel  start q0 q_1 a_16 cmp cnt st | st_next       c[18:0]      finish");
    do_reset(2);

    /* S0 */ drive_inputs(4'b0001, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step(); // S1
    drive_inputs(4'b0001, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step(); // S2
    stepn(4);
    $display();

    
    $display("MUL:");
    $display("Time    rst sel  start q0 q_1 a_16 cmp cnt st | st_next       c[18:0]      finish");
    do_reset(2);
    //                        sel   start   q0    q_1   a_16 ccm4  cnt
    /* S0  */ drive_inputs(4'b0010, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S1  */ drive_inputs(4'b0010, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S2  */ step();
    /* S9  */ drive_inputs(4'b0010, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 4'd0); step();
    /* S7  */ step();
    /* S8  */ step();
    /* S9  */ drive_inputs(4'b0010, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S5  */ drive_inputs(4'b0010, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S7  */ step();
    /* S8  */ step();
    /* S9  */ drive_inputs(4'b0010, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 4'd0); step();
    /* S4  */ drive_inputs(4'b0010, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S7  */ drive_inputs(4'b0010, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd15); step();
    /* S6  */ drive_inputs(4'b0010, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S10 */ step();
    /* S0  */ step();
    $display();

    $display("DIV:");
    $display("Time    rst sel  start q0 q_1 a_16 cmp cnt st | st_next       c[18:0]      finish");
    do_reset(2);
    //                        sel   start   q0    q_1   a_16 ccm4  cnt
    /* S0  */ drive_inputs(4'b0011, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S1  */ drive_inputs(4'b0011, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S2  */ step();
    /* S11 */ drive_inputs(4'b0011, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 4'd0); step();
    /* S4  */ step();
    /* S12 */ drive_inputs(4'b0011, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd5); step();
    /* S13 */ drive_inputs(4'b0011, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S8  */ drive_inputs(4'b0011, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S5  */ drive_inputs(4'b0011, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S12 */ drive_inputs(4'b0011, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 4'd15); step();
    /* S14 */ drive_inputs(4'b0011, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S10 */ drive_inputs(4'b0011, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S0  */ drive_inputs(4'b0011, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    $display();
    
    $display("MOD:");
    $display("Time    rst sel  start q0 q_1 a_16 cmp cnt st | st_next       c[18:0]      finish");
    do_reset(2);
    //                        sel   start   q0    q_1   a_16 ccm4  cnt
    /* S0  */ drive_inputs(4'b0100, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S1  */ drive_inputs(4'b0100, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S2  */ step();
    /* S11 */ drive_inputs(4'b0100, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 4'd0); step();
    /* S4  */ step();
    /* S12 */ drive_inputs(4'b0100, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd5); step();
    /* S13 */ drive_inputs(4'b0100, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S8  */ drive_inputs(4'b0100, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S5  */ drive_inputs(4'b0100, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S12 */ drive_inputs(4'b0100, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 4'd15); step();
    /* S14 */ drive_inputs(4'b0100, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S6  */ drive_inputs(4'b0100, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S0  */ drive_inputs(4'b0100, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    $display();
    
    $display("LSR:");
    $display("Time    rst sel  start q0 q_1 a_16 cmp cnt st | st_next       c[18:0]      finish");
    do_reset(2);
    //                        sel   start   q0    q_1   a_16 ccm4  cnt
    /* S0  */ drive_inputs(4'b0101, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S1  */ drive_inputs(4'b0101, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S2  */ drive_inputs(4'b0101, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S15 */ drive_inputs(4'b0101, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S8  */ drive_inputs(4'b0101, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S15 */ drive_inputs(4'b0101, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd5); step();
    /* S8  */ drive_inputs(4'b0101, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 4'd0); step();
    /* S10 */ drive_inputs(4'b0101, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S0  */ drive_inputs(4'b0101, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    $display();
    
    $display("LSL:");
    $display("Time    rst sel  start q0 q_1 a_16 cmp cnt st | st_next       c[18:0]      finish");
    do_reset(2);
    //                        sel   start   q0    q_1   a_16 ccm4  cnt
    /* S0  */ drive_inputs(4'b0110, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S1  */ drive_inputs(4'b0110, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S2  */ drive_inputs(4'b0110, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S16 */ drive_inputs(4'b0110, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S8  */ drive_inputs(4'b0110, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S16 */ drive_inputs(4'b0110, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd5); step();
    /* S8  */ drive_inputs(4'b0110, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 4'd0); step();
    /* S10 */ drive_inputs(4'b0110, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S0  */ drive_inputs(4'b0110, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    $display();
    
    $display("RSR:");
    $display("Time    rst sel  start q0 q_1 a_16 cmp cnt st | st_next       c[18:0]      finish");
    do_reset(2);
    //                        sel   start   q0    q_1   a_16 ccm4  cnt
    /* S0  */ drive_inputs(4'b0111, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S1  */ drive_inputs(4'b0111, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S2  */ drive_inputs(4'b0111, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S17 */ drive_inputs(4'b0111, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S8  */ drive_inputs(4'b0111, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S17 */ drive_inputs(4'b0111, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd5); step();
    /* S8  */ drive_inputs(4'b0111, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 4'd0); step();
    /* S10 */ drive_inputs(4'b0111, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S0  */ drive_inputs(4'b0111, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    $display();
    
    $display("RSL:");
    $display("Time    rst sel  start q0 q_1 a_16 cmp cnt st | st_next       c[18:0]      finish");
    do_reset(2);
    //                        sel   start   q0    q_1   a_16 ccm4  cnt
    /* S0  */ drive_inputs(4'b1000, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S1  */ drive_inputs(4'b1000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S2  */ drive_inputs(4'b1000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S18 */ drive_inputs(4'b1000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S8  */ drive_inputs(4'b1000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S18 */ drive_inputs(4'b1000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd5); step();
    /* S8  */ drive_inputs(4'b1000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 4'd0); step();
    /* S10 */ drive_inputs(4'b1000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S0  */ drive_inputs(4'b1000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    $display();
    
    $display("AND:");
    $display("Time    rst sel  start q0 q_1 a_16 cmp cnt st | st_next       c[18:0]      finish");
    do_reset(2);
    //                        sel   start   q0    q_1   a_16 ccm4  cnt
    /* S0  */ drive_inputs(4'b1001, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S1  */ drive_inputs(4'b1001, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S2  */ drive_inputs(4'b1001, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S19 */ drive_inputs(4'b1001, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S6  */ drive_inputs(4'b1001, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S0  */ drive_inputs(4'b1001, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    $display();
    
    $display("OR:");
    $display("Time    rst sel  start q0 q_1 a_16 cmp cnt st | st_next       c[18:0]      finish");
    do_reset(2);
    //                        sel   start   q0    q_1   a_16 ccm4  cnt
    /* S0  */ drive_inputs(4'b1010, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S1  */ drive_inputs(4'b1010, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S2  */ drive_inputs(4'b1010, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S20 */ drive_inputs(4'b1010, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S6  */ drive_inputs(4'b1010, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S0  */ drive_inputs(4'b1010, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    $display();

    $display("XOR:");
    $display("Time    rst sel  start q0 q_1 a_16 cmp cnt st | st_next       c[18:0]      finish");
    do_reset(2);
    //                        sel   start   q0    q_1   a_16 ccm4  cnt
    /* S0  */ drive_inputs(4'b1011, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S1  */ drive_inputs(4'b1011, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S2  */ drive_inputs(4'b1011, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S21 */ drive_inputs(4'b1011, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S6  */ drive_inputs(4'b1011, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S0  */ drive_inputs(4'b1011, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    $display();

    $display("NOT:");
    $display("Time    rst sel  start q0 q_1 a_16 cmp cnt st | st_next       c[18:0]      finish");
    do_reset(2);
    //                        sel   start   q0    q_1   a_16 ccm4  cnt
    /* S0  */ drive_inputs(4'b1100, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S1  */ drive_inputs(4'b1100, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S2  */ drive_inputs(4'b1100, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S22 */ drive_inputs(4'b1100, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S6  */ drive_inputs(4'b1100, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S0  */ drive_inputs(4'b1100, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    $display();

    $display("CMP:");
    $display("Time    rst sel  start q0 q_1 a_16 cmp cnt st | st_next       c[18:0]      finish");
    do_reset(2);
    //                        sel   start   q0    q_1   a_16 ccm4  cnt
    /* S0  */ drive_inputs(4'b1101, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S1  */ drive_inputs(4'b1101, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S2  */ drive_inputs(4'b1101, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S3  */ drive_inputs(4'b1101, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S5  */ drive_inputs(4'b1101, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S0  */ drive_inputs(4'b1101, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    $display();

    $display("TST:");
    $display("Time    rst sel  start q0 q_1 a_16 cmp cnt st | st_next       c[18:0]      finish");
    do_reset(2);
    //                        sel   start   q0    q_1   a_16 ccm4  cnt
    /* S0  */ drive_inputs(4'b1110, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S1  */ drive_inputs(4'b1110, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S2  */ drive_inputs(4'b1110, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S19 */ drive_inputs(4'b1110, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    /* S0  */ drive_inputs(4'b1110, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 4'd0); step();
    $display();

    stepn(3);
    $display("Testbench finished");
    $finish;
  end

endmodule