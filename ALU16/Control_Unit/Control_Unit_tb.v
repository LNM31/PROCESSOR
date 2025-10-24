`timescale 1ns/1ps
//`include "Control_Unit.v"
`include "Control_Unit_Structural.v"
module Control_Unit_tb;
  reg clk,rst_b;
  reg [1:0]s;
  reg start,q0,q_1,a_8; //q_neg1 e q[-1],begin e start
  reg [2:0]cnt;
  wire [10:0]c;
  wire finish;
  
  Control_Unit con(
    .clk(clk),
    .rst_b(rst_b),
    .s(s),
    .start(start),
    .q0(q0),
    .q_1(q_1),
    .a_8(a_8),
    .cnt(cnt),
    .c(c),
    .finish(finish)
  );
  
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
    clk=0;
    rst_b=0;
    s=11;
    start=0;
    q0=0;
    q_1=1;
    a_8=0;
    cnt=7;
    #50;
  end
  initial begin
    #25; rst_b=~rst_b;
  end
  
  initial begin
    #180; start=1;
    #80; start=0;
  end
  
  integer j;
  initial begin
    for(j=1;j<=100;j=j+1)
    begin
      #50; clk=~clk;
    end
    #50;
  end
  
  // initial begin
  //   $display("TIME\ts\tstart\tq0\tq_1\ta_8\tcnt\tc\t\tcurrent_state\tnext_state\tfinish");
  //   $monitor("%0t\t%b\t%b\t%b\t%b\t%b\t%b\t%b\t%d\t\t%d\t\t%b",$time,s,start,q0,q_1,a_8,cnt,c,con.st,con.st_next,finish);
  // end
endmodule