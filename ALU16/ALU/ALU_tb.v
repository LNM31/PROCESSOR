`timescale 1ns/1ps
`include "ALU.v"
module ALU_tb;
    reg clk, rst_b, start;
    reg [3:0] s;
    reg [15:0] inbus;
    wire[15:0] outbus;
    wire finish;
    wire negative, zero, carry, overflow;

    ALU alu(
        .clk(clk),
        .rst_b(rst_b),
        .start(start),
        .s(s),
        .inbus(inbus),
        .outbus(outbus),
        .finish(finish),
        .negative(negative),
        .zero(zero),
        .carry(carry),
        .overflow(overflow)
    );

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, ALU_tb);
        clk=0;
        rst_b=1;
        start=0;
        inbus=0;
        s=0;
    end

    integer i;
    initial begin
        for(i=0;i<1000;i=i+1)
        begin
            #50; clk=~clk;
        end
        #50;
    end

    initial begin // adunare
        #10; rst_b=0; 
        #20; rst_b=1; 
        #20; s=4'b0000; 
        #40; start=1; 
        #50; inbus=16'b0000100001100011; //140 inbus=8'b00111001;//M 2147
        #30; start=0; 
        #60; inbus=16'b0000000000000101;  //230 inbus=8'b01010110; //Q 5
    end

    initial begin // scadere
        #800;
        #10; rst_b=0; 
        #20; rst_b=1; 
        #20; s=4'b0001; 
        #40; start=1; 
        #50; inbus=16'b0000000000000101; //140 inbus=8'b00111001;//M 2147 
        #30; start=0; 
        #60; inbus=16'b0000100001100011;  //230 inbus=8'b01010110; //Q 5
    end

    initial begin // adunare cu overflow
        #1500;
        #10; rst_b=0;
        #20; rst_b=1;
        #20; s=4'b0000;
        #40; start=1;
        #50; inbus=16'd16389; //M=127 test overflow adunare 127 + 126 = -3 in c2
        #30; start=0;
        #60; inbus=16'd16386; //Q=126 test overflow adunare 127 + 126 = -3 in c2
    end

    initial begin // scadere cu overflow
        #2300;
        #10; rst_b=0;
        #20; rst_b=1;
        #20; s=4'b0001;
        #40; start=1;
        #50; inbus=16'b0000000000000001; //M=1 test overflow scadere -128 - 1 = 127 in c2
        #30; start=0;
        #60; inbus=16'b1000000000000000; //Q=-128 test overflow scadere -128 -1 = 127 in c2
    end

    initial begin // inmultire
        #3100;
        #10; rst_b=0;
        #20; rst_b=1;
        #20; s=4'b0010;
        #40; start=1;
        #50; inbus=16'b0000100001100011;//M -93 inmultire
        #30; start=0;
        #60; inbus=16'b0000000000000101;//Q -115 inmultire
        //00101001     11000111
    end

    initial begin // inmultire
        #12500;
        #10; rst_b=0;
        #20; rst_b=1;
        #20; s=4'b0010;
        #40; start=1;
        #50; inbus=16'b0000100001101011;//M -93 inmultire
        #30; start=0;
        #60; inbus=16'b0000000000001101;//Q -115 inmultire 
        //11011110    01000111
    end

    initial begin // inmultire
        #22800;
        #10; rst_b=0;
        #20; rst_b=1;
        #20; s=4'b0010;
        #40; start=1;
        #50; inbus=16'd2350;//M -93 inmultire
        #30; start=0;
        #60; inbus=16'd159;//Q -115 inmultire
        //00101100     00101001
    end

    initial begin // impartire
        #33300;
        #10; rst_b=0;
        #20; rst_b=1;
        #20; s=4'b0011;
        #40; start=1;
        #50; inbus=16'b0000000000000101;//Q -115 inmultire
        #30; start=0;
        #60; inbus=16'b0000100001100011;//M -93 inmultire
        //9     16
    end

    initial begin // impartire
        #43200;
        #10; rst_b=0;
        #20; rst_b=1;
        #20; s=4'b0011;
        #40; start=1;
        #50; inbus=16'd145;//M -93 inmultire
        #30; start=0;
        #60; inbus=16'd18921;//Q -115 inmultire
        //7     20
    end

    initial begin
        #43300;
        $display("A:%b", alu.a_out | 16'd7);
    end


endmodule