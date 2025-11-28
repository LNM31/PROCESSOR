`timescale 1ns/1ps
module ALU_tb2;
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

    localparam integer CLK_PERIOD = 100; // ns

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
        input        start_i,
        input [3:0]  s_i,
        input [15:0] inbus_i
    );
        begin
        start      = start_i;
        s          = s_i;
        inbus      = inbus_i;
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

    // Wave dumps and live monitor
    initial begin
        $dumpfile("dump1.fst");
        $dumpvars(0, ALU_tb2);
    end

    initial begin
        s = 4'd0; start = 1'd0; inbus = 16'd0; 

        // ADD      start  selector inbus
        do_reset(2);
        drive_inputs(1'b1, 4'b0000, 16'd5); step();
        drive_inputs(1'b0, 4'b0000, 16'd2147); step();
        stepn(5);
        
        // SUB      start  selector inbus
        do_reset(2);
        drive_inputs(1'b1, 4'b0001, 16'd5); step();
        drive_inputs(1'b0, 4'b0001, 16'd2147); step();
        stepn(5);
        
        $finish();
    end

endmodule