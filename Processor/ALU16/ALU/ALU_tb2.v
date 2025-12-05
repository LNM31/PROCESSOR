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

        // MUL      start  selector inbus
        do_reset(2);
        drive_inputs(1'b1, 4'b0010, 16'd5); step();
        drive_inputs(1'b0, 4'b0010, 16'd2147); step();
        stepn(60);

        // DIV      start  selector inbus
        do_reset(2);
        drive_inputs(1'b1, 4'b0011, 16'd5); step();
        drive_inputs(1'b0, 4'b0011, 16'd2147); step();
        stepn(70);

        // MOD      start  selector inbus
        do_reset(2);
        drive_inputs(1'b1, 4'b0100, 16'd5); step();
        drive_inputs(1'b0, 4'b0100, 16'd2147); step();
        stepn(70);

        // LSR      start  selector inbus
        do_reset(2);
        drive_inputs(1'b1, 4'b0101, 16'd3); step();
        drive_inputs(1'b0, 4'b0101, 16'd5324); step();
        stepn(10);

        // LSL      start  selector inbus
        do_reset(2);
        drive_inputs(1'b1, 4'b0110, 16'd4); step();
        drive_inputs(1'b0, 4'b0110, 16'd17); step();
        stepn(12);

        // RSR      start  selector inbus
        do_reset(2);
        drive_inputs(1'b1, 4'b0111, 16'd2); step();
        drive_inputs(1'b0, 4'b0111, 16'd7); step();
        stepn(12);

        // RSL      start  selector inbus
        do_reset(2);
        drive_inputs(1'b1, 4'b1000, 16'd3); step();
        drive_inputs(1'b0, 4'b1000, 16'd52523); step();
        stepn(12);

        // AND      start  selector inbus
        do_reset(2);
        drive_inputs(1'b1, 4'b1001, 16'd14); step();
        drive_inputs(1'b0, 4'b1001, 16'd11); step();
        stepn(5);

        // OR       start  selector inbus
        do_reset(2);
        drive_inputs(1'b1, 4'b1010, 16'b1111000000000000); step();
        drive_inputs(1'b0, 4'b1010, 16'b0000000000001111); step();
        stepn(5);

        // XOR      start  selector inbus
        do_reset(2);
        drive_inputs(1'b1, 4'b1011, 16'b1111001111000000); step();
        drive_inputs(1'b0, 4'b1011, 16'b0000000111101111); step();
        stepn(5);

        // NOT      start  selector inbus
        do_reset(2);
        drive_inputs(1'b1, 4'b1100, 16'd0); step();
        drive_inputs(1'b0, 4'b1100, 16'b0000111100001111); step();
        stepn(5);

        // CMP      start  selector inbus
        do_reset(2);
        drive_inputs(1'b1, 4'b1101, 16'd29); step();
        drive_inputs(1'b0, 4'b1101, 16'd15); step();
        stepn(5);

        // TST      start  selector inbus
        do_reset(2);
        drive_inputs(1'b1, 4'b1110, 16'd33753); step();
        drive_inputs(1'b0, 4'b1110, 16'd43532); step();
        stepn(5);
        
        $finish();
    end

endmodule