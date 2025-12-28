`timescale 1ns/1ps

module PC_tb;
    reg clk, rst_b;
    reg ld, inc;
    reg [15:0] in;
    wire [15:0] out;

    // Instantiere PC
    PC uut(
        .clk(clk),
        .rst_b(rst_b),
        .ld(ld),
        .inc(inc),
        .in(in),
        .out(out)
    );

    // Generare clock
    initial clk = 0;
    always #5 clk = ~clk;

    // Generare VCD pentru GTKWave
    initial begin
        $dumpfile("tb/pc_dump.vcd");
        $dumpvars(0, PC_tb);
    end

    // Test sequence
    initial begin
        $display("=== PC Testbench ===");
        $display("Time\tld\tinc\tin\t\tout");
        $monitor("%0t\t%b\t%b\t%h\t\t%h", $time, ld, inc, in, out);

        // Initializare
        rst_b = 0;
        ld = 0;
        inc = 0;
        in = 16'h0000;

        // Test 1: Reset
        #10;
        rst_b = 1;
        #10;
        $display("\n[TEST 1] Reset - out should be 0: %s", (out == 16'h0000) ? "PASS" : "FAIL");

        // Test 2: Load valoare 0x1234
        in = 16'h1234;
        ld = 1;
        #10;
        ld = 0;
        $display("[TEST 2] Load 0x1234 - out should be 0x1234: %s", (out == 16'h1234) ? "PASS" : "FAIL");

        // Test 3: Increment
        inc = 1;
        #10;
        $display("[TEST 3] Increment - out should be 0x1235: %s", (out == 16'h1235) ? "PASS" : "FAIL");
        #10;
        $display("[TEST 3] Increment - out should be 0x1236: %s", (out == 16'h1236) ? "PASS" : "FAIL");
        #10;
        $display("[TEST 3] Increment - out should be 0x1237: %s", (out == 16'h1237) ? "PASS" : "FAIL");
        inc = 0;

        // Test 4: Hold (nici load, nici increment)
        #10;
        $display("[TEST 4] Hold - out should still be 0x1237: %s", (out == 16'h1237) ? "PASS" : "FAIL");
        #10;
        $display("[TEST 4] Hold - out should still be 0x1237: %s", (out == 16'h1237) ? "PASS" : "FAIL");

        // Test 5: Prioritate ld > inc (ambele active, load castiga)
        in = 16'hABCD;
        ld = 1;
        inc = 1;
        #10;
        $display("[TEST 5] Priority ld > inc - out should be 0xABCD: %s", (out == 16'hABCD) ? "PASS" : "FAIL");
        ld = 0;
        inc = 0;

        // Test 6: Increment de la 0xFFFF (overflow)
        in = 16'hFFFF;
        ld = 1;
        #10;
        ld = 0;
        inc = 1;
        #10;
        $display("[TEST 6] Overflow - 0xFFFF + 1 should be 0x0000: %s", (out == 16'h0000) ? "PASS" : "FAIL");
        inc = 0;

        // Test 7: Load 0x0000 si increment
        in = 16'h0000;
        ld = 1;
        #10;
        ld = 0;
        inc = 1;
        #10;
        $display("[TEST 7] 0x0000 + 1 should be 0x0001: %s", (out == 16'h0001) ? "PASS" : "FAIL");

        #20;
        $display("\n=== PC Testbench Complete ===");
        $finish;
    end
endmodule
