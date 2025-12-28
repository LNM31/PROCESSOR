`timescale 1ns/1ps

module SP_tb;
    reg clk, rst_b;
    reg ld, inc, dec;
    reg [15:0] in;
    wire [15:0] out;

    // Instantiere SP
    SP uut(
        .clk(clk),
        .rst_b(rst_b),
        .ld(ld),
        .inc(inc),
        .dec(dec),
        .in(in),
        .out(out)
    );

    // Generare clock
    initial clk = 0;
    always #5 clk = ~clk;

    // Generare VCD pentru GTKWave
    initial begin
        $dumpfile("tb/sp_dump.vcd");
        $dumpvars(0, SP_tb);
    end

    // Test sequence
    initial begin
        $display("=== SP Testbench ===");
        $display("Time\tld\tinc\tdec\tin\t\tout");
        $monitor("%0t\t%b\t%b\t%b\t%h\t\t%h", $time, ld, inc, dec, in, out);

        // Initializare
        rst_b = 0;
        ld = 0;
        inc = 0;
        dec = 0;
        in = 16'h0000;

        // Test 1: Reset
        #10;
        rst_b = 1;
        #10;
        $display("\n[TEST 1] Reset - out should be 0: %s", (out == 16'h0000) ? "PASS" : "FAIL");

        // Test 2: Load valoare 512 (0x0200) - initializare SP
        in = 16'h0200;  // 512
        ld = 1;
        #10;
        ld = 0;
        $display("[TEST 2] Load 512 (0x0200) - out should be 0x0200: %s", (out == 16'h0200) ? "PASS" : "FAIL");

        // Test 3: Decrement (PUSH simulare - SP scade)
        dec = 1;
        #10;
        $display("[TEST 3] Decrement - out should be 0x01FF (511): %s", (out == 16'h01FF) ? "PASS" : "FAIL");
        #10;
        $display("[TEST 3] Decrement - out should be 0x01FE (510): %s", (out == 16'h01FE) ? "PASS" : "FAIL");
        #10;
        $display("[TEST 3] Decrement - out should be 0x01FD (509): %s", (out == 16'h01FD) ? "PASS" : "FAIL");
        dec = 0;

        // Test 4: Increment (POP simulare - SP creste)
        inc = 1;
        #10;
        $display("[TEST 4] Increment - out should be 0x01FE (510): %s", (out == 16'h01FE) ? "PASS" : "FAIL");
        #10;
        $display("[TEST 4] Increment - out should be 0x01FF (511): %s", (out == 16'h01FF) ? "PASS" : "FAIL");
        #10;
        $display("[TEST 4] Increment - out should be 0x0200 (512): %s", (out == 16'h0200) ? "PASS" : "FAIL");
        inc = 0;

        // Test 5: Hold
        #10;
        $display("[TEST 5] Hold - out should still be 0x0200: %s", (out == 16'h0200) ? "PASS" : "FAIL");
        #10;
        $display("[TEST 5] Hold - out should still be 0x0200: %s", (out == 16'h0200) ? "PASS" : "FAIL");

        // Test 6: Prioritate ld > inc (ambele active, load castiga)
        in = 16'h0100;
        ld = 1;
        inc = 1;
        #10;
        $display("[TEST 6] Priority ld > inc - out should be 0x0100: %s", (out == 16'h0100) ? "PASS" : "FAIL");
        ld = 0;
        inc = 0;

        // Test 7: Prioritate ld > dec (ambele active, load castiga)
        in = 16'h0300;
        ld = 1;
        dec = 1;
        #10;
        $display("[TEST 7] Priority ld > dec - out should be 0x0300: %s", (out == 16'h0300) ? "PASS" : "FAIL");
        ld = 0;
        dec = 0;

        // Test 8: Prioritate inc > dec (ambele active, increment castiga)
        inc = 1;
        dec = 1;
        #10;
        $display("[TEST 8] Priority inc > dec - out should be 0x0301: %s", (out == 16'h0301) ? "PASS" : "FAIL");
        inc = 0;
        dec = 0;

        // Test 9: Decrement de la 0x0000 (underflow)
        in = 16'h0000;
        ld = 1;
        #10;
        ld = 0;
        dec = 1;
        #10;
        $display("[TEST 9] Underflow - 0x0000 - 1 should be 0xFFFF: %s", (out == 16'hFFFF) ? "PASS" : "FAIL");
        dec = 0;

        // Test 10: Increment de la 0xFFFF (overflow)
        inc = 1;
        #10;
        $display("[TEST 10] Overflow - 0xFFFF + 1 should be 0x0000: %s", (out == 16'h0000) ? "PASS" : "FAIL");
        inc = 0;

        #20;
        $display("\n=== SP Testbench Complete ===");
        $finish;
    end
endmodule
