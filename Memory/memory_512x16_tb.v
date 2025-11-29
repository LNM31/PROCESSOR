`timescale 1ns/1ns
`include "memory_512x16.v"

module memory_512x16_tb;
    reg clk;
    reg rst_b;
    reg write;
    reg read;
    reg [8:0] addr;
    reg [15:0] din;
    wire [15:0] dout;

    // Instanțierea modulului de memorie
    memory_512x16 #(
        .AW(9),
        .DW(16),
        .INIT_FILE("")  // fără fișier de inițializare pentru test
    ) mem (
        .clk(clk),
        .rst_b(rst_b),
        .write(write),
        .read(read),
        .addr(addr),
        .din(din),
        .dout(dout)
    );

    // Generare clock
    initial begin
        clk = 0;
        forever #50 clk = ~clk;  // perioadă 100ns
    end

    // Test scenario
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, memory_512x16_tb, mem);
        
        // Inițializare
        rst_b = 0;
        write = 0;
        read = 0;
        addr = 9'd0;
        din = 16'd0;
        
        // Reset
        #100;
        rst_b = 1;
        #100;
        
        // Test 1: Scriere la adresa 0
        $display("Test 1: Scriere 0xABCD la adresa 0");
        write = 1;
        read = 0;
        addr = 9'd0;
        din = 16'hABCD;
        #100;
        
        // Test 2: Citire de la adresa 0
        $display("Test 2: Citire de la adresa 0");
        write = 0;
        read = 1;
        addr = 9'd0;
        #100;
        $display("Citit: 0x%h (asteptat: 0xABCD)", dout);
        
        // Test 3: Scriere secvențială la 10 adrese
        $display("Test 3: Scriere secventiala la 10 adrese");
        write = 1;
        read = 0;
        addr = 9'd0;
        repeat(10) begin
            din = {7'b0, addr};  // valoarea = adresa
            #100;
            addr = addr + 1;
        end
        
        // Test 4: Citire secvențială din 10 adrese
        $display("Test 4: Citire secventiala din 10 adrese");
        write = 0;
        read = 1;
        addr = 9'd0;
        repeat(10) begin
            #100;
            $display("Adresa %d: citit 0x%h", addr, dout);
            addr = addr + 1;
        end
        
        // Test 5: Scriere la adresa maximă (511)
        $display("Test 5: Scriere 0x1234 la adresa 511");
        write = 1;
        read = 0;
        addr = 9'd511;
        din = 16'h1234;
        #100;
        
        // Test 6: Citire de la adresa maximă
        $display("Test 6: Citire de la adresa 511");
        write = 0;
        read = 1;
        #100;
        $display("Citit: 0x%h (asteptat: 0x1234)", dout);
        
        // Test 7: Fără write (nu scrie)
        $display("Test 7: Incercare scriere cu write=0");
        write = 0;
        read = 0;
        addr = 9'd100;
        din = 16'hFFFF;
        #100;
        read = 1;
        #100;
        $display("Citit: 0x%h (nu ar trebui sa fie 0xFFFF)", dout);
        
        // Test 8: Reset în timpul operării
        $display("Test 8: Reset in timpul operarii");
        write = 1;
        read = 0;
        addr = 9'd50;
        din = 16'hDEAD;
        #50;
        rst_b = 0;
        #100;
        rst_b = 1;
        write = 0;
        read = 1;
        #100;
        $display("Citit dupa reset: 0x%h", dout);
        
        // Test 9: Read-after-write în același ciclu
        $display("Test 9: Read-after-write test");
        write = 1;
        read = 0;
        addr = 9'd200;
        din = 16'hCAFE;
        #100;
        $display("Read-after-write: dout=0x%h (asteptat: 0xCAFE)", dout);
        write = 0;
        
        #200;
        $display("=== Test finalizat! ===");
        $finish;
    end

    // Monitor pentru debug
    initial begin
        $monitor("Time=%0t | rst_b=%b write=%b read=%b addr=%d din=0x%h dout=0x%h", 
                 $time, rst_b, write, read, addr, din, dout);
    end

endmodule