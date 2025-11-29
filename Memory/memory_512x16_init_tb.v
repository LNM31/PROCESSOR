`timescale 1ns/1ps
`include "memory_512x16.v"

module memory_512x16_init_tb;
    reg clk;
    reg rst_b;
    reg write;
    reg read;
    reg [8:0] addr;
    reg [15:0] din;
    wire [15:0] dout;

    // Instanțierea modulului de memorie CU FISIER DE INITIALIZARE
    memory_512x16 #(
        .AW(9),
        .DW(16),
        .INIT_FILE("program.hex")  // incarca instructiuni din fisier
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
        forever #50 clk = ~clk;  // perioada 100ns
    end

    // Test scenario
    initial begin
        $dumpfile("dump_init.vcd");
        $dumpvars(0, memory_512x16_init_tb);
        
        // Initializare
        rst_b = 0;
        write = 0;
        read = 0;
        addr = 9'd0;
        din = 16'd0;
        
        // Reset
        #100;
        rst_b = 1;
        #100;
        
        $display("=== Test memorie initializata din fisier ===");
        $display("Fisier: program.hex");
        $display("");
        
        // Citeste primele 16 adrese (incarcate din fisier)
        $display("--- Citire date incarcate din fisier ---");
        write = 0;
        read = 1;
        addr = 9'd0;
        
        repeat(16) begin
            #100;
            $display("Adresa %3d: 0x%h", addr, dout);
            addr = addr + 1;
        end
        
        // Verifica ca restul memoriei e 0
        $display("");
        $display("--- Verificare adrese neinitializate (ar trebui sa fie 0) ---");
        addr = 9'd16;
        #100;
        $display("Adresa %3d: 0x%h", addr, dout);
        
        addr = 9'd100;
        #100;
        $display("Adresa %3d: 0x%h", addr, dout);
        
        addr = 9'd511;
        #100;
        $display("Adresa %3d: 0x%h", addr, dout);
        
        // Test scriere peste date existente
        $display("");
        $display("--- Test suprascriere ---");
        write = 1;
        read = 0;
        addr = 9'd0;
        din = 16'hAAAA;
        #100;
        $display("Scris 0xAAAA la adresa 0");
        
        write = 0;
        read = 1;
        #100;
        $display("Citit de la adresa 0: 0x%h (asteptat: 0xAAAA)", dout);
        
        // Verifica adresa 1 (nu s-a schimbat)
        addr = 9'd1;
        #100;
        $display("Adresa 1 neschimbata: 0x%h (asteptat: 0x0002)", dout);
        
        #200;
        $display("");
        $display("=== Test finalizat! ===");
        $finish;
    end

    // Monitor pentru debug (optional - comenteaza daca e prea verbose)
    // initial begin
    //     $monitor("Time=%0t | read=%b addr=%d dout=0x%h", 
    //              $time, read, addr, dout);
    // end

endmodule
