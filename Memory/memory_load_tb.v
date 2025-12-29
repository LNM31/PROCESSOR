`timescale 1ns/1ps

module memory_load_tb;

  reg clk;
  reg rst_b;
  reg write, read;
  reg [8:0] addr;
  reg [15:0] din;
  wire [15:0] dout;

  // Instanțiere memorie
  memory_512x16 #(
    .INIT_FILE("program.hex")
  ) mem (
    .clk(clk),
    .rst_b(rst_b),
    .write(write),
    .read(read),
    .addr(addr),
    .din(din),
    .dout(dout)
  );

  // Clock
  initial clk = 0;
  always #50 clk = ~clk;

  integer i;

  initial begin
    // Inițializare
    rst_b = 0;
    write = 0;
    read = 0;
    addr = 0;
    din = 0;

    #100;
    rst_b = 1;
    #100;

    // Afișează conținutul memoriei (primele 32 locații)
    $display("\n============================================");
    $display("  CONTINUT MEMORIE DUPA INCARCARE program.hex");
    $display("============================================\n");
    
    $display("  Addr  |  Hex   |  Decimal  |  Binary");
    $display("--------|--------|-----------|------------------");
    
    for (i = 0; i < 32; i = i + 1) begin
      $display("  %04h  |  %04h  |  %5d    |  %b", i[8:0], mem.mem[i], mem.mem[i], mem.mem[i]);
    end

    $display("\n============================================");
    $display("  ZONA INSTRUCTIUNI (0x00 - 0x0F)");
    $display("============================================\n");
    
    for (i = 0; i < 16; i = i + 1) begin
      $display("  Mem[%02d] = 0x%04h", i, mem.mem[i]);
    end

    $display("\n============================================");
    $display("  VERIFICARE CITIRE PRIN PORT");
    $display("============================================\n");

    // Testează citirea prin portul read
    for (i = 0; i < 16; i = i + 1) begin
      addr = i;
      read = 1;
      #100;
      $display("  Read Mem[%02d] = 0x%04h (dout)", i, dout);
    end
    read = 0;

    $display("\n============================================");
    $display("  TEST COMPLET");
    $display("============================================\n");

    $finish;
  end

endmodule
