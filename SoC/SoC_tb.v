`timescale 1ns/1ps

module SoC_tb;

  // Inputs
  reg start;
  reg clk;
  reg rst_b;

  // Outputs
  wire finish;

  // Instanțiere SoC
  SoC uut (
    .start(start),
    .clk(clk),
    .rst_b(rst_b),
    .finish(finish)
  );

  // Clock generator - 100ns period (10MHz)
  initial clk = 0;
  always #50 clk = ~clk;

  // Test sequence
  initial begin
    // Inițializare VCD pentru GTKWave
    $dumpfile("soc_dump.vcd");
    $dumpvars(0, SoC_tb);  // Toate semnalele din testbench
    
    // Inițializare semnale
    start = 0;
    rst_b = 0;

    // Reset activ (low) pentru 200ns
    #200;
    rst_b = 1;

    // Așteaptă puțin după reset
    #100;

    // Pornește execuția
    start = 1;
    #100;
    start = 0;  // Pulse pe start

    // Așteaptă să termine (finish = 1) sau timeout
    wait(finish == 1 || $time > 50000);

    if (finish) begin
      $display("===========================================");
      $display("SUCCES: CPU a terminat executia la t=%0dns", $time);
      $display("===========================================");
    end else begin
      $display("===========================================");
      $display("TIMEOUT: CPU nu a terminat in 50us");
      $display("===========================================");
    end

    // Mai rulează puțin pentru a vedea starea finală
    #500;

    // Afișează starea finală a registrelor
    $display("\n--- Stare finala registre ---");
    $display("PC  = %h", uut.cpu.pc_out);
    $display("IR  = %h", uut.cpu.ir_out);
    $display("AC  = %h", uut.cpu.ac_out);
    $display("X   = %h", uut.cpu.x_out);
    $display("Y   = %h", uut.cpu.y_out);
    $display("AR  = %h", uut.cpu.ar_out);
    $display("SP  = %h", uut.cpu.sp_out);
    
    $display("\n--- Memorie (adresele 6-11) ---");
    $display("Mem[0006] = %h", uut.memory.mem[6]);
    $display("Mem[0007] = %h", uut.memory.mem[7]);
    $display("Mem[0008] = %h", uut.memory.mem[8]);
    $display("Mem[0009] = %h", uut.memory.mem[9]);
    $display("Mem[000A] = %h", uut.memory.mem[10]);
    $display("Mem[000B] = %h", uut.memory.mem[11]);

    $finish;
  end

  //Monitor pentru debugging (opțional - decomentează dacă vrei)
  always @(posedge clk) begin
    if (rst_b && uut.cpu.c != 0) begin
      $display("t=%0dns | State=%b | PC=%h | IR=%h | AC=%h | AR=%h", 
               $time, uut.cpu.c, uut.cpu.pc_out, uut.cpu.ir_out, 
               uut.cpu.ac_out, uut.cpu.ar_out);
    end
  end

endmodule
