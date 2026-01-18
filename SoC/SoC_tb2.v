`timescale 1ns/1ps

module SoC_tb2;

  // Inputs
  reg clk, rst_b, start;

  // Outputs
  wire finish;

  // Instantiate SoC
  SoC uut (
    .start(start),
    .clk(clk),
    .rst_b(rst_b),
    .finish(finish)
  );

  // Clock
  localparam integer CLK_PERIOD = 100; // ns
  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  // Utility tasks

  // Reset for N cycles
  task automatic do_reset(input integer cycles);
    begin
      rst_b = 1'b0;
      start = 1'b0;
      repeat (cycles) @(posedge clk);
      rst_b = 1'b1;
      @(posedge clk); #0;
    end
  endtask

  // Advance one cycle + delta delay for NBA stabilization
  task automatic step; 
    begin 
      @(posedge clk); #0; 
    end 
  endtask

  // Advance N cycles
  task automatic stepn(input integer n);
    integer i;
    begin
      for (i = 0; i < n; i = i + 1) step();
    end
  endtask

  // Start the CPU (pulse on start)
  task automatic cpu_start;
    begin
      start = 1'b1;
      step();
      start = 1'b0;
    end
  endtask

  // Wait until finish or timeout
  // finish = 1 in S0 (idle), 0 while running, 1 when returning to S0 (done)
  task automatic wait_finish(input integer max_cycles);
    integer cnt;
    begin
      cnt = 0;
      // Wait for finish to become 0 (CPU has started)
      while (finish == 1 && cnt < max_cycles) begin
        step();
        cnt = cnt + 1;
      end
      // Wait for finish to become 1 (CPU has stopped, returns to S0)
      while (finish == 0 && cnt < max_cycles) begin
        step();
        cnt = cnt + 1;
      end
      if (finish)
        $display("CPU has finished after %0d clock cycles", cnt);
      else
        $display("TIMEOUT after %0d clock cycles!", cnt);
    end
  endtask

  // Functions for state decoding (One-Hot)
  // Function that returns the active state number (0-173) from one-hot
  function [7:0] get_state_num;
    input [173:0] qout;
    integer i;
    begin
      get_state_num = 8'd255;  // Default: invalid
      for (i = 0; i < 174; i = i + 1) begin
        if (qout[i]) get_state_num = i[7:0];
      end
    end
  endfunction

  // Wires for easy monitoring
  wire [15:0] PC = uut.cpu.pc_out;
  wire [15:0] IR = uut.cpu.ir_out;
  wire [15:0] AC = uut.cpu.ac_out;
  wire [15:0] AR = uut.cpu.ar_out;
  wire [15:0] X = uut.cpu.x_out;
  wire [15:0] Y = uut.cpu.y_out;
  wire [15:0] SP = uut.cpu.sp_out;
  wire [3:0]  FLAGS = uut.cpu.flags_out;
  wire [15:0] SEU = uut.cpu.seu_out;
  wire [15:0] ALU_OUT = uut.cpu.outbus_alu;
  
  // Control Unit
  wire [173:0] STATE = uut.cpu.cu.qout;  // FF States (One-Hot) - 174 states (S0-S173)
  wire [122:0] CTRL  = uut.cpu.c;        // Control signals - 123 signals
  
  // Memory
  wire [15:0] MEM_ADDR = uut.address;
  wire [15:0] MEM_DIN  = uut.mem_in_cpu_out;
  wire [15:0] MEM_DOUT = uut.mem_out_cpu_in;
  wire MEM_RD = uut.read;
  wire MEM_WR = uut.write;

  // VCD Dump
  initial begin
    $dumpfile("soc_tb2.vcd");
    $dumpvars(0, SoC_tb2);
  end

  // Monitor - Display every cycle
  integer cycle_count;
  initial cycle_count = 0;

  // Counters for checks
  integer test_passed, test_failed;
  initial begin
    test_passed = 0;
    test_failed = 0;
  end

  // Task for verification
  task automatic check(
    input [159:0] test_name,  // 20 characters
    input [15:0] actual,
    input [15:0] expected
  );
    begin
      if (actual === expected) begin
        $display("[PASS] %s: got %04h (expected %04h)", test_name, actual, expected);
        test_passed = test_passed + 1;
      end else begin
        $display("[FAIL] %s: got %04h (expected %04h)", test_name, actual, expected);
        test_failed = test_failed + 1;
      end
    end
  endtask

  task automatic check_mem(
    input [8:0] addr,
    input [15:0] expected
  );
    begin
      if (uut.memory.mem[addr] === expected) begin
        $display("[PASS] Mem[%03h] = %04h (expected %04h)", addr, uut.memory.mem[addr], expected);
        test_passed = test_passed + 1;
      end else begin
        $display("[FAIL] Mem[%03h] = %04h (expected %04h)", addr, uut.memory.mem[addr], expected);
        test_failed = test_failed + 1;
      end
    end
  endtask

  // Main test
  initial begin

    do_reset(3);

    // START CPU
    $display("\nCPU has started...\n");
    cpu_start();

    // Execution - wait for HLT
    wait_finish(10000);  // max 10000 cycles for all instructions

    #500;
    $finish;
  end

endmodule
