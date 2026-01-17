module SoC( // System on Chip
  input start,
  input clk, rst_b,
  output finish
);

  // Memory - CPU
  wire read, write;
  wire [15:0] mem_out_cpu_in, mem_in_cpu_out;
  wire [15:0] address;

  // Input_Unit - CPU
  wire inp_ack, inp_req;
  wire [15:0] inp_data;

  // Output_Unit - CPU
  wire out_ack, out_req;
  wire [15:0] out_data;

  CPU cpu(
    .clk(clk),
    .rst_b(rst_b),
    .start(start),
    .inp_ack(inp_ack),
    .inp_data(inp_data),
    .mem_in(mem_out_cpu_in),
    .out_ack(out_ack),

    .out_data(out_data),
    .out_req(out_req),
    .read(read),
    .write(write),
    .mem_out(mem_in_cpu_out),
    .address(address),
    .inp_req(inp_req),
    .finish(finish)
  );

  memory_512x16 #(
    .INIT_FILE("program_asip_mul.hex")
  ) memory (
      .clk(clk),
      .rst_b(rst_b),
      .write(write),
      .read(read),
      .addr(address[8:0]),
      .din(mem_in_cpu_out),
      .dout(mem_out_cpu_in)
  );

  input_unit #() input_unit (
      .clk(clk),
      .rst_b(rst_b),
      .inp_req(inp_req),
      .inp_data(inp_data),
      .inp_ack(inp_ack)
  );

  output_unit #() output_unit (
      .clk(clk),
      .rst_b(rst_b),
      .out_req(out_req),
      .out_data(out_data),
      .out_ack(out_ack)
  );

endmodule

