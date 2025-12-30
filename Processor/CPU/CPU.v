module CPU(
  input clk, rst_b,
  input start,
  input inp_ack,
  input [15:0] inp_data,
  input [15:0] mem_in,
  input out_ack,
  
  output [15:0] out_data,
  output out_req,
  output read, write,
  output [15:0] mem_out,
  output [15:0] address,
  output inp_req,
  output finish
);

  // ALU wires
  wire [15:0] outbus_alu;
  wire finish_alu, negative_alu, zero_alu, carry_alu, overflow_alu;

  // Control_Unit_CPU wires
  wire [15:0] c;
  wire finish_cu;

  // SignExtendUnit
  wire [15:0] seu_out;
  wire [1:0]  seu_controller_out;

  // Registers
  wire [15:0] ac_out, ar_out, ir_out, pc_out, sp_out, x_out, y_out;

  // FLAGS register
  wire [3:0] flags_out;

  // muxes wires
  wire [15:0] mux2s_out, mux_registers_out;

  mux_3s #(16) mux_registers_3s(
    .d0(x_out),
    .d1(y_out),
    .d2({16'b0}), // z, t, ...
    .d3({16'b0}),
    .d4({16'b0}),
    .d5({16'b0}),
    .d6({16'b0}),
    .d7({16'b0}),
    .sel({
      1'b0, // | ~c[12]
      1'b0, // | ~c[12]
      1'b0 | ir_out[9] | c[13] // | ~c[12]
    }),
    .o(mux_registers_out)
  );

  mux_2s #(16) mux_registers_2s(
    .d0(mux_registers_out),
    .d1(ac_out),
    .d2(seu_out),
    .d3({16'b0}),
    .sel({
      1'b0 | c[8], // 1'b0 = ~c[6] | ~c[7] | ~c[12] | ~c[13]
      1'b0 | c[15]  // 1'b0 = ~c[6] | ~c[7] | ~c[12] | ~c[13]
    }),
    .o(mux2s_out)
  );

  ALU alu(
    .clk(clk), // 1001
    .rst_b(rst_b),
    .start(c[6] | c[7]),      
    .s({1'b0, 1'b0, 1'b0, 1'b0}),  // default adunare
    .inbus(mux2s_out),                 // input  [15:0]
    .outbus(outbus_alu),      // output [15:0]
    .finish(finish_alu),      // output 1 bit
    .negative(negative_alu),  // output 1 bit
    .zero(zero_alu),          // output 1 bit
    .carry(carry_alu),        // output 1 bit
    .overflow(overflow_alu)   // output 1 bit
  );

  Control_Unit_CPU cu(
    .clk(clk),
    .rst_b(rst_b),
    .op(ir_out[15:10]),                    // input  [5:0]
    .ra(ir_out[9]),                    // input  1 bit
    .start(start),                 // input  1 bit
    .ack_alu(finish_alu),               // input  1 bit
    .finish(finish_cu),       // output  1 bit
    .c(c)                     // output [15:0]
  );

  SEU_Controller seu_controller(
    .opcode(ir_out[15:10]),
    .selector(seu_controller_out)
  );

  SignExtendUnit seu(
    .imm(ir_out[8:0]),                   // input  [8:0]
    .sel(seu_controller_out),                   // input  [1:0]
    .out(seu_out)             // output reg [15:0]
  );

  wire [15:0] mux_ac_out;
  mux_2s #(16) mux_ac(
    .d0(mem_in),
    .d1(outbus_alu),
    .d2(seu_out),
    .d3({16'b0}),
    .sel({
      1'b0 | c[0],
      1'b0 | c[0]
    }),
    .o(mux_ac_out)
  );

  AC ac(
    .clk(clk),
    .rst_b(rst_b),
    .en(c[0] | c[10]),                    // input 1 bit
    .in(mux_ac_out),                    // input [15:0]
    .out(ac_out)              // output[15:0]
  );
  
  wire [15:0] mux_ar_out;
  mux_3s #(16) mux_ar(
    .d0(pc_out),      // 000: FETCH - încarcă adresa din PC
    .d1(sp_out),      // 001: PUSH/POP - adresa stack
    .d2(seu_out),     // 010: Adresare directă
    .d3(outbus_alu),  // 011: Adresare indexată
    .d4({16'b0}),  
    .d5({16'b1}),  // nop
    .d6({16'b1}),  // nop
    .d7({16'b1}),  // nop
    .sel({
      1'b0 | c[0],                         // s[2]
      1'b0 | c[3] | c[9] | c[11] | c[14],  // s[1]
      1'b0 | c[9] | c[14]                  // s[0]
    }),     // sel urile cresc proportional cu numarul de instructiuni
    .o(mux_ar_out)
  );

  AR ar(
    .clk(clk),
    .rst_b(rst_b),
    .en(c[0] | c[1] | c[3] | c[9] | c[11] | c[14]),    // input 1 bit
    .in(mux_ar_out),                    // input [15:0]
    .out(ar_out)              // output[15:0]
  );

  FLAGS flags(
    .clk(clk),
    .rst_b(rst_b),
    .en(1'b1),                    // input 1 bit
    .in({negative_alu, zero_alu, carry_alu, overflow_alu}),     // input [3:0]
    .out(flags_out)           // output[3:0]
  );

  IR ir(
    .clk(clk),
    .rst_b(rst_b),
    .en(c[2]),                    // input 1 bit
    .in(mem_in),                    // input [15:0]
    .out(ir_out)              // output[15:0]
  );

  wire [15:0] mux_pc_out;
  mux_2s #(16) mux_pc(
    .d0(seu_out), //00
    .d1({16'b0}), //01
    .d2({16'b1}), //nop
    .d3({16'b1}), //nop
    .sel({
      1'b0,
      1'b0 | c[0]
    }), 
    .o(mux_pc_out)
  );

  PC pc(
    .clk(clk),
    .rst_b(rst_b),
    .ld(c[0]),                   // input 1 bit
    .inc(c[2]),                  // input 1 bit
    .in(mux_pc_out),                   // input [15:0]
    .out(pc_out)             // output[15:0]
  );
  
  SP sp(
    .clk(clk),
    .rst_b(rst_b),
    .ld(c[0]),                   // input 1 bit
    .inc(1'b0),                  // input 1 bit
    .dec(1'b0),                  // input 1 bit
    .in({16'd512}),                   // input [15:0]
    .out(sp_out)             // output[15:0]
  );

  wire [15:0] mux_x_out;
  mux_3s #(16) mux_x(
    .d0(mux_registers_out), // de la alt mux
    .d1(seu_out),
    .d2(mem_in),  //010
    .d3({16'b0}), //011
    .d4(ac_out), 
    .d5({16'b1}), //nop 
    .d6({16'b1}), //nop
    .d7({16'b1}), //nop
    .sel({
      1'b0,
      1'b0 | c[0] | c[4],
      1'b0 | c[0]
    }),
    .o(mux_x_out)
  );

  X x(
    .clk(clk),
    .rst_b(rst_b),
    .en(c[0] | c[4]),                   // input 1 bit
    .in(mux_x_out),                   // input [15:0]
    .out(x_out)              // output[15:0]
  );


  wire [15:0] mux_y_out;
  mux_3s #(16) mux_y(
    .d0(mux_registers_out), // de la alt mux
    .d1(seu_out),
    .d2(mem_in),  //010
    .d3({16'b0}), //011
    .d4(ac_out), 
    .d5({16'b1}), //nop 
    .d6({16'b1}), //nop
    .d7({16'b1}), //nop
    .sel({
      1'b0,
      1'b0 | c[0] | c[5],
      1'b0 | c[0]
    }),
    .o(mux_y_out)
  );

  Y y(
    .clk(clk),
    .rst_b(rst_b),
    .en(c[0] | c[5]),                  // input 1 bit
    .in(mux_y_out),              // input [15:0]
    .out(y_out)             // output[15:0]
  );

  assign out_data = 16'b0;
  assign out_req = 1'b0;
  assign inp_req = 1'b0;
  assign read = c[1] | c[3] | c[9];
  assign write = c[12] | c[13] | c[15];
  assign mem_out = mux2s_out;
  assign address = ar_out;
  assign finish = finish_cu;
  
endmodule