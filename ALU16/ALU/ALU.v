`include"../Combinational/RCA/RCA.v"
`include"../Control_Unit/Control_Unit_Structural.v"
`include"../Registers/A.v"
`include"../Registers/Q.v"
`include"../Registers/M.v"
`include"../Combinational/muxes/mux_2s.v"
`include"../Registers/ffd.v"
`include"../Registers/counter.v"

module ALU (
    input clk,rst_b,start,
    input[1:0]s,
    input [15:0]inbus,
    output [15:0]outbus,
    output negative, zero, carry, overflow,
    output finish
);
    wire [15:0]q_out,m_out;
    wire [16:0]a_out;
    wire [3:0]cnt_out;
    wire q_1out;
    wire [10:0]c;
    wire [16:0]z;
    wire overfl;

    A regA(.clk(clk),.rst_b(rst_b),.q15(q_out[15]),.sel( {c[0]|c[2]|c[3]|c[9], c[0]|c[2]|c[3]|c[6]}),.in({17{c[0]}}&17'd0 | {17{c[2]}}&q_out | {17{c[3]}}&z),.out(a_out));
    Q regQ(.clk(clk),.rst_b(rst_b),.a0(a_out[0]),.sel( {c[1]|c[9]|c[10] , c[1]|c[6]|c[10]} ),.in({16{c[1]}}&inbus |  {q_out[15:1],(~a_out[16])}&{16{c[10]}} ),.out(q_out),.q_1(q_1out));
    M regM(.clk(clk),.rst_b(rst_b),.en(c[0]),.in(inbus),.out(m_out));
    RCA adder(.x({1'b0,m_out} ^ {17{c[4]}}),.y(a_out),.ci(c[4]),.z(z),.co(carry),.overflow(overfl));
    counter COUNT(.clk(clk),.rst_b(rst_b),.c_up(c[7]),.out(cnt_out));
    Control_Unit CU(.clk(clk),.rst_b(rst_b),.s(s),.start(start),.q0(q_out[0]),.q_1(q_1out),.a_16(a_out[16]),.cnt(cnt_out),.c(c),.finish(finish));
    
    assign outbus={1'b0,{16{c[5]}}}&a_out | {{16{c[8]}}}&q_out;

    assign negative = z[15];
    assign zero = ~(z[15] | z[14] | z[13] | z[12] | z[11] | z[10] | z[9] | z[8] | z[7] | z[6] | z[5] | z[4] | z[3] | z[2] | z[1] | z[0]);
    assign overflow = overfl & ~s[1] & c[3]; 

endmodule