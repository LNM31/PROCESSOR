//`include "../Combinational/muxes/mux_3s.v"
//`include "ffd.v"

module Q(
    input clk,rst_b,
    input a0,
    input [2:0]  sel,
    input [15:0] in,
    output[15:0] out,
    output q_1 

);
    wire[15:0]mux_wire;
    wire mux_q;
    //                  same        shfit_right_A  shift_left_A    load       shfit_right   rotate_right  rotate_left  
    mux_3s#(.w(1)) m_1(.d0(q_1    ), .d1(out[0] ), .d2(1'b0   ), .d3(1'b0  ), .d4(out[0] ), .d5(1'b0 ),   .d6(1'b0   ), .d7(1'b0), .sel(sel), .o(mux_q       ));

    mux_3s#(.w(1))  m0(.d0(out[0] ), .d1(out[1] ), .d2(1'b0   ), .d3(in[0] ), .d4(out[1] ), .d5(out[1] ), .d6(out[15]), .d7(1'b0), .sel(sel), .o(mux_wire[0] ));
    mux_3s#(.w(1))  m1(.d0(out[1] ), .d1(out[2] ), .d2(out[0] ), .d3(in[1] ), .d4(out[2] ), .d5(out[2] ), .d6(out[0] ), .d7(1'b0), .sel(sel), .o(mux_wire[1] ));
    mux_3s#(.w(1))  m2(.d0(out[2] ), .d1(out[3] ), .d2(out[1] ), .d3(in[2] ), .d4(out[3] ), .d5(out[3] ), .d6(out[1] ), .d7(1'b0), .sel(sel), .o(mux_wire[2] ));
    mux_3s#(.w(1))  m3(.d0(out[3] ), .d1(out[4] ), .d2(out[2] ), .d3(in[3] ), .d4(out[4] ), .d5(out[4] ), .d6(out[2] ), .d7(1'b0), .sel(sel), .o(mux_wire[3] ));
    mux_3s#(.w(1))  m4(.d0(out[4] ), .d1(out[5] ), .d2(out[3] ), .d3(in[4] ), .d4(out[5] ), .d5(out[5] ), .d6(out[3] ), .d7(1'b0), .sel(sel), .o(mux_wire[4] ));
    mux_3s#(.w(1))  m5(.d0(out[5] ), .d1(out[6] ), .d2(out[4] ), .d3(in[5] ), .d4(out[6] ), .d5(out[6] ), .d6(out[4] ), .d7(1'b0), .sel(sel), .o(mux_wire[5] ));
    mux_3s#(.w(1))  m6(.d0(out[6] ), .d1(out[7] ), .d2(out[5] ), .d3(in[6] ), .d4(out[7] ), .d5(out[7] ), .d6(out[5] ), .d7(1'b0), .sel(sel), .o(mux_wire[6] ));
    mux_3s#(.w(1))  m7(.d0(out[7] ), .d1(out[8] ), .d2(out[6] ), .d3(in[7] ), .d4(out[8] ), .d5(out[8] ), .d6(out[6] ), .d7(1'b0), .sel(sel), .o(mux_wire[7] ));
    mux_3s#(.w(1))  m8(.d0(out[8] ), .d1(out[9] ), .d2(out[7] ), .d3(in[8] ), .d4(out[9] ), .d5(out[9] ), .d6(out[7] ), .d7(1'b0), .sel(sel), .o(mux_wire[8] ));
    mux_3s#(.w(1))  m9(.d0(out[9] ), .d1(out[10]), .d2(out[8] ), .d3(in[9] ), .d4(out[10]), .d5(out[10]), .d6(out[8] ), .d7(1'b0), .sel(sel), .o(mux_wire[9] ));
    mux_3s#(.w(1)) m10(.d0(out[10]), .d1(out[11]), .d2(out[9] ), .d3(in[10]), .d4(out[11]), .d5(out[11]), .d6(out[9] ), .d7(1'b0), .sel(sel), .o(mux_wire[10]));
    mux_3s#(.w(1)) m11(.d0(out[11]), .d1(out[12]), .d2(out[10]), .d3(in[11]), .d4(out[12]), .d5(out[12]), .d6(out[10]), .d7(1'b0), .sel(sel), .o(mux_wire[11]));
    mux_3s#(.w(1)) m12(.d0(out[12]), .d1(out[13]), .d2(out[11]), .d3(in[12]), .d4(out[13]), .d5(out[13]), .d6(out[11]), .d7(1'b0), .sel(sel), .o(mux_wire[12]));
    mux_3s#(.w(1)) m13(.d0(out[13]), .d1(out[14]), .d2(out[12]), .d3(in[13]), .d4(out[14]), .d5(out[14]), .d6(out[12]), .d7(1'b0), .sel(sel), .o(mux_wire[13]));
    mux_3s#(.w(1)) m14(.d0(out[14]), .d1(out[15]), .d2(out[13]), .d3(in[14]), .d4(out[15]), .d5(out[15]), .d6(out[13]), .d7(1'b0), .sel(sel), .o(mux_wire[14]));
    mux_3s#(.w(1)) m15(.d0(out[15]), .d1(a0     ), .d2(out[14]), .d3(in[15]), .d4(out[15]), .d5(out[0] ), .d6(out[14]), .d7(1'b0), .sel(sel), .o(mux_wire[15]));

    ffd  f_1(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_q       ), .q(q_1    ));
    ffd   f0(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_wire[0] ), .q(out[0] ));
    ffd   f1(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_wire[1] ), .q(out[1] ));
    ffd   f2(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_wire[2] ), .q(out[2] ));
    ffd   f3(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_wire[3] ), .q(out[3] ));
    ffd   f4(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_wire[4] ), .q(out[4] ));
    ffd   f5(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_wire[5] ), .q(out[5] ));
    ffd   f6(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_wire[6] ), .q(out[6] ));
    ffd   f7(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_wire[7] ), .q(out[7] ));
    ffd   f8(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_wire[8] ), .q(out[8] ));
    ffd   f9(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_wire[9] ), .q(out[9] ));
    ffd  f10(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_wire[10]), .q(out[10]));
    ffd  f11(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_wire[11]), .q(out[11]));
    ffd  f12(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_wire[12]), .q(out[12]));
    ffd  f13(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_wire[13]), .q(out[13]));
    ffd  f14(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_wire[14]), .q(out[14]));
    ffd  f15(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_wire[15]), .q(out[15]));
    
endmodule