module AR(
    input clk, rst_b,
    input en,
    input [15:0] in,
    output[15:0] out 

);
 
    ffd  f0(.clk(clk), .rst_b(rst_b), .en(en), .d(in[0] ), .q(out[0] ));
    ffd  f1(.clk(clk), .rst_b(rst_b), .en(en), .d(in[1] ), .q(out[1] ));
    ffd  f2(.clk(clk), .rst_b(rst_b), .en(en), .d(in[2] ), .q(out[2] ));
    ffd  f3(.clk(clk), .rst_b(rst_b), .en(en), .d(in[3] ), .q(out[3] ));
    ffd  f4(.clk(clk), .rst_b(rst_b), .en(en), .d(in[4] ), .q(out[4] ));
    ffd  f5(.clk(clk), .rst_b(rst_b), .en(en), .d(in[5] ), .q(out[5] ));
    ffd  f6(.clk(clk), .rst_b(rst_b), .en(en), .d(in[6] ), .q(out[6] ));
    ffd  f7(.clk(clk), .rst_b(rst_b), .en(en), .d(in[7] ), .q(out[7] ));
    ffd  f8(.clk(clk), .rst_b(rst_b), .en(en), .d(in[8] ), .q(out[8] ));
    ffd  f9(.clk(clk), .rst_b(rst_b), .en(en), .d(in[9] ), .q(out[9] ));
    ffd f10(.clk(clk), .rst_b(rst_b), .en(en), .d(in[10]), .q(out[10]));
    ffd f11(.clk(clk), .rst_b(rst_b), .en(en), .d(in[11]), .q(out[11]));
    ffd f12(.clk(clk), .rst_b(rst_b), .en(en), .d(in[12]), .q(out[12]));
    ffd f13(.clk(clk), .rst_b(rst_b), .en(en), .d(in[13]), .q(out[13]));
    ffd f14(.clk(clk), .rst_b(rst_b), .en(en), .d(in[14]), .q(out[14]));
    ffd f15(.clk(clk), .rst_b(rst_b), .en(en), .d(in[15]), .q(out[15]));
    
endmodule