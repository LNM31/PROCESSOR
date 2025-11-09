`include "ffd.v"
module Control_Unit(
  input clk, rst_b,
  input [3:0] s,
  input start, q0, q_1, a_16, cmp_cnt_m4,
  input [3:0] cnt,
  output [18:0] c,
  output finish
);

    wire [4:0] qout;
    ffd f4(.clk(clk), .rst_b(rst_b), .en(1'b1),
    .d(
          // some code
     ),
     .q(qout[4]));

    // to be continued
    ffd f3(.clk(clk), .rst_b(rst_b), .en(1'b1),
    .d(
          ~qout[3]&~qout[2]&qout[1]&~qout[0]&s[1]&~s[0] 
        | ~qout[3]&~qout[2]&qout[1]&~qout[0]&s[1]&s[0]
        | ~qout[3]&qout[2]&~qout[1]&~qout[0]&s[1]&s[0] 
        | ~qout[3]&qout[2]&~qout[1]&qout[0]&s[1]&s[0] 
        | ~qout[3]&qout[2]&qout[1]&~qout[0]&s[1] 
        | ~qout[3]&qout[2]&qout[1]&qout[0]&~(cnt[3]&cnt[2]&cnt[1]&cnt[0]) 
        | qout[3]&~qout[2]&~qout[1]&~qout[0]&s[1]&~s[0]
        | qout[3]&qout[2]&~qout[1]&~qout[0]&a_16&cnt[3]&cnt[2]&cnt[1]&cnt[0]
        | qout[3]&qout[2]&~qout[1]&~qout[0]&~(cnt[3]&cnt[2]&cnt[1]&cnt[0]) 
        | qout[3]&qout[2]&~qout[1]&qout[0]
     ),
     .q(qout[3]));

    // to be continued
    ffd f2(.clk(clk), .rst_b(rst_b), .en(1'b1),
    .d(
          ~qout[3]&~qout[2]&qout[1]&qout[0]&~s[1]&~s[0]
        | ~qout[3]&~qout[2]&qout[1]&qout[0]&~s[1]&s[0]
        | ~qout[3]&qout[2]&~qout[1]&~qout[0]&~s[1]&~s[0]
        | ~qout[3]&qout[2]&~qout[1]&~qout[0]&s[1]&~s[0]
        | ~qout[3]&qout[2]&~qout[1]&~qout[0]&s[1]&s[0]
        | ~qout[3]&qout[2]&~qout[1]&qout[0]&~s[1]&s[0]
        | ~qout[3]&qout[2]&~qout[1]&qout[0]&s[1]&~s[0]
        | ~qout[3]&qout[2]&~qout[1]&qout[0]&s[1]&s[0]
        | ~qout[3]&qout[2]&qout[1]&qout[0]&cnt[3]&cnt[2]&cnt[1]&cnt[0]
        | qout[3]&~qout[2]&~qout[1]&~qout[0]&s[1]&s[0]&a_16
        | qout[3]&~qout[2]&~qout[1]&~qout[0]&s[1]&s[0]&~a_16
        | qout[3]&~qout[2]&~qout[1]&qout[0]&~q0&q_1
        | qout[3]&~qout[2]&~qout[1]&qout[0]&q0&~q_1
        | qout[3]&~qout[2]&qout[1]&qout[0]&a_16
        | qout[3]&~qout[2]&qout[1]&qout[0]&~a_16
        | qout[3]&qout[2]&~qout[1]&~qout[0]&a_16&cnt[3]&cnt[2]&cnt[1]&cnt[0]
        | qout[3]&qout[2]&~qout[1]&~qout[0]&~a_16&cnt[3]&cnt[2]&cnt[1]&cnt[0]  
        | qout[3]&qout[2]&~qout[1]&~qout[0]&~(cnt[3]&cnt[2]&cnt[1]&cnt[0])  
        | qout[3]&~qout[2]&~qout[1]&qout[0]&(q0~^q_1)
        | qout[3]&qout[2]&qout[1]&~qout[0]
    ),
    .q(qout[2]));

    // to be continued
    ffd f1(.clk(clk), .rst_b(rst_b), .en(1'b1),
    .d(
          ~qout[3]&~qout[2]&~qout[1]&qout[0]
        | ~qout[3]&~qout[2]&qout[1]&~qout[0]&~s[1]
        | ~qout[3]&~qout[2]&qout[1]&~qout[0]&s[1]&s[0]
        | ~qout[3]&qout[2]&~qout[1]&~qout[0]&~s[1]&~s[0]
        | ~qout[3]&qout[2]&~qout[1]&~qout[0]&s[1]&~s[0]
        | ~qout[3]&qout[2]&~qout[1]&qout[0]&~s[1]&s[0]
        | ~qout[3]&qout[2]&~qout[1]&qout[0]&s[1]&~s[0]
        | ~qout[3]&qout[2]&qout[1]&~qout[0]&s[1]
        | ~qout[3]&qout[2]&qout[1]&qout[0]&cnt[3]&cnt[2]&cnt[1]&cnt[0]
        | qout[3]&qout[2]&~qout[1]&~qout[0]&a_16&cnt[3]&cnt[2]&cnt[1]&cnt[0]
        | qout[3]&qout[2]&~qout[1]&~qout[0]&~a_16&cnt[3]&cnt[2]&cnt[1]&cnt[0]
        | qout[3]&~qout[2]&~qout[1]&qout[0]&(q0~^q_1)
        | qout[3]&qout[2]&qout[1]&~qout[0]
    ),
    .q(qout[1]));

    // to be continued
    ffd f0(.clk(clk), .rst_b(rst_b), .en(1'b1),
    .d(
          ~qout[3]&~qout[2]&~qout[1]&~qout[0]&start
        | ~qout[3]&~qout[2]&qout[1]&~qout[0]&~s[1]
        | ~qout[3]&~qout[2]&qout[1]&~qout[0]&s[1]&~s[0]
        | ~qout[3]&~qout[2]&qout[1]&~qout[0]&s[1]&s[0]
        | ~qout[3]&~qout[2]&qout[1]&qout[0]&~s[1]&s[0]
        | ~qout[3]&qout[2]&~qout[1]&~qout[0]&s[1]&~s[0]
        | ~qout[3]&qout[2]&~qout[1]&qout[0]&s[1]&~s[0]
        | qout[3]&~qout[2]&~qout[1]&~qout[0]&s[1]&~s[0]
        | qout[3]&~qout[2]&~qout[1]&~qout[0]&s[1]&s[0]&~a_16
        | qout[3]&~qout[2]&~qout[1]&qout[0]&q0&~q_1
        | qout[3]&~qout[2]&qout[1]&qout[0]&~a_16
        | qout[3]&~qout[2]&~qout[1]&qout[0]&(q0~^q_1)
        | qout[3]&qout[2]&~qout[1]&~qout[0]&~(cnt[3]&cnt[2]&cnt[1]&cnt[0]) 
    ),
    .q(qout[0]));

    
    assign c[18] = 1'b0; // easter egg: M + N = love
    assign c[17] = 1'b0; // :======D
    assign c[16] = 1'b0;
    assign c[15] = 1'b0;
    assign c[14] = 1'b0;
    assign c[13] = 1'b0;
    assign c[12] = 1'b0;
    assign c[11] = 1'b0;
    assign c[10]=~qout[3]&qout[2]&~qout[1]&~qout[0]&s[1]&s[0] | ~qout[3]&qout[2]&~qout[1]&qout[0]&s[1]&s[0];
    assign c[9]=~qout[3]&~qout[2]&qout[1]&~qout[0]&s[1]&s[0] | qout[3]&qout[2]&~qout[1]&~qout[0]&~(cnt[3]&cnt[2]&cnt[1]&cnt[0]);
    assign c[8]=~qout[3]&qout[2]&qout[1]&~qout[0]&s[1];
    assign c[7]=~qout[3]&qout[2]&qout[1]&qout[0]&~(cnt[3]&cnt[2]&cnt[1]&cnt[0]) | qout[3]&qout[2]&~qout[1]&qout[0];
    assign c[6]=~qout[3]&qout[2]&~qout[1]&~qout[0]&s[1]&~s[0] | ~qout[3]&qout[2]&~qout[1]&qout[0]&s[1]&~s[0] | qout[3]&~qout[2]&~qout[1]&qout[0]&(q0^~q_1);
    assign c[5]=~qout[3]&qout[2]&~qout[1]&~qout[0]&~s[1]&~s[0] 
                | ~qout[3]&qout[2]&~qout[1]&qout[0]&~s[1]&s[0] 
                | ~qout[3]&qout[2]&qout[1]&qout[0]&cnt[3]&cnt[2]&cnt[1]&cnt[0] 
                | qout[3]&qout[2]&~qout[1]&~qout[0]&~a_16&cnt[3]&cnt[2]&cnt[1]&cnt[0] 
                | qout[3]&qout[2]&qout[1]&~qout[0];
    assign c[4]=~qout[3]&~qout[2]&qout[1]&qout[0]&~s[1]&s[0] | qout[3]&~qout[2]&~qout[1]&~qout[0]&s[1]&s[0]&~a_16 | qout[3]&~qout[2]&~qout[1]&qout[0]&q0&~q_1 | qout[3]&~qout[2]&qout[1]&qout[0]&~a_16;
    assign c[3]=~qout[3]&~qout[2]&qout[1]&qout[0]&~s[1]&~s[0] 
                | ~qout[3]&~qout[2]&qout[1]&qout[0]&~s[1]&s[0] 
                | qout[3]&~qout[2]&~qout[1]&~qout[0]&s[1]&s[0]&a_16 
                | qout[3]&~qout[2]&~qout[1]&~qout[0]&s[1]&s[0]&~a_16 
                | qout[3]&~qout[2]&~qout[1]&qout[0]&~q0&q_1 
                | qout[3]&~qout[2]&~qout[1]&qout[0]&q0&~q_1 
                | qout[3]&~qout[2]&qout[1]&qout[0]&a_16 
                | qout[3]&~qout[2]&qout[1]&qout[0]&~a_16 
                | qout[3]&qout[2]&~qout[1]&~qout[0]&a_16&cnt[3]&cnt[2]&cnt[1]&cnt[0];
    assign c[2]=~qout[3]&~qout[2]&qout[1]&~qout[0]&~s[1];
    assign c[1]=~qout[3]&~qout[2]&~qout[1]&qout[0];
    assign c[0]=~qout[3]&~qout[2]&~qout[1]&~qout[0]&start;
    assign finish=~qout[3]&qout[2]&qout[1]&~qout[0]&~s[1] | qout[3]&~qout[2]&qout[1]&~qout[0];
    
endmodule
