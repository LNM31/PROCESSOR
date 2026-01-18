//`include "../Registers/ffd.v"
module Control_Unit(
  input clk, rst_b,
  input [1:0] s,
  input start, q0, q_1, a_16,
  input [3:0] cnt,
  output [10:0] c,
  output finish
);

    wire [3:0] qout;
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
