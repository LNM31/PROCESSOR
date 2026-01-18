//`include "ffd.v"
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
          ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & ~s[0] & ~cmp_cnt_m4             //10
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & s[0] & ~cmp_cnt_m4              //12
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & ~s[0] & ~cmp_cnt_m4            //14
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & s[0]                           //15
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & s[1] & ~s[0]                           //16
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & s[1] & s[0]                            //17
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & s[2] & ~s[1] & ~s[0]                           //18
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & s[2] & s[1] & ~s[0]                            //20
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & ~s[0] & ~cmp_cnt_m4             //48
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & s[0] & ~cmp_cnt_m4              //50
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & ~s[0] & ~cmp_cnt_m4            //52


     ),
     .q(qout[4]));

    // to be continued
    ffd f3(.clk(clk), .rst_b(rst_b), .en(1'b1),
    .d(
          ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & ~s[0]                                                 //4 - bun
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0]                                                  //5
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0]                                                 //6
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & s[0] & cmp_cnt_m4                                     //7
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & s[0] & ~cmp_cnt_m4                                    //8
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & ~s[0] & cmp_cnt_m4                                     //9
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & s[0] & cmp_cnt_m4                                      //11
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & ~s[0] & cmp_cnt_m4                                    //13
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0]                                                  //26
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & qout[0] & ~s[3] & ~s[2] & s[1] & s[0]                                                   //29
        | ~qout[4] & ~qout[3] & qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & ~s[0]                                                  //32 - rau
        | ~qout[4] & ~qout[3] & qout[2] & qout[1] & qout[0] & ~(cnt[3] & cnt[2] & cnt[1] & cnt[0])                                           //39 - bun
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & ~s[0]                                                 //40 - bun
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & s[0] & cmp_cnt_m4                                     //45
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & s[0] & ~cmp_cnt_m4                                    //46
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & ~s[0] & cmp_cnt_m4                                     //47
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & s[0] & cmp_cnt_m4                                      //49
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & ~s[0] & cmp_cnt_m4                                    //51
        | ~qout[4] & qout[3] & qout[2] & ~qout[1] & ~qout[0] & a_16 & cnt[3] & cnt[2] & cnt[1] & cnt[0]                                      //64
        | ~qout[4] & qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0] & ~a_16 & cnt[3] & cnt[2] & cnt[1] & cnt[0]       //65
        | ~qout[4] & qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~(cnt[3] & cnt[2] & cnt[1] & cnt[0])                                          //67
        | ~qout[4] & qout[3] & qout[2] & ~qout[1] & qout[0]                                                                                  //68
        | ~qout[4] & qout[3] & qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0]                                                    //69                             
        | ~qout[4] & qout[3] & qout[2] & qout[1] & qout[0]                                                                                   //71
        | qout[4] & ~qout[3] & ~qout[2] & ~qout[1] & ~qout[0]                                                                                //72
        | qout[4] & ~qout[3] & ~qout[2] & ~qout[1] & qout[0]                                                                                 //73
        | qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0]                                                                                 //74
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0]                                                 //81
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0]                                                  //82
     ),
     .q(qout[3]));

    // to be continued
    ffd f2(.clk(clk), .rst_b(rst_b), .en(1'b1),
    .d(
          ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & s[0] & ~cmp_cnt_m4                                    //8
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & s[1] & ~s[0]                                                  //16
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & s[1] & s[0]                                                   //17
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & s[2] & ~s[1] & ~s[0]                                                  //18
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & qout[0] & ~s[3] & ~s[2] & ~s[1] & ~s[0]                                                 //21
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & qout[0] & ~s[3] & ~s[2] & ~s[1] & s[0]                                                  //22
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & qout[0] & s[3] & s[2] & ~s[1] & s[0]                                                    //23
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & ~s[1] & ~s[0]                                                //24
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & ~s[0]                                                 //25 -bun
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0]                                                  //26
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & qout[0] & ~s[3] & ~s[2] & ~s[1] & s[0]                                                  //27
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & qout[0] & ~s[3] & ~s[2] & s[1] & ~s[0]                                                  //28 -bun
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & qout[0] & ~s[3] & ~s[2] & s[1] & s[0]                                                   //29
        | ~qout[4] & ~qout[3] & qout[2] & qout[1] & qout[0] & cnt[3] & cnt[2] & cnt[1] & cnt[0]                                              //38 - bun
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0] & a_16                                           //41
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0] & ~a_16                                          //42
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0] & a_16                                          //43
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0] & ~a_16                                         //44
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & s[0] & ~cmp_cnt_m4                                    //46
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & qout[0] & ~q0 & q_1                                                                     //53 - bun
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & qout[0] & q0 & ~q_1                                                                     //54 - bun
        | ~qout[4] & qout[3] & ~qout[2] & qout[1] & qout[0] & a_16                                                                           //62
        | ~qout[4] & qout[3] & ~qout[2] & qout[1] & qout[0] & ~a_16                                                                          //63
        | ~qout[4] & qout[3] & qout[2] & ~qout[1] & ~qout[0] & a_16 & cnt[3] & cnt[2] & cnt[1] & cnt[0]                                      //64
        | ~qout[4] & qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0] & ~a_16 & cnt[3] & cnt[2] & cnt[1] & cnt[0]      //66
        | ~qout[4] & qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~(cnt[3] & cnt[2] & cnt[1] & cnt[0])                                          //67
        | ~qout[4] & qout[3] & qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0]                                                   //70
        | qout[4] & ~qout[3] & ~qout[2] & qout[1] & qout[0] & s[3] & ~s[2] & ~s[1] & s[0]                                                    //75
        | qout[4] & ~qout[3] & qout[2] & ~qout[1] & ~qout[0]                                                                                 //77
        | qout[4] & ~qout[3] & qout[2] & ~qout[1] & qout[0]                                                                                  //78
        | qout[4] & ~qout[3] & qout[2] & qout[1] & ~qout[0]                                                                                  //79
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & qout[0] & (q0~^q_1)                                                                     //80 -bun
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0]                                                 //81
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0]                                                  //82          
    ),
    .q(qout[2]));

    // to be continued
    ffd f1(.clk(clk), .rst_b(rst_b), .en(1'b1),
    .d(
          ~qout[4] & ~qout[3] & ~qout[2] & ~qout[1] & qout[0]                                                                                //2
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & ~s[1]                                                        //3
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0]                                                  //5
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0]                                                 //6
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & s[0] & cmp_cnt_m4                                     //7
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & s[0] & ~cmp_cnt_m4                                    //8
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & ~s[0] & cmp_cnt_m4                                     //9
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & s[0] & cmp_cnt_m4                                      //11
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & ~s[0] & cmp_cnt_m4                                    //13
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & ~s[0] & ~cmp_cnt_m4                                   //14
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & s[0]                                                  //15
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & s[2] & ~s[1] & ~s[0]                                                  //18
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & s[2] & ~s[1] & s[0]                                                   //19
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & s[2] & s[1] & ~s[0]                                                   //20
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & ~s[1] & ~s[0]                                                //24
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & ~s[0]                                                 //25 - bun
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & qout[0] & ~s[3] & ~s[2] & ~s[1] & s[0]                                                  //27
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & qout[0] & ~s[3] & ~s[2] & s[1] & ~s[0]                                                  //28 - bun
        | ~qout[4] & ~qout[3] & qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & ~s[0]                                                  //32 - rau
        | ~qout[4] & ~qout[3] & qout[2] & qout[1] & qout[0] & cnt[3] & cnt[2] & cnt[1] & cnt[0]                                              //38 - bun
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & s[0] & cmp_cnt_m4                                     //45
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & s[0] & ~cmp_cnt_m4                                    //46
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & ~s[0] & cmp_cnt_m4                                     //47
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & s[0] & cmp_cnt_m4                                      //49
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & ~s[0] & cmp_cnt_m4                                    //51
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & ~s[0] & ~cmp_cnt_m4                                   //52
        | ~qout[4] & qout[3] & qout[2] & ~qout[1] & ~qout[0] & a_16 & cnt[3] & cnt[2] & cnt[1] & cnt[0]                                      //64
        | ~qout[4] & qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0] & ~a_16 & cnt[3] & cnt[2] & cnt[1] & cnt[0]       //65
        | ~qout[4] & qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0] & ~a_16 & cnt[3] & cnt[2] & cnt[1] & cnt[0]      //66
        | ~qout[4] & qout[3] & qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0]                                                    //69
        | ~qout[4] & qout[3] & qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0]                                                   //70
        | qout[4] & ~qout[3] & ~qout[2] & qout[1] & qout[0] & s[3] & ~s[2] & ~s[1] & s[0]                                                    //75
        | qout[4] & ~qout[3] & qout[2] & ~qout[1] & ~qout[0]                                                                                 //77
        | qout[4] & ~qout[3] & qout[2] & ~qout[1] & qout[0]                                                                                  //78
        | qout[4] & ~qout[3] & qout[2] & qout[1] & ~qout[0]                                                                                  //79
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & qout[0] & (q0~^q_1)                                                                     //80 - bun



        
    ),
    .q(qout[1]));

    // to be continued
    ffd f0(.clk(clk), .rst_b(rst_b), .en(1'b1),
    .d(
          ~qout[4] & ~qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & start                                                                       //1
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & ~s[1]                                                        //3
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & ~s[0]                                                 //4
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0]                                                  //5
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0]                                                 //6
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & s[0] & ~cmp_cnt_m4                                    //8
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & s[0] & ~cmp_cnt_m4                                     //12
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & s[0]                                                  //15
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & s[1] & s[0]                                                   //17
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & s[2] & ~s[1] & s[0]                                                   //19
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & s[2] & s[1] & ~s[0]                                                   //20
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & qout[0] & ~s[3] & ~s[2] & ~s[1] & s[0]                                                  //22
        | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & qout[0] & s[3] & s[2] & ~s[1] & s[0]                                                    //23
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & ~s[0]                                                 //25 - bun
        | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & qout[0] & ~s[3] & ~s[2] & s[1] & ~s[0]                                                  //28 - bun
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & ~s[0]                                                 //40 - bun
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0] & ~a_16                                          //42
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0] & ~a_16                                         //44
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & s[0] & ~cmp_cnt_m4                                    //46
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & s[0] & ~cmp_cnt_m4                                     //50
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & qout[0] & q0 & ~q_1                                                                     //54 - bun
        | ~qout[4] & qout[3] & ~qout[2] & qout[1] & qout[0] & ~a_16                                                                          //63
        | ~qout[4] & qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~(cnt[3] & cnt[2] & cnt[1] & cnt[0])                                          //67
        | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & qout[0] & (q0~^q_1)                                                                     //80 - bun
//poate 84



        
    ),
    .q(qout[0]));

    
    assign c[18]  =   ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & s[2] & ~s[1] & ~s[0];                                                 //18
    assign c[17]  =   ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & s[1] & s[0];                                                  //17
    assign c[16]  =   ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & s[1] & ~s[0];                                                 //16
    assign c[15]  =   ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & s[0]                                                  //15
                    | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & s[2] & s[1] & ~s[0];                                                  //20
    assign c[14]  =   ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & ~s[0] & ~cmp_cnt_m4                                   //14
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & ~s[0] & ~cmp_cnt_m4;                                  //52
    assign c[13]  =   ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & s[0] & ~cmp_cnt_m4                                     //12
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & s[0] & ~cmp_cnt_m4;                                    //50
    assign c[12]  =   ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & ~s[0] & ~cmp_cnt_m4                                    //10
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & ~s[0] & ~cmp_cnt_m4;                                   //48
    assign c[11]  =   ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & s[0] & ~cmp_cnt_m4                                    //8
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & s[0] & ~cmp_cnt_m4;                                   //46
    assign c[10]  =   ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0]                                                  //26
                    | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & qout[0] & ~s[3] & ~s[2] & s[1] & s[0]                                                   //29
                    | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0]                                                 //81
                    | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0];                                                 //82 
    assign c[9]   =   ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0]                                                  //5
                    | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0]                                                 //6
                    | ~qout[4] & qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~(cnt[3] & cnt[2] & cnt[1] & cnt[0]);                                         //67
    assign c[8]   =   ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & s[0] & cmp_cnt_m4                                     //7
                    | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & ~s[0] & cmp_cnt_m4                                     //9
                    | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & s[0] & cmp_cnt_m4                                      //11
                    | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & ~s[0] & cmp_cnt_m4                                    //13
                    | ~qout[4] & ~qout[3] & qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & ~s[0]                                                  //32- rau
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & s[0] & cmp_cnt_m4                                     //45
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & ~s[0] & cmp_cnt_m4                                     //47
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & s[0] & cmp_cnt_m4                                      //49
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & ~s[0] & cmp_cnt_m4                                    //51
                    | ~qout[4] & qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0] & ~a_16 & cnt[3] & cnt[2] & cnt[1] & cnt[0]       //65
                    | ~qout[4] & qout[3] & qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0];                                                   //69
    assign c[7]   =   ~qout[4] & ~qout[3] & qout[2] & qout[1] & qout[0] & ~(cnt[3] & cnt[2] & cnt[1] & cnt[0])                                           //39 - bun
                    | ~qout[4] & qout[3] & qout[2] & ~qout[1] & qout[0]                                                                                  //68                          
                    | ~qout[4] & qout[3] & qout[2] & qout[1] & qout[0]                                                                                   //71
                    | qout[4] & ~qout[3] & ~qout[2] & ~qout[1] & ~qout[0]                                                                                //72
                    | qout[4] & ~qout[3] & ~qout[2] & ~qout[1] & qout[0]                                                                                 //73
                    | qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0];                                                                                //74
    assign c[6]   =   ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & ~s[0]                                                 //25 - bun
                    | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & qout[0] & ~s[3] & ~s[2] & s[1] & ~s[0]                                                  //28 - bun
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & qout[0] & (q0~^q_1);                                                                    //80 - bun
    assign c[5]   =   ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & ~s[1] & ~s[0]                                                //24
                    | ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & qout[0] & ~s[3] & ~s[2] & ~s[1] & s[0]                                                  //27
                    | ~qout[4] & ~qout[3] & qout[2] & qout[1] & qout[0] & cnt[3] & cnt[2] & cnt[1] & cnt[0]                                              //38 - bun
                    | ~qout[4] & qout[3] & qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0] & ~a_16 & cnt[3] & cnt[2] & cnt[1] & cnt[0]      //66
                    | ~qout[4] & qout[3] & qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0]                                                   //70
                    | qout[4] & ~qout[3] & ~qout[2] & qout[1] & qout[0] & s[3] & ~s[2] & ~s[1] & s[0]                                                    //75
                    | qout[4] & ~qout[3] & qout[2] & ~qout[1] & ~qout[0]                                                                                 //77
                    | qout[4] & ~qout[3] & qout[2] & ~qout[1] & qout[0]                                                                                  //78
                    | qout[4] & ~qout[3] & qout[2] & qout[1] & ~qout[0];                                                                                 //79
    assign c[4]   =   ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & qout[0] & ~s[3] & ~s[2] & ~s[1] & s[0]                                                  //22
                    | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & qout[0] & s[3] & s[2] & ~s[1] & s[0]                                                    //23 
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0] & ~a_16                                          //42
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0] & ~a_16                                         //44
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & qout[0] & q0 & ~q_1                                                                     //54 - bun
                    | ~qout[4] & qout[3] & ~qout[2] & qout[1] & qout[0] & ~a_16;                                                                         //63
    assign c[3]   =   ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & qout[0] & ~s[3] & ~s[2] & ~s[1] & ~s[0]                                                 //21
                    | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & qout[0] & ~s[3] & ~s[2] & ~s[1] & s[0]                                                  //22
                    | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & qout[0] & s[3] & s[2] & ~s[1] & s[0]                                                    //23
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0] & a_16                                           //41
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0] & ~a_16                                          //42
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0] & a_16                                          //43
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0] & ~a_16                                         //44
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & qout[0] & ~q0 & q_1                                                                     //53 - bun
                    | ~qout[4] & qout[3] & ~qout[2] & ~qout[1] & qout[0] & q0 & ~q_1                                                                     //54 - bun
                    | ~qout[4] & qout[3] & ~qout[2] & qout[1] & qout[0] & a_16                                                                           //62
                    | ~qout[4] & qout[3] & ~qout[2] & qout[1] & qout[0] & ~a_16                                                                          //63
                    | ~qout[4] & qout[3] & qout[2] & ~qout[1] & ~qout[0] & a_16 & cnt[3] & cnt[2] & cnt[1] & cnt[0];                                     //64
    assign c[2]   =   ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & ~s[1]                                                        //3
                    | ~qout[4] & ~qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & s[2] & ~s[1] & s[0];                                                  //19 
    assign c[1]   =   ~qout[4] & ~qout[3] & ~qout[2] & ~qout[1] & qout[0];                                                                               //2
    assign c[0]   =   ~qout[4] & ~qout[3] & ~qout[2] & ~qout[1] & ~qout[0] & start;                                                                      //1
    assign finish =   ~qout[4] & ~qout[3] & qout[2] & ~qout[1] & qout[0] & s[3] & s[2] & ~s[1] & s[0]                                                    //30
                    | ~qout[4] & ~qout[3] & qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & ~s[1]                                                         //31
                    | ~qout[4] & ~qout[3] & qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & ~s[0]                                                  //33
                    | ~qout[4] & ~qout[3] & qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & s[0]                                                   //34
                    | ~qout[4] & ~qout[3] & qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & s[1] & ~s[0]                                                   //35
                    | ~qout[4] & ~qout[3] & qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & s[1] & s[0]                                                    //36
                    | ~qout[4] & ~qout[3] & qout[2] & qout[1] & ~qout[0] & s[3] & s[2] & ~s[1] & ~s[0]                                                   //37
                    | ~qout[4] & qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & ~s[0]                                                  //55 - bun
                    | ~qout[4] & qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & ~s[2] & s[1] & s[0]                                                   //56
                    | ~qout[4] & qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & ~s[1] & s[0]                                                   //58
                    | ~qout[4] & qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & ~s[0]                                                   //59
                    | ~qout[4] & qout[3] & ~qout[2] & qout[1] & ~qout[0] & ~s[3] & s[2] & s[1] & s[0]                                                    //60
                    | ~qout[4] & qout[3] & ~qout[2] & qout[1] & ~qout[0] & s[3] & ~s[2] & ~s[1] & ~s[0]                                                  //61
                    | qout[4] & ~qout[3] & ~qout[2] & qout[1] & qout[0] & s[3] & s[2] & s[1] & ~s[0];                                                    //76
    
endmodule
