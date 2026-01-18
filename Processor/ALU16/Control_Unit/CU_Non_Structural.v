module Control_Unit(
  input clk,rst_b,
  input [1:0]s,
  input start,q0,q_1,a_8, //q_neg1 e q[-1],begin e start
  input [2:0]cnt,
  output reg [10:0]c,
  output reg finish
);
  localparam S0_ST=4'd0;
  localparam S1_ST=4'd1;
  localparam S2_ST=4'd2;
  localparam S3_ST=4'd3;
  localparam S4_ST=4'd4;
  localparam S5_ST=4'd5;
  localparam S6_ST=4'd6;
  localparam S7_ST=4'd7;
  localparam S8_ST=4'd8;
  localparam S9_ST=4'd9;
  localparam S10_ST=4'd10;
  localparam S11_ST=4'd11;
  localparam S12_ST=4'd12;
  localparam S13_ST=4'd13;
  localparam S14_ST=4'd14;

  
  reg[3:0] st,st_next;
  
  always@(posedge clk)
  begin
    case(st)
      S0_ST:
        if(start==1) st_next=S1_ST;
        else st_next=S0_ST;
      S1_ST: st_next=S2_ST;
      S2_ST:
        if(s[1]==0) st_next=S3_ST;
        else if(s==2'b10) st_next=S9_ST;
        else if(s==2'b11) st_next=S11_ST;
      S3_ST:
        if(s==2'b00) st_next=S4_ST;
        else if(s==2'b01) st_next=S5_ST;
      S4_ST:
        if(s==2'b00) st_next=S6_ST;
        else if(s==2'b10) st_next=S7_ST;
        else if(s==2'b11) st_next=S12_ST;
      S5_ST:
        if(s==2'b01) st_next=S6_ST;
        else if(s==2'b10) st_next=S7_ST;
        else if(s==2'b11) st_next=S12_ST;
      S6_ST:
        if(s[1]==0) st_next=S0_ST;
        else if(s[1]==1) st_next=S10_ST;
      S7_ST: 
        if(cnt==3'b111) st_next=S6_ST;
        else st_next=S8_ST;
      S8_ST:
        if(s==2'b10) st_next=S9_ST;
        else if(s==2'b11)
            begin
                if(a_8==1'b1) st_next=S4_ST;
                else if(a_8==1'b0) st_next=S5_ST;
            end
      S9_ST:
        if({q0,q_1}==2'b01) st_next=S4_ST;
        else if({q0,q_1}==2'b10) st_next=S5_ST;
        else st_next=S7_ST;
      S10_ST: st_next=S0_ST;
      S11_ST:
        if(a_8==1'b1) st_next=S4_ST;
        else if(a_8==1'b0) st_next=S5_ST;
      S12_ST:
        if(cnt==3'b111)
          begin
            if(a_8==1'b1) st_next=S14_ST;
            else if(a_8==1'b0) st_next=S6_ST;
          end
        else st_next=S13_ST;
      S13_ST: st_next=S8_ST;
      S14_ST: st_next=S6_ST;
    endcase
  end

  initial begin
    c=11'd0;
    finish=0;
  end
  always@(posedge clk)
  begin
    c=11'd0; //toate 0;
    finish=0;
    case(st)
      S0_ST:
        if(start==1) c=11'd1;//c0
        else begin c=11'd0; finish=0; end
      S1_ST: c=11'd2; //c1
      S2_ST:
        if(s[1]==0) c=11'd4; //c2
        else if(s==2'b10) c=11'd0; //c=0
        else if(s==2'b11) c=11'd512; //c9
      S3_ST:
        if(s==2'b00) c=11'd8; //c3
        else if(s==2'b01) c=11'd24; //c3,c4
      S4_ST:
        if(s==2'b00) c=11'd32; //c5
        else if(s==2'b10) c=11'd64; //c6
        else if(s==2'b11) c=11'd1024; //c10
      S5_ST:
        if(s==2'b01) c=11'd32; //c5
        else if(s==2'b10) c=11'd64; //c6
        else if(s==2'b11) c=11'd1024; //c10
      S6_ST:
        if(s[1]==0) begin c=11'd0; finish=1; end
        else if(s[1]==1) c=11'd256; //c8
      S7_ST: 
        if(cnt==3'b111) c=11'd32; //c5
        else c=11'd128; //c7
      S8_ST:
        if(s==2'b10) c=11'd0; //c=0
        else if(s==2'b11)
            begin
                if(a_8==1'b1) c=11'd8; //c3
                else if(a_8==1'b0) c=11'd24; //c3,c4
            end
      S9_ST:
        if({q0,q_1}==2'b01) c=11'd8; //c3
        else if({q0,q_1}==2'b10) c=11'd24; //c3,c4
        else c=11'd64; //c6
      S10_ST: begin c=11'd0; finish=1; end
      S11_ST:
        if(a_8==1'b1) c=11'd8; //c3
        else if(a_8==1'b0) c=11'd24; //c3,c4
      S12_ST:
        if(cnt==3'b111)
          begin
            if(a_8==1'b1) c=11'd8; //c3
            else if(a_8==1'b0) c=11'd32; //c5
          end
        else c=11'd512; //c9
      S13_ST: c=11'd128; //c7
      S14_ST: c=11'd32; //c5
    endcase
  end

  always @(posedge clk or negedge rst_b)
  begin
    if(!rst_b) st <= S0_ST;
    else st <= st_next;
  end
endmodule
