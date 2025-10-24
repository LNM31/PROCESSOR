module mux_1s #(
    parameter w=4
)(
  input  [w-1:0]d0,d1,
  input  sel,
  output [w-1:0]o
);
    assign o=(sel==1'b0) ? d0:
             (sel==1'b1) ? d1:{w{1'bz}};

endmodule

module mux_2s #(
    parameter w=4
)(
  input  [w-1:0]d0,d1,d2,d3,
  input [1:0] sel,
  output [w-1:0]o
);
    assign o=(sel==2'b00) ? d0:
         (sel==2'b01) ? d1:
         (sel==2'b10) ? d2:
         (sel==2'b11) ? d3:{w{1'bz}};

endmodule

module mux_3s #(
    parameter w=4
)(
  input  [w-1:0]d0,d1,d2,d3,d4,d5,d6,d7,
  input [2:0] sel,
  output [w-1:0]o
);
    assign o=(sel==3'b000) ? d0:
         (sel==3'b001) ? d1:
         (sel==3'b010) ? d2:
         (sel==3'b011) ? d3:
         (sel==3'b100) ? d4:
         (sel==3'b101) ? d5:
         (sel==3'b110) ? d6:
         (sel==3'b111) ? d7:{w{1'bz}};

endmodule

module encoder #(
    parameter w=1
)(
    input [2**w-1:0]in,
    output reg [w-1:0] out
);
    integer i;
    always @(*) 
    begin
        out = {w{1'b0}}; // default
        for (i = 2**w - 1; i >= 0; i = i - 1) begin
            if (in[i]) begin
                out = i[w-1:0]; // tăiem doar w biți
            end
        end
    end
endmodule