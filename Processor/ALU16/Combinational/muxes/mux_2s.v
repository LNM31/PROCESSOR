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