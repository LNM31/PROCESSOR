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