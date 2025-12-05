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