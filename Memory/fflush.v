module tb;

reg [15:0] input_data;
integer result;

initial begin
    $write("Introdu valoarea: ");
    $fflush();
    result = $fscanf(32'h8000_0000, "%d", input_data);
    $display("Ai introdus: %d (0x%h)", input_data, input_data);
    $display("fscanf returnat: %d", result);
    $finish;
end

endmodule