module counter_tb;

    reg clk, rst_b, c_up;
    wire [3:0] out;

    counter c1 (
      .clk(clk),
      .rst_b(rst_b),
      .c_up(c_up),
      .out(out)
    );

    initial begin
      $dumpfile("dump.vcd");
      $dumpvars;
      clk = 0;
      rst_b = 1;
      c_up = 1;
      #10; rst_b = 0;
      #20; rst_b = 1;
    end

    integer i;
    initial begin
        for(i=0;i<100;i=i+1)
        begin
            #50; clk=~clk;
        end
        #50;
    end

    initial begin
        $display("Time\tout");
        $monitor("%0t\t%b", $time, out);
    end
endmodule