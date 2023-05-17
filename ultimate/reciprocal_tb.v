`timescale 1ns/1ps

module reciprocal_tb;

    // Internal signals
    reg clk;
    reg rst;
    reg [26:0] data_in;
    wire [26:0] data_out;

    // Instantiate the DUT
    reciprocal dut (
        .clk(clk),
        .rst_n(rst),
        .num(data_in),
        .reciprocal(data_out)
    );

        // Clock generator
    always #1 clk = ~clk;

    // Reset generator
    initial begin
        rst = 1;
        #3 rst = 0;
    end

    // Test cases
    initial begin

        clk = 0;
        rst = 0;
        data_in = 0;
        #10;
        rst = 1;
        #10;
        rst = 0;

        // Test case: input = 2.0
        data_in = 27'b000000000010000000000000000;
        #10 $display("input = 2.0, data_out = %h, expected = 0.5", data_out);

        // Test case: input = 3.0
        data_in = 27'b000000000011000000000000000;
        #10 $display("input = 3.0, data_out = %h, expected = 0.333333", data_out);

        // Test case: input = 10.0
        data_in = 27'b000000001010000000000000000;
        #10 $display("input = 10.0, data_out = %h, expected = 0.1", data_out);

    end
endmodule
