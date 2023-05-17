`timescale 1ns/1ps

module ambv_tb;

    // Internal signals
    reg clk;
    reg rst;
    reg [26:0] in_a;
    reg [26:0] in_b;
    wire [26:0] result;

    // Instantiate the DUT
    ambv dut (
        .in_a(in_a),
        .in_b(in_b),
        .result(result)
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

        // Test case: in_a = 3.0, in_b = 3.0
        in_a = 27'b000000000011000000000000000;
        in_b = 27'b000000000011000000000000000;
        #10 $display("result = %h, expected = 3.75", result);

        // Test case: in_a = 3.0, in_b = 4.5
        in_a = 27'b000000000011000000000000000;
        in_b = 27'b000000000100100000000000000;
        #10 $display("result = %h, expected = 4.875", result);

        // Test case: in_a = 3.0, in_b = 6.0
        in_a = 27'b000000000011000000000000000;
        in_b = 27'b000000000110000000000000000;
        #10 $display("result = %h, expected = 6.0", result);

        // Test case: in_a = 4.5, in_b = 3.0
        in_a = 27'b000000000100100000000000000;
        in_b = 27'b000000000011000000000000000;
        #10 $display("result = %h, expected = 4.875", result);

        // Test case: in_a = 4.5, in_b = 4.5
        in_a = 27'b000000000100100000000000000;
        in_b = 27'b000000000100100000000000000;
        #10 $display("result = %h, expected = 6.0", result);

        // Test case: in_a = 4.5, in_b = 6.0
        in_a = 27'b000000000100100000000000000;
        in_b = 27'b000000000110000000000000000;
        #10 $display("result = %h, expected = 7.125", result);


    end
endmodule
