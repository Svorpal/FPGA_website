`timescale 1ns / 1ps

module TestDiv;

    // Inputs
    reg [26:0] i_dividend;
    reg [26:0] i_divisor;
    reg i_start;
    reg i_clk;

    // Outputs
    wire [26:0] o_quotient_out;
    wire o_complete;
    wire o_overflow;

    // Instantiate the Unit Under Test (UUT)
    div DUT (
        .i_dividend(i_dividend),
        .i_divisor(i_divisor),
        .i_start(i_start),
        .i_clk(i_clk),
        .o_quotient_out(o_quotient_out),
        .o_complete(o_complete),
        .o_overflow(o_overflow)
    );

    reg [10:0] count;

    initial begin
        // Initialize Inputs
        i_dividend = 27'h1;
        i_divisor = 27'h1;
        i_start = 0;
        i_clk = 0;

        count <= 0;

        // Wait 100 ns for global reset to finish
        #100;

        // Add stimulus here
        forever #2 i_clk = ~i_clk;
    end

    always @(posedge i_clk) begin
        if (count == 47) begin
            count <= 0;
            i_start <= 1'b1;
        end else begin                
            count <= count + 1;
            i_start <= 1'b0;
        end
    end

    always @(count) begin
        if (count == 47) begin
            if (i_divisor > 27'h0FFFFFF) begin
                i_divisor <= 27'h1;
                i_dividend = (i_dividend << 1) + 3;
            end else
                i_divisor = (i_divisor << 1) + 1;
        end
    end

    // Monitor the inputs and outputs
    always @(posedge o_complete) begin
        // The following comments describe the expected outputs based on simple examples:
        // Example 1: i_dividend = 9, i_divisor = 3, expected output (o_quotient_out) = 3
        // Example 2: i_dividend = 16, i_divisor = 4, expected output (o_quotient_out) = 4
        // Example 3: i_dividend = 15, i_divisor = 5, expected output (o_quotient_out) = 3

        $display ("%b,%b,%b, %b", i_dividend, i_divisor, o_quotient_out, o_overflow);
    end

endmodule
