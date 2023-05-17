module ambm (
    input [26:0] in_a,
    input [26:0] in_b,
    output reg [26:0] result
);

reg [26:0] max_val;
reg [26:0] min_val;
reg [26:0] diff;
reg [26:0] sum;

// Set alpha to 1
localparam [26:0] alpha = 27'b0000000000001000000000000000;

always @(*) begin
    // Find max and min of inputs
    if (in_a > in_b) begin
        max_val = in_a;
        min_val = in_b;
    end else begin
        max_val = in_b;
        min_val = in_a;
    end


    // Calculate sum (alpha * max + beta * min) using shifting
    // alpha = 1, so alpha * max = max
    // beta = 1/4, so beta * min = min >> 2
    sum = max_val + (min_val >> 2);

    result = sum;

end

// non_restoring_sqrt sqrt_inst (
//     .square_distance(product),
//     .sqrt_distance(temp_result)
// );


endmodule


