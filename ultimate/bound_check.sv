module bound_check(
    input signed [26:0] x,
    input signed [26:0] y,
    input signed [26:0] turnfactor,
    input signed [26:0] vx,
    input signed [26:0] vy,
    output signed [26:0] vx_chk,
    output signed [26:0] vy_chk
);

localparam signed [26:0]hitBottom = {12'sd380, 15'sd0};
localparam signed [26:0]hitTop = {12'sd100, 15'sd0};
localparam signed [26:0]hitLeft = {12'sd100, 15'sd0};
localparam signed [26:0]hitRight = {12'sd540, 15'sd0};

assign vy_chk = (y < hitTop) ? vy + turnfactor : (y > hitBottom) ? vy - turnfactor : vy;
assign vx_chk = (x < hitLeft) ? vx + turnfactor : (x > hitRight) ? vx - turnfactor : vx;

endmodule