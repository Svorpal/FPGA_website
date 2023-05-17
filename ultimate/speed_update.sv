module speed_update(
    input signed [26:0] xpos_avg,
    input signed [26:0] ypos_avg,
    input signed [26:0] xvel_avg,
    input signed [26:0] yvel_avg,
    input signed [26:0] x,
    input signed [26:0] y,
    input signed [26:0] ori_vx,
    input signed [26:0] ori_vy,
    input signed [26:0] neighboring_boids_val,
    input signed [26:0] centeringfactor,
    input signed [26:0] matchingfactor,
    output signed [26:0] update_vx,
    output signed [26:0] update_vy
);
    logic signed [26:0] xpos_avg_;
    logic signed [26:0] ypos_avg_;
    logic signed [26:0] xvel_avg_;
    logic signed [26:0] yvel_avg_;
    logic signed [26:0] mult1_out,mult2_out,mult3_out,mult4_out;
	 logic signed [26:0] mult1_in,mult2_in,mult3_in,mult4_in;
	
	//logic signed [26:0] mult5_out,mult6_out,mult7_out,mult8_out;
	
	signed_mult mult5(
        .a(xpos_avg),
        .b(neighboring_boids_val),
        .out(xpos_avg_)
    );
	signed_mult mult6(
        .a(ypos_avg),
        .b(neighboring_boids_val),
        .out(ypos_avg_)
    );
	signed_mult mult7(
        .a(xvel_avg),
        .b(neighboring_boids_val),
        .out(xvel_avg_)
    );
	signed_mult mult8(
        .a(yvel_avg),
        .b(neighboring_boids_val),
        .out(yvel_avg_)
    );
    
    //assign xpos_avg_ = ( (xpos_avg) >> neighboring_boids_val );
    //assign ypos_avg_ = ( (ypos_avg) >> neighboring_boids_val );
    //assign xvel_avg_ = ( (xvel_avg) >> neighboring_boids_val );
    //assign yvel_avg_ = ( (yvel_avg) >> neighboring_boids_val );
    assign mult1_in = xpos_avg_ - x;
	 assign mult2_in = xvel_avg_ - ori_vx;
	 assign mult3_in = ypos_avg_ - y;
	 assign mult4_in = yvel_avg_ - ori_vy;
	 
    signed_mult mult1(
        .a(mult1_in),
        .b(centeringfactor),
        .out(mult1_out)
    );
    signed_mult mult2(
        .a(mult2_in),
        .b(matchingfactor),
        .out(mult2_out)
    );
    signed_mult mult3(
        .a(mult3_in),
        .b(centeringfactor),
        .out(mult3_out)
    );
    signed_mult mult4(
        .a(mult4_in),
        .b(matchingfactor),
        .out(mult4_out)
    );
    
    // Add the centering/matching contributions to velocity
    assign update_vx = ori_vx + mult1_out + mult2_out;                      
    assign update_vy = ori_vy + mult3_out + mult4_out;
endmodule