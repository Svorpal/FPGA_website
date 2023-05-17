//`include "divider.v"
//`include "ambm.v"

module boids_update#(parameter NUM_OF_BOIDS = 10)(
    input clk,
    input reset,
    input [12:0] current_boids_num,
    input [(NUM_OF_BOIDS<<2)-1:0] [26:0] boids_attr_array,
    input enable,
    output logic finish,
    output logic signed  [26:0]pos_x, //12 bit integer, 15 fraction
    output logic signed  [26:0]pos_y,
    output logic signed  [26:0]vel_x,
    output logic signed  [26:0]vel_y
);

//addtional local vars
logic [7:0] counter;
logic [3:0] curr_state;
logic [3:0] next_state;

//localparams
localparam INIT = 4'd0;
localparam BOIDS_NUM_CHECK = 4'd1;
localparam CALC = 4'd2;
localparam SPD_UPDATE = 4'd3;
localparam BOUND_CHK = 4'd4;
localparam SPEED_LIMIT_CHK = 4'd5;
localparam POS_UPDATE = 4'd6;
localparam FINISH = 4'd7;

//? Define the turn factor for adjusting the boid's direction when near an edge
localparam signed [26:0] turnfactor = {12'sd0, 15'sb001100110011001};//(0.2);
//? Define the visual range for boids to perceive other boids within this distance
localparam signed [26:0] visualRange = {12'sd40, 15'sd0};//(40);
//? Define the protected range for boids to maintain a safe distance from other boids
localparam signed [26:0] protectedRange = {12'sd8, 15'sd0};//(8);
//? Define the centering factor for how strongly boids try to match the position of other boids
localparam signed [26:0] centeringfactor = {12'sd0, 15'sb000000000010000};//(0.0005);
//? Define the avoidance factor for how strongly boids try to avoid other boids within the protected range
localparam signed [26:0] avoidfactor = {12'sd0, 15'sb000011001100110};//(0.05);
//? Define the matching factor for how strongly boids try to match the velocity of other boids
localparam signed [26:0] matchingfactor = {12'sd0, 15'sb000011001100110};//(0.05);
//? Define the maximum speed a boid can move
localparam signed [26:0] maxspeed = {12'sd6, 15'sd0};//(6);
//? Define the minimum speed a boid can move
localparam signed [26:0] minspeed = {12'sd3, 15'sd0};//(3);
//? Define the maximum bias value to change boid's direction
localparam signed [26:0] maxbias = {12'sd0, 15'sb000000101000111};//(0.01);
//? Define the increment value for bias adjustment
localparam signed [26:0] bias_increment = {12'sd0, 15'sb000000000000001};//(0.00004);
//? Define the initial bias value for adjusting the boid's direction
localparam signed [26:0] biasval = {12'sd0, 15'sb000000000100000};//(0.001);

localparam signed [26:0] up_bound = {12'sd0, 15'sd0};  //0
localparam signed [26:0] down_bound = {12'sd480, 15'sd0}; //480
localparam signed [26:0] left_bound = {12'sd0, 15'sd0};  //0
localparam signed [26:0] right_bound = {12'sd640, 15'sd0}; //640

logic signed [26:0] xpos_avg = 0;
logic signed [26:0] ypos_avg = 0;
logic signed [26:0] xvel_avg = 0; 
logic signed [26:0] yvel_avg = 0;  
logic signed [26:0] close_dx = 0; 
logic signed [26:0] close_dy = 0;
logic[26:0] neighboring_boids = 0;
logic signed [26:0] neighboring_boids_val = 0;

logic[NUM_OF_BOIDS-1:0] [26:0] local_boids_attr_array; 
logic signed [26:0] x;
logic signed [26:0] y;
logic signed [26:0] vx;
logic signed [26:0] vy;
logic signed [26:0] x_o ;
logic signed [26:0] y_o ;
logic signed [26:0] vx_o;
logic signed [26:0] vy_o;
logic signed [26:0] bound_chk_vx;
logic signed [26:0] bound_chk_vy;
logic signed [26:0] update_vx;
logic signed [26:0] update_vy;

// Compute differences in x and y coordinates
logic signed [26:0] dx;
logic signed [26:0] dy;
logic signed [26:0] dx_out;
logic signed [26:0] dy_out;
logic signed [26:0] protectedRange_out;
logic signed [26:0] visualRange_out;
logic signed [26:0] squared_distance;
logic signed [26:0] speed;
logic signed [26:0] neg_visualRange;

logic signed [26:0] x_vx;
logic signed [26:0] y_vy;

assign x_vx = x+vx;
assign y_vy = y+vy;
assign dx = x - x_o;
assign dy = y - y_o;
assign squared_distance = dx_out + dy_out;
assign neg_visualRange = -visualRange;

signed_mult square_x(
    .out(dx_out),
    .a(dx),
    .b(dx)
);
signed_mult square_y(
    .out(dy_out),
    .a(dy),
    .b(dy)
);
signed_mult protectedRange_1(
    .out(protectedRange_out),
    .a(protectedRange),
    .b(protectedRange)
);
signed_mult visualRange_1(
    .out(visualRange_out),
    .a(visualRange),
    .b(visualRange)
);

logic signed [26:0] vx_square;
signed_mult vx_square_1(
    .a(vx),
    .b(vx),
    .out(vx_square)
);

logic signed [26:0] vy_square;
signed_mult vy_square_1(
    .a(vy),
    .b(vy),
    .out(vy_square)
);
assign speed = vx_square + vy_square;

logic signed [26:0] minspeed_square;
signed_mult min_speed_square(
    .a(minspeed),
    .b(minspeed),
    .out(minspeed_square)
);

logic signed [26:0] maxspeed_square;
signed_mult max_speed_square(
    .a(maxspeed),
    .b(maxspeed),
    .out(maxspeed_square)
);

logic signed [26:0] close_dx_avoidfactor;
logic signed [26:0] close_dy_avoidfactor;
signed_mult close_dx_avoidfactor_1(
    .out(close_dx_avoidfactor),
    .a(close_dx),
    .b(avoidfactor)
);
signed_mult close_dy_avoidfactor_1(
    .out(close_dy_avoidfactor),
    .a(close_dy),
    .b(avoidfactor)
);

speed_update spd_update(
    .xpos_avg(xpos_avg),
    .ypos_avg(ypos_avg),
    .xvel_avg(xvel_avg),
    .yvel_avg(yvel_avg),
    .x(x),
    .y(y),
    .ori_vx(vx),
    .ori_vy(vy),
    .neighboring_boids_val(neighboring_boids_val), 
    .centeringfactor(centeringfactor),
    .matchingfactor(matchingfactor),
    .update_vx(update_vx),
    .update_vy(update_vy)
);

neighboring_boids_LUT LUT_div(
    .neighboring_boids(neighboring_boids),
    .neighboring_boids_val_out(neighboring_boids_val)
);

bound_check bnd_chk(
    .x(x),
    .y(y),
    .turnfactor(turnfactor),
    .vx(vx),
    .vy(vy),
    .vx_chk(bound_chk_vx),
    .vy_chk(bound_chk_vy)
);

logic signed [26:0] sqrt_output;
ambm ambm1 (
    .in_a(vx_square),
    .in_b(vy_square),
    .result(sqrt_output)
);

logic o_complete;
logic o_overflow;
logic signed [26:0] o_quotient_out_vx, o_quotient_out_vy;
logic signed [26:0] o_quotient_out_vx_out, o_quotient_out_vy_out;

logic start;
logic signed [26:0]pos_vx, pos_vy;

assign pos_vx = (vx[26] == 1) ? -vx : vx;
assign pos_vy = (vy[26] == 1) ? -vy : vy;
/*
divider div_x(
    .i_dividend(pos_vx),//(sqrt_output),
    .i_divisor(sqrt_output), //vx
    .i_start(start),
    .i_clk(clk),
    .o_quotient_out(o_quotient_out_vx),
    .o_complete(o_complete),
    .o_overflow(o_overflow)
	);
	
divider div_y(
    .i_dividend(pos_vy),//(sqrt_output),
    .i_divisor(sqrt_output), //vy
    .i_start(start),
    .i_clk(clk),
    .o_quotient_out(o_quotient_out_vy),
    .o_complete(o_complete),
    .o_overflow(o_overflow)
);
assign o_quotient_out_vx_out = (vx[26] == 1) ? -o_quotient_out_vx: o_quotient_out_vx;
assign o_quotient_out_vy_out = (vy[26] == 1) ? -o_quotient_out_vy: o_quotient_out_vy;

logic signed [26:0] mult_out_1;
signed_mult mult5(
	.a(o_quotient_out_vx),
	.b(minspeed),
	.out(mult_out_1)
);

logic signed [26:0] mult_out_2;
signed_mult mult6(
	.a(o_quotient_out_vy),
	.b(minspeed),
	.out(mult_out_2)
);

logic signed [26:0] mult_out_3;
signed_mult mult7(
	.a(o_quotient_out_vx),
	.b(maxspeed),
	.out(mult_out_3)
);

logic signed [26:0] mult_out_4;
signed_mult mult8(
	.a(o_quotient_out_vx),
	.b(maxspeed),
	.out(mult_out_4)
);

*/
always@(posedge clk) begin
    if (reset) begin
        curr_state <= INIT;
    end 
    else begin
        curr_state <= next_state;
    end
end

always@(posedge clk)begin
    case(curr_state)
        INIT:begin
            counter <= 8'd0;
            close_dx <= 0;
            close_dy <= 0;
            x <= boids_attr_array[current_boids_num<<2];
            y <= boids_attr_array[(current_boids_num<<2)+1];
            vx <= boids_attr_array[(current_boids_num<<2)+2];
            vy <=boids_attr_array[(current_boids_num<<2)+3];
            neighboring_boids <= 0;
				xpos_avg <= 0;
				ypos_avg <= 0;
            xvel_avg <= 0;
            yvel_avg <= 0;
        end
        BOIDS_NUM_CHECK:begin
            if(counter != current_boids_num && counter < NUM_OF_BOIDS)begin
                x_o  <= boids_attr_array[(counter << 2)];
                y_o  <= boids_attr_array[(counter << 2)+1];
                vx_o <= boids_attr_array[(counter << 2)+2];
                vy_o <= boids_attr_array[(counter << 2)+3];
            end
            else begin
                counter <= counter + 1;
					 x_o  <= 0;
                y_o  <= 0;
                vx_o <= 0;
                vy_o <= 0;
            end
        end
        CALC:begin
            if(dx < visualRange && dy < visualRange && dx > neg_visualRange && dy > neg_visualRange)begin

                // Is squared distance less than the protected range?
                if ( squared_distance < protectedRange_out) begin

                    // If so, calculate difference in x/y-coordinates to nearfield boid
                    close_dx <= close_dx+dx;
                    close_dy <= close_dy+dy;
                end

                // If not in protected range, is the boid in the visual range?
                else if (squared_distance < visualRange_out)begin

                    // Add other boid's x/y-coord and x/y vel to accumulator variables
                    xpos_avg <= xpos_avg+x_o;
                    ypos_avg <= ypos_avg+y_o;
                    xvel_avg <= xvel_avg+vx_o;
                    yvel_avg <= yvel_avg+vy_o;

                    // Increment number of boids within visual range
                    neighboring_boids <= neighboring_boids+{12'd1, 15'd0};
                end
            end
            if(counter == (NUM_OF_BOIDS - 1))begin
                counter <= counter;
            end
            else begin
                counter <= counter + 1;
            end
        end
        SPD_UPDATE:begin
            // If there were any boids in the visual range . . .            
            if (neighboring_boids > {12'sd1,15'sd0}) begin
                vx <= update_vx ;//+ close_dx_avoidfactor;
                vy <= update_vy ;//+ close_dy_avoidfactor;
            end
            else begin
                vx <= vx ;//+ close_dx_avoidfactor;
                vy <= vy ;//+ close_dy_avoidfactor;
            end
        end
        BOUND_CHK:begin
             vx <= bound_chk_vx;
             vy <= bound_chk_vy;
        end
		  SPEED_LIMIT_CHK:begin
				if(speed < minspeed_square)begin
					start <= 1;
						 //if(vx[26] == 1)vx <= -mult_out_1;
						 //else vx <= mult_out_1;
						 //if(vy[26] == 1)vy <= -mult_out_2;
						 //else vy <= mult_out_2;
					vx <= vx + (vx >>> 3);//mult_out_1;
					vy <= vy + (vy >>> 3);//mult_out_2;
					
					end
				else if(speed > maxspeed_square)begin
					start <= 1;
						 //if(vx[26] == 1)vx <= -mult_out_3;
						 //else vx <= mult_out_3;
						 //if(vy[26] == 1)vy <= -mult_out_4;
						 //else vy <= mult_out_4;
					vx <= vx - (vx >>> 3);//mult_out_3;
					vy <= vy - (vy >>> 3);//mult_out_4;
				end
				else begin
					vx <= vx;
					vy <= vy;
				end
		  end
        POS_UPDATE:begin
				
				// update x
            if((x_vx) > right_bound)begin
                x <= x_vx - {12'sd15, 15'sd0};
                vx <= -vx;
            end
            else if ((x_vx) < left_bound)begin
                x <= x_vx + {12'sd15, 15'sd0};
                vx <= -vx;
            end
            else begin
                x <= x + vx;
                vx <= vx;
            end
				
				
				// update y
            if((y_vy) > down_bound)begin
                y <= y_vy - {12'sd15, 15'sd0};
                vy <= -vy;
            end
            else if((y_vy) < up_bound)begin
                y <= y_vy + {12'sd15, 15'sd0};
                vy <= -vy;
            end
            else begin
                y <= y + vy;
                vy <= vy;
            end
        end
        FINISH:begin
            pos_x <= x;
            pos_y <= y;
            vel_x <= vx;
            vel_y <= vy;
            counter <= 0;
        end
    endcase
end
always@(*)begin
    case(curr_state)
        INIT:begin
            next_state = BOIDS_NUM_CHECK;
        end
        BOIDS_NUM_CHECK:begin
            if(counter == (NUM_OF_BOIDS))begin
                next_state = SPD_UPDATE;
            end
            else if(counter != current_boids_num)begin
                next_state = CALC;
            end
            else begin
                next_state = BOIDS_NUM_CHECK;
            end
        end
        CALC:begin
            if(counter == (NUM_OF_BOIDS - 1))begin
                next_state = SPD_UPDATE;
            end
            else begin
                next_state = BOIDS_NUM_CHECK;
            end
        end
        SPD_UPDATE:begin
            next_state = BOUND_CHK;
        end
        BOUND_CHK:begin
            next_state = SPEED_LIMIT_CHK;
        end
		  SPEED_LIMIT_CHK:begin
				//if(speed < minspeed_square || speed > maxspeed_square)begin
				//	if(o_complete)next_state = POS_UPDATE;
				//	else next_state = SPEED_LIMIT_CHK;
				//end
				next_state = POS_UPDATE;
		  end
        POS_UPDATE:begin
            next_state = FINISH;
        end
        FINISH:begin
            if(enable) next_state = INIT;
            else next_state = FINISH;
        end
    
    endcase
end

always@(*)begin
    case(curr_state)
        INIT:begin
            finish = 0;
        end
        BOIDS_NUM_CHECK:begin
            finish = 0;
        end
        CALC:begin
            finish = 0;
        end
        SPD_UPDATE:begin
            finish = 0;
        end
        BOUND_CHK:begin
            finish = 0;
        end
        POS_UPDATE:begin
            finish = 0;
        end
        FINISH:begin
            finish = 1;
        end
        default:begin
            finish = 0;
        end
    endcase
end



endmodule

