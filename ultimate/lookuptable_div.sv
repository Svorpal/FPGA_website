module lookuptable_div(
    input  logic signed [26:0] neighboring_boids,
    output logic signed [26:0] neighboring_boids_val
);

logic signed [26:0] neighboring_boids_val_reg;

always@(*)begin
    case(neighboring_boids)
        27'd0: neighboring_boids_val_reg <= 27'd0;
        27'd1: neighboring_boids_val_reg <= 27'd0;
        27'd2: neighboring_boids_val_reg <= 27'd1;
        27'd3: neighboring_boids_val_reg <= 27'd1;
        27'd4: neighboring_boids_val_reg <= 27'd2;
        27'd5: neighboring_boids_val_reg <= 27'd2;
        27'd6: neighboring_boids_val_reg <= 27'd2;
        27'd7: neighboring_boids_val_reg <= 27'd2;
        27'd8: neighboring_boids_val_reg <= 27'd3;
        27'd9: neighboring_boids_val_reg <= 27'd3;
        27'd10: neighboring_boids_val_reg <= 27'd3;
        default: begin end
    endcase
end
assign neighboring_boids_val = neighboring_boids_val_reg;
endmodule