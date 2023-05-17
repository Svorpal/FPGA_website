module reciprocal (
  input clk,
  input rst_n,
  input [26:0] num,
  output reg [26:0] reciprocal
);

reg [26:0] num_reg;
reg [26:0] r;
reg [26:0] residue;
reg [5:0] counter;

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    num_reg <= 27'b0;
  end else begin
    num_reg <= num;
  end
end

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    r <= 27'b0;
    residue <= 27'b100000000000000000000000000;
    counter <= 6'b0;
  end else begin
    if (counter < 6'b100000) begin
      r <= r << 1;
      residue <= residue << 1;
      if (residue >= num_reg) begin
        residue <= residue - num_reg;
        r <= r + 1;
      end
      counter <= counter + 1;
    end else begin
      counter <= counter;
    end
  end
end

assign reciprocal = (num == 0) ? 27'b111111111111111111111111111 : r;

endmodule
