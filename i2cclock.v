// Новая тактовая для работы I2C
module i2cclock(
	input				clk_i,
	input [15:0]	divider_i,
	output reg		new_clk_o
);

reg [15:0]	counter;

always @(posedge clk_i) begin
	if(counter == divider_i) begin
		new_clk_o <= ~new_clk_o;
		counter <= 16'd0;
	end
	else
		counter <= counter + 1'b1;
end
endmodule
