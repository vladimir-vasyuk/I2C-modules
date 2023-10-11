// Модуль формирования сигнала сброса ===================================================
module reset_signal(
	input			clk_i,		// Тактовая
	input			rst_i,		// Сигнал сброса
	input			start_i,		// Сигнал формирования
	output		sreset_o		// Сигнал сброса
);

reg  [5:0]	tm = 6'b0;
wire			tm_enable;
assign tm_enable = (|tm) | start_i;
assign sreset_o = |tm;

always @(posedge clk_i, posedge rst_i) begin
	if(rst_i)
		tm <= 6'b0;
	else begin
		if(tm_enable) tm <= tm + 1'b1;
	end
end

endmodule
