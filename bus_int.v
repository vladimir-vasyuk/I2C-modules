// Модуль обработки сигнала требования прерывания =======================================
module bus_int(
	input			clk_i,		// Тактовая
	input			rst_i,		// Сигнал сброса
	input			ena_i,		// Сигнал разрешения прерывания
	input			req_i,		// Сигнал требования прерывания
	input			ack_i,		// Сигнал подтверждения прерывания
	output reg	irq_o			// Сигнал прерывания на шину
);

//wire int_reg = ena_i & req_i;

always @(posedge clk_i, posedge rst_i)
	if(rst_i) irq_o <= 1'b0;
	else begin
//		if(int_reg & (~irq_o))
		if(ena_i) begin
			if(req_i)
				irq_o <= 1'b1;
		end
		else
			irq_o <= 1'b0;
		if(ack_i & irq_o)
			irq_o <= 1'b0;
	end

endmodule
