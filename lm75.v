//=============================================
// LM75A - Digital temperature sensor and thermal Watchdog
//=============================================
module lm75(
	input						wb_clk_i,	// тактовая частота шины
	input						wb_rst_i,	// сброс
	input      [3:0]		wb_adr_i,	// адрес 
	input      [15:0]		wb_dat_i,	// входные данные
	output reg [15:0]		wb_dat_o,	// выходные данные
	input						wb_cyc_i,	// начало цикла шины
	input						wb_we_i,		// разрешение записи (0 - чтение)
	input						wb_stb_i,	// строб цикла шины
	input      [1:0]		wb_sel_i,	// выбор байтов для записи 
	output 					wb_ack_o,	// подтверждение выбора устройства

// обработка прерывания   
	output					irq,			// запрос
	input						iack,			// подтверждение

// I2C шина
	inout						scl,			// тактовая частота
	inout						sda			// линия данных
);

// Сигналы упраления обменом с шиной
wire bus_strobe = wb_cyc_i & wb_stb_i;			// строб цикла шины
wire bus_read_req = bus_strobe & ~wb_we_i;	// запрос чтения
wire bus_write_req = bus_strobe & wb_we_i;	// запрос записи

// Регистры модуля ======================================================================
// Регистр управления/состояния - csr - 175200 
reg  			csr_err = 0;		// ошибка
reg			csr_ri = 0;			// индикация приема данных
reg			csr_xi = 0;			// индикация передачи данных
reg  [1:0]	csr_reg = 0;		// регистр устройства
reg  [2:0]	csr_dva = 0;		// адрес устройства
reg			csr_rdy;				// готовность приема команды
reg  			int_ena;				// флаг разрешения прерываний
reg  [1:0]	csr_fu;				// функция
reg			csr_go = 0;			// разрешение работы
wire [15:0] csr;
assign csr = {csr_err, csr_ri, csr_xi, csr_reg[1:0], csr_dva[2:0], csr_rdy, int_ena, 3'b0, csr_fu[1:0], csr_go};

// Коды функций - csr_fu
// 00	- сброс
// 01	- запись данных
// 10	- чтение данных
// 11	- внутреннии операции
//
// Регистр устройства - csr_reg
// 00 - температура (2 bytes, RO)
// 01 - конфигурация (1 byte, RW)
// 10 - гистерезис (2 bytes, RW)
// 11 - порог превышения температуры (2 bytes, RW)
//

// Регистр данных - base+2
reg  [15:0]	dato;
reg  [7:0]	dati_h, dati_l;

// Регистры синхронизации операций с I2Cmaster
reg			i2cop, i2cop_complete;

// Обработка прерывания =================================================================
reg  			int_req;				// триггер запроса прерывания
bus_int inter(
	.clk_i(wb_clk_i),
	.rst_i(wb_rst_i),
	.ena_i(int_ena),
	.req_i(int_req),
	.ack_i(iack),
	.irq_o(irq)
);

// Формирование сигнала программного сброса==============================================
reg			start_soft_res = 1'b0;
wire			new_res;
wire			comb_res = new_res | wb_rst_i;
reset_signal soft_res(
	.clk_i(wb_clk_i),
	.rst_i(wb_rst_i),
	.start_i(start_soft_res),
	.sreset_o(new_res)
);

// Новая тактовая для I2C================================================================
wire			nclk;
i2cclock i2c_clk(
	.clk_i(wb_clk_i),
   .divider_i(16'd31),
	.new_clk_o(nclk)
);

// Мастер I2C ===========================================================================
wire [7:0]	lm75_dat_i, lm75_dat_o;
wire [7:0]	reg_adr = {6'b0,csr_reg[1:0]};
wire [6:0]	dev_adr = {4'b1001,csr_dva[2:0]};
wire [1:0]	dn = (csr_reg == 2'b01)? 2'b01 : 2'b10;
wire [1:0]	mdn = dn - 1'b1;
wire [4:0]	datnum = {3'b0, dn[1:0]};
wire			busy, dvalid, newdat, deverr;
wire			ur = 1'b1;
wire			rw = (csr_fu == 2'b10)? 1'b1 : 1'b0;
assign lm75_dat_o[7:0] = lnum? dato[15:8] : dato[7:0];

i2c_master i2cmaster(
   .clock_i(nclk),
   .reset_i(comb_res),
   .enable_i(enable),
   .rw_i(rw),
   .ur_i(ur),
   .dat_i(lm75_dat_o),
   .regadr_i(reg_adr),
   .devadr_i(dev_adr),
   .datnum_i({datnum}),
   .dat_o(lm75_dat_i),
   .busy_o(busy),
	.deverr_o(deverr),
   .dvalid_o(dvalid),
   .newdat_o(newdat),
   .sda(sda),
   .scl(scl)
);

// Формирователь ответа на цикл шины ====================================================
reg reply;
always @(posedge wb_clk_i)
   if(wb_rst_i)			reply <= 1'b0;
   else if(wb_stb_i)		reply <= 1'b1;
   else 						reply <= 1'b0;

assign wb_ack_o = reply & wb_stb_i;    

// Работа с шиной =======================================================================
//
always @(posedge wb_clk_i, posedge comb_res) begin
   if(comb_res) begin // Сброс модуля
		// Снимаем запрос на прерывания
		int_req <= 1'b0;

		// Сброс регистра управления
		csr_err  <= 1'b0; csr_rdy <= 1'b1;	// сбрасываем признак ошибки и устанавливаем готовность приема команды
		csr_fu <= 2'b0; csr_go <= 1'b0;		// сбрасываем код функции и регистр команды
		csr_reg <= 2'b0;							// сбрасываем адрес регистра
		int_ena <= 1'b0; int_req <= 1'b0;	// запрещаем прерывания и сбрасываем триггер запроса прерывания
		csr_ri <= 1'b0; csr_xi <= 1'b0;		// сбрасываем индикаторы приема/передачи

		start_soft_res <= 1'b0;					// Сброс регистра сброса
		i2cop <= 1'b0;								// Сброс индмкатора операции по шине I2C
	end

	else begin // Рабочие состояния
	//------------------------------------------------
	// Обработка шинных транзакций 
		// чтение регистров
		if(bus_read_req == 1'b1) begin
			case (wb_adr_i[2:1])
			2'b00: // Base - CSR
				wb_dat_o <= csr;
			2'b01: begin // Base+2 - чтение данных из буфера
				if(dn[0])
					wb_dat_o <= {8'b0, dati_l[7:0]};
				else
					wb_dat_o <= {dati_h[7:0], dati_l[7:0]};
			end
			endcase 
      end

		// запись регистров   
		else if(bus_write_req == 1'b1) begin
			if(~csr_go) begin
				if(wb_sel_i[0] == 1'b1) begin
					case(wb_adr_i[2:1])
						2'b00: begin // Base - CSR
							int_ena <= wb_dat_i[6];			// флаг разрешения прерывания
							csr_fu <= wb_dat_i[2:1];
							csr_go <= wb_dat_i[0];
							if(wb_dat_i[0] == 1'b1)
								csr_rdy <= 1'b0;				// сброс готовности если введена команда
						end
						2'b01:		 // Base+2 - DATOUT
							dato[7:0] <= wb_dat_i[7:0];
					endcase
				end // wb_sel_i[0] == 1'b1
				if(wb_sel_i[1] == 1'b1) begin
					case (wb_adr_i[2:1])
						2'b00: begin // Base - CSR
							if(wb_dat_i[15] == 1'b1)
								csr_err <= 1'b0;
							if(wb_dat_i[14] == 1'b1)
								csr_ri <= 1'b0;
							if(wb_dat_i[13] == 1'b1)
								csr_xi <= 1'b0;
							csr_reg <= wb_dat_i[12:11];
							csr_dva <= wb_dat_i[10:8];
						end
						2'b01:		 // Base+2 - DATOUT
							dato[15:8] <= wb_dat_i[15:8];
					endcase
				end // wb_sel_i[1] == 1'b1
			end // ~csr_go
		end // bus_write_req == 1'b1

	// Сброс требованияч прерывания
	if(int_req & irq)
		int_req <= 1'b0;

	//------------------------------------------------
	// Выполнение функций
		if(csr_go == 1'b1) begin
			case (csr_fu)
				2'b00: begin //сброс
					start_soft_res <= 1'b1;
				end
				default: begin	// запись/чтение данных
					if(~i2cop & ~i2cop_complete) begin
						csr_rdy <= 1'b0; csr_err <= 1'b0;
						i2cop <= 1'b1; int_req <= 1'b0;
					end
					else begin
						if(i2cop & i2cop_complete) begin
							i2cop <= 1'b0; csr_rdy <= 1'b1; int_req <= 1'b1;
							csr_go <= 1'b0; csr_err <= tmerr;
							if(rw)
								csr_ri <= 1'b1;
							else
								csr_xi <= 1'b1;
						end
					end
				end
			endcase
		end // Выполнение функций

		// Запись данных 
		if(dvalid) begin
			if(lnum)
				dati_h <= lm75_dat_i;
			else
				dati_l <= lm75_dat_i;
		end

	end // Рабочие состояния
end //always block
//
// ======================================================================================

// Формирование сигнала enable =====================================================
reg			prev_start;
reg			enable = 1'b0;
wire			start_front_pos = ~prev_start & i2cop;
always @(posedge nclk) begin
	prev_start <= i2cop;
end

always @(posedge nclk, posedge comb_res) begin
	if(comb_res) begin
		enable <= 1'b0;
   end
	else begin
		if(busy) begin
			enable <= 1'b0;
		end
		else
			if (start_front_pos) begin
				enable <= 1'b1;
			end
	end
end
// ======================================================================================

// Работа с мастером I2C ================================================================
//
// Конечный автомат обработки прерывания
localparam[1:0] i2c_idle  = 0;	// ожидание операции
localparam[1:0] i2c_op	  = 1;	// операция чтения
localparam[1:0] i2c_stop  = 2;	// операция остановки
reg  [1:0]	i2c_state;
reg			lnum;						// Номер байта
reg  [8:0]	tm;						// Таймаут счетчик 
wire			tmerr = &tm;			// Таймаут сигнал

always @(posedge nclk, posedge comb_res) begin
	if(comb_res) begin
		i2cop_complete <= 1'b0;		// Сброс индикатора окончания операции по шине I2C
   end
	else begin
		case(i2c_state)
			i2c_idle: begin
				if(enable) begin
					lnum <= mdn[0]; tm <= 9'b0;
					i2c_state <= i2c_op;
				end
			end
			i2c_op: begin
				if(busy & ~tmerr) begin
					tm <= tm + 1'b1;
					if(dvalid | newdat) begin
						if(lnum)
							lnum <= lnum - 1'b1;
					end
				end
				else
					i2c_state <= i2c_stop;
			end
			i2c_stop: begin
				if(~i2cop) begin
					i2cop_complete <= 1'b0;
					i2c_state <= i2c_idle;
				end
				else
					i2cop_complete <= 1'b1;
			end
		endcase
	end
end
// ======================================================================================

endmodule
