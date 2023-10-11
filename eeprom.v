module eeprom(
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
//	output reg				irq,			// запрос
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
// Регистр управления/состояния - csr - 175300 
reg  			csr_err = 0;		// ошибка
reg			csr_ri = 0;			// индикация приема данных
reg			csr_xi = 0;			// индикация передачи данных
reg			csr_rdy;				// готовность приема команды
reg  			int_ena;				// флаг разрешения прерываний
reg  [1:0]	csr_fu;				// функция
reg			csr_go = 0;			// разрешение работы
wire [15:0] csr;
assign csr = {csr_err, csr_ri, csr_xi, 5'o0, csr_rdy, int_ena, 3'o0, csr_fu[1:0], csr_go};

// Коды функций - csr_fu
// 00	сброс
// 01	запись данных
// 10	чтение данных
// 11	внутреннии операции

// Шины данных
wire [7:0]	eprm_dat_i, eprm_dat_o;
wire [7:0]	to_host, from_host;
assign from_host[7:0] = wb_dat_i[7:0];

// Регистры данных - base+2
// --- Буфер входных данных ---
buf8 i2cs_host(
	.data(eprm_dat_i),
	.rdaddress(hostr_adr),
	.rdclock(wb_clk_i),
	.wraddress(i2cw_adr),
	.wrclock(nclk),
	.wren(i2c_wren),
	.q(to_host)
);
reg  [4:0]	hostr_adr = 5'b0;
reg  [4:0]	i2cw_adr = 5'b0;
reg			hostr_inc = 1'b0;
wire			hostr_enable, i2cw_enable, i2c_wren;

// Вариант со счетчиком заполнения
//reg  [4:0]	i2c_host_cnt = 5'b0;
//assign hostr_enable = |i2c_host_cnt;
//assign i2cw_enable = ~i2c_host_cnt[4];

// Вариант с вычислением
wire [4:0]	i2c_host_occ = i2cw_adr - hostr_adr;
assign hostr_enable = (|i2c_host_occ)? 1'b1 : 1'b0;
assign i2cw_enable = (&i2c_host_occ)? 1'b0 : 1'b1;

// Сигнал записи в буфер входных данных
assign i2c_wren = dvalid & i2cw_enable;

// --- Буфер выходных данных ---
buf8 host_i2cs(
	.data(from_host),
	.rdaddress(i2cr_adr),
	.rdclock(nclk),
	.wraddress(hostw_adr),
	.wrclock(wb_clk_i),
	.wren(host_i2c_we),
	.q(eprm_dat_o)
);
reg  [4:0]	hostw_adr = 5'b0;
reg  [4:0]	i2cr_adr = 5'b0;
reg			hostw_inc = 1'b0;
wire			hostw_enable, i2cr_enable;
wire			host_i2c_we, host_wrdat;

// Вариант со счетчиком заполнения
//reg  [4:0]	host_i2c_cnt = 5'b0;
//assign hostw_enable = ~host_i2c_cnt[4];
//assign i2cr_enable = |host_i2c_cnt;

// Вариант с вычислением
wire [4:0]	host_i2c_occ = hostw_adr - i2cr_adr;
assign hostw_enable = (host_i2c_occ < 5'd16)? 1'b1 : 1'b0;
assign i2cr_enable = (|host_i2c_occ)? 1'b1 : 1'b0;

// Сигнал записи в буфер выходных данных
assign host_wrdat = bus_write_req & ~csr_go & (wb_adr_i[3:1] == 2'b001) & wb_sel_i[0];
assign host_i2c_we = hostw_enable & wb_we_i & host_wrdat;

// Регистр адреса чтения - base+4 (2'b0,rnum,raddr)
reg  [9:0]	raddr, raddr_old;
reg  [4:0]	rnum;

// Регистр адреса записи - base+6 (2'b0,wnum,waddr)
reg  [9:0]	waddr;
reg  [4:0]	wnum;

// *** Информационные регистры ***
// Регистр ошибок - base+10 (только чтение)
reg  [1:0]	errcode_int;
reg  [1:0]	errcode_ext;
//wire [15:0]	errcode = {lwnum[4:0],lrnum[4:0],1'b0,deverr,errcode_ext,errcode_int};
wire [15:0]	errcode = {9'b0,deverr,errcode_ext,errcode_int};
wire			errstate = |{deverr,errcode_ext,errcode_int};
// Коды ошибок
// 001	опережающее чтение из приемного буфера (i2cs_host)
// 002	опережающая запись в передающий буфер (host_i2cs)
// 004	опережающая запись в приемный буфер (i2cs_host)
// 010	опережающее чтение из передающего буфера (host_i2cs)
// 020	ошибка ведомого устройства

// Регистры адресов чтения - base+12 (только чтение)
wire [15:0]	irdadr = {i2cw_enable, hostr_enable, 4'b0, i2cw_adr[4:0], hostr_adr[4:0]};

// Регистры адресов записи - base+14 (только чтение)
wire [15:0]	iwradr = {hostw_enable, i2cr_enable, 4'b0, hostw_adr[4:0], i2cr_adr[4:0]};

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
wire [7:0]	reg_adr;
wire [9:0]	regw;
wire [4:0]	datnum;
wire			busy, dvalid, newdat, deverr;
reg			ur, rw;
wire [6:0]	dev_adr;
assign regw = rw? raddr : waddr;
assign datnum = rw? rnum : wnum;
assign dev_adr = {5'b10100,regw[9],regw[8]};
assign reg_adr = regw[7:0];

i2c_master i2cmaster(
   .clock_i(nclk),
   .reset_i(comb_res),
   .enable_i(enable),
   .rw_i(rw),
   .ur_i(ur),
   .dat_i(eprm_dat_o),
   .regadr_i(reg_adr),
   .devadr_i(dev_adr),
   .datnum_i(datnum),
   .dat_o(eprm_dat_i),
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
always @(posedge wb_clk_i) begin
   if(comb_res) begin // Сброс модуля
		// Снимаем запрос на прерывания
		int_req <= 1'b0; //irq <= 1'b0; int_state <= i_idle;

		// Сброс регистра управления
		csr_err  <= 1'b0; csr_rdy <= 1'b1;	// сбрасываем признак ошибки и устанавливаем готовность приема команды
		csr_fu <= 2'b0; csr_go <= 1'b0;		// сбрасываем код функции и регистр команды
		int_ena <= 1'b0; int_req <= 1'b0;	// запрещаем прерывания и сбрасываем триггер запроса прерывания
		csr_ri <= 1'b0; csr_xi <= 1'b0;		// сбрасываем индикаторы приема/передачи

		// Сброс регистров адреса
		raddr <= 10'b0; raddr_old <= 10'O1777; waddr <= 10'b0;

		rnum <= 5'b0; wnum <= 5'b0;			// Сброс внутренних счетчиков количества байт
		
		start_soft_res <= 1'b0;					// Сброс регистра сброса
		i2cop <= 1'b0;								// Сброс индмкатора операции по шине I2C
		errcode_int <= 2'b0;						// Сброс регистра внутренних ошибок

		// Сброс регистров адреса внутренненего кольцевого буфера и регистров инкремента
		hostw_adr <= 5'b0; hostr_adr <= 5'b0;
		hostw_inc <= 1'b0; hostr_inc <= 1'b0;
	end

	else begin // Рабочие состояния
/*	//------------------------------------------------
	// Обработка прерывания
		case (int_state) // Обработка прерывания
         i_idle: begin // Нет активного прерывания
				if((int_ena == 1'b1) & (int_req == 1'b1)) begin // Поднят флаг - переходим в состояние активного прерывания
					int_state <= i_req; 
					irq <= 1'b1;
            end 
            else irq <= 1'b0 ; // иначе снимаем запрос на прерывание
         end
			i_req: begin // Формирование запроса на прерывание         
				if(int_ena == 1'b0) int_state <= i_idle;  // прерывания запрещены
				else if(iack == 1'b1) begin // Если получено подтверждение прерывания от процессора
					irq <= 1'b0;			// снимаем запрос
					int_req <= 1'b0;		// очищаем триггер прерывания
					int_state <= i_wait;	// переходим к ожиданию окончания обработки
				end 
			end
         i_wait: begin // Ожидание окончания обработки прерывания
				if(iack == 1'b0) // ждем снятия сигнала iack
					int_state <= i_idle;
			end
      endcase
*/
	//------------------------------------------------
	// Обработка шинных транзакций 
		// чтение регистров
		if(bus_read_req == 1'b1) begin
			//wren <= 0;
			case (wb_adr_i[3:1])
			3'b000: // Base - CSR
				wb_dat_o <= csr;
			3'b001: begin // Base+2 - чтение данных из буфера
				wb_dat_o <= {8'b0,to_host};
				if(hostr_enable) begin
					hostr_inc <= 1'b1;
				end
				else begin
					csr_err <= 1'b1;
					errcode_int <= 2'b01;
				end
			end
			3'b010: // Base+4 - регистр адреса + кол-во байтов (чтение)
				wb_dat_o <= {ur, rnum, raddr};
			3'b011: // Base+6 - регистр адреса + кол-ва байтов (запись)
				wb_dat_o <= {1'o0, wnum, waddr};
			3'b100: // Base+10 - регистр ошибок
				wb_dat_o <= errcode;
			3'b101: // Base+12 - регистр адресов чтения
				wb_dat_o <= irdadr;
			3'b110: // Base+14 - регистр адресов записи
				wb_dat_o <= iwradr;
//			3'b111:
//				wb_dat_o <= {3'b0, i2c_host_occ, 3'b0, host_i2c_occ};
			endcase 
      end

		// запись регистров   
		else if(bus_write_req == 1'b1) begin
			if(~csr_go) begin
				if(wb_sel_i[0] == 1'b1) begin
					case(wb_adr_i[3:1])
						3'b000: begin // Base - CSR
							int_ena <= wb_dat_i[6];			// флаг разрешения прерывания
							csr_fu <= wb_dat_i[2:1];
							csr_go <= wb_dat_i[0];
							if(wb_dat_i[0] == 1'b1)
								csr_rdy <= 1'b0;				// сброс готовности если введена команда
						end
						3'b001: begin // Base+2 - DATOUT
							if(hostw_enable) begin
								hostw_inc <= 1'b1;
							end
							else begin
								errcode_int <= 2'b10;
								csr_err <= 1'b1;
							end
						end
						3'b010: // Base+4 - регистр адреса чтения
							raddr[7:0] <= wb_dat_i[7:0];
						3'b011: // Base+6 - регистр адреса записи
							waddr[7:0] <= wb_dat_i[7:0];
					endcase
				end // wb_sel_i[0] == 1'b1
				if(wb_sel_i[1] == 1'b1) begin
					case (wb_adr_i[3:1])
						3'b000: begin // Base - CSR
							if(wb_dat_i[15] == 1'b1)
								csr_err <= 1'b0;
							if(wb_dat_i[14] == 1'b1)
								csr_ri <= 1'b0;
							if(wb_dat_i[13] == 1'b1)
								csr_xi <= 1'b0;
						end
						3'b010: begin // Base+4 - регистр адреса + кол-во байтов (чтение)
							raddr[9:8] <= wb_dat_i[9:8];
							rnum[4:0] <= wb_dat_i[14:10];
						end
						3'b011: begin // Base+6 - регистр адреса + кол-во байтов (запись)
							waddr[9:8] <= wb_dat_i[9:8];
							wnum[4:0] <= wb_dat_i[14:10];
						end
					endcase
				end // wb_sel_i[1] == 1'b1
			end // ~csr_go
		end // bus_write_req == 1'b1

	// Инкремент адресных регистров
	if(hostr_inc) begin
		hostr_adr <= hostr_adr + 1'b1;
		hostr_inc <= 1'b0;
	end
	if(hostw_inc) begin
		hostw_adr <= hostw_adr + 1'b1;
		hostw_inc <= 1'b0;
	end

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
				2'b01: begin //запись данных
					rw <= 1'b0; csr_fu <= 2'b11;
				end
				2'b10: begin //чтение данных
					rw <= 1'b1; csr_fu <= 2'b11;
					// костыль для ur
			      if (raddr != raddr_old) begin
						raddr_old <= raddr;
						ur <= 1'b1;
					end
					else
						ur <= 1'b0;
				end
				2'b11: begin
					if(~i2cop & ~i2cop_complete) begin
						csr_rdy <= 1'b0; int_req <= 1'b0;
						i2cop <= 1'b1; errcode_int <= 2'b0; csr_err <= 1'b0;
					end
					else begin
						if(i2cop & i2cop_complete) begin
							i2cop <= 1'b0; csr_rdy <= 1'b1; int_req <= 1'b1;
							csr_go <= 1'b0;
							if(rw) begin
								csr_ri <= 1'b1;
							end
							else begin
//								waddr <= waddr + (wnum - lwnum);
								csr_xi <= 1'b1;
							end
							if(|errstate)
								csr_err <= 1'b1;
						end
					end
				end
			endcase
		end // Выполнение функций
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
localparam[1:0] i2c_read  = 1;	// операция чтения
localparam[1:0] i2c_write = 2;	// операция записи
localparam[1:0] i2c_stop  = 3;	// операция остановки
reg  [1:0]	i2c_state;
reg  [4:0]	lrnum, lwnum;

reg			i2cr_inc, i2cw_inc;
 
always @(posedge nclk, posedge comb_res) begin
	if(comb_res) begin
		// Сброс регистров адреса внешнего кольцевого буфера
		i2cw_adr <= 5'b0; i2cr_adr <= 5'b0;
		// Сброс индикатора окончания операции по шине I2C
		i2cop_complete <= 1'b0;
		//
		i2cr_inc <= 1'b0; i2cw_inc <= 1'b0;
   end
	else begin
		case(i2c_state)
			i2c_idle: begin
				if(enable) begin
					case(rw)
						1'b1: begin
							i2c_state <= i2c_read; lrnum <= rnum; errcode_ext <= 2'b0;
						end
						1'b0: begin
							i2c_state <= i2c_write; lwnum <= wnum; errcode_ext <= 2'b0;
						end
					endcase
				end
			end
			i2c_read: begin
				if(busy) begin
					if(dvalid)
						if(i2cw_enable) begin
							lrnum <= lrnum - 1'b1;
							i2cw_inc <= 1'b1;
						end
						else
							errcode_ext <= 2'b01;
					end
				else
					i2c_state <= i2c_stop;
			end
			i2c_write: begin
				if(busy) begin
					if(newdat) begin
						if(i2cr_enable) begin
							lwnum <= lwnum - 1'b1;
							i2cr_inc <= 1'b1;
						end
						else
							errcode_ext <= 2'b10;
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

		// Инкремент адресных регистров
		if(i2cw_inc) begin
			i2cw_adr <= i2cw_adr + 1'b1;
			i2cw_inc <= 1'b0;
		end
		if(i2cr_inc) begin
			i2cr_adr <= i2cr_adr + 1'b1;
			i2cr_inc <= 1'b0;
		end
	end
end
// ======================================================================================
/*
// Счетчики заполненности циклических буферов ===========================================
reg			prev_i2cr_inc, prev_i2cw_inc;
wire			i2cr_inc_clk = ~prev_i2cr_inc & i2cr_inc;
wire			i2cw_inc_clk = ~prev_i2cw_inc & i2cw_inc;
always @(posedge wb_clk_i) begin
	prev_i2cr_inc <= i2cr_inc;
	prev_i2cw_inc <= i2cw_inc;
end

always @(posedge wb_clk_i, posedge comb_res) begin
	if(comb_res) begin
		host_i2c_cnt <= 5'b0;
	end
	else begin
		if(i2cr_inc_clk)
			host_i2c_cnt <= host_i2c_cnt - 1'b1;
		if(hostw_inc)
			host_i2c_cnt <= host_i2c_cnt + 1'b1;
		if(i2cw_inc_clk)
			i2c_host_cnt <= i2c_host_cnt + 1'b1;
		if(hostr_inc)
			i2c_host_cnt <= i2c_host_cnt - 1'b1;
	end
end
// ======================================================================================
*/
endmodule
