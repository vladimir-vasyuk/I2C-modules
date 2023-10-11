//////////////////////////////////////////////////////////////////////////////////
//
// Base idea: Artin Isagholian (artinisagholian@gmail.com)
//
// Create Date:   21/08/2023
// Module Name:   i2c_mmaster
// Realized:      Byte-Write, Page-Write, Random-Read, Current-Address-Read, Sequential-Read
// 
// Dependencies: 
// 
// Revision:
// Revision 1.0 - Read/write operation works
// Revision 1.1 - Corrected Send-Ack operation
// Additional Comments: 
// 
//////////////////////////////////////////////////////////////////////////////////
module i2c_master(
	input					clock_i,		// I2C clock
	input					reset_i,		// Reset signal (active high)
	input					enable_i,	// Enable signal (active high)
	input					rw_i,			// Read/Write (1/0)
	input					ur_i,			// Use register (active high)
	input [7:0]			dat_i,		// Data to send to a slave
	input [7:0]			regadr_i,	// Register address value
	input [6:0]			devadr_i,	// Device address value
	input [4:0]			datnum_i,	// Number of bytes to send (to receive)
//
	output reg [7:0]	dat_o,		// Data received from a slave
	output reg			busy_o,		// Busy signal (active high)
	output reg			deverr_o,	// Device error (active high)
	output				dvalid_o,	// Data valid signal (active high)
	output				newdat_o,	// New data request signal (active high)
//
	inout					sda,			// I2C data line
	inout					scl			// I2C clock line
);

/*INSTANTATION TEMPLATE
i2c_mmaster i2c_master_inst(
   .clock_i    (),
   .reset_i    (),
   .enable_i   (),
   .rw_i       (),
   .ur_i       (),
   .dat_i      (),
   .regadr_i   (),
   .devadr_i   (),
   .dat_o      (),
   .busy_o     (),
   .dvalid_o   (),
   .newdat_o   (),
   .sda        (),
   .scl        ()
);
*/
localparam[3:0] S_IDLE       = 0;
localparam[3:0] S_START      = 1;
localparam[3:0] S_WRITE_ADR  = 2;
localparam[3:0] S_CHECK_ACK  = 3;
localparam[3:0] S_WRITE_REG  = 4;
localparam[3:0] S_RESTART    = 5;
localparam[3:0] S_READ_DATA  = 6;
localparam[3:0] S_SEND_STOP  = 7;
localparam[3:0] S_WRITE_DATA = 8;
localparam[3:0] S_SEND_ACK   = 9;

reg [3:0]	state, next_state;
reg			serial_clock;
reg [7:0]	saved_devadr;
reg [7:0]	saved_regadr;
reg [15:0]	saved_datnum;
reg [7:0]	saved_i2c_o;
reg [1:0]	process_counter;
reg [3:0]	bit_counter;
reg			serial_data;
reg			next_serial_data;
reg			last_ack;
reg			saved_rw_i;
reg			saved_ur_i;
reg			sda_enable;
reg			scl_enable;

wire			use_reg = ~saved_rw_i | saved_ur_i;
wire			last_bit = saved_rw_i & ~use_reg;
assign scl = (scl_enable) ? serial_clock : 1'bz;
assign sda = (sda_enable) ? serial_data  : 1'bz;
assign newdat_o = (state==S_WRITE_DATA && bit_counter==8'h7 && process_counter==2'h0)? 1'b1 : 1'b0;
assign dvalid_o = (state==S_SEND_ACK && process_counter==2'h0)? 1'b1 : 1'b0;

always @(*) begin
	if (state!=S_IDLE && state!=S_CHECK_ACK && state!=S_READ_DATA)
		sda_enable = 1'b1;
	else
		sda_enable = 1'b0;

	if (state!=S_IDLE && process_counter!=2'h1 && process_counter!=2'h2)
		scl_enable = 1'b1;
	else
		scl_enable = 1'b0;
end

always @(posedge clock_i) begin
	if (reset_i) begin
		state <= S_IDLE;
		deverr_o <= 1'b0;
	end
	else begin
		case(state)
			S_IDLE: begin
				next_state      <= S_IDLE;
				process_counter <= 2'h0;
				bit_counter     <= 4'b0;
				last_ack        <= 1'b0;
				busy_o          <= 1'b0;
				saved_ur_i      <= ur_i;
				saved_regadr    <= regadr_i;
				saved_datnum    <= datnum_i;
				serial_data     <= 1'b1;
				serial_clock    <= 1'b1;
				saved_rw_i      <= rw_i;
				if (enable_i) begin
					busy_o <= 1'b1;
					if(saved_datnum > 1'b0) begin
						state <= S_START;
						deverr_o <= 1'b0;
					end
					else begin
						state <= S_SEND_STOP;
						deverr_o <= 1'b1;
					end
				end
			end
			S_START: begin
				case(process_counter)
					0: begin
						saved_devadr <= {devadr_i,last_bit};
						process_counter <= process_counter + 1'b1;
					end
					1: begin
						serial_data <= 1'b0;
						process_counter <= process_counter + 1'b1;
					end
					2: begin
						bit_counter <= 4'd8;
						process_counter <= process_counter + 1'b1;
					end
					3: begin
						serial_clock <= 1'b0;
						process_counter <= process_counter + 1'b1;
						state <= S_WRITE_ADR;
						serial_data <= saved_devadr[7];
						saved_i2c_o <= dat_i;
					end
				endcase
			end
			S_WRITE_ADR: begin
				case(process_counter)
					0: begin
						serial_clock <= 1'b1;
						process_counter <= process_counter + 1'b1;
					end
					1: begin //check for clock stretching
						if(scl == 1'b1) begin
							process_counter <= process_counter + 1'b1;
						end
					end
					2: begin
						serial_clock <= 1'b0;
						bit_counter <= bit_counter - 1'b1;
						process_counter <= process_counter + 1'b1;
					end
					3: begin
						if(bit_counter == 1'b0) begin
							if(use_reg) begin
								next_serial_data <= saved_regadr[7];
								next_state <= S_WRITE_REG;
							end
							else begin
								if(saved_rw_i) begin
									next_state <= S_READ_DATA;
								end
								else begin
									next_state <= S_SEND_STOP;
								end
							end
							state <= S_CHECK_ACK;
							bit_counter <= 4'd8;
						end
						else begin
							serial_data <= saved_devadr[bit_counter-1];
						end
						process_counter <= process_counter + 1'b1;
					end
				endcase
			end
			S_CHECK_ACK: begin
				case(process_counter)
					0: begin
						serial_clock <= 1'b1;
						if(next_state == S_WRITE_DATA)
							next_serial_data <= saved_i2c_o[7];
						process_counter <= process_counter + 1'b1;
					end
					1: begin //check for clock stretching
						if(scl == 1'b1) begin
							process_counter <= process_counter + 1'b1;
						end
					end
					2: begin
						serial_clock <= 1'b0;
						if(sda == 1'b0) begin
							last_ack <= 1'b1;
						end
						process_counter <= process_counter + 1'b1;
					end
					3: begin
						if(last_ack == 1'b1) begin
							last_ack <= 1'b0;
							serial_data <= next_serial_data;
							state <= next_state;
						end
						else begin
							deverr_o <= 1'b1;
							state <= S_IDLE;
						end
						process_counter <= process_counter + 1'b1;
					end
				endcase
			end
			S_WRITE_REG: begin
				case(process_counter)
					0: begin
						serial_clock <= 1'b1;
						process_counter <= process_counter + 1'b1;
					end
					1: begin //check for clock stretching
						if(scl == 1'b1) begin
							process_counter <= process_counter + 1'b1;
						end
					end
					2: begin
						serial_clock <= 1'b0;
						bit_counter <= bit_counter - 1'b1;
						process_counter <= process_counter + 1'b1;
					end
					3: begin
						if(bit_counter == 1'b0) begin
							if(saved_rw_i == 1'b0) begin
								next_state <= S_WRITE_DATA;
								next_serial_data <= saved_i2c_o[7];
							end
							else begin
								next_state <= S_RESTART;
								next_serial_data <= 1'b1;
							end
							bit_counter <= 4'd8;
							serial_data <= 1'b0;
							state <= S_CHECK_ACK;
						end
						else begin
							serial_data <= saved_regadr[bit_counter-1];
						end
						process_counter <= process_counter + 1'b1;
					end
				endcase
			end
			S_WRITE_DATA: begin
				case(process_counter)
					0: begin
						serial_clock <= 1'b1;
						process_counter <= process_counter + 1'b1;
					end
					1: begin //check for clock stretching
						if(scl == 1'b1) begin
							process_counter <= process_counter + 1'b1;
						end
					end
					2: begin
						serial_clock <= 1'b0;
						bit_counter <= bit_counter - 1'b1;
						process_counter <= process_counter + 1'b1;
					end
					3: begin
						if(bit_counter == 1'b0) begin
							state <= S_CHECK_ACK;
							next_serial_data <= 1'b0;
							bit_counter <= 4'd8;
							serial_data <= 1'b0;
							saved_i2c_o <= dat_i;
							if(saved_datnum > 1'b1) begin
								saved_datnum <= saved_datnum - 1'b1;
								next_state <= S_WRITE_DATA;
							end
							else
								next_state <= S_SEND_STOP;
						end
						else begin
							serial_data <= saved_i2c_o[bit_counter-1];
						end
						process_counter <= process_counter + 1'b1;
					end
				endcase
			end
			S_RESTART: begin
				case(process_counter)
					0: begin
						process_counter <= process_counter + 1'b1;
					end
					1: begin
						process_counter <= process_counter + 1'b1;
						serial_clock <= 1'b1;
					end
					2: begin
						process_counter <= process_counter + 1'b1;
					end
					3: begin
						state <= S_START;
						next_state <= S_WRITE_ADR;
						saved_ur_i <= 1'b0;
						process_counter <= process_counter + 1'b1;
               end
				endcase
			end
			S_READ_DATA: begin
				case(process_counter)
					0: begin
						serial_clock <= 1'b1;
						process_counter <= process_counter + 1'b1;
					end
					1: begin //check for clock stretching
						if(scl == 1'b1) begin
							process_counter <= process_counter + 1'b1;
						end
					end
					2: begin
						serial_clock <= 1'b0;
						dat_o <= {dat_o[6:0],sda};
						bit_counter <= bit_counter - 1'b1;
						process_counter <= process_counter + 1'b1;
					end
					3: begin
						if(bit_counter == 1'b0) begin
							bit_counter <= 4'd8;
							state <= S_SEND_ACK;
							if(saved_datnum > 1'b1) begin
								saved_datnum <= saved_datnum - 1'b1;
								serial_data <= 1'b0;       // Send ACK
								next_state <= S_READ_DATA;
							end
							else begin
								next_state <= S_SEND_STOP;
								serial_data <= 1'b1;       // Send NACK
							end
						end
						process_counter <= process_counter + 1'b1;
					end
				endcase
			end
			S_SEND_ACK: begin
				case(process_counter)
					0: begin
						serial_clock <= 1'b1;
						process_counter <= process_counter + 1'b1;
					end
					1: begin //check for clock stretching
						if(scl == 1'b1) begin
							process_counter <= process_counter + 1'b1;
						end
					end
					2: begin
						process_counter <= process_counter + 1'b1;
						serial_clock <= 1'b0;
					end
					3: begin
						state <= next_state;
						process_counter <= process_counter + 1'b1;
						serial_data <= 1'b0;
					end
				endcase
			end
			S_SEND_STOP: begin
				case(process_counter)
					0: begin
						serial_clock <= 1'b1;
						process_counter <= process_counter + 1'b1;
					end
					1: begin //check for clock stretching
						if(scl == 1'b1) begin
							process_counter <= process_counter + 1'b1;
						end
               end
					2: begin
						process_counter <= process_counter + 1'b1;
						serial_data <= 1'b1;
					end
					3: begin
						state <= S_IDLE;
					end
				endcase
			end
		endcase
	end
end

endmodule
