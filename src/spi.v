/* 
 * Synchronous SPI slave
 *
 * Samples SPI using system clock. Avoids introducing
 * CDC conerns, and works perfectly at 10MHz, so it's
 * good enough for me.
 *
 */

module sync_spi_slave #(parameter CPOL = 0, parameter CPHA = 0)
	(
		input wire clk,
		input wire reset,
		
		input wire [7:0] miso_byte,

		input 	wire sck,
		input 	wire cs,
		input 	wire mosi,
		output  reg  miso,
		
		input 	wire 		enable,
		
		output 	reg [7:0] 	mosi_byte,
		output 	reg 		data_valid
	);
	
	reg sck_sync_ff;
	reg sck_sync;
	
	reg sck_norm_prev;
	
	wire sck_norm = sck_sync ^ CPOL;
	
	wire sck_rise =  sck_norm & ~sck_norm_prev;
	wire sck_fall = ~sck_norm &  sck_norm_prev;
	
	wire sample_edge = (CPHA == 0) ? sck_rise : sck_fall;
	
	reg cs_sync_ff   = 1;
	reg cs_sync	  = 1;
	reg cs_sync_prev = 1;
	
	assign data_read = (sr_index == 7);
	
	reg [2:0] sr_index  = 3'b0;

	reg [7:0] miso_byte_latched = 0;

	always @(posedge clk) begin
		sck_sync_ff <= sck;
		sck_sync <= sck_sync_ff;
		
		cs_sync_ff   <= cs;
		cs_sync	  <= cs_sync_ff;
		cs_sync_prev <= cs_sync;
		
		if (!cs_sync && cs_sync_prev) begin
			miso_byte_latched <= miso_byte;
		end

		data_valid	<= 1'b0;
		
		if (cs_sync || reset) begin
			mosi_byte 	<= 8'b0;
			sr_index	<= 3'b0;
			miso		<= 0;
		end
		else if (!cs_sync && enable) begin
			if (sample_edge) begin
				mosi_byte <= {mosi_byte[6:0], mosi};
				miso <= miso_byte_latched[7 - sr_index];

				if (sr_index == 7) begin
					data_valid 	<= 1'b1;
					sr_index 	<= 0;
				end
				else begin
					sr_index 	<= sr_index + 1;
				end
			end
		end
		else begin
			miso <= miso_byte[7];
		end
		
		sck_norm_prev <= sck_norm;
	end
endmodule
