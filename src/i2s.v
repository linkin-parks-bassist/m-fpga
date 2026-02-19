`default_nettype none

module i2s_trx #(parameter sample_size = 16)
(
	input wire sys_clk,
	input wire bclk,   input wire lrclk, input wire din, output reg dout,
	input wire enable, input wire reset, output reg rx_valid,
	input wire [sample_size-1:0] tx_l, input wire [sample_size-1:0] tx_r,
	output reg [sample_size-1:0] rx_l, output reg [sample_size-1:0] rx_r
);

	reg [7:0] ctr;

	reg [sample_size * 2 - 1:0] rx_sr;

	reg [sample_size-1:0] tx_l_latched;
	reg [sample_size-1:0] tx_r_latched;

	wire [sample_size-1:0] sample_out = lrclk_prev ? tx_r_latched : tx_l_latched;

	reg lrclk_prev;
	reg bclk_prev;

	always @(posedge sys_clk) begin
		bclk_prev <= bclk;

		if (reset) begin
			ctr <= 0;
			rx_sr <= 0;
			tx_r_latched <= 0;
			tx_l_latched <= 0;
			lrclk_prev <= 0;
			dout <= 0;
		end else if (enable) begin
			if (bclk & ~bclk_prev) begin
				lrclk_prev <= lrclk;
				rx_valid <= 0;
				
				if (lrclk != lrclk_prev) begin
					
					if (lrclk) begin
						rx_l <= rx_sr[sample_size * 2 - 1 : sample_size];
						rx_r <= rx_sr[sample_size * 1 - 1 : 0];

						rx_valid <= 1;

						tx_l_latched <= tx_l;
						tx_r_latched <= tx_r;
					end

					ctr <= 0;
				end else begin
					if (ctr < sample_size)
						rx_sr <= {rx_sr[sample_size * 2 - 2 : 0], din};
					
					ctr <= ctr + 1;
				end
			end else if (~bclk & bclk_prev) begin
				if (ctr < sample_size)
					dout <= sample_out[sample_size - ctr - 1];
				else
					dout <= 0;
			end
		end
	end
endmodule

`default_nettype wire
