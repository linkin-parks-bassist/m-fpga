`include "lut.vh"

module sin_2pi_lut_16 (
		input wire clk,
		input wire reset,

		input wire read,
		output reg valid,

		input wire signed [15:0] x,
		
		output reg signed [15:0] base_sample,
		output reg signed [15:0] next_sample,
		
		output wire [`LUT_FRAC_WIDTH - 1 : 0] frac
	);

	reg signed [15:0] sin_lut[0:2047];

	initial begin
		$readmemh("luts/sin_q15_full.hex", sin_lut);
	end
	
	wire [10:0] base_index = x[14:4];
	wire [10:0] next_index = (base_index == 2047) ? 0 : base_index + 1;
	
	assign frac = x[3:0];

	reg [10:0] read_addr = 0;
	reg [15:0] read_result;

	reg wait_one = 0;

	reg [3:0] state = 0;

	always @(posedge clk) begin
		read_result <= sin_lut[read_addr];
		wait_one <= 0;
		valid <= 0;

		if (reset) begin
			state <= 0;
		end
		else begin
			case (state)
				0: begin
					if (read) begin
						valid <= 0;
						read_addr <= base_index;
						state <= 1;
						wait_one <= 1;
					end
				end

				1: begin
					if (!wait_one) begin
						base_sample <= read_result;
						read_addr <= next_index;
						state <= 2;
						wait_one <= 1;
					end
				end

				2: begin
					if (!wait_one) begin
						next_sample <= read_result;
						state <= 0;
						valid <= 1;
					end
				end
			endcase
		end
	end
endmodule

module tanh_4_lut_16 (
		input wire clk,
		input wire reset,

		input wire read,
		output reg valid,

		input wire signed [15:0] x,
		
		output reg [15:0] base_sample,
		output reg [15:0] next_sample,
		
		output wire [`LUT_FRAC_WIDTH - 1 : 0] frac
	);

	reg signed [15:0] tanh_lut[0:2047];

	initial begin
		$readmemh("luts/tanh_q15.hex", tanh_lut);
	end

	wire signed [15:0] x_index = x + 16'sh8000;

	wire [10:0] base_index = x_index[15:5];
	wire [10:0] next_index = (base_index == 2047) ? 2047 : base_index + 1;
	
	assign frac = x[4:1];

	reg [10:0] read_addr = 0;
	reg [15:0] read_result;

	reg wait_one = 0;

	reg [3:0] state = 0;

	always @(posedge clk) begin
		read_result <= tanh_lut[read_addr];
		wait_one <= 0;

		if (reset) begin
			state <= 0;
			valid <= 0;
		end
		else begin
			valid <= 0;
			
			case (state)
				0: begin
					if (read) begin
						read_addr <= base_index;
						state <= 1;
						wait_one <= 1;
					end
				end

				1: begin
					if (!wait_one) begin
						base_sample <= read_result;
						read_addr <= next_index;
						state <= 2;
						wait_one <= 1;
					end
				end

				2: begin
					if (!wait_one) begin
						next_sample <= read_result;
						state <= 0;
						valid <= 1;
					end
				end
			endcase
		end
	end
endmodule

