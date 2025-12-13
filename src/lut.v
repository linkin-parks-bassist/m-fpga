module sin_2pi_lut_16
	(
		input wire signed [15:0] x,
		
		output wire [15:0] base_sample,
		output wire [15:0] next_sample,
		
		output wire [`LUT_FRAC_WIDTH - 1 : 0] frac
	);

	reg signed [15:0] sin_lut[0:2047];

	initial begin
		$readmemh("luts/sin_q15_full.hex", sin_lut);
	end
	
	wire [10:0] base_index = x[14:4];
	wire [10:0] next_index = (base_index == 2047) ? 0 : base_index + 1;
	
	assign base_sample = sin_lut[base_index];
	assign next_sample = sin_lut[next_index];
	
	assign frac = x[3:0];
endmodule

module tanh_4_lut_16
	(
		input wire signed [15:0] x,
		
		output wire [15:0] base_sample,
		output wire [15:0] next_sample,
		
		output wire [`LUT_FRAC_WIDTH - 1 : 0] frac
	);

	reg signed [15:0] tanh_lut[0:2047];

	initial begin
		$readmemh("luts/tanh_q15.hex", tanh_lut);
	end

	wire signed [15:0] x_index = x + 16'sh8000;

	wire [10:0] base_index = x_index[15:5];
	wire [10:0] next_index = (base_index == 2047) ? 2047 : base_index + 1;
	
	assign base_sample = $signed(tanh_lut[base_index]);
	assign next_sample = $signed(tanh_lut[next_index]);
	
	assign frac = x[4:1];
endmodule

