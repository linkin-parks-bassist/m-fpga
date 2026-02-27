module health_monitor #(parameter data_width = 16)
	(
		input wire clk,
		input wire enable,
		
		input wire reset,
		
		input wire sample_valid,
		
		input wire signed [data_width - 1 : 0] sample_in,
		
		output reg health,
		
		output wire envl_detect,
		output wire peak_detect
	);
	
	reg [data_width - 1 : 0] envelope;
	reg [data_width - 1 : 0] envelope_1;
	reg [data_width - 1 : 0] envelope_2;
	
	reg signed [data_width - 1 : 0] sample_in_r;
	reg [data_width - 1 : 0] abs;
	
	reg abs_valid;
	reg envelope_1_valid;
	reg envelope_2_valid;
	reg envelope_valid;
	
	reg [7:0] peak_ctr;
	reg [7:0] envelope_threshold_ctr;
	
	localparam envelope_threshold = (1 << (data_width - 2));
	localparam envelope_threshold_ctr_threshold = 32;
	localparam peak_ctr_threshold = 10;
	
	assign peak_detect = (peak_ctr > peak_ctr_threshold);
	assign envl_detect = (envelope_threshold_ctr > envelope_threshold_ctr_threshold);
	
	localparam sat_min = -(1 << (data_width - 1));
	localparam sat_max =  (1 << (data_width - 1)) - 1;
	
	always @(posedge clk) begin
		abs_valid <= sample_valid;
		envelope_1_valid <= abs_valid;
		envelope_2_valid <= envelope_1_valid;
		envelope_valid 	 <= envelope_2_valid;
		
		if (reset) begin
			health <= 1;
			peak_ctr <= 0;
			envelope_threshold_ctr <= 0;
			
			abs_valid <= 0;
			envelope_1_valid <= 0;
			envelope_2_valid <= 0;
			envelope_valid 	 <= 0;
		end else if (enable) begin
			if (sample_valid) begin
				abs <= sample_in < 0 ? -sample_in : sample_in;
				
				if (sample_in == sat_min || sample_in == sat_max)
					peak_ctr <= peak_ctr + (peak_detect ? 0 : 1);
				else
					peak_ctr <= 0;
				
				if (peak_detect)
					health <= 0;
			end
			
			if (abs_valid) envelope_1 		 <= (envelope >> 1) + (abs >> 3);
			if (envelope_1_valid) envelope_2 <=  envelope_1 + (envelope >> 2);
			if (envelope_2_valid) envelope   <=  envelope_2 + (envelope >> 3);
			
			if (envelope_valid) begin
				if (envelope > envelope_threshold)
					envelope_threshold_ctr <= envelope_threshold_ctr + (envl_detect ? 0 : 1);
				else
					envelope_threshold_ctr <= 0;
				
				if (envl_detect) health <= 0;
			end
		end
	end
endmodule
