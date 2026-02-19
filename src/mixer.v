`define MIXER_STATE_READY		 		0
`define MIXER_APPLY_INPUT_GAIN_1 		1
`define MIXER_APPLY_INPUT_GAIN_2 		2
`define MIXER_APPLY_INPUT_GAIN_3 		3
`define MIXER_APPLY_INPUT_GAIN_DONE		4
`define MIXER_MIX_PIPELINES_1	 		5
`define MIXER_MIX_PIPELINES_2	 		6
`define MIXER_MIX_PIPELINES_3	 		7
`define MIXER_APPLY_OUTPUT_GAIN_1 		8
`define MIXER_APPLY_OUTPUT_GAIN_2 		9
`define MIXER_APPLY_OUTPUT_GAIN_3 		10
`define MIXER_APPLY_OUTPUT_GAIN_DONE 	11
`define MIXER_REST				 		12

`default_nettype none

module mixer #(parameter data_width = 16, parameter gain_shift = 4) (
		input wire clk,
		input wire reset,

		input wire signed [data_width - 1 : 0] in_sample,
		output reg signed [data_width - 1 : 0] in_sample_out,
		
		input wire signed [data_width - 1 : 0] out_sample_in_a,
		input wire signed [data_width - 1 : 0] out_sample_in_b,
		
		output reg signed [data_width - 1 : 0] out_sample,
		
		input wire [data_width - 1 : 0] data_in,
		
		input wire in_sample_valid,
		input wire out_samples_valid,
		
		output reg in_sample_mixed,
		output reg out_sample_valid,
		
		input wire set_input_gain,
		input wire set_output_gain,
		
		input wire swap_pipelines,
		output reg pipelines_swapping,
		input wire current_pipeline
	);
	
	/* All gains herein are stored as q5.n */
	localparam signed sat_max = {{(data_width + 1){1'b0}}, {(data_width - 1){1'b1}}};
	localparam signed sat_min = {{(data_width + 1){1'b1}}, {(data_width - 1){1'b0}}};
	
	localparam signed sat_max_dw = sat_max[data_width - 1 : 0];
	localparam signed sat_min_dw = sat_min[data_width - 1 : 0];
	
	reg [data_width - 1 : 0] input_gain;
	reg [data_width - 1 : 0] output_a_gain;
	reg [data_width - 1 : 0] output_b_gain;
	reg [data_width - 1 : 0] output_gain;
	
	reg signed [data_width - 1 : 0] mul_arg_aa;
	reg signed [data_width - 1 : 0] mul_arg_ab;
	
	wire signed [2 * data_width - 1 : 0] prod_a = mul_arg_aa * mul_arg_ab;
	
	reg signed [2 * data_width - 1 : 0] prod_a_latched;

	wire signed [2 * data_width - 1 : 0] prod_a_shifted = $signed(prod_a_latched) >>> (data_width - 1 - gain_shift);
	wire signed [2 * data_width - 1 : 0] prod_a_shifted_sat = (prod_a_shifted > sat_max) ? sat_max : ((prod_a_shifted < sat_min) ? sat_min : prod_a_shifted);
	wire signed [	 data_width - 1 : 0] prod_a_final = prod_a_shifted_sat[data_width - 1 : 0];
	
	reg signed [data_width - 1 : 0] mul_arg_ba;
	reg signed [data_width - 1 : 0] mul_arg_bb;
	
	wire signed [2 * data_width - 1 : 0] prod_b = mul_arg_ba * mul_arg_bb;

	reg signed [2 * data_width - 1 : 0] prod_b_latched;
	
	wire signed [2 * data_width - 1 : 0] prod_b_shifted = $signed(prod_b_latched) >>> (data_width - 1 - gain_shift);
	wire signed [2 * data_width - 1 : 0] prod_b_shifted_sat = (prod_b_shifted > sat_max) ? sat_max : ((prod_b_shifted < sat_min) ? sat_min : prod_b_shifted);
	wire signed [	data_width - 1 : 0] prod_b_final = prod_b_shifted_sat[data_width - 1 : 0];
	
	wire signed [data_width - 1 : 0] prod_sum = prod_a_final + prod_b_final;
	wire signed [data_width - 1 : 0] prod_sum_final = (prod_sum > sat_max_dw) ? sat_max_dw : ((prod_sum < sat_min_dw) ? sat_min_dw : prod_sum);
	
	reg [7:0] state = 0;
	
	localparam [data_width - 1 : 0] unity_gain = 1 << (data_width - 1 - gain_shift);
	localparam [data_width - 1 : 0] switch_velocity = unity_gain >> 7;
	
	reg target_pipeline = 0;
	
	reg pipeline_swap_requested = 0;
	
	always @(posedge clk) begin
		in_sample_mixed  <= 0;
		out_sample_valid <= 0;
		
		if (swap_pipelines)
			pipeline_swap_requested <= 1;
		
		if (set_input_gain)
			input_gain <= data_in;

		if (set_output_gain)
			output_gain <= data_in;
		
		if (reset) begin
			pipelines_swapping 	<= 0;
			target_pipeline		<= 0;
			pipeline_swap_requested <= 0;
			
			input_gain  <= 1 << (data_width - 1 - gain_shift);
			output_gain <= 1 << (data_width - 1 - gain_shift);
			
			output_a_gain <= 1 << (data_width - 1 - gain_shift);
			output_b_gain <= 0;
		end
		else begin
			case (state)
				`MIXER_STATE_READY: begin
					if (swap_pipelines || pipeline_swap_requested) begin
						pipelines_swapping <= 1;
						target_pipeline <= ~target_pipeline;
						pipeline_swap_requested <= 0;
					end
					
					if (in_sample_valid) begin
						mul_arg_aa <= in_sample;
						mul_arg_ab <= input_gain;
						in_sample_mixed <= 0;
						state <= `MIXER_APPLY_INPUT_GAIN_1;
						
						if (pipelines_swapping) begin
							if (target_pipeline) begin
								if (output_a_gain == 0) begin
									output_b_gain <= unity_gain;
									output_a_gain <= 0;
									pipelines_swapping <= 0;
								end
								else begin
									output_b_gain <= output_b_gain + switch_velocity;
									output_a_gain <= output_a_gain - switch_velocity;
								end
							end
							else begin
								if (output_b_gain == 0) begin
									output_a_gain <= unity_gain;
									output_b_gain <= 0;
									pipelines_swapping <= 0;
								end
								else begin
									output_a_gain <= output_a_gain + switch_velocity;
									output_b_gain <= output_b_gain - switch_velocity;
								end
							end
						end
					end
					else if (out_samples_valid) begin
						mul_arg_aa <= out_sample_in_a;
						mul_arg_ab <= output_a_gain;
						
						mul_arg_ba <= out_sample_in_b;
						mul_arg_bb <= output_b_gain;
						
						state <= `MIXER_MIX_PIPELINES_1;
					end
				end
				
				`MIXER_APPLY_INPUT_GAIN_1: begin
					state <= `MIXER_APPLY_INPUT_GAIN_2;
				end
				
				`MIXER_APPLY_INPUT_GAIN_2: begin
					prod_a_latched <= prod_a;
					state <= `MIXER_APPLY_INPUT_GAIN_3;
				end

				`MIXER_APPLY_INPUT_GAIN_3: begin
					state <= `MIXER_APPLY_INPUT_GAIN_DONE;
				end

				`MIXER_APPLY_INPUT_GAIN_DONE: begin
					in_sample_out <= prod_a_final;
					in_sample_mixed <= 1;
					state <= `MIXER_REST;
				end
				
				`MIXER_MIX_PIPELINES_1: begin
					state <= `MIXER_MIX_PIPELINES_2;
				end
				
				`MIXER_MIX_PIPELINES_2: begin
					prod_a_latched <= prod_a;
					prod_b_latched <= prod_b;
					state <= `MIXER_MIX_PIPELINES_3;
				end

				`MIXER_MIX_PIPELINES_3: begin
					state <= `MIXER_APPLY_OUTPUT_GAIN_1;
				end

				`MIXER_APPLY_OUTPUT_GAIN_1: begin
					mul_arg_aa <= prod_sum_final;
					mul_arg_ab <= output_gain;
					state <= `MIXER_APPLY_OUTPUT_GAIN_2;
				end
				
				`MIXER_APPLY_OUTPUT_GAIN_2: begin
					state <= `MIXER_APPLY_OUTPUT_GAIN_3;
				end
				
				`MIXER_APPLY_OUTPUT_GAIN_3: begin
					prod_a_latched <= prod_a;
					state <= 11;
				end

				`MIXER_APPLY_OUTPUT_GAIN_DONE: begin
					out_sample <= prod_a_final;
					out_sample_valid <= 1;
					state <= `MIXER_REST;
				end
				
				`MIXER_REST: begin
					state <= `MIXER_STATE_READY;
				end
			endcase
		end
	end
	
endmodule

`default_nettype wire
