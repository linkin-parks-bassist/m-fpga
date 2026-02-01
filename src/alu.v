module dsp_core_alu #(parameter integer data_width = 16, parameter interp_bits = 8)
	(
		input wire clk,
		input wire reset,
		
		input wire trigger,
		
		input wire [7:0] op,
		input wire signed [data_width - 1 : 0] a,
		input wire signed [data_width - 1 : 0] b,
		input wire signed [data_width - 1 : 0] c,
		
		input wire signed [2 * data_width - 1 : 0] a_wide,
		input wire signed [2 * data_width - 1 : 0] b_wide,
		
		input wire [$clog2(data_width) - 1 : 0] shift,
		
		input wire no_shift,
		input wire saturate,
		
		output reg signed [    data_width - 1 : 0] result,
		output reg signed [2 * data_width - 1 : 0] result_wide,
		
		output reg result_valid,
		output reg ready
	);
	
	localparam signed sat_max = {{(data_width + 1){1'b0}}, {(data_width - 1){1'b1}}};
	localparam signed sat_min = {{(data_width + 1){1'b1}}, {(data_width - 1){1'b0}}};
	
	localparam signed sat_max_trunc = {1'b0, {(data_width - 1){1'b1}}};
	localparam signed sat_min_trunc = {1'b1, {(data_width - 1){1'b0}}};
	
	reg  interp_start;
	wire interp_ready;
	
	wire signed [data_width  - 1 : 0] interpolated;
	
	// Interpolator unit
	sequential_interp #(.data_width(data_width), .interp_bits(interp_bits)) interp
	(
		.clk(clk),
		.reset(reset),
		
		.start(interp_start),
		.ready(interp_ready),
		
		.base(a_latched),
		.target(b_latched),
		.frac(c_latched[data_width - 2 : data_width - interp_bits - 1]),
		.interpolated(interpolated)
	);
	
	reg [7:0] op_latched;
	
	reg  signed [data_width - 1 : 0] a_latched;
	reg  signed [data_width - 1 : 0] b_latched;
	reg  signed [data_width - 1 : 0] c_latched;
	
	reg  signed [2 * data_width - 1 : 0] a_wide_latched;
	wire signed [2 * data_width - 1 : 0] a_wide_latched_trunc = a_wide_latched[data_width - 1 : 0];
	
	reg  signed [2 * data_width - 1 : 0] b_wide_latched;
	wire signed [2 * data_width - 1 : 0] b_wide_latched_trunc = b_wide_latched[data_width - 1 : 0];
	
	reg  [$clog2(data_width) - 1 : 0] shift_latched;
	wire [$clog2(data_width) - 1 : 0] shift_current;
	
	reg  saturate_latched;
	reg  no_shift_latched;
	
	wire signed [    data_width - 1 : 0] sum_result 		= a_latched + b_latched;
	wire signed [2 * data_width - 1 : 0] sum_wide_result 	= a_wide_latched + {{(data_width){b_latched[data_width - 1]}}, b_latched};
	wire signed [2 * data_width - 1 : 0] sum_wide_wide		= a_wide_latched + b_wide_latched;
	wire signed [    data_width - 1 : 0] sub_result 		= a_latched - b_latched;
	wire signed [2 * data_width - 1 : 0] sub_wide_result 	= a_wide_latched - {{(data_width){b_latched[data_width - 1]}}, b_latched};
	wire signed [2 * data_width - 1 : 0] mul_result 		= a_latched * b_latched;
	wire signed [    data_width - 1 : 0] lsh1_result 		= a_latched << 1;
	wire signed [    data_width - 1 : 0] rsh1_result   		= a_latched >> 1;
	wire signed [    data_width - 1 : 0] arsh1_result  		= a_latched >>> 1;
	wire signed [    data_width - 1 : 0] lsh4_result 		= a_latched << 4;
	wire signed [    data_width - 1 : 0] rsh4_result   		= a_latched >> 4;
	wire signed [    data_width - 1 : 0] arsh4_result  		= a_latched >>> 4;
	wire signed [    data_width - 1 : 0] lsh8_result 		= a_latched << 8;
	wire signed [    data_width - 1 : 0] rsh8_result   		= a_latched >> 8;
	wire signed [    data_width - 1 : 0] arsh8_result  		= a_latched >>> 8;
	wire signed [2 * data_width - 1 : 0] arsh1_wide_result  = a_wide_latched >>> 1;
	wire signed [2 * data_width - 1 : 0] arsh4_wide_result  = a_wide_latched >>> 4;
	wire signed [2 * data_width - 1 : 0] arsh8_wide_result  = a_wide_latched >>> 8;
	wire signed [    data_width - 1 : 0] max_result 		= (a_latched > b_latched) ? a_latched : b_latched;
	
	wire signed [    data_width     : 0] sum 	 = a + b;
	wire signed [    data_width - 1 : 0] sum_sat = (saturate) ? ((sum > sat_max) ? sat_max : ((sum < sat_min) ? sat_min : sum)) : sum;
	wire signed [2 * data_width - 1 : 0] product = a * b;
	wire signed [    data_width - 1 : 0] sub	 = a - b;
	wire signed [    data_width - 1 : 0] max   	 = (a > b) ? a : b;
	wire signed [    data_width - 1 : 0] min   	 = (a < b) ? a : b;
	wire signed [2 * data_width - 1 : 0] sat   	 = (a_wide > sat_max) ? sat_max : ((a_wide < sat_min) ? sat_min : a_wide);
	wire signed [    data_width - 1 : 0] abs   	 = (a < 0) ? -a : a;
	wire signed [    data_width - 1 : 0] clamp   = (a < b) ? b : ((a > c) ? c : a);
	
	reg [7:0] state;
	reg [7:0] ret_state;
	reg [7:0] ctr;
	
	reg multiply;
	reg add;
	reg wide;
	reg shift_right;
	reg shift_arithmetic;
	reg ret_wide;
	reg interp_wait;
	
	wire [    data_width - 1 : 0] shifts[16:0];
	wire [2 * data_width - 1 : 0] shifts_wide[2:0];
	
	wire [$clog2(data_width) - 1 : 0] shift_decrement = (shift_latched >= 8) ? 8 : ((shift_latched >= 4) ? 4 : 1);
	wire [3:0] shift_index = {shift_arithmetic, shift_right, shift_latched >= 8, shift_latched >= 4};
	
	assign shifts[4'b0000] =  lsh1_result;
	assign shifts[4'b0001] =  lsh4_result;
	assign shifts[4'b0011] =  lsh8_result;
	assign shifts[4'b0100] =  rsh1_result;
	assign shifts[4'b0101] =  rsh4_result;
	assign shifts[4'b0111] =  rsh8_result;
	assign shifts[4'b1100] = arsh1_result;
	assign shifts[4'b1101] = arsh4_result;
	assign shifts[4'b1111] = arsh8_result;
	
	assign shifts_wide[0] = arsh1_result;
	assign shifts_wide[1] = arsh4_result;
	assign shifts_wide[2] = arsh8_result;
	
	always @(posedge clk) begin
		result_valid <= 0;
		interp_start <= 0;
		interp_wait  <= 0;
		
		if (reset) begin
			ready <= 1;
			state <= `ALU_STATE_IDLE;
		end else begin
			if ((trigger && ready) || !ready) begin
				if (trigger && ready) begin
					ready <= 0;
					ctr <= 0;
					
					op_latched <= op;
					a_latched  <= a;
					b_latched  <= b;
					c_latched  <= c;
					
					a_wide_latched <= a_wide;
					b_wide_latched <= b_wide;
					
					shift_latched <= shift;
					
					no_shift_latched <= no_shift;
					saturate_latched <= saturate;
					
					// Skip degenerate cases
					if ((op == `ALU_OP_LSH || op == `ALU_OP_RSH) && b > data_width - 1) begin
						result <= 0;
						ready  <= 1;
						result_valid <= 1;
						state <= `ALU_STATE_IDLE;
					end else if (op == `ALU_OP_ARSH && b > data_width - 1) begin
						result <= a[data_width - 1];
						ready <= 1;
						result_valid <= 1;
						state <= `ALU_STATE_IDLE;
					end else if (op == `ALU_OP_ARSH_WIDE && b > 2 * data_width - 1) begin
						result_wide <= {(2*data_width){a_wide[2 * data_width - 1]}};
						ready <= 1;
						result_valid <= 1;
						state <= `ALU_STATE_IDLE;
					end else begin
						case (op)
							`ALU_OP_LSH: begin
								state <= `ALU_STATE_SHIFT;
								ret_state <= `ALU_STATE_DONE;
								
								shift_latched <= b;
								
								wide <= 0;
								ret_wide <= 0;
								shift_right <= 0;
								shift_arithmetic <= 0;
							end
							
							`ALU_OP_RSH:  begin
								state <= `ALU_STATE_SHIFT;
								ret_state <= `ALU_STATE_DONE;
								
								shift_latched <= b;
								
								wide <= 0;
								ret_wide <= 0;
								shift_right <= 1;
								shift_arithmetic <= 0;
							end
							
							`ALU_OP_ARSH:  begin
								state <= `ALU_STATE_SHIFT;
								ret_state <= `ALU_STATE_DONE;
								
								shift_latched <= b;
								
								wide <= 0;
								ret_wide <= 0;
								shift_right <= 1;
								shift_arithmetic <= 1;
							end
							
							`ALU_OP_ARSH_WIDE:  begin
								state <= `ALU_STATE_SHIFT;
								ret_state <= `ALU_STATE_DONE;
								
								shift_latched <= b;
								
								wide <= 1;
								ret_wide <= 1;
								shift_right <= 1;
								shift_arithmetic <= 1;
							end
							
							`ALU_OP_MUL: begin
								a_wide_latched <= product;
								b_latched <= c;
								
								wide <= 1;
								ret_wide <= 0;
								shift_right <= 1;
								shift_arithmetic <= 1;
								
								if (no_shift) begin
									if (saturate) 
										state <= `ALU_STATE_SATURATE;
									else
										state <= `ALU_STATE_DONE;
								end else begin
									state <= `ALU_STATE_SHIFT;
									if (saturate) ret_state <= `ALU_STATE_SATURATE;
									else ret_state <= `ALU_STATE_DONE;
								end
							end
							
							`ALU_OP_MADD: begin
								a_wide_latched <= product;
								b_latched <= c;
								
								wide <= 1;
								ret_wide <= 0;
								shift_right <= 1;
								shift_arithmetic <= 1;
								
								if (no_shift) begin
									state <= `ALU_STATE_ACC;
								end else begin
									state <= `ALU_STATE_SHIFT;
									ret_state <= `ALU_STATE_ACC;
								end
							end
							
							`ALU_OP_MAC: begin
								a_wide_latched <= product;
								b_latched <= c;
								
								wide <= 1;
								ret_wide <= 1;
								shift_right <= 1;
								shift_arithmetic <= 1;
								
								if (no_shift) begin
									state <= `ALU_STATE_ACC;
								end else begin
									state <= `ALU_STATE_SHIFT;
									ret_state <= `ALU_STATE_ACC;
								end
							end
							
							`ALU_OP_LINTERP: begin
								state <= `ALU_STATE_INTERP;
								interp_start <= 1;
								interp_wait  <= 1;
							end
						endcase
					end
				end
				else begin
					ctr <= ctr + 1;
				end
				
				case (state)
					`ALU_STATE_SHIFT: begin
						if (wide) begin
							if (shift_latched >= 8) begin
								a_wide_latched <= arsh8_wide_result;
								
								if (shift_latched == 8) state <= ret_state;
								else shift_latched <= shift_latched - 8;
							end else if (shift_latched >= 4) begin
								a_wide_latched <= arsh4_wide_result;
								
								if (shift_latched == 4) state <= ret_state;
								else shift_latched <= shift_latched - 4;
							end else if (shift_latched > 0) begin
								a_wide_latched <= arsh1_wide_result;
								
								if (shift_latched == 1) state <= ret_state;
								else shift_latched <= shift_latched - 1;
							end else begin
								state <= ret_state;
							end
						end else begin
							a_latched <= shifts[shift_index];
							
							if (shift_latched == 8 || shift_latched == 4 || shift_latched == 1) begin
								state <= ret_state;
							end else begin
								shift_latched <= shift_latched - shift_decrement;
							end
						end
					end
						
					`ALU_STATE_ACC: begin
						if (op_latched == `ALU_OP_MAC) begin
							a_wide_latched <= sum_wide_wide;
							if (saturate) state <= `ALU_STATE_SATURATE;
							else state <= `ALU_STATE_DONE;
						end else begin
							if (wide) begin
								a_wide_latched <= sum_wide_result;
								if (saturate) state <= `ALU_STATE_SATURATE;
								else state <= `ALU_STATE_DONE;
							end else begin 
								a_latched <= sum_result;
								state <= `ALU_STATE_DONE;
							end
						end
					end
					
					`ALU_STATE_SATURATE: begin
						a_wide_latched <= (a_wide_latched > sat_max) ? sat_max : ((a_wide_latched < sat_min) ? sat_min : a_wide_latched);
						state <= `ALU_STATE_DONE;
					end
					
					`ALU_STATE_INTERP: begin
						if (!interp_wait && interp_ready) begin
							result <= interpolated;
							
							ready <= 1;
							result_valid <= 1;
							state <= `ALU_STATE_IDLE;
						end
					end
					
					`ALU_STATE_DONE: begin
						if (wide && ret_wide) result_wide <= a_wide_latched;
						else if (wide) result <= a_wide_latched_trunc;
						else result <= a_latched;
						
						ready <= 1;
						result_valid <= 1;
						state <= `ALU_STATE_IDLE;
					end
				endcase
			end else begin
				case (op)
					`ALU_OP_ADD:   result 		<= sum_sat;
					`ALU_OP_SUB:   result 		<= sub; 
					`ALU_OP_MUL:   result_wide  <= product;
					`ALU_OP_MIN:   result	 	<= min;
					`ALU_OP_MAX:   result  		<= max;
					`ALU_OP_ABS:   result 		<= abs;
					`ALU_OP_SAT:   result		<= sat;
					`ALU_OP_CLAMP: result_wide  <= clamp;
				endcase
			end
		end
	end
endmodule