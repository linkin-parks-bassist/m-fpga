`include "block.vh"
`include "lut.vh"

module pipeline_block
	#(
		parameter integer data_width 		= 16,
		parameter integer n_channels   		= 16,
		parameter integer n_registers  		= 16
	)
	(
		input wire clk,
		
		input wire tick,
		input wire reset,
		
		input  wire [data_width - 1 : 0] ch_in  [0 : n_channels - 1],
		output wire [data_width - 1 : 0] ch_out [0 : n_channels - 1],
		
		input wire [`BLOCK_INSTR_WIDTH - 1 : 0] instr_val,
		input wire instr_write,
		
		input wire [data_width - 1 		: 0] reg_val,
		input wire [$clog2(n_registers) - 1 : 0] reg_target,
		input wire reg_write,
		input wire reg_update,
		
		output reg [data_width - 1 : 0] mul_req_a,
		output reg [data_width - 1 : 0] mul_req_b,
		
		input wire signed [2 * data_width - 1 : 0] mul_result,
		input wire mul_done,
		
		output reg done,
		
		output wire lut_req,
		output reg signed [`LUT_HANDLE_WIDTH - 1 : 0] lut_handle,
		output reg signed [data_width - 1 : 0] lut_arg,
		input wire signed [data_width - 1 : 0] lut_data,
		input wire lut_ready,
		
		output reg delay_read_req,
		output reg delay_write_req,
		output reg signed [data_width - 1 : 0] delay_buf_handle,
		output reg signed [data_width - 1 : 0] delay_buf_data_out,
		input wire signed [data_width - 1 : 0] delay_buf_data_in,
		input wire delay_buf_read_ready,
		input wire delay_buf_write_ready,
		input wire delay_buf_invalid_read,
		input wire delay_buf_invalid_write
	);
	
	genvar j;
	generate
		for (j = 0; j < n_channels; j = j + 1) begin
			assign ch_out[j] = ch_regs[j];
		end
	endgenerate
	
	//
	// Block instruction
	//
	reg  [`BLOCK_INSTR_WIDTH 	- 1 : 0] instr;
	
	//
	// Instruction decoding
	//
	wire [`BLOCK_INSTR_OP_WIDTH - 1 : 0] operation = instr[`BLOCK_INSTR_OP_WIDTH - 1 : 0];
	
	wire [`BLOCK_REG_ADDR_WIDTH - 1 : 0] src_a = instr[1 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH - 1 : 0 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH];
	wire [`BLOCK_REG_ADDR_WIDTH - 1 : 0] src_b = instr[2 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH - 1 : 1 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH];
	wire [`BLOCK_REG_ADDR_WIDTH - 1 : 0] src_c = instr[3 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH - 1 : 2 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH];
	wire [`BLOCK_REG_ADDR_WIDTH - 1 : 0] dest  = instr[4 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH - 1 : 3 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH];
	
	localparam operand_type_start_index = 4 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH;
	
	wire src_a_reg = instr[operand_type_start_index + 0];
	wire src_b_reg = instr[operand_type_start_index + 1];
	wire src_c_reg = instr[operand_type_start_index + 2];
	wire dest_reg  = instr[operand_type_start_index + 3];
	
	localparam pms_start_index = operand_type_start_index + 4;
	
	//
	// Local storage: channels and registers
	//
	reg signed [data_width  - 1 : 0] ch_regs [0 : n_channels  - 1];
	reg signed [data_width	- 1 : 0] regs 	 [0 : n_registers - 1];
	reg signed [data_width	- 1 : 0] reg_updates[0 : n_registers - 1];
	reg reg_updated[0 : n_registers - 1];
	
	wire signed [data_width - 1 : 0] unified_regs [0 : n_channels + n_registers - 1];
	
	genvar k;
	generate
		for (k = 0; k < n_channels + n_registers; k = k + 1) begin
			if (k < n_channels) begin
				assign unified_regs[k] = ch_regs[k];
			end
			else begin
				assign unified_regs[k] = regs[k - n_channels];
			end
		end
	endgenerate
	
	localparam int UNIFIED_W = $clog2(n_channels + n_registers);
	
	wire [$clog2(n_channels + n_registers) - 1 : 0] src_a_unified_addr =
		src_a_reg ? UNIFIED_W'(src_a) + UNIFIED_W'(n_channels) : UNIFIED_W'(src_a);
	wire [$clog2(n_channels + n_registers) - 1 : 0] src_b_unified_addr =
		src_b_reg ? UNIFIED_W'(src_b) + UNIFIED_W'(n_channels) : UNIFIED_W'(src_b);
	wire [$clog2(n_channels + n_registers) - 1 : 0] src_c_unified_addr =
		src_c_reg ? UNIFIED_W'(src_c) + UNIFIED_W'(n_channels) : UNIFIED_W'(src_c);
	wire [$clog2(n_channels + n_registers) - 1 : 0] dest_unified_addr =
		dest_reg ? UNIFIED_W'(dest) + UNIFIED_W'(n_channels) : UNIFIED_W'(dest);
	
	wire signed [data_width - 1 : 0] src_a_val = unified_regs[src_a_unified_addr];
	wire signed [data_width - 1 : 0] src_b_val = unified_regs[src_b_unified_addr];
	wire signed [data_width - 1 : 0] src_c_val = unified_regs[src_c_unified_addr];
	
	task automatic unified_write;
		input bit is_reg;
		input [UNIFIED_W-1:0] uaddr;
		input signed [data_width-1:0] value;
	begin
		if (is_reg)
			regs[4'(uaddr - UNIFIED_W'(n_channels))] <= value;
		else
			ch_regs[4'(uaddr)] <= value;
	end
	endtask
	
	task automatic write_dest;
		input signed [data_width-1:0] value;
	begin
		if (dest_reg)
			regs[4'(dest - UNIFIED_W'(n_channels))] <= value;
		else
			ch_regs[4'(dest)] <= value;
	end
	endtask
	
	reg signed [data_width - 1 : 0] src_a_latched;
	reg signed [data_width - 1 : 0] src_b_latched;
	reg signed [data_width - 1 : 0] src_c_latched;
	
	// Explicitly sized saturation limits in FULL multiply width
	localparam signed sat_max = {{(data_width + 1){1'b0}}, {(data_width - 1){1'b1}}};
	localparam signed sat_min = {{(data_width + 1){1'b1}}, {(data_width - 1){1'b0}}};
	
	localparam signed sat_max_q3_29 = sat_max << 14;
	localparam signed sat_min_q3_29 = sat_min << 14;
	
	localparam signed sat_max_dwe = sat_max[data_width : 0];
	localparam signed sat_min_dwe = sat_min[data_width : 0];
	
	localparam signed sat_max_dw = sat_max[data_width - 1 : 0];
	localparam signed sat_min_dw = sat_min[data_width - 1 : 0];
	
	//
	// Local adder
	//
	reg signed [data_width - 1 : 0] summand_a = 0;
	reg signed [data_width - 1 : 0] summand_b = 0;
	
	wire signed [data_width:0] summand_a_ext =
		{ summand_a[data_width-1], summand_a };

	wire signed [data_width:0] summand_b_ext =
		{ summand_b[data_width-1], summand_b };

	// Async perform addition on given summands. Saturate in procedural block
	wire signed [data_width:0] sum_ext = summand_a_ext + summand_b_ext;
	
	wire [data_width - 1 : 0] sum = (sum_ext > sat_max_dwe) ? sat_max_dw : ((sum_ext < sat_min_dwe) ? sat_min_dw : sum_ext[data_width-1:0]);
	
	//
	// Saturation and format compensation for muls
	//
	localparam integer MAX_SHIFT = 2 * data_width - 2;
	localparam integer SHIFT_WIDTH = $clog2(MAX_SHIFT + 1);
	
	wire [SHIFT_WIDTH - 1 : 0] instr_shift =
		(operation == `BLOCK_INSTR_BIQ_DF1) ? {{(SHIFT_WIDTH - 1){1'b0}}, 1'b1} :
		{{(SHIFT_WIDTH - `BLOCK_PMS_WIDTH){1'b0}}, instr[pms_start_index + `BLOCK_PMS_WIDTH - 1 : pms_start_index]};

	// Arithmetic shift
	wire signed [2 * data_width - 1 : 0] mul_result_shifted =
		$signed(mul_result) >>> (15 - instr_shift);

	// Final saturation and narrowing
	wire signed [data_width-1:0] mul_result_final =
		(mul_result_shifted > sat_max) ?  sat_max[data_width-1:0] :
		(mul_result_shifted < sat_min) ?  sat_min[data_width-1:0] :
							   mul_result_shifted[data_width-1:0];
	
	reg signed [data_width - 1 : 0] prod;
	
	reg wait_one = 0;
	
	wire mul_ready = ~wait_one & mul_done;
	
	reg [8:0] state = 0;
	
	reg [data_width - 1 : 0] result;

	reg  signed [2 * data_width - 1 : 0] accumulator;
	
	reg  signed [2 * data_width - 1 : 0] acc_summand;
	wire signed [2 * data_width - 1 : 0] acc_sum = accumulator + acc_summand;
	
	wire signed [2 * data_width - 1 : 0] acc_q3_29_sat = (accumulator > sat_max_q3_29) ? sat_max_q3_29 : ((accumulator < sat_min_q3_29) ? sat_min_q3_29 : accumulator);
	
	integer i;
	always @(posedge clk) begin
	
		if (done) begin
			result <= ch_out[0];
		end
	
		//
		// Accept any register writes
		//
		if (reg_write) begin
			regs[reg_target] <= reg_val;
		end
		else if (reg_update) begin
			reg_updates[reg_target] <= reg_val;
			reg_updated[reg_target] <= 1;
		end
		
		//
		// Accept new instruction
		//
		if (instr_write) begin
			instr <= instr_val;
		end
		
		//
		// On tick: shift in channels from prev stage
		//
		if (tick) begin
			for (i = 0; i < n_channels; i++) begin
				ch_regs[i] <= ch_in[i];
			end
			
			for (i = 0; i < n_registers; i++) begin
				if (reg_updated[i]) begin
					if (reg_updates[i] < regs[i]) begin
						regs[i] <= regs[i] - 1;
					end
					else if (reg_updates[i] > regs[i]) begin
						regs[i] <= regs[i] + 1;
					end
					else begin
						reg_updated[i] <= 0;
					end
				end
			end
			
			state <= `BLOCK_STATE_BEGIN;
			done <= 0;
		end
		else if (!done) begin
			case (operation)
				`BLOCK_INSTR_NOP: begin
					state <= `BLOCK_STATE_DONE;
					done <= 1;
				end
				
				`BLOCK_INSTR_ADD: begin
					case (state)
						`BLOCK_STATE_BEGIN: begin
							summand_a <=  src_a_val;
							summand_b <= -src_b_val;
							state <= `BLOCK_STATE_ADD_WAIT;
						end
						
						`BLOCK_STATE_ADD_WAIT: begin
							state <= `BLOCK_STATE_ADD_STORE;
						end
						
						`BLOCK_STATE_ADD_STORE: begin
							unified_write(dest_reg, dest_unified_addr, sum);
							
							state <= `BLOCK_STATE_DONE;
							done <= 1;
						end
						
						default: begin end
					endcase
					
					done <= 1;
				end
				
				`BLOCK_INSTR_SUB: begin
					case (state)
						`BLOCK_STATE_BEGIN: begin
							summand_a <=  src_a_val;
							summand_b <= -src_b_val;
							state <= `BLOCK_STATE_SUB_WAIT;
						end
						
						`BLOCK_STATE_SUB_WAIT: begin
							state <= `BLOCK_STATE_SUB_STORE;
						end
						
						`BLOCK_STATE_SUB_STORE: begin
							unified_write(dest_reg, dest_unified_addr, sum);
							
							state <= `BLOCK_STATE_DONE;
							done <= 1;
						end
						
						default: begin
							state <= `BLOCK_STATE_DONE;
							done <= 1;
						end
					endcase
					
					done <= 1;
				end
				
				`BLOCK_INSTR_LSH: begin
					unified_write(dest_reg, dest_unified_addr, src_a_val << 1);
					state <= `BLOCK_STATE_DONE;
					done <= 1;
				end
				
				`BLOCK_INSTR_RSH: begin
					unified_write(dest_reg, dest_unified_addr, src_a_val >> 1);
					state <= `BLOCK_STATE_DONE;
					done <= 1;
				end
				
				`BLOCK_INSTR_RSH: begin
					unified_write(dest_reg, dest_unified_addr, src_a_val >>> 1);
					state <= `BLOCK_STATE_DONE;
					done <= 1;
				end
				
				`BLOCK_INSTR_MUL: begin
					case (state)
						`BLOCK_STATE_BEGIN: begin
							mul_req_a <= src_a_reg ? regs[src_a] : ch_regs[src_a];
							mul_req_b <= src_b_reg ? regs[src_b] : ch_regs[src_b];
							
							wait_one <= 1;
							state <= `BLOCK_STATE_MUL_WAIT;
						end
						
						`BLOCK_STATE_MUL_WAIT: begin
							if (mul_ready) begin
								prod  <= mul_result_final;
								state <= `BLOCK_STATE_MUL_STORE;
							end
							wait_one <= 0;
						end
						
						`BLOCK_STATE_MUL_STORE: begin
							if (dest_reg) begin
								regs[dest] <= prod;
							end
							else begin
								ch_regs[dest] <= prod;
							end
							
							state <= `BLOCK_STATE_DONE;
							done <= 1;
						end
						
						default: begin end
					endcase
				end
				
				`BLOCK_INSTR_MAC: begin
					case (state)
						`BLOCK_STATE_BEGIN: begin
							mul_req_a <= src_a_reg ? regs[src_a] : ch_regs[src_a];
							mul_req_b <= src_b_reg ? regs[src_b] : ch_regs[src_b];
							
							wait_one <= 1;
							state <= `BLOCK_STATE_MAC_MUL_WAIT;
						end
						
						`BLOCK_STATE_MAC_MUL_WAIT: begin
							if (mul_ready) begin
								summand_a  <= mul_result_final;
								summand_b <= src_c_reg ? regs[src_c] : ch_regs[src_c];
								state <= `BLOCK_STATE_MAC_ADD_WAIT;
							end
							wait_one <= 0;
						end
						
						`BLOCK_STATE_MAC_ADD_WAIT: begin
							state <= `BLOCK_STATE_MAC_STORE;
						end
						
						`BLOCK_STATE_MAC_STORE: begin
							if (dest_reg) begin
								regs[dest] <= sum;
							end
							else begin
								ch_regs[dest] <= sum;
							end
							
							state <= `BLOCK_STATE_DONE;
							done <= 1;
						end
						
						default: begin end
					endcase
				end
				
				`BLOCK_INSTR_ABS: begin
					if (src_a_reg) begin
						if (dest_reg) begin
							regs[dest] <= regs[src_a] < 0 ? -regs[src_a] : regs[src_a];
						end
						else begin
							ch_regs[dest] <= regs[src_a] < 0 ? -regs[src_a] : regs[src_a];
						end
					end
					else begin
						if (dest_reg) begin
							regs[dest] <= ch_regs[src_a] < 0 ? -ch_regs[src_a] : ch_regs[src_a];
						end
						else begin
							ch_regs[dest] <= ch_regs[src_a] < 0 ? -ch_regs[src_a] : ch_regs[src_a];
						end
					end
					
					done <= 1;
				end
				
				//y[n] = b0*x[n] + b1*x[n−1] + b2*x[n−2] − a1*y[n−1] − a2*y[n−2]
				`BLOCK_INSTR_BIQ_DF1: begin
					case (state)
						`BLOCK_STATE_BEGIN: begin
							src_a_latched <= src_a_val;
							
							// src b points to the start of the coefficients
							// so that one can use channels as coefficients,
							// for dynamically changing filters
							regs[4] <= unified_regs[src_b_unified_addr + 0];
							regs[5] <= unified_regs[src_b_unified_addr + 1];
							regs[6] <= unified_regs[src_b_unified_addr + 2];
							regs[7] <= unified_regs[src_b_unified_addr + 3];
							regs[8] <= unified_regs[src_b_unified_addr + 4];
						
							mul_req_a <= src_a_val;
							mul_req_b <= unified_regs[src_b_unified_addr + 0];
							
							wait_one <= 1;
							state <= `BLOCK_STATE_BIQUAD_DF1_MUL_1_WAIT;
						end
					
						`BLOCK_STATE_BIQUAD_DF1_MUL_1_WAIT: begin
							if (mul_ready) begin
								accumulator <= mul_result;
								
								mul_req_a <= regs[0];
								mul_req_b <= regs[5];
								
								wait_one <= 1;
								state <= `BLOCK_STATE_BIQUAD_DF1_MAC_1_MUL_WAIT;
							end
							else begin
								wait_one <= 0;
							end
						end
						
						`BLOCK_STATE_BIQUAD_DF1_MAC_1_MUL_WAIT: begin
							if (mul_ready) begin
								acc_summand <= mul_result;
								
								state <= `BLOCK_STATE_BIQUAD_DF1_MAC_1_ACCUMULATE;
							end
							wait_one <= 0;
						end
						
						`BLOCK_STATE_BIQUAD_DF1_MAC_1_ACCUMULATE: begin
							accumulator <= acc_sum;
							
							mul_req_a <= regs[1];
							mul_req_b <= regs[6];
							
							wait_one <= 1;
							state <= `BLOCK_STATE_BIQUAD_DF1_MAC_2_MUL_WAIT;
						end
						
						`BLOCK_STATE_BIQUAD_DF1_MAC_2_MUL_WAIT: begin
							if (mul_ready) begin
								acc_summand <= mul_result;
								
								state <= `BLOCK_STATE_BIQUAD_DF1_MAC_2_ACCUMULATE;
							end
							wait_one <= 0;
						end
						
						`BLOCK_STATE_BIQUAD_DF1_MAC_2_ACCUMULATE: begin
							accumulator <= acc_sum;
							
							mul_req_a <= regs[2];
							mul_req_b <= regs[7];
							
							wait_one <= 1;
							state <= `BLOCK_STATE_BIQUAD_DF1_MAC_3_MUL_WAIT;
						end
						
						`BLOCK_STATE_BIQUAD_DF1_MAC_3_MUL_WAIT: begin
							if (mul_ready) begin
								acc_summand <= -mul_result;
								state <= `BLOCK_STATE_BIQUAD_DF1_MAC_3_ACCUMULATE;
							end
							wait_one <= 0;
						end
						
						`BLOCK_STATE_BIQUAD_DF1_MAC_3_ACCUMULATE: begin
							accumulator <= acc_sum;
							
							mul_req_a <= regs[3];
							mul_req_b <= regs[8];
							
							wait_one <= 1;
							state <= `BLOCK_STATE_BIQUAD_DF1_MAC_4_MUL_WAIT;
						end
						
						`BLOCK_STATE_BIQUAD_DF1_MAC_4_MUL_WAIT: begin
							if (mul_ready) begin
								acc_summand <= -mul_result;
								
								state <= `BLOCK_STATE_BIQUAD_DF1_MAC_4_ACCUMULATE;
							end
							wait_one <= 0;
						end
						
						`BLOCK_STATE_BIQUAD_DF1_MAC_4_ACCUMULATE: begin
							accumulator <= acc_sum;
							state <= `BLOCK_STATE_BIQUAD_DF1_STORE;
						end
						
						`BLOCK_STATE_BIQUAD_DF1_STORE: begin
							write_dest({$signed(acc_q3_29_sat) >>> 14}[data_width - 1 : 0]);
							
							regs[1] <= regs[0];
							regs[0] <= src_a_latched;
							
							regs[3] <= regs[2];
							regs[2] <= {$signed(acc_q3_29_sat) >>> 14}[data_width - 1 : 0];
							
							state <= `BLOCK_STATE_DONE;
							done <= 1;
						end
					endcase
				end
				
				`BLOCK_INSTR_LUT: begin
					case (state)
						`BLOCK_STATE_BEGIN: begin
							lut_arg 	<= ((src_a_val << instr_shift) & ((1 << data_width) - 1)) | ((data_width)'(src_a_val[15]) << data_width);
							lut_handle 	<= src_b_val[`LUT_HANDLE_WIDTH - 1 : 0];
							lut_req 	<= 1;
							state 		<= `BLOCK_STATE_LUT_WAIT1;
						end
						
						`BLOCK_STATE_LUT_WAIT1: begin
							state <= `BLOCK_STATE_LUT_WAIT2;
						end
						
						`BLOCK_STATE_LUT_WAIT2: begin
							if (lut_ready) begin
								write_dest(lut_data);
								lut_req <= 0;
								state <= `BLOCK_STATE_DONE;
								done <= 1;
							end
						end
					endcase
				end
				
				// For the envelope follower,
				// an akwardness is this:
				// I want to compute a*e + (1-a)*|x|.
				// Clearly, a will be stored in regs[0]
				// by the controller, as it is a parameter.
				// On the other hand, 1-a could be computed locally
				// However this brings up problems of representation
				// a will be represented as a q1.n as it is small and
				// this is the same format as x. However, I cannot then go
				// and compute 1-a conveniently; first of all, 1.0 is not
				// representable as a q1.n. Therefor I cannot just
				// send (1, -a) to the in-block adder. I *could* go
				// ahead and compute 1-a right here using q2.n, but this
				// would 1. be awkward, and 2. invoke extra silicon for
				// the n+2-bit adder, which seems like a waste
				//
				// Therefore I mandate that the MCU provide a pre-computed
				// 1-a, stored in regs[1]. I assume here that that has
				// been done to satisfaction, and get on with it, storing
				// the current value in regs[2].
				`BLOCK_INSTR_ENVD: begin
					case (state)
						`BLOCK_STATE_BEGIN: begin
							mul_req_a <= regs[2];
							mul_req_b <= regs[0];
							
							wait_one <= 1;
							state <= `BLOCK_STATE_ENVD_MUL_1_WAIT;
						end
						
						`BLOCK_STATE_ENVD_MUL_1_WAIT: begin
							if (mul_ready) begin
								regs[3] <= mul_result_final;
								
								mul_req_a <= src_a_val[data_width - 1] ? -src_a_val : src_a_val;
								mul_req_b <= regs[1];
								
								wait_one <= 1;
								state <= `BLOCK_STATE_ENVD_MUL_2_WAIT;
							end
							wait_one <= 0;
						end
						
						`BLOCK_STATE_ENVD_MUL_2_WAIT: begin
							if (mul_ready) begin
								summand_a <= mul_result_final;
								summand_b <= regs[3];
								
								state <= `BLOCK_STATE_ENVD_ADD_WAIT;
							end
							wait_one <= 0;
						end
						
						`BLOCK_STATE_ENVD_ADD_WAIT: begin
							state <= `BLOCK_STATE_ENVD_STORE;
						end
						
						`BLOCK_STATE_ENVD_STORE: begin
							write_dest(sum);
							regs[2] <= sum;
							state <= `BLOCK_STATE_DONE;
							done <= 1;
						end
					endcase
				end
				
				`BLOCK_INSTR_DELAY: begin
					case (state)
						`BLOCK_STATE_BEGIN: begin
							delay_buf_handle 	<= regs[0];
							delay_buf_data_out 	<= regs[1];
							delay_read_req		<= 1;
							
							state <= `BLOCK_STATE_DELAY_READ_WAIT;
						end
						
						`BLOCK_STATE_DELAY_READ_WAIT: begin
							if (delay_buf_invalid_read) begin
								state <= `BLOCK_STATE_DONE;
								done <= 1;
							end
							if (delay_buf_read_ready) begin
								mul_req_a <= delay_buf_data_in;
								mul_req_b <= regs[2];
								wait_one <= 1;
								
								delay_read_req <= 0;
								state <= `BLOCK_STATE_DELAY_MUL_WAIT;
							end
						end
						
						`BLOCK_STATE_DELAY_MUL_WAIT: begin
							if (mul_ready) begin
								summand_a <= mul_result_final;
								summand_b <= src_b_val;
								
								state <= `BLOCK_STATE_DELAY_ADD_WAIT;
							end
							wait_one <= 0;
						end
						
						`BLOCK_STATE_DELAY_ADD_WAIT: begin
							write_dest(sum);
							delay_buf_handle 	<= regs[0];
							delay_buf_data_out 	<= sum;
							delay_write_req		<= 1;
							
							state <= `BLOCK_STATE_DELAY_WRITE_WAIT;
							wait_one <= 1;
						end
						
						`BLOCK_STATE_DELAY_WRITE_WAIT: begin
							if (wait_one) begin
								wait_one <= 0;
							end
							else if (delay_buf_write_ready) begin
								delay_write_req <= 0;
								done <= 1;
							end
						end
					endcase
				end
			endcase
		end
		
		if (reset) begin
			instr <= `BLOCK_INSTR_NOP;
			
			for (i = 0; i < n_channels; i++) begin
				ch_regs[i] <= 0;
			end
			
			for (i = 0; i < n_registers; i++) begin
				regs[i] <= 0;
				reg_updates[i] <= 0;
				reg_updated[i] <= 0;
			end
			
			lut_req <= 0;
			state <= `BLOCK_STATE_DONE;
			done <= 1;
		end
	end
endmodule
