`include "block.vh"
`include "lut.vh"
`include "seq.vh"

module dsp_core #(
		parameter integer data_width 		= 16,
		parameter integer n_blocks			= 256,
		parameter integer n_channels   		= 16,
		parameter integer n_registers  		= 16
	) (
		input wire clk,
		input wire reset,
		
		input wire tick,
	
		input wire signed [data_width - 1 : 0] sample_in,
		output reg signed [data_width - 1 : 0] sample_out,
		
		output reg ready,
		
		input wire command_reg_write,
		input wire command_instr_write,
		
		input wire [$clog2(n_blocks)	  					 - 1 : 0] command_block_target,
		input wire [$clog2(n_blocks) + `BLOCK_REG_ADDR_WIDTH - 1 : 0] command_reg_target,
		input wire [`BLOCK_INSTR_WIDTH 	  					 - 1 : 0] command_instr_write_val,
		input wire signed [data_width 	  					 - 1 : 0] command_reg_write_val,
		
		output reg reg_write_ack,
		
		output reg lut_req,
		output reg signed [`LUT_HANDLE_WIDTH - 1 : 0] lut_handle,
		output reg signed [data_width - 1 : 0] lut_arg,
		input wire signed [data_width - 1 : 0] lut_data,
		input wire lut_ready,
		
		output reg delay_read_req,
		output reg delay_write_req,
		output reg signed [data_width - 1 : 0] delay_req_handle,
		output reg signed [data_width - 1 : 0] delay_req_arg,
		input wire signed [data_width - 1 : 0] delay_req_data_in,
		input wire delay_read_ready,
		input wire delay_write_ready
	);
	
	reg wait_one = 0;
	
	reg signed [data_width - 1 : 0] mul_req_a;
	reg signed [data_width - 1 : 0] mul_req_b;
	
	wire signed [2 * data_width - 1 : 0] mul_result = mul_req_a * mul_req_b;
	
	wire mul_ready = ~wait_one;
	
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

    wire saturate = ~instr[operand_type_start_index + 4];
	localparam pms_start_index = operand_type_start_index + 5;

    wire [SHIFT_WIDTH - 1 : 0] instr_shift = {{(SHIFT_WIDTH - `BLOCK_PMS_WIDTH){1'b0}}, instr[pms_start_index + `BLOCK_PMS_WIDTH - 1 : pms_start_index]};
	
	reg signed [data_width - 1 : 0] src_a_latched;
	reg signed [data_width - 1 : 0] src_b_latched;
	reg signed [data_width - 1 : 0] src_c_latched;
	
	// Explicitly sized saturation limits in FULL multiply width
	localparam signed sat_max = {{(data_width + 1){1'b0}}, {(data_width - 1){1'b1}}};
	localparam signed sat_min = {{(data_width + 1){1'b1}}, {(data_width - 1){1'b0}}};
	
	localparam signed sat_max_q3_29 = sat_max << (data_width - 1);
	localparam signed sat_min_q3_29 = sat_min << (data_width - 1);
	
	localparam signed sat_max_dwe = sat_max[data_width : 0];
	localparam signed sat_min_dwe = sat_min[data_width : 0];
	
	localparam signed sat_max_dw = sat_max[data_width - 1 : 0];
	localparam signed sat_min_dw = sat_min[data_width - 1 : 0];
	
	wire signed [data_width * 2 - 1 : 0] sat_max_shifted = sat_max >>> instr_shift;
	wire signed [data_width * 2 - 1 : 0] sat_min_shifted = sat_min >>> instr_shift;
	
	wire signed [data_width - 1 : 0] sat_max_shifted_dw = sat_max_shifted[data_width - 1 : 0];
	wire signed [data_width - 1 : 0] sat_min_shifted_dw = sat_min_shifted[data_width - 1 : 0];
	
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
	
	wire [data_width - 1 : 0] sum_sat = (sum_ext > sat_max_dwe) ? sat_max_dw : ((sum_ext < sat_min_dwe) ? sat_min_dw : sum_ext[data_width-1:0]);
	wire [data_width - 1 : 0] sum_nsat = sum_ext[data_width-1:0];
	
    wire [data_width - 1 : 0] sum_final = saturate ? sum_sat : sum_nsat;

	//
	// Saturation and format compensation for muls
	//
	localparam integer MAX_SHIFT = 2 * data_width - 2;
	localparam integer SHIFT_WIDTH = $clog2(MAX_SHIFT + 1);

	// Arithmetic shift
	wire signed [2 * data_width - 1 : 0] mul_result_shifted =
		$signed(mul_result) >>> (data_width - 1 - instr_shift);

	// Final saturation and narrowing
	wire signed [data_width-1:0] mul_result_sat =
		(mul_result_shifted > sat_max) ?  sat_max[data_width-1:0] :
		(mul_result_shifted < sat_min) ?  sat_min[data_width-1:0] :
							   mul_result_shifted[data_width-1:0];
	
    wire signed [data_width-1:0] mul_result_nsat = mul_result_shifted[data_width-1:0];
    wire signed [data_width-1:0] mul_result_final = saturate ? mul_result_sat : mul_result_nsat;
	
	reg [12:0] state = `BLOCK_STATE_READY;
	reg [12:0] ret_state;
	
	reg [$clog2(n_blocks) 	- 1 : 0] current_block = 0;
	reg [`BLOCK_INSTR_WIDTH - 1 : 0] instrs[n_blocks - 1 : 0];
	
	reg reg_write = 0;
	reg [$clog2(n_blocks) + `BLOCK_REG_ADDR_WIDTH - 1 : 0] reg_write_addr;
	reg [$clog2(n_blocks) + `BLOCK_REG_ADDR_WIDTH - 1 : 0] reg_fetch_addr;
	
	reg signed [data_width - 1 : 0] reg_write_val;
	reg signed [data_width - 1 : 0] reg_fetch;
	
    reg ch_write = 0;
    reg [`BLOCK_REG_ADDR_WIDTH - 1 : 0] ch_write_addr;
    reg [`BLOCK_REG_ADDR_WIDTH - 1 : 0] ch_fetch_addr;
    
    reg signed [data_width - 1 : 0] ch_write_val;
    reg signed [data_width - 1 : 0] ch_fetch;
	
    reg signed [data_width - 1 : 0] ch_regs [n_channels - 1 : 0];
	reg signed [data_width - 1 : 0] regs 	[n_blocks * n_registers - 1 : 0];
	
	reg [$clog2(n_blocks) - 1 : 0] last_block = 0;
	
	reg signed [data_width - 1 : 0] work;
	
	reg command_reg_write_prev = 0;
	reg command_reg_write_rose = 0;
	
	reg [15:0] cycle_ctr = 0;
	
	integer i;
	always @(posedge clk) begin
		wait_one <= 0;
		reg_write_ack <= 0;
		
		instr 		<= instrs	[current_block];
		reg_fetch 	<= regs		[reg_fetch_addr];
		ch_fetch 	<= ch_regs	[ch_fetch_addr];
		
		if (command_instr_write) begin
			instrs[command_block_target] <= command_instr_write_val;
			last_block <= (command_block_target > last_block) ? command_block_target : last_block;
		end
		
		command_reg_write_prev <= command_reg_write;
		
		if (command_reg_write & ~ command_reg_write_prev)
			command_reg_write_rose <= 1;
		
		if (reg_write) begin
			regs[reg_write_addr] <= reg_write_val;
		end
		
		if (ch_write) begin
			ch_regs[ch_write_addr] <= ch_write_val;
		end
		
		ch_write  <= 0;
		reg_write <= 0;
		
		if (state != `CORE_STATE_READY)
			cycle_ctr <= cycle_ctr + 1;
		
		if (reset) begin
			state <= `CORE_STATE_READY;
			ready <= 1;
			
			for (i = 0; i < n_channels; i = i + 1)
				ch_regs[i] <= 0;
		end else begin
			case (state)
				`CORE_STATE_READY: begin
					ready <= 1;
					
					if (tick) begin
						ch_regs[0] <= sample_in;
						
						current_block <= 0;
						ready <= 0;
						state <= `CORE_STATE_BLOCK_START;
						
						cycle_ctr <= 0;
					end
					
					if (command_reg_write_rose) begin
						reg_write_addr <= {command_block_target[$clog2(n_blocks) - 1 : 0], command_reg_target[`BLOCK_REG_ADDR_WIDTH - 1 : 0]};
						reg_write_val <= command_reg_write_val;
						reg_write <= 1;
						reg_write_ack <= 1;
						command_reg_write_rose <= 0;
					end
				end
				
				`CORE_STATE_FINISH_BLOCK: begin
					if (dest_reg) begin
						reg_write_val <= work;
						reg_write_addr <= {current_block, dest};
						reg_write <= 1;
					end
					else begin
						ch_write_val <= work;
						ch_write_addr <= dest;
						ch_write <= 1;
					end
					
					state <= `CORE_STATE_CONTINUE;
				end
				
				`CORE_STATE_CONTINUE: begin
					if (current_block == last_block || current_block == n_blocks - 1) begin
						current_block 	<= 0;
						ch_fetch_addr 	<= 0;
						
						wait_one <= 1;
						state <= `CORE_STATE_FINISH;
					end
					else begin
						current_block <= current_block + 1;
						state <= `CORE_STATE_BLOCK_START;
					end
				end
				
				`CORE_STATE_FINISH: begin
					if (!wait_one) begin
						sample_out <= ch_fetch;
						state <= `CORE_STATE_READY;
					end
				end
				
				`CORE_STATE_FETCH_SRC_A: begin
					if (src_a_reg) begin
						reg_fetch_addr <= {current_block, src_a};
					end
					else begin
						ch_fetch_addr <= src_a;
					end
					
					wait_one <= 1;
					state <= `CORE_STATE_FETCH_SRC_A_2;
				end
				
				`CORE_STATE_FETCH_SRC_A_2: begin
					if (!wait_one) begin
						if (src_a_reg) begin
							src_a_latched <= reg_fetch;
						end
						else begin
							src_a_latched <= ch_fetch;
						end
						state <= ret_state;
					end
				end
				
				`CORE_STATE_FETCH_SRC_B: begin
					if (src_b_reg) begin
						reg_fetch_addr <= {current_block, src_b};
					end
					else begin
						ch_fetch_addr <= src_b;
					end
					
					wait_one <= 1;
					state <= `CORE_STATE_FETCH_SRC_B_2;
				end
				
				`CORE_STATE_FETCH_SRC_B_2: begin
					if (!wait_one) begin
						if (src_b_reg) begin
							src_b_latched <= reg_fetch;
						end
						else begin
							src_b_latched <= ch_fetch;
						end
						state <= ret_state;
					end
				end
				
				`CORE_STATE_FETCH_SRC_C: begin
					if (src_c_reg) begin
						reg_fetch_addr <= {current_block, src_c};
					end
					else begin
						ch_fetch_addr <= src_c;
					end
					
					wait_one <= 1;
					state <= `CORE_STATE_FETCH_SRC_C_2;
				end
				
				`CORE_STATE_FETCH_SRC_C_2: begin
					if (!wait_one) begin
						if (src_c_reg) begin
							src_c_latched <= reg_fetch;
						end
						else begin
							src_c_latched <= ch_fetch;
						end
						state <= ret_state;
					end
				end
				
				`CORE_STATE_BLOCK_START: begin
					state <= `CORE_STATE_FETCH_SRC_A;
					ret_state <= `CORE_STATE_DISPATCH;
				end
					
				`CORE_STATE_DISPATCH: begin
					case (operation)
						`BLOCK_INSTR_NOP: begin
							state <= `CORE_STATE_CONTINUE;
						end
						
						`BLOCK_INSTR_ADD: begin
							state <= `CORE_STATE_FETCH_SRC_B;
							ret_state <= `CORE_STATE_ADD_1;
						end

						`BLOCK_INSTR_SUB: begin
							state <= `CORE_STATE_FETCH_SRC_B;
							ret_state <= `CORE_STATE_SUB_1;
						end

						`BLOCK_INSTR_LSH: begin
							work <= {src_a_latched[data_width - 2 : 0], 1'b0};
							state <= `CORE_STATE_FINISH_BLOCK;
						end

						`BLOCK_INSTR_RSH: begin
							work <= {1'b0, src_a_latched[data_width - 1 : 1]};
							state <= `CORE_STATE_FINISH_BLOCK;
						end

						`BLOCK_INSTR_ARSH: begin
							work <= src_a_latched >>> 1;
							state <= `CORE_STATE_FINISH_BLOCK;
						end

						`BLOCK_INSTR_MUL: begin
							state <= `CORE_STATE_FETCH_SRC_B;
							ret_state <= `CORE_STATE_MUL_1;
						end

						`BLOCK_INSTR_MAC: begin
							state <= `CORE_STATE_FETCH_SRC_B;
							ret_state <= `CORE_STATE_MAC_1;
						end

						`BLOCK_INSTR_ABS: begin
							work <= (src_a_latched < 0) ? -src_a_latched : src_a_latched;
							state <= `CORE_STATE_FINISH_BLOCK;
						end
						
						`BLOCK_INSTR_GW: begin
							state <= `CORE_STATE_FETCH_SRC_B;
							ret_state <= `CORE_STATE_GW_1;
						end
						
						`BLOCK_INSTR_MOV: begin
							work <= src_a_latched;
							state <= `CORE_STATE_FINISH_BLOCK;
						end
						
						`BLOCK_INSTR_CLAMP: begin
							work <= (src_a_latched < sat_min_shifted_dw) ? sat_min_shifted_dw
																 : ((src_a_latched > sat_max_shifted_dw) ? sat_max_shifted_dw
																										 : src_a_latched);
							state <= `CORE_STATE_FINISH_BLOCK;
						end

						`BLOCK_INSTR_LUT: begin
							//bleh
						end

						`BLOCK_INSTR_DELAY: begin
							//bleh
						end
					endcase
				end

				`CORE_STATE_ADD_1: begin
					summand_a <= src_a_latched;
					summand_b <= src_b_latched;
					
					state <= `CORE_STATE_ADD_2;
				end
				
				`CORE_STATE_ADD_2: begin
					work <= sum_final;
					state <= `CORE_STATE_FINISH_BLOCK;
				end

				`CORE_STATE_SUB_1: begin
					summand_a <=  src_a_latched;
					summand_b <= -src_b_latched;
					
					state <= `CORE_STATE_SUB_2;
				end
				
				`CORE_STATE_SUB_2: begin
					work <= sum_final;
					state <= `CORE_STATE_FINISH_BLOCK;
				end

				`CORE_STATE_MUL_1: begin
					mul_req_a <= src_a_latched;
					mul_req_b <= src_b_latched;
					
					state <= `CORE_STATE_MUL_2;
				end
				
				`CORE_STATE_MUL_2: begin
					work <= mul_result_final;
					state <= `CORE_STATE_FINISH_BLOCK;
				end

				`CORE_STATE_MAC_1: begin
					state <= `CORE_STATE_FETCH_SRC_C;
					ret_state <= `CORE_STATE_MAC_2;
				end
				
				`CORE_STATE_MAC_2: begin
					mul_req_a <= src_a_latched;
					mul_req_b <= src_b_latched;
					
					state <= `CORE_STATE_MAC_3;
				end
				
				`CORE_STATE_MAC_3: begin
					summand_a <= mul_result_final;
					summand_b <= src_c_latched;
					
					state <= `CORE_STATE_MAC_4;
				end
				
				`CORE_STATE_MAC_4: begin
					work <= sum_final;
					state <= `CORE_STATE_FINISH_BLOCK;
				end

				`CORE_STATE_LUT_1: begin
					
				end

				`CORE_STATE_DELAY_1: begin
					
				end
				
				`CORE_STATE_GW_1: begin
					reg_write_addr 	<= src_a_latched[$clog2(n_blocks) + `BLOCK_REG_ADDR_WIDTH - 1 : 0];
					reg_write_val 	<= src_b_latched;
					reg_write 		<= 1;
					state <= `CORE_STATE_CONTINUE;
				end

			endcase
		end
	end
	
endmodule
