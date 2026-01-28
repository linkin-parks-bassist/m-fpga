`include "block.vh"
`include "lut.vh"
`include "seq.vh"
`include "instr_dec.vh"

module dsp_core #(
		parameter integer data_width 		= 16,
		parameter integer n_blocks			= 256,
		parameter integer n_channels   		= 16,
		parameter integer n_registers  		= 16,
		parameter integer memory_size		= 128
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
		input wire delay_write_ack,
		
		input wire reset_state
	);
	
	reg wait_one = 0;
	
	reg signed [data_width - 1 : 0] mul_req_a;
	reg signed [data_width - 1 : 0] mul_req_b;
	
	//
	// Fixed hardware multiplier
	//
	wire signed [2 * data_width - 1 : 0] mul_result = mul_req_a * mul_req_b;

	//
	// Multiplication post-processing
	//
	reg signed  [2 * data_width - 1 : 0] mul_result_latched;
	wire signed [2 * data_width - 1 : 0] mul_result_latched_saturated =
        (mul_result_latched > sat_max) ?  sat_max[data_width-1:0] :
		(mul_result_latched < sat_min) ?  sat_min[data_width-1:0] :
							   mul_result_latched[data_width-1:0];
    wire signed [2 * data_width - 1 : 0] mul_result_latched_shifted =
            $signed(mul_result_latched) >>> (data_width - 1 - instr_shift);
	reg signed  [2 * data_width - 1 : 0] mul_result_latched_shifted_latched;
	wire signed [2 * data_width - 1 : 0] mul_result_latched_shifted_latched_saturated =
        (mul_result_latched_shifted_latched > sat_max) ?  sat_max[data_width-1:0] :
		(mul_result_latched_shifted_latched < sat_min) ?  sat_min[data_width-1:0] :
							   mul_result_latched_shifted_latched[data_width-1:0];
	
	wire mul_ready = ~wait_one;
	
	//
	// Block instruction
	//
	reg  [`BLOCK_INSTR_WIDTH 	- 1 : 0] instr;
	reg  [`BLOCK_INSTR_WIDTH 	- 1 : 0] next_instr;
	reg  [`BLOCK_INSTR_WIDTH 	- 1 : 0] instr_fetch;
	reg  [$clog2(n_blocks) 		- 1 : 0] instr_fetch_addr;
	
	reg instr_write = 0;
	reg [$clog2(n_blocks) 	- 1 : 0] instr_write_addr;
	reg [`BLOCK_INSTR_WIDTH - 1 : 0] instr_write_val;
	
	reg latch_instr;
	reg latch_instr_wait;
	
	reg latch_next_instr = 0;
	reg latch_next_instr_wait = 0;
	
	reg [`BLOCK_INSTR_WIDTH - 1 : 0] instrs[n_blocks - 1 : 0];
	
	//
	// Instruction decoding
	//
	
	wire [`BLOCK_INSTR_OP_WIDTH - 1 : 0] operation;
	
	wire [`BLOCK_REG_ADDR_WIDTH - 1 : 0] src_a;
	wire [`BLOCK_REG_ADDR_WIDTH - 1 : 0] src_b;
	wire [`BLOCK_REG_ADDR_WIDTH - 1 : 0] src_c;
	wire [`BLOCK_REG_ADDR_WIDTH - 1 : 0] dest;
	
	wire src_a_reg;
	wire src_b_reg;
	wire src_c_reg;

	wire saturate;

	wire [`SHIFT_WIDTH - 1 : 0] instr_shift;
	
	wire [`BLOCK_RES_ADDR_WIDTH - 1 : 0] res_addr;
	
	wire no_shift;
	
	instr_decoder #(.data_width(data_width)) dec(
		.clk(clk),
		
		.instr(instr),
		
		.operation(operation),
		
		.src_a(src_a),
		.src_b(src_b),
		.src_c(src_c),
		.dest(dest),
		
		.src_a_reg(src_a_reg),
		.src_b_reg(src_b_reg),
		.src_c_reg(src_c_reg),

		.saturate(saturate),

		.instr_shift(instr_shift),
		
		.res_addr(res_addr),
		
		.no_shift(no_shift)
	); 

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
	
	wire [data_width - 1 : 0] resource_addr = instr[`BLOCK_INSTR_WIDTH - 1 : `BLOCK_INSTR_WIDTH - 1 - data_width];
	
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
	// FSM state
	//
	reg [16:0] state = `BLOCK_STATE_READY;
	reg [16:0] ret_state;
	
	reg [$clog2(n_blocks) 	- 1 : 0] current_block = 0;
	
	//
	// Block registers BSRAM
	//
	reg signed [data_width - 1 : 0] regs[n_blocks * n_registers - 1 : 0];
	
	reg reg_write = 0;
	reg signed [data_width - 1 : 0] reg_write_val;
	
	reg [$clog2(n_blocks) + `BLOCK_REG_ADDR_WIDTH - 1 : 0] reg_write_addr;
	reg [$clog2(n_blocks) + `BLOCK_REG_ADDR_WIDTH - 1 : 0] reg_fetch_addr;
	
	reg signed [data_width - 1 : 0] reg_fetch;
	
	//
	// Channels BSRAM
	//
    reg signed [data_width - 1 : 0] ch_regs [n_channels - 1 : 0];
    
    reg ch_write = 0;
    reg [`BLOCK_REG_ADDR_WIDTH - 1 : 0] ch_write_addr;
    reg [`BLOCK_REG_ADDR_WIDTH - 1 : 0] ch_fetch_addr;
    
    reg signed [data_width - 1 : 0] ch_write_val;
    reg signed [data_width - 1 : 0] ch_fetch;
	
	//
	// Scratchpad memory BSRAM
	//
	reg signed [data_width - 1 : 0] memory [memory_size - 1 : 0];
	
    reg mem_write = 0;
	reg signed [data_width - 1 : 0] mem_write_val;
	
	reg [$clog2(memory_size) - 1 : 0] mem_write_addr;
    reg [$clog2(memory_size) - 1 : 0] mem_fetch_addr;
    
    reg signed [data_width - 1 : 0] mem_fetch;
    
	// Address of last block with instruction written
	reg [$clog2(n_blocks) - 1 : 0] last_block = 0;
	
	// To hold the final result of computation of a block
	reg signed [data_width - 1 : 0] work;
	reg no_write_work = 0;
	
	// Latching edges of register write pulses
	reg command_reg_write_prev = 0;
	reg command_reg_write_rose = 0;
	
	// Useful for debugging
	reg [15:0] cycle_ctr = 0;
	
	// Wide accumulator for MAC blocks
	reg signed [2 * data_width - 1 : 0] accumulator;
	wire [2 * data_width - 1 : 0] accumulator_sat = (accumulator > sat_max) ? sat_max : ((accumulator < sat_min) ? sat_min : accumulator);
	
	
	//
	// linear interpolation unit
	//
	localparam interp_bits = 8;
	
	sequential_interp #(.data_width(data_width), .interp_bits(interp_bits)) interp
	(
		.clk(clk),
		.reset(),
		
		.start(interp_start),
		.ready(interp_ready),
		
		.base(interp_arg_a),
		.target(interp_arg_b),
		.frac(interp_frac),
		.interpolated(interpolated)
	);
	
	reg interp_start;
	wire interp_ready;
	wire signed [data_width - 1 : 0] interpolated;

	reg signed [data_width - 1 : 0] interp_arg_a;
	reg signed [data_width - 1 : 0] interp_arg_b;
	reg [interp_bits - 1 : 0] interp_frac;
	
	// Sequential reset counters
	reg [$clog2(memory_size) : 0] mem_reset_ctr;
	reg [$clog2(n_blocks) 	 : 0] blk_reset_ctr;
	
	integer i;
	// Main sequential block
	always @(posedge clk) begin
		wait_one <= 0;
		
		instr_fetch	<= instrs	[instr_fetch_addr];
		reg_fetch 	<= regs		[reg_fetch_addr  ];
		ch_fetch 	<= ch_regs	[ch_fetch_addr   ];
		mem_fetch 	<= memory	[mem_fetch_addr  ];
		
		interp_start <= 0;
		
		if (latch_next_instr_wait) begin
			latch_next_instr_wait <= 0;
		end else if (latch_next_instr) begin
			next_instr <= instr_fetch;
			latch_next_instr <= 0;
		end
		
		if (command_instr_write) begin
			instr_write_val <= command_instr_write_val;
			instr_write_addr <= command_block_target;
			instr_write <= 1;
			
			last_block <= (command_block_target > last_block) ? command_block_target : last_block;
		end
		
		if (instr_write) begin
			instrs[instr_write_addr] <= instr_write_val;
			instr_write <= 0;
		end
		
		command_reg_write_prev <= command_reg_write;
		
		if (command_reg_write & ~command_reg_write_prev)
			command_reg_write_rose <= 1;
		
		if (mem_write) begin
			memory[mem_write_addr] <= mem_write_val;
		end
		
		if (reg_write) begin
			regs[reg_write_addr] <= reg_write_val;
		end
		
		if (ch_write) begin
			ch_regs[ch_write_addr] <= ch_write_val;
		end
		
		ch_write  <= 0;
		reg_write <= 0;
		mem_write <= 0;
		
		no_write_work <= 0;
		
		if (state != `CORE_STATE_READY)
			cycle_ctr <= cycle_ctr + 1;
		
		if (reset_state) begin
			state <= `CORE_STATE_RESETTING;
			
			mem_reset_ctr <= 0;
			blk_reset_ctr <= 0;
		end
		
		if (reset) begin
			state <= `CORE_STATE_READY;
			ready <= 1;
			
			for (i = 0; i < n_channels; i = i + 1)
				ch_regs[i] <= 0;
			
			last_block <= 0;
			
			instr <= 0;
			current_block <= 0;
			latch_next_instr <= 0;
		end else begin
			case (state)
				`CORE_STATE_RESETTING: begin
					if (mem_reset_ctr < memory_size) begin
						mem_write_addr <= mem_reset_ctr;
						mem_write_val <= 0;
						mem_write <= 1;
						
						mem_reset_ctr <= mem_reset_ctr + 1;
					end
					
					if (blk_reset_ctr < n_blocks) begin
						instr_write_addr <= current_block;
						instr_write_val <= 0;
						instr_write <= 1;
						
						current_block <= current_block + 1;
					end
					
					if (mem_reset_ctr >= memory_size && blk_reset_ctr >= n_blocks) begin
						current_block <= 0;
						state <= `CORE_STATE_READY;
					end
				end
				
				// Idle state 
				`CORE_STATE_READY: begin
					ready <= 1;
					
					// If there is a new sample coming in,
					// Load it and enter the start state
					if (tick) begin
						ch_regs[0] <= sample_in;
						
						current_block <= 0;
						ready <= 0;
						state <= `CORE_STATE_BLOCK_START;
						
						cycle_ctr <= 0;
					end
					
					if (command_reg_write_rose) begin
						reg_write_addr <= {command_block_target[$clog2(n_blocks) - 1 : 0], command_reg_target[`BLOCK_REG_ADDR_WIDTH - 1 : 0]};
						reg_write_val  <= command_reg_write_val;
						reg_write <= 1;
						command_reg_write_rose <= 0;
					end
				end
				
				`CORE_STATE_BLOCK_START: begin
                    instr_fetch_addr <= (current_block == last_block) ? 0 : current_block + 1;
                    latch_next_instr <= 1;
                    latch_next_instr_wait <= 1;
                    
					state <= `CORE_STATE_FETCH_SRC_A;
					ret_state <= `CORE_STATE_DISPATCH;
				end
				
				`CORE_STATE_FINISH_BLOCK: begin
					if (!no_write_work) begin
						ch_write_val  <= work;
						ch_write_addr <= dest;
						ch_write 	  <= 1;
					end
					
					instr <= next_instr;
					if (current_block >= last_block) begin
						current_block <= 0;
						
						if (!no_write_work && dest == 0) begin
							sample_out <= work;
							state <= `CORE_STATE_READY;
						end else begin
							ch_fetch_addr <= 0;
							wait_one <= 1;
							state <= `CORE_STATE_FINISH;
						end
					end else begin
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
					
				`CORE_STATE_DISPATCH: begin
					case (operation)
						`BLOCK_INSTR_NOP: begin
							no_write_work <= 1;
							state <= `CORE_STATE_FINISH_BLOCK;
						end
						
						`BLOCK_INSTR_ADD: begin
							state <= `CORE_STATE_FETCH_SRC_B;
							ret_state <= `CORE_STATE_ADD_1;
						end

						`BLOCK_INSTR_SUB: begin
							state <= `CORE_STATE_FETCH_SRC_B;
							ret_state <= `CORE_STATE_SUB_1;
						end

						// For shift (unary) instructions,
						// Repurpose the src_b instruction field
						// to specify shift size. Default to 1;
						// only accept alternates of 4 or 8, to avoid
						// big barrel shifter
						`BLOCK_INSTR_LSH: begin
							if (src_b == 4) begin
								work <= {src_a_latched[data_width - 5 : 0], 4'b0};
							end else if (src_b == 8) begin
								work <= {src_a_latched[data_width - 7 : 0], 8'b0};
							end else begin
								work <= {src_a_latched[data_width - 2 : 0], 1'b0};
							end
							state <= `CORE_STATE_FINISH_BLOCK;
						end

						`BLOCK_INSTR_RSH: begin
							if (src_b == 4) begin
								work <= {4'b0, src_a_latched[data_width - 1 : 4]};
							end else if (src_b == 8) begin
								work <= {8'b0, src_a_latched[data_width - 1 : 84]};
							end else begin
								work <= {1'b0, src_a_latched[data_width - 1 : 1]};
							end
							state <= `CORE_STATE_FINISH_BLOCK;
						end

						`BLOCK_INSTR_ARSH: begin
							if (src_b == 4) begin
								work <= src_a_latched >>> 4;
							end else if (src_b == 8) begin
								work <= src_a_latched >>> 8;
							end else begin
								work <= src_a_latched >>> 1;
							end
							state <= `CORE_STATE_FINISH_BLOCK;
						end

						`BLOCK_INSTR_MUL: begin
							state <= `CORE_STATE_FETCH_SRC_B;
							ret_state <= `CORE_STATE_MUL_1;
						end

						`BLOCK_INSTR_MADD: begin
							state <= `CORE_STATE_FETCH_SRC_B;
							ret_state <= `CORE_STATE_MADD_1;
						end

						`BLOCK_INSTR_ABS: begin
							work <= (src_a_latched < 0) ? -src_a_latched : src_a_latched;
							state <= `CORE_STATE_FINISH_BLOCK;
						end
						
						`BLOCK_INSTR_SAVE: begin
							state <= `CORE_STATE_SAVE_1;
						end
						
						`BLOCK_INSTR_LOAD: begin
							mem_fetch_addr <= res_addr;
							state <= `CORE_STATE_LOAD_1;
							wait_one <= 1;
						end
						
						`BLOCK_INSTR_MOV: begin
							work <= src_a_latched;
							state <= `CORE_STATE_FINISH_BLOCK;
						end
						
						`BLOCK_INSTR_CLAMP: begin
							state <= `CORE_STATE_FETCH_SRC_B;
							ret_state <= `CORE_STATE_CLAMP_1;
						end

						`BLOCK_INSTR_LUT: begin
							lut_arg 	<= ((src_a_latched << instr_shift) & ((1 << data_width) - 1)) | (src_a_latched[data_width - 1] << data_width);
							lut_handle 	<= res_addr[`LUT_HANDLE_WIDTH - 1 : 0];
							lut_req 	<= 1;
							wait_one	<= 1;
							state 		<= `CORE_STATE_LUT_1;
						end

						`BLOCK_INSTR_DELAY_READ: begin
							state <= `CORE_STATE_DELAY_READ_1;
						end

						`BLOCK_INSTR_DELAY_WRITE: begin
							state <= `CORE_STATE_DELAY_WRITE_1;
						end
						
						`BLOCK_INSTR_MACZ: begin
							accumulator <= 0;
							state <= `CORE_STATE_FETCH_SRC_B;
							ret_state <= `CORE_STATE_MAC_1;
						end
						
						`BLOCK_INSTR_MAC: begin
							state <= `CORE_STATE_FETCH_SRC_B;
							ret_state <= `CORE_STATE_MAC_1;
						end
						
						`BLOCK_INSTR_MOV_ACC: begin
							work <= (saturate) ? accumulator_sat[data_width - 1 : 0] : accumulator[data_width - 1 : 0];
							state <= `CORE_STATE_FINISH_BLOCK;
						end
						
						`BLOCK_INSTR_MOV_UACC: begin
							work <= accumulator[2 * data_width - 1 : data_width];
							state <= `CORE_STATE_FINISH_BLOCK;
						end
						
						`BLOCK_INSTR_LINTERP: begin
							state <= `CORE_STATE_FETCH_SRC_B;
							ret_state <= `CORE_STATE_LINTERP_1;
						end
						
						`BLOCK_INSTR_FRAC_DELAY: begin
							state <= `CORE_STATE_FRAC_DELAY_1;
						end
						
						`BLOCK_INSTR_LOAD_ACC: begin
							mem_fetch_addr <= res_addr;
							wait_one <= 1;
							state <= `CORE_STATE_LOAD_ACC_1;
						end
						
						`BLOCK_INSTR_SAVE_ACC: begin
							mem_write_addr <= res_addr;
							mem_write_val <= accumulator[2 * data_width - 1 : data_width];
							mem_write <= 1;
							state <= `CORE_STATE_SAVE_ACC_1;
						end
						
						`BLOCK_INSTR_ACC: begin
							accumulator <= (no_shift) ? accumulator + {{(data_width){1'b0}}, src_a_latched} : accumulator + {{(data_width){src_a_latched[data_width - 1]}}, src_a_latched};
							no_write_work <= 1;
							state <= `CORE_STATE_FINISH_BLOCK;
						end
						
						`BLOCK_INSTR_CLEAR_ACC: begin
							accumulator <= 0;
							no_write_work <= 1;
							state <= `CORE_STATE_FINISH_BLOCK;
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
                    mul_result_latched <= mul_result;
                    state <= `CORE_STATE_MUL_3;
                end

				`CORE_STATE_MUL_3: begin
					if (no_shift) begin
						work <= saturate ? mul_result_latched_saturated : mul_result_latched;
						state <= `CORE_STATE_FINISH_BLOCK;
					end else begin
						mul_result_latched_shifted_latched <= mul_result_latched_shifted;
						state <= `CORE_STATE_MUL_4;
					end
                end

                `CORE_STATE_MUL_4: begin
					work <= saturate ? mul_result_latched_shifted_latched_saturated : mul_result_latched_shifted_latched;
					state <= `CORE_STATE_FINISH_BLOCK;
				end

				`CORE_STATE_MADD_1: begin
					state <= `CORE_STATE_FETCH_SRC_C;
					ret_state <= `CORE_STATE_MADD_2;
				end
				
				`CORE_STATE_MADD_2: begin
					mul_req_a <= src_a_latched;
					mul_req_b <= src_b_latched;
					
					state <= `CORE_STATE_MADD_3;
				end
				
				`CORE_STATE_MADD_3: begin
                    mul_result_latched <= mul_result;
                    state <= `CORE_STATE_MADD_4;
                end

                `CORE_STATE_MADD_4: begin
                    mul_result_latched_shifted_latched <= mul_result_latched_shifted;
                    state <= `CORE_STATE_MADD_5;
                end

                `CORE_STATE_MADD_5: begin
					summand_a <= saturate ? mul_result_latched_shifted_latched_saturated : mul_result_latched_shifted_latched;
					summand_b <= src_c_latched;
					
					state <= `CORE_STATE_MADD_6;
				end
				
				`CORE_STATE_MADD_6: begin
					work <= sum_final;
					state <= `CORE_STATE_FINISH_BLOCK;
				end

				`CORE_STATE_LUT_1: begin
					if (!wait_one && lut_ready) begin
						work <= lut_data;
						lut_req <= 0;
						state <= `CORE_STATE_FINISH_BLOCK;
					end
				end

				`CORE_STATE_DELAY_READ_1: begin
					delay_req_handle 	<= res_addr;
					delay_req_arg 		<= src_a_latched;
					delay_read_req		<= 1;
					wait_one			<= 1;
					
					state <= `CORE_STATE_DELAY_READ_2;
				end

				`CORE_STATE_DELAY_READ_2: begin
					if (!wait_one && delay_read_ready) begin
						work <= delay_req_data_in;
						delay_read_req <= 0;
						
						state <= `CORE_STATE_FINISH_BLOCK;
					end
				end

                `CORE_STATE_DELAY_WRITE_1: begin
					delay_req_handle 	<= res_addr;
					delay_req_arg 		<= src_a_latched;
					delay_write_req		<= 1;
					wait_one 			<= 1;
					
					state <= `CORE_STATE_DELAY_WRITE_2;
				end
				
				`CORE_STATE_DELAY_WRITE_2: begin
					//if (!wait_one && delay_write_ack) begin
						delay_write_req <= 0;
						
						no_write_work <= 1;
						state <= `CORE_STATE_FINISH_BLOCK;
					//end
				end
				
				`CORE_STATE_SAVE_1: begin
					mem_write_addr 	<= res_addr[$clog2(memory_size) - 1 : 0];
					mem_write_val 	<= src_a_latched;
					mem_write 		<= 1;
					
					no_write_work <= 1;
					state <= `CORE_STATE_FINISH_BLOCK;
				end
				
				`CORE_STATE_LOAD_1: begin
					if (!wait_one) begin
						work <= mem_fetch;
						current_block <= current_block + 1;
						state <= `CORE_STATE_FINISH_BLOCK;
					end
				end
				
				`CORE_STATE_CLAMP_1: begin
					state <= `CORE_STATE_FETCH_SRC_C;
					ret_state <= `CORE_STATE_CLAMP_2;
				end
				
				`CORE_STATE_CLAMP_2: begin
					work <= (src_a_latched < src_b_latched) ? src_b_latched : ((src_a_latched > src_c_latched) ? src_c_latched : src_a_latched);
					state <= `CORE_STATE_FINISH_BLOCK;
				end
				
				`CORE_STATE_MAC_1: begin
					mul_req_a <= src_a_latched;
					mul_req_b <= src_b_latched;
					
					state <= `CORE_STATE_MAC_2;
				end
				
				`CORE_STATE_MAC_2: begin
                    mul_result_latched <= mul_result;
                    if (no_shift)
						state <= `CORE_STATE_MAC_4;
					else
						state <= `CORE_STATE_MAC_3;
                end
				
				`CORE_STATE_MAC_3: begin
					mul_result_latched <= mul_result_latched_shifted;
                    state <= `CORE_STATE_MAC_4;
                end

				`CORE_STATE_MAC_4: begin
					accumulator <= accumulator + mul_result_latched;
					
					no_write_work <= 1;
					state <= `CORE_STATE_FINISH_BLOCK;
				end
				
				`CORE_STATE_LINTERP_1: begin
					state <= `CORE_STATE_FETCH_SRC_C;
					ret_state <= `CORE_STATE_LINTERP_2;
				end
				
				`CORE_STATE_LINTERP_2: begin
					interp_arg_a <= src_b_latched;
					interp_arg_b <= src_c_latched;
					interp_frac  <= src_a_latched[data_width - 2 : data_width - 2 - interp_bits];
					interp_start <= 1;
					wait_one <= 1;
					state <= `CORE_STATE_LINTERP_3;
				end
				
				`CORE_STATE_LINTERP_3: begin
					if (!wait_one && interp_ready) begin
						work <= interpolated;
						state <= `CORE_STATE_FINISH_BLOCK;
					end
				end
				
				`CORE_STATE_FRAC_DELAY_1: begin
					if (accumulator[2*data_width-1]) begin // refuse negative delays
						work <= sample_in;
						state <= `CORE_STATE_FINISH_BLOCK;
					end else begin
						delay_req_arg <= accumulator[2*data_width-1:data_width];
						delay_req_handle <= res_addr;
						delay_read_req <= 1;
						wait_one <= 1;
						state <= `CORE_STATE_FRAC_DELAY_2;
					end
				end
				
				`CORE_STATE_FRAC_DELAY_2: begin
					if (!wait_one && delay_read_ready) begin
						interp_arg_a  <= delay_req_data_in;
						delay_req_arg <= accumulator[2*data_width-1:data_width]+1;
						delay_read_req <= 1;
						wait_one <= 1;
						state <= `CORE_STATE_FRAC_DELAY_3;
					end
				end
				
				`CORE_STATE_FRAC_DELAY_3: begin
					if (!wait_one && delay_read_ready) begin
						delay_read_req <= 0;
						interp_arg_b <= delay_req_data_in;
						interp_frac <= accumulator[data_width - 1 : data_width - interp_bits];
						interp_start <= 1;
						wait_one <= 1;
						state <= `CORE_STATE_FRAC_DELAY_4;
					end
				end
				
				`CORE_STATE_FRAC_DELAY_4: begin
					if (!wait_one && interp_ready) begin
						work <= interpolated;
						state <= `CORE_STATE_FINISH_BLOCK;
					end
				end
				
				`CORE_STATE_LOAD_ACC_1: begin
					if (wait_one) begin
						mem_fetch_addr <= res_addr + 1;
					end else begin
						accumulator <= {accumulator[data_width - 1 : 0], mem_fetch};
						state <= `CORE_STATE_LOAD_ACC_2;
					end
				end
				
				`CORE_STATE_LOAD_ACC_2: begin
					accumulator <= {accumulator[data_width - 1 : 0], mem_fetch};
					no_write_work <= 1;
					state <= `CORE_STATE_FINISH_BLOCK;
				end
				
				`CORE_STATE_SAVE_ACC_1: begin
					mem_write_addr <= res_addr + 1;
					mem_write_val <= accumulator[data_width - 1 : 0];
					mem_write <= 1;
					no_write_work <= 1;
					state <= `CORE_STATE_FINISH_BLOCK;
				end
			endcase
		end
	end
endmodule
