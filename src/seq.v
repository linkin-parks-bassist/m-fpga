`include "block.vh"
`include "lut.vh"
`include "seq.vh"
`include "instr_dec.vh"

module dsp_core #(
		parameter integer data_width 		= 16,
		parameter integer n_blocks			= 256,
		parameter integer n_channels   		= 16,
		parameter integer n_registers  		= 16,
		parameter integer memory_size		= n_blocks
	) (
		input wire clk,
		input wire reset,
		
		input wire enable,
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
		
		input wire full_reset,
		output wire resetting
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
	
	assign resetting = (state == `CORE_STATE_RESETTING);
	
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
		
		if (full_reset) begin
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
						mem_write_addr 	<= mem_reset_ctr;
						mem_write_val 	<= 0;
						mem_write 		<= 1;
						
						mem_reset_ctr <= mem_reset_ctr + 1;
					end
					
					if (blk_reset_ctr < n_blocks) begin
						instr_write_addr 	<= blk_reset_ctr;
						instr_write_val 	<= 0;
						instr_write			<= 1;
						
						blk_reset_ctr <= blk_reset_ctr + 1;
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
					if (tick && enable) begin
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
								work <= {8'b0, src_a_latched[data_width - 1 : 8]};
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

`define ALU_OP_ADD 			1
`define ALU_OP_SUB 			2
`define ALU_OP_LSH 			3
`define ALU_OP_RSH 			4
`define ALU_OP_ARSH 		5
`define ALU_OP_ARSH_WIDE	9
`define ALU_OP_MUL 			6
`define ALU_OP_MADD			7
`define ALU_OP_ABS			8
`define ALU_OP_MIN			9
`define ALU_OP_MAX			10
`define ALU_OP_CLAMP		16
`define ALU_OP_MAC			18
`define ALU_OP_SAT			19
`define ALU_OP_LINTERP		20

`define ALU_STATE_SHIFT 	2
`define ALU_STATE_SATURATE 	3
`define ALU_STATE_ACC 		4
`define ALU_STATE_INTERP	5
`define ALU_STATE_DONE 		1
`define ALU_STATE_IDLE 		0

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

module dsp_core_2 #(
		parameter integer data_width 	= 16,
		parameter integer n_blocks		= 256,
		parameter integer n_channels   	= 16,
		parameter integer n_block_regs  = 2,
		parameter integer memory_size	= n_blocks
	) (
		input wire clk,
		input wire reset,
		
		input wire enable,
		input wire tick,
	
		input wire signed [data_width - 1 : 0] sample_in,
		output reg signed [data_width - 1 : 0] sample_out,
		
		output reg ready,
		
		input wire command_reg_write,
		input wire command_instr_write,
		
		input wire [$clog2(n_blocks)  	 - 1  : 0] command_block_target,
		input wire [$clog2(n_block_regs) - 1  : 0] command_reg_target,
		input wire [31					  	  : 0] command_instr_write_val,
		input wire signed [data_width 	  - 1 : 0] command_reg_write_val,
		
		output reg lut_req,
		output reg signed [data_width - 1 : 0] lut_handle,
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
		
		input wire full_reset,
		output reg resetting
	);
	
	localparam signed sat_max = {{(data_width + 1){1'b0}}, {(data_width - 1){1'b1}}};
	localparam signed sat_min = {{(data_width + 1){1'b1}}, {(data_width - 1){1'b0}}};
	
	localparam block_addr_w 	= $clog2(n_blocks);
	localparam mem_addr_w 		= $clog2(memory_size);
	localparam ch_addr_w 		= $clog2(n_channels);
	localparam reg_addr_w		= $clog2(n_blocks) + $clog2(n_block_regs);
	
	reg [31 			  : 0] instrs [n_blocks - 1 : 0];
	
	reg [block_addr_w - 1 : 0] instr_read_addr;
	reg [block_addr_w - 1 : 0] instr_write_addr;
	reg [31 			  : 0] instr_read_val;
	reg [31 			  : 0] instr_write_val;
	
	reg instr_write_enable;
	
	reg [data_width   - 1 : 0] block_regs [n_block_regs * n_blocks - 1 : 0];
	
	reg [reg_addr_w - 1 : 0] reg_read_addr;
	reg [reg_addr_w - 1 : 0] reg_write_addr;
	reg [data_width - 1 : 0] reg_read_val;
	reg [data_width - 1 : 0] reg_write_val;
	reg reg_write_enable;
	
	reg signed [data_width - 1 : 0] channels [n_channels - 1 : 0];
	
	reg [ch_addr_w  - 1 : 0] channel_read_addr;
	reg [ch_addr_w  - 1 : 0] channel_write_addr;
	reg [data_width - 1 : 0] channel_read_val;
	reg [data_width - 1 : 0] channel_write_val;
	
	reg channel_write_enable;
	
	reg signed [data_width - 1 : 0] mem [memory_size - 1 : 0];
	
	reg [mem_addr_w - 1 : 0] mem_read_addr;
	reg [mem_addr_w - 1 : 0] mem_write_addr;
	reg [data_width - 1 : 0] mem_read_val;
	reg [data_width - 1 : 0] mem_write_val;
	
	reg mem_write_enable;
	
	always @(posedge clk) begin
		instr_read_val 	 <=     instrs[instr_read_addr  ];
		channel_read_val <=   channels[channel_read_addr];
		reg_read_val 	 <= block_regs[reg_read_addr    ];
		mem_read_val 	 <= 	   mem[mem_read_addr    ];
		
		if   (instr_write_enable)     instrs[instr_write_addr  ] <=   instr_write_val;
		if (channel_write_enable)   channels[channel_write_addr] <= channel_write_val;
		if     (reg_write_enable) block_regs[reg_write_addr    ] <=     reg_write_val;
		if     (mem_write_enable) 		 mem[mem_write_addr    ] <=     mem_write_val;
	end
	
	reg [31 : 0] instr;
	
	reg latch_instr;
	reg latch_instr_next;
	reg latch_instr_now;
	
	reg [31 : 0] next_instr;
	
	reg latch_next_instr;
	reg latch_next_instr_next;
	reg latch_next_instr_now;
		
	wire [4 : 0] operation;
	
	wire [3 : 0] src_a;
	wire [3 : 0] src_b;
	wire [3 : 0] src_c;
	wire [3 : 0] dest;
	
	wire src_a_reg;
	wire src_b_reg;
	wire src_c_reg;

	wire src_a_needed;
	wire src_b_needed;
	wire src_c_needed;

	wire saturate;

	wire [4 : 0] instr_shift;
	wire no_shift;
	
	wire [7 : 0] res_addr;
	
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
		
		.src_a_needed(src_a_needed),
		.src_b_needed(src_b_needed),
		.src_c_needed(src_c_needed),

		.saturate(saturate),

		.instr_shift(instr_shift),
		
		.res_addr(res_addr),
		
		.no_shift(no_shift)
	);
	
	reg alu_trigger;
	
	reg  [7:0] alu_op;
	reg  signed [data_width - 1 : 0] alu_a;
	reg  signed [data_width - 1 : 0] alu_b;
	reg  signed [data_width - 1 : 0] alu_c;
	
	reg  signed [2 * data_width - 1 : 0] alu_a_wide;
	reg  signed [2 * data_width - 1 : 0] alu_b_wide;
	
	reg [$clog2(data_width) - 1 : 0] alu_shift;

	reg alu_no_shift;
	reg alu_saturate;
	
	wire signed [    data_width - 1 : 0] alu_result;
	wire signed [2 * data_width - 1 : 0] alu_result_wide;
	
	reg signed [data_width - 1 : 0] alu_result_latched;
	reg write_alu_result;
	
	reg signed [data_width - 1 : 0] lut_result_latched;
	reg write_lut_result;
	
	reg signed [data_width - 1 : 0] delay_result_latched;
	reg write_delay_result;
	
	reg signed [data_width - 1 : 0] mem_result_latched;
	reg write_mem_result;
	
	wire alu_result_valid;
	wire alu_ready;

	// (input|output)[ \t]*(wire|reg)[ \t]*(signed|)[ \t]*(\[[a-z_0-9: \-\+\*]*\]|)[ \t]*([a-z_]*)
	// .\5(\5)
	dsp_core_alu #(.data_width(data_width)) alu
	(
		.clk(clk),
		.reset(reset),
		
		.trigger(alu_trigger),
		
		.op(alu_op),
		.a(alu_a),
		.b(alu_b),
		.c(alu_c),
		
		.a_wide(alu_a_wide),
		.b_wide(alu_b_wide),
		
		.shift(alu_shift),
		.no_shift(no_shift),
		.saturate(saturate),
		
		.result(alu_result),
		.result_wide(alu_result_wide),
		
		.result_valid(alu_result_valid),
		.ready(alu_ready)
	);
	
	reg  [$clog2(n_blocks) : 0] current_block;
	reg  [$clog2(n_blocks) : 0]    last_block;
	wire [$clog2(n_blocks) : 0]    next_block = (current_block == last_block) ? 0 : current_block + 1;

	reg latch_src_a_next;
	reg latch_src_a;
	reg src_a_valid;
	
	reg signed [data_width - 1 : 0] src_a_latched;

	reg latch_src_b_next;
	reg latch_src_b;
	reg src_b_valid;
	
	reg signed [data_width - 1 : 0] src_b_latched;

	reg latch_src_c_next;
	reg latch_src_c;
	reg src_c_valid;
	
	reg signed [data_width - 1 : 0] src_c_latched;
	
	always @(posedge clk) begin
		if (reset | full_reset | exec_done | (tick & enable)) begin
			latch_src_a_next <= 0;
			latch_src_a		 <= 0;
			src_a_valid	 	 <= 0;
			latch_src_b_next <= 0;
			latch_src_b		 <= 0;
			src_b_valid	 	 <= 0;
			latch_src_c_next <= 0;
			latch_src_c		 <= 0;
			src_c_valid	 	 <= 0;
		end else if (latch_src_a_next) begin
			latch_src_a_next <= 0;
			latch_src_a 	 <= 1;
		end else if (latch_src_a) begin
			src_a_latched 	<= (src_a_reg) ? reg_read_val : channel_read_val;
			latch_src_a 	<= 0;
			src_a_valid 	<= 1;
		end else if (src_a_needed & ~src_a_valid) begin
			if (src_a_reg)
				reg_read_addr <= {current_block, src_a[0]};
			else
				channel_read_addr <= src_a;
			latch_src_a_next <= 1;
		end
		
		if (latch_src_b_next) begin
			latch_src_b_next <= 0;
			latch_src_b 	 <= 1;
		end else if (latch_src_b) begin
			src_b_latched 	<= (src_b_reg) ? reg_read_val : channel_read_val;
			latch_src_b 	<= 0;
			src_b_valid 	<= 1;
		end else if (src_b_needed & ~src_b_valid && (!(src_a_needed & ~src_a_valid) | latch_src_a_next)) begin
			if (src_b_reg)
				reg_read_addr <= {current_block, src_b[0]};
			else
				channel_read_addr <= src_b;
			latch_src_b_next <= 1;
		end
		
		if (latch_src_c_next) begin
			latch_src_c_next <= 0;
			latch_src_c 	 <= 1;
		end else if (latch_src_c) begin
			src_c_latched 	<= (src_c_reg) ? reg_read_val : channel_read_val;
			latch_src_c 	<= 0;
			src_c_valid 	<= 1;
		end else if (src_c_needed & ~src_c_valid
			&& (!(src_a_needed & ~src_a_valid) | latch_src_a) && (!(src_b_needed & ~src_b_valid) | latch_src_b_next))
		begin
			if (src_c_reg)
				reg_read_addr <= {current_block, src_c[0]};
			else
				channel_read_addr <= src_c;
			latch_src_c_next <= 1;
		end
	end
	
	reg [7:0] state;
	reg executing;
	reg exec_done;
	reg block_boundary;
	
	// Sequential reset counters
	reg [$clog2(memory_size) : 0] mem_reset_ctr;
	reg [$clog2(n_blocks) 	 : 0] blk_reset_ctr;
	
	always @(posedge clk) begin
		latch_next_instr <= 0;
		block_boundary   <= 0;
		
		instr_write_enable   	<= 0;
		channel_write_enable 	<= 0;
		reg_write_enable     	<= 0;
		mem_write_enable 		<= 0;
		
		latch_instr <= 0;
		latch_instr_next <= 0;
		latch_instr_now <= 0;
		
		latch_next_instr <= 0;
		latch_next_instr_next <= 0;
		latch_next_instr_now <= 0;
		
		if (latch_instr_now) instr <= instr_read_val;
		if (latch_instr_next) latch_instr_now <= 1;
		if (latch_instr) begin
			instr_read_addr <= current_block;
			latch_instr_next <= 1;
			latch_next_instr <= 1;
		end
		
		if (latch_next_instr_now) next_instr <= instr_read_val;
		if (latch_next_instr_next) latch_next_instr_now <= 1;
		if (latch_next_instr) begin
			instr_read_addr <= next_block;
			latch_next_instr_next <= 1;
		end
	
		if (reset || full_reset) begin
			if (full_reset || resetting) begin
				resetting <= 1;
				
				mem_reset_ctr <= 0;
				blk_reset_ctr <= 0;
				
				ready <= 0;
			end else begin
				ready		<= 1;
				latch_instr <= 1;
			end
			
			executing 	   <= 0;
			block_boundary <= 0;
			
			current_block <= 0;
			state 		  <= 0;
		end else if (resetting) begin
			ready <= 0;
			
			if (mem_reset_ctr < memory_size) begin
				mem_write_addr 	 <= mem_reset_ctr;
				mem_write_val 	 <= 0;
				mem_write_enable <= 1;
				
				mem_reset_ctr <= mem_reset_ctr + 1;
			end
			
			if (blk_reset_ctr < n_blocks) begin
				instr_write_addr 	<= blk_reset_ctr;
				instr_write_val 	<= 0;
				instr_write_enable	<= 1;
				
				blk_reset_ctr <= blk_reset_ctr + 1;
			end
			
			if (mem_reset_ctr >= memory_size && blk_reset_ctr >= n_blocks) begin
				resetting 	<= 0;
				state 		<= 0;
				latch_instr <= 1;
				ready		<= 1;
			end
			
		end else if (command_reg_write || command_instr_write) begin
			if (command_reg_write) begin
				reg_write_addr   <= {command_block_target, command_reg_target[0]};
				reg_write_val    <= command_reg_write_val;
				reg_write_enable <= 1;
			end
			
			if (command_instr_write) begin
				instr_write_addr   <= command_block_target;
				instr_write_val    <= command_instr_write_val;
				instr_write_enable <= 1;
				
				last_block <= (command_block_target > last_block) ? command_block_target : last_block;
				latch_instr <= 1;
			end
		end else begin
			case (state)
				0: begin
					if (tick && enable) begin
						channel_write_addr 	 <= 0;
						channel_write_val  	 <= sample_in;
						channel_write_enable <= 1;
						latch_next_instr <= 1;
						
						executing <= 1;
						state <= 1;
						ready <= 0;
					end
				end
				
				1: begin
					state <= 2;
				end
				
				2: begin
					if (exec_done) begin
						block_boundary 	 <= 1;
						latch_next_instr <= 1;
						instr 			 <= next_instr;
						current_block 	 <= next_block;
						
						state 			 <= 1;
						
						channel_write_addr <= dest;
						
						if (write_result) begin
							channel_write_val 	 <= result_sat;
							channel_write_enable <= 1;
						end
						
						if (write_alu_result) begin
							channel_write_val 	 <= alu_result_latched;
							channel_write_enable <= 1;
						end
						
						if (write_lut_result) begin
							channel_write_val 	 <= lut_result_latched;
							channel_write_enable <= 1;
						end
						
						if (write_delay_result) begin
							channel_write_val 	 <= delay_result_latched;
							channel_write_enable <= 1;
						end
						
						if (write_mem_result) begin
							channel_write_val 	 <= mem_result_latched;
							channel_write_enable <= 1;
						end
						
						if (current_block == last_block) begin
							state <= 0;
							sample_out <= write_result ? result_sat : channels[0];
							ready <= 1;
							executing <= 0;
						end
					end
				end
				
				3: begin
					
				end
				
				4: begin
					
				end
			endcase
		end
	end
	
	reg [7:0] exec_state;
	
	reg signed [data_width - 1 : 0] shift;
	
	reg latch_shift_next;
	reg latch_shift;
	
	reg  signed [2 * data_width - 1 : 0] accumulator;
	reg  signed [2 * data_width - 1 : 0] accumulator_sat = (accumulator > sat_max) ? sat_max : ((accumulator < sat_min) ? sat_min : accumulator);
	wire signed [    data_width - 1 : 0] upper_accumulator = accumulator[2 * data_width - 1 : data_width];
	wire signed [    data_width - 1 : 0] lower_accumulator = accumulator[    data_width - 1 :          0];
	
	always @(posedge clk) begin
		latch_shift_next <= 0;
		latch_shift 	 <= 0;
		
		if (latch_shift_next)
			latch_shift <= 1;
		
		if (latch_shift)
			shift <= data_width - 1 - instr_shift;
		
		if (block_boundary | tick)
			latch_shift_next <= 1;
	end
	
	reg  signed [    data_width - 1 : 0] result;
	reg  signed [2 * data_width - 1 : 0] result_wide;
	wire signed [data_width - 1 : 0] result_sat = saturate ? ((result > sat_max) ? sat_max : ((result < sat_min) ? sat_min : result)) : result;
	reg write_result;
	
	always @(posedge clk) begin
		write_result <= 0;
		alu_trigger  <= 0;
		
		write_alu_result 	<= 0;
		write_lut_result 	<= 0;
		write_delay_result 	<= 0;
		write_mem_result 	<= 0;
		
		if (reset | full_reset | exec_done | block_boundary | tick) begin
			exec_done  <= 0;
			exec_state <= 0;
		end else if (executing) begin
			case (operation)
				`BLOCK_INSTR_NOP: begin
					if (exec_state == 0) exec_state <= 1;
					else exec_done <= 1;
				end
			
				`BLOCK_INSTR_ADD: begin
					case (exec_state)
						0: begin
							if (src_a_valid && src_b_valid) begin
								alu_op <= `ALU_OP_ADD;
								alu_a  <= src_a_latched;
								alu_b  <= src_b_latched;
								exec_state <= 1;
							end
						end
						1: exec_state <= 2;
						2: begin
							alu_result_latched <= alu_result;
							
							exec_done 	 <= 1;
							write_alu_result <= 1;
							exec_state   <= 3;
						end
					endcase
				end
				
				`BLOCK_INSTR_SUB: begin
					case (exec_state)
						0: begin
							if (src_a_valid && src_b_valid) begin
								alu_op <= `ALU_OP_SUB;
								alu_a  <= src_a_latched;
								alu_b  <= src_b_latched;
								exec_state <= 1;
							end
						end
						1: exec_state <= 2;
						2: begin
							alu_result_latched <= alu_result;
							write_alu_result <= 1;
							
							exec_done 	 <= 1;
							exec_state   <= 3;
						end
					endcase
				end
				
				`BLOCK_INSTR_CLAMP: begin
					case (exec_state)
						0: begin
							if (src_a_valid && src_b_valid) begin
								alu_op <= `ALU_OP_CLAMP;
								alu_a  <= src_a_latched;
								alu_b  <= src_b_latched;
								exec_state <= 1;
							end
						end
						1: exec_state <= 2;
						2: begin
							alu_result_latched <= alu_result;
							write_alu_result <= 1;
							
							exec_done 	 <= 1;
							exec_state   <= 3;
						end
					endcase
				end
				
				`BLOCK_INSTR_ABS: begin
					case (exec_state)
						0: begin
							if (src_a_valid && src_b_valid) begin
								alu_op <= `ALU_OP_ABS;
								alu_a  <= src_a_latched;
								alu_b  <= src_b_latched;
								exec_state <= 1;
							end
						end
						1: exec_state <= 2;
						2: begin
							alu_result_latched <= alu_result;
							write_alu_result <= 1;
							
							exec_done 	 <= 1;
							exec_state   <= 3;
						end
					endcase
				end
				
				`BLOCK_INSTR_LSH: begin
					case (exec_state)
						0: begin
							if (src_a_valid) begin
								alu_op <= `ALU_OP_LSH;
								alu_a  <= src_a_latched;
								alu_b  <= src_b;
								alu_trigger <= 1;
								exec_state  <= 1;
							end
						end
						1: begin
							if (alu_result_valid) begin
								alu_result_latched <= alu_result;
								write_alu_result <= 1;
								exec_done 	 <= 1;
								exec_state	 <= 2;
							end
						end
					endcase
				end
				
				`BLOCK_INSTR_RSH: begin
					case (exec_state)
						0: begin
							if (src_a_valid) begin
								alu_op <= `ALU_OP_RSH;
								alu_a  <= src_a_latched;
								alu_b  <= src_b;
								alu_trigger <= 1;
								exec_state  <= 1;
							end
						end
						1: begin
							if (alu_result_valid) begin
								alu_result_latched <= alu_result;
								write_alu_result <= 1;
								exec_done 	 <= 1;
								exec_state	 <= 2;
							end
						end
					endcase
				end
				
				`BLOCK_INSTR_ARSH: begin
					case (exec_state)
						0: begin
							if (src_a_valid) begin
								alu_op <= `ALU_OP_ARSH;
								alu_a  <= src_a_latched;
								alu_b  <= src_b;
								alu_trigger <= 1;
								exec_state  <= 1;
							end
						end
						1: begin
							if (alu_result_valid) begin
								alu_result_latched <= alu_result;
								write_alu_result <= 1;
								exec_done 	 <= 1;
								exec_state	 <= 2;
							end
						end
					endcase
				end
				
				`BLOCK_INSTR_MUL: begin
					case (exec_state)
						0: begin
							if (src_a_valid && src_b_valid) begin
								alu_op 		<= `ALU_OP_MUL;
								alu_a  		<= src_a_latched;
								alu_b  		<= src_b_latched;
								alu_shift 	<= shift;
								alu_trigger <= 1;
								
								exec_state <= 1;
							end
						end
						
						1: begin
							if (alu_result_valid) begin
								alu_result_latched <= alu_result;
								write_alu_result <= 1;
								exec_done 	 <= 1;
								exec_state	 <= 2;
							end
						end
					endcase
				end
				
				`BLOCK_INSTR_MADD: begin
					case (exec_state)
						0: begin
							if (src_a_valid && src_b_valid && src_c_valid) begin
								alu_op 		<= `ALU_OP_MADD;
								alu_a  		<= src_a_latched;
								alu_b  		<= src_b_latched;
								alu_c  		<= src_c_latched;
								alu_shift 	<= shift;
								alu_trigger <= 1;
								
								exec_state <= 1;
							end
						end
						
						1: begin
							if (alu_result_valid) begin
								alu_result_latched <= alu_result;
								write_alu_result <= 1;
								exec_done 	 <= 1;
								exec_state	 <= 2;
							end
						end
					endcase
				end
					
				`BLOCK_INSTR_MACZ: begin
					case (exec_state)
						0: begin
							if (src_a_valid && src_b_valid) begin
								alu_op 		<= `ALU_OP_MAC;
								alu_a  		<= src_a_latched;
								alu_b  		<= src_b_latched;
								alu_b_wide  <= 0;
								alu_no_shift<= no_shift;
								alu_shift 	<= shift;
								alu_trigger <= 1;
								
								exec_state <= 1;
							end
						end
						
						1: begin
							if (alu_result_valid) begin
								accumulator <= alu_result_wide;
								exec_done 	<= 1;
								exec_state 	<= 2;
							end
						end
					endcase
				end
					
				`BLOCK_INSTR_MAC: begin
					case (exec_state)
						0: begin
							if (src_a_valid && src_b_valid) begin
								alu_op 		<= `ALU_OP_MAC;
								alu_a  		<= src_a_latched;
								alu_b  		<= src_b_latched;
								alu_b_wide	<= accumulator;
								alu_shift 	<= shift;
								alu_trigger <= 1;
								
								exec_state <= 1;
							end
						end
						
						1: begin
							if (alu_result_valid) begin
								accumulator <= alu_result_wide;
								exec_done 	<= 1;
								exec_state 	<= 2;
							end
						end
					endcase
				end
					
				`BLOCK_INSTR_LINTERP: begin
					case (exec_state)
						0: begin
							if (src_a_valid && src_b_valid && src_c_valid) begin
								alu_op 		<= `ALU_OP_LINTERP;
								alu_a  		<= src_a_latched;
								alu_b  		<= src_b_latched;
								alu_c 		<= src_c_latched;
								alu_trigger <= 1;
								
								exec_state <= 1;
							end
						end
						
						1: begin
							if (alu_result_valid) begin
								accumulator <= alu_result;
								exec_done 	<= 1;
								exec_state 	<= 2;
							end
						end
					endcase
				end
				
				`BLOCK_INSTR_MOV: begin
					if (src_a_valid) begin
						result 		 <= src_a_latched;
						exec_done 	 <= 1;
						write_result <= 1;
					end
				end
				
				`BLOCK_INSTR_MOV_ACC: begin
					case (exec_state)
						0: begin
							exec_state <= 1;
						end
						
						1: begin
							result <= (saturate) ? accumulator_sat[data_width - 1 : 0] : accumulator[data_width - 1 : 0];
							write_result <= 1;
							exec_done <= 1;
						end
					endcase
				end
				
				`BLOCK_INSTR_MOV_UACC: begin
					case (exec_state)
						0: begin
							exec_state <= 1;
						end
						
						1: begin
							result 		 <= accumulator[2 * data_width - 1 : data_width];
							write_result <= 1;
							exec_done 	 <= 1;
						end
					endcase
				end
				
				`BLOCK_INSTR_CLEAR_ACC: begin
					case (exec_state)
						0: begin
							exec_state <= 1;
							accumulator <= 0;
						end
						
						1: begin
							exec_done <= 1;
						end
					endcase
				end
				
				`BLOCK_INSTR_ACC: begin
					case (exec_state)
						0: begin
							exec_state <= 1;
						end
					
						1: begin
							if (src_a_valid) begin
								accumulator <= (no_shift) ? accumulator + {{(data_width){src_a_latched[data_width-1]}}, src_a_latched} : accumulator + {{(data_width){src_a_latched[data_width - 1]}}, src_a_latched};
								exec_done <= 1;
							end
						end
					endcase
				end
				
				`BLOCK_INSTR_LOAD_ACC: begin
					case (exec_state)
						0: begin
							mem_read_addr <= res_addr;
							exec_state <= 1;
						end
						
						1: begin
							mem_read_addr <= res_addr + 1;
							exec_state <= 2;
						end
						
						2: begin
							accumulator <= {accumulator[data_width - 1 : 0], mem_read_val};
							exec_state <= 3;
						end
						
						3: begin
							accumulator <= {accumulator[data_width - 1 : 0], mem_read_val};
							exec_done 	<= 1;
							exec_state 	<= 4;
						end
					endcase
				end
				
				`BLOCK_INSTR_SAVE_ACC: begin
					case (exec_state)
						0: begin
							mem_write_addr 		<= res_addr;
							mem_write_val 		<= accumulator[2 * data_width - 1 : data_width];
							mem_write_enable 	<= 1;
							exec_state 			<= 1;
						end
						1: begin
							mem_write_addr 		<= res_addr + 1;
							mem_write_val 		<= accumulator[data_width - 1 : 0];
							mem_write_enable 	<= 1;
							exec_state 			<= 2;
							exec_done 			<= 1;
						end
					endcase
				end
				
				`BLOCK_INSTR_SAVE: begin
					if (src_a_valid) begin
						mem_write_addr   <= res_addr;
						mem_write_val    <= src_a_latched;
						mem_write_enable <= 1;
						
						exec_done 	 <= 1;
					end
				end
				
				`BLOCK_INSTR_LOAD: begin
					case (exec_state)
						0: begin
							mem_read_addr <= res_addr;
							exec_state <= 1;
						end
						1: exec_state <= 2;
						2: begin
							mem_result_latched <= mem_read_val;
							write_mem_result <= 1;
							exec_done <= 1;
						end
					endcase
				end
				
				`BLOCK_INSTR_LUT: begin
					case (exec_state)
						0: begin
							if (src_a_valid) begin
								lut_handle 	<= res_addr;
								lut_arg		<= src_a_latched;
								lut_req		<= 1;
								exec_state 	<= 1;
							end
						end
						1: exec_state <= 2;
						2: begin
							if (lut_ready) begin
								lut_result_latched <= lut_data;
								write_lut_result <= 1;
								exec_done <= 1;
								lut_req <= 0;
							end
						end
					endcase
				end
				
				`BLOCK_INSTR_DELAY_READ: begin
					case (exec_state)
						0: begin
							if (src_a_valid) begin
								delay_req_handle 	<= res_addr;
								delay_req_arg		<= src_a_latched;
								delay_read_req		<= 1;
								exec_state 			<= 1;
							end
						end
						1: exec_state <= 2;
						2: begin
							if (delay_read_ready) begin
								delay_result_latched <= delay_req_data_in;
								write_delay_result 	<= 1;
								exec_done 		<= 1;
								delay_read_req 	<= 0;
							end
						end
					endcase
				end
				
				`BLOCK_INSTR_DELAY_WRITE: begin
					case (exec_state)
						0: begin
							if (src_a_valid) begin
								delay_req_handle 	<= res_addr;
								delay_req_arg		<= src_a_latched;
								delay_write_req		<= 1;
								exec_state 			<= 1;
							end
						end
						1: begin
							delay_write_req <= 0;
							exec_done		<= 1;
						end
					endcase
				end
				
				`BLOCK_INSTR_FRAC_DELAY: begin
					case (exec_state)
						0: begin
							delay_req_arg 	 <= accumulator[2*data_width-1] ? 0 : upper_accumulator;
							delay_req_handle <= res_addr;
							delay_read_req 	 <= 1;
							
							exec_state <= 1;
						end
						
						1: begin
							exec_state <= 2;
						end
						
						2: begin
							if (delay_read_ready) begin
								alu_a <= delay_req_data_in;
								
								delay_req_arg 	<= delay_req_arg + 1;
								delay_read_req 	<= 1;
								exec_state 		<= 3;
							end
						end
						
						3: begin
							if (delay_read_ready) begin
								alu_b			<= delay_req_data_in;
								delay_read_req 	<= 0;
								alu_c 			<= lower_accumulator;
								alu_op 			<= `ALU_OP_LINTERP;
								alu_trigger 	<= 1;
								
								exec_state <= 4;
							end
						end
						
						4: begin
							if (alu_result_valid) begin
								alu_result_latched <= alu_result;
								
								write_alu_result <= 1;
								
								exec_done  <= 1;
								exec_state <= 5;
							end
						end
					endcase
				end
			endcase
		end
	end
endmodule

