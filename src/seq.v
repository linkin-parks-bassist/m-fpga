`include "instr_dec.vh"
`include "block.vh"
`include "lut.vh"
`include "seq.vh"
`include "alu.vh"

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
							state <= 3;
						end
					end
				end
				
				3: begin
					state <= 4;
				end
				
				4: begin
					ready <= 1;
					executing <= 0;
					sample_out <= channels[0];
					state <= 0;
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

        mem_write_enable    <= 0;
		
		if (reset | full_reset | exec_done | block_boundary | tick) begin
			exec_done  <= 0;
			exec_state <= 0;

            if (full_reset)
                mem_reset_ctr <= 0;
		end else if (resetting) begin
			if (mem_reset_ctr < memory_size) begin
				mem_write_addr 	 <= mem_reset_ctr;
				mem_write_val 	 <= 0;
				mem_write_enable <= 1;
				
				mem_reset_ctr <= mem_reset_ctr + 1;
			end
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

