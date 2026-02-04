`include "instr_dec.vh"
`include "block.vh"
`include "lut.vh"
`include "seq.vh"
`include "alu.vh"

module dsp_core_3 #(
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
		
		input wire [$clog2(n_blocks)  	 - 1 : 0] command_block_target,
		input wire [$clog2(n_block_regs) - 1 : 0] command_reg_target,
		input wire [31					     : 0] command_instr_write_val,
		input wire signed [data_width 	 - 1 : 0] command_reg_write_val,
		
		output reg lut_req,
		output reg signed [data_width - 1 : 0] lut_handle,
		output reg signed [data_width - 1 : 0] lut_arg,
		input wire signed [data_width - 1 : 0] lut_data,
		input wire lut_ready,
		
		output wire delay_read_req,
		output wire delay_write_req,
		output wire signed [data_width - 1 : 0] delay_req_handle,
		output wire signed [data_width - 1 : 0] delay_write_data,
		output wire signed [data_width - 1 : 0] delay_write_inc,
		input  wire signed [data_width - 1 : 0] delay_read_data,
		input  wire delay_read_ready,
		input  wire delay_write_ack,
		
		input wire full_reset,
		output reg resetting
	);
	
	always @(posedge clk) begin
		if (tick)
			sample_out <= channels[0];
	end
	
	localparam block_addr_w 	= $clog2(n_blocks);
	localparam mem_addr_w 		= $clog2(memory_size);
	localparam ch_addr_w 		= $clog2(n_channels);
	localparam reg_addr_w		= $clog2(n_blocks) + $clog2(n_block_regs);
	
	reg [31 : 0] instrs [n_blocks - 1 : 0];
	
	reg [$clog2(n_blocks) - 1 : 0] last_block;
	reg [$clog2(n_blocks) - 1 : 0] n_blocks_running;
	
	wire [block_addr_w - 1 : 0] instr_read_addr  = instr_read_addr_ifds;
	wire [block_addr_w - 1 : 0] instr_write_addr = (resetting) ? blk_reset_ctr : command_block_target;
	reg  [31 			   : 0] instr_read_val;
	wire [31 			   : 0] instr_write_val  = (resetting) ? 0 : command_instr_write_val;
	
	wire instr_write_enable = (resetting) ? 1 : command_instr_write;
	
	reg [data_width   - 1 : 0] block_regs [n_block_regs * n_blocks - 1 : 0];
	
	wire [reg_addr_w - 1 : 0] reg_read_addr;
	wire [reg_addr_w - 1 : 0] reg_write_addr = {command_block_target, command_reg_target};
	reg  signed [data_width - 1 : 0] reg_read_val;
	wire signed [data_width - 1 : 0] reg_write_val = command_reg_write_val;
	wire reg_write_enable = command_reg_write;
	
	reg signed [data_width - 1 : 0] channels [n_channels - 1 : 0];
	
	wire [ch_addr_w  - 1 : 0] channel_read_addr;
	wire [ch_addr_w  - 1 : 0] channel_write_addr;
	reg  signed [data_width - 1 : 0] channel_read_val;
	wire signed [data_width - 1 : 0] channel_write_val;
	
	wire channel_write_enable;
	
	reg signed [data_width - 1 : 0] mem [memory_size - 1 : 0];
	
	wire [mem_addr_w - 1 : 0] mem_read_addr;
	wire [mem_addr_w - 1 : 0] mem_write_addr = (resetting) ? mem_reset_ctr : mem_read_addr;
	reg  signed [data_width - 1 : 0] mem_read_val;
	wire signed [data_width - 1 : 0] mem_write_val = (resetting) ? 0 : mem_write_val_pl;
	wire signed [data_width - 1 : 0] mem_write_val_pl;
	
	wire mem_write_enable = resetting | mem_write_req;
	
	always @(posedge clk) begin
		if (reset) begin
			last_block <= 0;
			n_blocks_running <= 0;
		end else if (full_reset) begin
			last_block <= 0;
			n_blocks_running <= 0;
		end
	
		instr_read_val <= instrs[instr_read_addr];
		
		if (instr_write_enable) begin
            instrs[instr_write_addr] <= instr_write_val;
            
            if (instr_write_addr >= last_block) begin
				last_block <= instr_write_addr;
				n_blocks_running <= instr_write_addr + 1;
			end
        end
    end

	integer i;
    always @(posedge clk) begin
		if (reset | resetting) begin
			for (i = 0; i < 16; i = i + 1) begin
				channels[i] = 0;
			end
		end else begin
			channel_read_val <= channels[channel_read_addr];
			if (channel_write_enable)
				channels[channel_write_addr] <= channel_write_val;
		end
    end

    always @(posedge clk) begin
		reg_read_val <= block_regs[reg_read_addr];
		if (reg_write_enable)
            block_regs[reg_write_addr] <= reg_write_val;
    end

    always @(posedge clk) begin
		mem_read_val <= mem[mem_read_addr];
		if (mem_write_enable)
            mem[mem_write_addr] <= mem_write_val;
    end
    
    wire [2 * data_width - 1 : 0] acc_write_val;
	wire acc_write_enable;
	
	reg signed [2 * data_width - 1 : 0] accumulator;
	
	always @(posedge clk) begin
		if (reset) begin
			accumulator <= 0;
		end else if (full_reset) begin
			accumulator <= 0;
		end else if (acc_write_enable) begin
			accumulator <= acc_write_val;
		end
	end
	
	wire out_ready_ifds;
	wire out_valid_ifds;
	wire [$clog2(n_blocks) - 1 : 0] block_out_ifds;
	
	wire [$clog2(n_blocks) - 1 : 0] instr_read_addr_ifds;
	wire [31 : 0] 					instr_read_val_ifds = instr_read_val;
	
	wire [4 : 0] operation_out_ifds;
	
	wire [3 : 0] src_a_out_ifds;
	wire [3 : 0] src_b_out_ifds;
	wire [3 : 0] src_c_out_ifds;
	
	wire src_a_reg_out_ifds;
	wire src_b_reg_out_ifds;
	wire src_c_reg_out_ifds;
	
	wire [3 : 0] dest_out_ifds;
	wire dest_acc_out_ifds;
	
	wire [7 : 0] res_addr_out_ifds;
	
	wire [4 : 0] shift_out_ifds;
	wire no_shift_out_ifds;
	
	wire saturate_out_ifds;
	wire subtract_out_ifds;
	wire signedness_out_ifds;
	wire use_accumulator_out_ifds;
	
	wire arg_a_needed_out_ifds;
	wire arg_b_needed_out_ifds;
	wire arg_c_needed_out_ifds;
	
	wire [`N_INSTR_BRANCHES - 1 : 0] branch_out_ifds;
	
	wire commits_out_ifds;
	wire ext_write_out_ifds;
	
	instr_fetch_decode_stage #(.data_width(data_width), .n_blocks(n_blocks), .n_block_regs(n_block_regs)) instruction_fetch_stage
		(
			.clk(clk),
			.reset(reset | resetting),
			.enable(enable),
		
			.sample_tick(tick),
		
			.n_blocks_running(n_blocks_running),
			.last_block(last_block),
			
			.instr_read_addr(instr_read_addr_ifds),
			.instr_read_val(instr_read_val_ifds),
			.out_valid(out_valid_ifds),
			.out_ready(out_ready_ifds),
			.block_out(block_out_ifds),
			.operation_out(operation_out_ifds),
			.src_a_out(src_a_out_ifds),
			.src_b_out(src_b_out_ifds),
			.src_c_out(src_c_out_ifds),
			
			.src_a_reg_out(src_a_reg_out_ifds),
			.src_b_reg_out(src_b_reg_out_ifds),
			.src_c_reg_out(src_c_reg_out_ifds),
			
			.dest_out(dest_out_ifds),
			.dest_acc_out(dest_acc_out_ifds),

			.saturate_out(saturate_out_ifds),
			.use_accumulator_out(use_accumulator_out_ifds),
			.subtract_out(subtract_out_ifds),
			.signedness_out(signedness_out_ifds),

			.instr_shift_out(shift_out_ifds),
			.no_shift_out(no_shift_out_ifds),
			
			.res_addr_out(res_addr_out_ifds),
			
			.arg_a_needed_out(arg_a_needed_out_ifds),
			.arg_b_needed_out(arg_b_needed_out_ifds),
			.arg_c_needed_out(arg_c_needed_out_ifds),
			.branch_out(branch_out_ifds),
			.commits_out(commits_out_ifds),
			.ext_write_out(ext_write_out_ifds)
		);
	
	wire [4 : 0] operation_out_ofs;
	
	wire [3 : 0] dest_out_ofs;
	wire dest_acc_out_ofs;

	wire signed [data_width - 1 : 0] arg_a_out_ofs;
	wire signed [data_width - 1 : 0] arg_b_out_ofs;
	wire signed [data_width - 1 : 0] arg_c_out_ofs;
	
	wire signed [2 * data_width - 1 : 0] accumulator_out_ofs;

	wire saturate_out_ofs;
	wire use_accumulator_out_ofs;
	wire subtract_out_ofs;
	wire signedness_out_ofs;

	wire [4 : 0] shift_out_ofs;
	wire no_shift_out_ofs;
	
	wire [7 : 0] res_addr_out_ofs;

	wire [`N_INSTR_BRANCHES - 1 : 0] out_ready_ofs = {in_ready_muls, in_ready_delay, in_ready_lut, in_ready_mem};
	wire [`N_INSTR_BRANCHES - 1 : 0] out_valid_ofs;
	
	wire commits_out_ofs;
	wire [8:0] commit_id_out_ofs;
	wire ext_write_out_ofs;
	
	wire [$clog2(n_blocks) - 1 : 0] block_out_ofs;

	operand_fetch_stage #(.data_width(data_width), .n_blocks(n_blocks), .n_block_regs(n_block_regs)) operand_fetch_stage
		(
			.clk(clk),
			.reset(reset | resetting),
			.enable(enable),
		
			.n_blocks_running(n_blocks_running),
		
			.sample_tick(tick),
			.in_valid(out_valid_ifds),
			.in_ready(out_ready_ifds),
			.block_in(block_out_ifds),
			.block_out(block_out_ofs),
			.operation_in(operation_out_ifds),
			.dest_in(dest_out_ifds),
			.dest_acc_in(dest_acc_out_ifds),

			.src_a_in(src_a_out_ifds),
			.src_b_in(src_b_out_ifds),
			.src_c_in(src_c_out_ifds),

			.src_a_reg_in(src_a_reg_out_ifds),
			.src_b_reg_in(src_b_reg_out_ifds),
			.src_c_reg_in(src_c_reg_out_ifds),
			.saturate_in(saturate_out_ifds),
			.use_accumulator_in(use_accumulator_out_ifds),
			.subtract_in(subtract_out_ifds),
			.signedness_in(signedness_out_ifds),

			.shift_in(shift_out_ifds),
			.no_shift_in(no_shift_out_ifds),
			
			.res_addr_in(res_addr_out_ifds),
			
			.arg_a_needed_in(arg_a_needed_out_ifds),
			.arg_b_needed_in(arg_b_needed_out_ifds),
			.arg_c_needed_in(arg_c_needed_out_ifds),
			
			.saturate_out(saturate_out_ofs),
			.use_accumulator_out(use_accumulator_out_ofs),
			.subtract_out(subtract_out_ofs),
			.signedness_out(signedness_out_ofs),

			.shift_out(shift_out_ofs),
			.no_shift_out(no_shift_out_ofs),
			.res_addr_out(res_addr_out_ofs),
			
			.operation_out(operation_out_ifds),
			.dest_out(dest_out_ofs),
			.dest_acc_out(dest_acc_out_ofs),
			
			.commits(commits_out_ifds),
			.commits_out(commits_out_ofs),
			.ext_write_in(ext_write_out_ifds),
			.ext_write_out(ext_write_out_ofs),
			.commit_id_out(commit_id_out_ofs),

			.arg_a_out(arg_a_out_ofs),
			.arg_b_out(arg_b_out_ofs),
			.arg_c_out(arg_c_out_ofs),

			.accumulator_in(accumulator),
			.accumulator_out(accumulator_out_ofs),
			
			.branch(branch_out_ifds),
			
			.channel_write_addr(channel_write_addr),
			.channel_write_val(channel_write_val),
			.channel_write_enable(channel_write_enable),
			.channel_read_addr(channel_read_addr),
			.channel_read_val(channel_read_val),
			
			.reg_read_addr(reg_read_addr),
			.reg_read_val(reg_read_val),
			
			.acc_write_val(acc_write_val),
			.acc_write_enable(acc_write_enable),
			
			.out_valid(out_valid_ofs),
			.out_ready(out_ready_ofs)
		);
	
	wire signed [2 * data_width - 1 : 0] accumulator_in;
	
	wire [7 : 0] operation_out_muls;
	
	wire [4:0] shift_out_muls;
	
	wire no_shift_out_muls;
	wire saturate_out_muls;
	wire signedness_out_muls;
	wire use_accumulator_out_muls;
	wire subtract_out_muls;
	
	wire signed [data_width - 1 : 0] arg_a_out_muls;
	wire signed [data_width - 1 : 0] arg_b_out_muls;
	wire signed [data_width - 1 : 0] arg_c_out_muls;
	
	wire signed [2 * data_width - 1 : 0] product_out_muls;
	
	wire signed [2 * data_width - 1 : 0] accumulator_out_muls;
	
	wire [3:0] dest_out_muls;
	wire dest_acc_out_muls;
	
	wire [8:0] commit_id_out_muls;

	multiply_stage #(.data_width(data_width)) multiply_stage
		(
			.clk(clk),
			.reset(reset | resetting),
			.enable(enable),
			.in_valid(out_valid_ofs[`INSTR_BRANCH_MAIN]),
			.in_ready(in_ready_muls),
			.operation_in(operation_out_ofs),
			.shift_in(shift_out_ofs),
			.shift_out(shift_out_muls),
			.no_shift_in(no_shift_out_ofs),
			.no_shift_out(no_shift_out_muls),
			.saturate_in(saturate_out_ofs),
			.signedness_in(signedness_out_ofs),
			.use_accumulator_in(use_accumulator_out_ofs),
			.subtract_in(subtract_out_ofs),
			.arg_a_in(arg_a_out_ofs),
			.arg_b_in(arg_b_out_ofs),
			.arg_c_in(arg_c_out_ofs),
			.accumulator_in(accumulator_out_ofs),
			.dest_in(dest_out_ofs),
			.dest_acc_in(dest_acc_out_ofs),
			.operation_out(operation_out_muls),
			.saturate_out(saturate_out_muls),
			.signedness_out(signedness_out_muls),
			.use_accumulator_out(use_accumulator_out_muls),
			.subtract_out(subtract_out_muls),
			.arg_a_out(arg_a_out_muls),
			.arg_b_out(arg_b_out_muls),
			.arg_c_out(arg_c_out_muls),
			.product_out(product_out_muls),
			.accumulator_out(accumulator_out_muls),
			.dest_out(dest_out_muls),
			.dest_acc_out(dest_acc_out_muls),
			.out_valid(out_valid_muls),
			.out_ready(in_ready_shs),
			.commit_id_in(commit_id_out_ofs),
			.commit_id_out(commit_id_out_muls)
		);

	wire in_ready_shs;
	wire out_valid_shs;
	
	wire [4:0] operation_out_shs;
	
	wire signed [data_width - 1 : 0] arg_a_out_shs;
	wire signed [data_width - 1 : 0] arg_b_out_shs;
	wire signed [data_width - 1 : 0] arg_c_out_shs;
	
	wire signed [2 * data_width - 1 : 0] accumulator_out_shs;
	wire signed [2 * data_width - 1 : 0] product_out_shs;
	
	wire [8:0] commit_id_out_shs;
	
	wire [3:0] dest_out_shs;

	shift_stage #(.data_width(data_width)) shift_stage
		(
			.clk(clk),
			.reset(reset | resetting),
			.enable(enable),
			.in_valid(out_valid_muls),
			.in_ready(in_ready_shs),
			.operation_in(operation_out_muls),
			.shift_in(shift_out_muls),
			.no_shift_in(no_shift_out_muls),
			.saturate_in(saturate_out_muls),
			.signedness_in(signedness_out_muls),
			.use_accumulator_in(use_accumulator_out_muls),
			.subtract_in(subtract_out_muls),
			.product_in(product_out_muls),
			.arg_a_in(arg_a_out_muls),
			.arg_b_in(arg_b_out_muls),
			.arg_c_in(arg_c_out_muls),
			.accumulator_in(accumulator_out_muls),
			.dest_in(dest_out_muls),
			.dest_acc_in(dest_acc_out_muls),
			.operation_out(operation_out_shs),
			.saturate_out(saturate_out_shs),
			.signedness_out(signedness_out_shs),
			.use_accumulator_out(use_accumulator_out_shs),
			.subtract_out(subtract_out_shs),
			.arg_a_out(arg_a_out_shs),
			.arg_b_out(arg_b_out_shs),
			.arg_c_out(arg_c_out_shs),
			.product_out(product_out_shs),
			.accumulator_out(accumulator_out_shs),
			.dest_out(dest_out_shs),
			.dest_acc_out(dest_acc_out_shs),
			.out_valid(out_valid_shs),
			.out_ready(in_ready_ariths),
			.commit_id_in(commit_id_out_muls),
			.commit_id_out(commit_id_out_shs)
		);

	wire in_ready_ariths;
	wire out_valid_ariths;
	wire [8:0] commit_id_out_ariths;
	
	wire [2 * data_width - 1 : 0] result_out_ariths;
	
	wire [3:0] dest_out_ariths;

	arithmetic_stage #(.data_width(data_width)) arithmetic_stage
		(
			.clk(clk),
			.reset(reset | resetting),
			.enable(enable),
			.in_valid(out_valid_shs),
			.in_ready(in_ready_ariths),
			.operation_in(operation_out_shs),
			.saturate_in(saturate_out_shs),
			.signedness_in(signedness_out_shs),
			.use_accumulator_in(use_accumulator_out_shs),
			.subtract_in(subtract_out_shs),
			.product_in(product_out_shs),
			.arg_a_in(arg_a_out_shs),
			.arg_b_in(arg_b_out_shs),
			.arg_c_in(arg_c_out_shs),
			.accumulator_in(accumulator_out_shs),
			.dest_in(dest_out_shs),
			.dest_out(dest_out_ariths),
			.dest_acc_in(dest_acc_out_shs),
			.saturate_out(saturate_out_ariths),
			.result_out(result_out_ariths),
			.dest_acc_out(dest_acc_out_ariths),
			.out_valid(out_valid_ariths),
			.out_ready(in_ready_sats),
			.commit_id_in(commit_id_out_shs),
			.commit_id_out(commit_id_out_ariths)
		);

	wire in_ready_sats;
	wire [8:0] commit_id_out_sats;

	saturate_stage #(.data_width(data_width)) saturate_stage
		(
			.clk(clk),
			.reset(reset | resetting),
			.enable(enable),
			.in_valid(out_valid_ariths),
			.in_ready(in_ready_sats),
			.saturate_in(saturate_out_ariths),
			.dest_in(dest_out_ariths),
			.dest_acc_in(dest_acc_out_ariths),
			.result_in(result_out_ariths),
			.result_out		(   result_final_stages[`INSTR_BRANCH_MAIN]),
			.dest_out		(     dest_final_stages[`INSTR_BRANCH_MAIN]),
			.dest_acc_out	( dest_acc_final_stages[`INSTR_BRANCH_MAIN]),
			.out_valid		(out_valid_final_stages[`INSTR_BRANCH_MAIN]),
			.out_ready		(in_ready_commit_master[`INSTR_BRANCH_MAIN]),
			.commit_id_in(commit_id_out_ariths),
			.commit_id_out	(commit_id_final_stages[`INSTR_BRANCH_MAIN])
		);

	wire [data_width - 1 : 0] arg_a_out_delay;
	wire [data_width - 1 : 0] arg_b_out_delay;
	wire [2 * data_width - 1 : 0] result_out_delay;

	wire [8:0] commit_id_out_delay;

	resource_branch #(.data_width(data_width), .handle_width(8), .n_blocks(n_blocks)) delay_stage
		(
			.clk(clk),
			.reset(reset | resetting),
			.enable(enable),
			
			.in_valid(out_valid_ofs[`INSTR_BRANCH_DELAY]),
			.in_ready(in_ready_delay),
			
			.out_valid(out_valid_final_stages[`INSTR_BRANCH_DELAY]),
			.out_ready(in_ready_commit_master[`INSTR_BRANCH_DELAY]),
			
			.block_in(block_out_ofs),
			.block_out(),
			
			.write(ext_write_out_ofs),
			
			.handle_in(res_addr_out_ofs),
			
			.arg_a_in(arg_a_out_ofs),
			.arg_b_in(arg_b_out_ofs),
			
			.handle_out(delay_req_handle),
			.arg_a_out(delay_write_data),
			.arg_b_out(delay_write_inc),
			
			.dest_in(dest_out_ofs),
			.dest_out(dest_final_stages[`INSTR_BRANCH_DELAY]),
			
			.read_req(delay_read_req),
			.write_req(delay_write_req),
			
			.read_ready(delay_read_ready),
			.write_ack(1),
			
			.data_in(delay_read_data),
			.result_out(result_final_stages[`INSTR_BRANCH_DELAY]),
			
			.commit_id_in (commit_id_out_ofs),
			.commit_id_out(commit_id_final_stages[`INSTR_BRANCH_DELAY])
		);

	wire [data_width - 1 : 0] arg_a_out_lut;
	wire [data_width - 1 : 0] arg_b_out_lut;
	wire [2 * data_width - 1 : 0] result_out_lut;

	resource_branch #(.data_width(data_width), .handle_width(8)) lut_stage
		(
			.clk(clk),
			.reset(reset | resetting),
			.enable(enable),
			.in_valid(out_valid_ofs[`INSTR_BRANCH_LUT]),
			.in_ready(in_ready_lut),
			.out_valid(out_valid_final_stages[`INSTR_BRANCH_LUT]),
			.out_ready(in_ready_commit_master[`INSTR_BRANCH_LUT]),
			
			.block_in(block_out_ofs),
			.block_out(),
			
			.write(0),
			.handle_in(res_addr_out_ofs),
			.arg_a_in(arg_a_out_ofs),
			.arg_b_in(arg_b_out_ofs),
			.read_req(lut_req),
			.write_req(),
			.handle_out(lut_handle),
			.arg_a_out(lut_req_arg),
			
			.dest_in(dest_out_ofs),
			.dest_out(dest_final_stages[`INSTR_BRANCH_LUT]),
			
			.data_in(lut_data),
			.read_ready(lut_ready),
			.write_ack(1),
			
			.result_out(result_final_stages[`INSTR_BRANCH_LUT]),
			
			.commit_id_in(commit_id_out_ofs),
			.commit_id_out(commit_id_final_stages[`INSTR_BRANCH_LUT])
		);

	wire [data_width - 1 : 0] arg_a_out_mem;
	wire [data_width - 1 : 0] arg_b_out_mem;
	
	wire [2 * data_width - 1 : 0] result_out_mem;

	wire [8:0] commit_id_out_mem;
	
	wire mem_read_req;
	reg  mem_read_req_1;
	reg  mem_read_req_2;
	
	wire mem_write_req;
	reg  mem_write_ack;
	
	always @(posedge clk) begin
		if (reset) begin
			mem_read_req_1 <= 0;
			mem_read_req_2 <= 0;
			mem_write_ack  <= 0;
		end if (full_reset) begin
			mem_read_req_1 <= 0;
			mem_read_req_2 <= 0;
			mem_write_ack  <= 0;
		end if (resetting) begin
			mem_read_req_1 <= 0;
			mem_read_req_2 <= 0;
			mem_write_ack  <= 0;
		end else begin
			mem_read_req_1 <= mem_read_req;
			mem_read_req_2 <= mem_read_req_1;
			mem_write_ack  <= mem_write_req;
		end
	end

	resource_branch #(.data_width(data_width), .handle_width(8)) mem_stage
		(
			.clk(clk),
			.reset(reset),
			
			.in_valid(out_valid_ofs[`INSTR_BRANCH_MEM]),
			.in_ready(in_ready_mem),
			
			.out_valid(out_valid_final_stages[`INSTR_BRANCH_MEM]),
			.out_ready(in_ready_commit_master[`INSTR_BRANCH_MEM]),
			
			.block_in(block_out_ofs),
			.block_out(),
			
			.write(ext_write_out_ofs),
			
			.handle_in(res_addr_out_ofs),
			
			.arg_a_in(arg_a_out_ofs),
			.arg_b_in(arg_b_out_ofs),
			
			.dest_in(dest_out_ofs),
			.dest_out(dest_final_stages[`INSTR_BRANCH_MEM]),
			
			.handle_out(mem_read_addr),
			
			.read_req(mem_read_req),
			.write_req(mem_write_req),
			
			.arg_a_out(mem_write_val_pl),
			.arg_b_out(),
			
			.data_in(mem_read_val),
			.read_ready(mem_read_req_2),
			
			.write_ack(mem_write_ack),
			
			.result_out(result_final_stages[`INSTR_BRANCH_MEM]),
			
			.commit_id_in(commit_id_out_ofs),
			.commit_id_out(commit_id_out_mem)
		);

	wire [`N_INSTR_BRANCHES - 1 : 0]							 out_valid_final_stages;
	wire [2 * data_width - 1 	: 0]    result_final_stages[`N_INSTR_BRANCHES - 1 : 0];
	wire [3					 	: 0]      dest_final_stages[`N_INSTR_BRANCHES - 1 : 0];
	wire [`N_INSTR_BRANCHES - 1 : 0]  dest_acc_final_stages;
	wire [8 				 	: 0] commit_id_final_stages[`N_INSTR_BRANCHES - 1 : 0];

	wire [`N_INSTR_BRANCHES - 1 : 0] in_ready_commit_master;
	
	wire 		[ch_addr_w  - 1 : 0] channel_write_addr_pl;
	wire signed [data_width - 1 : 0] channel_write_val_pl;
	wire channel_write_enable_pl;
	
	wire signed [2 * data_width - 1 : 0] accumulator_write_val;
	wire accumulator_write_enable;

	commit_master #(.data_width(data_width)) commit_master
		(
			.clk(clk),
			.reset(reset),
			.enable(enable),
			.sample_tick(tick),
			.sample_in(sample_in),
			
			.in_valid(out_valid_final_stages),
			.result(result_final_stages),
			.dest(dest_final_stages),
			.dest_acc(dest_acc_final_stages),
			.commit_id(commit_id_final_stages),
			.in_ready(in_ready_commit_master),
			.channel_write_addr(channel_write_addr),
			.channel_write_val(channel_write_val),
			.channel_write_enable(channel_write_enable),
			.acc_write_val(acc_write_val),
			.acc_write_enable(acc_write_enable)
		);
	
	// Sequential reset counters
	reg [$clog2(memory_size) : 0] mem_reset_ctr;
	reg [$clog2(n_blocks) 	 : 0] blk_reset_ctr;
	
	always @(posedge clk) begin
		if (reset) begin
			ready <= 1;
			resetting <= 0;
        end else if (full_reset) begin
            resetting <= 1;
		end else if (resetting) begin
			ready <= 0;
            
			if (blk_reset_ctr < n_blocks) begin
				blk_reset_ctr <= blk_reset_ctr + 1;
			end

			if (mem_reset_ctr < memory_size) begin
				mem_reset_ctr <= mem_reset_ctr + 1;
			end
			
			if (mem_reset_ctr >= memory_size && blk_reset_ctr >= n_blocks) begin
				resetting 	<= 0;
				ready		<= 1;
			end
		end else if (enable) begin
			
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
		
		input wire [$clog2(n_blocks)  	 - 1 : 0] command_block_target,
		input wire [$clog2(n_block_regs) - 1 : 0] command_reg_target,
		input wire [31					     : 0] command_instr_write_val,
		input wire signed [data_width 	 - 1 : 0] command_reg_write_val,
		
		output reg lut_req,
		output reg signed [data_width - 1 : 0] lut_handle,
		output reg signed [data_width - 1 : 0] lut_arg,
		input wire signed [data_width - 1 : 0] lut_data,
		input wire lut_ready,
		
		output reg delay_read_req,
		output reg delay_write_req,
		output reg signed [data_width - 1 : 0] delay_req_handle,
		output reg signed [data_width - 1 : 0] delay_req_arg,
		input wire signed [data_width - 1 : 0] delay_read_data,
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
		instr_read_val <= instrs[instr_read_addr];
		if (instr_write_enable)
            instrs[instr_write_addr] <= instr_write_val;
    end

    always @(posedge clk) begin
		channel_read_val <= channels[channel_read_addr];
		if (channel_write_enable)
            channels[channel_write_addr] <= channel_write_val;
    end

    always @(posedge clk) begin
		reg_read_val <= block_regs[reg_read_addr];
		if (reg_write_enable)
            block_regs[reg_write_addr] <= reg_write_val;
    end

    always @(posedge clk) begin
		mem_read_val <= mem[mem_read_addr];
		if (mem_write_enable)
            mem[mem_write_addr] <= mem_write_val;
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
		if (reset) begin
			latch_src_a_next <= 0;
			latch_src_a		 <= 0;
			src_a_valid	 	 <= 0;
			latch_src_b_next <= 0;
			latch_src_b		 <= 0;
			src_b_valid	 	 <= 0;
			latch_src_c_next <= 0;
			latch_src_c		 <= 0;
			src_c_valid	 	 <= 0;
        end else if (full_reset | exec_done | (tick & enable)) begin
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
		if (reset) begin
			ready		<= 1;
			latch_instr <= 1;
			
			executing 	   <= 0;
			block_boundary <= 0;
			
			current_block <= 0;
			state 		  <= 0;
        end else if (full_reset) begin
            resetting <= 1;
				
            blk_reset_ctr <= 0;
            
            ready <= 0;
            
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
			
		end else begin
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

            if (command_reg_write || command_instr_write) begin
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
	
    reg alu_a_hold = 0;
    reg alu_b_hold = 0;

    reg alu_a_latch_delay = 0;
    reg alu_b_latch_delay = 0;

	always @(posedge clk) begin
		if (reset) begin
			exec_done  <= 0;
			exec_state <= 0;

            write_result <= 0;
            alu_trigger  <= 0;
            
            write_alu_result 	<= 0;
            write_lut_result 	<= 0;
            write_delay_result 	<= 0;
            write_mem_result 	<= 0;

            mem_write_enable    <= 0;
        end else if (full_reset) begin
            exec_done  <= 0;
			exec_state <= 0;

            write_result <= 0;
            alu_trigger  <= 0;
            
            write_alu_result 	<= 0;
            write_lut_result 	<= 0;
            write_delay_result 	<= 0;
            write_mem_result 	<= 0;

            mem_write_enable    <= 0;

            if (full_reset)
                mem_reset_ctr <= 0;
		end else if (resetting) begin
            write_result <= 0;
            alu_trigger  <= 0;
            
            write_alu_result 	<= 0;
            write_lut_result 	<= 0;
            write_delay_result 	<= 0;
            write_mem_result 	<= 0;

            mem_write_enable    <= 0;

			if (mem_reset_ctr < memory_size) begin
				mem_write_addr 	 <= mem_reset_ctr;
				mem_write_val 	 <= 0;
				mem_write_enable <= 1;
				
				mem_reset_ctr <= mem_reset_ctr + 1;
			end
        end else begin
            write_result <= 0;
            alu_trigger  <= 0;
            
            write_alu_result 	<= 0;
            write_lut_result 	<= 0;
            write_delay_result 	<= 0;
            write_mem_result 	<= 0;

            mem_write_enable    <= 0;

            if (!alu_a_hold) begin
                alu_a <= src_a_latched;
                if (alu_a_latch_delay)
                    alu_a <= delay_read_data;
            end

            if (!alu_b_hold) begin
                alu_b <= src_b_latched;
                if (alu_b_latch_delay)
                    alu_b <= delay_read_data;
            end

            delay_req_handle <= res_addr;

            lut_handle  <= res_addr;
            lut_arg		<= src_a_latched;

            if (exec_done | block_boundary | tick) begin
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
                                    delay_req_arg		<= src_a_latched;
                                    delay_read_req		<= 1;
                                    exec_state 			<= 1;
                                end
                            end
                            1: exec_state <= 2;
                            2: begin
                                if (delay_read_ready) begin
                                    delay_result_latched <= delay_read_data;
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
                                delay_read_req 	 <= 1;

                                alu_a_latch_delay <= 1;
                                alu_b_latch_delay <= 1;
                                
                                exec_state <= 1;
                            end
                            
                            1: begin
                                exec_state <= 2;
                            end
                            
                            2: begin
                                if (delay_read_ready) begin
                                    alu_a_hold <= 1;
                                    alu_a_latch_delay <= 0;

                                    delay_req_arg 	<= delay_req_arg + 1;
                                    delay_read_req 	<= 1;
                                    exec_state 		<= 3;
                                end
                            end
                            
                            3: begin
                                if (delay_read_ready) begin
                                    alu_b_hold <= 1;
                                    alu_b_latch_delay <= 0;

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
                                    
                                    alu_a_hold <= 0;
                                    alu_b_hold <= 0;

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
    end
endmodule

