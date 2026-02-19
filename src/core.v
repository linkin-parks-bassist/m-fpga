`include "instr_dec.vh"
`include "madd.vh"
`include "core.vh"
`include "lut.vh"

`default_nettype none

/*
 * The Central Processing Unit
 *
 * Features:
 * - emulation of `DSP blocks'
 * - pipelined execution
 * - scoreboard for hazard detection
 * - branched execution pipeline
 * - single-cycle issue 
 * - maximal throughput MAC instructions
 * 
 */

module dsp_core #(
		parameter integer data_width 	= 16,
		parameter integer n_blocks		= 256,
		parameter integer memory_size	= 1024
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
		
		input wire [$clog2(n_blocks) - 1 : 0] command_block_target,
		input wire command_reg_target,
		input wire [31 : 0] command_instr_write_val,
		input wire signed [data_width - 1 : 0] command_reg_write_val,
		
		output reg lut_req,
		output reg signed [data_width - 1 : 0] lut_handle,
		output reg signed [data_width - 1 : 0] lut_arg,
		input wire signed [data_width - 1 : 0] lut_data,
		input wire lut_valid,
		
		output wire delay_read_req,
		output wire delay_write_req,
		output wire signed [data_width - 1 : 0] delay_req_handle,
		output wire signed [data_width - 1 : 0] delay_write_data,
		output wire signed [data_width - 1 : 0] delay_write_inc,
		input  wire signed [data_width - 1 : 0] delay_read_data,
		input  wire delay_read_valid,
		input  wire delay_write_ack,
		
		input wire reg_writes_commit,
		output wire regfile_syncing,
		
		input wire full_reset,
		output reg resetting
	);
	
	localparam block_addr_w 	= $clog2(n_blocks);
	localparam mem_addr_w 		= $clog2(memory_size);
	localparam ch_addr_w 		= $clog2(16);
	localparam reg_addr_w		= $clog2(n_blocks) + 1;
	localparam full_width		= 2 * data_width + 8;
	
	always @(posedge clk) begin
		if (tick) sample_out <= channels[0];
	end
	
	reg [31 : 0] instrs [n_blocks - 1 : 0];
	
	reg [block_addr_w - 1 : 0] last_block;
	reg [block_addr_w     : 0] n_blocks_running;
	
	wire [block_addr_w - 1 : 0] block_read_addr  = block_read_addr_bfds;
	wire [block_addr_w - 1 : 0] instr_write_addr = (resetting) ? blk_reset_ctr : command_block_target;
	reg  [31 			   : 0] instr_read_val;
	wire [31 			   : 0] instr_write_val  = (resetting) ? 0 : command_instr_write_val;
	
	wire instr_write_enable = (resetting) ? 1 : command_instr_write;
	
	reg signed [data_width - 1 : 0] channels [16 - 1 : 0];
	
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
	
		instr_read_val <= instrs[block_read_addr];
		
		if (instr_write_enable & ~resetting) begin
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
	
	wire register_read_valid_a;
	wire signed [2 * data_width - 1 : 0] register_read_packed_a;
	wire signed [data_width - 1 : 0] register_0_read_a;
	wire signed [data_width - 1 : 0] register_1_read_a;
	reg regfile_sync_a;
	wire regfile_syncing_a;

	block_regfile #(.data_width(data_width), .n_blocks(n_blocks)) regfile_a (
		.clk(clk),
		.reset(reset),
		
		.n_active_blocks(n_blocks_running),
		
		.read_addr(block_read_addr),
		.read_valid(register_read_valid_a),
		.write_addr(command_block_target),
		.write_value(command_reg_write_val),
		.write_select(command_reg_target),
		.write_enable(command_reg_write & active_regfile == 1),
		
		.registers_packed_out(register_read_packed_a),
		.register_0_out(register_0_read_a),
		.register_1_out(register_1_read_a),
		
		.sync(regfile_sync_a),
		.sync_addr(block_read_addr),
		.sync_value(register_read_packed_b),
		.syncing(regfile_syncing_a)
	);
	
	wire register_read_valid_b;
	wire signed [2 * data_width - 1 : 0] register_read_packed_b;
	wire signed [data_width - 1 : 0] register_0_read_b;
	wire signed [data_width - 1 : 0] register_1_read_b;
	reg regfile_sync_b;
	wire regfile_syncing_b;

	block_regfile #(.data_width(data_width), .n_blocks(n_blocks)) regfile_b (
		.clk(clk),
		.reset(reset),
		
		.n_active_blocks(n_blocks_running),
		
		.read_addr(block_read_addr),
		.read_valid(register_read_valid_b),
		.write_addr(command_block_target),
		.write_value(command_reg_write_val),
		.write_select(command_reg_target),
		.write_enable(command_reg_write & active_regfile == 0),
		
		.registers_packed_out(register_read_packed_b),
		.register_0_out(register_0_read_b),
		.register_1_out(register_1_read_b),
		
		.sync(regfile_sync_b),
		.sync_addr(block_read_addr),
		.sync_value(register_read_packed_a),
		.syncing(regfile_syncing_b)
	);
	
	assign regfile_syncing = (active_regfile) ? regfile_syncing_b : regfile_syncing_a;

	wire [block_addr_w - 1 : 0] reg_read_addr = (command_reg_write) ? command_block_target : block_read_addr;
	
	wire signed [data_width - 1 : 0] register_0_read_value = (active_regfile) ? register_0_read_b : register_0_read_a;
	wire signed [data_width - 1 : 0] register_1_read_value = (active_regfile) ? register_1_read_b : register_1_read_a;

	reg active_regfile;
	
	always @(posedge clk) begin
		regfile_sync_a <= 0;
		regfile_sync_b <= 0;
		
		if (reset) begin
			active_regfile <= 0;
		end else if (reg_writes_commit) begin
			if (active_regfile == 0) begin
				regfile_sync_a <= 1;
			end else if (active_regfile == 1) begin
				regfile_sync_b <= 1;
			end
			
			active_regfile <= ~active_regfile;
		end
	end

	always @(posedge clk) begin
		mem_read_val <= mem[mem_read_addr];
		if (mem_write_enable)
			mem[mem_write_addr] <= mem_write_val;
	end
	
	reg signed [full_width - 1 : 0] accumulator;
	wire [full_width - 1 : 0] abs_accumulator = (accumulator < 0) ? -accumulator : accumulator;
	
	localparam signed [full_width - 1 : 0] sat_max = ( 1 << (2 * data_width - 1)) - 1;
	localparam signed [full_width - 1 : 0] sat_min = (-1 << (2 * data_width - 1));
	
	wire signed [full_width - 1 : 0] acc_sat = accumulator < sat_min ? sat_min : ((accumulator > sat_max) ? sat_max : accumulator);
	wire signed [data_width - 1 : 0] acc_norm = acc_sat[2 * data_width - 1 : data_width];
	
	wire signed [full_width - 1 : 0] accumulator_write_val;
	wire accumulator_write_enable;
	wire accumulator_add_enable;
	
	always @(posedge clk) begin
		if (reset) begin
			accumulator <= 0;
		end else if (full_reset) begin
			accumulator <= 0;
		end else if (accumulator_write_enable) begin
			if (accumulator_add_enable)
				accumulator <= accumulator + accumulator_write_val;
			else
				accumulator <= accumulator_write_val;
		end
	end
	
	/*--------------------*/
	/**********************/
	/* Execution pipeline */
	/**********************/
	/*--------------------*/
	
	/******************************/
	/* Block Fetch & Decode Stage */
	/******************************/
	
	block_fetch_decode_stage #(.data_width(data_width), .n_blocks(n_blocks)) fetch_decode_stage (
		.clk(clk),
		.reset(reset | resetting),
		.enable(enable),
	
		.n_blocks_running(n_blocks_running),
		
		.block_read_addr(block_read_addr_bfds),
		.instr_read_val(instr_read_val_bfds),
		
		.out_valid(out_valid_bfds),
		.out_ready(out_ready_bfds),
		
		.block_out(block_out_bfds),
		
		.operation_out(operation_out_bfds),
		.misc_op_out(misc_op_out_bfds),
		
		.register_0_in(register_0_read_value),
		.register_1_in(register_1_read_value),
		.register_0_out(register_0_out_bfds),
		.register_1_out(register_1_out_bfds),
		
		.src_a_out(src_a_out_bfds),
		.src_b_out(src_b_out_bfds),
		.src_c_out(src_c_out_bfds),
		.dest_out(dest_out_bfds),
		
		.src_a_reg_out(src_a_reg_out_bfds),
		.src_b_reg_out(src_b_reg_out_bfds),
		.src_c_reg_out(src_c_reg_out_bfds),
		
		.saturate_disable_out(saturate_disable_out_bfds),
		.shift_disable_out(shift_disable_out_bfds),
		.signedness_out(signedness_out_bfds),
		
		.shift_out(shift_out_bfds),
		.res_addr_out(res_addr_out_bfds),
		
		.arg_a_needed_out(arg_a_needed_out_bfds),
		.arg_b_needed_out(arg_b_needed_out_bfds),
		.arg_c_needed_out(arg_c_needed_out_bfds),
		
		.accumulator_needed_out(accumulator_needed_out_bfds),
		
		.writes_channel_out(writes_channel_out_bfds),
		.writes_accumulator_out(writes_accumulator_out_bfds),
		.commit_flag_out(commit_flag_out_bfds),
		.writes_external_out(writes_external_out_bfds),
		
		.branch_out(branch_out_bfds)
	);
	
	/***********************/
	/* Operand Fetch Stage */
	/***********************/
	
	operand_fetch_stage #(.data_width(data_width), .n_blocks(n_blocks)) operand_fetch_stage (
		.clk(clk),
		.reset(reset | resetting),
		
		.enable(enable),
	
		.in_valid(out_valid_bfds),
		.in_ready(out_ready_bfds),
		
		.out_valid(out_valid_ofs),
		.out_ready(in_ready_router),
		
		.n_blocks_running(n_blocks_running),
		
		.block_in(block_out_bfds),
		.block_out(block_out_ofs),
		
		.operation_in(operation_out_bfds),
		.operation_out(operation_out_ofs),

		.misc_op_in(misc_op_out_bfds),
		.misc_op_out(misc_op_out_ofs),
		
		.register_0_in(register_0_out_bfds),
		.register_0_out(register_0_out_ofs),
		.register_1_in(register_1_out_bfds),
		.register_1_out(register_1_out_ofs),
		
		.dest_in(dest_out_bfds),
		.dest_out(dest_out_ofs),

		.src_a_in(src_a_out_bfds),
		.src_b_in(src_b_out_bfds),
		.src_c_in(src_c_out_bfds),

		.src_a_reg_in(src_a_reg_out_bfds),
		.src_b_reg_in(src_b_reg_out_bfds),
		.src_c_reg_in(src_c_reg_out_bfds),
		
		.arg_a_needed_in(arg_a_needed_out_bfds),
		.arg_b_needed_in(arg_b_needed_out_bfds),
		.arg_c_needed_in(arg_c_needed_out_bfds),

		.arg_a_out(arg_a_out_ofs),
		.arg_b_out(arg_b_out_ofs),
		.arg_c_out(arg_c_out_ofs),
		
		.saturate_disable_in(saturate_disable_out_bfds),
		.saturate_disable_out(saturate_disable_out_ofs),
		
		.signedness_in(signedness_out_bfds),
		.signedness_out(signedness_out_ofs),
		
		.accumulator_needed_in(accumulator_needed_out_bfds),
		.accumulator_needed_out(accumulator_needed_out_ofs),

		.shift_in(shift_out_bfds),
		.shift_out(shift_out_ofs),
		.shift_disable_in(shift_disable_out_bfds),
		.shift_disable_out(shift_disable_out_ofs),
		
		.res_addr_in(res_addr_out_bfds),
		.res_addr_out(res_addr_out_ofs),
		
		.writes_channel_in(writes_channel_out_bfds),
		
		.writes_external_in(writes_external_out_bfds),
		.writes_external_out(writes_external_out_ofs),
		
		.writes_accumulator_in(writes_accumulator_out_bfds),
		
		.commit_id_out(commit_id_out_ofs),
		
		.commit_flag_in(commit_flag_out_bfds),
		.commit_flag_out(commit_flag_out_ofs),

		.branch_in(branch_out_bfds),
		.branch_out(branch_out_ofs),
		
		.channel_write_addr(channel_write_addr),
		.channel_write_val(channel_write_val),
		.channel_write_enable(channel_write_enable),
		
		.accumulator_write_enable(accumulator_write_enable)
	);
	
	/*****************/
	/* Branch Router */
	/*****************/
	
	branch_router #(.data_width(data_width), .n_blocks(n_blocks), .full_width(full_width)) router (
		.clk(clk),
		.reset(reset | resetting),
		
		.enable(enable),
	
		.sample_tick(tick),
		
		.in_valid(out_valid_ofs),
		.in_ready(in_ready_router),
		
		.out_valid(out_valid_router),
		.out_ready(out_ready_router),
		
		.block_in(block_out_ofs),
		.block_out(block_out_router),

		.operation_in(operation_out_ofs),
		.operation_out(operation_out_router),

		.misc_op_in(misc_op_out_ofs),
		.misc_op_out(misc_op_out_router),
		
		.dest_in(dest_out_ofs),
		.dest_out(dest_out_router),

		.arg_a_in(arg_a_out_ofs),
		.arg_b_in(arg_b_out_ofs),
		.arg_c_in(arg_c_out_ofs),

		.arg_a_out(arg_a_out_router),
		.arg_b_out(arg_b_out_router),
		.arg_c_out(arg_c_out_router),
		
		.saturate_disable_in(saturate_disable_out_ofs),
		.saturate_disable_out(saturate_disable_out_router),
		
		.signedness_in(signedness_out_ofs),
		.signedness_out(signedness_out_router),
		
		.writes_external_in(writes_external_out_ofs),
		.writes_external_out(writes_external_out_router),
		
		.accumulator_needed_in(accumulator_needed_out_ofs),
		.accumulator_needed_out(accumulator_needed_out_router),

		.shift_in(shift_out_ofs),
		.shift_out(shift_out_router),
		.shift_disable_in(shift_disable_out_ofs),
		.shift_disable_out(shift_disable_out_router),
		
		.res_addr_in(res_addr_out_ofs),
		.res_addr_out(res_addr_out_router),
		
		.commit_id_in(commit_id_out_ofs),
		.commit_id_out(commit_id_out_router),
		
		.commit_flag_in(commit_flag_out_ofs),
		.commit_flag_out(commit_flag_out_router),
		
		.accumulator_in(accumulator),
		.accumulator_out(accumulator_out_router),
		
		.branch(branch_out_ofs)
	);

	/**************************/
	/* Main arithmetic branch */
	/**************************/
	madd_pipeline #(.data_width(data_width), .n_blocks(n_blocks), .full_width(full_width)) madd_branch (
		.clk(clk),
		.reset(reset | resetting),
		
		.enable(enable),
		
		.in_valid(out_valid_router[`INSTR_BRANCH_MADD]),
		.in_ready(in_ready_madd),
		
		.out_valid(out_valid_final_stages[`INSTR_BRANCH_MADD]),
		.out_ready(in_ready_commit_stage[`INSTR_BRANCH_MADD]),
		
		.block_in(block_out_router),
		.block_out(block_out_final_stages[`INSTR_BRANCH_MADD]),
		
		.shift			 (shift_out_router),
		.shift_disable	 (shift_disable_out_router),
		.signedness		 (signedness_out_router),
		.saturate_disable(saturate_disable_out_router),
		
		.arg_a_in(arg_a_out_router),
		.arg_b_in(arg_b_out_router),
		.arg_c_in(arg_c_out_router),
		
		.result_out(result_final_stages[`INSTR_BRANCH_MADD]),
		
		.dest_in(dest_out_router),
		.dest_out(dest_final_stages[`INSTR_BRANCH_MADD]),
		
		.commit_id_in(commit_id_out_router),
		.commit_id_out(commit_id_final_stages[`INSTR_BRANCH_MADD]),
		
		.commit_flag_in(commit_flag_out_router),
		.commit_flag_out(commit_flag_final_stages[`INSTR_BRANCH_MADD])
	);
	
	/*********************************************/
	/* MAC branch; the accumulator owning branch */
	/*********************************************/
	mac_pipeline #(.data_width(data_width), .n_blocks(n_blocks), .full_width(full_width), .shift_type(`SHIFT_TYPE_LSH)) mac_branch (
		.clk(clk),
		.reset(reset | resetting),
		
		.enable(enable),
		
		.in_valid(out_valid_router[`INSTR_BRANCH_MAC]),
		.in_ready(in_ready_mac),
		
		.out_valid(out_valid_final_stages[`INSTR_BRANCH_MAC]),
		.out_ready(in_ready_commit_stage[`INSTR_BRANCH_MAC]),
		
		.block_in(block_out_router),
		.block_out(block_out_final_stages[`INSTR_BRANCH_MAC]),
		
		.shift				(shift_out_router),
		.shift_disable		(shift_disable_out_router),
		.signedness_in		(signedness_out_router),
		.saturate_disable_in(saturate_disable_out_router),
		
		.arg_a_in(arg_a_out_router),
		.arg_b_in(arg_b_out_router),
		.arg_c_in(arg_c_out_router),
		
		.result_out(result_final_stages[`INSTR_BRANCH_MAC]),
		
		.dest_in(dest_out_router),
		.dest_out(dest_final_stages[`INSTR_BRANCH_MAC]),
		
		.commit_id_in(commit_id_out_router),
		.commit_id_out(commit_id_final_stages[`INSTR_BRANCH_MAC]),
		
		.commit_flag_in(commit_flag_out_router),
		.commit_flag_out(commit_flag_final_stages[`INSTR_BRANCH_MAC])
	);
	
	/************************************************/
	/* Misc branch; other operations, data movement */
	/************************************************/
	misc_branch #(.data_width(data_width), .n_blocks(n_blocks), .full_width(full_width)) misc (
		.clk(clk),
		.reset(reset | resetting),
		
		.enable(enable),
				
		.in_valid(out_valid_router[`INSTR_BRANCH_MISC]),
		.in_ready(in_ready_misc),
		
		.out_valid(out_valid_final_stages[`INSTR_BRANCH_MISC]),
		.out_ready(in_ready_commit_stage[`INSTR_BRANCH_MISC]),
		
		.block_in(block_out_router),
		.block_out(block_out_final_stages[`INSTR_BRANCH_MISC]),
		
		.arg_a_in(arg_a_out_router),
		.arg_b_in(arg_b_out_router),
		.arg_c_in(arg_c_out_router),
		
		.accumulator_in(accumulator_out_router),
		
		.operation_in(operation_out_router),
		.misc_op_in(misc_op_out_router),
		
		.saturate_disable_in(saturate_disable_out_router),
		.shift_in(shift_out_router),
		
		.result_out(result_final_stages[`INSTR_BRANCH_MISC]),
		
		.dest_in(dest_out_router),
		.dest_out(dest_final_stages[`INSTR_BRANCH_MISC]),
		
		.commit_id_in(commit_id_out_router),
		.commit_id_out(commit_id_final_stages[`INSTR_BRANCH_MISC]),
		
		.commit_flag_in(commit_flag_out_router),
		.commit_flag_out(commit_flag_final_stages[`INSTR_BRANCH_MISC])
	);

	/**********/
	/* Delays */
	/**********/
	resource_branch #(.data_width(data_width), .handle_width(8), .n_blocks(n_blocks), .full_width(full_width)) delay_stage (
		.clk(clk),
		.reset(reset | resetting),
		
		.enable(enable),
		
		.in_valid(out_valid_router[`INSTR_BRANCH_DELAY]),
		.in_ready(in_ready_delay),
		
		.out_valid(out_valid_final_stages[`INSTR_BRANCH_DELAY]),
		.out_ready(in_ready_commit_stage[`INSTR_BRANCH_DELAY]),
		
		.block_in(block_out_router),
		.block_out(block_out_final_stages[`INSTR_BRANCH_DELAY]),
		
		.write(writes_external_out_router),
		
		.handle_in(res_addr_out_router),
		.handle_out(delay_req_handle),
		
		.arg_a_in(arg_a_out_router),
		.arg_a_out(delay_write_data),
		
		.arg_b_in(arg_b_out_router),
		.arg_b_out(delay_write_inc),
		
		.dest_in(dest_out_router),
		.dest_out(dest_final_stages[`INSTR_BRANCH_DELAY]),
		
		.read_req(delay_read_req),
		.write_req(delay_write_req),
		
		.read_valid(delay_read_valid),
		.write_ack(1),
		
		.data_in(delay_read_data),
		.result_out(result_final_stages[`INSTR_BRANCH_DELAY]),
		
		.commit_id_in (commit_id_out_router),
		.commit_id_out(commit_id_final_stages[`INSTR_BRANCH_DELAY])
	);

	/********/
	/* LUTs */
	/********/
	resource_branch #(.data_width(data_width), .handle_width(8), .full_width(full_width)) lut_stage (
		.clk(clk),
		.reset(reset | resetting),
		
		.enable(enable),
		
		.in_valid(out_valid_router[`INSTR_BRANCH_LUT]),
		.in_ready(in_ready_lut),
		
		.out_valid(out_valid_final_stages[`INSTR_BRANCH_LUT]),
		.out_ready(in_ready_commit_stage[`INSTR_BRANCH_LUT]),
		
		.block_in(block_out_router),
		.block_out(block_out_final_stages[`INSTR_BRANCH_LUT]),
		
		.write(0),
		.handle_in(res_addr_out_router),
		.handle_out(lut_handle),
		
		.arg_a_in(arg_a_out_router),
		.arg_a_out(lut_arg),
		
		.arg_b_in(arg_b_out_router),
		.arg_b_out(),
		
		.read_req(lut_req),
		.write_req(),
		
		.dest_in(dest_out_router),
		.dest_out(dest_final_stages[`INSTR_BRANCH_LUT]),
		
		.read_valid(lut_valid),
		.data_in(lut_data),
		
		.write_ack(1),
		
		.result_out(result_final_stages[`INSTR_BRANCH_LUT]),
		
		.commit_id_in(commit_id_out_router),
		.commit_id_out(commit_id_final_stages[`INSTR_BRANCH_LUT])
	);
	
	/**********/
	/* Memory */
	/**********/
	resource_branch #(.data_width(data_width), .handle_width(8), .full_width(full_width)) mem_stage (
		.clk(clk),
		.reset(reset | resetting),
		
		.enable(enable),
		
		.in_valid(out_valid_router[`INSTR_BRANCH_MEM]),
		.in_ready(in_ready_mem),
		
		.out_valid(out_valid_final_stages[`INSTR_BRANCH_MEM]),
		.out_ready(in_ready_commit_stage[`INSTR_BRANCH_MEM]),
		
		.block_in(block_out_router),
		.block_out(block_out_final_stages[`INSTR_BRANCH_MEM]),
		
		.write(writes_external_out_router),
		
		.handle_in(res_addr_out_router),
		
		.arg_a_in(arg_a_out_router),
		.arg_a_out(mem_write_val_pl),
		
		.arg_b_in(arg_b_out_router),
		.arg_b_out(),
		
		.dest_in(dest_out_router),
		.dest_out(dest_final_stages[`INSTR_BRANCH_MEM]),
		
		.handle_out(mem_read_addr),
		
		.read_req(mem_read_req),
		.write_req(mem_write_req),
		
		.data_in(mem_read_val),
		.read_valid(mem_read_valid),
		
		.write_ack(mem_write_ack),
		
		.result_out(result_final_stages[`INSTR_BRANCH_MEM]),
		
		.commit_id_in(commit_id_out_router),
		.commit_id_out(commit_id_final_stages[`INSTR_BRANCH_MEM])
	);
	
	/*****************/
	/* Commit stages */
	/*****************/
	genvar k;
	generate
		for (k = 0; k < `N_INSTR_BRANCHES; k = k + 1) begin : commit_stages
			commit_stage #(.data_width(data_width), .n_blocks(n_blocks)) commit_stage_inst
				(.clk(clk), .enable(enable), .reset(reset | resetting), 
				
				  .in_valid(out_valid_final_stages[k]),  .in_ready(in_ready_commit_stage[k]), 
				 .out_valid(out_valid_commit_stage[k]),  .out_ready(in_ready_commit_master[k]),
				 
						 .block_in(block_out_final_stages[k]),		 .block_out(block_out_commit_stage[k]),
						.result_in   (result_final_stages[k]),		.result_out   (result_commit_stage[k]),
					  .dest_in		 (dest_final_stages[k]),				 .dest_out(dest_commit_stage[k]),
					 .commit_id_in(commit_id_final_stages[k]),	 .commit_id_out(commit_id_commit_stage[k]),
				 .commit_flag_in(commit_flag_final_stages[k]), .commit_flag_out(commit_flag_commit_stage[k]));
		end
	endgenerate
	
	/*****************/
	/* Commit master */
	/*****************/
	commit_master #(.data_width(data_width), .n_blocks(n_blocks), .full_width(full_width)) commit_master (
		.clk(clk),
		.reset(reset | resetting),
		
		.enable(enable),
		
		.sample_tick(tick),
		.sample_in(sample_in),
		
		.in_valid(out_valid_commit_stage),
		.in_ready(in_ready_commit_master),
		
		.block_in(block_out_commit_stage),
		
		.result(result_commit_stage),
		.dest(dest_commit_stage),
		
		.commit_flag(commit_flag_commit_stage),
		
		.commit_id(commit_id_commit_stage),
		
		.channel_write_addr(channel_write_addr),
		.channel_write_val(channel_write_val),
		.channel_write_enable(channel_write_enable),
		
		.accumulator_write_val(accumulator_write_val),
		.accumulator_add_enable(accumulator_add_enable),
		.accumulator_write_enable(accumulator_write_enable),
		
		.byte_probe()
	);

	/*---------------------------*/
	/*****************************/
	/* Execution pipeline wiring */
	/*****************************/
	/*---------------------------*/
	
	/******************/
	/* Primary stages */
	/******************/
	
	// Block fetch/decode stage
	wire out_ready_bfds;
	wire out_valid_bfds;
	wire [$clog2(n_blocks) - 1 : 0] block_out_bfds;
	wire [$clog2(n_blocks) - 1 : 0] block_read_addr_bfds;
	wire [31 : 0] instr_read_val_bfds = instr_read_val;
	wire [data_width - 1 : 0] register_0_out_bfds;
	wire [data_width - 1 : 0] register_1_out_bfds;
	wire [4 : 0] operation_out_bfds;
	wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_out_bfds;
	wire [3 : 0] src_a_out_bfds;
	wire [3 : 0] src_b_out_bfds;
	wire [3 : 0] src_c_out_bfds;
	wire [3 : 0] dest_out_bfds;
	wire src_a_reg_out_bfds;
	wire src_b_reg_out_bfds;
	wire src_c_reg_out_bfds;
	wire saturate_disable_out_bfds;
	wire shift_disable_out_bfds;
	wire signedness_out_bfds;
	wire [4  : 0] shift_out_bfds;
	wire [11 : 0] res_addr_out_bfds;
	wire arg_a_needed_out_bfds;
	wire arg_b_needed_out_bfds;
	wire arg_c_needed_out_bfds;
	wire accumulator_needed_out_bfds;
	wire writes_channel_out_bfds;
	wire writes_accumulator_out_bfds;
	wire commit_flag_out_bfds;
	wire writes_external_out_bfds;
	wire [$clog2(`N_INSTR_BRANCHES) - 1 : 0] branch_out_bfds;
	
	// Operand fetch stage
	wire out_ready_ofs;
	wire out_valid_ofs;
	wire [$clog2(n_blocks) - 1 : 0] block_out_ofs;
	wire [4 : 0] operation_out_ofs;
	wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_out_ofs;
	wire [3 : 0] dest_out_ofs;
	wire signed [data_width - 1 : 0] register_0_out_ofs;
	wire signed [data_width - 1 : 0] register_1_out_ofs;
	wire signed [data_width - 1 : 0] arg_a_out_ofs;
	wire signed [data_width - 1 : 0] arg_b_out_ofs;
	wire signed [data_width - 1 : 0] arg_c_out_ofs;
	wire writes_external_out_ofs;
	wire saturate_disable_out_ofs;
	wire signedness_out_ofs;
	wire accumulator_needed_out_ofs;
	wire [4 : 0] shift_out_ofs;
	wire shift_disable_out_ofs;
	wire [7 : 0] res_addr_out_ofs;
	wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_out_ofs;
	wire commit_flag_out_ofs;
	wire [$clog2(`N_INSTR_BRANCHES) - 1 : 0] branch_out_ofs;
	
	// Branch Router
	wire in_ready_router;
	wire [`N_INSTR_BRANCHES - 1 : 0] out_valid_router;
	wire [`N_INSTR_BRANCHES - 1 : 0] out_ready_router;
	wire [$clog2(n_blocks)  - 1 : 0] block_out_router;
	wire [4 : 0] operation_out_router;
	wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_out_router;
	wire [3 : 0] dest_out_router;
	wire signed [data_width - 1 : 0] arg_a_out_router;
	wire signed [data_width - 1 : 0] arg_b_out_router;
	wire signed [data_width - 1 : 0] arg_c_out_router;
	wire saturate_disable_out_router;
	wire signedness_out_router;
	wire accumulator_needed_out_router;
	wire writes_external_out_router;
	wire [4 : 0] shift_out_router;
	wire shift_disable_out_router;
	wire [7 : 0] res_addr_out_router;
	wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_out_router;
	wire commit_flag_out_router;
	wire signed [full_width - 1 : 0] accumulator_out_router;
	assign out_ready_router[0] = in_ready_madd;
	assign out_ready_router[1] = in_ready_mac;
	assign out_ready_router[2] = in_ready_misc;
	assign out_ready_router[3] = in_ready_delay;
	assign out_ready_router[4] = in_ready_lut;
	assign out_ready_router[5] = in_ready_mem;
	
	
	/************/
	/* Branches */
	/************/
	wire in_ready_madd;
	wire in_ready_mac;
	wire in_ready_misc;
	
	// Delay branch
	wire in_ready_delay;
	wire [data_width - 1 : 0] arg_a_out_delay;
	wire [data_width - 1 : 0] arg_b_out_delay;
	wire [full_width - 1 : 0] result_out_delay;
	wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_out_delay;
	
	
	// LUT Branch
	wire in_ready_lut;
	wire [data_width - 1 : 0] arg_a_out_lut;
	wire [data_width - 1 : 0] arg_b_out_lut;
	wire [full_width - 1 : 0] result_out_lut;
	
	// Memory branch
	wire [data_width   - 1 : 0] arg_a_out_mem;
	wire [data_width   - 1 : 0] arg_b_out_mem;
	wire [full_width   - 1 : 0] result_out_mem;
	wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_out_mem;
	wire mem_read_req;
	reg  mem_read_valid;
	wire mem_write_req;
	wire mem_write_ack = 1;
	wire in_ready_mem;
	
	// Later (when there are more cores) the memory will be moved
	// outside for shared access & arbitration. For now,
	// simply give "read_ready" after a single cycle
	always @(posedge clk) begin
		mem_read_valid <= 0;
		
		if (!reset && !full_reset && !resetting)
			mem_read_valid <= mem_read_req;
	end
	
	// Commit stages
	wire [`N_INSTR_BRANCHES - 1 : 0] out_valid_final_stages;
	wire [`N_INSTR_BRANCHES - 1 : 0] out_valid_commit_stage;
	wire [$clog2(n_blocks)  - 1 : 0] block_out_final_stages [`N_INSTR_BRANCHES - 1 : 0];
	wire [$clog2(n_blocks)  - 1 : 0] block_out_commit_stage [`N_INSTR_BRANCHES - 1 : 0];
	wire [full_width    - 1 : 0] result_final_stages	[`N_INSTR_BRANCHES - 1 : 0];
	wire [full_width    - 1 : 0] result_commit_stage	[`N_INSTR_BRANCHES - 1 : 0];
	wire [3					 	: 0] dest_final_stages		[`N_INSTR_BRANCHES - 1 : 0];
	wire [3					 	: 0] dest_commit_stage		[`N_INSTR_BRANCHES - 1 : 0];
	wire [`COMMIT_ID_WIDTH  - 1 : 0] commit_id_final_stages	[`N_INSTR_BRANCHES - 1 : 0];
	wire [`COMMIT_ID_WIDTH  - 1 : 0] commit_id_commit_stage	[`N_INSTR_BRANCHES - 1 : 0];
	wire [`N_INSTR_BRANCHES - 1 : 0] commit_flag_final_stages;
	wire [`N_INSTR_BRANCHES - 1 : 0] commit_flag_commit_stage;
	wire [`N_INSTR_BRANCHES - 1 : 0] in_ready_commit_stage;
	
	/*****************/
	/* Commit master */
	/*****************/
	wire [`N_INSTR_BRANCHES - 1 : 0] in_ready_commit_master;
	
	/*******************/
	/* Reset mechanism */
	/*******************/
	
	// When the pipelines are swapped, the (now) back pipeline
	// is instructed by the control unit to perform a `full reset';
	// i.e., to reset its memory and blocks to zero, so as it can
	// be re-programmed again from a clean slate.
	
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

`default_nettype wire
