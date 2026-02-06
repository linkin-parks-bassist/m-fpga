`default_nettype none

`include "instr_dec.vh"
`include "lut.vh"
`include "core.vh"


module dsp_core #(
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
	localparam reg_addr_w		= $clog2(n_blocks) + 1;
	
	reg [31 : 0] instrs [n_blocks - 1 : 0];
	
	reg [$clog2(n_blocks) - 1 : 0] last_block;
	reg [$clog2(n_blocks) - 1 : 0] n_blocks_running;
	
	wire [block_addr_w - 1 : 0] instr_read_addr  = instr_read_addr_ifds;
	wire [block_addr_w - 1 : 0] instr_write_addr = (resetting) ? blk_reset_ctr : command_block_target;
	reg  [31 			   : 0] instr_read_val;
	wire [31 			   : 0] instr_write_val  = (resetting) ? 0 : command_instr_write_val;
	
	wire instr_write_enable = (resetting) ? 1 : command_instr_write;
	
	reg [data_width   - 1 : 0] block_regs [2 * n_blocks - 1 : 0];
	
	wire [reg_addr_w - 1 : 0] reg_read_addr;
	wire [reg_addr_w - 1 : 0] reg_write_addr = {command_block_target, command_reg_target[0]};
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

	wire [reg_addr_w - 1 : 0] reg_read_addr_actual = {reg_read_addr[$clog2(n_blocks)+3:4], reg_read_addr[0]};

    always @(posedge clk) begin
		if (|reg_read_addr[3:1]) begin
			case (reg_read_addr[3:0])
				`POS_ONE_REGISTER_ADDR: reg_read_val <= 1 << (data_width - 2);
				`NEG_ONE_REGISTER_ADDR: reg_read_val <= 1 << (data_width - 1);
				default: 				reg_read_val <= 0;
			endcase
		end else begin
			reg_read_val <= block_regs[reg_read_addr_actual];
		end
		
		
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
	wire acc_add_enable;
	
	reg signed [2 * data_width - 1 : 0] accumulator;
	
	always @(posedge clk) begin
		if (reset) begin
			accumulator <= 0;
		end else if (full_reset) begin
			accumulator <= 0;
		end else if (acc_write_enable) begin
			accumulator <= acc_write_val;
		end else if (acc_add_enable) begin
			accumulator <= accumulator + acc_write_val;
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
	wire [3 : 0] dest_out_ifds;
	
	wire src_a_reg_out_ifds;
	wire src_b_reg_out_ifds;
	wire src_c_reg_out_ifds;
	
	wire saturate_disable_out_ifds;
	wire shift_disable_out_ifds;
	wire signedness_out_ifds;
	
	wire [4  : 0] shift_out_ifds;
	wire [11 : 0] res_addr_out_ifds;
	
	wire arg_a_needed_out_ifds;
	wire arg_b_needed_out_ifds;
	wire arg_c_needed_out_ifds;
	
	wire acc_needed_out_ifds;
	
	wire writes_channel_out_ifds;
	wire writes_acc_out_ifds;
	wire commit_flag_out_ifds;
	wire writes_external_out_ifds;
	
	wire [$clog2(`N_INSTR_BRANCHES) - 1 : 0] branch_out_ifds;
	
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
			.dest_out(dest_out_ifds),
			
			.src_a_reg_out(src_a_reg_out_ifds),
			.src_b_reg_out(src_b_reg_out_ifds),
			.src_c_reg_out(src_c_reg_out_ifds),
			
			.saturate_disable_out(saturate_disable_out_ifds),
			.shift_disable_out(shift_disable_out_ifds),
			.signedness_out(signedness_out_ifds),
			
			.shift_out(shift_out_ifds),
			.res_addr_out(res_addr_out_ifds),
			
			.arg_a_needed_out(arg_a_needed_out_ifds),
			.arg_b_needed_out(arg_b_needed_out_ifds),
			.arg_c_needed_out(arg_c_needed_out_ifds),
			
			.acc_needed_out(acc_needed_out_ifds),
			
			.writes_channel_out(writes_channel_out_ifds),
			.writes_acc_out(writes_acc_out_ifds),
			.commit_flag_out(commit_flag_out_ifds),
			.writes_external_out(writes_external_out_ifds),
			
			.branch_out(branch_out_ifds)
		);
	
	wire [`N_INSTR_BRANCHES - 1 : 0] out_ready_ofs;
	wire [`N_INSTR_BRANCHES - 1 : 0] out_valid_ofs;
	
	assign out_ready_ofs[0] = in_ready_madd;
	assign out_ready_ofs[1] = in_ready_mac;
	assign out_ready_ofs[2] = 0; // to be implemented
	assign out_ready_ofs[3] = in_ready_delay;
	assign out_ready_ofs[4] = in_ready_lut;
	assign out_ready_ofs[5] = in_ready_mem;
	
	wire [$clog2(n_blocks) - 1 : 0] block_out_ofs;
	wire [4 : 0] operation_out_ofs;
	wire [3 : 0] dest_out_ofs;
	wire signed [data_width - 1 : 0] arg_a_out_ofs;
	wire signed [data_width - 1 : 0] arg_b_out_ofs;
	wire signed [data_width - 1 : 0] arg_c_out_ofs;
	wire writes_external_out_ofs;
	wire saturate_disable_out_ofs;
	wire signedness_out_ofs;
	wire [4 : 0] shift_out_ofs;
	wire shift_disable_out_ofs;
	wire [7 : 0] res_addr_out_ofs;
	wire [8:0] commit_id_out_ofs;
	wire commit_flag_out_ofs;
	wire signed [2 * data_width - 1 : 0] accumulator_out_ofs;
	wire [3 : 0] channel_read_addr_ofs;
	wire [$clog2(n_blocks) + 4 - 1 : 0] reg_read_addr_ofs;
	
	operand_fetch_stage #(.data_width(data_width), .n_blocks(n_blocks), .n_block_regs(n_block_regs)) operand_fetch_stage
		(
			.clk(clk),
			.reset(reset | resetting),
			
			.enable(enable),
		
			.in_valid(out_valid_ifds),
			.in_ready(out_ready_ifds),
			
			.out_valid(out_valid_ofs),
			.out_ready(out_ready_ofs),
			
			.n_blocks_running(n_blocks_running),
			
			.block_in(block_out_ifds),
			.block_out(block_out_ofs),
			
			.operation_in(operation_out_ifds),
			.operation_out(operation_out_ofs),
			
			.dest_in(dest_out_ifds),
			.dest_out(dest_out_ofs),

			.src_a_in(src_a_out_ifds),
			.src_b_in(src_b_out_ifds),
			.src_c_in(src_c_out_ifds),

			.src_a_reg_in(src_a_reg_out_ifds),
			.src_b_reg_in(src_b_reg_out_ifds),
			.src_c_reg_in(src_c_reg_out_ifds),
			
			.arg_a_needed_in(arg_a_needed_out_ifds),
			.arg_b_needed_in(arg_b_needed_out_ifds),
			.arg_c_needed_in(arg_c_needed_out_ifds),

			.arg_a_out(arg_a_out_ofs),
			.arg_b_out(arg_b_out_ofs),
			.arg_c_out(arg_c_out_ofs),
			
			.saturate_disable_in(saturate_disable_out_ifds),
			.saturate_disable_out(saturate_disable_out_ofs),
			
			.signedness_in(signedness_out_ifds),
			.signedness_out(signedness_out_ofs),
			
			.acc_needed_in(acc_needed_out_ifds),

			.shift_in(shift_out_ifds),
			.shift_out(shift_out_ofs),
			.shift_disable_in(shift_disable_out_ifds),
			.shift_disable_out(shift_disable_out_ofs),
			
			.res_addr_in(res_addr_out_ifds),
			.res_addr_out(res_addr_out_ofs),
			
			.writes_channel_in(writes_channel_out_ifds),
			
			.writes_external_in(writes_external_out_ifds),
			.writes_external_out(writes_external_out_ofs),
			
			.writes_acc_in(writes_acc_out_ifds),
			
			.commit_id_out(commit_id_out_ofs),
			
			.commit_flag_in(commit_flag_out_ifds),
			.commit_flag_out(commit_flag_out_ofs),
			
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
			.acc_write_enable(acc_write_enable)
		);
	
	wire in_ready_madd;

	madd_pipeline #(.data_width(data_width), .n_blocks(n_blocks)) madd_branch
		(
			.clk(clk),
			.reset(reset | resetting),
			
			.enable(enable),
			
			.in_valid(out_valid_ofs[`INSTR_BRANCH_MADD]),
			.in_ready(in_ready_madd),
			
			.out_valid(out_valid_final_stages[`INSTR_BRANCH_MADD]),
			.out_ready(in_ready_commit_master[`INSTR_BRANCH_MADD]),
			
			.shift			 (shift_out_ofs),
			.shift_disable	 (shift_disable_out_ofs),
			.signedness		 (signedness_out_ofs),
			.saturate_disable(saturate_disable_out_ofs),
			
			.arg_a_in(arg_a_out_ofs),
			.arg_b_in(arg_b_out_ofs),
			.arg_c_in(arg_c_out_ofs),
			
			.result_out(result_final_stages[`INSTR_BRANCH_MADD]),
			
			.dest_in(dest_out_ofs),
			.dest_out(dest_final_stages[`INSTR_BRANCH_MADD]),
			
			.commit_id_in(commit_id_out_ofs),
			.commit_id_out(commit_id_final_stages[`INSTR_BRANCH_MADD]),
			
			.commit_flag_in(commit_flag_out_ofs),
			.commit_flag_out(commit_flag_final_stages[`INSTR_BRANCH_MADD])
		);

	wire in_ready_mac;

	mac_pipeline #(.data_width(data_width), .n_blocks(n_blocks)) mac_branch
		(
			.clk(clk),
			.reset(reset | resetting),
			
			.enable(enable),
			
			.in_valid(out_valid_ofs[`INSTR_BRANCH_MAC]),
			.in_ready(in_ready_mac),
			
			.out_valid(out_valid_final_stages[`INSTR_BRANCH_MAC]),
			.out_ready(in_ready_commit_master[`INSTR_BRANCH_MAC]),
			
			.shift				(shift_out_ofs),
			.shift_disable		(shift_disable_out_ofs),
			.signedness_in		(signedness_out_ofs),
			.saturate_disable_in(saturate_disable_out_ofs),
			
			.arg_a_in(arg_a_out_ofs),
			.arg_b_in(arg_b_out_ofs),
			.arg_c_in(arg_c_out_ofs),
			
			.result_out(result_final_stages[`INSTR_BRANCH_MAC]),
			
			.dest_in(dest_out_ofs),
			.dest_out(dest_final_stages[`INSTR_BRANCH_MAC]),
			
			.commit_id_in(commit_id_out_ofs),
			.commit_id_out(commit_id_final_stages[`INSTR_BRANCH_MAC]),
			
			.commit_flag_in(commit_flag_out_ofs),
			.commit_flag_out(commit_flag_final_stages[`INSTR_BRANCH_MAC])
		);

	wire in_ready_delay;
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
			
			.write(writes_external_out_ofs),
			
			.handle_in(res_addr_out_ofs),
			.handle_out(delay_req_handle),
			
			.arg_a_in(arg_a_out_ofs),
			.arg_a_out(delay_write_data),
			
			.arg_b_in(arg_b_out_ofs),
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

	wire in_ready_lut;
	wire [    data_width - 1 : 0] arg_a_out_lut;
	wire [    data_width - 1 : 0] arg_b_out_lut;
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
			.handle_out(lut_handle),
			
			.arg_a_in(arg_a_out_ofs),
			.arg_a_out(lut_arg),
			
			.arg_b_in(arg_b_out_ofs),
			.arg_b_out(),
			
			.read_req(lut_req),
			.write_req(),
			
			.dest_in(dest_out_ofs),
			.dest_out(dest_final_stages[`INSTR_BRANCH_LUT]),
			
			.read_ready(lut_ready),
			.data_in(lut_data),
			
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
		mem_read_req_1 <= 0;
		mem_read_req_2 <= 0;
		mem_write_ack  <= 0;
		
		if (!reset && !full_reset && !resetting) begin
			mem_read_req_1 <= mem_read_req;
			mem_read_req_2 <= mem_read_req_1;
			mem_write_ack  <= mem_write_req;
		end
	end
	
	wire in_ready_mem;

	resource_branch #(.data_width(data_width), .handle_width(8)) mem_stage
		(
			.clk(clk),
			.reset(reset),
			
			.enable(enable),
			
			.in_valid(out_valid_ofs[`INSTR_BRANCH_MEM]),
			.in_ready(in_ready_mem),
			
			.out_valid(out_valid_final_stages[`INSTR_BRANCH_MEM]),
			.out_ready(in_ready_commit_master[`INSTR_BRANCH_MEM]),
			
			.block_in(block_out_ofs),
			.block_out(),
			
			.write(writes_external_out_ofs),
			
			.handle_in(res_addr_out_ofs),
			
			.arg_a_in(arg_a_out_ofs),
			.arg_a_out(mem_write_val_pl),
			
			.arg_b_in(arg_b_out_ofs),
			.arg_b_out(),
			
			.dest_in(dest_out_ofs),
			.dest_out(dest_final_stages[`INSTR_BRANCH_MEM]),
			
			.handle_out(mem_read_addr),
			
			.read_req(mem_read_req),
			.write_req(mem_write_req),
			
			.data_in(mem_read_val),
			.read_ready(mem_read_req_2),
			
			.write_ack(mem_write_ack),
			
			.result_out(result_final_stages[`INSTR_BRANCH_MEM]),
			
			.commit_id_in(commit_id_out_ofs),
			.commit_id_out(commit_id_final_stages[`INSTR_BRANCH_MEM])
		);

	wire [`N_INSTR_BRANCHES - 1 : 0] out_valid_final_stages;
	wire [`N_INSTR_BRANCHES - 1 : 0] in_ready_commit_master;
	
	wire [2 * data_width - 1 	: 0] result_final_stages	[`N_INSTR_BRANCHES - 1 : 0];
	wire [3					 	: 0] dest_final_stages		[`N_INSTR_BRANCHES - 1 : 0];
	wire [8 				 	: 0] commit_id_final_stages	[`N_INSTR_BRANCHES - 1 : 0];
	wire [`N_INSTR_BRANCHES - 1 : 0] commit_flag_final_stages;

	
	wire 		[ch_addr_w  - 1 : 0] channel_write_addr_pl;
	wire signed [data_width - 1 : 0] channel_write_val_pl;
	wire channel_write_enable_pl;
	
	wire signed [2 * data_width - 1 : 0] accumulator_write_val;
	wire accumulator_write_enable;
	wire accumulator_add_enable;

	commit_master #(.data_width(data_width)) commit_master
		(
			.clk(clk),
			.reset(reset),
			
			.enable(enable),
			
			.sample_tick(tick),
			.sample_in(sample_in),
			
			.in_valid(out_valid_final_stages),
			.in_ready(in_ready_commit_master),
			
			.result(result_final_stages),
			.dest(dest_final_stages),
			
			.commit_flag(commit_flag_final_stages),
			
			.commit_id(commit_id_final_stages),
			
			.channel_write_addr(channel_write_addr),
			.channel_write_val(channel_write_val),
			.channel_write_enable(channel_write_enable),
			
			.acc_write_val(acc_write_val),
			.acc_add_enable(acc_add_enable),
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

`default_nettype wire
