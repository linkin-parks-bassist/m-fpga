`include "instr_dec.vh"
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
		
		input  wire [n_channels * data_width - 1 : 0] ch_in_flat,
        output wire [n_channels * data_width - 1 : 0] ch_out_flat,

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
		
		output reg lut_req,
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
	
    wire [data_width-1:0] ch_in  [0:n_channels-1];
    wire [data_width-1:0] ch_out [0:n_channels-1];


	genvar j;
	generate
		for (j = 0; j < n_channels; j = j + 1) begin : CHANNEL_OUT
            assign ch_in[j] = ch_in_flat[j * data_width +: data_width];
			assign ch_out[j] = ch_regs[j];
            assign ch_out_flat[j * data_width +: data_width] = ch_out[j];
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

    wire saturate = ~instr[operand_type_start_index + 4];

    wire [SHIFT_WIDTH - 1 : 0] instr_shift =
		(operation == `BLOCK_INSTR_BIQ_DF1) ? {{(SHIFT_WIDTH - 1){1'b0}}, 1'b1} :
		{{(SHIFT_WIDTH - `BLOCK_PMS_WIDTH){1'b0}}, instr[pms_start_index + `BLOCK_PMS_WIDTH - 1 : pms_start_index]};

    reg [`BLOCK_REG_ADDR_WIDTH - 1 : 0] ch_fetch_addr;
    reg [data_width - 1 : 0] ch_fetch;

    reg [`BLOCK_REG_ADDR_WIDTH - 1 : 0] reg_fetch_addr;
    reg [data_width - 1 : 0] reg_fetch;
	
	localparam pms_start_index = operand_type_start_index + 4;
	
	//
	// Local storage: channels and registers
	//
	reg signed [data_width  - 1 : 0] ch_regs [0 : n_channels  - 1];
	reg signed [data_width	- 1 : 0] regs 	 [0 : n_registers - 1];
	
	task automatic req_src;
		input is_reg;
		input [`BLOCK_REG_ADDR_WIDTH - 1 : 0] src;
	begin
		if (is_reg)
			reg_fetch_addr <= src;
		else
			ch_fetch_addr <= src;

        wait_one <= 1;
	end
	endtask
	
	task automatic write_dest;
		input signed [data_width-1:0] value;
	begin
		if (dest_reg)
			regs[dest - n_channels] <= value;
		else
			ch_regs[dest] <= value;
	end
	endtask
	
    reg [2:0] srcs_fetched;
    reg [2:0] srcs_reqd;

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

	reg signed [data_width - 1 : 0] prod;
	
	reg wait_one = 0;
	
	wire mul_ready = ~wait_one & mul_done;
	
	reg [8:0] state = 0;

	integer i;
	always @(posedge clk) begin
        wait_one <= 0;

        ch_fetch <= ch_regs[ch_fetch_addr];
        reg_fetch <= regs[reg_fetch_addr];
	
		//
		// Accept any register writes
		//
    /*
		if (reg_write) begin
			regs[reg_target] <= reg_val;
		end
		else if (reg_update) begin
			reg_updates[reg_target] <= reg_val;
			reg_updated[reg_target] <= 1;
		end
        */
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
			for (i = 0; i < n_channels; i = i + 1) begin
				ch_regs[i] <= ch_in[i];
			end
			
			/*for (i = 0; i < n_registers; i = i + 1) begin
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
			end*/
			
			state <= `BLOCK_STATE_FETCH_SRCS;
            srcs_fetched <= 0;
            srcs_reqd <= 0;

			done <= 0;
		end
        else if (!done) begin
            if (state == `BLOCK_STATE_FETCH_SRCS) begin
                if (!srcs_fetched[0]) begin
                    if (!srcs_reqd[0]) begin
                        req_src(src_a_reg, src_a);
                        srcs_reqd[0] <= 1;
                    end
                    else if (!wait_one) begin
                        if (src_a_reg) src_a_latched <= reg_fetch;
                        else           src_a_latched <= ch_fetch;

                        srcs_fetched[0] <= 1;
                    end
                end
                else if (!srcs_fetched[1]) begin
                    if (!srcs_reqd[1]) begin
                        req_src(src_b_reg, src_b);
                        srcs_reqd[1] <= 1;
                    end
                    else if (!wait_one) begin
                        if (src_b_reg) src_b_latched <= reg_fetch;
                        else           src_b_latched <= ch_fetch;

                        srcs_fetched[1] <= 1;
                    end
                end
                else if (!srcs_fetched[2]) begin
                    if (!srcs_reqd[2]) begin
                        req_src(src_c_reg, src_c);
                        srcs_reqd[2] <= 1;
                    end
                    else if (!wait_one) begin
                        if (src_c_reg) src_c_latched <= reg_fetch;
                        else           src_c_latched <= ch_fetch;

                        srcs_fetched[2] <= 1;
                    end
                end
                else begin
                    state <= `BLOCK_STATE_BEGIN;
                end
            end
            else begin
                case (operation)
                    `BLOCK_INSTR_NOP: begin
                        state <= `BLOCK_STATE_DONE;
                        done <= 1;
                    end
                    
                    `BLOCK_INSTR_ADD: begin
                        case (state)
                            `BLOCK_STATE_BEGIN: begin
                                summand_a <=  src_a_latched;
                                summand_b <=  src_b_latched;
                                state <= `BLOCK_STATE_ADD_WAIT;
                            end
                            
                            `BLOCK_STATE_ADD_WAIT: begin
                                state <= `BLOCK_STATE_ADD_STORE;
                            end
                            
                            `BLOCK_STATE_ADD_STORE: begin
                                write_dest(sum_final);
                                
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
                                summand_a <=  src_a_latched;
                                summand_b <= -src_b_latched;
                                state <= `BLOCK_STATE_SUB_WAIT;
                            end
                            
                            `BLOCK_STATE_SUB_WAIT: begin
                                state <= `BLOCK_STATE_SUB_STORE;
                            end
                            
                            `BLOCK_STATE_SUB_STORE: begin
                                write_dest(sum_final);
                                
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
                        write_dest(src_a_latched << 1);
                        state <= `BLOCK_STATE_DONE;
                        done <= 1;
                    end
                    
                    `BLOCK_INSTR_RSH: begin
                        write_dest(src_a_latched >> 1);
                        state <= `BLOCK_STATE_DONE;
                        done <= 1;
                    end
                    
                    `BLOCK_INSTR_RSH: begin
                        write_dest(src_a_latched >>> 1);
                        state <= `BLOCK_STATE_DONE;
                        done <= 1;
                    end
                    
                    `BLOCK_INSTR_MUL: begin
                        case (state)
                            `BLOCK_STATE_BEGIN: begin
                                mul_req_a <= src_a_latched;
                                mul_req_b <= src_b_latched;
                                
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
                                write_dest(prod);
                                
                                state <= `BLOCK_STATE_DONE;
                                done <= 1;
                            end
                            
                            default: begin end
                        endcase
                    end
                    
                    `BLOCK_INSTR_MAC: begin
                        case (state)
                            `BLOCK_STATE_BEGIN: begin
                                mul_req_a <= src_a_latched;
                                mul_req_b <= src_b_latched;
                                
                                wait_one <= 1;
                                state <= `BLOCK_STATE_MAC_MUL_WAIT;
                            end
                            
                            `BLOCK_STATE_MAC_MUL_WAIT: begin
                                if (mul_ready) begin
                                    summand_a  <= mul_result_final;
                                    summand_b <= src_c_latched;
                                    state <= `BLOCK_STATE_MAC_ADD_WAIT;
                                end
                                wait_one <= 0;
                            end
                            
                            `BLOCK_STATE_MAC_ADD_WAIT: begin
                                state <= `BLOCK_STATE_MAC_STORE;
                            end
                            
                            `BLOCK_STATE_MAC_STORE: begin
                                write_dest(sum_final);
                                
                                state <= `BLOCK_STATE_DONE;
                                done <= 1;
                            end
                            
                            default: begin end
                        endcase
                    end
                    
                    `BLOCK_INSTR_ABS: begin
                        write_dest((src_a_latched < 0) ? -src_a_latched : src_a_latched);
                        state <= `BLOCK_STATE_DONE;
                        done <= 1;
                    end
                    
                    //y[n] = b0*x[n] + b1*x[n−1] + b2*x[n−2] − a1*y[n−1] − a2*y[n−2]
                    /*`BLOCK_INSTR_BIQ_DF1: begin
                        case (state)
                            `BLOCK_STATE_BEGIN: begin
                                src_a_latched <= src_a_latched;
                                
                                
                            
                                mul_req_a <= src_a_latched;
                                mul_req_b <= regs[4];
                                
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
                                write_dest(acc_q3_29_sat_sh14[data_width - 1 : 0]);
                                
                                regs[1] <= regs[0];
                                regs[0] <= src_a_latched;
                                
                                regs[3] <= regs[2];
                                regs[2] <= acc_q3_29_sat_sh14[data_width - 1 : 0];
                                
                                state <= `BLOCK_STATE_DONE;
                                done <= 1;
                            end
                        endcase
                    end*/
                    
                    `BLOCK_INSTR_LUT: begin
                        case (state)
                            `BLOCK_STATE_BEGIN: begin
                                lut_arg 	<= ((src_a_latched << instr_shift) & ((1 << data_width) - 1)) | (src_a_latched[data_width - 1] << data_width);
                                lut_handle 	<= src_b_latched[`LUT_HANDLE_WIDTH - 1 : 0];
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
                                    
                                    mul_req_a <= src_a_latched[data_width - 1] ? -src_a_latched : src_a_latched;
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
                                write_dest(sum_final);
                                regs[2] <= sum_final;
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
                                    summand_b <= src_b_latched;
                                    
                                    state <= `BLOCK_STATE_DELAY_ADD_WAIT;
                                end
                                wait_one <= 0;
                            end
                            
                            `BLOCK_STATE_DELAY_ADD_WAIT: begin
                                write_dest(sum_final);
                                delay_buf_handle 	<= regs[0];
                                delay_buf_data_out 	<= sum_final;
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
        end
		
		if (reset) begin
			instr <= `BLOCK_INSTR_NOP;
			
			for (i = 0; i < n_channels; i = i + 1) begin
				ch_regs[i] <= 0;
			end
			
			for (i = 0; i < n_registers; i = i + 1) begin
				regs[i] <= 0;
				//reg_updates[i] <= 0;
				//reg_updated[i] <= 0;
			end
			
			lut_req <= 0;
			state <= `BLOCK_STATE_DONE;
			done <= 1;
		end
	end
endmodule
