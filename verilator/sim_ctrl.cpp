#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "sim_main.h"

#define IBM(x) ((1u << x) - 1)

int range_bits(int x, int n, int start)
{
	return (x >> start) & ((1u << n) - 1);
}

uint32_t encode_type_a_instr(int opcode, int src_a, int src_b, int src_c, int dest,
										 int a_reg, int b_reg, int c_reg, int dest_reg, int shift, int sat)
{
	return ((uint32_t)opcode)
		| ((uint32_t)(src_a & IBM(BLOCK_REG_ADDR_WIDTH)) 	<< (0 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH))
		| ((uint32_t)(src_b & IBM(BLOCK_REG_ADDR_WIDTH)) 	<< (1 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH))
		| ((uint32_t)(src_c & IBM(BLOCK_REG_ADDR_WIDTH)) 	<< (2 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH))
		| ((uint32_t)(dest  & IBM(BLOCK_REG_ADDR_WIDTH))  	<< (3 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH))
		| ((uint32_t)!!a_reg 	<< (BLOCK_INSTR_OP_TYPE_START + 0))
		| ((uint32_t)!!b_reg 	<< (BLOCK_INSTR_OP_TYPE_START + 1))
		| ((uint32_t)!!c_reg 	<< (BLOCK_INSTR_OP_TYPE_START + 2))
		| ((uint32_t)!!dest_reg	<< (BLOCK_INSTR_OP_TYPE_START + 3))
		| ((uint32_t)(shift & IBM(SHIFT_WIDTH)) << (BLOCK_INSTR_PMS_START))
		| ((uint32_t)!!sat   << (BLOCK_INSTR_OP_TYPE_START + 4));
}

uint32_t encode_type_b_instr(int opcode, int src_a, int src_b, int dest, int res_addr)
{
	return ((uint32_t)opcode)
		| ((uint32_t)(src_a & IBM(BLOCK_REG_ADDR_WIDTH)) 	<< (0 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH))
		| ((uint32_t)(src_b & IBM(BLOCK_REG_ADDR_WIDTH)) 	<< (1 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH))
		| ((uint32_t)(dest  & IBM(BLOCK_REG_ADDR_WIDTH))  	<< (2 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH))
		| ((uint32_t)(res_addr & IBM(BLOCK_RES_ADDR_WIDTH)) << (3 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH));
}

uint32_t encode_block_instr(int opcode, int src_a, int src_b, int src_c, int dest, int a_reg, int b_reg, int c_reg, int dest_reg, int shift, int sat, int res_addr)
{
	if (opcode == BLOCK_INSTR_DELAY
	 || opcode == BLOCK_INSTR_SAVE
	 || opcode == BLOCK_INSTR_LOAD)
	{
		return encode_type_b_instr(opcode, src_a, src_b, dest, res_addr);
	}
	else
	{
		return encode_type_a_instr(opcode, src_a, src_b, src_c, dest, a_reg, b_reg, c_reg, dest_reg, shift, sat);
	}
}

uint32_t encode_block_instr_str(block_instr instr)
{
	return encode_block_instr(
		instr.opcode,
		instr.src_a, instr.src_b, instr.src_c, instr.dest,
		instr.src_a_reg, instr.src_b_reg, instr.src_c_reg, instr.dest_reg,
		instr.shift, instr.sat, instr.res_addr
	);
}

int opcode_format(int opcode)
{
	return (opcode == BLOCK_INSTR_DELAY
		 || opcode == BLOCK_INSTR_SAVE
		 || opcode == BLOCK_INSTR_LOAD) ? INSTR_FORMAT_B : INSTR_FORMAT_A;
}

int instr_format(block_instr instr)
{
	return opcode_format(instr.opcode);
}

resource_report empty_resource_report()
{
	return (resource_report){0, 0, 0};
}

void print_resource_report(const resource_report *res)
{
	if (!res) 	printf("(NULL)\n");
	else 		printf("Resource usage report:\n\t Blocks: %d\n\t Delays: %d\n\t Memory: %d\n", res->block_usage, res->sram_delay_usage, res->mem_usage);
}

block_instr unpack_instr_code(uint32_t code)
{
	block_instr result;
	
	result.opcode = range_bits(code, BLOCK_INSTR_OP_WIDTH, 0);
	
	result.src_a = range_bits(code, BLOCK_REG_ADDR_WIDTH, 0 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH);
	result.src_b = range_bits(code, BLOCK_REG_ADDR_WIDTH, 1 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH);
	
	switch (instr_format(result))
	{
		case INSTR_FORMAT_A:
			result.src_c = range_bits(code, BLOCK_REG_ADDR_WIDTH, 2 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH);
			result.dest  = range_bits(code, BLOCK_REG_ADDR_WIDTH, 3 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH);
			
			result.src_a_reg = range_bits(code, 1, BLOCK_INSTR_OP_TYPE_START + 0);
			result.src_b_reg = range_bits(code, 1, BLOCK_INSTR_OP_TYPE_START + 1);
			result.src_c_reg = range_bits(code, 1, BLOCK_INSTR_OP_TYPE_START + 2);
			result.dest_reg  = range_bits(code, 1, BLOCK_INSTR_OP_TYPE_START + 3);
			
			result.shift = range_bits(code, 4, BLOCK_INSTR_PMS_START);
			
			result.sat = range_bits(code, 1, BLOCK_INSTR_OP_TYPE_START + 4);
			break;
		
		case INSTR_FORMAT_B:
			result.dest  = range_bits(code, BLOCK_REG_ADDR_WIDTH, 2 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH);
			result.src_c = 0;
			
			result.src_a_reg = 0;
			result.src_b_reg = 0;
			result.src_c_reg = 0;
			result.dest_reg  = 0;
			
			result.shift = 0;
			
			result.sat = 0;
			
			result.res_addr = range_bits(code, BLOCK_RES_ADDR_WIDTH, 3 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH);
			break;
	}
	
	

    return result;
}


char *opcode_to_string(uint32_t opcode)
{
	switch (opcode)
	{
		case BLOCK_INSTR_NOP: return (char*)"BLOCK_INSTR_NOP";
		case BLOCK_INSTR_ADD: return (char*)"BLOCK_INSTR_ADD";
		case BLOCK_INSTR_SUB: return (char*)"BLOCK_INSTR_SUB";
		case BLOCK_INSTR_LSH: return (char*)"BLOCK_INSTR_LSH";
		case BLOCK_INSTR_RSH: return (char*)"BLOCK_INSTR_RSH";
		case BLOCK_INSTR_ARSH: return (char*)"BLOCK_INSTR_ARSH";
		case BLOCK_INSTR_MUL: return (char*)"BLOCK_INSTR_MUL";
		case BLOCK_INSTR_MAD: return (char*)"BLOCK_INSTR_MAD";
		case BLOCK_INSTR_ABS: return (char*)"BLOCK_INSTR_ABS";
		case BLOCK_INSTR_LUT: return (char*)"BLOCK_INSTR_LUT";
		case BLOCK_INSTR_ENVD: return (char*)"BLOCK_INSTR_ENVD";
		case BLOCK_INSTR_DELAY: return (char*)"BLOCK_INSTR_DELAY";
		case BLOCK_INSTR_SAVE: return (char*)"BLOCK_INSTR_SAVE";
		case BLOCK_INSTR_LOAD: return (char*)"BLOCK_INSTR_LOAD";
		case BLOCK_INSTR_MOV: return (char*)"BLOCK_INSTR_MOV";
		case BLOCK_INSTR_CLAMP: return (char*)"BLOCK_INSTR_CLAMP";
	}
	
	return NULL;
}

void print_instruction(block_instr instr)
{
	switch (instr_format(instr))
	{
		case INSTR_FORMAT_A:
			printf("Instruction = %s(%d, %d, %d, %d, %d, %d, %d, %d, %d, %d)",
						opcode_to_string(instr.opcode),
						instr.src_a,
						instr.src_b,
						instr.src_c,
						instr.dest,
						instr.src_a_reg,
						instr.src_b_reg,
						instr.src_c_reg,
						instr.dest_reg,
						instr.shift,
						instr.sat);
			break;
		
		case INSTR_FORMAT_B:
			printf("Instruction = %s(%d, %d, %d, 0x%04x)",
						opcode_to_string(instr.opcode),
							instr.src_a,
							instr.src_b,
							instr.dest,
							instr.res_addr);
			break;
	}
}

int rectify_block_instr(const resource_report *cxt, resource_report *local, block *blk, uint32_t *out)
{
	if (!cxt || !local || !blk || !out)
		return 1;
	
	local->block_usage 		= 0;
	local->mem_usage   		= 0;
	local->sram_delay_usage = 0;
	
	block_instr rectified = blk->instr;
	
	if (instr_format(blk->instr) == INSTR_FORMAT_B)
	{
		switch (blk->instr.opcode)
		{
			case BLOCK_INSTR_DELAY:
				rectified.res_addr += cxt->sram_delay_usage;
				local->sram_delay_usage = blk->instr.res_addr + 1;
				break;
			
			case BLOCK_INSTR_SAVE:
			case BLOCK_INSTR_LOAD:
				rectified.res_addr += cxt->mem_usage;
				local->mem_usage = blk->instr.res_addr + 1;
				break;
		}
	}
	
	*out = encode_block_instr_str(rectified);
	
	local->block_usage = 1;
	
	return 0;
}

int rectify_block_sequence(const resource_report *cxt, resource_report *report, block **blocks, int n, uint32_t *out)
{
	if (!cxt || !report || !blocks || !out)
		return 1;
	
	int ret_val;
	resource_report local;
	
	for (int i = 0; i < n; i++)
	{
		if (!blocks[i])
			continue;
		
		ret_val = rectify_block_instr(cxt, &local, blocks[i], &out[i]);
		
		report->block_usage += local.block_usage;
		
		report->mem_usage 			= (local.mem_usage > report->mem_usage) ? local.mem_usage : report->mem_usage;
		report->sram_delay_usage 	= (local.sram_delay_usage > report->sram_delay_usage) ? local.sram_delay_usage : report->sram_delay_usage;
	}
	
	return 0;
}

int16_t float_to_q_nminus1(float x, int shift)
{
    int n = 15 - shift;

    float scale = (float)(1 << n);

    float max =  (float)((1 << 15) - 1) / scale;
    float min = -(float)(1 << 15)       / scale;

    if (x > max) x = max;
    if (x < min) x = min;

    return (int16_t)lrintf(x * scale);
}

float eval_derived_quantity_expr(derived_quantity_expr *expr)
{	
	float x;
	
	if (!expr)
		return 0.0;
	
	float ret_val;
	
	switch (expr->call)
	{
		case DQE_CALL_CONST: ret_val = expr->value; break;
		case DQE_CALL_PARAM: ret_val = (expr->param) ? expr->param->value : 0; break;
		case DQE_CALL_ADD: ret_val = (eval_derived_quantity_expr(expr->sub_exprs[0]) + eval_derived_quantity_expr(expr->sub_exprs[1])); break;
		case DQE_CALL_SUB: ret_val = eval_derived_quantity_expr(expr->sub_exprs[0]) - eval_derived_quantity_expr(expr->sub_exprs[1]); break;
		case DQE_CALL_MUL: ret_val = eval_derived_quantity_expr(expr->sub_exprs[0]) * eval_derived_quantity_expr(expr->sub_exprs[1]); break;
		case DQE_CALL_DIV: ret_val = eval_derived_quantity_expr(expr->sub_exprs[0]) / eval_derived_quantity_expr(expr->sub_exprs[1]); break;
		case DQE_CALL_ABS: ret_val = fabs(eval_derived_quantity_expr(expr->sub_exprs[0])); break;
		case DQE_CALL_SQR: x = eval_derived_quantity_expr(expr->sub_exprs[0]); ret_val = x * x; break;
		case DQE_CALL_EXP: ret_val = exp(eval_derived_quantity_expr(expr->sub_exprs[0])); break;
		case DQE_CALL_LOG: ret_val = log(eval_derived_quantity_expr(expr->sub_exprs[0])); break;
		case DQE_CALL_POW: ret_val = pow(eval_derived_quantity_expr(expr->sub_exprs[0]), eval_derived_quantity_expr(expr->sub_exprs[1])); break;
		case DQE_CALL_SIN: ret_val = sin(eval_derived_quantity_expr(expr->sub_exprs[0])); break;
		case DQE_CALL_COS: ret_val = cos(eval_derived_quantity_expr(expr->sub_exprs[0])); break;
		case DQE_CALL_TAN: ret_val = tan(eval_derived_quantity_expr(expr->sub_exprs[0])); break;
	}
	
	return ret_val;
}

int block_register_write_bytes(int block_no, block_register reg, uint8_t *out, int max)
{
	if (!out)
		return 0;
	
	if (max < 5)
		return 0;
	
	if (N_BLOCKS > 255 && max < 6)
		return 0;
	
	float v = eval_derived_quantity_expr(reg.expr);
	
	int16_t s = float_to_q_nminus1(v, reg.format);
	
	int pos = 0;
	out[pos++] = COMMAND_WRITE_BLOCK_REG;
	
	if (N_BLOCKS > 255)
		out[pos++] = (block_no & 0xFF00) >> 8;
	
	out[pos++] = block_no & 0x00FF;
	
	out[pos++] = reg.reg;
	
	out[pos++] = (s & 0xFF00) >> 8;
	out[pos++] = (s & 0x00FF);
	
	return pos;
}

int effect_bytes(effect *eff, const resource_report *cxt, resource_report *report, uint8_t *out, int max)
{
	if (!eff || !cxt || !report || !out)
		return 0;
	
	uint32_t *instr_sequence = (uint32_t*)malloc(eff->n_blocks * sizeof(uint32_t));
	
	*report = empty_resource_report();
	
	rectify_block_sequence(cxt, report, eff->blocks, eff->n_blocks, instr_sequence);
	
	int pos = 0;
	int block_n;
	for (int i = 0; i < eff->n_blocks && pos + 6 < max; i++)
	{
		block_n = i + cxt->block_usage;
		
		out[pos++] = COMMAND_WRITE_BLOCK_INSTR;
		out[pos++] = block_n;
		out[pos++] = (instr_sequence[i] & 0xFF000000) >> 24;
		out[pos++] = (instr_sequence[i] & 0x00FF0000) >> 16;
		out[pos++] = (instr_sequence[i] & 0x0000FF00) >> 8;
		out[pos++] = (instr_sequence[i] & 0x000000FF) >> 0;
		
		for (int j = 0; j < N_BLOCKS_REGS; j++)
		{
			if (eff->blocks[i]->regs[j])
				pos += block_register_write_bytes(block_n, *eff->blocks[i]->regs[j], &out[pos], max - pos);
		}
	}
	
	return pos;
}

pipeline *new_pipeline()
{
	pipeline *result = (pipeline*)malloc(sizeof(pipeline));
	
	result->effects = (effect**)malloc(sizeof(effect*) * 64);
	result->n_effects = 0;
	result->effect_array_len = 64;
	
	return result;
}

int pipeline_add_effect(pipeline *pl, effect *eff)
{
	if (!pl || !eff)
		return 1;
	
	if (pl->n_effects < pl->effect_array_len)
	{
		pl->effects[pl->n_effects++] = eff;
	}
	else
	{
		return 2; // dont care at the moment
	}
	
	return 0;
}

effect *new_effect()
{
	effect *result = (effect*)malloc(sizeof(effect));
	
	result->blocks = (block**)malloc(sizeof(block*) * 32);
	result->block_array_len = 32;
	result->n_blocks = 0;
	
	result->params = (parameter**)malloc(sizeof(parameter*) * 32);
	result->param_array_len = 32;
	result->n_params = 0;
	
	return result;
}

int effect_add_block(effect *eff, block *blk)
{
	if (!eff || !blk)
		return 1;
	
	if (eff->n_blocks < eff->block_array_len)
	{
		eff->blocks[eff->n_blocks++] = blk;
	}
	else
	{
		return 2; // dont care at the moment
	}
	
	return 0;
}

int effect_add_param(effect *eff, parameter *param)
{
	if (!eff || !param)
		return 1;
	
	if (eff->n_params < eff->param_array_len)
	{
		eff->params[eff->n_params++] = param;
	}
	else
	{
		return 2; // dont care at the moment
	}
	
	return 0;
}

block *new_block()
{
	block *result = (block*)malloc(sizeof(block));
	
	for (int i = 0; i < N_BLOCKS_REGS; i++)
		result->regs[i] = NULL;
	
	return result;
}

block_instr block_instr_type_a_str(int opcode, int src_a, int src_b, int src_c, int dest, int a_reg, int b_reg, int c_reg, int dest_reg, int shift, int sat)
{
	return (block_instr){opcode, src_a, src_b, src_c, dest, a_reg, b_reg, c_reg, dest_reg, shift, sat};
}

block_instr block_instr_type_b_str(int opcode, int src_a, int src_b, int dest, int res_addr)
{
	return (block_instr){opcode, src_a, src_b, 0, dest, 0, 0, 0, 0, 0, 0, res_addr};
}

block *new_block_with_instr(block_instr instr)
{
	block *result = (block*)malloc(sizeof(block));
	
	for (int i = 0; i < N_BLOCKS_REGS; i++)
		result->regs[i] = NULL;
	
	result->instr = instr;
	
	return result;
}

parameter *new_parameter(char *name, float val)
{
	parameter *result = (parameter*)malloc(sizeof(parameter));
	
	result->value = val;
	result->name = name;
	
	return result;
}

block_register *new_block_register(int reg, int format, derived_quantity_expr *dqe)
{
	block_register *result = (block_register*)malloc(sizeof(block_register));
	
	result->reg = reg;
	result->format = format;
	result->expr = dqe;
	
	return result;
}

int block_add_register(block *blk, int i, block_register *p)
{
	if (!blk || !p || i < 0 || i > N_BLOCKS_REGS)
		return 1;

	p->reg = i;
	
	blk->regs[i] = p;
	return 0;
}

derived_quantity_expr *new_const_dqe(float v)
{
	derived_quantity_expr *dqe = (derived_quantity_expr*)malloc(sizeof(derived_quantity_expr));
	
	dqe->value = v;
	dqe->call = DQE_CALL_CONST;
	dqe->sub_exprs = NULL;
	
	return dqe;
}

derived_quantity_expr *new_param_dqe(parameter *param)
{
	derived_quantity_expr *dqe = (derived_quantity_expr*)malloc(sizeof(derived_quantity_expr));
	
	dqe->param = param;
	dqe->call = DQE_CALL_PARAM;
	dqe->sub_exprs = NULL;
	
	return dqe;
}

derived_quantity_expr *new_unary_call_dqe(int call, derived_quantity_expr *arg)
{
	derived_quantity_expr *dqe = (derived_quantity_expr*)malloc(sizeof(derived_quantity_expr));
	
	dqe->call = call;
	dqe->call = DQE_CALL_PARAM;
	dqe->sub_exprs = (derived_quantity_expr**)malloc(sizeof(derived_quantity_expr*));
	dqe->sub_exprs[0] = arg;
	
	return dqe;
}

derived_quantity_expr *new_dqe_from_expr(char *str, parameter **params, int n_params, char **next)
{
	if (!str)
		return NULL;
	
	derived_quantity_expr *dqe = (derived_quantity_expr*)malloc(sizeof(derived_quantity_expr));
	
	dqe->param = NULL;
	dqe->sub_exprs = NULL;
	
	int pos = 0;
	
	int bufpos = 0;
	char buf[256];
	
	int state = 0;
	char c;
	
	parameter *param;
	
	int arithmetic = 0;
	int unary_call = 0;
	int binary_call = 0;
	int type = 0;
	
	char *sub_next = NULL;
	
	while (1)
	{
		c = str[pos];
		
		switch (state)
		{
			case -1:
				free(dqe);
				return NULL;
				
			case 0:
				if (c == ' ')
					break;
				
				if (c == '(')
				{
					pos++;
					continue;
				}
				
				if (c == 0)
				{
					state = -1;
				}
				else if (c == '+')
				{
					dqe->call = DQE_CALL_ADD;
					binary_call = 1;
				}
				else if (c == '-')
				{
					dqe->call = DQE_CALL_SUB;
					binary_call = 1;
				}
				else if (c == '*')
				{
					dqe->call = DQE_CALL_MUL;
					binary_call = 1;
				}
				else if (c == '/')
				{
					dqe->call = DQE_CALL_DIV;
					binary_call = 1;
				}
				else if (('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z') || c == '_')
				{
					state = 1;
					buf[0] = c;
					bufpos = 1;
				}
				else if ('0' <= c && c <= '9')
				{
					state = 2;
					buf[0] = c;
					bufpos = 1;
				}
				else
				{
					state = -1;
				}
				
				break;
				
			case 1:
				if (('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z') || c == '_')
				{
					buf[bufpos++] = c;
				}
				else if (c == ' ' || c == ')' || c == '0')
				{
					buf[bufpos++] = 0;
					
					if (strcmp(buf, "abs") == 0)
					{
						dqe->call = DQE_CALL_ABS;
						unary_call = 1;
					}
					else if (strcmp(buf, "sqr") == 0)
					{
						dqe->call = DQE_CALL_SQR;
						unary_call = 1;
					}
					else if (strcmp(buf, "exp") == 0)
					{
						dqe->call = DQE_CALL_EXP;
						unary_call = 1;
					}
					else if (strcmp(buf, "log") == 0)
					{
						dqe->call = DQE_CALL_LOG;
						unary_call = 1;
					}
					else if (strcmp(buf, "sin") == 0)
					{
						dqe->call = DQE_CALL_SIN;
						unary_call = 1;
					}
					else if (strcmp(buf, "cos") == 0)
					{
						dqe->call = DQE_CALL_COS;
						unary_call = 1;
					}
					else if (strcmp(buf, "tan") == 0)
					{
						dqe->call = DQE_CALL_TAN;
						unary_call = 1;
					}
					else  if (strcmp(buf, "pow") == 0)
					{
						dqe->call = DQE_CALL_POW;
						binary_call = 1;
					}
					else if (params)
					{
						for (int i = 0; i < n_params; i++)
						{
							if (!params[i])
								continue;
							
							if (strcmp(params[i]->name, buf) == 0)
							{
								dqe->call = DQE_CALL_PARAM;
								dqe->param = params[i];
								if (next)
									*next = &str[pos+1];
								return dqe;
							}
						}
						
						state = -1;
					}
					else
					{
						state = -1;
					}
				}
				else
				{
					state = -1;
				}
				
				break;
			
			case 2:
				if (('0' <= c && c <= '9') || c == '.')
				{
					buf[bufpos++] = c;
				}
				else if (c == ' ' || c == ')' || c == ',' || c == 0)
				{
					buf[bufpos] = 0;
					
					dqe->call = DQE_CALL_CONST;
					dqe->value = strtof(buf, NULL);
					
					if (next)
						*next = &str[pos+1];
					
					return dqe;
				}
				else
				{
					state = -1;
				}
				
				break;
			
			default:
				state = -1;
		}
		
		if (unary_call)
		{
			dqe->sub_exprs = (derived_quantity_expr**)malloc(sizeof(derived_quantity_expr*) * 1);
			dqe->sub_exprs[0] = new_dqe_from_expr(&str[pos+1], params, n_params, next);
			
			return dqe;
		}
		
		if (binary_call)
		{
			dqe->sub_exprs = (derived_quantity_expr**)malloc(sizeof(derived_quantity_expr*) * 2);
			dqe->sub_exprs[0] = new_dqe_from_expr(&str[pos+1], params, n_params, &sub_next);
			dqe->sub_exprs[1] = new_dqe_from_expr(sub_next, params, n_params, next);
			
			return dqe;
		}
		
		pos++;
	}
	
	return dqe;
}

int effect_set_reg(effect *eff, int block_no, int reg, int format, char *expr)
{
	if (!eff)
		return 1;
	
	if (block_no < 0 || block_no > eff->n_blocks || reg < 0 || reg > N_BLOCKS_REGS)
		return 2;
	
	block_register *bp = new_block_register(reg, format, new_dqe_from_expr(expr, eff->params, eff->n_params, NULL));
	
	block_add_register(eff->blocks[block_no], reg, bp);
	
	return 0;
}

effect *create_amplifier_effect(float gain)
{
	effect *amp = new_effect();
	parameter *param = new_parameter("gain", gain);
	effect_add_param(amp, param);
	
	block *blk1 = new_block_with_instr(block_instr_type_a_str(BLOCK_INSTR_MUL, 0, 0, 0, 0, 0, 1, 0, 0, 4, 0));
	effect_add_block(amp, blk1);
	effect_set_reg(amp, 0, 0, 4, "pow 10 (/ gain 20)");
	
	return amp;
}

transfer_sequence new_transfer_sequence()
{
	transfer_sequence seq;
	
	seq.buf = (uint8_t*)malloc(sizeof(uint8_t) * 256);
	seq.len = 0;
	seq.buf_len = 256;
	
	return seq;
}

int tfseq_append(transfer_sequence *tfseq, uint8_t byte)
{
	if (!tfseq)
		return 1;
	
	if (tfseq->len >= tfseq->buf_len)
	{
		tfseq->buf_len = tfseq->buf_len * 2;
		tfseq->buf = (uint8_t*)realloc(tfseq->buf, tfseq->buf_len * sizeof(uint8_t));
	}
	
	tfseq->buf[tfseq->len++] = byte;
	
	return 0;
}

int read_out_transfer_sequence(transfer_sequence tfseq)
{
	uint8_t *bytes = tfseq.buf;
	int n = tfseq.len;
	
	printf("Reading out transfer sequence %p (length %d)\n", bytes, n);
	
	
	if (!bytes || n < 1)
		return 1;
	
	int i = 0;
	
	int state = 0;
	int ret_state;
	int skip = 0;
	
	uint8_t byte;
	
	int ctr;
	
	uint8_t reg_no;
	int16_t value;
	uint32_t instruction;
	
	block_instr instr_str;
	
	while (i < n)
	{
		byte = bytes[i];
		
		printf("\tByte %d: 0x%04x. ", i, byte);
		
		switch (state)
		{
			case 0: // expecting a command
				switch (byte)
				{
					
					case COMMAND_WRITE_BLOCK_INSTR:
						printf("Command WRITE_BLOCK_INSTR");
						state = 1;
						ret_state = 2;
						ctr = 0;
						instruction = 0;
						break;

					case COMMAND_WRITE_BLOCK_REG:
						printf("Command WRITE_BLOCK_REG");
						state = 1;
						ret_state = 3;
						value = 0;
						ctr = 0;
						break;

					case COMMAND_UPDATE_BLOCK_REG:
						printf("Command UPDATE_BLOCK_REG");
						break;

					case COMMAND_ALLOC_SRAM_DELAY:
						printf("Command ALLOC_SRAM_DELAY");
						break;

					case COMMAND_SWAP_PIPELINES:
						printf("Command SWAP_PIPELINES");
						break;

					case COMMAND_RESET_PIPELINE:
						printf("Command RESET_PIPELINE");
						break;

					case COMMAND_SET_INPUT_GAIN:
						printf("Command SET_INPUT_GAIN");
						break;

					case COMMAND_SET_OUTPUT_GAIN:
						printf("Command SET_OUTPUT_GAIN");
						break;

				}
				break;
			
			case 1: // expecting block number
				printf("Block number %d", byte);
				state = ret_state;
				break;
			
			case 2: // expecting instruction
				if (ctr == 3)
				{
					state = 0;
					
					instruction = (instruction << 8) | byte;
					
					instr_str = unpack_instr_code(instruction);
					
					print_instruction(instr_str);
				}
				else
				{
					instruction = (instruction << 8) | byte;
					ctr++;
				}
				break;
			
			case 3: // expecting register number then register value
				printf("Register %d", byte);
				state = 4;
				break;
			
			case 4:
				if (ctr == 1)
				{
					state = 0;
					
					value = (value << 8) | byte;
					printf("Value: 0b%016b", value);
				}
				else
				{
					value = (value << 8) | byte;
					ctr++;
				}
				break;
				
			default:
				printf("Unknown :(\n");
				return 1;
		}
		
		printf("\n");
		
		i++;
	}
	
	return 0;
}

int integrate_resource_report(resource_report *res, resource_report local)
{
	if (!res)
		return 1;
	
	res->block_usage += local.block_usage;
	res->sram_delay_usage += local.sram_delay_usage;
	res->mem_usage += local.mem_usage;
	
	return 0;
}

transfer_sequence pipeline_transfer_sequence(pipeline *pl)
{
	transfer_sequence seq;
	
	seq.buf = NULL;
	seq.buf_len = 0;
	seq.len = 0;
	
	if (!pl)
		return seq;
	
	seq = new_transfer_sequence();
	
	resource_report res = empty_resource_report();
	resource_report eff_usage;
	
	uint8_t buf[4096];
	int count;
	
	for (int i = 0; i < pl->n_effects; i++)
	{
		count = effect_bytes(pl->effects[i], &res, &eff_usage, buf, 4096);
		
		integrate_resource_report(&res, eff_usage);
		
		for (int j = 0; j < count; j++)
			tfseq_append(&seq, buf[j]);
	}
	
	return seq;
}

int send_transfer_sequence(transfer_sequence seq)
{
	if (!seq.buf)
		return 1;
	
	int i = 0;
	int ret_val;
	
	read_out_transfer_sequence(seq);
	
	printf("Sending transfer sequence (length %d)\n", seq.len);
	while (i < seq.len)
	{
		printf("\tByte %d = 0x%04x = 0b%08b\n", i, seq.buf[i], seq.buf[i]);
		do 
		{
			ret_val = spi_send(seq.buf[i]);
			
			if (ret_val == 1)
				tick();
			else if (ret_val == 2)
				return 2;
		} while (ret_val);
		
		i++;
	}
	
	return 0;
}
