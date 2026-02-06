#include <cstdio>
#include <cstdint>
#include <string.h>
#include <math.h>
#include "sim_ctrl.h"
#include "sim_io.h"

#define NO_ERROR 			0
#define ERR_NULL_PTR 		1
#define ERR_BAD_ARGS 		2
#define ERR_ALLOC_FAIL 		3
#define ERR_UNIMPLEMENTED 	4

#define m_alloc 	malloc
#define m_realloc 	realloc
#define m_strndup 	strndup
#define m_free 		free

char bin_buf[35];

char *binary_print_8(uint8_t x)
{
	sprintf(bin_buf, "0b%08b", x);
	return bin_buf;
}

char *binary_print_16(uint16_t x)
{
	sprintf(bin_buf, "0b%016b", x);
	return bin_buf;
}

char *binary_print_32(uint32_t x)
{
	sprintf(bin_buf, "0b%032b", x);
	return bin_buf;
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


int16_t float_to_q15(float x)
{
	if (x >= 0.999969482421875f) return  32767;
    if (x <= -1.0f)              return -32768;
    
    return (int16_t)lrintf(x * 32768.0f);
}

m_dsp_block_instr m_dsp_block_instr_nop()
{
	return m_dsp_block_instr_type_a_str(BLOCK_INSTR_NOP, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

m_dsp_block_instr m_dsp_block_instr_madd(int src_a, int src_a_reg, int src_b, int src_b_reg, int src_c, int src_c_reg, int dest, int shift)
{
	return m_dsp_block_instr_type_a_str(BLOCK_INSTR_MADD, src_a, src_a_reg, src_b, src_b_reg, src_c, src_c_reg, dest, shift, 0);
}

m_dsp_block_instr m_dsp_block_instr_madd_noshift(int src_a, int src_a_reg, int src_b, int src_b_reg, int src_c, int src_c_reg, int dest)
{
	return m_dsp_block_instr_type_a_str(BLOCK_INSTR_MADD, src_a, src_a_reg, src_b, src_b_reg, src_c, src_c_reg, dest, NO_SHIFT, 0);
}

m_dsp_block_instr m_dsp_block_instr_madd_unsat(int src_a, int src_a_reg, int src_b, int src_b_reg, int src_c, int src_c_reg, int dest, int shift)
{
	return m_dsp_block_instr_type_a_str(BLOCK_INSTR_MADD, src_a, src_a_reg, src_b, src_b_reg, src_c, src_c_reg, dest, shift, 1);
}

m_dsp_block_instr m_dsp_block_instr_madd_unsat_noshift(int src_a, int src_a_reg, int src_b, int src_b_reg, int src_c, int src_c_reg, int dest)
{
	return m_dsp_block_instr_type_a_str(BLOCK_INSTR_MADD, src_a, src_a_reg, src_b, src_b_reg, src_c, src_c_reg, dest, NO_SHIFT, 1);
}

m_dsp_block_instr m_dsp_block_instr_macz(int src_a, int src_a_reg, int src_b, int src_b_reg, int shift)
{
	return m_dsp_block_instr_type_a_str(BLOCK_INSTR_MACZ, src_a, src_a_reg, src_b, src_b_reg, 0, 0, 0, shift, 0);
}

m_dsp_block_instr m_dsp_block_instr_macz_noshift(int src_a, int src_a_reg, int src_b, int src_b_reg)
{
	return m_dsp_block_instr_type_a_str(BLOCK_INSTR_MACZ, src_a, src_a_reg, src_b, src_b_reg, 0, 0, 0, NO_SHIFT, 0);
}

m_dsp_block_instr m_dsp_block_instr_mac(int src_a, int src_a_reg, int src_b, int src_b_reg, int shift)
{
	return m_dsp_block_instr_type_a_str(BLOCK_INSTR_MAC, src_a, src_a_reg, src_b, src_b_reg, 0, 0, 0, shift, 0);
}
m_dsp_block_instr m_dsp_block_instr_mac_noshift(int src_a, int src_a_reg, int src_b, int src_b_reg)
{
	return m_dsp_block_instr_type_a_str(BLOCK_INSTR_MAC, src_a, src_a_reg, src_b, src_b_reg, 0, 0, 0, NO_SHIFT, 0);
}

m_dsp_block_instr m_dsp_block_instr_mov_acc(int dest)
{
	return m_dsp_block_instr_type_a_str(BLOCK_INSTR_MOV_ACC, 0, 0, 0, 0, 0, 0, dest, 0, 0);
}

m_dsp_block_instr m_dsp_block_instr_mov_uacc(int dest)
{
	return m_dsp_block_instr_type_a_str(BLOCK_INSTR_MOV_UACC, 0, 0, 0, 0, 0, 0, dest, 0, 0);
}

m_dsp_block_instr m_dsp_block_instr_mov_lacc(int dest)
{
	return m_dsp_block_instr_type_a_str(BLOCK_INSTR_MOV_LACC, 0, 0, 0, 0, 0, 0, dest, 0, 0);
}

m_dsp_block_instr m_dsp_block_instr_lsh(int src_a, int src_a_reg, int shift, int dest)
{
	return m_dsp_block_instr_type_a_str(BLOCK_INSTR_LSH, src_a, src_a_reg, shift, 0, 0, 0, dest, 0, 0);
}

m_dsp_block_instr m_dsp_block_instr_rsh(int src_a, int src_a_reg, int shift, int dest)
{
	return m_dsp_block_instr_type_a_str(BLOCK_INSTR_RSH, src_a, src_a_reg, shift, 0, 0, 0, dest, 0, 0);
}

m_dsp_block_instr m_dsp_block_instr_arsh(int src_a, int src_a_reg, int shift, int dest)
{
	return m_dsp_block_instr_type_a_str(BLOCK_INSTR_ARSH, src_a, src_a_reg, shift, 0, 0, 0, dest, 0, 0);
}

m_dsp_block_instr m_dsp_block_instr_abs(int src_a, int src_a_reg, int dest)
{
	return m_dsp_block_instr_type_a_str(BLOCK_INSTR_ABS, src_a, src_a_reg, 0, 0, 0, 0, dest, 0, 0);
}

m_dsp_block_instr m_dsp_block_instr_clamp(int src_a, int src_a_reg, int src_b, int src_b_reg, int src_c, int src_c_reg, int dest)
{
	return m_dsp_block_instr_type_a_str(BLOCK_INSTR_CLAMP, src_a, src_a_reg, src_b, src_b_reg, src_c, src_c_reg, dest, 0, 0);
}

m_dsp_block_instr m_dsp_block_instr_lut_read(int src_a, int src_a_reg, int lut, int dest)
{
	return m_dsp_block_instr_type_b_str(BLOCK_INSTR_LUT_READ, src_a, src_a_reg, 0, 0, dest, lut);
}

m_dsp_block_instr m_dsp_block_instr_delay_read(int buffer, int dest)
{
	return m_dsp_block_instr_type_b_str(BLOCK_INSTR_DELAY_READ, 0, 0, 0, 0, dest, buffer);
}

m_dsp_block_instr m_dsp_block_instr_delay_write(int src, int src_reg, int inc, int inc_reg, int buffer)
{
	return m_dsp_block_instr_type_b_str(BLOCK_INSTR_DELAY_WRITE, src, src_reg, inc, inc_reg, 0, buffer);
}

m_dsp_block_instr m_dsp_block_instr_mem_write(int src, int src_reg, int addr)
{
	return m_dsp_block_instr_type_b_str(BLOCK_INSTR_MEM_WRITE, src, src_reg, 0, 0, 0, addr);
}

m_dsp_block_instr m_dsp_block_instr_mem_read(int addr, int dest)
{
	return m_dsp_block_instr_type_b_str(BLOCK_INSTR_MEM_READ, 0, 0, 0, 0, dest, addr);
}


m_dsp_block *new_m_dsp_block()
{
	m_dsp_block *result = (m_dsp_block*)m_alloc(sizeof(m_dsp_block));
	
	for (int i = 0; i < N_BLOCKS_REGS; i++)
		result->reg_vals[i] = NULL;
	
	return result;
}

m_dsp_block *new_m_dsp_block_with_instr(m_dsp_block_instr instr)
{
	m_dsp_block *result = (m_dsp_block*)m_alloc(sizeof(m_dsp_block));
	
	for (int i = 0; i < N_BLOCKS_REGS; i++)
		result->reg_vals[i] = NULL;
	
	result->instr = instr;
	
	return result;
}

int m_dsp_block_add_register_val(m_dsp_block *blk, int i, m_dsp_register_val *p)
{
	if (!blk || !p)
		return ERR_NULL_PTR;

	if (i < 0 || i >= N_BLOCKS_REGS)
		return ERR_BAD_ARGS;

	p->reg = i;
	
	blk->reg_vals[i] = p;
	return NO_ERROR;
}

// Compute arity in the sense of, how many sub-dq's it uses.
// this is used to guard accesses to the array dq->sub_dqs.
// therefore, if in doubt, return 0.
// it should not return x if dq->val.sub_dq[x-1]
// is not a valid pointer to another dq
int m_derived_quantity_arity(m_derived_quantity *dq)
{
	if (!dq) return NO_ERROR;
	
	// if the type is nonsense, return 0
	if (dq->type < 0 || dq->type > M_DERIVED_QUANTITY_TYPE_MAX_VAL)
		return NO_ERROR;
	
	if (dq->type == M_DERIVED_QUANTITY_CONST_FLT || dq->type == M_DERIVED_QUANTITY_CONST_INT || dq->type == M_DERIVED_QUANTITY_REFERENCE)
		return NO_ERROR;
	
	// arity is at least 1 if we reach this point. there are more arity 1 types than arity 2, but also arity 2 will
	// be more common, bc arithmetic. and none of arity 3. therefore, check the arity 2 case, then return 1 otherwise
	if (dq->type == M_DERIVED_QUANTITY_FCALL_ADD
	 || dq->type == M_DERIVED_QUANTITY_FCALL_SUB
	 || dq->type == M_DERIVED_QUANTITY_FCALL_MUL
	 || dq->type == M_DERIVED_QUANTITY_FCALL_DIV
	 || dq->type == M_DERIVED_QUANTITY_FCALL_POW)
		return 2;
	
	return 1;
}

m_parameter_pll *m_parameter_pll_append(m_parameter_pll *pll, m_parameter *param)
{
	m_parameter_pll *nl = malloc(sizeof(m_parameter_pll));
	nl->data = param;
	nl->next = NULL;
	
	if (!pll)
		return nl;
	
	m_parameter_pll *current = pll;
	
	while (current->next)
		current = current->next;
	
	current->next = nl;
	
	return pll;
}

m_parameter *new_m_parameter_wni(const char *name, const char *name_internal, float value, float min, float max)
{
	m_parameter *param = malloc(sizeof(m_parameter));
	
	param->name_internal = name_internal;
	param->value = value;
	
	return param;
}

#define DQ_MAX_RECURSION_DEPTH 256

static float m_derived_quantity_compute_rec(m_derived_quantity *dq, m_parameter_pll *params, int depth)
{
	if (!dq) return 0.0;
	
	if (dq->type == M_DERIVED_QUANTITY_REFERENCE)
	{
		if (!dq->val.ref_name || !params)
			return 0.0;
		
		m_parameter_pll *current;
		m_parameter *param;
		
		int cmplen = strlen(dq->val.ref_name) + 1;
		
		current = params;
		
		while (current)
		{
			param = current->data;
			if (param && param->name_internal)
			{
				if (strncmp(dq->val.ref_name, param->name_internal, cmplen) == 0)
					return param->value;
			}
			
			current = current->next;
		}
		
		return 0.0;
	}
	
	if (dq->type == M_DERIVED_QUANTITY_CONST_FLT)
	{
		return dq->val.val_float;
	}
	else if (dq->type == M_DERIVED_QUANTITY_CONST_INT)
	{
		return (float)dq->val.val_int;
	}
	
	if (depth > DQ_MAX_RECURSION_DEPTH || !dq->val.sub_dqs)
		return 0.0;
	
	float x = 0.0;
	float ret_val = 0.0;
	
	switch (dq->type)
	{
		case M_DERIVED_QUANTITY_FCALL_ADD:
			ret_val = (m_derived_quantity_compute_rec(dq->val.sub_dqs[0], params, depth + 1) + m_derived_quantity_compute_rec(dq->val.sub_dqs[1], params, depth + 1));
			break;

		case M_DERIVED_QUANTITY_FCALL_SUB:
			ret_val = m_derived_quantity_compute_rec(dq->val.sub_dqs[0], params, depth + 1) - m_derived_quantity_compute_rec(dq->val.sub_dqs[1], params, depth + 1);
			break;

		case M_DERIVED_QUANTITY_FCALL_MUL:
			ret_val = m_derived_quantity_compute_rec(dq->val.sub_dqs[0], params, depth + 1) * m_derived_quantity_compute_rec(dq->val.sub_dqs[1], params, depth + 1);
			break;

		case M_DERIVED_QUANTITY_FCALL_DIV:
			x = m_derived_quantity_compute_rec(dq->val.sub_dqs[1], params, depth + 1);
			
			if (fabsf(x) < 1e-20)
				return 0.0; // avoid division by zero by just returning 0 lol. idk. what else to do?
			
			ret_val = m_derived_quantity_compute_rec(dq->val.sub_dqs[0], params, depth + 1) / x;
			break;

		case M_DERIVED_QUANTITY_FCALL_ABS:
			ret_val = fabs(m_derived_quantity_compute_rec(dq->val.sub_dqs[0], params, depth + 1));
			break;

		case M_DERIVED_QUANTITY_FCALL_SQR: x = m_derived_quantity_compute_rec(dq->val.sub_dqs[0], params, depth + 1); ret_val = x * x;
			break;

		case M_DERIVED_QUANTITY_FCALL_EXP:
			ret_val = exp(m_derived_quantity_compute_rec(dq->val.sub_dqs[0], params, depth + 1));
			break;

		case M_DERIVED_QUANTITY_FCALL_LOG:
			ret_val = log(m_derived_quantity_compute_rec(dq->val.sub_dqs[0], params, depth + 1));
			break;

		case M_DERIVED_QUANTITY_FCALL_POW:
			ret_val = pow(m_derived_quantity_compute_rec(dq->val.sub_dqs[0], params, depth + 1),
						  m_derived_quantity_compute_rec(dq->val.sub_dqs[1], params, depth + 1));
			break;
		case M_DERIVED_QUANTITY_FCALL_SIN:
			ret_val = sin(m_derived_quantity_compute_rec(dq->val.sub_dqs[0], params, depth + 1));
			break;

		case M_DERIVED_QUANTITY_FCALL_COS:
			ret_val = cos(m_derived_quantity_compute_rec(dq->val.sub_dqs[0], params, depth + 1));
			break;

		case M_DERIVED_QUANTITY_FCALL_TAN:
			ret_val = tan(m_derived_quantity_compute_rec(dq->val.sub_dqs[0], params, depth + 1));
			break;
	}
	
	return ret_val;
}

float m_derived_quantity_compute(m_derived_quantity *dq, m_parameter_pll *params)
{
	return m_derived_quantity_compute_rec(dq, params, 0);
}

int m_derived_quantity_references_param_rec(m_derived_quantity *dq, m_parameter *param, int depth)
{
	if (!dq || !param)
		return 0;
	
	if (!param->name_internal)
		return 0;
	
	int arity = m_derived_quantity_arity(dq);
	
	if (arity == 0)
	{
		if (dq->type != M_DERIVED_QUANTITY_REFERENCE)
			return 0;
	
		if (!dq->val.ref_name)
				return 0;
			
		return (strncmp(dq->val.ref_name, param->name_internal, strlen(dq->val.ref_name) + 1) == 0);
	}
	
	if (depth > DQ_MAX_RECURSION_DEPTH)
		return 0;
	
	for (int i = 0; i < arity; i++)
	{
		if (m_derived_quantity_references_param_rec(dq->val.sub_dqs[i], param, depth + 1))
			return 1;
	}
	
	return 0;
}

int m_derived_quantity_references_param(m_derived_quantity *dq, m_parameter *param)
{
	return m_derived_quantity_references_param_rec(dq, param, 0);
}

int m_dsp_block_uses_param(m_dsp_block *blk, m_parameter *param)
{
	if (!blk || !param)
		return NO_ERROR;
	
	for (int i = 0; i < M_DSP_BLOCK_N_REGS; i++)
	{
		if (blk->reg_vals[i])
		{
			if (m_derived_quantity_references_param(blk->reg_vals[i]->dq, param))
				return ERR_NULL_PTR;
		}
	}
	
	return NO_ERROR;
}

int m_fpga_transfer_batch_append_block_register_write(m_fpga_transfer_batch *batch, int block_no, m_dsp_register_val *reg_val, m_parameter_pll *params)
{
	if (!batch)
		return ERR_NULL_PTR;
	
	if (!reg_val)
		return ERR_NULL_PTR;
	
	if (!reg_val->dq)
		return ERR_BAD_ARGS;
	
	int ret_val;
	
	if ((ret_val = m_fpga_batch_append(batch, COMMAND_WRITE_BLOCK_REG)) != NO_ERROR) return ret_val;
	
	if (N_BLOCKS > 255)
	{
		if ((ret_val = m_fpga_batch_append(batch, (block_no & 0xFF00) >> 8)) != NO_ERROR) return ret_val;
	}
	
	if ((ret_val = m_fpga_batch_append(batch, block_no & 0x00FF)) != NO_ERROR) return ret_val;
	if ((ret_val = m_fpga_batch_append(batch, reg_val->reg)) 	  != NO_ERROR) return ret_val;
	
	int16_t s = 0;
	
	if (reg_val->dq)
	{
		float v = m_derived_quantity_compute(reg_val->dq, params);
		
		if (reg_val->format == DSP_REG_FORMAT_LITERAL)
		{
			s = (int16_t)v;
		}
		else
		{
			s = float_to_q_nminus1(v, reg_val->format);
		}
	}
	
	if ((ret_val = m_fpga_batch_append(batch, (s & 0xFF00) >> 8)) != NO_ERROR) return ret_val;
	if ((ret_val = m_fpga_batch_append(batch, (s & 0x00FF))) 	  != NO_ERROR) return ret_val;
	
	return NO_ERROR;
}

int m_fpga_transfer_batch_append_block_register_update(m_fpga_transfer_batch *batch, int block_no, m_dsp_register_val *reg_val, m_parameter_pll *params)
{
	if (!batch)
		return ERR_NULL_PTR;
	
	if (!reg_val)
		return ERR_NULL_PTR;
	
	if (!reg_val->dq)
		return ERR_BAD_ARGS;
	
	int ret_val;
	
	if ((ret_val = m_fpga_batch_append(batch, COMMAND_UPDATE_BLOCK_REG)) != NO_ERROR) return ret_val;
	
	if (N_BLOCKS > 255)
	{
		if ((ret_val = m_fpga_batch_append(batch, (block_no & 0xFF00) >> 8)) != NO_ERROR) return ret_val;
	}
	
	if ((ret_val = m_fpga_batch_append(batch, block_no & 0x00FF)) != NO_ERROR) return ret_val;
	if ((ret_val = m_fpga_batch_append(batch, reg_val->reg)) 	  != NO_ERROR) return ret_val;
	
	int16_t s = 0;
	
	if (reg_val->dq)
	{
		float v = m_derived_quantity_compute(reg_val->dq, params);
		
		if (reg_val->format == DSP_REG_FORMAT_LITERAL)
		{
			s = (int16_t)v;
		}
		else
		{
			s = float_to_q_nminus1(v, reg_val->format);
		}
	}
	
	if ((ret_val = m_fpga_batch_append(batch, (s & 0xFF00) >> 8)) != NO_ERROR) return ret_val;
	if ((ret_val = m_fpga_batch_append(batch, (s & 0x00FF))) 	  != NO_ERROR) return ret_val;
	
	return NO_ERROR;
}

int m_fpga_transfer_batch_append_block_register_writes(m_fpga_transfer_batch *batch, m_dsp_block *blk, int blk_index, m_parameter_pll *params)
{
	if (!batch || !blk)
		return ERR_NULL_PTR;
	
	int ret_val;
	
	for (int j = 0; j < N_BLOCKS_REGS; j++)
	{
		if (blk->reg_vals[j])
		{
			if ((ret_val = m_fpga_transfer_batch_append_block_register_write(batch, blk_index, blk->reg_vals[j], params)) != NO_ERROR)
				return ret_val;
		}
	}
	
	return NO_ERROR;
}

int m_fpga_transfer_batch_append_block_register_updates(m_fpga_transfer_batch *batch, m_dsp_block *blk, int blk_index, m_parameter_pll *params)
{
	if (!batch || !blk)
		return ERR_NULL_PTR;
	
	int ret_val;
	
	for (int j = 0; j < N_BLOCKS_REGS; j++)
	{
		if (blk->reg_vals[j])
		{
			if ((ret_val = m_fpga_transfer_batch_append_block_register_update(batch, blk_index, blk->reg_vals[j], params)) != NO_ERROR)
				return ret_val;
		}
	}
	
	return NO_ERROR;
}

int m_dsp_block_instr_encode_resource_aware(const m_fpga_resource_report *cxt, m_fpga_resource_report *local, m_dsp_block *blk, uint32_t *out)
{
	if (!cxt || !local || !blk || !out)
		return ERR_NULL_PTR;
	
	local->blocks = 0;
	local->memory = 0;
	local->sdelay = 0;
	local->ddelay = 0;
	local->luts   = 0;
	
	m_dsp_block_instr rectified = blk->instr;
	
	if (m_dsp_block_instr_format(blk->instr) == INSTR_FORMAT_B)
	{
		switch (blk->instr.opcode)
		{
			case BLOCK_INSTR_DELAY_READ:
			case BLOCK_INSTR_DELAY_WRITE:
				rectified.res_addr += cxt->ddelay;
				local->ddelay = blk->instr.res_addr + 1;
				break;
			
			case BLOCK_INSTR_MEM_READ:
			case BLOCK_INSTR_MEM_WRITE:
				rectified.res_addr += cxt->memory;
				local->memory = blk->instr.res_addr + 1;
				break;
			
			case BLOCK_INSTR_LUT_READ:
				if (rectified.res_addr >= STOCK_LUTS)
				{
					rectified.res_addr += cxt->luts;
					local->luts = blk->instr.res_addr + 1;
				}
				break;
		}
	}
	
	*out = m_encode_dsp_block_instr(rectified);
	
	local->blocks = 1;
	
	return NO_ERROR;
}

int m_dsp_blocks_encode_resource_aware(const m_fpga_resource_report *cxt, m_fpga_resource_report *report, m_dsp_block **blocks, int n, uint32_t *out)
{
	if (!cxt || !report || !blocks || !out)
		return ERR_NULL_PTR;
	
	int ret_val;
	m_fpga_resource_report local;
	
	for (int i = 0; i < n; i++)
	{
		if (!blocks[i])
			continue;
		
		ret_val = m_dsp_block_instr_encode_resource_aware(cxt, &local, blocks[i], &out[i]);
		
		report->blocks += local.blocks;
		
		report->memory 	= (local.memory > report->memory) ? local.memory : report->memory;
		report->sdelay 	= (local.sdelay > report->sdelay) ? local.sdelay : report->sdelay;
		report->ddelay 	= (local.ddelay > report->ddelay) ? local.ddelay : report->ddelay;
		report->luts 	= (local.luts   >   report->luts) ? local.luts   : report->luts;
	}
	
	return NO_ERROR;
}

int m_fpga_transfer_batch_append_effect_register_writes(
		m_fpga_transfer_batch *batch,
		m_effect_desc *eff, int blocks_start,
		m_parameter_pll *params
	)
{
	if (!eff || !batch)
		return ERR_NULL_PTR;
	
	if (!eff->blocks)
		return ERR_BAD_ARGS;
	
	for (int i = 0; i < eff->n_blocks; i++)
	{
		if (eff->blocks[i])
			m_fpga_transfer_batch_append_block_register_writes(batch, eff->blocks[i], blocks_start + i, params);
	}
	
	return NO_ERROR;
}

int m_fpga_transfer_batch_append_effect_register_updates(
		m_fpga_transfer_batch *batch,
		m_effect_desc *eff, int blocks_start,
		m_parameter_pll *params
	)
{
	if (!eff || !batch)
		return ERR_NULL_PTR;
	
	if (!eff->blocks)
		return ERR_BAD_ARGS;
	
	for (int i = 0; i < eff->n_blocks; i++)
	{
		if (eff->blocks[i])
			m_fpga_transfer_batch_append_block_register_updates(batch, eff->blocks[i], blocks_start + i, params);
	}
	
	return NO_ERROR;
}

int m_fpga_transfer_batch_append_resource_request(m_fpga_transfer_batch *batch, m_fpga_resource_report *cxt, m_fpga_resource_req *req)
{
	if (!batch)
		return ERR_NULL_PTR;
	
	if (!cxt)
		return ERR_NULL_PTR;
	
	if (!req)
		return ERR_BAD_ARGS;
	
	switch (req->type)
	{
		case M_FPGA_RESOURCE_DDELAY:
			m_fpga_batch_append(batch, COMMAND_ALLOC_SRAM_DELAY);
			m_fpga_batch_append_16(batch, (uint16_t)req->data);
			break;
		
		default:
			return ERR_BAD_ARGS;
	}
	
	return NO_ERROR;
}

int m_fpga_transfer_batch_append_effect(
		m_effect_desc *eff,
		const m_fpga_resource_report *cxt,
		m_fpga_resource_report *report,
		m_parameter_pll *params,
		m_fpga_transfer_batch *batch
	)
{
	if (!eff || !cxt || !report || !batch)
		return ERR_NULL_PTR;
	
	if (eff->n_blocks == 0)
		return NO_ERROR;
	
	if (!eff->blocks || eff->n_blocks > N_BLOCKS)
		return ERR_BAD_ARGS;
	
	uint32_t instr_seq[eff->n_blocks];
	
	for (int i = 0; i < eff->n_res_reqs; i++)
	{
		m_fpga_transfer_batch_append_resource_request(batch, cxt, eff->res_reqs[i]);
	}
	
	*report = m_empty_fpga_resource_report();
	
	m_dsp_blocks_encode_resource_aware(cxt, report, eff->blocks, eff->n_blocks, instr_seq);
	
	int ret_val;
	
	int block_n;
	for (int i = 0; i < eff->n_blocks; i++)
	{
		block_n = i + cxt->blocks;
		
		if ((ret_val = m_fpga_batch_append(batch, COMMAND_WRITE_BLOCK_INSTR)) != NO_ERROR) return ret_val;
		if (N_BLOCKS > 255)
		{
			if ((ret_val = m_fpga_batch_append(batch, (block_n & 0xFF00) >> 8)) != NO_ERROR) return ret_val;
		}
		if ((ret_val = m_fpga_batch_append(batch, block_n & 0x00FF)) != NO_ERROR) return ret_val;
		
		if ((ret_val = m_fpga_batch_append(batch, (instr_seq[i] & 0xFF000000) >> 24)) != NO_ERROR) return ret_val;
		if ((ret_val = m_fpga_batch_append(batch, (instr_seq[i] & 0x00FF0000) >> 16)) != NO_ERROR) return ret_val;
		if ((ret_val = m_fpga_batch_append(batch, (instr_seq[i] & 0x0000FF00) >> 8 )) != NO_ERROR) return ret_val;
		if ((ret_val = m_fpga_batch_append(batch, (instr_seq[i] & 0x000000FF) >> 0 )) != NO_ERROR) return ret_val;
	}
	
	if ((ret_val = m_fpga_transfer_batch_append_effect_register_writes(batch, eff, cxt->blocks, params)) != NO_ERROR) return ret_val;
	
	return NO_ERROR;
}

m_derived_quantity m_derived_quantity_const_float(float v)
{
	m_derived_quantity result;
	result.type = M_DERIVED_QUANTITY_CONST_FLT;
	result.val.val_float = v;
	return result;
}

m_derived_quantity m_derived_quantity_const_int(int v)
{
	m_derived_quantity result;
	result.type = M_DERIVED_QUANTITY_CONST_INT;
	result.val.val_int = v;
	return result;
}

m_derived_quantity *new_m_derived_quantity_const_float(float v)
{
	m_derived_quantity *result = m_alloc(sizeof(m_derived_quantity));
	
	if (!result)
		return NULL;
	
	*result = m_derived_quantity_const_float(v);
	
	return result;
}


m_derived_quantity *new_m_derived_quantity_const_int(int v)
{
	m_derived_quantity *result = m_alloc(sizeof(m_derived_quantity));
	
	if (!result)
		return NULL;
	
	*result = m_derived_quantity_const_int(v);
	
	return result;
}

m_derived_quantity *new_m_derived_quantity_from_string_rec(char *str, char **next)
{
	if (!str)
		return NULL;
	
	m_derived_quantity *dq = (m_derived_quantity*)m_alloc(sizeof(m_derived_quantity));
	
	if (!dq)
		return NULL;
	
	dq->type = M_DERIVED_QUANTITY_CONST_FLT;
	dq->val.val_float = 0.0;
	
	int pos = 0;
	
	int bufpos = 0;
	int len = strlen(str);
	
	char buf[len];
	
	int state = 0;
	char c;
	
	int arithmetic = 0;
	int unary_call = 0;
	int binary_call = 0;
	int type = 0;
	
	char *sub_next = NULL;
	
	while (pos < len + 1)
	{
		c = str[pos];
		
		switch (state)
		{
			case -1:
				m_free(dq);
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
					dq->type = M_DERIVED_QUANTITY_FCALL_ADD;
					binary_call = 1;
				}
				else if (c == '-')
				{
					dq->type = M_DERIVED_QUANTITY_FCALL_SUB;
					binary_call = 1;
				}
				else if (c == '*')
				{
					dq->type = M_DERIVED_QUANTITY_FCALL_MUL;
					binary_call = 1;
				}
				else if (c == '/')
				{
					dq->type = M_DERIVED_QUANTITY_FCALL_DIV;
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
				if (('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z') || ('0' <= c && c <= '9') || c == '_')
				{
					buf[bufpos++] = c;
				}
				else if (c == ' ' || c == ')' || c == 0)
				{
					buf[bufpos++] = 0;
					
					if (strcmp(buf, "abs") == 0)
					{
						dq->type = M_DERIVED_QUANTITY_FCALL_ABS;
						unary_call = 1;
					}
					else if (strcmp(buf, "sqr") == 0)
					{
						dq->type = M_DERIVED_QUANTITY_FCALL_SQR;
						unary_call = 1;
					}
					else if (strcmp(buf, "exp") == 0)
					{
						dq->type = M_DERIVED_QUANTITY_FCALL_EXP;
						unary_call = 1;
					}
					else if (strcmp(buf, "log") == 0)
					{
						dq->type = M_DERIVED_QUANTITY_FCALL_LOG;
						unary_call = 1;
					}
					else if (strcmp(buf, "sin") == 0)
					{
						dq->type = M_DERIVED_QUANTITY_FCALL_SIN;
						unary_call = 1;
					}
					else if (strcmp(buf, "cos") == 0)
					{
						dq->type = M_DERIVED_QUANTITY_FCALL_COS;
						unary_call = 1;
					}
					else if (strcmp(buf, "tan") == 0)
					{
						dq->type = M_DERIVED_QUANTITY_FCALL_TAN;
						unary_call = 1;
					}
					else  if (strcmp(buf, "pow") == 0)
					{
						dq->type = M_DERIVED_QUANTITY_FCALL_POW;
						binary_call = 1;
					}
					else
					{
						dq->type = M_DERIVED_QUANTITY_REFERENCE;
						dq->val.ref_name = m_strndup(buf, len);
						if (next) *next = &str[pos+1];
						return dq;
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
					
					dq->type = M_DERIVED_QUANTITY_CONST_FLT;
					dq->val.val_float = strtof(buf, NULL);
					
					if (next)
						*next = &str[pos+1];
					
					return dq;
				}
				else
				{
					state = -1;
				}
				
				break;
			
			default:
				state = -1;
				break;
		}
		
		if (unary_call)
		{
			dq->val.sub_dqs = (m_derived_quantity**)m_alloc(sizeof(m_derived_quantity*) * 1);
			if (!dq->val.sub_dqs) // alloc fail; delete everything and bail
			{
				m_free(dq);
				return NULL;
			}
			dq->val.sub_dqs[0] = new_m_derived_quantity_from_string_rec(&str[pos+1], next);
			
			return dq;
		}
		
		if (binary_call)
		{
			dq->val.sub_dqs = (m_derived_quantity**)m_alloc(sizeof(m_derived_quantity*) * 2);
			if (!dq->val.sub_dqs) // alloc fail; delete everything and bail
			{
				m_free(dq);
				return NULL;
			}
			dq->val.sub_dqs[0] = new_m_derived_quantity_from_string_rec(&str[pos+1], &sub_next);
			dq->val.sub_dqs[1] = new_m_derived_quantity_from_string_rec(sub_next, next);
			
			return dq;
		}
		
		pos++;
	}
	
	return dq;
}

m_derived_quantity *new_m_derived_quantity_from_string(char *str)
{
	return new_m_derived_quantity_from_string_rec(str, NULL);
}

m_fpga_resource_req *new_fpga_resource_req(int type, int data)
{
	m_fpga_resource_req *req = malloc(sizeof(m_fpga_resource_req));
	
	if (!req)
		return NULL;
	
	req->type = type;
	req->data = data;
	
	return req;
}

m_effect_desc *new_m_effect_desc(const char *name)
{
	m_effect_desc *result = (m_effect_desc*)m_alloc(sizeof(m_effect_desc));
	
	if (!result)
		return NULL;
	
	result->blocks = (m_dsp_block**)m_alloc(sizeof(m_dsp_block*) * 32);
	
	if (!result->blocks)
	{
		m_free(result);
		return NULL;
	}
	
	result->block_array_len = 32;
	result->n_blocks = 0;
	
	for (int i = 0; i < result->block_array_len; i++)
		result->blocks[i] = NULL;
	
	result->params = (m_parameter**)m_alloc(sizeof(m_parameter*) * 32);
	
	if (!result->params)
	{
		m_free(result->blocks);
		m_free(result);
		return NULL;
	}
	
	result->param_array_len = 32;
	result->n_params = 0;
	
	for (int i = 0; i < result->param_array_len; i++)
		result->params[i] = NULL;
	
	result->res_reqs = (m_fpga_resource_req**)m_alloc(sizeof(m_fpga_resource_req*) * 8);
	
	if (!result->res_reqs)
	{
		m_free(result->blocks);
		m_free(result->params);
		m_free(result);
		return NULL;
	}
	
	result->res_req_array_len = 8;
	result->n_res_reqs = 0;
	
	for (int i = 0; i < result->res_req_array_len; i++)
		result->res_reqs[i] = NULL;
	
	result->name = name;
	
	return result;
}

int m_effect_desc_add_block(m_effect_desc *eff, m_dsp_block *blk)
{
	if (!eff || !blk)
		return ERR_NULL_PTR;
	
	if (eff->n_blocks >= eff->block_array_len)
	{
		m_dsp_block **np = m_realloc(eff->blocks, eff->block_array_len * 2 * sizeof(m_dsp_block*));
		
		if (!np)
			return ERR_ALLOC_FAIL;
		
		eff->blocks = np;
		eff->block_array_len *= 2;
	}
	
	eff->blocks[eff->n_blocks++] = blk;
	
	return NO_ERROR;
}

int m_effect_desc_add_param(m_effect_desc *eff, m_parameter *param)
{
	if (!eff || !param)
		return ERR_NULL_PTR;
	
	if (eff->n_params >= eff->param_array_len)
	{
		m_parameter **np = m_realloc(eff->params, eff->param_array_len * 2 * sizeof(m_parameter*));
		
		if (!np)
			return ERR_ALLOC_FAIL;
		
		eff->params = np;
		eff->param_array_len *= 2;
	}
	
	eff->params[eff->n_params++] = param;
	
	return NO_ERROR;
}

int m_effect_desc_add_resource_request(m_effect_desc *eff, m_fpga_resource_req *req)
{
	if (!eff || !req)
		return ERR_NULL_PTR;
	
	if (eff->n_res_reqs >= eff->res_req_array_len)
	{
		m_fpga_resource_req **np = m_realloc(eff->res_reqs, eff->res_req_array_len * 2 * sizeof(m_fpga_resource_req*));
		
		if (!np)
			return ERR_ALLOC_FAIL;
		
		eff->res_reqs = np;
		eff->res_req_array_len *= 2;
	}
	
	eff->res_reqs[eff->n_res_reqs++] = req;
	
	return NO_ERROR;
}

m_dsp_register_val *new_m_dsp_register_val(int reg, int format, m_derived_quantity *dq)
{
	if (!dq)
		return NULL;
	
	m_dsp_register_val *result = (m_dsp_register_val*)m_alloc(sizeof(m_dsp_register_val));
	
	if (!result)
		return NULL;
	
	result->reg = reg;
	result->format = format;
	result->dq = dq;
	
	return result;
}

m_dsp_register_val *new_m_dsp_register_val_literal(int reg, int16_t literal_value)
{
	m_dsp_register_val *result = (m_dsp_register_val*)m_alloc(sizeof(m_dsp_register_val));
	
	result->reg = reg;
	result->format = DSP_REG_FORMAT_LITERAL;
	
	result->dq = new_m_derived_quantity_const_int(literal_value);
	
	return result;
}

int m_effect_desc_add_register_val_literal(m_effect_desc *eff, int block_no, int reg, uint16_t val)
{
	if (!eff)
		return ERR_NULL_PTR;
	
	if (block_no < 0 || block_no > eff->n_blocks || reg < 0 || reg > N_BLOCKS_REGS)
		return ERR_BAD_ARGS;
	
	m_dsp_register_val *bp = new_m_dsp_register_val_literal(reg, val);
	
	return m_dsp_block_add_register_val(eff->blocks[block_no], reg, bp);
}

int m_effect_desc_add_register_val(m_effect_desc *eff, int block_no, int reg, int format, char *expr)
{
	if (!eff)
		return ERR_NULL_PTR;
	
	if (block_no < 0 || block_no > eff->n_blocks || reg < 0 || reg > N_BLOCKS_REGS)
		return ERR_BAD_ARGS;
	
	m_dsp_register_val *bp = new_m_dsp_register_val(reg, format, new_m_derived_quantity_from_string(expr));
	
	return m_dsp_block_add_register_val(eff->blocks[block_no], reg, bp);
}

#define IBM(x) ((1u << (x)) - 1)
#define range_bits(x, n, start) (((x) >> (start)) & IBM(n))
#define place_bits(x, y, val) ((IBM((x)-(y)+1) & ((uint32_t)val)) << y) 

int m_fpga_block_opcode_format(int opcode)
{
	return (!!(opcode & (1 << 5))) ? INSTR_FORMAT_B : INSTR_FORMAT_A;
}

int m_dsp_block_instr_format(m_dsp_block_instr instr)
{
	return m_fpga_block_opcode_format(instr.opcode);
}

uint32_t m_encode_dsp_block_instr(m_dsp_block_instr instr)
{
	if (m_fpga_block_opcode_format(instr.opcode) == INSTR_FORMAT_B)
	{
		return place_bits(5,  0, instr.opcode) | (1 << 5)
		 | place_bits( 9,  6, instr.src_a) | ((!!instr.src_a_reg) << 10)
		 | place_bits(14, 11, instr.src_b) | ((!!instr.src_b_reg) << 15)
		 | place_bits(19, 16, instr.dest)
		 | place_bits(27, 20, instr.res_addr);
	}
	else
	{
		return place_bits( 5,  0, instr.opcode)
		 | place_bits( 9,  6, instr.src_a) | ((!!instr.src_a_reg) << 10)
		 | place_bits(14, 11, instr.src_b) | ((!!instr.src_b_reg) << 15)
		 | place_bits(19, 16, instr.src_c) | ((!!instr.src_c_reg) << 20)
		 | place_bits(24, 21, instr.dest)
		 | place_bits(29, 25, instr.shift) | ((!!instr.sat) << 30) | ((!!instr.no_shift) << 31);
	}
}

m_dsp_block_instr m_dsp_block_instr_type_a_str(int opcode, int src_a, int src_a_reg, int src_b, int src_b_reg, int src_c, int src_c_reg, int dest, int shift, int sat)
{
	m_dsp_block_instr res;
	
	res.opcode = opcode;
	res.src_a = src_a;
	res.src_a_reg = src_a_reg;
	res.src_b = src_b;
	res.src_b_reg = src_b_reg;
	res.src_c = src_c;
	res.src_c_reg = src_c_reg;
	res.dest = dest;
	
	
	res.no_shift = (shift == NO_SHIFT);
	res.shift = (shift == NO_SHIFT) ? 0 : shift;
	res.sat = sat;
	
	res.res_addr = 0;
	
	return res;
}

m_dsp_block_instr m_dsp_block_instr_type_b_str(int opcode, int src_a, int src_a_reg, int src_b, int src_b_reg, int dest, int res_addr)
{
	m_dsp_block_instr res;
	
	res.opcode = opcode;
	res.src_a = src_a;
	res.src_a_reg = src_a_reg;
	res.src_b = src_b;
	res.src_b_reg = src_b_reg;
	res.src_c = 0;
	res.src_c_reg = 0;
	res.dest = dest;
	
	res.no_shift = 0;
	res.shift = 0;
	res.sat = 0;
	
	res.res_addr = res_addr;
	
	return res;
}

m_dsp_block_instr m_decode_dsp_block_instr(uint32_t code)
{
	m_dsp_block_instr result;
	result.opcode = range_bits(code, 5, 0);
	int format = !!(code & (1 << 5));
	
	result.src_a 	 = range_bits(code, 4, 6);
	result.src_a_reg = !!(code & (1 << 10));
	
	result.src_b 	 = range_bits(code, 4, 11);
	result.src_b_reg = !!(code & (1 << 15));
	
	result.no_shift = 0;
	
	if (format)
	{
		result.src_c = 0;
		result.src_c_reg = 0;
		
		result.dest = range_bits(code, 4, 16);
		result.shift = 0;
		result.sat = 0;
		
		result.res_addr = range_bits(code, 8, 20);
	}
	else
	{
		result.src_c 	 = range_bits(code, 4, 16);
		result.src_c_reg = !!(code & (1 << 20));
		
		result.dest = range_bits(code, 4, 21);
		result.shift = range_bits(code, 5, 25);
		result.sat = !!(code & (1 << 30));
		result.no_shift = !!(code & (1 << 31));
		
		result.res_addr = 0;
	}

    return result;
}

m_fpga_transfer_batch m_new_fpga_transfer_batch()
{
	m_fpga_transfer_batch seq;
	
	seq.buf = m_alloc(sizeof(uint32_t) * N_BLOCKS);
	seq.len = 0;
	seq.buf_len = (int)(sizeof(uint32_t) * N_BLOCKS);
	
	return seq;
}

void m_free_fpga_transfer_batch(m_fpga_transfer_batch batch)
{
	if (batch.buf) m_free(batch.buf);
}

int m_fpga_batch_append_32(m_fpga_transfer_batch *seq, uint32_t x)
{
	uint8_t bytes[4];
	
	bytes[0] = range_bits(x, 8, 24);
	bytes[1] = range_bits(x, 8, 16);
	bytes[2] = range_bits(x, 8,  8);
	bytes[3] = range_bits(x, 8,  0);
	
	int ret_val;
	
	if ((ret_val = m_fpga_batch_append(seq, bytes[0])) != NO_ERROR)
		return ret_val;
	
	if ((ret_val = m_fpga_batch_append(seq, bytes[1])) != NO_ERROR)
		return ret_val;
	
	if ((ret_val = m_fpga_batch_append(seq, bytes[2])) != NO_ERROR)
		return ret_val;
	
	if ((ret_val = m_fpga_batch_append(seq, bytes[3])) != NO_ERROR)
		return ret_val;
	
	return ret_val;
}

int m_fpga_batch_append_16(m_fpga_transfer_batch *seq, uint16_t x)
{
	uint8_t bytes[2];
	
	bytes[0] = range_bits(x, 8,  8);
	bytes[1] = range_bits(x, 8,  0);
	
	int ret_val;
	
	if ((ret_val = m_fpga_batch_append(seq, bytes[0])) != NO_ERROR)
		return ret_val;
	
	if ((ret_val = m_fpga_batch_append(seq, bytes[1])) != NO_ERROR)
		return ret_val;
	
	return ret_val;
}

int m_fpga_batch_append(m_fpga_transfer_batch *seq, uint8_t byte)
{
	if (!seq)
		return ERR_NULL_PTR;
	
	if (seq->len >= seq->buf_len)
	{
		uint8_t *new_ptr = m_realloc(seq->buf,(seq->buf_len * 2) * sizeof(uint8_t));
		
		if (!new_ptr)
			return ERR_ALLOC_FAIL;
		
		seq->buf = new_ptr;
		seq->buf_len *= 2;
	}
	
	seq->buf[seq->len++] = byte;
	
	return NO_ERROR;
}


int m_fpga_batch_concat(m_fpga_transfer_batch *seq, m_fpga_transfer_batch *seq2)
{
	if (!seq || !seq2)
		return ERR_NULL_PTR;
	
	if (!seq2->buf)
		return ERR_BAD_ARGS;
	
	if (!seq->buf)
	{
		seq->buf = m_alloc(seq2->len);
		
		if (!seq->buf)
			return ERR_ALLOC_FAIL;
		
		seq->buf_len = seq2->len;
		seq->len = 0;
	}
	else if (seq->len + seq2->len > seq->buf_len)
	{
		int new_len = seq->buf_len;
		while (new_len < seq->len + seq2->len)
			new_len *= 2;
		
		uint8_t *new_ptr = m_realloc(seq->buf, new_len * sizeof(uint8_t));
		
		if (!new_ptr) return ERR_ALLOC_FAIL;
		
		seq->buf_len = new_len;
		seq->buf = new_ptr;
	}
	
	for (int i = 0; i < seq2->len; i++)
		seq->buf[seq->len + i] = seq2->buf[i];
	
	seq->len += seq2->len;
	
	return 0;
}

char *m_dsp_block_opcode_to_string(uint32_t opcode)
{
	switch (opcode)
	{
		case BLOCK_INSTR_NOP: 			return (char*)"BLOCK_INSTR_NOP";
		case BLOCK_INSTR_LSH: 			return (char*)"BLOCK_INSTR_LSH";
		case BLOCK_INSTR_RSH: 			return (char*)"BLOCK_INSTR_RSH";
		case BLOCK_INSTR_ARSH: 			return (char*)"BLOCK_INSTR_ARSH";
		case BLOCK_INSTR_MADD: 			return (char*)"BLOCK_INSTR_MADD";
		case BLOCK_INSTR_ABS: 			return (char*)"BLOCK_INSTR_ABS";
		case BLOCK_INSTR_LUT_READ:		return (char*)"BLOCK_INSTR_LUT_READ";
		case BLOCK_INSTR_DELAY_READ: 	return (char*)"BLOCK_INSTR_DELAY_READ";
		case BLOCK_INSTR_DELAY_WRITE: 	return (char*)"BLOCK_INSTR_DELAY_WRITE";
		case BLOCK_INSTR_MEM_WRITE:		return (char*)"BLOCK_INSTR_MEM_WRITE";
		case BLOCK_INSTR_MEM_READ:		return (char*)"BLOCK_INSTR_MEM_READ";
		case BLOCK_INSTR_CLAMP: 		return (char*)"BLOCK_INSTR_CLAMP";
		case BLOCK_INSTR_MACZ: 			return (char*)"BLOCK_INSTR_MACZ";
		case BLOCK_INSTR_MAC: 			return (char*)"BLOCK_INSTR_MAC";
		case BLOCK_INSTR_MOV_ACC: 		return (char*)"BLOCK_INSTR_MOV_ACC";
		case BLOCK_INSTR_MOV_UACC: 		return (char*)"BLOCK_INSTR_MOV_UACC";
		case BLOCK_INSTR_MOV_LACC: 		return (char*)"BLOCK_INSTR_MOV_LACC";
	}
	
	return NULL;
}

void print_instruction(m_dsp_block_instr instr)
{
	switch (m_dsp_block_instr_format(instr))
	{
		case INSTR_FORMAT_A:
			printf("Instruction = %s(%d, %d, %d, %d, %d, %d, %d, %d, %d, %d)",
						m_dsp_block_opcode_to_string(instr.opcode),
						instr.src_a,
						instr.src_a_reg,
						instr.src_b,
						instr.src_b_reg,
						instr.src_c,
						instr.src_c_reg,
						instr.dest,
						
						instr.no_shift,
						instr.shift,
						instr.sat);
			break;
		
		case INSTR_FORMAT_B:
			printf("Instruction = %s(%d, %d, %d, %d, %d, 0x%04x)",
						m_dsp_block_opcode_to_string(instr.opcode),
							instr.src_a,
							instr.src_a_reg,
							instr.src_b,
							instr.src_b_reg,
							instr.dest,
							instr.res_addr);
			break;
	}
}

int m_fpga_batch_print(m_fpga_transfer_batch seq)
{
	int n = seq.len;
	
	printf("Reading out FPGA transfer batch %p (length %d)\n", seq.buf, n);
	
	if (!seq.buf)
	{
		printf("Buffer is NULL!\n");
	}
	
	if (n < 1)
	{
		printf("Batch has no bytes!\n");
	}
	
	int i = 0;
	
	int state = 0;
	int ret_state;
	int skip = 0;
	
	uint8_t byte;
	
	int ctr = 0;
	
	uint8_t reg_no = 0;
	int16_t value = 0;
	uint32_t instruction = 0;
	
	m_dsp_block_instr instr_str;
	
	while (i < n)
	{
		byte = seq.buf[i];
		
		printf("\tByte %s%d: 0x%02X. ", (n > 9 && i < 10) ? " " : "", i, byte);
		
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
						state = 1;
						ret_state = 3;
						value = 0;
						ctr = 0;
						break;

					case COMMAND_ALLOC_SRAM_DELAY:
						printf("Command ALLOC_SRAM_DELAY");
						state = 4;
						value = 0;
						ctr = 0;
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
					
					instr_str = m_decode_dsp_block_instr(instruction);
					
					printf("Word: %s; ", binary_print_32(instruction));
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
					printf("Value: %s = %d = %f (in q1.15)", binary_print_16(value), value, (float)value / (powf(2.0, 15)));
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

void m_fpga_set_input_gain(float gain_db)
{
	float v = powf(10, gain_db / 20.0);
	uint16_t s = float_to_q_nminus1(v, 5);
			
	printf("Telling FPGA to change input gain to %fdB = %f = 0b%d%d%d%d%d.%d%d%d%d%d%d%d%d%d%d%d\n", gain_db, v,
		!!(s & (1 << 15)),
		!!(s & (1 << 14)),
		!!(s & (1 << 13)),
		!!(s & (1 << 12)),
		!!(s & (1 << 11)),
		!!(s & (1 << 10)),
		!!(s & (1 << 9)),
		!!(s & (1 << 8)),
		!!(s & (1 << 7)),
		!!(s & (1 << 6)),
		!!(s & (1 << 5)),
		!!(s & (1 << 4)),
		!!(s & (1 << 3)),
		!!(s & (1 << 2)),
		!!(s & (1 << 1)),
		!!(s & (1 << 0)));
	
	spi_send(COMMAND_SET_INPUT_GAIN);
	spi_send((s & 0xFF00) >> 8);
	spi_send(s & 0x00FF);
}

void m_fpga_set_output_gain(float gain_db)
{
	float v = powf(10, gain_db / 20.0);
	uint16_t s = float_to_q_nminus1(v, 5);
			
	printf("Telling FPGA to change input gain to %fdB = %f = 0b%d%d%d%d%d.%d%d%d%d%d%d%d%d%d%d%d\n", gain_db, v,
		!!(s & (1 << 15)),
		!!(s & (1 << 14)),
		!!(s & (1 << 13)),
		!!(s & (1 << 12)),
		!!(s & (1 << 11)),
		!!(s & (1 << 10)),
		!!(s & (1 << 9)),
		!!(s & (1 << 8)),
		!!(s & (1 << 7)),
		!!(s & (1 << 6)),
		!!(s & (1 << 5)),
		!!(s & (1 << 4)),
		!!(s & (1 << 3)),
		!!(s & (1 << 2)),
		!!(s & (1 << 1)),
		!!(s & (1 << 0)));
	
	spi_send(COMMAND_SET_OUTPUT_GAIN);
	spi_send((s & 0xFF00) >> 8);
	spi_send(s & 0x00FF);
}

m_fpga_resource_report m_empty_fpga_resource_report()
{
	m_fpga_resource_report result;
	memset(&result, 0, sizeof(m_fpga_resource_report));
	return result;
}

int m_fpga_resource_report_integrate(m_fpga_resource_report *cxt, m_fpga_resource_report *local)
{
	if (!cxt || !local)
		return ERR_NULL_PTR;
	
	cxt->blocks += local->blocks;
	cxt->memory += local->memory;
	cxt->sdelay += local->sdelay;
	cxt->ddelay += local->ddelay;
	cxt->luts 	+= local->luts;
	
	return NO_ERROR;
}

/*
m_effect_desc *create_flanger_eff_desc()
{
	m_effect_desc *eff = new_m_effect_desc("Flanger");
	m_parameter *param1 = new_m_parameter_wni("Center", "center", 5, 0.1, 20.0);
	m_parameter *param2 = new_m_parameter_wni("Depth", "depth", 4.0, 0.1, 10.0);
	m_parameter *param3 = new_m_parameter_wni("Rate", "rate", 1.0, 0.0, 2.0);
	m_parameter *param4 = new_m_parameter_wni("Strength", "strength", 0.8, 0.0, 1.0);
	
	m_effect_desc_add_param(eff, param1);
	m_effect_desc_add_param(eff, param2);
	m_effect_desc_add_param(eff, param3);
	m_effect_desc_add_param(eff, param4);
	
	// Load phase accumulator from mem[0]
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_load_acc(0)));
	// Add phase accumulator (wrapping)
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_accu(0, 1)));
	m_effect_desc_add_register_val(eff, 1, 0, DSP_REG_FORMAT_LITERAL, "* rate 6087.0");
	// Save new phase accumulator
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_save_acc(0)));
	// Put accumulator on ch0
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_mov_uacc(1)));
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_lsh4(1, 0, 1)));
	
	// take sin of phase accumulator
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_lut(1, 0, 0, 2)));
	
	// shift sin(phase_acc) to q12.4
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_arsh8(2, 0, 3)));
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_arsh4(3, 0, 3)));
	
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_mul_noshift(3, 0, 0, 1, 4)));
	m_effect_desc_add_register_val(eff, 8, 0, DSP_REG_FORMAT_LITERAL, "* 44.1 depth");
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_add(4, 0, 0, 1, 5)));
	m_effect_desc_add_register_val(eff, 9, 0, DSP_REG_FORMAT_LITERAL, "* 8 * 44.1 center");
	
	// clamp to valid range
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_clamp(5, 0, 0, 1, 1, 1, 6)));
	m_effect_desc_add_register_val(eff, 10, 0, DSP_REG_FORMAT_LITERAL, "0");
	m_effect_desc_add_register_val(eff, 10, 1, DSP_REG_FORMAT_LITERAL, "* 8 * 44.1 + center depth");
	// get fractional delay
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_frac_delay(6, 0, 0, 7)));
	// write to delay buffer
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_delay_write(0, 0, 0)));
	// mix in fractional delay
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_madd(0, 1, 7, 0, 0, 0, 0, 0)));
	m_effect_desc_add_register_val(eff, 13, 0, 0, "- 0 strength");
	
	m_effect_desc_add_resource_request(eff, new_fpga_resource_req(M_FPGA_RESOURCE_DDELAY, 8192));
	
	return eff;
}
*/
