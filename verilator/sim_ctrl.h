#ifndef HEADER_H_
#define HEADER_H_

#define BLOCK_INSTR_NOP 	0
#define BLOCK_INSTR_ADD 	1
#define BLOCK_INSTR_SUB 	2
#define BLOCK_INSTR_LSH 	3
#define BLOCK_INSTR_RSH 	4
#define BLOCK_INSTR_ARSH 	5
#define BLOCK_INSTR_MUL 	6
#define BLOCK_INSTR_MAD		7
#define BLOCK_INSTR_ABS		8
#define BLOCK_INSTR_LUT		9
#define BLOCK_INSTR_ENVD 	10
#define BLOCK_INSTR_DELAY 	11
#define BLOCK_INSTR_SAVE 	12
#define BLOCK_INSTR_LOAD	13
#define BLOCK_INSTR_MOV		14
#define BLOCK_INSTR_CLAMP	15
#define BLOCK_INSTR_MACZ	16
#define BLOCK_INSTR_MAC		17
#define BLOCK_INSTR_MOV_ACC	18

#define BLOCK_INSTR_OP_WIDTH 5
#define BLOCK_REG_ADDR_WIDTH 4

#define BLOCK_INSTR_OP_TYPE_START 	(4 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH)
#define BLOCK_INSTR_PMS_START		(BLOCK_INSTR_OP_TYPE_START + 5)
#define SHIFT_WIDTH 4

#define BLOCK_RES_ADDR_WIDTH	8

#define COMMAND_WRITE_BLOCK_INSTR 	0b10010000
#define COMMAND_WRITE_BLOCK_REG 	0b11100000
#define COMMAND_UPDATE_BLOCK_REG 	0b11101001
#define COMMAND_ALLOC_SRAM_DELAY 	0b00100000
#define COMMAND_SWAP_PIPELINES 		0b00000001
#define COMMAND_RESET_PIPELINE 		0b00001001
#define COMMAND_SET_INPUT_GAIN 		0b00000010
#define COMMAND_SET_OUTPUT_GAIN 	0b00000011

typedef struct
{
	int block_usage;
	int mem_usage;
	int sram_delay_usage;
} resource_report;

typedef struct
{
	int opcode;
	int src_a;
	int src_b;
	int src_c;
	int dest;
	
	int src_a_reg;
	int src_b_reg;
	int src_c_reg;
	int dest_reg;
	
	int shift;
	int sat;
	
	int res_addr;
} block_instr;

typedef struct
{
	char *name;
	float value;
} parameter;

#define DQE_CALL_CONST 	0
#define DQE_CALL_PARAM 	1
#define DQE_CALL_ADD 	2
#define DQE_CALL_SUB 	3
#define DQE_CALL_MUL 	4
#define DQE_CALL_DIV 	5
#define DQE_CALL_ABS 	6
#define DQE_CALL_SQR 	7
#define DQE_CALL_EXP 	8
#define DQE_CALL_LOG 	9
#define DQE_CALL_POW 	10
#define DQE_CALL_SIN 	11
#define DQE_CALL_COS 	12
#define DQE_CALL_TAN 	13

#define N_BLOCKS 255
#define N_BLOCKS_REGS 2

#define INSTR_FORMAT_A 0
#define INSTR_FORMAT_B 1

typedef struct derived_quantity_expr
{
	int call;
	float value;
	parameter *param;
	struct derived_quantity_expr **sub_exprs;
} derived_quantity_expr;

typedef struct
{
	int reg;
	int format;
	
	derived_quantity_expr *expr;
} block_register;

typedef struct
{
	block_instr instr;
	block_register *regs[N_BLOCKS_REGS];
} block;

typedef struct
{
	int n_blocks;
	int block_array_len;
	block **blocks;
	
	int n_params;
	int param_array_len;
	parameter **params;
} effect;

typedef struct
{
	int n_effects;
	int effect_array_len;
	effect **effects;
} pipeline;

typedef struct
{
	int len;
	uint8_t *buf;
	int buf_len;
} transfer_sequence;

pipeline *new_pipeline();
int pipeline_add_effect(pipeline *pl, effect *eff);
effect *new_effect();
effect *create_amplifier_effect(float gain);

transfer_sequence pipeline_transfer_sequence(pipeline *pl);

int send_transfer_sequence(transfer_sequence seq);

int16_t float_to_q_nminus1(float x, int shift);

#endif
