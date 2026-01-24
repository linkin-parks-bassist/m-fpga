#ifndef M_EFFECT_H_
#define M_EFFECT_H_

#define BLOCK_INSTR_NOP 		0
#define BLOCK_INSTR_ADD 		1
#define BLOCK_INSTR_SUB 		2
#define BLOCK_INSTR_LSH 		3
#define BLOCK_INSTR_RSH 		4
#define BLOCK_INSTR_ARSH 		5
#define BLOCK_INSTR_MUL 		6
#define BLOCK_INSTR_MAD			7
#define BLOCK_INSTR_ABS			8
#define BLOCK_INSTR_LUT			9
#define BLOCK_INSTR_ENVD 		10
#define BLOCK_INSTR_DELAY_READ 	11
#define BLOCK_INSTR_DELAY_WRITE 12
#define BLOCK_INSTR_SAVE 		13
#define BLOCK_INSTR_LOAD		14
#define BLOCK_INSTR_MOV			15
#define BLOCK_INSTR_CLAMP		16
#define BLOCK_INSTR_MACZ		17
#define BLOCK_INSTR_MAC			18
#define BLOCK_INSTR_MOV_ACC		19

#define STOCK_LUTS 2

#define BLOCK_INSTR_WIDTH	32
#define BLOCK_INSTR_OP_WIDTH 5
#define BLOCK_REG_ADDR_WIDTH 4

#define BLOCK_INSTR_OP_TYPE_START 	(4 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH)
#define BLOCK_INSTR_PMS_START		(BLOCK_INSTR_OP_TYPE_START + 5)
#define SHIFT_WIDTH 4

#define BLOCK_RES_ADDR_WIDTH		8

#define COMMAND_WRITE_BLOCK_INSTR 	0b10010000
#define COMMAND_WRITE_BLOCK_REG 	0b11100000
#define COMMAND_UPDATE_BLOCK_REG 	0b11101000
#define COMMAND_ALLOC_SRAM_DELAY 	0b00100000
#define COMMAND_SWAP_PIPELINES 		0b00000001
#define COMMAND_RESET_PIPELINE 		0b00001001
#define COMMAND_SET_INPUT_GAIN 		0b00000010
#define COMMAND_SET_OUTPUT_GAIN 	0b00000011

#define INSTR_FORMAT_A 0
#define INSTR_FORMAT_B 1

#define N_BLOCKS 255
#define N_BLOCKS_REGS 2

#define M_DSP_BLOCK_N_REGS 2

typedef struct
{
	int opcode;
	int src_a;
	int src_a_reg;
	int src_b;
	int src_b_reg;
	int src_c;
	int src_c_reg;
	int dest;
	
	int shift;
	int sat;
	
	int res_addr;
} m_dsp_block_instr;

int m_dsp_block_instr_format(m_dsp_block_instr instr);
uint32_t m_encode_dsp_block_instr(m_dsp_block_instr instr);

m_dsp_block_instr m_dsp_block_instr_type_a_str(int opcode, int src_a, int a_reg, int src_b, int b_reg, int src_c, int c_reg, int dest,  int shift, int sat);
m_dsp_block_instr m_dsp_block_instr_type_b_str(int opcode, int src_a, int a_reg, int src_b, int b_reg, int dest, int res_addr);

typedef struct
{
	int len;
	uint8_t *buf;
	int buf_len;
} m_fpga_transfer_batch;

m_fpga_transfer_batch m_new_fpga_transfer_batch();
void m_free_fpga_transfer_batch(m_fpga_transfer_batch batch);

int m_send_bytes_to_fpga(uint8_t *buf, int n);
int m_send_byte_to_fpga(uint8_t byte);

int m_fpga_send_byte(uint8_t byte);

void m_fpga_set_input_gain(float gain_db);
void m_fpga_set_output_gain(float gain_db);

int m_fpga_batch_append(m_fpga_transfer_batch *seq, uint8_t byte);
int m_fpga_batch_append_16(m_fpga_transfer_batch *seq, uint16_t x);
int m_fpga_batch_append_32(m_fpga_transfer_batch *seq, uint32_t x);

int m_fpga_transfer_batch_send(m_fpga_transfer_batch batch);
int m_fpga_batch_print(m_fpga_transfer_batch seq);

typedef struct
{
	unsigned int blocks;
	unsigned int memory;
	unsigned int sdelay;
	unsigned int ddelay;
	unsigned int luts;
} m_fpga_resource_report;

m_fpga_resource_report m_empty_fpga_resource_report();
int m_fpga_resource_report_integrate(m_fpga_resource_report *cxt, m_fpga_resource_report *local);

#define M_DERIVED_QUANTITY_CONST_FLT 0
#define M_DERIVED_QUANTITY_CONST_INT 1
#define M_DERIVED_QUANTITY_REFERENCE 2
#define M_DERIVED_QUANTITY_FCALL_ADD 3
#define M_DERIVED_QUANTITY_FCALL_SUB 4
#define M_DERIVED_QUANTITY_FCALL_MUL 5
#define M_DERIVED_QUANTITY_FCALL_DIV 6
#define M_DERIVED_QUANTITY_FCALL_ABS 7
#define M_DERIVED_QUANTITY_FCALL_SQR 8
#define M_DERIVED_QUANTITY_FCALL_EXP 9
#define M_DERIVED_QUANTITY_FCALL_LOG 10
#define M_DERIVED_QUANTITY_FCALL_POW 11
#define M_DERIVED_QUANTITY_FCALL_SIN 12
#define M_DERIVED_QUANTITY_FCALL_COS 13
#define M_DERIVED_QUANTITY_FCALL_TAN 14

#define M_DERIVED_QUANTITY_TYPE_MAX_VAL M_DERIVED_QUANTITY_FCALL_TAN

typedef struct {
	float value;
	char *name_internal;
} m_parameter;

m_parameter *new_m_parameter_wni(const char *name, const char *name_internal, float value, float min, float max);

typedef struct m_parameter_pll {
	m_parameter *data;
	struct m_parameter_pll *next;
} m_parameter_pll;

m_parameter_pll *m_parameter_pll_append(m_parameter_pll *pll, m_parameter *param);

typedef struct m_derived_quantity
{
	int type;
	union {
		float val_float;
		int16_t val_int;
		char *ref_name;
		struct m_derived_quantity **sub_dqs;
	} val;
} m_derived_quantity;

m_derived_quantity *new_m_derived_quantity_from_string(char *str);

int m_derived_quantity_references_param(m_derived_quantity *dq, m_parameter *param);
float m_derived_quantity_compute(m_derived_quantity *dq, m_parameter_pll *params);

#define DSP_REG_FORMAT_LITERAL 0xFFFF

typedef struct
{
	int reg;
	int format;
	
	m_derived_quantity *dq;
} m_dsp_register_val;

m_dsp_register_val *new_m_dsp_register_val(int reg, int format, m_derived_quantity *dq);
m_dsp_register_val *new_m_dsp_register_val_literal(int reg, int16_t literal_value);

typedef struct
{
	m_dsp_block_instr instr;
	m_dsp_register_val *reg_vals[M_DSP_BLOCK_N_REGS];
} m_dsp_block;

m_dsp_block *new_m_dsp_block();
m_dsp_block *new_m_dsp_block_with_instr(m_dsp_block_instr instr);
int m_dsp_block_add_register_val(m_dsp_block *blk, int i, m_dsp_register_val *p);
int m_dsp_block_uses_param(m_dsp_block *blk, m_parameter *param);

typedef struct
{
	const char *name;
	
	int n_blocks;
	int block_array_len;
	m_dsp_block **blocks;
	
	int n_params;
	int param_array_len;
	m_parameter **params;
} m_effect_desc;

typedef struct m_effect_desc_pll {
	m_parameter *data;
	struct m_effect_desc_pll *next;
} m_effect_desc_pll;

m_effect_desc *new_m_effect_desc(const char *name);
int m_effect_desc_add_block(m_effect_desc *eff, m_dsp_block *blk);
int m_effect_desc_add_param(m_effect_desc *eff, m_parameter *param);

int m_effect_desc_add_register_val_literal(m_effect_desc *eff, int block_no, int reg, uint16_t val);
int m_effect_desc_add_register_val(m_effect_desc *eff, int block_no, int reg, int format, char *expr);

int m_fpga_transfer_batch_append_effect_register_writes(
		m_fpga_transfer_batch *batch,
		m_effect_desc *eff, int blocks_start,
		m_parameter_pll *params
	);
int m_fpga_transfer_batch_append_effect_register_updates(
		m_fpga_transfer_batch *batch,
		m_effect_desc *eff, int blocks_start,
		m_parameter_pll *params
	);

int m_fpga_transfer_batch_append_effect(
		m_effect_desc *eff,
		const m_fpga_resource_report *cxt,
		m_fpga_resource_report *report,
		m_parameter_pll *params,
		m_fpga_transfer_batch *batch
	);

int16_t float_to_q_nminus1(float x, int shift);
int16_t float_to_q15(float x);

#endif
