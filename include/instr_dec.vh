`define BLOCK_INSTR_FORMAT_A 0
`define BLOCK_INSTR_FORMAT_B 1

// Do nothing. This doesn't even use a branch;
// it isn't passed on from the instruction
// fetch/decode stage
`define BLOCK_INSTR_NOP 			0

// Regular arithmetic writing to channels.
// Uses main branch
`define BLOCK_INSTR_MADD			1
`define BLOCK_INSTR_ARSH 			2

// Single stage math operations/moves, writing
// to channels. Uses MISC branch
`define BLOCK_INSTR_LSH 			3
`define BLOCK_INSTR_RSH 			4
`define BLOCK_INSTR_ABS				5
`define BLOCK_INSTR_MIN			    6
`define BLOCK_INSTR_MAX			    7
`define BLOCK_INSTR_MOV_ACC			8
`define BLOCK_INSTR_MOV_LACC		9
`define BLOCK_INSTR_MOV_UACC		10

`define N_MISC_OPS 8

// Accumulator MAC instructions. Uses MAC branch
// _MAC_: acc = a * b + acc
// _MACZ: acc = a * b + 0
// UMAC_: a and b are treated as unsigned
//
// MAC branch is specialised as it does not
// wait on the accumulator as a dependency
// the addition is done in the commit stage
// therefore is it much faster!
`define BLOCK_INSTR_MACZ			11
`define BLOCK_INSTR_UMACZ			12
`define BLOCK_INSTR_MAC				13
`define BLOCK_INSTR_UMAC			14

// Interfacing with `resources'. Each has its own branch
`define BLOCK_INSTR_LUT_READ		15
`define BLOCK_INSTR_DELAY_READ 		16
`define BLOCK_INSTR_DELAY_WRITE 	17
`define BLOCK_INSTR_MEM_READ 		18
`define BLOCK_INSTR_MEM_WRITE		19

`define N_INSTR_BRANCHES 	6

`define INSTR_BRANCH_MADD   0
`define INSTR_BRANCH_MAC    1
`define INSTR_BRANCH_MISC   2
`define INSTR_BRANCH_DELAY  3
`define INSTR_BRANCH_LUT 	4
`define INSTR_BRANCH_MEM 	5

`define BLOCK_OP_TYPE_WIDTH 3

`define BLOCK_INSTR_WIDTH 		32

`define BLOCK_INSTR_OP_WIDTH 	5
`define BLOCK_REG_ADDR_WIDTH   	4
`define BLOCK_PMS_WIDTH			5
`define BLOCK_RES_ADDR_WIDTH	8

`define SHIFT_WIDTH   		5

`define POS_ONE_REGISTER_ADDR  	4'd3
`define NEG_ONE_REGISTER_ADDR  	4'd4
`define ZERO_REGISTER_ADDR 		4'd5
