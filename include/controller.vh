`define COMMAND_BEGIN_PROGRAM	 	8'd1
`define COMMAND_WRITE_BLOCK_INSTR 	8'd2
`define COMMAND_WRITE_BLOCK_REG_0 	8'd3
`define COMMAND_WRITE_BLOCK_REG_1 	8'd4
`define COMMAND_ALLOC_DELAY 		8'd5
`define COMMAND_END_PROGRAM	 		8'd10
`define COMMAND_SET_INPUT_GAIN 		8'd11
`define COMMAND_SET_OUTPUT_GAIN 	8'd12
`define COMMAND_UPDATE_BLOCK_REG_0 	8'd13
`define COMMAND_UPDATE_BLOCK_REG_1 	8'd14
`define COMMAND_COMMIT_REG_UPDATES 	8'd15

// If we're in a 'waiting' state, but no new data has
// appeared for a whole 100ms, then it's likely
// there was an alignment mistake, possibly
// in software, or a transfer corruption.
// In this case, reset the controller, so that
// the machine doesn't get permanently locked up!
`define CONTROLLER_TIMEOUT_CYCLES	32'd11250000
