`define RR_ARBITER_STATE_READY 		0
`define RR_ARBITER_STATE_WAIT 		1

`default_nettype none

module rr_arbiter
	#(
		parameter req_data_width = 16,
		parameter server_data_width = 16,
		parameter n_clients = 32
	)
	(
		input wire clk,
		input wire reset,
		
		input wire [(n_clients * req_data_width) - 1 : 0] req_data,
		input wire [n_clients  - 1 : 0] reqs,
		
		output reg [server_data_width - 1 : 0] data_out,
		output reg [n_clients  - 1 : 0] readies,
		
		output reg [req_data_width - 1 : 0] arbiter_req_data,
		output reg arbiter_req,
		
		input wire [server_data_width - 1 : 0] server_data,
		input wire server_ready
	);

	localparam client_id_width = $clog2(n_clients);
	
	reg [client_id_width - 1 : 0] client = 0;
	
	reg [7:0] state = `RR_ARBITER_STATE_READY;

	reg wait_one = 0;

	integer i;
	always @(posedge clk) begin
		arbiter_req <= 0;
		
		if (reset) begin
			client 		<= 0;
			state 		<= `RR_ARBITER_STATE_READY;
			readies 	<= 0;
			wait_one 	<= 0;
		end
		else begin
			readies <= 0;
			case (state)
				`RR_ARBITER_STATE_READY: begin
					if (reqs[client]) begin
						arbiter_req_data <= req_data[n_clients * client +: req_data_width];
						arbiter_req <= 1;
					
						state <= `RR_ARBITER_STATE_WAIT;
						wait_one <= 1;
					end
					else begin
						if (client == (n_clients - 1)) begin
							client <= 0;
						end
						else begin
							client <= client + 1;
						end
					end
				end
				
				`RR_ARBITER_STATE_WAIT: begin
					arbiter_req <= 0;
					
					if (!wait_one && server_ready) begin
						data_out <= server_data;
						readies[client] <= 1;
						
						state <= `RR_ARBITER_STATE_READY;
						
						if (client == (n_clients - 1)) begin
							client <= 0;
						end
						else begin
							client <= client + 1;
						end
					end
					
					wait_one <= 0;
				end
			endcase
		end
	end
endmodule

module rr_arbiter_handle
	#(
		parameter req_data_width 	= 16,
		parameter handle_width 		= 8,
		parameter server_data_width = 16,
		parameter n_clients 		= 32
	)
	(
		input wire clk,
		input wire reset,
		
		input wire [n_clients * req_data_width - 1 : 0] req_data_flat,
		input wire [n_clients * handle_width   - 1 : 0] req_handles_flat,
		input wire [n_clients  	   - 1 : 0] reqs,
		
		output reg [server_data_width - 1 : 0] data_out,
		output reg [n_clients  - 1 : 0] readies,
		
		output reg [req_data_width 	- 1 : 0] arbiter_req_data,
		output reg [handle_width 	- 1 : 0] arbiter_req_handle,
		output reg arbiter_req,
		
		input wire [server_data_width - 1 : 0] server_data,
		input wire server_ready
	);

	wire [req_data_width-1:0] req_data	 [0:n_clients-1];
	wire [handle_width-1:0]   req_handles  [0:n_clients-1];

	genvar i;
	generate
		for (i = 0; i < n_clients; i = i + 1) begin : UNFLATTEN
			assign req_data[i] =
				req_data_flat[i * req_data_width +: req_data_width];

			assign req_handles[i] =
				req_handles_flat[i * handle_width +: handle_width];
		end
	endgenerate


	localparam client_id_width = $clog2(n_clients);
	
	reg [client_id_width 	- 1 : 0] client = 0;
	
	reg [7:0] state = `RR_ARBITER_STATE_READY;

	reg wait_one = 0;

	always @(posedge clk) begin
		arbiter_req <= 0;
		
		if (reset) begin
			client 	<= 0;
			state 	<= `RR_ARBITER_STATE_READY;
			readies <= 0;
		end
		else begin
			readies <= 0;
			case (state)
				`RR_ARBITER_STATE_READY: begin
					if (reqs[client]) begin
						arbiter_req_data 	<= req_data[client];
						arbiter_req_handle 	<= req_handles[client];
						arbiter_req 		<= 1;
					
						state <= `RR_ARBITER_STATE_WAIT;
						wait_one <= 1;
					end
					else begin
						if (client == (n_clients - 1)) begin
							client <= 0;
						end
						else begin
							client <= client + 1;
						end
					end
				end
				
				`RR_ARBITER_STATE_WAIT: begin
					arbiter_req <= 0;
					
					if (!wait_one && server_ready) begin
						data_out <= server_data;
						readies[client] <= 1;
						
						state <= `RR_ARBITER_STATE_READY;
						
						if (client == (n_clients - 1)) begin
							client <= 0;
						end
						else begin
							client <= client + 1;
						end
					end
					
					wait_one <= 0;
				end
			endcase
		end
	end
endmodule
	
`default_nettype wire
