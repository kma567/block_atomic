`define  T_RL	   	16
`define  T_WL      	16
`define  T_AL		6
`define  T_RCD   	2	// AL = 3
`define  T_CCD		8
`define  T_RP	   	10
`define  T_WIDTH 	2
`define  T_MRD		6
`define  T_MOD		30
`define  T_RLL		1024
`define  T_ZQCS   	128
`define  T_RFC		88

`define	 M_PRE		0	// PRE
`define  M_NOP1 	`M_PRE+`T_WIDTH // NOP after PRE
`define  M_MRS1		`M_NOP1+`T_RP // Set MR1 with DLL Set
`define  M_NOP2		`M_MRS1+`T_WIDTH // NOP after DLL Set
`define  M_MRS0		`M_NOP2+`T_MRD // Set MR0 with DLL Reset
`define  M_NOP3		`M_MRS0+`T_WIDTH // NOP after DLL Reset
`define  M_ZQCS		`M_NOP3+`T_MOD // Send ZQCS
`define  M_NOP4		`M_ZQCS+`T_WIDTH // NOP after ZQCS

`define  MR_MRS1	`M_NOP4+`T_ZQCS // Set MR1 with DLL Set
`define  MR_NOP1	`MR_MRS1+`T_WIDTH // NOP after DLL Set
`define  MR_MRS0	`MR_NOP1+`T_MRD // Set MR0 with DLL Reset
`define  MR_NOP2	`MR_MRS0+`T_WIDTH // NOP after DLL Reset
`define  MR_DONE	`MR_NOP2+`T_RLL // DONE

`define  REF_REF	0
`define  REF_NOP	`REF_REF+2
`define  REF_DONE	`REF_NOP+`T_RFC

`define  GNR_ACT	0
`define  GNR_NOP1	`GNR_ACT+2
`define  GNR_READ	`GNR_NOP1+`T_RCD
`define  GNR_NOP2	`GNR_READ+2
`define  GNR_LSN	`GNR_NOP2+`T_RL-2
`define  GNR_DATA	`GNR_LSN+1
`define  GNR_DONE	`GNR_DATA+6+`T_RP

`define  GNW_ACT	0
`define  GNW_NOP1	`GNW_ACT+2
`define  GNW_WRITE	`GNW_NOP1+`T_RCD
`define  GNW_NOP2	`GNW_WRITE+2
`define  GNW_DQS	`GNW_NOP2+`T_WL-4
`define  GNW_GET	`GNW_DQS+1
`define  GNW_DM		`GNW_GET+1
`define  GNW_TSB	`GNW_DM+9
`define  GNW_DONE	`GNW_TSB+20

`define  ATRW_ACT	0
`define  ATRW_NOP1	`ATRW_ACT+2
`define  ATRW_READ	`ATRW_NOP1+`T_RCD
`define  ATRW_NOP2	`ATRW_READ+2
`define  ATRW_WRITE  `ATRW_NOP2+`T_CCD+4-2
`define  ATRW_NOP3  `ATRW_WRITE+2
`define  ATRW_LSN	`ATRW_NOP2+`T_RL-2
`define  ATRW_DATA	`ATRW_LSN+1
`define  ATRW_DQS	`ATRW_NOP3+`T_WL-4
`define  ATRW_GET	`ATRW_DQS+1
`define  ATRW_DM	`ATRW_GET+1
`define  ATRW_TSB	`ATRW_DM+9
`define  ATRW_DONE	`ATRW_TSB+18

module Processing_logic(
	// Outputs
	DATA_get, 
	CMD_get,
	RETURN_put, 
	RETURN_address, RETURN_data,  //construct RETURN_data_in
	cs_bar, ras_bar, cas_bar, we_bar,  // read/write function
	BA, A, DM,
	DQS_out, DQ_out,
	ts_con, modify_setting,
	// Inputs
	clk, ck, reset, ready, 
	CMD_empty, CMD_data_out, DATA_data_out,
	RETURN_full,
	DQS_in, DQ_in
	);

	parameter BL = 8;  // Burst Length
	parameter BT = 0;  // Burst Type
	parameter CL = 5;  // CAS Latency (CL)
	parameter AL = 3;  // Posted CAS# Additive Latency (AL)


	input 	 	  clk, ck, reset, ready;
	input 	 	  CMD_empty, RETURN_full;
	input [33:0]  CMD_data_out;
	input [15:0]  DATA_data_out;
	input [15:0]  DQ_in;
	input [1:0]   DQS_in;

	output reg 			CMD_get;
	output reg		 	DATA_get, RETURN_put;
	output reg [25:0] 	RETURN_address;
	output wire [15:0] 	RETURN_data;
	output reg			cs_bar, ras_bar, cas_bar, we_bar;
	output reg [2:0]	BA;
	output reg [12:0] 	A;
	output reg [1:0]	DM;
	output reg [15:0]  	DQ_out;
	output reg [1:0]   	DQS_out;
	output reg ts_con;
	output reg modify_setting;
	
	reg [2:0] read_pointer;
	reg [2:0] state;
	reg	[11:0] counter;
	reg [12:0] refresh_counter;
	
	reg [25:0] CMD_addr;
	reg [1:0] CMD_sz;
	reg [2:0] CMD_op;
	reg BLOCK_cmd;
	
	reg [15:0] ATOMIC_data;
	reg	ATOMIC_cmd;
	reg [15:0] ATOMIC_data_in;
	
	reg listen;	
	reg DM_flag;


	
	localparam	[2:0]
		IDLE 		= 3'b000,
		MODIFY		= 3'b001,
		DECODE		= 3'b010,
		REFRESH	 	= 3'b011,
		S_GNR 		= 3'b100,
		S_GNW 		= 3'b101,
		S_ATRW 		= 3'b110;
		

		
	localparam	[2:0]
		SCR 		= 3'b001,
		SCW 		= 3'b010,
		BLR			= 3'b011,
		BLW			= 3'b100,
		ATW			= 3'b101,
		ATR			= 3'b110;
		
	localparam	[3:0]
		NOP			= 4'b0111,
		ACT			= 4'b0011,
		READ		= 4'b0101,
		WRITE		= 4'b0100,
		PRE			= 4'b0010,
		MRS			= 4'b0000,
		ZQCS		= 4'b0110,
		REF			= 4'b0001;	
		

always @(posedge clk)
    if (reset)
	begin
		modify_setting <= 0;
		
	    counter <= 0;
		refresh_counter <= 0;
		state <= IDLE;
		
		CMD_get <= 0;
		DATA_get <= 0;
		RETURN_put <= 0;
		
		ts_con <= 0;
		DM_flag <= 0;
		{cs_bar, ras_bar, cas_bar, we_bar} <= NOP;
		listen <= 0;
		
		BLOCK_cmd <= 0;
		ATOMIC_cmd <= 0;		
	end
	else
	  begin
	  
        RETURN_address <= CMD_addr;
		ATOMIC_data <= ATOMIC_data_in;
		
		if (ready)
			refresh_counter <= refresh_counter + 1;
		
		case (state)
		
			IDLE:
			begin
				counter <= 0;
				if (ready)
				begin
					if (!modify_setting)
						state <= MODIFY;
					if (refresh_counter[12])
					begin
						CMD_get <= 0;
						if (!ck)
							state <= REFRESH;
					end
					else
					begin
						if (!CMD_empty && !CMD_get)
							CMD_get <= 1;
						if (CMD_get)
						begin
							CMD_get <= 0;
							state <= DECODE;
						end
					end
				end
			end
			
			MODIFY:
			begin
				counter <= counter + 1;
				 case (counter)
				 
					`M_PRE:  begin
								{cs_bar, ras_bar, cas_bar, we_bar} <= PRE; // EMRS command
								A[10] <= 1;
							 end
							 
					`M_NOP1: begin
								{cs_bar, ras_bar, cas_bar, we_bar} <= NOP;
							 end
							 
					`M_MRS1: begin
								{cs_bar, ras_bar, cas_bar, we_bar} <= MRS; // EMRS command 
								A[12] <= 0; // Output enabled
								A[11] <= 0; // TDQS enabled for 8 banks
								A[10] <= 0;
								{A[9], A[6], A[2]} <= 3'b001; // A[9,6,2] RTT disabled
								A[8] <= 0;
								A[7] <= 0; // write leveling disabled					 
								{A[5], A[1]} <= 2'b00; // A[5,1] Output driver
								A[4:3] <= 2'b01; // AL = CL - 1 = 4
								A[0] <= 0; // DLL enabled
								BA <= 3'b001;
							end
							
					`M_NOP2: {cs_bar, ras_bar, cas_bar, we_bar} <= NOP; // NOP command

					`M_MRS0: begin
								{cs_bar, ras_bar, cas_bar, we_bar} <= MRS; // EMRS command  
								A[12] <= 0; // DLL off
								A[11:9] <= 3'b010; // Write Recovery = 6
								A[8] <= 1; // DLL Reset
								A[7] <= 0;
								A[6:4] <= 3'b001; // CAS latency = 5
								A[3] <= 0; // Read type = sequential
								A[2] <= 0;
								A[1:0] <= 2'b10; // Burst length = 4;
								BA <= 3'b000; // MRS
							end
								
					`M_NOP3: {cs_bar, ras_bar, cas_bar, we_bar} <= NOP; // NOP command

					`M_ZQCS: begin
								{cs_bar, ras_bar, cas_bar, we_bar} <= ZQCS; 
								A[10] <= 0; 
							end
							
					`M_NOP4: {cs_bar, ras_bar, cas_bar, we_bar} <= NOP; // NOP command	

					`MR_MRS1: begin
								{cs_bar, ras_bar, cas_bar, we_bar} <= MRS; // EMRS command
								A[12] <= 0; // Output enabled
								A[11] <= 0; // TDQS enabled for 8 banks
								A[10] <= 0;
								{A[9], A[6], A[2]} <= 3'b000; // A[9,6,2] RTT disabled
								A[8] <= 0;
								A[7] <= 0; // write leveling disabled					 
								{A[5], A[1]} <= 2'b00; // A[5,1] Output driver
								A[4:3] <= 2'b10; // AL = CL - 2 = 3
								A[0] <= 0; // DLL enabled
								BA <= 3'b001;
							end
							
					`MR_NOP1: {cs_bar, ras_bar, cas_bar, we_bar} <= NOP; // NOP command

					`MR_MRS0: begin
								{cs_bar, ras_bar, cas_bar, we_bar} <= MRS; // EMRS command  
								A[12] <= 0; // DLL off
								A[11:9] <= 3'b010; // Write Recovery = 6
								A[8] <= 1; // DLL Reset
								A[7] <= 0;
								A[6:4] <= 3'b001; // CAS latency = 5
								A[3] <= 0; // Read type = sequential
								A[2] <= 0;
								A[1:0] <= 2'b00; // Burst length = 8;
								BA <= 3'b000; // MRS
							end
								
					`MR_NOP2: {cs_bar, ras_bar, cas_bar, we_bar} <= NOP; // NOP command
					
					`MR_DONE: begin
							state <= IDLE; // done
							modify_setting <= 1;
						end
					
				endcase
			end
			
			REFRESH:
			begin
				counter <= counter + 1;
				case (counter)
				
					`REF_REF:
					begin
						{cs_bar, ras_bar, cas_bar, we_bar} <= REF;
					end
					
					`REF_NOP:
					begin
						{cs_bar, ras_bar, cas_bar, we_bar} <= NOP;
					end
					
					`REF_DONE:
					begin
						state <= IDLE;
						refresh_counter <= 0;
					end
					
				endcase
			end
			
			DECODE:
			begin
				CMD_addr <= CMD_data_out[30:5];
				CMD_sz <= CMD_data_out[4:3];
				CMD_op <= CMD_data_out[2:0];
				if (!ck)
				begin					
					case (CMD_data_out[33:31])
					
						SCR:
						begin
							state <= S_GNR;
							BLOCK_cmd <= 0;
							ATOMIC_cmd <= 0;
						end
						
						SCW:
						begin
							state <= S_GNW;
							BLOCK_cmd <= 0;
							ATOMIC_cmd <= 0;
						end
						
						BLR:
						begin
							state <= S_GNR;
							BLOCK_cmd <= 1;
							ATOMIC_cmd <= 0;
						end
						
						BLW:
						begin
							state <= S_GNW;
							BLOCK_cmd <= 1;
							ATOMIC_cmd <= 0;
						end
						
						ATR, ATW:
						begin
							state <= S_ATRW;
							BLOCK_cmd <= 0;
							ATOMIC_cmd <= 1;
						end
						
						default: state <= IDLE;
						
					endcase
				end				
			end
			
			S_GNR:
			begin
				counter <= counter + 1;
				case (counter)
					
					`GNR_ACT:
					begin
						{cs_bar, ras_bar, cas_bar, we_bar} <= ACT;
						A[12:0] <= CMD_addr[22:10];
						BA <= CMD_addr[25:23];
					end
					
					`GNR_NOP1:
					begin
						{cs_bar, ras_bar, cas_bar, we_bar} <= NOP;
					end
					
					`GNR_READ:
					begin
						{cs_bar, ras_bar, cas_bar, we_bar} <= READ;
						A[10] <= 1; // auto precharge
						A[9:0] <= CMD_addr[9:0];
					end
					
					`GNR_NOP2:
					begin
						{cs_bar, ras_bar, cas_bar, we_bar} <= NOP;
					end
					
					`GNR_LSN:
					begin
						listen <= 1;						
					end
					
					`GNR_DATA:
					begin
						listen <= 0;
						read_pointer <= 0;
					end
					
					`GNR_DATA+1:
					begin
						RETURN_put <= 1;
						CMD_addr <= CMD_addr + 1;
					end
					
					`GNR_DATA+2, `GNR_DATA+3, `GNR_DATA+4, `GNR_DATA+5, `GNR_DATA+6, `GNR_DATA+7, `GNR_DATA+8:
					begin
						if (!BLOCK_cmd)
							RETURN_put <= 0;
						CMD_addr <= CMD_addr + 1;
						read_pointer <= read_pointer + 1;
					end
					
					`GNR_DATA+9:
					begin
						RETURN_put <= 0;
					end
					
					`GNR_DONE:
					begin
						if (CMD_sz == 0 || !BLOCK_cmd)
							state <= IDLE;
						else
						begin
							counter <= `GNR_ACT;
							CMD_sz <= CMD_sz - 1;
						end							
					end
					
				endcase
			end
			
			S_GNW:
			begin
				counter <= counter + 1;
				case (counter)
				
					`GNW_ACT:
					begin
						{cs_bar, ras_bar, cas_bar, we_bar} <= ACT;
						A[12:0] <= CMD_addr[22:10];
						BA <= CMD_addr[25:23];
					end
					
					`GNW_NOP1:
					begin
						{cs_bar, ras_bar, cas_bar, we_bar} <= NOP;
					end
				
					`GNW_WRITE:
					begin
						{cs_bar, ras_bar, cas_bar, we_bar} <= WRITE;
						A[10] <= 1; // auto precharge
						A[9:0] <= CMD_addr[9:0];
					end
					
					`GNW_NOP2:
					begin
						{cs_bar, ras_bar, cas_bar, we_bar} <= NOP;
					end
					
					`GNW_DQS:
					begin
						ts_con <= 1;
						DQS_out <= 0;
					end
					
					`GNW_GET:
					begin
						DATA_get <= 1;
						DQS_out <= ~DQS_out;
					end
					
					`GNW_DM:
					begin
						DQS_out <= ~DQS_out;
						if (CMD_addr[2:0] == 0)
							DM_flag <= 1;
						if (!BLOCK_cmd)
							DATA_get <= 0;
					end
					
					`GNW_DM+1, `GNW_DM+2, `GNW_DM+3, `GNW_DM+4, `GNW_DM+5, `GNW_DM+6:  
					begin
						DQS_out <= ~DQS_out;
						if (counter == `GNW_DM + CMD_addr[2:0])
							DM_flag <= 1;
						if (DM_flag && !BLOCK_cmd)
							DM_flag <= 0;
					end
					
					`GNW_DM+7:
					begin
						DATA_get <= 0;
						DQS_out <= ~DQS_out;
						if (counter == `GNW_DM + CMD_addr[2:0])
							DM_flag <= 1;
						if (DM_flag && !BLOCK_cmd)
							DM_flag <= 0;
					end
					
					`GNW_DM+8:
					begin
						DQS_out <= ~DQS_out;
						DM_flag <= 0;						
						CMD_addr <= CMD_addr + 8;
					end
					
					`GNW_TSB:
					begin
						ts_con <= 0;
					end
					
					`GNW_DONE:
					begin
						if (CMD_sz == 0 || !BLOCK_cmd)
							state <= IDLE;
						else
						begin
							counter <= `GNW_ACT;
							CMD_sz <= CMD_sz - 1;
						end	
					end				
					
				endcase
			end
			
			S_ATRW:
			begin
				counter <= counter + 1;
				case (counter)
				
					`ATRW_ACT:
					begin
						{cs_bar, ras_bar, cas_bar, we_bar} <= ACT;
						A[12:0] <= CMD_addr[22:10];
						BA <= CMD_addr[25:23];
					end
					
					`ATRW_NOP1:
					begin
						{cs_bar, ras_bar, cas_bar, we_bar} <= NOP;
					end
					
					`ATRW_READ:
					begin
						{cs_bar, ras_bar, cas_bar, we_bar} <= READ;
						A[10] <= 0; // NOT auto precharge
						A[9:0] <= CMD_addr[9:0];
					end
					
					`ATRW_NOP2:
					begin
						{cs_bar, ras_bar, cas_bar, we_bar} <= NOP;
					end
					
					`ATRW_WRITE:
					begin
						{cs_bar, ras_bar, cas_bar, we_bar} <= WRITE;
						A[10] <= 1;
					end
					
					`ATRW_NOP3:
					begin
						{cs_bar, ras_bar, cas_bar, we_bar} <= NOP;
					end
					
					
					`ATRW_LSN:
					begin
						listen <= 1;						
					end
					
					`ATRW_DATA:
					begin
						listen <= 0;
						read_pointer <= 0;
					end
					
					`ATRW_DATA+1:
					begin
						DATA_get <= 1;
						if (CMD_data_out[33:31] == ATR)
							RETURN_put <= 1;
					end
					
					`ATRW_DATA+2:
					//`ATRW_DATA+3, `ATRW_DATA+4, `ATRW_DATA+5, `ATRW_DATA+6, `ATRW_DATA+7, `ATRW_DATA+8:
					begin
						DATA_get <= 0;
						RETURN_put <= 0;
					end
					
					`ATRW_DQS:
					begin
						ts_con <= 1;
						DQS_out <= 0;
					end
					
					`ATRW_GET:
					begin
						DQS_out <= ~DQS_out;
					end
					
					`ATRW_DM:
					begin
						DQS_out <= ~DQS_out;
						if (CMD_addr[2:0] == 0)
							DM_flag <= 1;
					end
					
					`ATRW_DM+1, `ATRW_DM+2, `ATRW_DM+3, `ATRW_DM+4, `ATRW_DM+5, `ATRW_DM+6, `ATRW_DM+7, `ATRW_DM+8:  
					begin
						DQS_out <= ~DQS_out;
						if (counter == `ATRW_DM + CMD_addr[2:0])
							DM_flag <= 1;
						if (DM_flag)
							DM_flag <= 0;
					end
					
					`ATRW_TSB:
					begin
						ts_con <= 0;
					end
					
					`ATRW_DONE:
					begin
						state <= IDLE;
					end
				
				endcase
			end

		endcase
		
	  end
		
always @(CMD_op, RETURN_data, DATA_data_out)
begin
	case (CMD_op)
	
		3'b000: ATOMIC_data_in = ~RETURN_data;
		3'b001: ATOMIC_data_in = RETURN_data + DATA_data_out;
		3'b010: ATOMIC_data_in = RETURN_data | DATA_data_out;
		3'b011: ATOMIC_data_in = RETURN_data & DATA_data_out;
		3'b100: ATOMIC_data_in = RETURN_data ^ DATA_data_out;
		3'b101: ATOMIC_data_in = {RETURN_data[7:0], RETURN_data[15:8]};
		3'b110: ATOMIC_data_in = {1'b0,RETURN_data[15:1]};
		3'b111: ATOMIC_data_in = {RETURN_data[14:0], 1'b0};
	
	endcase
end

ddr3_ring_buffer8 ring_buffer(RETURN_data, listen, DQS_in, read_pointer[2:0], DQ_in, reset);


always @(negedge clk)
  begin
    DQ_out <= (ATOMIC_cmd)? ATOMIC_data : DATA_data_out;
    if(DM_flag)
        DM <= 2'b00;
    else
        DM <= 2'b11;
  end
 
endmodule // ddr_controller
