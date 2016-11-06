// Define Counter Values by setting the counter value for each stage in the INIT process
// There is a NOP command issued 1 DDR clock cycle after most commands
// More information about each stage is in the Verilog code below

`define T_WIDTH 2
`define T_XPR	76
`define T_MRD	6
`define T_MOD	30
`define T_ZQINIT  1024
`define S_RSTL	19'd0 // Set RST low
`define S_CKEL	`S_RSTL+125000  // Set CKE low
`define S_RSTH	`S_CKEL+10 // Set RST high
`define S_NOP1	`S_RSTH+312500 // Send NOP with CKE high
`define S_MRS2	`S_NOP1+`T_XPR // Set MR2
`define S_NOP2 	`S_MRS2+`T_WIDTH // NOP after MRS2
`define S_MRS3	`S_NOP2+`T_MRD // Set MR3 
`define S_NOP3	`S_MRS3+`T_WIDTH // NOP after MRS3
`define S_MRS1	`S_NOP3+`T_MRD // Set MR1 with DLL Set
`define S_NOP4	`S_MRS1+`T_WIDTH // NOP after DLL Set
`define S_MRS0	`S_NOP4+`T_MRD // Set MR0 with DLL Reset
`define S_NOP5	`S_MRS0+`T_WIDTH // NOP after DLL Reset
`define S_ZQCL	`S_NOP5+`T_MOD // Send ZQCL
`define S_NOP6	`S_ZQCL+`T_WIDTH // NOP after ZQCL
`define S_DONE	`S_NOP6+`T_ZQINIT // Memory is ready

/////////////////////////////////////////////////////////


module ddr3_init_engine(
   // Outputs
   ready, csbar, rasbar, casbar, webar, ba, a, odt, ts_con, cke, resetbar,
   // Inputs
   clk, reset, init, ck
   );


input clk, reset, init, ck;
output ready, csbar, rasbar, casbar, webar, odt, ts_con, cke, resetbar;
output [2:0] ba;
output [12:0] a;

reg ready;
reg cke;
reg csbar;
reg rasbar;
reg casbar;
reg webar;
reg resetbar;
reg [2:0] ba;
reg [12:0] a;
reg odt;
reg [15:0] dq_out;
reg [1:0] dqs_out;
reg [1:0] dqsbar_out;   
reg ts_con;
reg INIT, RESET;
   
reg [18:0]  counter;
reg flag;

localparam	[3:0]	
		NOP 		= 4'b0111,
		ZQCL	 	= 4'b0110,
		MRS			= 4'b0000;		
		
		
always @(posedge clk)
begin
	INIT <= init;
	RESET <= reset;

	if (RESET)
	begin
		flag <= 0;
		cke <= 0;
		odt <= 0;
		a <= 0;
		ba <= 0;
		ts_con <= 0;
		csbar <= 0;
		casbar <= 0;
		rasbar <= 0;
		webar <= 0;
		resetbar <= 0;
		ready <= 0;		
	end
	else if (flag == 0 && INIT == 1)
	begin
		// On INIT signal, set a flag to start the initialization routine and clear the counter
		flag <= 1;
		counter <= 0;
	end
	else if (flag == 1)
	begin
		counter <= counter + 1;
		case (counter)

		`S_RSTL:  begin
		resetbar <= 0; 
		end		
		`S_CKEL:  begin 
		cke <= 0;
		end
		
		`S_RSTH: begin 
			resetbar <= 1; 
			end
		`S_NOP1: begin
			cke <= 1;
			{csbar, rasbar, casbar, webar} <= NOP;
			end// NOP command

       	`S_MRS2: begin
                	{csbar, rasbar, casbar, webar} <= MRS; // EMRS command
					a[12:11] <= 0;
                	a[10:9] <= 0; // Dynamic ODT disabled
					a[8] <= 0;
					a[7:6] <= 0; // normal self-refresh
					a[5:3] <= 3'b000; // CAS write latency = 5
					a[2:0] <= 0;
					ba <= 3'b010;
	                end
		`S_NOP2: {csbar, rasbar, casbar, webar} <= NOP; // NOP command

        `S_MRS3: begin
        	        {csbar, rasbar, casbar, webar} <= MRS; // EMRS command 
        	        a <= 0;  // MPR disabled
        	        ba <= 3'b011;
	                end
        `S_NOP3: {csbar, rasbar, casbar, webar} <= NOP; // NOP command

        `S_MRS1: begin
        	        {csbar, rasbar, casbar, webar} <= MRS; // EMRS command 
					a[12] <= 0; // Output enabled
					a[11] <= 0; // TDQS enabled for 8 banks
					a[10] <= 0;
					{a[9], a[6], a[2]} <= 3'b000; // a[9,6,2] RTT disabled
					a[8] <= 0;
					a[7] <= 0; // write leveling disabled					 
					{a[5], a[1]} <= 2'b00; // a[5,1] Output driver
					a[4:3] <= 2'b10; // AL = CL - 2 = 3
					a[0] <= 0; // DLL enabled
					ba <= 3'b001;
	                end
        `S_NOP4: {csbar, rasbar, casbar, webar} <= NOP; // NOP command

        `S_MRS0: begin
        	        {csbar, rasbar, casbar, webar} <= MRS; // EMRS command 
        	        a[12] <= 0; // DLL off
               		a[11:9] <= 3'b010; // Write Recovery = 6
                	a[8] <= 1; // DLL Reset
					a[7] <= 0;
					a[6:4] <= 3'b001; // CAS latency = 5
					a[3] <= 0; // Read type = sequential
					a[2] <= 0;
					a[1:0] <= 2'b00; // Burst length = 8;
                	ba <= 3'b000; // MRS
	                end
        `S_NOP5: {csbar, rasbar, casbar, webar} <= NOP; // NOP command


		`S_ZQCL: begin
				{csbar, rasbar, casbar, webar} <= ZQCL; 
				a[10] <= 1; 
				end
		`S_NOP6: {csbar, rasbar, casbar, webar} <= NOP; // NOP command

				
       	`S_DONE: begin
				{csbar, rasbar, casbar, webar} <= NOP; // Finally done - Just send NOPs
				ready <= 1; // done
				end
		
		default: begin
			flag <= 1;
                end
	endcase
	end
end

endmodule
