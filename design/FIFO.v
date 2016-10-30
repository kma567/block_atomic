
module FIFO (clk, reset, data_in, put, get, data_out, empty, full, fillcount);
parameter DEPTH_P2 = 5; // 2^ DEPTH_P2;
parameter WIDTH = 8;

output 	[WIDTH-1:0] 	data_out;
output 					empty, full;
output 	[DEPTH_P2:0] 	fillcount; //when it's full 1 more bit is needed

input 	[WIDTH-1:0] 	data_in;
input 					put, get;
input 					reset, clk;

reg						empty, full;
reg 	[DEPTH_P2:0] 	fillcount;
reg 	[WIDTH-1:0] 	fifo_array [2**DEPTH_P2-1:0];
reg		[WIDTH-1:0]		data_out;
reg 	[DEPTH_P2-1:0] 	rd_ptr;
reg 	[DEPTH_P2-1:0] 	wr_ptr;


always @ (posedge clk)
begin
	if(reset)
	begin
		rd_ptr <= 0;
		wr_ptr <= 0;
		empty <= 1;
		full <= 0;
		fillcount <= 0;
	end
	else
	begin
		if (put && !full && get && !empty)
		begin
			fifo_array[wr_ptr] <= data_in;
			data_out <= fifo_array[rd_ptr];
			wr_ptr <= wr_ptr + 1;
			rd_ptr <= rd_ptr + 1;
		end
		else if (put && !full)
		begin
			fifo_array[wr_ptr] <= data_in;
			wr_ptr <= wr_ptr + 1;
			fillcount <= fillcount + 1;
			
			if (empty)
				empty <= 0;				
			if (fillcount == 2**DEPTH_P2 - 1)
				full <= 1;
		end
		else if (get && !empty)
		begin
			data_out <= fifo_array[rd_ptr];
			rd_ptr <= rd_ptr + 1;
			fillcount <= fillcount - 1;
			
			if (full)
				full <= 0;
			if (fillcount == 1)
				empty <= 1;
		end
	end
end

endmodule