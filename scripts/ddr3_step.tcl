set design_name ddr3_controller ;
set clk_period 1.6;
set posedge 0.0;
set negedge [expr $clk_period * 0.5];

analyze -f verilog ./design/SSTL18DDR3.v;
analyze -f verilog ./design/SSTL18DDR3DIFF.v;
analyze -f verilog ./design/SSTL18DDR3INTERFACE.v;
analyze -f verilog ./design/FIFO.v;
analyze -f verilog ./design/ddr3_init_engine.v;
analyze -f verilog ./design/Processing_logic.v;
analyze -f verilog ./design/ddr3_ring_buffer8.v;

read_verilog ./design/ddr3_controller.v ;

set_dont_touch [ find cell XPL/ring_buffer/DELAY*]
get_attribute [ find cell XPL/ring_buffer/DELAY*] dont_touch

# set_dont_touch [ find cell process_logic_ddr2/clk_buffer2/DELAY*]
# get_attribute [ find cell process_logic_ddr2/clk_buffer2/DELAY*] dont_touch

# Setting $design_name as current working design.
# Use this command before setting any constraints.
current_design $design_name ;

#get_libs;
#insert_buffer [get_pins XPL/ring_buffer/r*_reg*/clocked_on] gscl45nm/BUFX4;
uniquify ;

link ;