create_clock -name sys_clk -period 20.000 -waveform {0.000 10.000} [get_ports {sys_clk}]
create_clock -name eth_rx_clk -period 8.000 -waveform {0.000 4.000} [get_ports {eth_rx_clk}]
create_generated_clock -name eth_clk -source [get_ports {sys_clk}] -master_clock sys_clk -multiply_by 2.5 [get_pins {u_eth_pll/pll_inst.clkc[0]}]
create_generated_clock -name eth_clk90 -source [get_ports {sys_clk}] -master_clock sys_clk -multiply_by 2.5 -phase 90 [get_pins {u_eth_pll/pll_inst.clkc[1]}]
create_generated_clock -name pixel_clk_5x -source [get_ports {sys_clk}] -master_clock sys_clk -multiply_by 2.5 [get_nets {pixel_clk_5x}]
create_generated_clock -name pixel_clk -source [get_ports {sys_clk}] -master_clock sys_clk -divide_by 2 [get_nets {pixel_clk}]
create_generated_clock -name mem_clk -source [get_ports {sys_clk}] -master_clock sys_clk -divide_by 1.25 -phase 0 [get_pins {u_mem_pll/pll_inst.clkc[1]}]
create_generated_clock -name mem_clk_shift -source [get_ports {sys_clk}] -master_clock sys_clk -divide_by 1.25 -phase 172.8 [get_pins {u_mem_pll/pll_inst.clkc[2]}]

set_false_path -from [get_ports {sys_rst_n}]
set_false_path -to [get_regs -hier {*/s1}]
set_false_path -to [get_regs -hier {*/request_sync1_mem}]
set_false_path -to [get_regs -hier {*/index_sync1_mem[*]}]
set_false_path -to [get_regs -hier {*/ack_sync1}]
set_false_path -to [get_regs -hier {*/req_sync1}]
set_false_path -to [get_regs -hier {*/index_sync1[*]}]
set_false_path -to [get_regs -hier {*busy_sync1}]
set_false_path -to [get_regs -hier {*commit_sync1}]

# Audio commands are stable levels; sector completion acknowledges stop.
set_false_path -to [get_regs -hier {u_storage_ingest/u_wav/run_s1}]
set_false_path -to [get_regs -hier {u_video/u_control/manual_s1}]
set_false_path -to [get_regs -hier {u_wav_fifo/end_s1}]
# Gray bus crossings are bounded by their source clock periods.
set_max_delay -from [get_regs -hier {u_wav_fifo/wg[*]}] -to [get_regs -hier {u_wav_fifo/wg_s1[*]}] 25.000 -datapath_only
set_max_delay -from [get_regs -hier {u_wav_fifo/rg[*]}] -to [get_regs -hier {u_wav_fifo/rg_s1[*]}] 40.000 -datapath_only

# The inherited FIFO uses Gray pointers. Limit each crossing to one source
# clock period instead of declaring the complete pixel/memory domains false.
set_max_delay -from [get_regs -hier {u_video/u_reader/u_read_fifo/wr_to_rd_cross_inst/primary_addr_gray_reg[*]}] -to [get_regs -hier {u_video/u_reader/u_read_fifo/wr_to_rd_cross_inst/sync_r1[*]}] 25.000 -datapath_only
set_max_delay -from [get_regs -hier {u_video/u_reader/u_read_fifo/rd_to_wr_cross_inst/primary_addr_gray_reg[*]}] -to [get_regs -hier {u_video/u_reader/u_read_fifo/rd_to_wr_cross_inst/sync_r1[*]}] 40.000 -datapath_only


# RGMII command link. Each domain crossing is a toggle (or a hold register
# that is stable for the whole frame) followed by a 3-flop synchronizer.
set_false_path -to [get_regs -hier {u_eth/good_meta[0]}]
set_false_path -to [get_regs -hier {u_eth/bad_meta[0]}]
set_false_path -to [get_regs -hier {u_eth/act_meta[0]}]
set_false_path -to [get_regs -hier {u_eth/done_meta[0]}]
set_false_path -to [get_regs -hier {u_eth/send_meta[0]}]
set_false_path -from [get_regs -hier {u_eth/rx_cmd_hold[*] u_eth/rx_seq_hold[*] u_eth/rx_packet_count[*]}]
set_false_path -from [get_regs -hier {u_eth/command_hold[*] u_eth/sequence_hold[*]}]
# Stream transmitter (pixel_clk -> eth_clk): line/audio halves are handed
# over by toggles with 3-flop synchronizers and read only while full.
set_false_path -to [get_regs -hier {u_stream_tx/line_meta[0] u_stream_tx/st_meta[0] u_stream_tx/drop_meta[0] u_stream_tx/lfree_meta[0]}]
set_false_path -from [get_regs -hier {u_stream_tx/line_tag* u_stream_tx/status_hold[*]}]
set_false_path -to [get_regs -hier {u_eth/strm_meta[0] u_eth/txany_meta[0]}]
set_false_path -from [get_clocks {pixel_clk}] -to [get_clocks {eth_clk}]
set_clock_groups -asynchronous -group [get_clocks {eth_rx_clk}]
