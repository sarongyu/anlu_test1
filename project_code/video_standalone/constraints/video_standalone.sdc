create_clock -name sys_clk -period 20.000 -waveform {0.000 10.000} [get_ports {sys_clk}]
create_clock -name eth_rx_clk -period 8.000 -waveform {0.000 4.000} [get_ports {eth_rx_clk}]
create_generated_clock -name eth_clk -source [get_ports {sys_clk}] -master_clock sys_clk -multiply_by 2.5 [get_pins {u_eth_pll/pll_inst.clkc[0]}]
create_generated_clock -name eth_clk90 -source [get_ports {sys_clk}] -master_clock sys_clk -multiply_by 2.5 -phase 90 [get_pins {u_eth_pll/pll_inst.clkc[1]}]
create_generated_clock -name pixel_clk_5x -source [get_ports {sys_clk}] -master_clock sys_clk -multiply_by 2.5 [get_nets {pixel_clk_5x}]
create_generated_clock -name pixel_clk -source [get_ports {sys_clk}] -master_clock sys_clk -divide_by 2 [get_nets {pixel_clk}]
create_generated_clock -name mem_clk -source [get_ports {sys_clk}] -master_clock sys_clk -divide_by 1.25 -phase 0 [get_pins {u_mem_pll/pll_inst.clkc[1]}]
create_generated_clock -name mem_clk_shift -source [get_ports {sys_clk}] -master_clock sys_clk -divide_by 1.25 -phase 172.8 [get_pins {u_mem_pll/pll_inst.clkc[2]}]
set_false_path -from [get_ports {key4_n key1_n key2_n}]
# Frame store: toggles with 3-flop synchronizers; ready_bank/req_* are
# stable for at least a line when sampled.
set_false_path -to [get_regs -hier {u_video/u_frames/ack_meta[0] u_video/u_frames/ready_meta[0] u_video/u_frames/req_meta[0] u_video/u_frames/full_meta[0] u_video/u_frames/bgact_meta[0]}]
set_false_path -from [get_regs -hier {u_video/u_frames/ready_bank u_video/u_frames/req_row[*] u_video/u_frames/req_bgline[*] u_video/u_frames/req_slot u_video/u_frames/req_bank u_video/u_frames/req_vid u_video/u_frames/req_bg u_video/u_frames/req_first}]
set_false_path -to [get_regs -hier {u_shown_sync/s1 u_bg_sync/s1 u_fault_sync/s1 u_full_pix/s1 u_wav_stopped/s1 u_wav_fault/s1}]
set_false_path -to [get_regs -hier {u_wav_wreset/s1 u_wav_wreset/s2 u_wav_rreset/s1 u_wav_rreset/s2}]
set_false_path -to [get_regs -hier {u_phy/symbols_sync1[*]}]
set_false_path -to [get_regs -hier {u_phy/toggle_sync1}]
set_max_delay -from [get_clocks {pixel_clk_5x}] -to [get_ports {HDMI_CLK_P HDMI_D0_P HDMI_D1_P HDMI_D2_P}] 8.000 -datapath_only

# RGMII command link. Each domain crossing is a toggle (or a hold register
# that is stable for the whole frame) followed by a 3-flop synchronizer.
set_false_path -to [get_regs -hier {u_eth/bad_meta[0]}]
set_false_path -to [get_regs -hier {u_eth/done_meta[0]}]
set_false_path -to [get_regs -hier {u_eth/send_meta[0]}]
set_false_path -from [get_regs -hier {u_eth/command_hold[*] u_eth/sequence_hold[*]}]
# Stream receiver: line slots (rx_clk -> mem_clk) and audio FIFO (rx_clk ->
# pix_clk). Each crossing is a toggle or Gray pointer with a 3-flop / 2-flop
# synchronizer; slot contents and slot_line are stable while committed.
set_false_path -to [get_regs -hier {u_stream/commit_meta[0] u_stream/free_meta[0] u_stream/st_meta[0]}]
set_false_path -from [get_regs -hier {u_stream/slot_line* u_stream/status_hold[*]}]
set_false_path -from [get_clocks {eth_rx_clk}] -to [get_clocks {mem_clk}]
set_false_path -from [get_clocks {eth_rx_clk}] -to [get_clocks {pixel_clk}]
set_false_path -from [get_clocks {pixel_clk}] -to [get_clocks {eth_rx_clk}]
# WAV reader (mem_clk) -> PCM FIFO -> audio executor (pixel_clk), as on the
# main board: Gray pointers bounded by one source period, level signals.
set_false_path -to [get_regs -hier {u_video/u_wav/run_s1 u_video/sel_meta[0] u_wav_fifo/end_s1}]
set_max_delay -from [get_regs -hier {u_wav_fifo/wg[*]}] -to [get_regs -hier {u_wav_fifo/wg_s1[*]}] 25.000 -datapath_only
set_max_delay -from [get_regs -hier {u_wav_fifo/rg[*]}] -to [get_regs -hier {u_wav_fifo/rg_s1[*]}] 40.000 -datapath_only
set_clock_groups -asynchronous -group [get_clocks {eth_rx_clk}]
