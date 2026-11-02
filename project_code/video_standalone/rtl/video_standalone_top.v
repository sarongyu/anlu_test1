`timescale 1ns/1ps

// Second board, plan A: final HDMI_A output of the two-board player.
// Plays VID1.RAW (320x240 RGB332, 10 fps, looping) from its own TF card
// through SDRAM and composes it with the main board's finished picture,
// which arrives over Ethernet (rgmii_command_link STREAM_RX):
//   KEY_2 / power-up: main-board picture fullscreen, local video in a
//                     160x120 window at the top right;
//   KEY_1:            local video fullscreen 640x480.
// Audio is produced here from this board's TF card: in window mode the
// main board's rules (image arpeggio, then AUDIO1.WAV; auto slideshow plays
// AUDIO1.WAV) driven by the playback state the main board sends every 1 ms;
// in fullscreen AUDIO2.WAV, looping. KEY_1/KEY_2 are also sent to the main
// board as commands (it ACKs).
module video_standalone_top (
    input wire sys_clk, sd_miso,
    input wire key1_n, key2_n, key4_n,
    input wire eth_rx_clk, eth_rx_ctl,
    input wire [3:0] eth_rx_d,
    output wire eth_tx_clk, eth_tx_ctl,
    output wire [3:0] eth_tx_d,
    output wire sd_cs_n, sd_sclk, sd_mosi,
    output wire HDMI_CLK_P, HDMI_D2_P, HDMI_D1_P, HDMI_D0_P,
    output wire [3:0] led
);
    // The original reset was KEY1; KEY4 is the dedicated manual reset now.
    wire sys_rst_n = key4_n;
    wire pixel_clk, pixel_clk_5x, video_locked;
    wire mem_clk, mem_clk_shift, feedback_unused, mem_locked;
    wire mem_rst_n, pix_rst_n, pix5_rst_n;
    clk_wize u_video_pll (.refclk(sys_clk),.reset(!sys_rst_n),
        .clk0_out(pixel_clk_5x),.clk1_out(pixel_clk),.locked(video_locked));
    sys_pll u_mem_pll (.refclk(sys_clk),.reset(!sys_rst_n),
        .clk0_out(feedback_unused),.clk1_out(mem_clk),
        .clk2_out(mem_clk_shift),.locked(mem_locked));
    reset_sync u_mem_reset (.clk(mem_clk),.ext_rst_n(sys_rst_n && mem_locked),.rst_n_sync(mem_rst_n));
    reset_sync u_pix_reset (.clk(pixel_clk),.ext_rst_n(sys_rst_n && video_locked),.rst_n_sync(pix_rst_n));
    reset_sync u_pix5_reset (.clk(pixel_clk_5x),.ext_rst_n(sys_rst_n && video_locked),.rst_n_sync(pix5_rst_n));

    wire hs, vs, de;
    wire [11:0] x, y;
    video_timing_ctrl u_timing (.pclk(pixel_clk),.rst_n(pix_rst_n),
        .hsync(hs),.vsync(vs),.de(de),.x(x),.y(y));

    // ------------------------------------------------------------------
    // Ethernet: commands out, master picture/audio in
    // ------------------------------------------------------------------
    wire net_send,net_ready,net_received,net_activity;
    wire [7:0] net_tx_cmd,net_tx_seq,net_rx_cmd,net_rx_seq;
    wire [15:0] net_packets;
    wire net_fullscreen,net_live,net_ack;
    wire [7:0] net_last_seq;
    wire net_led_rx,net_led_good,net_led_bad,net_led_tx,net_led_stream;
    // RGMII TX clocks: 125 MHz 0 deg for data, 90 deg for TXC (vendor example).
    // The PHY has no FPGA reset pin.
    wire eth_clk, eth_clk90, eth_locked;
    rgmii_tx_pll u_eth_pll (.refclk(sys_clk),.reset(!sys_rst_n),
        .extlock(eth_locked),.clk0_out(eth_clk),.clk90_out(eth_clk90));
    wire net_rst_n = sys_rst_n && eth_locked;
    wire s_rst_n, s_valid, s_audio, s_commit, s_abort;
    wire [7:0] s_byte;
    wire [10:0] s_index;
    wire [8:0] s_tag;
    wire [23:0] s_status;
    rgmii_command_link #(.LOCAL_ID(8'h02),.PEER_ID(8'h01),.STREAM_RX(1)) u_eth (
        .sys_clk(sys_clk),.rst_n(net_rst_n),
        .tx_clk(eth_clk),.tx_clk90(eth_clk90),.rx_clk(eth_rx_clk),
        .rx_d(eth_rx_d),.rx_ctl(eth_rx_ctl),
        .tx_d(eth_tx_d),.tx_ctl(eth_tx_ctl),.tx_clk_o(eth_tx_clk),
        .send(net_send),.send_cmd(net_tx_cmd),.send_seq(net_tx_seq),
        .send_ready(net_ready),.got_cmd(net_received),
        .got_data(net_rx_cmd),.got_seq(net_rx_seq),
        .packet_count(net_packets),.activity(net_activity),
        .led_rx_activity(net_led_rx),.led_rx_good(net_led_good),
        .led_rx_bad(net_led_bad),.led_tx(net_led_tx),.led_stream(net_led_stream),
        .blk_ready(1'b0),.blk_audio(1'b0),.blk_line(9'd0),.blk_status(24'd0),
        .blk_take(),.blk_done(),.blk_raddr(),.blk_rdata(32'd0),
        .s_rst_n(s_rst_n),.s_valid(s_valid),.s_audio(s_audio),.s_byte(s_byte),
        .s_index(s_index),.s_tag(s_tag),.s_status(s_status),
        .s_commit(s_commit),.s_abort(s_abort));
    net_command_policy #(.SLAVE(1)) u_net_policy (
        .clk(sys_clk),.rst_n(net_rst_n),
        .key1_n(key1_n),.key2_n(key2_n),
        .rx_pulse(net_received),.rx_cmd(net_rx_cmd),.rx_seq(net_rx_seq),
        .tx_ready(net_ready),.tx_send(net_send),
        .tx_cmd(net_tx_cmd),.tx_seq(net_tx_seq),
        .fullscreen(net_fullscreen),.link_alive(net_live),
        .last_sequence(net_last_seq),.acknowledged(net_ack));

    wire bg_line_ready, bg_line_done, bg_active;
    wire [8:0] bg_line_no, bg_line_raddr;
    wire [31:0] bg_line_rdata;
    wire m_ready, m_auto, m_commit;
    wire [1:0] m_media;
    wire [7:0] m_volume;
    wire [1:0] m_brightness, m_period;
    wire m_settings_valid;
    wire [15:0] lines_dropped;
    media_stream_rx u_stream (
        .rx_clk(eth_rx_clk), .rx_rst_n(s_rst_n),
        .s_valid(s_valid), .s_audio(s_audio), .s_byte(s_byte),
        .s_index(s_index), .s_tag(s_tag), .s_status(s_status),
        .s_commit(s_commit), .s_abort(s_abort),
        .mem_clk(mem_clk), .mem_rst_n(mem_rst_n),
        .line_ready(bg_line_ready), .line_no(bg_line_no), .line_take(1'b0),
        .line_raddr(bg_line_raddr), .line_rdata(bg_line_rdata), .line_done(bg_line_done),
        .pix_clk(pixel_clk), .pix_rst_n(pix_rst_n),
        .m_ready(m_ready), .m_auto(m_auto), .m_media(m_media),
        .m_volume(m_volume), .m_commit(m_commit),
        .m_brightness(m_brightness),.m_period(m_period),.m_settings_valid(m_settings_valid),
        .lines_dropped(lines_dropped));

    // ------------------------------------------------------------------
    // Audio: arpeggio / AUDIO1 / AUDIO2 from this board's TF card
    // ------------------------------------------------------------------
    wire wav_run, wav_select, wav_room, wav_pcm_valid, wav_stopped;
    wire [32:0] wav_pcm_data;
    wire [7:0] wav_error;
    wire full_pix;
    sync_2ff u_full_pix (.clk(pixel_clk),.rst_n(pix_rst_n),.async_in(net_fullscreen),.sync_out(full_pix));
    wire wav_stopped_pix, wav_fault_pix;
    sync_2ff u_wav_stopped (.clk(pixel_clk),.rst_n(pix_rst_n),.async_in(wav_stopped),.sync_out(wav_stopped_pix));
    sync_2ff u_wav_fault (.clk(pixel_clk),.rst_n(pix_rst_n),.async_in(|wav_error),.sync_out(wav_fault_pix));
    wire wav_wrst, wav_rrst, wav_pop, wav_valid, wav_prefilled;
    wire [32:0] wav_data;
    reset_sync u_wav_wreset (.clk(mem_clk),.ext_rst_n(mem_rst_n && !wav_stopped),.rst_n_sync(wav_wrst));
    reset_sync u_wav_rreset (.clk(pixel_clk),.ext_rst_n(pix_rst_n && wav_run && !wav_stopped_pix),.rst_n_sync(wav_rrst));
    pcm_async_fifo u_wav_fifo (.wc(mem_clk),.wrst(wav_wrst),
        .wen(wav_pcm_valid),.din(wav_pcm_data),.room_sector(wav_room),
        .rc(pixel_clk),.rrst(wav_rrst),
        .pop(wav_pop),.valid(wav_valid),.dout(wav_data),.prefilled(wav_prefilled));
    // Fullscreen uses full volume; window mode uses the main board's volume.
    wire [7:0] audio_volume = full_pix ? 8'd255 : m_volume;
    wire pcm_valid, pcm_ready;
    wire signed [15:0] pcm_left, pcm_right;
    wire audio_overflow;
    slave_audio_executor u_audio (
        .clk(pixel_clk),.rst_n(pix_rst_n),
        .playback_ready(m_ready),.auto_enabled(m_auto),.manual_image_commit(m_commit),
        .fullscreen(full_pix),.media_id(m_media),.volume(audio_volume),
        .wav_stopped(wav_stopped_pix),.wav_prefilled(wav_prefilled),
        .wav_valid(wav_valid),.wav_data(wav_data),.wav_fault(wav_fault_pix),
        .wav_run(wav_run),.wav_pop(wav_pop),.wav_select(wav_select),
        .sample_valid(pcm_valid),.sample_ready(pcm_ready),
        .sample_left(pcm_left),.sample_right(pcm_right),
        .overflow(audio_overflow),.blip_active(),.sample_is_wav());

    // ------------------------------------------------------------------
    // Local video + composite
    // ------------------------------------------------------------------
    wire [23:0] pixel_rgb;
    wire image_ready, frame_shown;
    wire [7:0] error_code;
    video_storage_player u_video (.mem_clk(mem_clk),.mem_clk_shift(mem_clk_shift),
        .mem_rst_n(mem_rst_n),.pixel_clk(pixel_clk),.pixel_rst_n(pix_rst_n),
        .x(x),.y(y),.de(de),.fullscreen(net_fullscreen),
        .pixel_rgb(pixel_rgb),.image_ready(image_ready),.frame_shown(frame_shown),
        .error_code(error_code),
        .bg_line_ready(bg_line_ready),.bg_line_no(bg_line_no),
        .bg_line_raddr(bg_line_raddr),.bg_line_rdata(bg_line_rdata),
        .bg_line_done(bg_line_done),.bg_active(bg_active),
        .wav_run(wav_run),.wav_select(wav_select),.wav_room(wav_room),
        .wav_pcm_valid(wav_pcm_valid),.wav_pcm_data(wav_pcm_data),
        .wav_stopped(wav_stopped),.wav_error(wav_error),
        .sd_cs_n(sd_cs_n),.sd_sclk(sd_sclk),.sd_mosi(sd_mosi),.sd_miso(sd_miso));

    // The compositor output follows x/y/de by two clocks; delay the timing
    // bundle by the same amount and register once more with the colour.
    reg [9:0] x_q1, x_q2, x_q3;
    reg [9:0] y_q1, y_q2;
    wire [23:0] status_rgb;
    slave_status_osd u_status_osd (
        .clk(pixel_clk),.rst_n(pix_rst_n),.de(de_q2),
        .x(x_q2),.y(y_q2),.rgb_in(pixel_rgb),
        .fullscreen(full_pix),.auto_enabled(m_auto),.master_ready(m_ready),
        .volume(m_volume),.brightness(m_brightness),.period(m_period),
        .settings_valid(m_settings_valid),.rgb_out(status_rgb));
    reg hs_q1, vs_q1, de_q1, hs_q2, vs_q2, de_q2, hs_q3, vs_q3, de_q3;
    reg [23:0] rgb_q;
    always @(posedge pixel_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            x_q1 <= 0; x_q2 <= 0; x_q3 <= 0;
            y_q1 <= 0; y_q2 <= 0;
            hs_q1 <= 1; hs_q2 <= 1; hs_q3 <= 1;
            vs_q1 <= 1; vs_q2 <= 1; vs_q3 <= 1;
            de_q1 <= 0; de_q2 <= 0; de_q3 <= 0; rgb_q <= 0;
        end else begin
            x_q1 <= x[9:0]; x_q2 <= x_q1; x_q3 <= x_q2;
            y_q1 <= y[9:0]; y_q2 <= y_q1;
            hs_q1 <= hs; hs_q2 <= hs_q1; hs_q3 <= hs_q2;
            vs_q1 <= vs; vs_q2 <= vs_q1; vs_q3 <= vs_q2;
            de_q1 <= de; de_q2 <= de_q1; de_q3 <= de_q2;
            rgb_q <= status_rgb;
        end
    end
    wire [9:0] symbol0, symbol1, symbol2;
    wire hdmi_seq_error, hdmi_phase_error;
    hdmi_audio_symbol_core u_hdmi (.pixel_clk(pixel_clk),.rst_n(pix_rst_n),
        .video_hsync(hs_q3),.video_vsync(vs_q3),.video_de(de_q3),
        .video_x(x_q3),.video_rgb(rgb_q),.sample_valid(pcm_valid),
        .sample_ready(pcm_ready),.sample_left(pcm_left),.sample_right(pcm_right),
        .channel0_symbol(symbol0),.channel1_symbol(symbol1),
        .channel2_symbol(symbol2),.sequence_error(hdmi_seq_error));
    hdmi_tmds_eg4_phy u_phy (.pixel_clk(pixel_clk),.pixel_rst_n(pix_rst_n),
        .clk_5x(pixel_clk_5x),.clk_5x_rst_n(pix5_rst_n),
        .channel0_symbol(symbol0),.channel1_symbol(symbol1),
        .channel2_symbol(symbol2),.tmds_clock_p(HDMI_CLK_P),
        .tmds_data0_p(HDMI_D0_P),.tmds_data1_p(HDMI_D1_P),.tmds_data2_p(HDMI_D2_P),
        .phase_error(hdmi_phase_error));

    // LEDs (silkscreen LEDn+1 = led[n]):
    //   led[0] toggles on every displayed video frame (2.5 Hz at 5 fps)
    //   led[1] main-board picture is being received and shown
    //   led[2] fullscreen mode
    //   led[3] steady: video, WAV or HDMI error; blinking: bad Ethernet frame
    wire shown_sys, bg_sys;
    sync_2ff u_shown_sync (.clk(sys_clk),.rst_n(sys_rst_n),.async_in(frame_shown),.sync_out(shown_sys));
    sync_2ff u_bg_sync (.clk(sys_clk),.rst_n(sys_rst_n),.async_in(bg_active),.sync_out(bg_sys));
    wire fault = error_code != 0 || hdmi_seq_error || hdmi_phase_error ||
                 audio_overflow || wav_fault_pix;
    wire fault_sys;
    sync_2ff u_fault_sync (.clk(sys_clk),.rst_n(sys_rst_n),.async_in(fault),.sync_out(fault_sys));
    assign led = {fault_sys || net_led_bad, net_fullscreen, bg_sys, shown_sys};
endmodule
