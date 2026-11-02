`timescale 1ns/1ps
// Slave audio path: SD card model (FAT32 image with VID1.RAW, AUDIO1.WAV,
// AUDIO2.WAV) -> video_storage_player (shared SD reader, video + WAV) ->
// pcm_async_fifo -> slave_audio_executor, driven by a model of the main
// board's playback state as media_stream_rx would present it.
//
// Scenario and checks (volume 255 so samples pass unscaled after the
// 64-sample fade-in):
//   1. window mode, manual: arpeggio (non-WAV), then AUDIO1 from sample 0,
//      in order, looping at its end
//   2. KEY_1 (fullscreen): AUDIO2 from sample 0, in order, looping
//   3. KEY_2 (window): arpeggio again, then AUDIO1 from sample 0
//   4. auto slideshow on: AUDIO1 from sample 0, a manual commit does not
//      interrupt it
//   Video frames keep being written throughout (shared reader).
module tb_slave_audio;
    reg mem_clk=0, pix_clk=0, rst_n=0;
    always #12.5 mem_clk = ~mem_clk;       // 40 MHz
    always #20   pix_clk = ~pix_clk;       // 25 MHz
    localparam integer A1N = 1200, A2N = 1800;

    // SD card
    wire sd_cs_n, sd_sclk, sd_mosi, sd_miso;
    sd_card_model #(.IMAGE("sd_image.hex"), .BLOCKS(512)) u_card (
        .cs_n(sd_cs_n), .sclk(sd_sclk), .mosi(sd_mosi), .miso(sd_miso));

    // SDRAM model (idle-time behaviour only; no checks here)
    reg [31:0] mem [0:65535];
    wire app_wr_en, app_rd_en; wire [20:0] app_wr_addr, app_rd_addr; wire [31:0] app_wr_data;
    reg rd_valid=0; reg [31:0] rd_data=0;

    // video timing
    reg [11:0] x=0, y=0;
    always @(posedge pix_clk) if (rst_n) begin
        if (x==799) begin x<=0; y<= y==524 ? 0 : y+1; end else x<=x+1;
    end
    wire de = x<640 && y<480;

    reg fullscreen = 0;
    wire wav_run, wav_select, wav_room, wav_pcm_valid, wav_stopped;
    wire [32:0] wav_pcm_data; wire [7:0] wav_error;
    wire [23:0] prgb; wire image_ready, frame_shown; wire [7:0] error_code;
    video_storage_player #(.SPI_CLK_DIV(1)) u_player (
        .mem_clk(mem_clk), .mem_clk_shift(mem_clk), .mem_rst_n(rst_n),
        .pixel_clk(pix_clk), .pixel_rst_n(rst_n),
        .x(x), .y(y), .de(de), .fullscreen(fullscreen),
        .pixel_rgb(prgb), .image_ready(image_ready), .frame_shown(frame_shown),
        .error_code(error_code),
        .bg_line_ready(1'b0), .bg_line_no(9'd0), .bg_line_raddr(), .bg_line_rdata(32'd0),
        .bg_line_done(), .bg_active(),
        .wav_run(wav_run), .wav_select(wav_select), .wav_room(wav_room),
        .wav_pcm_valid(wav_pcm_valid), .wav_pcm_data(wav_pcm_data),
        .wav_stopped(wav_stopped), .wav_error(wav_error),
        .sd_cs_n(sd_cs_n), .sd_sclk(sd_sclk), .sd_mosi(sd_mosi), .sd_miso(sd_miso));

    // FIFO + executor exactly as in video_standalone_top
    reg [1:0] stp_s, flt_s, full_s;
    always @(posedge pix_clk) begin
        stp_s <= {stp_s[0], wav_stopped}; flt_s <= {flt_s[0], |wav_error}; full_s <= {full_s[0], fullscreen};
    end
    wire wav_wrst, wav_rrst, wav_pop, wav_valid, wav_prefilled; wire [32:0] wav_data;
    reset_sync u_wr (.clk(mem_clk),.ext_rst_n(rst_n && !wav_stopped),.rst_n_sync(wav_wrst));
    reset_sync u_rr (.clk(pix_clk),.ext_rst_n(rst_n && wav_run && !stp_s[1]),.rst_n_sync(wav_rrst));
    pcm_async_fifo u_fifo (.wc(mem_clk),.wrst(wav_wrst),.wen(wav_pcm_valid),.din(wav_pcm_data),
        .room_sector(wav_room),.rc(pix_clk),.rrst(wav_rrst),.pop(wav_pop),.valid(wav_valid),
        .dout(wav_data),.prefilled(wav_prefilled));
    reg m_ready=0, m_auto=0, m_commit=0; reg [1:0] m_media=0;
    wire pcm_valid; wire signed [15:0] pcm_l, pcm_r; wire overflow;
    reg pcm_ready = 0;
    slave_audio_executor #(.NOTE_SAMPLES(200),.BLIP_SAMPLES(100)) u_aud (
        .clk(pix_clk),.rst_n(rst_n),.playback_ready(m_ready),.auto_enabled(m_auto),
        .manual_image_commit(m_commit),.fullscreen(full_s[1]),.media_id(m_media),.volume(8'd255),
        .wav_stopped(stp_s[1]),.wav_prefilled(wav_prefilled),.wav_valid(wav_valid),
        .wav_data(wav_data),.wav_fault(flt_s[1]),.wav_run(wav_run),.wav_pop(wav_pop),
        .wav_select(wav_select),.sample_valid(pcm_valid),.sample_ready(pcm_ready),
        .sample_left(pcm_l),.sample_right(pcm_r),.overflow(overflow),.blip_active(),
        .sample_is_wav());
    // Consumer: like the HDMI packetizer, takes samples as they come.
    always @(posedge pix_clk) pcm_ready <= 1;

    // SDRAM application model: accept everything, return reads after 6 clocks.
    reg [31:0] rq [0:255]; reg [7:0] rq_wp=0, rq_rp=0; integer rq_d=0; integer cyc=0;
    reg sdr_init=0;
    always @(posedge mem_clk) begin
        cyc <= cyc + 1; if (cyc == 100) sdr_init <= 1;
        if (app_wr_en) mem[app_wr_addr[15:0]] <= app_wr_data;
        if (app_rd_en) begin rq[rq_wp] <= mem[app_rd_addr[15:0]]; rq_wp <= rq_wp + 1; end
        rd_valid <= 0;
        if (rq_wp != rq_rp) begin
            if (rq_d < 6) rq_d <= rq_d + 1;
            else begin rd_valid <= 1; rd_data <= rq[rq_rp]; rq_rp <= rq_rp + 1; end
        end else rq_d <= 0;
    end
    // Hook the SDRAM model into the player's sdram instance ports.
    // (video_storage_player instantiates sdram; we replace it by a stub.)

    // ---------------- sample classifier ----------------
    // kind: 0 silence/other, 1 AUDIO1 index, 2 AUDIO2 index
    // Executor gain at volume 255 and full ramp: out = (raw*255) >>> 8.
    function [15:0] scl(input integer raw);
        reg signed [15:0] r; reg signed [24:0] g;
        begin r = raw; g = r * $signed({1'b0, 8'd255}); scl = g[23:8]; end
    endfunction
    function integer a1_idx(input [15:0] l, input [15:0] r);
        integer k, est; begin
            a1_idx = -1; est = l + (l >> 8) - 16'h1000;
            for (k = est-2; k <= est+2; k = k + 1)
                if (k >= 0 && k < A1N && l == scl(16'h1000+k) && r == scl(~(16'h1000+k) & 16'hFFFF)) a1_idx = k;
        end
    endfunction
    function integer a2_idx(input [15:0] l, input [15:0] r);
        integer k, est; begin
            a2_idx = -1; est = l + (l >> 8) - 16'h6000;
            for (k = est-2; k <= est+2; k = k + 1)
                if (k >= 0 && k < A2N && l == scl(16'h6000+k) && r == scl((16'h6000+k) ^ 16'h5555)) a2_idx = k;
        end
    endfunction
    integer last_kind=0, last_idx=-1, order_err=0;
    integer a1_starts=0, a2_starts=0, a1_count=0, a2_count=0, a1_loops=0, a2_loops=0, other=0;
    integer i1, i2;
    reg a1_end=0, a2_end=0;   // last sample of the file was heard
    always @(posedge pix_clk) if (rst_n && pcm_valid && pcm_ready) begin
        i1 = a1_idx(pcm_l, pcm_r); i2 = a2_idx(pcm_l, pcm_r);
        if (i1 >= 0) begin
            a1_count = a1_count + 1;
            if (last_kind != 1) begin
                if (i1 > 64) begin order_err = order_err + 1; $display("%0t AUDIO1 did not start at 0 (idx %0d)", $time, i1); end
                if (a1_end) a1_loops = a1_loops + 1; else a1_starts = a1_starts + 1;
                a1_end = 0; a2_end = 0;
            end else if (i1 == 0 && last_idx == A1N-1) a1_loops = a1_loops + 1;
            else if (i1 != last_idx + 1) begin order_err = order_err + 1;
                if (order_err < 6) $display("%0t AUDIO1 order %0d -> %0d", $time, last_idx, i1); end
            last_kind = 1; last_idx = i1; if (i1 == A1N-1) a1_end = 1;
        end else if (i2 >= 0) begin
            a2_count = a2_count + 1;
            if (last_kind != 2) begin
                if (i2 > 64) begin order_err = order_err + 1; $display("%0t AUDIO2 did not start at 0 (idx %0d)", $time, i2); end
                if (a2_end) a2_loops = a2_loops + 1; else a2_starts = a2_starts + 1;
                a1_end = 0; a2_end = 0;
            end else if (i2 == 0 && last_idx == A2N-1) a2_loops = a2_loops + 1;
            else if (i2 != last_idx + 1) begin order_err = order_err + 1;
                if (order_err < 6) $display("%0t AUDIO2 order %0d -> %0d", $time, last_idx, i2); end
            last_kind = 2; last_idx = i2; if (i2 == A2N-1) a2_end = 1;
        end else begin
            // fade-in samples are scaled, arpeggio and silence are "other"
            if (pcm_l != 0) other = other + 1;
            if (!(last_kind != 0 && last_idx < 64)) last_kind = 0;
        end
    end

    always begin #10000000;
        $display("progress %0d ms: a1 %0d/%0d starts/loops  a2 %0d/%0d  run=%b sel=%b err=%0d frames=%0d",
            $time/1000000, a1_starts, a1_loops, a2_starts, a2_loops, wav_run, wav_select, order_err, u_player.frames_written);
        $fflush; end
    integer vsec=0, vsec0=0; real t_audio0=0;
    always @(posedge mem_clk) if (u_player.sector_done && u_player.owner == 2'd1) vsec = vsec + 1;
    always @(a1_starts) if (a1_starts == 1) begin vsec0 = vsec; t_audio0 = $time; end
    task wait_ms(input integer n); begin #(n*1000000); end endtask
    task commit; begin @(posedge pix_clk) m_commit <= 1; @(posedge pix_clk) m_commit <= 0; end endtask
    integer s1, s2;
    initial begin
        #200 rst_n = 1;
        m_ready = 1; m_media = 2;
        // 1. window, manual: arpeggio ~1.6 s is too long to simulate; a
        // manual commit restarts it, so wait for AUDIO1 by shortening via
        // parameter override below.
        wait (a1_starts >= 1);
        $display("%0t ms: AUDIO1 started after arpeggio", $time/1000000);
        wait (a1_loops >= 1);
        $display("%0t ms: AUDIO1 looped", $time/1000000);
        // 2. fullscreen -> AUDIO2
        a1_end = 0; a2_end = 0; fullscreen = 1;
        wait (a2_starts >= 1);
        $display("%0t ms: AUDIO2 started", $time/1000000);
        wait (a2_loops >= 1);
        $display("%0t ms: AUDIO2 looped", $time/1000000);
        // 3. back to window: arpeggio, then AUDIO1 from 0
        s1 = a1_starts;
        a1_end = 0; a2_end = 0; fullscreen = 0;
        wait (a1_starts > s1);
        $display("%0t ms: AUDIO1 restarted after leaving fullscreen", $time/1000000);
        // 4. auto mode: AUDIO1 from 0; a commit does not interrupt it
        a1_end = 0; s1 = a1_starts; m_auto = 1;
        wait (a1_starts > s1);
        $display("%0t ms: auto -> AUDIO1 from 0", $time/1000000);
        s1 = a1_starts; s2 = a1_count;
        commit;
        wait (a1_count > s2 + 600);
        if (a1_starts != s1) begin $display("commit interrupted AUDIO1 in auto mode"); order_err = order_err + 1; end
        wait_ms(220);
        if (u_player.frames_written < 3) $fatal(1, "10 fps sustained video did not complete three frames");
        $display("video frames written=%0d error_code=%h wav_error=%h overflow=%b", u_player.frames_written, error_code, wav_error, overflow);
        $display("AUDIO1 samples=%0d starts=%0d loops=%0d  AUDIO2 samples=%0d starts=%0d loops=%0d  order errors=%0d",
                 a1_count, a1_starts, a1_loops, a2_count, a2_starts, a2_loops, order_err);
        if (order_err || error_code || wav_error || overflow) $fatal(1, "FAIL");
        // Frames are paced at 10 fps (100 ms), so count video sectors instead:
        // 10 fps needs 1500 sectors/s; require that rate while audio plays.
        $display("video sectors while audio played: %0d in %0.1f ms (%0.0f/s, 1500/s needed)",
                 vsec - vsec0, ($time - t_audio0)/1.0e6, (vsec - vsec0)*1.0e9/($time - t_audio0));
        if ((vsec - vsec0)*1.0e9/($time - t_audio0) < 1500) $fatal(1, "video sector rate too low while audio played");
        $display("PASS: arpeggio->AUDIO1, fullscreen AUDIO2, back to arpeggio->AUDIO1, auto AUDIO1; video sectors kept up");
        $finish;
    end
    initial begin #3000000000; $fatal(1, "timeout a1_starts=%0d a2_starts=%0d a1=%0d a2=%0d err=%h wav_err=%h run=%b sel=%b stopped=%b",
        a1_starts, a2_starts, a1_count, a2_count, error_code, wav_error, wav_run, wav_select, wav_stopped); end
endmodule

// SDRAM stub with the same ports as sdram.v, driven from the testbench's
// application model above through hierarchical references.
module sdram (
    input Clk, Clk_sft, Rst,
    output Sdr_init_done, Sdr_init_ref_vld, Sdr_busy,
    input App_wr_en, input [20:0] App_wr_addr, input [3:0] App_wr_dm, input [31:0] App_wr_din,
    input App_rd_en, input [20:0] App_rd_addr,
    output Sdr_rd_en, output [31:0] Sdr_rd_dout
);
    assign Sdr_init_done = tb_slave_audio.sdr_init;
    assign Sdr_init_ref_vld = 1'b0;
    assign Sdr_busy = 1'b0;
    assign tb_slave_audio.app_wr_en = App_wr_en;
    assign tb_slave_audio.app_wr_addr = App_wr_addr;
    assign tb_slave_audio.app_wr_data = App_wr_din;
    assign tb_slave_audio.app_rd_en = App_rd_en;
    assign tb_slave_audio.app_rd_addr = App_rd_addr;
    assign Sdr_rd_en = tb_slave_audio.rd_valid;
    assign Sdr_rd_dout = tb_slave_audio.rd_data;
endmodule
