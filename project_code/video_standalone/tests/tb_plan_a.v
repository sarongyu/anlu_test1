`timescale 1ns/1ps
// Plan A end to end: a master picture/audio source -> media_stream_tx ->
// rgmii_command_link (master) -> wire -> rgmii_command_link (slave) ->
// media_stream_rx -> video_sdram_frames + SD video source -> composite.
//
// The master runs its pixel clock 0.1% fast against the slave, so the
// background frame selection has to cope with drift. Checks:
//   * window mode: every background pixel equals the master pattern of the
//     master frame that was selected for display (whole frame, one frame
//     only: no tearing), and the 160x120 window shows the video;
//   * fullscreen: every pixel is the video;
//   * audio: the slave plays the master's sample sequence in order.
module tb_plan_a;
    parameter integer H_TOTAL = 800, V_TOTAL = 525;
    // Shortened frames keep the run time reasonable; widths stay real.
    reg mem_clk=0, pix_s=0, pix_m=0, clk50=0, clk125=0, clk125_90=0, rst_n=0;
    always #12.5 mem_clk=~mem_clk;          // 40 MHz
    always #20   pix_s=~pix_s;              // slave 25 MHz
    always #19.98 pix_m=~pix_m;             // master 0.1% fast
    always #10   clk50=~clk50;
    initial begin #0.3; forever #4 clk125=~clk125; end
    initial begin #2.3; forever #4 clk125_90=~clk125_90; end

    // ---------------- master picture + audio source ----------------
    reg [11:0] mx=0, my=0;
    always @(posedge pix_m) if (rst_n) begin
        if (mx==H_TOTAL-1) begin mx<=0; my<= my==V_TOTAL-1 ? 0 : my+1; end else mx<=mx+1;
    end
    integer mframe=0;
    always @(posedge pix_m) if (rst_n && mx==H_TOTAL-1 && my==V_TOTAL-1) mframe=mframe+1;
    wire m_de = mx<640 && my<480;
    wire m_vs = !(my>=490 && my<492);
    function [15:0] mpix(input integer f, input integer px, input integer py);
        mpix = (f*4099 + px*37 + py*1013) & 16'hFFFF;
    endfunction
    reg [23:0] m_rgb; reg m_de_q, m_vs_q; reg [11:0] mx_q, my_q;
    always @(posedge pix_m) begin
        m_de_q<=m_de; m_vs_q<=m_vs; mx_q<=mx; my_q<=my;
        // RGB565 expanded so that the 565 packing recovers it exactly
        begin : mk
            reg [15:0] p; p = mpix(mframe, mx, my);
            m_rgb <= {p[15:11],3'b0, p[10:5],2'b0, p[4:0],3'b0};
        end
    end
    // audio: one sample per 520.8 master clocks (48 kHz at 25 MHz)
    reg [24:0] macc=0; reg m_pcm_valid=0; reg [15:0] m_pcm_n=0;
    always @(posedge pix_m) begin
        m_pcm_valid<=0;
        if (rst_n) begin
            if (macc>=25000000-48000) begin macc<=macc+48000-25000000; m_pcm_valid<=1; m_pcm_n<=m_pcm_n+1; end
            else macc<=macc+48000;
        end
    end

    // ---------------- master side ----------------
    wire blk_ready, blk_audio, blk_take, blk_done; wire [8:0] blk_line, blk_raddr; wire [31:0] blk_rdata;
    wire [23:0] blk_status; reg [5:0] m_commits = 0;
    wire [15:0] m_dropped;
    media_stream_tx u_mtx (
        .pix_clk(pix_m), .pix_rst_n(rst_n),
        .vid_vs(m_vs_q), .vid_de(m_de_q), .vid_rgb(m_rgb),
        .status_in({8'd200, m_commits, 2'd2, 6'd0, 1'b1, 1'b1}),
        .tx_clk(clk125), .tx_rst_n(u_mlink.tx_rst_n),
        .blk_ready(blk_ready), .blk_audio(blk_audio), .blk_line(blk_line),
        .blk_status(blk_status),
        .blk_take(blk_take), .blk_raddr(blk_raddr), .blk_rdata(blk_rdata),
        .blk_done(blk_done), .lines_dropped(m_dropped));
    wire [3:0] m_txd, s_txd; wire m_txctl, s_txctl, m_txc, s_txc;
    wire [3:0] s_rxd, m_rxd; wire s_rxctl, m_rxctl, s_rxc, m_rxc;
    assign #1.0 s_rxd = m_txd; assign #1.0 s_rxctl = m_txctl; assign #1.0 s_rxc = m_txc;
    assign #1.0 m_rxd = s_txd; assign #1.0 m_rxctl = s_txctl; assign #1.0 m_rxc = s_txc;
    rgmii_command_link #(.LOCAL_ID(8'h01),.PEER_ID(8'h02),.STRETCH(24'd20),.STREAM_TX(1)) u_mlink (
        .sys_clk(clk50),.rst_n(rst_n),.tx_clk(clk125),.tx_clk90(clk125_90),
        .rx_clk(m_rxc),.rx_d(m_rxd),.rx_ctl(m_rxctl),
        .tx_d(m_txd),.tx_ctl(m_txctl),.tx_clk_o(m_txc),
        .send(1'b0),.send_cmd(8'h0),.send_seq(8'h0),.send_ready(),
        .got_cmd(),.got_data(),.got_seq(),.packet_count(),.activity(),
        .led_rx_activity(),.led_rx_good(),.led_rx_bad(),.led_tx(),.led_stream(),
        .blk_ready(blk_ready),.blk_audio(blk_audio),.blk_line(blk_line),.blk_status(blk_status),
        .blk_take(blk_take),.blk_done(blk_done),.blk_raddr(blk_raddr),.blk_rdata(blk_rdata),
        .s_rst_n(),.s_valid(),.s_audio(),.s_byte(),.s_index(),.s_tag(),.s_status(),.s_commit(),.s_abort());

    // ---------------- slave side ----------------
    wire s_srst, sv, sa, sc, sab; wire [7:0] sb; wire [10:0] si; wire [8:0] st; wire [23:0] sst;
    wire s_bad;
    rgmii_command_link #(.LOCAL_ID(8'h02),.PEER_ID(8'h01),.STRETCH(24'd20),.STREAM_RX(1)) u_slink (
        .sys_clk(clk50),.rst_n(rst_n),.tx_clk(clk125),.tx_clk90(clk125_90),
        .rx_clk(s_rxc),.rx_d(s_rxd),.rx_ctl(s_rxctl),
        .tx_d(s_txd),.tx_ctl(s_txctl),.tx_clk_o(s_txc),
        .send(1'b0),.send_cmd(8'h0),.send_seq(8'h0),.send_ready(),
        .got_cmd(),.got_data(),.got_seq(),.packet_count(),.activity(),
        .led_rx_activity(),.led_rx_good(),.led_rx_bad(s_bad),.led_tx(),.led_stream(),
        .blk_ready(1'b0),.blk_audio(1'b0),.blk_line(9'd0),.blk_status(24'd0),
        .blk_take(),.blk_done(),.blk_raddr(),.blk_rdata(32'd0),
        .s_rst_n(s_srst),.s_valid(sv),.s_audio(sa),.s_byte(sb),.s_index(si),.s_tag(st),
        .s_status(sst),.s_commit(sc),.s_abort(sab));
    wire bl_ready, bl_done; wire [8:0] bl_no, bl_raddr; wire [31:0] bl_rdata;
    wire [15:0] ldrop; wire mr, ma, mc; wire [1:0] mm; wire [7:0] mv;
    media_stream_rx u_srx (
        .rx_clk(s_rxc), .rx_rst_n(s_srst),
        .s_valid(sv), .s_audio(sa), .s_byte(sb), .s_index(si), .s_tag(st),
        .s_status(sst), .s_commit(sc), .s_abort(sab),
        .mem_clk(mem_clk), .mem_rst_n(rst_n),
        .line_ready(bl_ready), .line_no(bl_no), .line_take(1'b0),
        .line_raddr(bl_raddr), .line_rdata(bl_rdata), .line_done(bl_done),
        .pix_clk(pix_s), .pix_rst_n(rst_n),
        .m_ready(mr), .m_auto(ma), .m_media(mm), .m_volume(mv), .m_commit(mc),
        .lines_dropped(ldrop));

    // SDRAM model (as tb_video_sdram_frames), 1M words
    reg [31:0] mem [0:1048575];
    reg init_done=0, busy=0, refresh=0;
    wire app_wr_en, app_rd_en; wire [20:0] app_wr_addr, app_rd_addr; wire [31:0] app_wr_data;
    reg rd_valid=0; reg [31:0] rd_data=0;
    reg [31:0] rq_data [0:255]; reg [7:0] rq_wp=0, rq_rp=0;
    integer rq_delay=0, cycle=0, burst_violations=0, max_burst=0, cur_burst=0;
    reg admitted=0, busy_q=0, refresh_q=0;
    always @(posedge mem_clk) begin
        cycle <= cycle + 1;
        if (cycle==2000) init_done <= 1;
        refresh <= (cycle % 1250) > 1180;
        busy <= ($random % 37) == 0;
        if (app_wr_en) mem[app_wr_addr[19:0]] <= app_wr_data;
        if ((app_wr_en || app_rd_en) && !admitted && (busy_q || refresh_q || !init_done))
            burst_violations = burst_violations + 1;
        if (app_wr_en || app_rd_en) begin cur_burst = cur_burst + 1; if (cur_burst > max_burst) max_burst = cur_burst; end
        else cur_burst = 0;
        admitted <= app_wr_en || app_rd_en;
        busy_q <= busy; refresh_q <= refresh;
        if (app_rd_en) begin rq_data[rq_wp] <= mem[app_rd_addr[19:0]]; rq_wp <= rq_wp + 1; end
        rd_valid <= 0;
        if (rq_wp != rq_rp) begin
            if (rq_delay < 6) rq_delay <= rq_delay + 1;
            else begin rd_valid <= 1; rd_data <= rq_data[rq_rp]; rq_rp <= rq_rp + 1; end
        end else rq_delay <= 0;
    end

    // SD video byte source at 10 MHz SPI rate, sectors with tail gaps
    function [7:0] vpix(input integer f, input integer px, input integer py);
        vpix = (f*61 + px*3 + py*7) & 8'hFF;
    endfunction
    reg byte_valid=0; reg [7:0] byte_data=0;
    wire sector_allow, frame_partial;
    integer src_frame=0, src_off=0, in_sector=0, pace=0, tail=0;
    reg feeding=0;
    always @(posedge mem_clk) begin
        byte_valid <= 0;
        if (rst_n) begin
            if (!feeding && tail != 0) tail <= tail - 1;
            if (!feeding && tail == 0 && sector_allow) begin feeding <= 1; in_sector <= 0; end
            if (feeding) begin
                pace <= pace + 1;
                if (pace % 32 == 0) begin
                    byte_valid <= 1;
                    byte_data <= vpix(src_frame, src_off % 320, src_off / 320);
                    if (src_off == 76799) begin src_off <= 0; src_frame <= src_frame + 1; end
                    else src_off <= src_off + 1;
                    if (in_sector == 511) begin feeding <= 0; tail <= 40; end else in_sector <= in_sector + 1;
                end
            end
        end
    end

    reg [11:0] x=0, y=0;
    always @(posedge pix_s) if (rst_n) begin
        if (x==H_TOTAL-1) begin x<=0; y<= y==V_TOTAL-1 ? 0 : y+1; end else x<=x+1;
    end
    wire de = x<640 && y<480;
    reg fullscreen=0;
    wire [31:0] frames_written; wire [7:0] frame_error;
    wire [23:0] prgb; wire image_ready, shown, bg_active; wire [15:0] bg_frames;
    video_sdram_frames #(.FRAME_TICKS(2000)) dut (
        .mem_clk(mem_clk), .mem_rst_n(rst_n),
        .byte_valid(byte_valid), .byte_data(byte_data),
        .sector_allow(sector_allow), .frame_partial(frame_partial),
        .frames_written(frames_written), .frame_error(frame_error),
        .sdr_init_done(init_done), .sdr_busy(busy), .sdr_refresh(refresh),
        .app_wr_en(app_wr_en), .app_wr_addr(app_wr_addr), .app_wr_data(app_wr_data),
        .app_rd_en(app_rd_en), .app_rd_addr(app_rd_addr),
        .sdr_rd_valid(rd_valid), .sdr_rd_data(rd_data),
        .bg_line_ready(bl_ready), .bg_line_no(bl_no), .bg_line_raddr(bl_raddr),
        .bg_line_rdata(bl_rdata), .bg_line_done(bl_done),
        .bg_active(bg_active), .bg_frames(bg_frames),
        .pixel_clk(pix_s), .pixel_rst_n(rst_n),
        .x(x), .y(y), .de(de), .fullscreen(fullscreen),
        .pixel_rgb(prgb), .image_ready(image_ready), .frame_shown_toggle(shown));

    // ---------------- checks ----------------
    // Video frame on screen
    reg [1:0] shown_q; integer vframe=-1;
    always @(posedge pix_s) begin shown_q <= {shown_q[0], shown}; if (shown_q[1]!=shown_q[0]) vframe = vframe + 1; end
    always @(posedge pix_s) if (x==0 && y==0) $display("%0t ms: local frame, bg_active=%b bg_stored=%0d master_frames=%0d vframe=%0d checked=%0d err=%0d commits=%0d full=%0d",
        $time/1000000000, bg_active, bg_frames, mframe, vframe, frames_checked, errors, commits_seen, fullscreen);
    // Background: identify the master frame of each displayed frame from its
    // first background pixel, then require every other background pixel to
    // match that same frame.
    reg [11:0] x1,y1,x2,y2; reg de1,de2;
    integer full_now=0, bgf=-1, errors=0, bg_checked=0, v_checked=0, frames_checked=0;
    integer seen_bg_frames [0:63]; integer nseen=0;
    integer exp, sx, sy, f, k;
    reg [15:0] p16;
    always @(posedge pix_s) begin
        x1<=x; y1<=y; de1<=de; x2<=x1; y2<=y1; de2<=de1;
        if (x==0 && y==V_TOTAL-1) full_now = fullscreen;
    end
    function [23:0] exp565(input [15:0] p);
        exp565 = {p[15:11], p[15:13], p[10:5], p[10:9], p[4:0], p[4:2]};
    endfunction
    always @(posedge pix_s) if (rst_n && image_ready && de2 && y < 480 && x2 < 640 && y2 < 480) begin
        if (full_now) begin
            exp = vpix(vframe, x2/2, y2/2);
            if (prgb !== {exp[7:5],exp[7:5],exp[7:6],exp[4:2],exp[4:2],exp[4:3],exp[1:0],exp[1:0],exp[1:0],exp[1:0]}) begin
                if (errors<6) $display("full mismatch x=%0d y=%0d", x2, y2); errors=errors+1; end
            v_checked = v_checked + 1;
        end else if (x2 >= 480 && y2 < 120) begin
            exp = vpix(vframe, (x2-480)*2, y2*2);
            if (prgb !== {exp[7:5],exp[7:5],exp[7:6],exp[4:2],exp[4:2],exp[4:3],exp[1:0],exp[1:0],exp[1:0],exp[1:0]}) begin
                if (errors<6) $display("win mismatch x=%0d y=%0d", x2, y2); errors=errors+1; end
            v_checked = v_checked + 1;
        end else if (bg_active) begin
            if (x2==0 && y2==0) begin
                // find which master frame this is
                bgf = -1;
                for (k=0; k<mframe+1; k=k+1) if (prgb === exp565(mpix(k,0,0))) bgf = k;
                if (bgf < 0) begin $display("bg frame not identified at frame start"); errors=errors+1; end
                else begin seen_bg_frames[nseen%64]=bgf; nseen=nseen+1; frames_checked=frames_checked+1; end
            end else if (bgf >= 0) begin
                if (prgb !== exp565(mpix(bgf, x2, y2))) begin
                    if (errors<6) $display("bg mismatch f=%0d x=%0d y=%0d got %h exp %h", bgf, x2, y2, prgb, exp565(mpix(bgf,x2,y2)));
                    errors=errors+1;
                end
                bg_checked = bg_checked + 1;
            end
        end
    end
    // Status: the slave must see the master's state, and one commit pulse
    // per increment of the master's commit counter (never at connection).
    integer commits_seen=0, a_count=0, a_err=0;
    always @(posedge pix_s) if (mc) commits_seen = commits_seen + 1;
    always @(posedge pix_m) if (rst_n && mx==0 && my==0 && (mframe==3 || mframe==5 || mframe==6)) m_commits <= m_commits + 1;
    always @(posedge pix_s) if (mr) a_count = a_count + 1;

    initial begin
        #500 rst_n = 1;
        // Window mode for several frames, then fullscreen, then window again.
        wait (frames_checked >= 4);
        wait (y == 300); fullscreen = 1;
        wait (y == 0); wait (y == 300); wait (y == 0); wait (y == 300);
        fullscreen = 0;
        wait (frames_checked >= 7);
        wait (y == 479); wait (y == 0);
        $display("bg frames on screen:");
        for (k=0;k<nseen && k<64;k=k+1) $write(" %0d", seen_bg_frames[k]); $display("");
        $display("master frames=%0d stored=%0d, drops tx=%0d rx=%0d, status ready=%b auto=%b media=%0d vol=%0d commits=%0d, max burst=%0d",
                 mframe, bg_frames, m_dropped, ldrop, mr, ma, mm, mv, commits_seen, max_burst);
        if (frame_error) $fatal(1, "frame_error=%h", frame_error);
        if (burst_violations) $fatal(1, "%0d bursts started while busy/refresh", burst_violations);
        if (max_burst > 128) $fatal(1, "burst longer than 128 words: %0d", max_burst);
        if (errors) $fatal(1, "%0d pixel errors (bg %0d, video %0d checked)", errors, bg_checked, v_checked);
        if (bg_checked < 600000) $fatal(1, "too few background pixels checked: %0d", bg_checked);
        if (!mr || !ma || mm != 2 || mv != 200) $fatal(1, "status not received");
        if (commits_seen != 3) $fatal(1, "expected 3 commit pulses, got %0d", commits_seen);
        if (s_bad) $fatal(1, "bad frame at slave");
        $display("PASS: plan A composite, %0d bg + %0d video pixels, %0d frames identified, status and %0d commits received",
                 bg_checked, v_checked, frames_checked, commits_seen);
        $finish;
    end
    initial if ($test$plusargs("probe")) begin #200000000; $display("probe: 200 ms bg_active=%b bg_frames=%0d mframe=%0d image_ready=%b vframe=%0d written=%0d err=%h ldrop=%0d mdrop=%0d commits=%0d sbad=%b", bg_active, bg_frames, mframe, image_ready, vframe, frames_written, frame_error, ldrop, m_dropped, commits_seen, s_bad); $finish; end
    initial begin #900000000 $fatal(1, "timeout frames_checked=%0d bg_frames=%0d vframe=%0d err=%h", frames_checked, bg_frames, vframe, frame_error); end
endmodule
