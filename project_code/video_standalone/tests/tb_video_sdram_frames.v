`timescale 1ns/1ps
// video_sdram_frames with a behavioural SDRAM application port: random
// busy/refresh windows, reads returning after a variable latency.
// Feeds three 320x240 frames whose pixel value encodes (frame, x, y), shows
// them fullscreen and in the 160x120 window, and checks every pixel.
module tb_video_sdram_frames;
    reg [7:0] raw [0:230399];
    initial if ($test$plusargs("real")) $readmemh("vid_first3.hex", raw);
    parameter integer H_TOTAL = 800, V_TOTAL = 525;
    parameter integer INIT_CYCLES = 20000;   // 500 us, longer than many lines
    reg mem_clk=0, pix_clk=0, rst_n=0;
    always #12.5 mem_clk = ~mem_clk;     // 40 MHz
    always #20   pix_clk = ~pix_clk;     // 25 MHz

    // Behavioural SDRAM
    reg [31:0] mem [0:65535];
    reg init_done=0, busy=0, refresh=0;
    wire app_wr_en, app_rd_en;
    wire [20:0] app_wr_addr, app_rd_addr;
    wire [31:0] app_wr_data;
    reg rd_valid=0; reg [31:0] rd_data=0;
    reg [31:0] rq_data [0:255]; reg [7:0] rq_wp=0, rq_rp=0;
    integer rq_delay=0, cycle=0;
    integer burst_violations=0;
    reg admitted=0, busy_q=0, refresh_q=0;
    always @(posedge mem_clk) begin
        cycle <= cycle + 1;
        if (cycle==INIT_CYCLES) init_done <= 1;
        // refresh window every 1250 cycles, busy blips at random
        refresh <= (cycle % 1250) > 1180;
        busy <= ($random % 37) == 0;
        if (app_wr_en) mem[app_wr_addr[15:0]] <= app_wr_data;
        // The DUT samples busy/refresh on the clock before its first en.
        if ((app_wr_en || app_rd_en) && !admitted && (busy_q || refresh_q || !init_done))
            burst_violations = burst_violations + 1;
        admitted <= app_wr_en || app_rd_en;
        busy_q <= busy; refresh_q <= refresh;
        if (app_rd_en) begin
            rq_data[rq_wp] <= mem[app_rd_addr[15:0]]; rq_wp <= rq_wp + 1;
        end
        rd_valid <= 0;
        if (rq_wp != rq_rp) begin
            if (rq_delay < 6) rq_delay <= rq_delay + 1;
            else begin rd_valid <= 1; rd_data <= rq_data[rq_rp]; rq_rp <= rq_rp + 1; end
        end else rq_delay <= 0;
    end

    // Video timing
    reg [11:0] x=0, y=0;
    always @(posedge pix_clk) if (rst_n) begin
        if (x==H_TOTAL-1) begin x<=0; y<= y==V_TOTAL-1 ? 0 : y+1; end else x<=x+1;
    end
    wire de = x<640 && y<480;

    // Byte source standing in for the SD streamer: sectors of 512 bytes,
    // started only when sector_allow, ~1 byte per 4 mem clocks.
    reg use_real; initial use_real = $test$plusargs("real");
    function [7:0] pix(input integer f, input integer px, input integer py);
        pix = use_real ? raw[f*76800 + py*320 + px] : (f*61 + px*3 + py*7) & 8'hFF;
    endfunction
    reg byte_valid=0; reg [7:0] byte_data=0;
    wire sector_allow, frame_partial;
    integer src_frame=0, src_off=0, in_sector=0, pace=0, tail=0;
    reg feeding=0;
    reg fullscreen=1;
    always @(posedge mem_clk) begin
        byte_valid <= 0;
        if (rst_n && src_frame < 3) begin
            // The real streamer sees sector_done (CRC, end of block) many
            // clocks after the last data byte before it asks again.
            if (!feeding && tail != 0) tail <= tail - 1;
            if (!feeding && tail == 0 && sector_allow) begin feeding <= 1; in_sector <= 0; end
            if (feeding) begin
                pace <= pace + 1;
                if (pace % 32 == 0) begin
                    byte_valid <= 1;
                    byte_data <= pix(src_frame, src_off % 320, src_off / 320);
                    if (src_off == 76799) begin src_off <= 0; src_frame <= src_frame + 1; end
                    else src_off <= src_off + 1;
                    if (in_sector == 511) begin feeding <= 0; tail <= 40; end else in_sector <= in_sector + 1;
                end
            end
        end
    end

    wire [31:0] frames_written; wire [7:0] frame_error;
    wire vid_valid; wire [7:0] vid_byte, vid_sector, vid_frame; wire [8:0] vid_index;
    wire [7:0] pixel_index; wire image_ready, shown;
    video_sdram_frames #(.FRAME_TICKS(2000)) dut (
        .mem_clk(mem_clk), .mem_rst_n(rst_n),
        .byte_valid(byte_valid), .byte_data(byte_data),
        .sector_allow(sector_allow), .frame_partial(frame_partial),
        .frames_written(frames_written), .frame_error(frame_error),
        .sdr_init_done(init_done), .sdr_busy(busy), .sdr_refresh(refresh),
        .app_wr_en(app_wr_en), .app_wr_addr(app_wr_addr), .app_wr_data(app_wr_data),
        .app_rd_en(app_rd_en), .app_rd_addr(app_rd_addr),
        .sdr_rd_valid(rd_valid), .sdr_rd_data(rd_data),
        .vid_valid(vid_valid), .vid_byte(vid_byte), .vid_index(vid_index),
        .vid_sector(vid_sector), .vid_frame(vid_frame),
        .pixel_clk(pix_clk), .pixel_rst_n(rst_n),
        .x(x), .y(y), .de(de), .fullscreen(fullscreen),
        .pixel_index(pixel_index), .image_ready(image_ready),
        .frame_shown_toggle(shown));

    // vid_* stream check: index/sector follow the byte offset
    integer vid_err=0, vid_count=0;
    always @(posedge mem_clk) if (vid_valid) begin
        if (vid_index != vid_count % 512 || vid_sector != (vid_count % 76800) / 512 ||
            vid_frame != vid_count / 76800) vid_err = vid_err + 1;
        vid_count = vid_count + 1;
    end

    // Pixel check: output is two pixel clocks behind x/y.
    reg [11:0] x1, y1, x2, y2; reg de1, de2;
    integer shown_frame = -1, checked=0, errors=0, mode_full_now=1;
    reg [1:0] shown_q;
    always @(posedge pix_clk) begin
        x1<=x; y1<=y; de1<=de; x2<=x1; y2<=y1; de2<=de1;
        shown_q <= {shown_q[0], shown};
        if (x==0 && y==V_TOTAL-1) mode_full_now = fullscreen;
    end
    // Frame index on screen follows each shown toggle.
    always @(posedge pix_clk) if (shown_q[1] != shown_q[0]) shown_frame = shown_frame + 1;
    integer exp, sx, sy, in_win;
    always @(posedge pix_clk) if (rst_n && image_ready && y2 < 480 && x2 < 640 && y < 480) begin
        if (mode_full_now) begin
            sx = x2/2; sy = y2/2; in_win = de2;
        end else begin
            in_win = de2 && x2 >= 480 && y2 < 120;
            sx = (x2-480)*2; sy = y2*2;
        end
        exp = in_win ? pix(shown_frame, sx, sy) : 0;
        if (x2==x && 0) ;
        if (pixel_index !== exp[7:0]) begin
            if (errors < 8) $display("pixel mismatch f=%0d full=%0d x=%0d y=%0d got %h exp %h",
                shown_frame, mode_full_now, x2, y2, pixel_index, exp[7:0]);
            errors = errors + 1;
        end
        checked = checked + 1;
    end

    initial begin
        #500 rst_n = 1;
        // Frame 0 fullscreen, frame 1 small window, frame 2 fullscreen.
        wait (shown_frame == 0); @(posedge pix_clk);
        wait (y == 100); fullscreen = 0;
        wait (shown_frame == 1);
        wait (y == 300); wait (y == 0);   // one full small-window refresh
        wait (y == 200); fullscreen = 1;
        wait (shown_frame == 2);
        wait (y == 479); wait (y == 0);
        if (frame_error) $fatal(1, "frame_error=%h", frame_error);
        if (burst_violations) $fatal(1, "%0d bursts started while busy/refresh", burst_violations);
        if (vid_err) $fatal(1, "vid stream %0d errors", vid_err);
        if (errors) $fatal(1, "%0d pixel errors of %0d", errors, checked);
        if (frames_written != 3) $fatal(1, "frames_written=%0d", frames_written);
        $display("PASS: 3 frames via SDRAM, %0d pixels checked (full + 160x120 window), vid stream ok", checked);
        $finish;
    end
    initial begin #400000000 $fatal(1, "timeout shown=%0d written=%0d", shown_frame, frames_written); end
endmodule
