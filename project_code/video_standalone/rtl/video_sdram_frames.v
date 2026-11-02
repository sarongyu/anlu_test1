`timescale 1ns/1ps

// Slave frame store and compositor (plan A).
//
// SDRAM layout (32-bit words):
//   0x000000 + bank*0x8000   two 320x240 RGB332 video frames (19200 words)
//   0x040000 * (1 + n)       three 640x480 RGB565 background frames
//                            (153600 words, 320 words per line, n = 0..2)
//
// Video: file bytes are packed four per word and written in bursts of up
// to 16 words. After the last word of a frame is in SDRAM the bank is
// offered to the display, which switches during vertical blanking and
// acknowledges; only then may the next frame's first sector start. A new
// frame also waits FRAME_TICKS after the previous one started (10 fps).
//
// Background: the master's composite picture arrives as 320-word lines
// (bg_line_*). Lines go into the write buffer W; when line 479 is written,
// or a lower line number shows that a new master frame began, W becomes
// the newest complete buffer R and writing moves to the buffer that is
// neither R nor the one on screen (D). At each local frame start D takes R
// if R is newer. With BG_DECIMATE=1 only every second master frame is
// stored (30 fps): the lines of the other frames are released unwritten,
// which halves the SDRAM write load. The two boards' frame rates differ slightly, so a master
// frame is now and then shown twice or skipped, never torn. If no line
// arrives for BG_TIMEOUT clocks the background turns black.
//
// Display: for each line the pixel domain asks, one line ahead, for the
// video row (fullscreen, or window lines 0..119) and the background line
// (window mode); the memory side reads them into two-line buffers. Reads
// take priority over writes. SDRAM use follows media_frame_reader /
// media_frame_loader: a batch starts only when the controller is
// initialised, not busy and not refreshing, en stays high for the whole
// batch (at most 128 words) and read data is counted, not timed.
//
// Output: fullscreen shows the video 2x over 640x480. Window mode shows the
// background with the video, every second pixel and line, in a 160x120
// window at the top right (x 480..639, y 0..119). pixel_rgb follows x/y/de
// by two pixel clocks; layout, video bank and background buffer change
// only at the last line of a frame.
module video_sdram_frames #(
    parameter integer FRAME_TICKS = 4000000,
    parameter integer H_ACT = 640, V_ACT = 480, V_TOTAL = 525,
    parameter integer BG_TIMEOUT = 20000000,
    parameter integer BG_DECIMATE = 1
)(
    input  wire        mem_clk, mem_rst_n,
    input  wire        byte_valid,
    input  wire [7:0]  byte_data,
    output wire        sector_allow,
    output wire        frame_partial,
    output reg  [31:0] frames_written,
    output reg  [7:0]  frame_error,
    // SDRAM application port (sdram.v)
    input  wire        sdr_init_done, sdr_busy, sdr_refresh,
    output reg         app_wr_en,
    output reg  [20:0] app_wr_addr,
    output reg  [31:0] app_wr_data,
    output reg         app_rd_en,
    output reg  [20:0] app_rd_addr,
    input  wire        sdr_rd_valid,
    input  wire [31:0] sdr_rd_data,
    // Background lines from media_stream_rx (mem domain)
    input  wire        bg_line_ready,
    input  wire [8:0]  bg_line_no,
    output wire [8:0]  bg_line_raddr,
    input  wire [31:0] bg_line_rdata,     // registered read of bg_line_raddr
    output reg         bg_line_done,
    output reg         bg_active,
    output reg  [15:0] bg_frames,
    // Pixel domain
    input  wire        pixel_clk, pixel_rst_n,
    input  wire [11:0] x, y,
    input  wire        de,
    input  wire        fullscreen,
    output reg  [23:0] pixel_rgb,
    output reg         image_ready,
    output reg         frame_shown_toggle
);
    localparam integer ROW_WORDS   = 80;          // 320 bytes of video
    localparam integer BG_WORDS    = 320;         // 640 RGB565 pixels
    localparam integer FRAME_BYTES = 76800;       // 320 x 240
    localparam integer FRAME_WORDS = 19200;
    localparam integer FIFO_DEPTH  = 256;
    localparam integer BURST       = 16;
    localparam integer MAX_BURST   = 128;
    localparam integer GAP         = 12;
    localparam [7:0] ERR_LATE_BYTE = 8'h86, ERR_FIFO = 8'h87, ERR_FETCH = 8'h88;

    function [20:0] bg_base;
        input [1:0] n;
        bg_base = n == 2'd0 ? 21'h040000 : n == 2'd1 ? 21'h080000 : 21'h0C0000;
    endfunction

    // ------------------------------------------------------------------
    // Memory domain: video frame writer state
    // ------------------------------------------------------------------
    reg [16:0] frame_byte;
    reg [23:0] pack;
    reg write_bank, ready_bank, ready_toggle;
    reg wait_swap, flush_pending, frame_started;
    reg [22:0] frame_timer;
    reg [2:0] ack_meta;
    reg ack_toggle;                       // pixel domain
    reg [14:0] wr_word_idx;

    reg [31:0] fifo_mem [0:FIFO_DEPTH-1];
    reg [7:0] fifo_wp, fifo_rp;
    reg [8:0] fifo_count;
    reg [31:0] fifo_q;
    reg push_q;
    wire accept = byte_valid && !wait_swap;
    wire push = accept && frame_byte[1:0] == 2'd3;
    wire pop;

    // A sector (128 words) may start only when the SDRAM is up and the FIFO
    // has room for all of it, so the FIFO cannot overflow even if SDRAM
    // writes stall for the whole sector.
    assign sector_allow = sdr_init_done && !wait_swap && !flush_pending &&
        fifo_count <= FIFO_DEPTH - 128 &&
        (frame_byte != 0 || !frame_started || frame_timer == 0);
    assign frame_partial = frame_byte != 0;

    always @(posedge mem_clk) begin
        if (push) fifo_mem[fifo_wp] <= {byte_data, pack};
        fifo_q <= fifo_mem[pop ? fifo_rp + 1'b1 : fifo_rp];
    end

    // ------------------------------------------------------------------
    // Line requests from the pixel domain
    // ------------------------------------------------------------------
    reg req_toggle;                       // pixel domain, fields below too
    reg [7:0] req_row;
    reg [8:0] req_bgline;
    reg req_slot, req_bank, req_vid, req_bg, req_first;
    reg [2:0] req_meta, shown_meta;
    reg fetch_pending;
    reg [7:0] p_row;
    reg [8:0] p_bgline;
    reg p_slot, p_bank, p_vid, p_bg, p_first;
    // active fetch
    reg f_active, f_vid, f_bg, f_slot;
    reg [20:0] f_vaddr, f_baddr;
    reg [8:0] f_bidx;                     // background words issued

    // Two-line buffers, written in mem_clk, read in pixel_clk.
    reg [31:0] vline [0:159];
    reg [31:0] bline [0:639];
    reg vl_we, bl_we;
    reg [7:0] vl_waddr;
    reg [9:0] bl_waddr;
    reg [31:0] l_wdata;
    always @(posedge mem_clk) begin
        if (vl_we) vline[vl_waddr] <= l_wdata;
        if (bl_we) bline[bl_waddr] <= l_wdata;
    end

    // ------------------------------------------------------------------
    // Background buffer bookkeeping (memory domain)
    // ------------------------------------------------------------------
    reg [1:0] bg_w, bg_r, bg_d;
    reg bg_fresh, bg_have, bg_lines_in_w;
    reg [8:0] bg_seen_last;
    reg bg_seen_any, bg_par;
    reg bg_armed;
    reg [20:0] bgw_base;
    reg [8:0] bgw_idx;
    reg [8:0] bgw_line;
    reg [24:0] bg_idle;

    // ------------------------------------------------------------------
    // Memory sequencer
    // ------------------------------------------------------------------
    localparam [3:0] S_IDLE=0, S_ACC=1, S_RD=2, S_RDWAIT=3, S_WR=4, S_BW=5,
                     S_GAP=6, S_ARM=7, S_ARM2=8;
    reg [3:0] state;
    reg [7:0] burst_n, burst_i, ret_count;
    reg burst_bg;                         // current read burst targets bline
    reg [9:0] ret_base;
    reg [4:0] wr_left;
    reg [4:0] gap;
    reg [9:0] wait_count;
    // A batch's first en is issued on the clock that sees admission, like
    // start_burst/app_rd_en in media_frame_reader.
    wire admit = sdr_init_done && !sdr_busy && !sdr_refresh;
    wire fetch_busy = f_active || fetch_pending;
    wire rd_start = state == S_IDLE && f_active && admit;
    wire wr_start = state == S_IDLE && !fetch_busy && admit && fifo_count != 0 &&
                    !push_q && (fifo_count >= BURST || flush_pending);
    wire bw_start = state == S_IDLE && !fetch_busy && !wr_start && admit && bg_armed;
    wire [4:0] wr_n = fifo_count >= BURST ? BURST : fifo_count[4:0];
    wire [8:0] bw_left = BG_WORDS - bgw_idx;
    wire [7:0] bw_n = bw_left >= MAX_BURST ? MAX_BURST : bw_left[7:0];
    wire [8:0] fb_left = BG_WORDS - f_bidx;
    wire [7:0] fb_n = fb_left >= MAX_BURST ? MAX_BURST : fb_left[7:0];
    wire [14:0] p_row_base = {1'b0, p_row, 6'b0} + {3'b0, p_row, 4'b0};
    assign pop = wr_start || state == S_WR;
    // Background line source: read address leads by one while words go out.
    wire bw_go = bw_start || state == S_BW;
    assign bg_line_raddr = bw_go ? bgw_idx + 1'b1 : bgw_idx;
    wire bg_new_mframe = bg_seen_any && bg_line_no <= bg_seen_last;
    wire bg_par_now = bg_new_mframe ? ~bg_par : bg_par;
    wire bg_keep = !BG_DECIMATE || !bg_par_now;

    always @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin
            frame_byte <= 0; pack <= 0;
            write_bank <= 0; ready_bank <= 0; ready_toggle <= 0;
            wait_swap <= 0; flush_pending <= 0; frame_started <= 0;
            frame_timer <= 0; ack_meta <= 0; wr_word_idx <= 0;
            frames_written <= 0; frame_error <= 0;
            fifo_wp <= 0; fifo_rp <= 0; fifo_count <= 0; push_q <= 0;
            req_meta <= 0; shown_meta <= 0; fetch_pending <= 0;
            p_row <= 0; p_bgline <= 0; p_slot <= 0; p_bank <= 0;
            p_vid <= 0; p_bg <= 0; p_first <= 0;
            f_active <= 0; f_vid <= 0; f_bg <= 0; f_slot <= 0;
            f_vaddr <= 0; f_baddr <= 0; f_bidx <= 0;
            vl_we <= 0; bl_we <= 0; vl_waddr <= 0; bl_waddr <= 0; l_wdata <= 0;
            bg_w <= 2'd0; bg_r <= 2'd1; bg_d <= 2'd2;
            bg_fresh <= 0; bg_have <= 0; bg_lines_in_w <= 0;
            bg_seen_last <= 0; bg_seen_any <= 0; bg_par <= 0;
            bg_armed <= 0; bgw_base <= 0; bgw_idx <= 0; bgw_line <= 0; bg_idle <= 0;
            bg_line_done <= 0; bg_active <= 0; bg_frames <= 0;
            state <= S_IDLE; burst_n <= 0; burst_i <= 0; ret_count <= 0;
            burst_bg <= 0; ret_base <= 0; wr_left <= 0; gap <= 0; wait_count <= 0;
            app_wr_en <= 0; app_wr_addr <= 0; app_wr_data <= 0;
            app_rd_en <= 0; app_rd_addr <= 0;
        end else begin
            app_wr_en <= 0; app_rd_en <= 0; vl_we <= 0; bl_we <= 0;
            bg_line_done <= 0;
            push_q <= push;
            ack_meta <= {ack_meta[1:0], ack_toggle};
            req_meta <= {req_meta[1:0], req_toggle};
            shown_meta <= {shown_meta[1:0], image_ready};
            if (frame_timer != 0) frame_timer <= frame_timer - 1'b1;
            if (ack_meta[2] != ack_meta[1]) wait_swap <= 0;

            // Video writer
            if (byte_valid && wait_swap) frame_error <= ERR_LATE_BYTE;
            if (accept) begin
                if (frame_byte == 0) begin
                    frame_timer <= FRAME_TICKS - 1; frame_started <= 1;
                end
                case (frame_byte[1:0])
                    2'd0: pack[7:0] <= byte_data;
                    2'd1: pack[15:8] <= byte_data;
                    2'd2: pack[23:16] <= byte_data;
                    default: ;
                endcase
                if (frame_byte == FRAME_BYTES-1) begin
                    frame_byte <= 0; wait_swap <= 1; flush_pending <= 1;
                end else frame_byte <= frame_byte + 1'b1;
            end
            if (push) begin
                fifo_wp <= fifo_wp + 1'b1;
                if (fifo_count == FIFO_DEPTH) frame_error <= ERR_FIFO;
            end
            if (pop) fifo_rp <= fifo_rp + 1'b1;
            fifo_count <= fifo_count + push - pop;
            if (flush_pending && fifo_count == 0 && !push && state != S_WR && !wr_start &&
                wr_word_idx == FRAME_WORDS) begin
                flush_pending <= 0; wr_word_idx <= 0;
                ready_bank <= write_bank; write_bank <= !write_bank;
                ready_toggle <= !ready_toggle;
                frames_written <= frames_written + 1'b1;
            end

            // Line requests. Before the first frame is shown the buffers are
            // not displayed, so a request that overtakes another is only an
            // error once image_ready is set.
            if (req_meta[2] != req_meta[1]) begin
                if (fetch_busy && shown_meta[2]) frame_error <= ERR_FETCH;
                fetch_pending <= 1;
                p_row <= req_row; p_bgline <= req_bgline; p_slot <= req_slot;
                p_bank <= req_bank; p_vid <= req_vid; p_bg <= req_bg; p_first <= req_first;
            end

            // Background link watchdog
            if (bg_line_done) bg_idle <= 0;
            else if (bg_idle != BG_TIMEOUT) bg_idle <= bg_idle + 1'b1;
            else begin bg_have <= 0; bg_fresh <= 0; end

            // Read data returns, counted and routed to the burst's buffer
            if (sdr_rd_valid && ret_count != burst_n) begin
                l_wdata <= sdr_rd_data;
                if (burst_bg) begin bl_we <= 1; bl_waddr <= ret_base + ret_count; end
                else begin vl_we <= 1; vl_waddr <= ret_base[7:0] + ret_count; end
                ret_count <= ret_count + 1'b1;
            end

            case (state)
                S_IDLE:
                    if (!f_active && fetch_pending) begin
                        // Take the request; pick the frame's background buffer.
                        fetch_pending <= 0;
                        if (p_first) begin
                            if (bg_fresh) begin bg_d <= bg_r; bg_fresh <= 0; end
                            bg_active <= bg_have;
                        end
                        state <= S_ACC;
                    end else if (rd_start) begin
                        ret_count <= 0; wait_count <= 0;
                        app_rd_en <= 1;
                        if (f_vid) begin
                            burst_bg <= 0; burst_n <= ROW_WORDS;
                            ret_base <= f_slot ? 10'd80 : 10'd0;
                            app_rd_addr <= f_vaddr;
                            f_vid <= 0;
                        end else begin
                            burst_bg <= 1; burst_n <= fb_n;
                            ret_base <= (f_slot ? 10'd320 : 10'd0) + f_bidx;
                            app_rd_addr <= f_baddr + f_bidx;
                            f_bidx <= f_bidx + fb_n;
                        end
                        burst_i <= 1;
                        state <= S_RD;
                    end else if (wr_start) begin
                        app_wr_en <= 1;
                        app_wr_addr <= {5'b0, write_bank, wr_word_idx};
                        app_wr_data <= fifo_q;
                        wr_word_idx <= wr_word_idx + 1'b1;
                        wr_left <= wr_n - 1'b1;
                        if (wr_n == 1) begin gap <= GAP; state <= S_GAP; end
                        else state <= S_WR;
                    end else if (bw_start) begin
                        app_wr_en <= 1;
                        app_wr_addr <= bgw_base + bgw_idx;
                        app_wr_data <= bg_line_rdata;
                        bgw_idx <= bgw_idx + 1'b1;
                        burst_n <= bw_n; burst_i <= 1;
                        if (bw_n == 1) state <= S_GAP; else state <= S_BW;
                        gap <= GAP;
                    end else if (!fetch_busy && !bg_armed && bg_line_ready) begin
                        state <= S_ARM;
                    end
                S_ACC: begin
                    f_active <= 1; f_slot <= p_slot; f_bidx <= 0;
                    f_vid <= p_vid;
                    f_bg <= p_bg && bg_active;
                    f_vaddr <= {5'b0, p_bank, p_row_base};
                    f_baddr <= bg_base(bg_d) + {p_bgline, 8'b0} + {2'b0, p_bgline, 6'b0};
                    // Nothing to read (window mode without background).
                    if (!p_vid && !(p_bg && bg_active)) f_active <= 0;
                    state <= S_IDLE;
                end
                S_RD: begin
                    app_rd_en <= 1;
                    app_rd_addr <= app_rd_addr + 1'b1;
                    burst_i <= burst_i + 1'b1;
                    if (burst_i + 1'b1 == burst_n) state <= S_RDWAIT;
                end
                S_RDWAIT: begin
                    wait_count <= wait_count + 1'b1;
                    if (ret_count == burst_n || wait_count == 10'h3FF) begin
                        if (ret_count != burst_n) begin
                            frame_error <= ERR_FETCH; ret_count <= burst_n;
                        end
                        // Fetch finished when no video is left and the
                        // background line is complete (or not wanted).
                        if (!f_vid && (!f_bg || f_bidx == BG_WORDS)) f_active <= 0;
                        gap <= GAP; state <= S_GAP;
                    end
                end
                S_WR: begin
                    app_wr_en <= 1;
                    app_wr_addr <= {5'b0, write_bank, wr_word_idx};
                    app_wr_data <= fifo_q;
                    wr_word_idx <= wr_word_idx + 1'b1;
                    wr_left <= wr_left - 1'b1;
                    if (wr_left == 1) begin gap <= GAP; state <= S_GAP; end
                end
                S_BW: begin
                    app_wr_en <= 1;
                    app_wr_addr <= bgw_base + bgw_idx;
                    app_wr_data <= bg_line_rdata;
                    bgw_idx <= bgw_idx + 1'b1;
                    burst_i <= burst_i + 1'b1;
                    if (burst_i + 1'b1 == burst_n) state <= S_GAP;
                end
                S_ARM: begin
                    // A line number not above the previous one means the
                    // master started a new frame. If line 479 of the last
                    // stored frame never arrived, close W now.
                    bg_seen_last <= bg_line_no; bg_seen_any <= 1;
                    if (bg_new_mframe) begin
                        bg_par <= ~bg_par;
                        if (bg_lines_in_w) begin
                            bg_r <= bg_w; bg_w <= 2'd3 - bg_d - bg_w;
                            bg_fresh <= 1; bg_have <= 1; bg_lines_in_w <= 0;
                            bg_frames <= bg_frames + 1'b1;
                        end
                    end
                    if (bg_keep) state <= S_ARM2;
                    else begin
                        // Skipped frame: release the slot unwritten. The gap
                        // lets the slot counter update before the next look.
                        bg_line_done <= 1; gap <= 3; state <= S_GAP;
                    end
                end
                S_ARM2: begin
                    bgw_base <= bg_base(bg_w) + {bg_line_no, 8'b0} + {2'b0, bg_line_no, 6'b0};
                    bgw_line <= bg_line_no; bgw_idx <= 0; bg_armed <= 1;
                    state <= S_IDLE;
                end
                default: begin   // S_GAP
                    if (bg_armed && bgw_idx == BG_WORDS) begin
                        // Whole line in SDRAM: release the slot.
                        bg_armed <= 0; bgw_idx <= 0; bg_line_done <= 1;
                        bg_lines_in_w <= 1;
                        if (bgw_line == V_ACT-1) begin
                            bg_r <= bg_w; bg_w <= 2'd3 - bg_d - bg_w;
                            bg_fresh <= 1; bg_have <= 1; bg_lines_in_w <= 0;
                            bg_frames <= bg_frames + 1'b1;
                        end
                    end
                    if (gap != 0) gap <= gap - 1'b1;
                    else state <= S_IDLE;
                end
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Pixel domain
    // ------------------------------------------------------------------
    reg [2:0] ready_meta;
    reg ready_seen;
    reg [1:0] full_meta, bgact_meta;
    reg disp_bank, mode_full;
    reg [1:0] bsel_q;
    reg win_q, bg_q_en, bhalf_q;
    reg [31:0] lb_q, bb_q;
    wire last_line_start = x == 0 && y == V_TOTAL-1;
    wire new_frame = ready_meta[2] != ready_seen;
    wire [11:0] next_y = y + 1'b1;

    wire in_small = de && x >= H_ACT-160 && y < 120;
    wire win = mode_full ? de : in_small;
    wire [11:0] small_dx = x - (H_ACT-160);
    wire [8:0] sx = mode_full ? x[9:1] : {small_dx[7:0], 1'b0};

    wire [7:0] rd_addr = (y[0] ? 8'd80 : 8'd0) + sx[8:2];
    wire [9:0] bg_addr = (y[0] ? 10'd320 : 10'd0) + x[9:1];
    always @(posedge pixel_clk) begin
        lb_q <= vline[rd_addr];
        bb_q <= bline[bg_addr];
    end

    wire [7:0] vbyte = bsel_q == 0 ? lb_q[7:0] : bsel_q == 1 ? lb_q[15:8] :
                       bsel_q == 2 ? lb_q[23:16] : lb_q[31:24];
    wire [15:0] bpix = bhalf_q ? bb_q[31:16] : bb_q[15:0];
    wire [23:0] v_rgb = {vbyte[7:5], vbyte[7:5], vbyte[7:6],
                         vbyte[4:2], vbyte[4:2], vbyte[4:3],
                         vbyte[1:0], vbyte[1:0], vbyte[1:0], vbyte[1:0]};
    wire [23:0] b_rgb = {bpix[15:11], bpix[15:13], bpix[10:5], bpix[10:9],
                         bpix[4:0], bpix[4:2]};

    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            ready_meta <= 0; ready_seen <= 0; full_meta <= 0; bgact_meta <= 0;
            ack_toggle <= 0; disp_bank <= 0; mode_full <= 0;
            image_ready <= 0; frame_shown_toggle <= 0;
            req_toggle <= 0; req_row <= 0; req_bgline <= 0; req_slot <= 0;
            req_bank <= 0; req_vid <= 0; req_bg <= 0; req_first <= 0;
            bsel_q <= 0; win_q <= 0; bg_q_en <= 0; bhalf_q <= 0;
            pixel_rgb <= 0;
        end else begin
            ready_meta <= {ready_meta[1:0], ready_toggle};
            full_meta <= {full_meta[0], fullscreen};
            bgact_meta <= {bgact_meta[0], bg_active};
            if (last_line_start) begin
                // Commit layout and video bank, then fetch line 0.
                mode_full <= full_meta[1];
                if (new_frame) begin
                    ready_seen <= ready_meta[2];
                    disp_bank <= ready_bank;        // stable since its toggle
                    ack_toggle <= !ack_toggle;
                    image_ready <= 1;
                    frame_shown_toggle <= !frame_shown_toggle;
                end
                req_toggle <= !req_toggle;
                req_first <= 1; req_row <= 0; req_bgline <= 0; req_slot <= 0;
                req_bank <= new_frame ? ready_bank : disp_bank;
                req_vid <= 1;
                req_bg <= !full_meta[1];
            end else if (x == 0 && y < V_ACT-1) begin
                req_toggle <= !req_toggle;
                req_first <= 0;
                req_row <= mode_full ? next_y[8:1] : {next_y[6:0], 1'b0};
                req_bgline <= next_y[8:0];
                req_slot <= next_y[0];
                req_bank <= disp_bank;
                req_vid <= mode_full || next_y < 120;
                req_bg <= !mode_full;
            end

            // Stage 1: line buffer reads; stage 2: select and expand.
            bsel_q <= sx[1:0]; win_q <= win && image_ready; bhalf_q <= x[0];
            // bg_active changes during the last line, before line 0 shows.
            bg_q_en <= de && !mode_full && bgact_meta[1];
            pixel_rgb <= win_q ? v_rgb : bg_q_en ? b_rgb : 24'h000000;
        end
    end
endmodule
