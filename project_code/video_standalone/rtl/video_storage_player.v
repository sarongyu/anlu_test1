`timescale 1ns/1ps

// Plays VID1.RAW (320x240 RGB332, 76800 bytes per frame, any whole number
// of frames) from FAT32 at 10 fps through two SDRAM frame banks, looping,
// and composes it over the master's picture (bg_line_*), see
// video_sdram_frames. The same card holds AUDIO1.WAV and AUDIO2.WAV, read
// by wav_sd_stream when wav_run is set (wav_select: 0 AUDIO1, 1 AUDIO2).
// One SD reader serves both: sector requests are queued and granted one at
// a time, audio first (its FIFO is the tighter deadline), and each
// requester only sees the data and completion of its own sectors.
// The SD data phase runs at mem_clk/(2*SPI_CLK_DIV) = 20 MHz.
module video_storage_player #(
    parameter integer SPI_CLK_DIV = 1
)(
    input wire mem_clk, mem_clk_shift, mem_rst_n, pixel_clk, pixel_rst_n,
    input wire [11:0] x, y,
    input wire de, fullscreen,
    output wire [23:0] pixel_rgb,
    output wire image_ready, frame_shown,
    output reg [7:0] error_code,
    input  wire bg_line_ready,
    input  wire [8:0] bg_line_no,
    output wire [8:0] bg_line_raddr,
    input  wire [31:0] bg_line_rdata,
    output wire bg_line_done, bg_active,
    // WAV reader (mem_clk domain except wav_run / wav_select)
    input  wire wav_run, wav_select,
    input  wire wav_room,
    output wire wav_pcm_valid,
    output wire [32:0] wav_pcm_data,
    output wire wav_stopped,
    output wire [7:0] wav_error,
    output wire sd_cs_n, sd_sclk, sd_mosi,
    input wire sd_miso
);
    localparam integer FRAME_BYTES = 76800;
    reg init_start, scan_start, file_start;
    reg [2:0] state;
    localparam BOOT=0, INIT=1, SCAN=2, PLAY=3, FAIL=5;
    wire init_done, init_ready, high_capacity;
    wire [7:0] init_error, sector_error, scan_error, file_error;
    wire sector_req, sector_busy, sector_done, sector_valid;
    wire [31:0] sector_lba;
    wire [7:0] sector_byte;
    wire [8:0] sector_byte_index;
    wire scan_req, file_req, scan_busy, scan_done;
    wire [31:0] scan_lba, file_lba;
    wire [3:0] resource_valid;
    wire [31:0] first_cluster, file_size, fat_start_lba, data_start_lba, max_cluster;
    wire [31:0] a1_cluster, a1_size, a2_cluster, a2_size;
    wire [7:0] sectors_per_cluster;
    wire file_busy, file_done, file_valid, sector_allow, frame_partial;
    wire [7:0] file_byte;
    wire [31:0] file_byte_offset, frames_written;
    wire [7:0] frame_error;

    // ------------------------------------------------------------------
    // SD sector arbiter: video side (scanner, then file streamer) and the
    // WAV reader. Requests are single-clock pulses and are latched.
    // ------------------------------------------------------------------
    wire a_req;
    wire [31:0] a_lba;
    reg pend_v, pend_a;
    reg [31:0] lba_v, lba_a;
    reg [1:0] owner;                 // 0 none, 1 video, 2 audio
    reg sd_req;
    reg [31:0] sd_lba;
    wire v_req = scan_req | file_req;
    always @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin
            pend_v <= 0; pend_a <= 0; lba_v <= 0; lba_a <= 0;
            owner <= 0; sd_req <= 0; sd_lba <= 0;
        end else begin
            sd_req <= 0;
            if (sector_done) owner <= 0;
            if (owner == 0 && !sector_busy && !sd_req) begin
                if (pend_a) begin
                    owner <= 2; pend_a <= 0; sd_req <= 1; sd_lba <= lba_a;
                end else if (pend_v) begin
                    owner <= 1; pend_v <= 0; sd_req <= 1; sd_lba <= lba_v;
                end
            end
            // A requester pulses once and waits for its own completion.
            if (v_req) begin pend_v <= 1; lba_v <= scan_req ? scan_lba : file_lba; end
            if (a_req) begin pend_a <= 1; lba_a <= a_lba; end
        end
    end
    assign sector_req = sd_req;
    assign sector_lba = sd_lba;
    wire v_busy = pend_v || owner == 2'd1;
    wire v_valid = sector_valid && owner == 2'd1;
    wire v_done = sector_done && owner == 2'd1;
    wire a_busy = pend_a || owner == 2'd2;
    wire a_valid = sector_valid && owner == 2'd2;
    wire a_done = sector_done && owner == 2'd2;

    sd_media_service #(.SPI_CLK_DIV(SPI_CLK_DIV)) u_sd (
        .clk(mem_clk), .rst_n(mem_rst_n), .init_start(init_start),
        .init_busy(), .init_done(init_done), .init_ready(init_ready),
        .init_error(init_error), .card_high_capacity(high_capacity),
        .card_ocr(), .owner_reader(), .sector_req(sector_req),
        .sector_lba(sector_lba), .sector_busy(sector_busy),
        .sector_done(sector_done), .sector_valid(sector_valid),
        .sector_byte(sector_byte), .sector_byte_index(sector_byte_index),
        .sector_error(sector_error), .sd_cs_n(sd_cs_n),
        .sd_sclk(sd_sclk), .sd_mosi(sd_mosi), .sd_miso(sd_miso)
    );
    fat32_volume_scanner #(.REQUIRE_ALL_IMAGES(0)) u_scan (
        .clk(mem_clk), .rst_n(mem_rst_n), .scan_start(scan_start),
        .scan_busy(scan_busy), .scan_done(scan_done), .fs_error(scan_error),
        .partition_lba(), .sectors_per_cluster(sectors_per_cluster),
        .fat_start_lba(fat_start_lba), .data_start_lba(data_start_lba),
        .max_cluster(max_cluster), .audio_valid(), .audio_cluster(), .audio_size(),
        .resource_valid(resource_valid), .resource0_cluster(first_cluster),
        .resource1_cluster(a1_cluster), .resource2_cluster(a2_cluster), .resource3_cluster(),
        .resource0_size(file_size), .resource1_size(a1_size), .resource2_size(a2_size),
        .resource3_size(), .sector_req(scan_req), .sector_lba(scan_lba),
        .sector_busy(v_busy), .sector_valid(v_valid),
        .sector_byte(sector_byte), .sector_byte_index(sector_byte_index),
        .sector_done(v_done), .sector_error(sector_error)
    );
    fat32_file_streamer #(.MAX_VISITED_CLUSTERS(2),.TRACK_HISTORY(0)) u_file (
        .clk(mem_clk), .rst_n(mem_rst_n), .file_start(file_start),
        .sector_allow(sector_allow), .first_cluster(first_cluster),
        .file_size(file_size), .sectors_per_cluster(sectors_per_cluster),
        .fat_start_lba(fat_start_lba), .data_start_lba(data_start_lba),
        .max_cluster(max_cluster), .file_busy(file_busy), .file_done(file_done),
        .file_valid(file_valid), .file_byte(file_byte),
        .file_byte_offset(file_byte_offset), .fs_error(file_error),
        .sector_req(file_req), .sector_lba(file_lba), .sector_busy(v_busy),
        .sector_valid(v_valid), .sector_byte(sector_byte),
        .sector_byte_index(sector_byte_index), .sector_done(v_done),
        .sector_error(sector_error)
    );

    // WAV reader. The file choice is taken over only while the reader is
    // stopped, so a running stream never sees its cluster or size change.
    reg [1:0] sel_meta;
    reg sel_mem;
    always @(posedge mem_clk or negedge mem_rst_n)
        if (!mem_rst_n) begin sel_meta <= 0; sel_mem <= 0; end
        else begin
            sel_meta <= {sel_meta[0], wav_select};
            if (wav_stopped) sel_mem <= sel_meta[1];
        end
    wire storage_ready = state == PLAY;
    wire [31:0] pcm_word;
    wire pcm_last;
    assign wav_pcm_data = {pcm_last, pcm_word};
    wav_sd_stream u_wav (.clk(mem_clk), .rst_n(mem_rst_n), .run_request(wav_run),
        .storage_ready(storage_ready),
        .audio_valid(sel_mem ? resource_valid[2] : resource_valid[1]),
        .audio_cluster(sel_mem ? a2_cluster : a1_cluster),
        .audio_size(sel_mem ? a2_size : a1_size),
        .fat_start_lba(fat_start_lba), .data_start_lba(data_start_lba),
        .max_cluster(max_cluster), .sectors_per_cluster(sectors_per_cluster),
        .room_sector(wav_room),
        .pcm_valid(wav_pcm_valid), .pcm_data(pcm_word), .pcm_last(pcm_last),
        .stopped(wav_stopped), .eof(), .error(wav_error),
        .reader_req(a_req), .reader_lba(a_lba),
        .reader_busy(a_busy || !storage_ready),
        .reader_valid(a_valid), .reader_done(a_done),
        .reader_byte(sector_byte), .reader_byte_index(sector_byte_index),
        .reader_error(sector_error));

    wire sdr_init_done, sdr_refresh, sdr_busy, sdr_rd_valid;
    wire app_wr_en, app_rd_en;
    wire [20:0] app_wr_addr, app_rd_addr;
    wire [31:0] app_wr_data, sdr_rd_data;
    sdram u_sdram (
        .Clk(mem_clk), .Clk_sft(mem_clk_shift), .Rst(~mem_rst_n),
        .Sdr_init_done(sdr_init_done), .Sdr_init_ref_vld(sdr_refresh),
        .Sdr_busy(sdr_busy),
        .App_wr_en(app_wr_en), .App_wr_addr(app_wr_addr),
        .App_wr_dm(4'b0000), .App_wr_din(app_wr_data),
        .App_rd_en(app_rd_en), .App_rd_addr(app_rd_addr),
        .Sdr_rd_en(sdr_rd_valid), .Sdr_rd_dout(sdr_rd_data)
    );
    video_sdram_frames u_frames (
        .mem_clk(mem_clk), .mem_rst_n(mem_rst_n),
        .byte_valid(file_valid), .byte_data(file_byte),
        .sector_allow(sector_allow), .frame_partial(frame_partial),
        .frames_written(frames_written), .frame_error(frame_error),
        .sdr_init_done(sdr_init_done), .sdr_busy(sdr_busy), .sdr_refresh(sdr_refresh),
        .app_wr_en(app_wr_en), .app_wr_addr(app_wr_addr), .app_wr_data(app_wr_data),
        .app_rd_en(app_rd_en), .app_rd_addr(app_rd_addr),
        .sdr_rd_valid(sdr_rd_valid), .sdr_rd_data(sdr_rd_data),
        .bg_line_ready(bg_line_ready), .bg_line_no(bg_line_no),
        .bg_line_raddr(bg_line_raddr), .bg_line_rdata(bg_line_rdata),
        .bg_line_done(bg_line_done), .bg_active(bg_active), .bg_frames(),
        .pixel_clk(pixel_clk), .pixel_rst_n(pixel_rst_n),
        .x(x), .y(y), .de(de), .fullscreen(fullscreen),
        .pixel_rgb(pixel_rgb), .image_ready(image_ready),
        .frame_shown_toggle(frame_shown)
    );

    // Error codes: 10 card init, 80 VID1.RAW missing or shorter than one
    // frame, 82 read error or length not a whole number of frames,
    // 86/87/88 frame store (late byte, FIFO overflow, line fetch).
    always @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin
            state <= BOOT; init_start <= 0; scan_start <= 0;
            file_start <= 0; error_code <= 0;
        end else begin
            init_start <= 0; scan_start <= 0; file_start <= 0;
            if (frame_error != 0) error_code <= frame_error;
            case (state)
                BOOT: if (sdr_init_done) begin init_start <= 1; state <= INIT; end
                INIT: if (init_done) begin
                    if (init_error != 0 || !init_ready) begin
                        error_code <= 8'h10; state <= FAIL;
                    end else begin scan_start <= 1; state <= SCAN; end
                end
                SCAN: if (scan_done) begin
                    if (scan_error != 0 || !resource_valid[0] || file_size < FRAME_BYTES) begin
                        error_code <= 8'h80; state <= FAIL;
                    end else begin file_start <= 1; state <= PLAY; end
                end
                PLAY: if (file_done) begin
                    if (file_error != 0 || frame_partial) begin
                        error_code <= 8'h82; state <= FAIL;
                    end else file_start <= 1;   // loop
                end
                default: state <= state;
            endcase
        end
    end
endmodule
