`timescale 1ns/1ps

module media_video_pipeline #(
    parameter integer IMAGE_WORDS = 307200,
    parameter integer FIFO_DEPTH = 512,
    parameter integer H_ACT = 640,
    parameter integer H_FP = 16,
    parameter integer H_SYNC = 96,
    parameter integer H_BP = 48,
    parameter integer V_ACT = 480,
    parameter integer V_FP = 10,
    parameter integer V_SYNC = 2,
    parameter integer V_BP = 33,
    parameter [31:0] DEFAULT_PERIOD_CYCLES = 32'd250000000,
    parameter [31:0] MIN_PERIOD_CYCLES = 32'd50000000,
    parameter [31:0] MAX_PERIOD_CYCLES = 32'd3000000000,
    parameter integer RESTORE_ON_BOOT = 0,
    // Non-zero enables the fade-out/black/fade-in transition on manual
    // switches and a fade-in on automatic slideshow switches. Zero keeps the
    // legacy direct-switch behaviour for the existing regression suites.
    parameter integer FADE_FRAMES = 0
) (
    input  wire        ctrl_clk,
    input  wire        ctrl_rst_n,
    input  wire        next_event,
    input  wire        toggle_auto_event,
    input  wire        period_cfg_valid,
    input  wire [31:0] period_cfg_cycles,
    input  wire        auto_cfg_valid,
    input  wire        auto_cfg_enabled,
    input  wire        restore_valid,
    input  wire [1:0]  restore_index,
    input  wire        restore_ready,
    input  wire [3:0]  image_complete,
    input  wire        player_enable,
    input  wire        preview_toggle,
    output wire        preview_mode_out,

    input  wire        mem_clk,
    input  wire        mem_rst_n,
    input  wire        controller_init_done,
    input  wire        controller_busy,
    input  wire        refresh_active,
    output wire        memory_request, memory_active,
    output wire        app_rd_en,
    output wire [20:0] app_rd_addr,
    input  wire        sdram_rd_valid,
    input  wire [31:0] sdram_rd_data,

    input  wire [7:0]  brightness,
    input  wire        pix_clk,
    input  wire        pix_rst_n,
    output wire        video_clk,
    output wire        video_hs,
    output wire        video_vs,
    output wire        video_de,
    output wire [23:0] video_rgb,
    output wire [9:0]  video_x,

    output wire manual_audio_commit,
    output wire [1:0]  active_index,
    output wire [1:0]  pending_index,
    output wire        auto_enabled,
    output wire        restore_done,
    output wire        image_index_commit,
    output wire        auto_image_commit_out,
    output wire        frame_start,
    output wire        reader_busy,
    output wire        reader_underflow,
    output wire [7:0]  video_error
);
    wire raw_hs;
    wire raw_vs;
    wire raw_de;
    wire [11:0] raw_x;
    wire [11:0] raw_y;
    reg raw_vs_q;
    reg [9:0] video_x_q;
    wire index_commit;
    assign image_index_commit = index_commit;
    wire [7:0] control_error;
    wire request_busy;
    wire [31:0] active_period_cycles;
    wire [23:0] reader_rgb;
    wire reader_pixel_valid;
    wire reader_done;
    wire [1:0] reader_index;
    wire [7:0] reader_error;
    wire player_enable_pix;
    reg preview_s1,preview_s2,preview_seen,preview_mode_pix;
    reg [3:0] preview_ready_s1,preview_ready_s2;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            preview_s1<=0;preview_s2<=0;preview_seen<=0;preview_mode_pix<=0;
            preview_ready_s1<=0;preview_ready_s2<=0;
        end else begin
            preview_s1<=preview_toggle;
            preview_s2<=preview_s1;
            preview_ready_s1<=image_complete;
            preview_ready_s2<=preview_ready_s1;
            if (frame_start && preview_s2 != preview_seen) begin
                preview_seen<=preview_s2;
                preview_mode_pix<=~preview_mode_pix;
            end
        end
    end
    assign preview_mode_out=preview_mode_pix;
    reg [7:0] reader_error_pix1;
    reg [7:0] reader_error_pix2;
    reg [7:0] control_error_pix1;
    reg [7:0] control_error_pix2;
    // The fifth SDRAM frame is only displayed once all four preview tiles
    // have finished writing. This avoids showing uninitialized SDRAM data.
    wire preview_visible = !preview_mode_pix || (&preview_ready_s2);
    wire [23:0] selected_rgb =
        (player_enable_pix && reader_pixel_valid && reader_error_pix2 == 0 && preview_visible) ?
        reader_rgb : 24'h000000;

    sync_2ff u_player_enable_pix (
        .clk(pix_clk), .rst_n(pix_rst_n), .async_in(player_enable),
        .sync_out(player_enable_pix)
    );

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            reader_error_pix1 <= 8'd0;
            reader_error_pix2 <= 8'd0;
            control_error_pix1 <= 8'd0;
            control_error_pix2 <= 8'd0;
        end else begin
            reader_error_pix1 <= reader_error;
            reader_error_pix2 <= reader_error_pix1;
            control_error_pix1 <= control_error;
            control_error_pix2 <= control_error_pix1;
        end
    end

    video_timing_ctrl #(
        .H_ACT(H_ACT), .H_FP(H_FP), .H_SYNC(H_SYNC), .H_BP(H_BP),
        .V_ACT(V_ACT), .V_FP(V_FP), .V_SYNC(V_SYNC), .V_BP(V_BP)
    ) u_timing (
        .pclk(pix_clk), .rst_n(pix_rst_n), .hsync(raw_hs),
        .vsync(raw_vs), .de(raw_de), .x(raw_x), .y(raw_y)
    );

    // Fade transition: manual switch requests are held until the display has
    // faded to black, then released to the control FSM at the frame boundary.
    // The bypass branch (FADE_FRAMES=0) wires next_event straight through so
    // legacy regression timing is unchanged.
    wire fade_next_event;
    wire [7:0] fade_alpha;
    wire wipe_active;
    wire [4:0] wipe_ramp;
    wire [1:0] reveal_kind;
    wire auto_image_commit;
    assign auto_image_commit_out = auto_image_commit;
    video_fade_transition #(.FADE_FRAMES(FADE_FRAMES)) u_fade (
        .ctrl_clk(ctrl_clk), .ctrl_rst_n(ctrl_rst_n),
        .next_event_in(next_event), .next_event_out(fade_next_event),
        .fade_busy(),
        .pix_clk(pix_clk), .pix_rst_n(pix_rst_n),
        .player_active(player_enable_pix), .frame_start(frame_start),
        .index_commit(index_commit), .auto_image_commit(auto_image_commit),
        .auto_enabled(auto_enabled), .alpha_out(fade_alpha), .fade_active(),
        .wipe_active(wipe_active), .ramp_out(wipe_ramp), .reveal_kind(reveal_kind)
    );

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n)
            raw_vs_q <= 1'b1;
        else
            raw_vs_q <= raw_vs;
    end
    assign frame_start = raw_vs_q && !raw_vs;

    // rgb_video_adapter registers hs/vs/de/rgb by one pixel. Register x in
    // parallel so the HDMI Data Island scheduler sees the coordinate belonging
    // to the same output character, rather than the next raw VTC position.
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n)
            video_x_q <= 10'd0;
        else
            video_x_q <= raw_x[9:0];
    end
    assign video_x = video_x_q;

    media_player_control #(
        .DEFAULT_PERIOD_CYCLES(DEFAULT_PERIOD_CYCLES),
        .MIN_PERIOD_CYCLES(MIN_PERIOD_CYCLES),
        .MAX_PERIOD_CYCLES(MAX_PERIOD_CYCLES),
        .RESTORE_ON_BOOT(RESTORE_ON_BOOT)
    ) u_control (
        .sys_clk(ctrl_clk), .sys_rst_n(ctrl_rst_n),
        .next_event(fade_next_event), .toggle_auto_event(toggle_auto_event),
        .period_cfg_valid(period_cfg_valid),
        .period_cfg_cycles(period_cfg_cycles),
        .auto_cfg_valid(auto_cfg_valid),
        .auto_cfg_enabled(auto_cfg_enabled),
        .restore_valid(restore_valid), .restore_index(restore_index),
        .restore_ready(restore_ready),
        .image_complete(image_complete), .auto_enabled(auto_enabled),
        .restore_done(restore_done),
        .active_period_cycles(active_period_cycles),
        .pending_index(pending_index), .control_error(control_error),
        .request_busy(request_busy), .pix_clk(pix_clk),
        .pix_rst_n(pix_rst_n), .frame_start(frame_start),
        .manual_audio_commit(manual_audio_commit), .auto_image_commit(auto_image_commit),
        .active_index(active_index), .index_commit(index_commit)
    );

    media_frame_reader #(
        .ADDR_WIDTH(21), .IMAGE_WORDS(IMAGE_WORDS),
        .FIFO_DEPTH(FIFO_DEPTH), .FIFO_MARGIN(4)
    ) u_reader (
        .mem_clk(mem_clk), .mem_rst_n(mem_rst_n),
        .controller_init_done(controller_init_done && player_enable),
        .controller_busy(controller_busy), .refresh_active(refresh_active),
        .memory_request(memory_request), .memory_active(memory_active), .app_rd_en(app_rd_en), .app_rd_addr(app_rd_addr),
        .sdram_rd_valid(sdram_rd_valid), .sdram_rd_data(sdram_rd_data),
        .pix_clk(pix_clk), .pix_rst_n(pix_rst_n),
        .frame_start(frame_start && player_enable_pix), .active_index(active_index),
        .preview_mode(preview_mode_pix),
        .pixel_consume(raw_de && player_enable_pix), .pixel_rgb(reader_rgb),
        .pixel_valid(reader_pixel_valid), .underflow(reader_underflow),
        .reader_busy(reader_busy), .frame_read_done(reader_done),
        .reader_index(reader_index), .reader_error(reader_error)
    );

    // Fade multiplies the selected frame by the ramp alpha; an automatic
    // slideshow switch instead reveals the new frame left-to-right: pixels
    // right of the moving boundary stay black, the rest show full colour.
    // With FADE_FRAMES=0 the direct assignment keeps every pixel bit-identical
    // to the legacy path so strict regression comparisons still hold.
    wire [23:0] faded_rgb;
    generate
        if (FADE_FRAMES == 0) begin : g_fade_bypass
            // Session brightness applies in the bypass path too. The 9-bit
            // (brightness+1) factor keeps brightness=255 bit-exact (256/256),
            // so legacy regression vectors and the direct-switch behaviour
            // are unchanged at full brightness.
            wire [8:0] eff_bypass = {1'b0, brightness} + 9'd1;
            wire [16:0] bm_r = {9'd0, selected_rgb[23:16]} * eff_bypass;
            wire [16:0] bm_g = {9'd0, selected_rgb[15:8]} * eff_bypass;
            wire [16:0] bm_b = {9'd0, selected_rgb[7:0]} * eff_bypass;
            assign faded_rgb = {bm_r[15:8], bm_g[15:8], bm_b[15:8]};
        end else begin : g_fade_mul
            // Session-level brightness folds into the transition alpha with
            // one extra 8x8 multiply, so the steady display and the fade both
            // scale together.
            // (brightness+1)/256 keeps full brightness bit-exact (256/256)
            // while lower steps land within 0.4% of brightness/255.
            // Nine-bit unity (256) preserves exact pixels at full brightness.
            // Alpha zero must remain fully black.
            wire [8:0] bright_scale = {1'b0, brightness} + 9'd1;
            wire [8:0] fade_scale = fade_alpha == 0 ? 9'd0 : {1'b0, fade_alpha} + 9'd1;
            wire [17:0] scale_product = bright_scale * fade_scale;
            wire [8:0] eff_alpha = preview_mode_pix ? 9'd256 : scale_product[17:8];
            wire [16:0] fade_mul_r = {9'd0, selected_rgb[23:16]} * eff_alpha;
            wire [16:0] fade_mul_g = {9'd0, selected_rgb[15:8]}  * eff_alpha;
            wire [16:0] fade_mul_b = {9'd0, selected_rgb[7:0]}   * eff_alpha;
            wire [23:0] alpha_rgb =
                {fade_mul_r[15:8], fade_mul_g[15:8], fade_mul_b[15:8]};
            // Reveal boundary advances one H_ACT/FADE_FRAMES column step per
            // frame; the comparison uses the same raw_x that aligns with
            // selected_rgb at this point in the pipeline.
            localparam integer WIPE_X_STEP = H_ACT / FADE_FRAMES;
            localparam integer WIPE_Y_STEP = V_ACT / FADE_FRAMES;
            wire [11:0] wipe_x_boundary = wipe_ramp * WIPE_X_STEP;
            wire [11:0] wipe_y_boundary = wipe_ramp * WIPE_Y_STEP;
            wire [11:0] center_x_radius = wipe_ramp * (H_ACT / (2 * FADE_FRAMES));
            wire [11:0] center_y_radius = wipe_ramp * (V_ACT / (2 * FADE_FRAMES));
            wire reveal_pixel = (reveal_kind == 2'd0) ? (raw_x < wipe_x_boundary) :
                                (reveal_kind == 2'd1) ? (raw_y < wipe_y_boundary) :
                                (raw_x >= H_ACT/2 - center_x_radius &&
                                 raw_x < H_ACT/2 + center_x_radius &&
                                 raw_y >= V_ACT/2 - center_y_radius &&
                                 raw_y < V_ACT/2 + center_y_radius);
            assign faded_rgb = (wipe_active && !preview_mode_pix) ?
                (reveal_pixel ? alpha_rgb : 24'h000000) :
                alpha_rgb;
        end
    endgenerate

    rgb_video_adapter u_video_adapter (
        .pclk(pix_clk), .rst_n(pix_rst_n), .hs_in(raw_hs),
        .vs_in(raw_vs), .de_in(raw_de),
        .rgb_in(faded_rgb), .video_clk(video_clk),
        .video_hs(video_hs), .video_vs(video_vs),
        .video_de(video_de), .video_rgb(video_rgb)
    );

    assign video_error = reader_error_pix2 != 0 ? reader_error_pix2 :
                         reader_underflow ? 8'h63 : control_error_pix2;
endmodule
