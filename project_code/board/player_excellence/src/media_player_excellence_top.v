`timescale 1ns/1ps

// Excellence top: everything in the final competition candidate plus the
// KEY4 parameter menu (short press = volume ladder, one-second hold =
// slideshow period ladder) and the UART status reporter (boot banner, per
// switch lines, parameter changes, error codes) on the board USB-UART.
module media_player_excellence_top (
    input  wire       sys_clk,
    input  wire       sys_rst_n,
    input  wire       key2_n,
    input  wire       key3_n,
    input  wire       key4_n,
    input  wire       eth_rx_clk,
    input  wire       eth_rx_ctl,
    input  wire [3:0] eth_rx_d,
    output wire       eth_tx_clk,
    output wire       eth_tx_ctl,
    output wire [3:0] eth_tx_d,
    output wire       uart_tx_o,
    output wire       sd_cs_n,
    output wire       sd_sclk,
    output wire       sd_mosi,
    input  wire       sd_miso,
    input  wire       flash_miso,
    output wire       flash_cs_n,
    output wire       flash_sclk,
    output wire       flash_mosi,
    output wire       flash_wp_n,
    output wire       flash_hold_n,
    output wire       HDMI_CLK_P,
    output wire       HDMI_D2_P,
    output wire       HDMI_D1_P,
    output wire       HDMI_D0_P,
    output wire [3:0] led
);
    wire mem_feedback_clk;
    wire mem_clk;
    wire mem_clk_shift;
    wire mem_locked;
    sys_pll u_mem_pll (
        .refclk(sys_clk), .reset(~sys_rst_n),
        .clk0_out(mem_feedback_clk), .clk1_out(mem_clk),
        .clk2_out(mem_clk_shift), .locked(mem_locked)
    );

    wire pixel_clk_5x;
    wire pixel_clk;
    wire video_locked;
    clk_wize u_video_pll (
        .refclk(sys_clk), .reset(~sys_rst_n),
        .clk0_out(pixel_clk_5x), .clk1_out(pixel_clk),
        .locked(video_locked)
    );

    wire mem_rst_n;
    wire pix_rst_n;
    wire pix_5x_rst_n;
    wire ctrl_rst_n;
    reset_sync u_mem_reset (
        .clk(mem_clk), .ext_rst_n(sys_rst_n & mem_locked),
        .rst_n_sync(mem_rst_n)
    );
    reset_sync u_pix_reset (
        .clk(pixel_clk), .ext_rst_n(sys_rst_n & video_locked),
        .rst_n_sync(pix_rst_n)
    );
    reset_sync u_pix_5x_reset (
        .clk(pixel_clk_5x), .ext_rst_n(sys_rst_n & video_locked),
        .rst_n_sync(pix_5x_rst_n)
    );
    reset_sync u_ctrl_reset (
        .clk(sys_clk), .ext_rst_n(sys_rst_n & video_locked),
        .rst_n_sync(ctrl_rst_n)
    );

    wire sdram_init_done;
    wire sdram_refresh_active;
    wire sdram_busy;
    wire write_request, write_active, read_request, read_active;
    wire writer_blocked, reader_blocked;
    sdram_burst_arbiter u_memory_arbiter (
        .clk(mem_clk), .rst_n(mem_rst_n), .controller_busy(sdram_busy),
        .refresh_active(sdram_refresh_active), .write_request(write_request),
        .read_request(read_request), .write_active(write_active), .read_active(read_active),
        .writer_blocked(writer_blocked), .reader_blocked(reader_blocked));
    wire ingest_app_wr_en;
    wire [20:0] ingest_app_wr_addr;
    wire [3:0] ingest_app_wr_mask;
    wire [31:0] ingest_app_wr_data;
    wire player_app_rd_en;
    wire [20:0] player_app_rd_addr;
    wire sdram_rd_valid;
    wire [31:0] sdram_rd_data;

    sdram u_sdram (
        .Clk(mem_clk), .Clk_sft(mem_clk_shift), .Rst(~mem_rst_n),
        .Sdr_init_done(sdram_init_done),
        .Sdr_init_ref_vld(sdram_refresh_active), .Sdr_busy(sdram_busy),
        .App_wr_en(ingest_app_wr_en), .App_wr_addr(ingest_app_wr_addr),
        .App_wr_dm(ingest_app_wr_mask), .App_wr_din(ingest_app_wr_data),
        .App_rd_en(player_app_rd_en), .App_rd_addr(player_app_rd_addr),
        .Sdr_rd_en(sdram_rd_valid), .Sdr_rd_dout(sdram_rd_data)
    );

    reg ingest_start;
    reg ingest_started;
    always @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin
            ingest_start <= 1'b0;
            ingest_started <= 1'b0;
        end else begin
            ingest_start <= 1'b0;
            if (sdram_init_done && !ingest_started) begin
                ingest_start <= 1'b1;
                ingest_started <= 1'b1;
            end
        end
    end

    wire ingest_busy;
    wire ingest_done;
    wire [2:0] ingest_error_stage;
    wire [7:0] ingest_error_code;
    wire [7:0] ingest_error_detail;
    wire card_ready;
    wire card_high_capacity;
    wire [3:0] resource_valid;
    wire [3:0] image_complete;
    reg player_enable_mem;
    wire restore_done_sys;
    wire restore_done_mem;
    wire playback_restored_sys;
    sync_2ff u_restore_done_mem (
        .clk(mem_clk), .rst_n(mem_rst_n),
        .async_in(playback_restored_sys), .sync_out(restore_done_mem)
    );
    wire wav_run,wav_stopped,wav_write,wav_room,wav_pop,wav_valid,wav_prefilled;
    wire [32:0] wav_write_data,wav_data;wire [7:0] wav_error;
    wire wav_stopped_pix,wav_fault_pix,player_enable_pix_audio;
    wire manual_audio_commit;
    sync_2ff u_wav_stopped(.clk(pixel_clk),.rst_n(pix_rst_n),.async_in(wav_stopped),.sync_out(wav_stopped_pix));
    sync_2ff u_wav_fault(.clk(pixel_clk),.rst_n(pix_rst_n),.async_in(|wav_error),.sync_out(wav_fault_pix));
    sync_2ff u_audio_enable(.clk(pixel_clk),.rst_n(pix_rst_n),.async_in(player_enable_mem),.sync_out(player_enable_pix_audio));
    wire wav_wrst,wav_rrst;
    reset_sync u_wav_wreset(.clk(mem_clk),.ext_rst_n(mem_rst_n&&!wav_stopped),.rst_n_sync(wav_wrst));
    reset_sync u_wav_rreset(.clk(pixel_clk),.ext_rst_n(pix_rst_n&&wav_run&&!wav_stopped_pix),.rst_n_sync(wav_rrst));
    pcm_async_fifo u_wav_fifo(.wc(mem_clk),.wrst(wav_wrst),
      .wen(wav_write),.din(wav_write_data),.room_sector(wav_room),
      .rc(pixel_clk),.rrst(wav_rrst),
      .pop(wav_pop),.valid(wav_valid),.dout(wav_data),.prefilled(wav_prefilled));
    media_storage_ingest_top u_storage_ingest (
        .audio_run(wav_run),.audio_stopped(wav_stopped),.audio_pcm_valid(wav_write),
        .audio_pcm_data(wav_write_data),.audio_room_sector(wav_room),.audio_error(wav_error),
        .clk(mem_clk), .rst_n(mem_rst_n), .start(ingest_start),
        .busy(ingest_busy), .done(ingest_done),
        .error_stage(ingest_error_stage), .error_code(ingest_error_code),
        .error_detail(ingest_error_detail), .card_ready(card_ready),
        .card_high_capacity(card_high_capacity),
        .resource_valid(resource_valid), .image_complete(image_complete),
        .controller_init_done(sdram_init_done),
        .controller_busy(writer_blocked), .refresh_active(sdram_refresh_active),
        .memory_request(write_request), .memory_active(write_active),
        .app_wr_en(ingest_app_wr_en), .app_wr_addr(ingest_app_wr_addr),
        .app_wr_mask(ingest_app_wr_mask), .app_wr_data(ingest_app_wr_data),
        .sd_cs_n(sd_cs_n), .sd_sclk(sd_sclk),
        .sd_mosi(sd_mosi), .sd_miso(sd_miso)
    );

    // Display the first completed bank while later banks load under burst arbitration.
    reg ingest_finished;
    wire [1:0] active_index_mem;
    genvar ready_bit;
    generate for (ready_bit=0; ready_bit<2; ready_bit=ready_bit+1) begin: g_ready_index
        sync_2ff u_ready_index(.clk(mem_clk), .rst_n(mem_rst_n),
            .async_in(active_index[ready_bit]), .sync_out(active_index_mem[ready_bit]));
    end endgenerate
    wire storage_error_mem;
    always @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin
            player_enable_mem <= 1'b0;
            ingest_finished <= 1'b0;
        end else begin
            if (ingest_done) ingest_finished <= 1'b1;
            if (restore_done_mem && image_complete[active_index_mem])
                player_enable_mem <= 1'b1;
        end
    end

    wire key2_press;
    wire key3_press;
    wire config_save_event;
    reg playback_save_pulse;
    reg playback_save_pending;
    reg playback_auto_dirty;
    reg [33:0] playback_checkpoint_cycles;
    reg [1:0] playback_last_index;
    reg playback_resume_started;
    reg [31:0] playback_index_settle;
    reg [1:0] playback_saved_index;
    reg playback_saved_mode;
    reg [31:0] playback_saved_period;
    reg [7:0] playback_saved_volume;
    reg playback_saved_valid;
    reg playback_save_inflight;
    reg playback_pending_after_save;
    reg playback_manual_during_save;
    reg playback_auto_checkpoint_pending;
    reg playback_saved_index_pending;
    reg playback_loaded_snapshot;
    wire [1:0] active_index_sys;
    wire image_index_commit;
    wire auto_image_commit_pix;
    reg [1:0] active_index_pix_q;
    reg [1:0] committed_index_pix;
    reg committed_toggle_pix;
    wire [1:0] committed_index_sync;
    wire committed_toggle_sync;
    reg committed_toggle_seen;
    reg [1:0] playback_index_sys;
    reg committed_auto_pix;
    wire committed_auto_sync;
    reg playback_commit_auto_sys;
    always @(posedge pixel_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            active_index_pix_q <= 2'd0;
            committed_index_pix <= 2'd0;
            committed_toggle_pix <= 1'b0;
            committed_auto_pix <= 1'b0;
        end else begin
            active_index_pix_q <= active_index;
            if (image_index_commit && active_index != active_index_pix_q) begin
                committed_index_pix <= active_index;
                committed_toggle_pix <= ~committed_toggle_pix;
                committed_auto_pix <= auto_image_commit_pix;
            end
        end
    end
    sync_2ff u_commit_toggle_sync (
        .clk(sys_clk), .rst_n(ctrl_rst_n),
        .async_in(committed_toggle_pix), .sync_out(committed_toggle_sync)
    );
    sync_2ff u_commit_auto_sync (
        .clk(sys_clk), .rst_n(ctrl_rst_n),
        .async_in(committed_auto_pix), .sync_out(committed_auto_sync)
    );
    genvar commit_bit;
    generate for (commit_bit=0; commit_bit<2; commit_bit=commit_bit+1) begin: g_commit_index_sync
        sync_2ff u_sync (.clk(sys_clk), .rst_n(ctrl_rst_n),
            .async_in(committed_index_pix[commit_bit]),
            .sync_out(committed_index_sync[commit_bit]));
    end endgenerate
    always @(posedge sys_clk or negedge ctrl_rst_n) begin
        if (!ctrl_rst_n) begin
            committed_toggle_seen <= 1'b0;
            playback_index_sys <= 2'd0;
            playback_commit_auto_sys <= 1'b0;
        end else if (committed_toggle_seen != committed_toggle_sync) begin
            committed_toggle_seen <= committed_toggle_sync;
            playback_index_sys <= committed_index_sync;
            playback_commit_auto_sys <= committed_auto_sync;
        end
    end
    wire config_save_locked;
    wire config_ready;
    wire config_busy;
    wire config_load_done;
    wire config_save_done;
    wire config_apply_valid;
    wire config_rejected;
    wire config_fatal_error;
    wire [7:0] config_error_code;
    wire config_request_dropped;
    wire stored_config_valid;
    wire [31:0] stored_config_version;
    wire [31:0] config_period_ms;
    wire [31:0] config_period_cycles;
    wire [7:0] config_volume;
    wire [7:0] config_mode;
    wire [7:0] requested_config_mode;
    wire [31:0] stored_period_ms;
    wire [7:0] stored_volume;
    wire [7:0] stored_mode;
    wire [1:0] stored_index;
    wire playback_mode_dirty;
    wire [31:0] saving_period_ms;
    wire [7:0] saving_volume;
    wire [7:0] saving_mode;
    wire [1:0] saving_index;
    wire [23:0] config_jedec_id;
    wire [7:0] config_status1, config_status2, config_status3;
    wire auto_enabled;

    // Human-operated chord contract: a single key acts on release; while the
    // first key remains held, the second key may join and start a 2 s save hold.
    wire key3_long_press;
    wire chord_preview_toggle;

    dual_key_chord_router #(
        .DEBOUNCE_CYCLES(1000000),
        .HOLD_CYCLES(100000000),
        .KEY3_LONG_CYCLES(50000000)
    ) u_key_router (
        .clk(sys_clk), .rst_n(ctrl_rst_n), .save_enabled(config_ready),
        .key2_n(key2_n), .key3_n(key3_n), .key2_event(key2_press),
        .key3_event(key3_press), .key3_long_event(key3_long_press),
        .preview_toggle(chord_preview_toggle),
        .save_event(config_save_event),
        .save_locked(config_save_locked), .key2_stable_n(),
        .key3_stable_n()
    );

    // Session-level brightness ladder: KEY3 held alone for one second and
    // released steps 100/75/50/25%% and back to 100%%. Not persisted, so the
    // Flash configuration record layout stays unchanged.
    reg [1:0] brightness_level;
    always @(posedge sys_clk or negedge ctrl_rst_n) begin
        if (!ctrl_rst_n)
            brightness_level <= 2'd0;
        else if (key3_long_press && config_ready)
            brightness_level <= brightness_level + 1'b1;
    end
    reg [7:0] brightness_sys;
    always @* begin
        case (brightness_level)
            2'd0: brightness_sys = 8'd255;
            2'd1: brightness_sys = 8'd192;
            2'd2: brightness_sys = 8'd128;
            default: brightness_sys = 8'd64;
        endcase
    end

    // KEY4 short press steps volume; a one-second hold steps slideshow period.
    // A short KEY2+KEY3 chord toggles preview on release; the original two
    // second hold still saves the configuration without toggling preview.
    wire menu_cfg_valid;
    wire [31:0] menu_cfg_period;
    wire [7:0] menu_cfg_volume;
    wire menu_evt;
    wire [1:0] menu_kind;

    param_menu_controller #(
        .CLKS_PER_MS(50000)
    ) u_param_menu (
        .clk(sys_clk), .rst_n(ctrl_rst_n),
        .key4_n(key4_n), .service_ready(config_ready),
        .live_volume(config_volume), .live_period_ms(config_period_ms),
        .cfg_update_valid(menu_cfg_valid),
        .cfg_period_ms(menu_cfg_period), .cfg_volume(menu_cfg_volume),
        .menu_event(menu_evt), .menu_kind(menu_kind), .menu_level()
    );

    player_config_persistence u_player_config (
        .clk(sys_clk), .rst_n(ctrl_rst_n),
        .cfg_update_valid((key3_press || menu_cfg_valid) && config_ready),
        .cfg_period_ms(menu_cfg_valid ? menu_cfg_period : config_period_ms),
        .cfg_volume(menu_cfg_valid ? menu_cfg_volume : config_volume),
        // KEY3 toggles the slideshow in the same cycle, so the persisted
        // mode must be the inverted current value; a KEY4 menu update leaves
        // the mode untouched and must record it as-is, otherwise every
        // volume/period change silently flips auto-slideshow - and writes
        // the flipped state back to flash across reboots.
        .cfg_mode(requested_config_mode),
        .playback_index(playback_index_sys),
        .save_request(config_save_event || playback_save_pulse),
        .service_ready(config_ready),
        .busy(config_busy), .load_done(config_load_done),
        .save_done(config_save_done), .apply_valid(config_apply_valid),
        .config_rejected(config_rejected),
        .fatal_error(config_fatal_error), .error_code(config_error_code),
        .request_dropped(config_request_dropped),
        .stored_config_valid(stored_config_valid),
        .active_version(stored_config_version),
        .live_period_ms(config_period_ms),
        .live_period_cycles(config_period_cycles),
        .live_volume(config_volume), .live_mode(config_mode),
        .saved_period_ms(stored_period_ms), .saved_volume(stored_volume),
        .saved_mode(stored_mode), .saved_index(stored_index),
        .playback_mode_dirty(playback_mode_dirty),
        .saving_period_ms(saving_period_ms), .saving_volume(saving_volume),
        .saving_mode(saving_mode), .saving_index(saving_index),
        .jedec_id(config_jedec_id), .status1(config_status1),
        .status2(config_status2), .status3(config_status3),
        .flash_miso(flash_miso), .flash_cs_n(flash_cs_n),
        .flash_sclk(flash_sclk), .flash_mosi(flash_mosi),
        .flash_wp_n(flash_wp_n), .flash_hold_n(flash_hold_n)
    );

    reg config_save_ok_latched;
    always @(posedge sys_clk or negedge ctrl_rst_n) begin
        if (!ctrl_rst_n)
            config_save_ok_latched <= 1'b0;
        else if (config_save_done && !config_fatal_error)
            config_save_ok_latched <= 1'b1;
    end

    wire [3:0] image_complete_sys;
    wire ingest_finished_sys;
    sync_2ff u_ingest_finished_sys (
        .clk(sys_clk), .rst_n(ctrl_rst_n),
        .async_in(ingest_finished), .sync_out(ingest_finished_sys)
    );
    genvar bit_index;
    generate
        for (bit_index = 0; bit_index < 4; bit_index = bit_index + 1) begin : g_image_sync
            sync_2ff u_sync (
                .clk(sys_clk), .rst_n(ctrl_rst_n),
                .async_in(image_complete[bit_index]),
                .sync_out(image_complete_sys[bit_index])
            );
        end
    endgenerate

    wire video_clk;
    wire video_hs;
    wire video_vs;
    wire video_de;
    wire [23:0] video_rgb;
    wire [9:0] video_x;
    wire [1:0] active_index;
    wire [1:0] pending_index;
    wire [1:0] restore_saved_index =
        config_mode[3] ? config_mode[2:1] : 2'd0;
    // Wait for the saved image. If loading ends without it, use the first
    // completed image instead of reading an uninitialized SDRAM bank.
    wire [1:0] restore_index =
        image_complete_sys[restore_saved_index] ? restore_saved_index :
        image_complete_sys[0] ? 2'd0 :
        image_complete_sys[1] ? 2'd1 :
        image_complete_sys[2] ? 2'd2 : 2'd3;
    wire restore_ready = image_complete_sys[restore_saved_index] ||
                         (ingest_finished_sys && (|image_complete_sys));
    assign playback_restored_sys = restore_done_sys &&
                                   playback_index_sys == restore_index;
    assign requested_config_mode =
        {4'd0,config_mode[3:1],key3_press ? ~auto_enabled : auto_enabled};
    // Five-minute automatic checkpoints bound Flash erase frequency. Manual
    // switches and mode/parameter changes are saved as soon as it is idle.
    localparam [33:0] CHECKPOINT_CYCLES = 34'd15000000000;
    always @(posedge sys_clk or negedge ctrl_rst_n) begin
        if (!ctrl_rst_n) begin
            playback_save_pulse <= 1'b0;
            playback_save_pending <= 1'b0;
            playback_auto_dirty <= 1'b0;
            playback_checkpoint_cycles <= 34'd0;
            playback_last_index <= 2'd0;
            playback_resume_started <= 1'b0;
            playback_index_settle <= 32'd0;
            playback_saved_index <= 2'd0;
            playback_saved_mode <= 1'b0;
            playback_saved_period <= 32'd0;
            playback_saved_volume <= 8'd0;
            playback_saved_valid <= 1'b0;
            playback_save_inflight <= 1'b0;
            playback_pending_after_save <= 1'b0;
            playback_manual_during_save <= 1'b0;
            playback_auto_checkpoint_pending <= 1'b0;
            playback_saved_index_pending <= 1'b0;
            playback_loaded_snapshot <= 1'b0;
        end else begin
            playback_save_pulse <= 1'b0;
            if (config_load_done) begin
                // apply_config updates the live outputs on this same edge.
                // Capture the restored values on the following cycle.
                playback_saved_valid <= stored_config_valid;
                playback_loaded_snapshot <= 1'b1;
            end
            if (playback_loaded_snapshot) begin
                playback_saved_index <= config_mode[3] ? config_mode[2:1] : 2'd0;
                playback_saved_mode <= config_mode[0];
                playback_saved_period <= config_period_ms;
                playback_saved_volume <= config_volume;
                playback_loaded_snapshot <= 1'b0;
            end
            if (config_save_done && !config_fatal_error &&
                playback_save_inflight) begin
                playback_saved_index_pending <= 1'b1;
                playback_saved_valid <= 1'b1;
                playback_save_inflight <= 1'b0;
                if (playback_pending_after_save ||
                    playback_index_sys != saving_index ||
                    auto_enabled != saving_mode[0] ||
                    config_period_ms != saving_period_ms ||
                    config_volume != saving_volume) begin
                    if ((playback_pending_after_save &&
                         playback_manual_during_save) ||
                        !auto_enabled ||
                        auto_enabled != saving_mode[0] ||
                        config_period_ms != saving_period_ms ||
                        config_volume != saving_volume)
                        playback_save_pending <= 1'b1;
                    else if (playback_index_sys != saving_index)
                        playback_auto_dirty <= 1'b1;
                end
                playback_pending_after_save <= 1'b0;
                playback_manual_during_save <= 1'b0;
            end
            if (playback_saved_index_pending) begin
                playback_saved_index <= stored_index;
                playback_saved_mode <= stored_mode[0];
                playback_saved_period <= stored_period_ms;
                playback_saved_volume <= stored_volume;
                playback_saved_index_pending <= 1'b0;
            end
            if (config_fatal_error)
                playback_save_inflight <= 1'b0;
            if (config_fatal_error)
                playback_saved_index_pending <= 1'b0;
            if ((config_save_event || playback_save_pulse) && config_ready) begin
                playback_save_inflight <= 1'b1;
                playback_manual_during_save <= 1'b0;
            end
            if (!playback_resume_started && playback_restored_sys) begin
                playback_resume_started <= 1'b1;
                playback_last_index <= playback_index_sys;
            end
            if (playback_resume_started) begin
                if (playback_checkpoint_cycles < CHECKPOINT_CYCLES)
                    playback_checkpoint_cycles <= playback_checkpoint_cycles + 1'b1;
                if (playback_index_sys != playback_last_index) begin
                    playback_last_index <= playback_index_sys;
                    playback_index_settle <= 32'd0;
                    if (playback_commit_auto_sys)
                        playback_auto_dirty <= 1'b1;
                    else if (playback_save_inflight &&
                             !playback_commit_auto_sys) begin
                        playback_pending_after_save <= 1'b1;
                        playback_manual_during_save <= 1'b1;
                    end
                    else playback_save_pending <= 1'b1;
                end
                if (playback_index_settle < 32'd50000000 &&
                    playback_index_sys == playback_last_index)
                    playback_index_settle <= playback_index_settle + 1'b1;
                if (playback_auto_dirty &&
                    playback_checkpoint_cycles == CHECKPOINT_CYCLES &&
                    !playback_auto_checkpoint_pending) begin
                    playback_auto_checkpoint_pending <= 1'b1;
                    playback_save_pending <= 1'b1;
                    playback_auto_dirty <= 1'b0;
                    playback_checkpoint_cycles <= 34'd0;
                end
            end
            if (!playback_resume_started && playback_restored_sys &&
                playback_index_sys != restore_saved_index)
                playback_save_pending <= 1'b1;
            if (playback_mode_dirty && config_ready &&
                !playback_save_inflight && !playback_save_pulse)
                playback_save_pending <= 1'b1;
            if (playback_save_pending && config_ready &&
                !playback_save_inflight &&
                !playback_saved_index_pending &&
                (playback_auto_checkpoint_pending ||
                 playback_index_settle == 32'd50000000) &&
                !key3_press && !menu_cfg_valid && !config_save_event) begin
                if (playback_auto_checkpoint_pending ||
                    !playback_saved_valid ||
                    playback_index_sys != playback_saved_index ||
                    auto_enabled != playback_saved_mode ||
                    config_period_ms != playback_saved_period ||
                    config_volume != playback_saved_volume)
                    playback_save_pulse <= 1'b1;
                playback_auto_checkpoint_pending <= 1'b0;
                playback_save_pending <= 1'b0;
            end
            if (playback_resume_started && config_ready &&
                !playback_save_inflight && !playback_save_pending &&
                playback_index_settle == 32'd50000000 &&
                (config_period_ms != playback_saved_period ||
                 config_volume != playback_saved_volume ||
                 auto_enabled != playback_saved_mode))
                playback_save_pending <= 1'b1;
        end
    end
    wire frame_start;
    wire reader_busy;
    wire reader_underflow;
    wire [7:0] video_error;
    wire preview_mode_pix;

    // FADE_FRAMES=16 gives roughly a 0.27 s ramp at 640x480@60 on both the
    // fade-out and fade-in halves of a manual switch.
    wire [7:0] brightness_pix;
    genvar bright_bit;
    generate
        for (bright_bit = 0; bright_bit < 8; bright_bit = bright_bit + 1) begin : g_bright_sync
            sync_2ff u_sync (
                .clk(pixel_clk), .rst_n(pix_rst_n),
                .async_in(brightness_sys[bright_bit]),
                .sync_out(brightness_pix[bright_bit])
            );
        end
    endgenerate

    media_video_pipeline #(
        .FADE_FRAMES(16), .RESTORE_ON_BOOT(1)
    ) u_video (
        .brightness(brightness_pix),
        .ctrl_clk(sys_clk), .ctrl_rst_n(ctrl_rst_n),
        .next_event(key2_press), .toggle_auto_event(key3_press),
        .period_cfg_valid(config_apply_valid),
        .period_cfg_cycles(config_period_cycles),
        .auto_cfg_valid(config_apply_valid),
        .auto_cfg_enabled(config_mode[0]),
        .restore_valid(config_load_done), .restore_index(restore_index),
        .restore_ready(restore_ready),
        .image_complete(image_complete_sys),
        .preview_toggle(chord_preview_toggle), .preview_mode_out(preview_mode_pix),
        .player_enable(player_enable_mem), .mem_clk(mem_clk),
        .mem_rst_n(mem_rst_n), .controller_init_done(sdram_init_done),
        .controller_busy(reader_blocked), .refresh_active(sdram_refresh_active),
        .memory_request(read_request), .memory_active(read_active),
        .app_rd_en(player_app_rd_en), .app_rd_addr(player_app_rd_addr),
        .sdram_rd_valid(sdram_rd_valid), .sdram_rd_data(sdram_rd_data),
        .pix_clk(pixel_clk), .pix_rst_n(pix_rst_n),
        .video_clk(video_clk), .video_hs(video_hs),
        .video_vs(video_vs),
        .video_de(video_de), .video_rgb(video_rgb), .video_x(video_x),
        .manual_audio_commit(manual_audio_commit),.active_index(active_index),
        .pending_index(pending_index), .auto_enabled(auto_enabled),
        .restore_done(restore_done_sys),
        .image_index_commit(image_index_commit),
        .auto_image_commit_out(auto_image_commit_pix),
        .frame_start(frame_start), .reader_busy(reader_busy),
        .reader_underflow(reader_underflow), .video_error(video_error)
    );

    // Audio-video linkage: the displayed image picks the sustained note and
    // the persisted volume scales it. Both are slow, frame-granular signals
    // crossing into the registered video clock domain.
    wire [1:0] active_index_video;
    wire [7:0] volume_video;
    genvar media_bit;
    generate
        for (media_bit = 0; media_bit < 2; media_bit = media_bit + 1) begin : g_media_sync
            sync_2ff u_sync (
                .clk(video_clk), .rst_n(pix_rst_n),
                .async_in(active_index[media_bit]),
                .sync_out(active_index_video[media_bit])
            );
        end
    endgenerate
    generate
        for (media_bit = 0; media_bit < 8; media_bit = media_bit + 1) begin : g_volume_sync
            sync_2ff u_sync (
                .clk(video_clk), .rst_n(pix_rst_n),
                .async_in(config_volume[media_bit]),
                .sync_out(volume_video[media_bit])
            );
        end
    endgenerate

    wire [9:0] audio_symbol0;
    wire [9:0] audio_symbol1;
    wire [9:0] audio_symbol2;
    wire audio_sequence_error;
    wire audio_tone_overflow;
    wire audio_pcm_contract_error;
    wire audio_phy_phase_error;
    wire audio_blip_active;

    // D4 diagnoses the former aggregate 0x71: keep WAV/SD detail and
    // distinguish symbol sequencing, FIFO overflow, PCM contract and PHY.
    // Cross-domain status wires (declared before first use to keep
    // synthesis logs free of HDL-5373 used-before-declaration noise).
    wire player_enable_sys;
    wire storage_error_sys;
    wire video_fault_sys;
    wire audio_fault_sys;
    wire wav_diag_valid,hdmi_diag_valid;
    wire [7:0] wav_diag_code,hdmi_diag_code;
    wire [7:0] wav_detail = wav_error==8'h10 ? 8'h95 :
                           wav_error==8'h24 ? 8'h96 :
                           wav_error==8'h25 ? 8'h97 :
                           wav_error==8'h23 ? 8'h98 : wav_error;
    wire [7:0] hdmi_detail = audio_sequence_error ? 8'h72 :
                            audio_tone_overflow ? 8'h73 :
                            audio_pcm_contract_error ? 8'h74 : 8'h75;
    sticky_error_cdc u_wav_error_snapshot (
        .src_clk(mem_clk),.src_rst_n(mem_rst_n),
        .src_error(wav_error!=0),.src_code(wav_detail),.src_latched(),
        .dst_clk(sys_clk),.dst_rst_n(ctrl_rst_n),
        .dst_valid(wav_diag_valid),.dst_code(wav_diag_code)
    );
    sticky_error_cdc u_hdmi_error_snapshot (
        .src_clk(pixel_clk),.src_rst_n(pix_rst_n),
        .src_error(audio_sequence_error||audio_tone_overflow||audio_pcm_contract_error||audio_phy_phase_error),
        .src_code(hdmi_detail),.src_latched(),
        .dst_clk(sys_clk),.dst_rst_n(ctrl_rst_n),
        .dst_valid(hdmi_diag_valid),.dst_code(hdmi_diag_code)
    );
    wire [3:0] video_error_sys;

    // Storage failures surface the real ingest error code (SD init
    // 0x01-0x0c, filesystem 0x10-0x22, decoder 0x31-0x38, loader 0x41-0x46)
    // so the OSD and UART distinguish "no card" (CMD0 timeout) from
    // "no FAT/no BMP"; the blanket 0xFE remains for code-less failures.
    wire [7:0] storage_reported_error;
    // Publish an immutable code and its valid flag together in sys_clk.
    // The previous direct ingest_error_code -> UART path crossed mem_clk
    // without a coherent snapshot and failed final setup timing.
    sticky_error_cdc u_storage_error_snapshot (
        .src_clk(mem_clk), .src_rst_n(mem_rst_n),
        .src_error(ingest_done &&
                   (ingest_error_stage != 0 || ingest_error_code != 0 ||
                    image_complete != 4'hf)),
        .src_code(ingest_error_code), .src_latched(storage_error_mem),
        .dst_clk(sys_clk), .dst_rst_n(ctrl_rst_n),
        .dst_valid(storage_error_sys), .dst_code(storage_reported_error)
    );
    wire [7:0] reported_error = storage_error_sys ? storage_reported_error :
                                 config_fatal_error ? config_error_code :
                                 video_fault_sys ? {4'b0, video_error_sys} :
                                 wav_diag_valid ? wav_diag_code :
                                 hdmi_diag_valid ? hdmi_diag_code :
                                 8'd0;

    // Status OSD: bank number, auto mode and fault code rendered in the
    // top-left corner. Purely combinational, so the video_x/video_de
    // alignment feeding the HDMI Data Island scheduler is preserved.
    wire auto_enabled_video;
    wire [7:0] reported_error_video;
    sync_2ff u_auto_osd_sync (
        .clk(video_clk), .rst_n(pix_rst_n),
        .async_in(auto_enabled), .sync_out(auto_enabled_video)
    );
    genvar osd_bit;
    generate
        for (osd_bit = 0; osd_bit < 8; osd_bit = osd_bit + 1) begin : g_osd_err
            sync_2ff u_sync (
                .clk(video_clk), .rst_n(pix_rst_n),
                .async_in(reported_error[osd_bit]),
                .sync_out(reported_error_video[osd_bit])
            );
        end
    endgenerate

    wire [23:0] video_rgb_osd;
    wire video_hs_osd;
    wire video_vs_osd;
    wire video_de_osd;
    wire [9:0] video_x_osd;
    osd_overlay_24 #(.OSD_X(8), .OSD_Y(8)) u_osd (
        .clk(video_clk), .rst_n(pix_rst_n),
        .hsync(video_hs), .vsync(video_vs), .de(video_de),
        .rgb_in(video_rgb), .pixel_x(video_x),
        .active_index(active_index_video), .auto_enabled(auto_enabled_video),
        .preview_mode(preview_mode_pix),
        .error_code(reported_error_video),
        .rgb_out(video_rgb_osd),
        .hsync_out(video_hs_osd), .vsync_out(video_vs_osd),
        .de_out(video_de_osd), .pixel_x_out(video_x_osd),
        .osd_active()
    );

    // E5 extension: audio waveform overlay, tapped from the samples the
    // bridge is actually transmitting. Combinational overlay - the timing
    // bundle stays aligned.
    wire [23:0] video_rgb_wave;
    wire audio_pcm_tap_valid;
    wire audio_pcm_tap_is_wav;
    wire signed [15:0] audio_pcm_tap_left;
    promax_audio_waveform #(
        .WAVE_X(64), .WAVE_Y(336), .WAVE_WIDTH(512), .WAVE_HEIGHT(64)
    ) u_audio_waveform (
        .clk(video_clk), .rst_n(pix_rst_n),
        .vsync(video_vs_osd), .de(video_de_osd),
        .rgb_in(video_rgb_osd), .pixel_x(video_x_osd),
        .enable(!preview_mode_pix),
        .pcm_tap_valid(audio_pcm_tap_valid), .pcm_tap_left(audio_pcm_tap_left),
        .pcm_tap_is_wav(audio_pcm_tap_is_wav),
        .rgb_out(video_rgb_wave), .waveform_active()
    );

    // E1 extension: per-picture Chinese scrolling subtitle banner. Registered
    // stage - rgb and the whole timing bundle shift by the same clock.
    wire [23:0] video_rgb_sub;
    wire video_hs_sub;
    wire video_vs_sub;
    wire video_de_sub;
    wire [9:0] video_x_sub;
    wire [3:0] image_complete_video;
    genvar subtitle_bit;
    generate for (subtitle_bit=0; subtitle_bit<4; subtitle_bit=subtitle_bit+1) begin: g_subtitle_ready
        sync_2ff u_complete(.clk(video_clk), .rst_n(pix_rst_n),
            .async_in(image_complete[subtitle_bit]), .sync_out(image_complete_video[subtitle_bit]));
    end endgenerate
    subtitle_scroll_24 #(
        .SUB_Y(438), .SCROLL_STEP(2), .H_ACT(640)
    ) u_subtitle (
        .clk(video_clk), .rst_n(pix_rst_n),
        .hsync(video_hs_osd), .vsync(video_vs_osd), .de(video_de_osd),
        .pixel_x(video_x_osd), .rgb_in(video_rgb_wave),
        .active_index(active_index_video),
        .enable(player_enable_pix_audio && image_complete_video[active_index_video] && !preview_mode_pix),
        .hsync_out(video_hs_sub), .vsync_out(video_vs_sub),
        .de_out(video_de_sub), .pixel_x_out(video_x_sub),
        .rgb_out(video_rgb_sub), .subtitle_active()
    );

    // Plan A: the finished picture no longer leaves through this board's
    // HDMI_A; media_stream_tx sends it to the second board. The second
    // board plays the arpeggio and AUDIO1/AUDIO2 itself from the playback
    // state in the status frames. This executor still runs, reading
    // AUDIO1.WAV from this board's card, because its samples drive the
    // on-screen waveform.
    wire audio_sample_valid;
    wire signed [15:0] audio_sample_left, audio_sample_right;
    wire audio_sample_is_wav;
    audio_playback_executor u_audio_executor (
        .clk(video_clk),.rst_n(pix_rst_n),
        .playback_ready(player_enable_pix_audio),.auto_enabled(auto_enabled_video),
        .manual_image_commit(manual_audio_commit),
        .media_id(active_index_video),.volume(volume_video),
        .wav_stopped(wav_stopped_pix),.wav_prefilled(wav_prefilled),
        .wav_valid(wav_valid),.wav_data(wav_data),.wav_fault(wav_fault_pix),
        .wav_run(wav_run),.wav_pop(wav_pop),
        .sample_valid(audio_sample_valid),.sample_ready(1'b1),
        .sample_left(audio_sample_left),.sample_right(audio_sample_right),
        .overflow(audio_tone_overflow),.blip_active(audio_blip_active),
        .sample_is_wav(audio_sample_is_wav));
    assign audio_pcm_tap_valid = audio_sample_valid;
    assign audio_pcm_tap_left = audio_sample_left;
    assign audio_pcm_tap_is_wav = audio_sample_is_wav;
    assign audio_sequence_error = 1'b0;
    assign audio_pcm_contract_error = 1'b0;
    assign audio_phy_phase_error = 1'b0;
    // HDMI_A of this board is unused in plan A.
    assign HDMI_CLK_P = 1'b0;
    assign HDMI_D0_P = 1'b0;
    assign HDMI_D1_P = 1'b0;
    assign HDMI_D2_P = 1'b0;

    sync_2ff u_player_enable_sys (
        .clk(sys_clk), .rst_n(ctrl_rst_n),
        .async_in(player_enable_mem), .sync_out(player_enable_sys)
    );
    sync_2ff u_video_fault_sys (
        .clk(sys_clk), .rst_n(ctrl_rst_n),
        .async_in(video_error != 0), .sync_out(video_fault_sys)
    );
    assign audio_fault_sys = wav_diag_valid || hdmi_diag_valid;

    genvar error_bit_index;
    generate
        for (error_bit_index = 0; error_bit_index < 4;
             error_bit_index = error_bit_index + 1) begin : g_video_error_sync
            sync_2ff u_sync (
                .clk(sys_clk), .rst_n(ctrl_rst_n),
                .async_in(video_error[error_bit_index]),
                .sync_out(video_error_sys[error_bit_index])
            );
        end
    endgenerate

    // UART status reporter: boot banner, image switch lines, parameter
    // menu changes and fault codes on the board USB-UART at 115200 8N1.
    genvar uart_bit;
    generate
        for (uart_bit = 0; uart_bit < 2; uart_bit = uart_bit + 1) begin : g_media_sys
            sync_2ff u_sync (
                .clk(sys_clk), .rst_n(ctrl_rst_n),
                .async_in(active_index[uart_bit]),
                .sync_out(active_index_sys[uart_bit])
            );
        end
    endgenerate

    reg [1:0] active_index_sys_q;
    always @(posedge sys_clk or negedge ctrl_rst_n) begin
        if (!ctrl_rst_n)
            active_index_sys_q <= 2'd0;
        else
            active_index_sys_q <= active_index_sys;
    end
    wire switch_evt = (active_index_sys != active_index_sys_q);


    uart_status_reporter #(.CLKS_PER_BIT(434)) u_uart_report (
        .clk(sys_clk), .rst_n(ctrl_rst_n),
        .enable(player_enable_sys), .media_id(active_index_sys),
        .volume(config_volume), .period_ms(config_period_ms),
        .auto_enabled(auto_enabled),
        .switch_event(switch_evt), .menu_event(menu_evt),
        .menu_kind(menu_kind), .error_code(reported_error),
        .bright_event(key3_long_press && config_ready),
        .bright_value(brightness_sys),
        .uart_tx(uart_tx_o)
    );

    // Competition-facing playback indicators with fault-code priority:
    // storage all-on, config code, audio 4'hD, video code, save-ok 4'b0101,
    // then the running bank/auto/auto state.
    wire net_send, net_ready, net_received, net_activity;
    wire [7:0] net_tx_cmd,net_tx_seq,net_rx_cmd,net_rx_seq;
    wire [15:0] net_packets;
    wire net_fullscreen,net_live,net_ack;
    wire [7:0] net_last_seq;
    wire net_led_rx,net_led_good,net_led_bad,net_led_tx,net_led_stream;
    // RGMII TX clocks: 125 MHz 0 deg for data, 90 deg for TXC (vendor example).
    wire eth_clk, eth_clk90, eth_locked;
    rgmii_tx_pll u_eth_pll (.refclk(sys_clk),.reset(~sys_rst_n),
        .extlock(eth_locked),.clk0_out(eth_clk),.clk90_out(eth_clk90));
    wire net_rst_n = sys_rst_n && eth_locked;
    // Picture and audio for the second board (plan A).
    wire blk_ready, blk_audio, blk_take, blk_done;
    wire [8:0] blk_line, blk_raddr;
    wire [23:0] blk_status;
    wire [31:0] blk_rdata;
    wire [15:0] stream_lines_dropped;
    reg [1:0] eth_tx_rst_q;
    always @(posedge eth_clk or negedge net_rst_n)
        if (!net_rst_n) eth_tx_rst_q <= 0; else eth_tx_rst_q <= {eth_tx_rst_q[0], 1'b1};
    // Audio state for the second board, which plays the arpeggio and the WAV
    // files itself. All fields are already in the video_clk domain; manual
    // image commits are counted so a missed status frame cannot lose one.
    reg [5:0] manual_commit_count;
    always @(posedge video_clk or negedge pix_rst_n)
        if (!pix_rst_n) manual_commit_count <= 6'd0;
        else if (manual_audio_commit) manual_commit_count <= manual_commit_count + 1'b1;
    // Reserved status bits carry settings validity, period and brightness.
    wire [1:0] status_period_sys = config_period_ms == 32'd2000 ? 2'd0 :
        config_period_ms == 32'd5000 ? 2'd1 : config_period_ms == 32'd10000 ? 2'd2 : 2'd3;
    wire [1:0] status_period_pix, status_bright_pix;
    genvar status_bit;
    generate for (status_bit=0;status_bit<2;status_bit=status_bit+1) begin: g_status_settings
        sync_2ff u_period (.clk(video_clk),.rst_n(pix_rst_n),
            .async_in(status_period_sys[status_bit]),.sync_out(status_period_pix[status_bit]));
        sync_2ff u_bright (.clk(video_clk),.rst_n(pix_rst_n),
            .async_in(brightness_level[status_bit]),.sync_out(status_bright_pix[status_bit]));
    end endgenerate
    wire [23:0] stream_status = {volume_video, manual_commit_count,
                                 active_index_video, 2'b10, status_period_pix, status_bright_pix,
                                 auto_enabled_video, player_enable_pix_audio};
    media_stream_tx u_stream_tx (
        .pix_clk(video_clk), .pix_rst_n(pix_rst_n),
        .vid_vs(video_vs_sub), .vid_de(video_de_sub), .vid_rgb(video_rgb_sub),
        .status_in(stream_status),
        .tx_clk(eth_clk), .tx_rst_n(eth_tx_rst_q[1]),
        .blk_ready(blk_ready), .blk_audio(blk_audio), .blk_line(blk_line),
        .blk_status(blk_status),
        .blk_take(blk_take), .blk_raddr(blk_raddr), .blk_rdata(blk_rdata),
        .blk_done(blk_done), .lines_dropped(stream_lines_dropped));
    rgmii_command_link #(.LOCAL_ID(8'h01),.PEER_ID(8'h02),.STREAM_TX(1)) u_eth (
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
        .blk_ready(blk_ready),.blk_audio(blk_audio),.blk_line(blk_line),
        .blk_status(blk_status),
        .blk_take(blk_take),.blk_done(blk_done),.blk_raddr(blk_raddr),.blk_rdata(blk_rdata),
        .s_rst_n(),.s_valid(),.s_audio(),.s_byte(),.s_index(),.s_tag(),.s_status(),
        .s_commit(),.s_abort());
    // Show network LEDs for one second after any RX burst, so a burst that
    // fails CRC is visible too; otherwise keep the original indicators.
    reg [25:0] net_led_window;
    always @(posedge sys_clk or negedge net_rst_n)
        if (!net_rst_n) net_led_window <= 0;
        else if (net_led_rx) net_led_window <= 26'd50000000;
        else if (net_led_window != 0) net_led_window <= net_led_window - 1'b1;
    net_command_policy #(.SLAVE(0)) u_net_policy (
        .clk(sys_clk),.rst_n(net_rst_n),
        .key1_n(1'b1),.key2_n(1'b1),
        .rx_pulse(net_received),.rx_cmd(net_rx_cmd),.rx_seq(net_rx_seq),
        .tx_ready(net_ready),.tx_send(net_send),
        .tx_cmd(net_tx_cmd),.tx_seq(net_tx_seq),
        .fullscreen(net_fullscreen),.link_alive(net_live),
        .last_sequence(net_last_seq),.acknowledged(net_ack));
    // Network LEDs for one second after a command arrives (silkscreen
    // LEDn+1 = led[n]): led[0] picture/audio frames being sent, led[1]
    // command received, led[2] fullscreen requested, led[3] bad RX frame.
    assign led = net_led_window != 0 ? {net_led_bad, net_fullscreen,
                                        net_led_good, net_led_tx} :
                 storage_error_sys ? 4'hf :
                 config_fatal_error ? {1'b1,config_error_code[2:0]} :
                 audio_fault_sys ? 4'hd :
                 video_fault_sys ? video_error_sys :
                 config_save_ok_latched ? 4'b0101 :
                 player_enable_sys ? {1'b0, auto_enabled, active_index} :
                 {1'b0, 1'b0, &image_complete_sys,
                  mem_locked & video_locked};

endmodule

// Grant complete continuous batches. Display reads have priority when both
// request; write opportunities appear as the display FIFO fills. Keep the
// owner until all registered commands/read returns drain, then turn around.
module sdram_burst_arbiter (
    input wire clk, rst_n, controller_busy, refresh_active,
    input wire write_request, read_request, write_active, read_active,
    output wire writer_blocked, reader_blocked
);
    reg [1:0] owner;
    reg seen_active;
    reg [2:0] guard;
    wire owner_active = owner == 1 ? read_active : write_active;
    assign reader_blocked = controller_busy || refresh_active || owner != 1 || seen_active;
    assign writer_blocked = controller_busy || refresh_active || owner != 2 || seen_active;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin owner<=0;seen_active<=0;guard<=0;end
        else if (owner==0) begin
            seen_active<=0;
            if(guard!=0)guard<=guard-1'b1;
            else if(!controller_busy && !refresh_active) begin
                if(read_request)owner<=1;
                else if(write_request)owner<=2;
            end
        end else begin
            if(owner_active)seen_active<=1;
            if(seen_active && !owner_active)begin owner<=0;guard<=3;end
            // A frame reload/error may withdraw a request before admission.
            else if(!seen_active && !owner_active &&
                    !(owner==1 ? read_request : write_request))begin owner<=0;guard<=3;end
        end
    end
endmodule
