`timescale 1ns/1ps

// Player-owned boundary between the registered video bundle and the verified
// Route B HDMI packet/symbol chain. The EG4 DDR PHY remains in the board Top so
// this bridge can be exercised in ordinary RTL simulation without vendor pads.
// media_id drives the per-image tone selection and volume scales the output,
// turning the tone source into the audio half of the AV-linked feature.
module media_hdmi_audio_bridge #(parameter ENABLE_WAV=0)(
    input wire playback_ready,auto_enabled,manual_image_commit,
    input wire wav_stopped,wav_prefilled,wav_valid,wav_fault,
    input wire [32:0] wav_data,output wire wav_run,wav_pop,
    input  wire        pixel_clk,
    input  wire        pixel_rst_n,
    input  wire        video_hsync,
    input  wire        video_vsync,
    input  wire        video_de,
    input  wire [9:0]  video_x,
    input  wire [23:0] video_rgb,
    input  wire [1:0]  media_id,
    input  wire [7:0]  volume,
    output wire [9:0]  channel0_symbol,
    output wire [9:0]  channel1_symbol,
    output wire [9:0]  channel2_symbol,
    output wire        sequence_error,
    output wire        tone_overflow,
    output wire        pcm_contract_error,
    output wire        blip_active,
    // PCM tap of the samples actually handed to the packet chain; used by the
    // waveform overlay so the display shows the audio being transmitted.
    output wire        pcm_tap_valid,
    output wire signed [15:0] pcm_tap_left,
    output wire        pcm_tap_is_wav
);
    wire source_valid;
    wire source_ready;
    wire signed [15:0] source_left;
    wire signed [15:0] source_right;
    wire source_is_wav;
    wire sample_valid;
    wire sample_ready;
    wire signed [15:0] sample_left;
    wire signed [15:0] sample_right;
    assign pcm_tap_valid = sample_valid && sample_ready;
    assign pcm_tap_left = sample_left;
    // The contract adapter passes samples through without buffering.
    assign pcm_tap_is_wav = source_is_wav;

    generate if(ENABLE_WAV)begin:g_wav
    audio_playback_executor u_executor(.clk(pixel_clk),.rst_n(pixel_rst_n),
      .playback_ready(playback_ready),.auto_enabled(auto_enabled),.manual_image_commit(manual_image_commit),
      .media_id(media_id),.volume(volume),.wav_stopped(wav_stopped),.wav_prefilled(wav_prefilled),
      .wav_valid(wav_valid),.wav_data(wav_data),.wav_fault(wav_fault),.wav_run(wav_run),.wav_pop(wav_pop),
      .sample_valid(source_valid),.sample_ready(source_ready),.sample_left(source_left),.sample_right(source_right),
      .overflow(tone_overflow),.blip_active(blip_active),.sample_is_wav(source_is_wav));
    end else begin:g_legacy
    assign source_is_wav=1'b0;
    assign wav_run=0;assign wav_pop=0;
    pcm_media_tone u_tone (
        .clk(pixel_clk), .rst_n(pixel_rst_n),
        .media_id(media_id), .volume(volume),
        .sample_valid(source_valid), .sample_ready(source_ready),
        .sample_left(source_left), .sample_right(source_right),
        .overflow(tone_overflow), .blip_active(blip_active)
    );

    end endgenerate

    m06_pcm_contract_adapter u_m06_pcm_contract (
        .clk(pixel_clk), .rst_n(pixel_rst_n),
        .source_valid(source_valid), .source_ready(source_ready),
        .source_left(source_left), .source_right(source_right),
        .pcm_valid(sample_valid), .pcm_ready(sample_ready),
        .pcm_left(sample_left), .pcm_right(sample_right),
        .contract_error(pcm_contract_error)
    );

    hdmi_audio_symbol_core u_symbol_core (
        .pixel_clk(pixel_clk), .rst_n(pixel_rst_n),
        .video_hsync(video_hsync), .video_vsync(video_vsync),
        .video_de(video_de), .video_x(video_x), .video_rgb(video_rgb),
        .sample_valid(sample_valid), .sample_ready(sample_ready),
        .sample_left(sample_left), .sample_right(sample_right),
        .channel0_symbol(channel0_symbol),
        .channel1_symbol(channel1_symbol),
        .channel2_symbol(channel2_symbol),
        .sequence_error(sequence_error)
    );
endmodule
