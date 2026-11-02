`timescale 1ns/1ps

// Policy requests only. The executor must preserve PCM handshakes, perform
// fades and acknowledge FIFO flushing before restarting WAV from byte zero.
// Integrated by audio_playback_executor.
module audio_playback_policy (
    input wire clk,
    input wire rst_n,
    input wire playback_ready,
    input wire auto_enabled,
    // Pulse at successful MANUAL image commit, not at the raw key edge.
    input wire manual_image_commit,
    // Pulse when the complete arpeggio has been audibly consumed.
    input wire arpeggio_done,
    // Pulse at audible WAV EOF, after buffered data has drained.
    input wire wav_playback_done,
    output wire [1:0] requested_source, // 0=idle, 1=media, 2=WAV
    output reg arpeggio_restart,
    output reg wav_restart,
    output reg wav_stop
);
    localparam [1:0] IDLE=0, MEDIA=1, WAV=2;
    reg [1:0] state;
    reg auto_q;
    assign requested_source=state;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE;
            auto_q<=0;
            arpeggio_restart<=0;
            wav_restart<=0;
            wav_stop<=0;
        end else begin
            auto_q<=auto_enabled;
            arpeggio_restart<=0;
            wav_restart<=0;
            wav_stop<=0;
            if (!playback_ready) begin
                if (state==WAV) wav_stop<=1;
                state<=IDLE;
            end else if (auto_enabled) begin
                // Auto mode wins over KEY2 on the same edge. Picture changes
                // during auto mode never interrupt or restart the WAV.
                if (state!=WAV || !auto_q || wav_playback_done)
                    wav_restart<=1;
                state<=WAV;
            end else if (manual_image_commit) begin
                if (state==WAV) wav_stop<=1;
                state<=MEDIA;
                arpeggio_restart<=1;
            end else if (state==IDLE) begin
                // Startup default: one media phrase followed by looping WAV.
                state<=MEDIA;
                arpeggio_restart<=1;
            end else if (state==MEDIA && arpeggio_done) begin
                state<=WAV;
                wav_restart<=1;
            end else if (state==WAV && wav_playback_done) begin
                wav_restart<=1;
            end
            // Disabling auto alone keeps the current WAV position.
        end
    end
endmodule
