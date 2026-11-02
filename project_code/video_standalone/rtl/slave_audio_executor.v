`timescale 1ns/1ps

// Slave-board audio source: the main board's audio_playback_executor with a
// policy that adds the fullscreen rule. The tone, ramp, volume and FIFO
// code is unchanged from rtl/audio/audio_playback_executor.v.
//
// Policy (slave_audio_policy):
//   fullscreen:     AUDIO2.WAV from the start, looping; master image
//                   changes are ignored.
//   leaving it:     same as power-up with the master's current state:
//                   auto mode -> AUDIO1.WAV from the start, otherwise the
//                   current image's arpeggio, then AUDIO1.WAV.
//   otherwise:      the main board's rules (audio_playback_policy).
// wav_select tells the reader which file to open (0 AUDIO1, 1 AUDIO2); it
// changes only together with a WAV restart or stop.
module slave_audio_policy (
    input wire clk, rst_n,
    input wire playback_ready, auto_enabled, fullscreen,
    input wire manual_image_commit, arpeggio_done, wav_playback_done,
    output wire [1:0] requested_source,       // 0 idle, 1 media, 2 WAV
    output wire wav_select,
    output reg arpeggio_restart, wav_restart, wav_stop
);
    localparam [2:0] IDLE=0, MEDIA=1, WAV=2, FULL=3;
    reg [2:0] state;
    reg auto_q;
    assign requested_source = state == FULL ? 2'd2 : state[1:0];
    assign wav_select = state == FULL;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; auto_q <= 0;
            arpeggio_restart <= 0; wav_restart <= 0; wav_stop <= 0;
        end else begin
            auto_q <= auto_enabled;
            arpeggio_restart <= 0; wav_restart <= 0; wav_stop <= 0;
            if (fullscreen) begin
                if (state != FULL || wav_playback_done) wav_restart <= 1;
                state <= FULL;
            end else if (state == FULL) begin
                // Back to the normal rules from a clean start.
                wav_stop <= 1; state <= IDLE;
            end else if (!playback_ready) begin
                if (state == WAV) wav_stop <= 1;
                state <= IDLE;
            end else if (auto_enabled) begin
                if (state != WAV || !auto_q || wav_playback_done)
                    wav_restart <= 1;
                state <= WAV;
            end else if (manual_image_commit) begin
                if (state == WAV) wav_stop <= 1;
                state <= MEDIA;
                arpeggio_restart <= 1;
            end else if (state == IDLE) begin
                state <= MEDIA;
                arpeggio_restart <= 1;
            end else if (state == MEDIA && arpeggio_done) begin
                state <= WAV;
                wav_restart <= 1;
            end else if (state == WAV && wav_playback_done) begin
                wav_restart <= 1;
            end
        end
    end
endmodule

module slave_audio_executor #(parameter CLOCK_HZ=25000000, SAMPLE_HZ=48000,
 BLIP_SAMPLES=4800,NOTE_SAMPLES=12000,parameter [19:0] BLIP_INC=32768)(
 input wire clk,rst_n,playback_ready,auto_enabled,manual_image_commit,fullscreen,
 input wire [1:0] media_id,input wire [7:0] volume,
 input wire wav_stopped,wav_prefilled,wav_valid,input wire [32:0] wav_data,
 input wire wav_fault,output reg wav_run,output wire wav_pop,output wire wav_select,
 output wire sample_valid,input wire sample_ready,output wire signed [15:0] sample_left,sample_right,
 output reg overflow,output wire blip_active,output wire sample_is_wav);
 wire [1:0] source;wire arp_restart,wav_restart,wav_stop;
 reg arp_done,wav_done;
 // Fullscreen plays AUDIO2 whether or not the main board is present.
 wire ready_eff=playback_ready||fullscreen;
 slave_audio_policy policy(.clk(clk),.rst_n(rst_n),.playback_ready(playback_ready),
 .auto_enabled(auto_enabled),.fullscreen(fullscreen),.manual_image_commit(manual_image_commit),
 .arpeggio_done(arp_done),.wav_playback_done(wav_done),.requested_source(source),
 .wav_select(wav_select),
 .arpeggio_restart(arp_restart),.wav_restart(wav_restart),.wav_stop(wav_stop));
 reg [2:0] wstate;reg [3:0] wait_count;reg primed;
 reg [3:0] epoch;reg [24:0] rate_accumulator;reg [19:0] phase;
 reg [12:0] blip_countdown;reg [2:0] note_index;reg [14:0] note_sample_count;
 reg arp_produced;reg [6:0] ramp;
 reg [37:0] fifo[0:15];reg [3:0] wp,rp;reg [4:0] count;
 reg [15:0] wav_tags;
 assign sample_is_wav=wav_tags[rp];
 wire pop=sample_valid&&sample_ready;
 wire tick=rate_accumulator>=CLOCK_HZ-SAMPLE_HZ;
 wire push=tick&&(count<16||pop);
 wire wav_take=source==2&&wstate==3&&primed&&wav_valid;
 assign wav_pop=push&&wav_take&&!wav_restart&&!wav_stop;
 assign sample_valid=count!=0;
 assign sample_left=fifo[rp][15:0];assign sample_right=fifo[rp][31:16];
 assign blip_active=source==1&&blip_countdown!=0;
 wire arp_last=blip_countdown==0&&note_index==5&&note_sample_count==NOTE_SAMPLES-1&&!arp_produced;
    function [19:0] melody_inc;
        input [1:0] melody;
        input [2:0] note;
        begin
            case ({melody, note})
                5'b00_000: melody_inc = 20'd11426;  // C5 523 Hz
                5'b00_001: melody_inc = 20'd14396;  // E5 659 Hz
                5'b00_010: melody_inc = 20'd17127;  // G5 784 Hz
                5'b00_011: melody_inc = 20'd22860;  // C6 1046 Hz
                5'b00_100: melody_inc = 20'd17127;  // G5
                5'b00_101: melody_inc = 20'd14396;  // E5
                5'b01_000: melody_inc = 20'd9634;   // A4 440 Hz
                5'b01_001: melody_inc = 20'd11426;  // C5
                5'b01_010: melody_inc = 20'd14396;  // E5
                5'b01_011: melody_inc = 20'd19269;  // A5 880 Hz
                5'b01_100: melody_inc = 20'd14396;  // E5
                5'b01_101: melody_inc = 20'd11426;  // C5
                5'b10_000: melody_inc = 20'd11426;  // C5
                5'b10_001: melody_inc = 20'd12833;  // D5 587 Hz
                5'b10_010: melody_inc = 20'd14396;  // E5
                5'b10_011: melody_inc = 20'd17127;  // G5
                5'b10_100: melody_inc = 20'd19269;  // A5
                5'b10_101: melody_inc = 20'd22860;  // C6
                5'b11_000: melody_inc = 20'd11426;  // C5
                5'b11_001: melody_inc = 20'd8566;   // G4 392 Hz
                5'b11_010: melody_inc = 20'd14396;  // E5
                5'b11_011: melody_inc = 20'd11426;  // C5
                5'b11_100: melody_inc = 20'd8566;   // G4
                5'b11_101: melody_inc = 20'd14396;  // E5
                default:   melody_inc = 20'd11426;
            endcase
        end
    endfunction

    wire [19:0] melody_note_inc = melody_inc(media_id, note_index);
    wire [19:0] phase_inc = (blip_countdown != 0) ? BLIP_INC : melody_note_inc;

    function signed [15:0] sine_sample;
        input [3:0] index;
        begin
            case (index)
                4'd0: sine_sample = 16'sd0;
                4'd1: sine_sample = 16'sd4592;
                4'd2: sine_sample = 16'sd8485;
                4'd3: sine_sample = 16'sd11087;
                4'd4: sine_sample = 16'sd12000;
                4'd5: sine_sample = 16'sd11087;
                4'd6: sine_sample = 16'sd8485;
                4'd7: sine_sample = 16'sd4592;
                4'd8: sine_sample = 16'sd0;
                4'd9: sine_sample = -16'sd4592;
                4'd10: sine_sample = -16'sd8485;
                4'd11: sine_sample = -16'sd11087;
                4'd12: sine_sample = -16'sd12000;
                4'd13: sine_sample = -16'sd11087;
                4'd14: sine_sample = -16'sd8485;
                default: sine_sample = -16'sd4592;
            endcase
        end
    endfunction

 wire signed [15:0] raw_left=source==1&&!arp_produced?sine_sample(phase[19:16]):wav_take?$signed(wav_data[15:0]):16'sd0;
 wire signed [15:0] raw_right=source==1&&!arp_produced?sine_sample(phase[19:16]):wav_take?$signed(wav_data[31:16]):16'sd0;
 wire signed [24:0] gain_l=raw_left*$signed({1'b0,volume});
 wire signed [24:0] gain_r=raw_right*$signed({1'b0,volume});
 wire signed [23:0] fade_l=$signed(gain_l[23:8])*$signed({1'b0,ramp});
 wire signed [23:0] fade_r=$signed(gain_r[23:8])*$signed({1'b0,ramp});
 wire [15:0] out_l=fade_l>>>6,out_r=fade_r>>>6;
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin wav_tags<=0;wstate<=0;wait_count<=0;wav_run<=0;primed<=0;epoch<=0;rate_accumulator<=0;phase<=0;
 blip_countdown<=0;note_index<=0;note_sample_count<=0;arp_produced<=1;ramp<=0;wp<=0;rp<=0;count<=0;overflow<=0;arp_done<=0;wav_done<=0;end
 else begin
 arp_done<=0;wav_done<=0;
 if(tick)rate_accumulator<=rate_accumulator+SAMPLE_HZ-CLOCK_HZ;else rate_accumulator<=rate_accumulator+SAMPLE_HZ;
 if(pop)begin
   rp<=rp+1'b1;
   if(fifo[rp][37:34]==epoch)begin
    if(fifo[rp][33:32]==1)arp_done<=1;
    if(fifo[rp][33:32]==2)wav_done<=1;
   end
 end
 if(push)begin
   wav_tags[wp]<=wav_take;
   fifo[wp]<={epoch,(source==1&&arp_last?2'd1:wav_take&&wav_data[32]?2'd2:2'd0),out_r,out_l};wp<=wp+1'b1;
   if(ramp<64 && (source==1||wav_take))ramp<=ramp+1'b1;
   if(source==1&&!arp_produced)begin
    phase<=phase+phase_inc;
    if(blip_countdown!=0)blip_countdown<=blip_countdown-1'b1;
    else if(note_sample_count==NOTE_SAMPLES-1)begin note_sample_count<=0;if(note_index==5)arp_produced<=1;else note_index<=note_index+1'b1;end
    else note_sample_count<=note_sample_count+1'b1;
   end
 end else if(tick)overflow<=1;
 case({push,pop})2'b10:count<=count+1'b1;2'b01:count<=count-1'b1;default:begin end endcase
 case(wstate)
 0:begin wav_run<=0;primed<=0;end
 1:begin wav_run<=0;if(wait_count<15)wait_count<=wait_count+1'b1;
   if(wav_stopped&&wait_count>=8)begin wav_run<=1;wstate<=2;end end
 2:if(!wav_stopped)wstate<=3;
 3:begin if(wav_prefilled)primed<=1;if(wav_fault)begin wav_run<=0;wstate<=0;end end
 default:wstate<=0;endcase
 if(wav_restart)begin wav_run<=0;wstate<=1;wait_count<=0;primed<=0;ramp<=0;epoch<=epoch+1'b1;end
 if(wav_stop||!ready_eff)begin wav_run<=0;wstate<=0;primed<=0;end
 if(arp_restart)begin epoch<=epoch+1'b1;phase<=0;blip_countdown<=BLIP_SAMPLES;note_index<=0;note_sample_count<=0;arp_produced<=0;ramp<=0;end
 end end
endmodule
