`timescale 1ns/1ps

// Stop acknowledges only after the shared physical sector and replay complete.
module wav_sd_stream(input wire clk,rst_n,run_request,storage_ready,audio_valid,
 input wire [31:0] audio_cluster,audio_size,fat_start_lba,data_start_lba,max_cluster,
 input wire [7:0] sectors_per_cluster,input wire room_sector,
 output wire pcm_last,output wire pcm_valid,output wire [31:0] pcm_data,output reg stopped,
 output reg eof,output reg [7:0] error,
 output wire reader_req,output wire [31:0] reader_lba,
 input wire reader_busy,reader_valid,reader_done,input wire [7:0] reader_byte,reader_error,
 input wire [8:0] reader_byte_index);
 reg run_s1,run_s2,local_reset,start;reg [2:0] state;
 wire req,busy,valid,done;wire [31:0] lba;wire [7:0] data,sector_error;wire [8:0] idx;
 wire file_valid,file_done,file_busy;wire [7:0] fb,fe,pe;wire [31:0] off;wire pdone;
 wire rr=rst_n&&!local_reset;
 sd_sector_replay_buffer replay(.clk(clk),.rst_n(rst_n),
 .sector_req(req),.sector_lba(lba),.sector_busy(busy),.sector_valid(valid),.sector_byte(data),
 .sector_byte_index(idx),.sector_done(done),.sector_error(sector_error),
 .reader_req(reader_req),.reader_lba(reader_lba),.reader_busy(reader_busy),.reader_valid(reader_valid),
 .reader_byte(reader_byte),.reader_byte_index(reader_byte_index),.reader_done(reader_done),.reader_error(reader_error));
 fat32_file_streamer #(.MAX_VISITED_CLUSTERS(2),.TRACK_HISTORY(0)) stream(
 .clk(clk),.rst_n(rr),.file_start(start),.sector_allow(room_sector&&run_s2),
 .first_cluster(audio_cluster),.file_size(audio_size),.sectors_per_cluster(sectors_per_cluster),
 .fat_start_lba(fat_start_lba),.data_start_lba(data_start_lba),.max_cluster(max_cluster),
 .file_busy(file_busy),.file_done(file_done),.file_valid(file_valid),.file_byte(fb),.file_byte_offset(off),.fs_error(fe),
 .sector_req(req),.sector_lba(lba),.sector_busy(busy||!run_s2||state!=2),.sector_valid(valid),
 .sector_byte(data),.sector_byte_index(idx),.sector_done(done),.sector_error(sector_error));
 wav_pcm_parser parser(.clk(clk),.rst_n(rr),.start(start),.byte_valid(file_valid),.byte_data(fb),
 .file_size(audio_size),.file_done(file_done),.pcm_last(pcm_last),.pcm_valid(pcm_valid),.pcm_data(pcm_data),.data_done(pdone),.error(pe));
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin run_s1<=0;run_s2<=0;local_reset<=1;start<=0;state<=0;stopped<=1;eof<=0;error<=0;end
 else begin
 run_s1<=run_request;run_s2<=run_s1;start<=0;
 case(state)
 0:begin stopped<=1;local_reset<=1;eof<=0;
 if(run_s2&&storage_ready)begin stopped<=0;local_reset<=0;error<=0;state<=1;end end
 1:begin if(!audio_valid)begin error<=8'h90;state<=3;end
 else begin start<=1;state<=2;end end
 2:begin
 if(pdone)eof<=1;
 if(fe!=0||pe!=0)begin error<=fe!=0?fe:pe;state<=3;end
 // Preserve physical SD failure before the generic file layer folds it
 // into 0x10. CRC-rejected sectors still never reach the PCM parser.
 if(done&&sector_error!=0)begin
   error<=sector_error<16 ? (8'hb0|sector_error) : 8'hbf;
   state<=3;
 end
 if(!run_s2)state<=3;
 end
 3:if(!busy&&!reader_busy&&!reader_req&&!req)begin local_reset<=1;state<=4;end
 4:begin if(!run_s2)begin state<=0;stopped<=1;end end
 default:state<=0;endcase
 end end
endmodule
