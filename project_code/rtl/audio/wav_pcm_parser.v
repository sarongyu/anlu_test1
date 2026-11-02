`timescale 1ns/1ps

// RIFF chunk walker, not a fixed 44-byte header assumption. PCM16 stereo 48 kHz.
module wav_pcm_parser(input wire clk,rst_n,start,byte_valid,
 input wire [7:0] byte_data,input wire [31:0] file_size,input wire file_done,
 output reg pcm_last,output reg pcm_valid,output reg [31:0] pcm_data,output reg data_done,
 output reg [7:0] error);
 reg [2:0] state;
 reg [31:0] pos,id,len,left,word_acc;
 reg [4:0] n;
 reg fmt_ok;
 reg [15:0] tag,channels,align,bits;
 reg [31:0] rate;
 wire [31:0] assembled={byte_data,word_acc[31:8]};
 always @(posedge clk or negedge rst_n) begin
 if(!rst_n) begin state<=0;pos<=0;id<=0;len<=0;left<=0;n<=0;word_acc<=0;fmt_ok<=0;
 tag<=0;channels<=0;align<=0;bits<=0;rate<=0;pcm_valid<=0;pcm_last<=0;pcm_data<=0;data_done<=0;error<=0;end
 else begin
 pcm_valid<=0;data_done<=0;
 if(start) begin state<=0;pos<=0;n<=0;word_acc<=0;fmt_ok<=0;error<=0;end
 else if(byte_valid && error==0) begin
 pos<=pos+1'b1;word_acc<=assembled;
 case(state)
 0: begin
     if(pos==3 && assembled!=32'h46464952) error<=8'h91;
     if(pos==7 && (assembled<4 || assembled>file_size-8)) error<=8'h91;
     if(pos==11) begin if(assembled!=32'h45564157)error<=8'h91;else begin state<=1;n<=0;end end
    end
 1: begin if(n==3) id<=assembled;
     if(n==7) begin
       len<=assembled;left<=assembled;n<=0;
       if(assembled>file_size-pos-1)error<=8'h92;
       else if(id==32'h61746164) begin
         if(!fmt_ok)error<=8'ha0;
         else if(assembled==0)error<=8'ha1;
         else if(assembled[1:0]!=0)error<=8'ha2;
         else state<=3;
       end else if(id==32'h20746d66) begin
         if(assembled<16)error<=8'h9f;else state<=2;
       end else if(assembled==0)state<=1;else state<=4;
     end else n<=n+1'b1;
    end
 2: begin
     case(n)
     1:tag<=assembled[31:16];3:channels<=assembled[31:16];7:rate<=assembled;
     13:align<=assembled[31:16];15:begin bits<=assembled[31:16];
       if(tag!=1)error<=8'h9a;
       else if(channels!=2)error<=8'h9b;
       else if(rate!=48000)error<=8'h9c;
       else if(align!=4)error<=8'h9d;
       else if(assembled[31:16]!=16)error<=8'h9e;
       else fmt_ok<=1;end
     endcase
     if(n<16)n<=n+1'b1;
     left<=left-1'b1;
     if(left==1)begin n<=0;state<=len[0]?5:1;end
    end
 3: begin left<=left-1'b1;
     if(n==3)begin pcm_data<=assembled;pcm_valid<=1;pcm_last<=(left==1);n<=0;end else n<=n+1'b1;
     if(left==1)begin data_done<=1;state<=6;end
    end
 4:begin left<=left-1'b1;if(left==1)begin n<=0;state<=len[0]?5:1;end end
 5:begin n<=0;state<=1;end
 default:begin end
 endcase
 end
 if(file_done && state!=6 && error==0)error<=8'h94;
 end end
endmodule
