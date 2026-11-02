`timescale 1ns/1ps

// Gray pointers with synchronous block-memory read and a held output slot.
module pcm_async_fifo #(parameter A=11)(input wire wc,wrst,wen,input wire [32:0] din,
 output wire room_sector,output wire prefilled,input wire rc,rrst,pop,output wire valid,output wire [32:0] dout);
 reg [32:0] mem[0:(1<<A)-1];
 reg [A:0] wp,wg,rp,rg;
 reg [A:0] rg_s1,rg_s2,wg_s1,wg_s2;
 reg [32:0] q;reg qv;reg end_written,end_s1,end_s2;
 integer i,j;reg [A:0] rbin,wbin;
 always @* begin rbin[A]=rg_s2[A];for(i=A-1;i>=0;i=i-1)rbin[i]=rbin[i+1]^rg_s2[i];end
 always @* begin wbin[A]=wg_s2[A];for(j=A-1;j>=0;j=j-1)wbin[j]=wbin[j+1]^wg_s2[j];end
 assign prefilled=(wbin-rp)>=512 || end_s2;
 assign room_sector=(wp-rbin)<((1<<A)-132);
 assign valid=qv;assign dout=q;
 wire take=(!qv||pop)&&(rg!=wg_s2);
 always @(posedge wc)if(wen)mem[wp[A-1:0]]<=din;
 always @(posedge wc or negedge wrst)begin
 if(!wrst)begin wp<=0;wg<=0;rg_s1<=0;rg_s2<=0;end_written<=0;end
 else begin rg_s1<=rg;rg_s2<=rg_s1;if(wen)begin wp<=wp+1'b1;wg<=((wp+1'b1)>>1)^(wp+1'b1);if(din[32])end_written<=1;end end end
 always @(posedge rc or negedge rrst)begin
 if(!rrst)begin rp<=0;rg<=0;wg_s1<=0;wg_s2<=0;q<=0;qv<=0;end_s1<=0;end_s2<=0;end
 else begin wg_s1<=wg;wg_s2<=wg_s1;end_s1<=end_written;end_s2<=end_s1;
 if(take)begin q<=mem[rp[A-1:0]];qv<=1;rp<=rp+1'b1;rg<=((rp+1'b1)>>1)^(rp+1'b1);end
 else if(pop)qv<=0;end end
endmodule
