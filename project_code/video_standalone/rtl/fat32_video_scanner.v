`timescale 1ns/1ps

module fat32_volume_scanner #(parameter REQUIRE_ALL_IMAGES = 0) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        scan_start,
    output wire        scan_busy,
    output reg         scan_done,
    output reg  [7:0]  fs_error,
    output reg  [31:0] partition_lba,
    output reg  [7:0]  sectors_per_cluster,
    output reg  [31:0] fat_start_lba,
    output reg  [31:0] data_start_lba,
    output reg  [31:0] max_cluster,
    output reg audio_valid,
    output reg [31:0] audio_cluster, audio_size,
    output reg  [3:0]  resource_valid,
    output reg  [31:0] resource0_cluster,
    output reg  [31:0] resource1_cluster,
    output reg  [31:0] resource2_cluster,
    output reg  [31:0] resource3_cluster,
    output reg  [31:0] resource0_size,
    output reg  [31:0] resource1_size,
    output reg  [31:0] resource2_size,
    output reg  [31:0] resource3_size,
    output reg         sector_req,
    output reg  [31:0] sector_lba,
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error
);


    // Capture only BPB fields and one directory entry as bytes arrive. This
    // avoids a 512-byte register array and many parallel variable read muxes.
    localparam IDLE=0,REQ0=1,WAIT0=2,REQBOOT=3,WAITBOOT=4,LAYOUT=5,ROOT=6,
               REQDIR=7,WAITDIR=8,NEXTDIR=9,REQFAT=10,WAITFAT=11,PARSEFAT=12,FINISH=13;
    reg [3:0] state;reg [7:0] jump,spc,nfats,ptype;reg [15:0] bps,reserved,roots,fat16,signature;
    reg [31:0] total,fat_size,bpb_root,mbr_lba,root_cluster,root_lba,fat_link,hops;
    reg [7:0] root_sector;reg [2:0] shift;
    reg [87:0] name;reg [7:0] attr;reg [31:0] entry_cluster,entry_size;
    reg dir_end;wire [4:0] k=sector_byte_index[4:0];
    wire boot_capture=state==WAIT0||state==WAITBOOT;
    assign scan_busy=state!=IDLE;
    function [2:0] spc_shift;input [7:0] x;begin case(x)
    2:spc_shift=1;4:spc_shift=2;8:spc_shift=3;16:spc_shift=4;
    32:spc_shift=5;64:spc_shift=6;128:spc_shift=7;default:spc_shift=0;endcase end endfunction
    task fail;input [7:0] e;begin fs_error<=e;state<=FINISH;end endtask
    always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
      state<=IDLE;scan_done<=0;fs_error<=0;partition_lba<=0;sectors_per_cluster<=0;
      fat_start_lba<=0;data_start_lba<=0;max_cluster<=0;resource_valid<=0;
      resource0_cluster<=0;resource1_cluster<=0;resource2_cluster<=0;resource3_cluster<=0;
      resource0_size<=0;resource1_size<=0;resource2_size<=0;resource3_size<=0;
      audio_valid<=0;audio_cluster<=0;audio_size<=0;sector_req<=0;sector_lba<=0;
      jump<=0;spc<=0;nfats<=0;ptype<=0;bps<=0;reserved<=0;roots<=0;fat16<=0;signature<=0;
      total<=0;fat_size<=0;bpb_root<=0;mbr_lba<=0;root_cluster<=0;root_lba<=0;fat_link<=0;hops<=0;
      root_sector<=0;shift<=0;name<=0;attr<=0;entry_cluster<=0;entry_size<=0;dir_end<=0;
    end else begin
      scan_done<=0;sector_req<=0;
      if(sector_valid&&boot_capture)begin
        case(sector_byte_index)
        0:jump<=sector_byte;11:bps[7:0]<=sector_byte;12:bps[15:8]<=sector_byte;
        13:spc<=sector_byte;14:reserved[7:0]<=sector_byte;15:reserved[15:8]<=sector_byte;
        16:nfats<=sector_byte;17:roots[7:0]<=sector_byte;18:roots[15:8]<=sector_byte;
        22:fat16[7:0]<=sector_byte;23:fat16[15:8]<=sector_byte;
        32:total[7:0]<=sector_byte;33:total[15:8]<=sector_byte;34:total[23:16]<=sector_byte;35:total[31:24]<=sector_byte;
        36:fat_size[7:0]<=sector_byte;37:fat_size[15:8]<=sector_byte;38:fat_size[23:16]<=sector_byte;39:fat_size[31:24]<=sector_byte;
        44:bpb_root[7:0]<=sector_byte;45:bpb_root[15:8]<=sector_byte;46:bpb_root[23:16]<=sector_byte;47:bpb_root[31:24]<={4'd0,sector_byte[3:0]};
        450:ptype<=sector_byte;454:mbr_lba[7:0]<=sector_byte;455:mbr_lba[15:8]<=sector_byte;456:mbr_lba[23:16]<=sector_byte;457:mbr_lba[31:24]<=sector_byte;
        510:signature[7:0]<=sector_byte;511:signature[15:8]<=sector_byte;default:begin end endcase
      end
      if(sector_valid&&state==WAITDIR&&!dir_end)begin
        if(k<11)name<={name[79:0],sector_byte};
        case(k)
        0:if(sector_byte==0)dir_end<=1;
        11:attr<=sector_byte;20:entry_cluster[23:16]<=sector_byte;21:entry_cluster[31:24]<={4'd0,sector_byte[3:0]};
        26:entry_cluster[7:0]<=sector_byte;27:entry_cluster[15:8]<=sector_byte;
        28:entry_size[7:0]<=sector_byte;29:entry_size[15:8]<=sector_byte;30:entry_size[23:16]<=sector_byte;
        31:begin
          if((attr&8'h18)==0)begin
            case(name)
            // Slave board: VID1.RAW (resource 0), AUDIO1.WAV (resource 1),
            // AUDIO2.WAV (resource 2).
            88'h5649443120202020524157:begin resource_valid[0]<=1;resource0_cluster<=entry_cluster;resource0_size<={sector_byte,entry_size[23:0]};end
            88'h415544494f312020574156:begin resource_valid[1]<=1;resource1_cluster<=entry_cluster;resource1_size<={sector_byte,entry_size[23:0]};end
            88'h415544494f322020574156:begin resource_valid[2]<=1;resource2_cluster<=entry_cluster;resource2_size<={sector_byte,entry_size[23:0]};end
            default:begin end endcase
          end
        end default:begin end endcase
      end
      if(sector_valid&&state==WAITFAT&&sector_byte_index[8:2]==root_cluster[6:0])begin
        case(sector_byte_index[1:0])0:fat_link[7:0]<=sector_byte;1:fat_link[15:8]<=sector_byte;
        2:fat_link[23:16]<=sector_byte;3:fat_link[31:24]<={4'd0,sector_byte[3:0]};endcase
      end
      case(state)
      IDLE:if(scan_start)begin fs_error<=0;partition_lba<=0;resource_valid<=0;audio_valid<=0;hops<=0;state<=REQ0;end
      REQ0:if(!sector_busy)begin sector_req<=1;sector_lba<=0;state<=WAIT0;end
      WAIT0:if(sector_done)begin
        if(sector_error!=0)fail(8'h10);else if(signature!=16'haa55)fail(8'h20);
        else if(jump==8'heb||jump==8'he9)state<=LAYOUT;
        else if(ptype==8'h0b||ptype==8'h0c)begin partition_lba<=mbr_lba;sector_lba<=mbr_lba;state<=REQBOOT;end
        else fail(8'h21);end
      REQBOOT:if(!sector_busy)begin sector_req<=1;state<=WAITBOOT;end
      WAITBOOT:if(sector_done)begin if(sector_error!=0)fail(8'h10);else if(signature!=16'haa55)fail(8'h20);else state<=LAYOUT;end
      LAYOUT:begin
        if(bps!=512||spc==0||(spc&(spc-1'b1))!=0||reserved==0||(nfats!=1&&nfats!=2)||
           roots!=0||fat16!=0||fat_size==0||bpb_root<2||total<=reserved+(nfats==2?(fat_size<<1):fat_size))fail(8'h21);
        else begin
          sectors_per_cluster<=spc;shift<=spc_shift(spc);fat_start_lba<=partition_lba+reserved;
          data_start_lba<=partition_lba+reserved+(nfats==2?(fat_size<<1):fat_size);
          max_cluster<=((total-reserved-(nfats==2?(fat_size<<1):fat_size))>>spc_shift(spc))+1'b1;
          root_cluster<=bpb_root;state<=ROOT;end
      end
      ROOT:begin root_sector<=0;dir_end<=0;
        root_lba<=data_start_lba+((root_cluster-2)<<shift);
        sector_lba<=data_start_lba+((root_cluster-2)<<shift);state<=REQDIR;end
      REQDIR:if(!sector_busy)begin sector_req<=1;state<=WAITDIR;end
      WAITDIR:if(sector_done)begin if(sector_error!=0)fail(8'h10);else state<=NEXTDIR;end
      NEXTDIR:begin
        if(dir_end||resource_valid[2:0]==3'b111)begin if(!REQUIRE_ALL_IMAGES || resource_valid==15)state<=FINISH;else fail(8'h22);end
        else if(root_sector+1'b1<spc)begin root_sector<=root_sector+1'b1;sector_lba<=root_lba+root_sector+1'b1;state<=REQDIR;end
        else begin sector_lba<=fat_start_lba+(root_cluster>>7);state<=REQFAT;end
      end
      REQFAT:if(!sector_busy)begin sector_req<=1;state<=WAITFAT;end
      WAITFAT:if(sector_done)begin if(sector_error!=0)fail(8'h10);else state<=PARSEFAT;end
      PARSEFAT:begin
        if(fat_link>=32'h0ffffff8)begin if(!REQUIRE_ALL_IMAGES || resource_valid==15)state<=FINISH;else fail(8'h22);end
        else if(fat_link<2||fat_link>max_cluster||hops>=max_cluster)fail(8'h24);
        else begin root_cluster<=fat_link;hops<=hops+1'b1;state<=ROOT;end
      end
      FINISH:begin scan_done<=1;state<=IDLE;end
      default:state<=IDLE;endcase
    end end
endmodule
