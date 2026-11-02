`timescale 1ns/1ps
module tb_video_frame_buffer;
  reg mem_clk=0, pixel_clk=0, mem_rst_n=0, pixel_rst_n=0;
  always #12.5 mem_clk=~mem_clk;
  always #20 pixel_clk=~pixel_clk;
  reg byte_valid=0, frame_start=0;
  reg [7:0] byte_data=0;
  reg [31:0] byte_offset=0;
  reg [14:0] pixel_address=0;
  wire sector_allow, frame_error, image_ready;
  wire [31:0] frames_written;
  wire [7:0] pixel_index;
  video_frame_buffer dut(
    .mem_clk(mem_clk),.mem_rst_n(mem_rst_n),.byte_valid(byte_valid),
    .byte_data(byte_data),.byte_offset(byte_offset),.sector_allow(sector_allow),
    .frames_written(frames_written),.frame_error(frame_error),
    .pixel_clk(pixel_clk),.pixel_rst_n(pixel_rst_n),
    .frame_start(frame_start),.pixel_address(pixel_address),
    .pixel_index(pixel_index),.image_ready(image_ready));
  integer i;
  task send_frame; input [7:0] fill; input integer base; begin
    for(i=0;i<16384;i=i+1) begin
      @(negedge mem_clk); byte_valid=1; byte_data=fill; byte_offset=base+i;
    end
    @(negedge mem_clk); byte_valid=0;
  end endtask
  task swap_frame; begin
    repeat(6) @(negedge pixel_clk); frame_start=1;
    @(negedge pixel_clk); frame_start=0;
    repeat(8) @(negedge pixel_clk);
  end endtask
  initial begin
    repeat(5) @(negedge mem_clk); mem_rst_n=1; pixel_rst_n=1;
    send_frame(8'h35,0);
    if(frames_written!==1 || frame_error || !image_ready && frames_written!==1) $fatal(1,"first frame write failed");
    swap_frame();
    pixel_address=15'd1234; repeat(5) @(negedge pixel_clk);
    if(!image_ready || pixel_index!==8'h35 || frame_error) $fatal(1,"first frame display failed %h",pixel_index);
    send_frame(8'hc9,16384); swap_frame();
    pixel_address=15'd9999; repeat(5) @(negedge pixel_clk);
    if(frames_written!==2 || pixel_index!==8'hc9 || frame_error) $fatal(1,"second frame display failed %h",pixel_index);
    $display("FRAME_BUFFER_PASS"); $finish;
  end
  initial begin #10000000; $fatal(1,"timeout"); end
endmodule
