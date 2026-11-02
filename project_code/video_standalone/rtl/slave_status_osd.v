// Master style: 5x7 glyphs, 8-pixel cells, yellow FFFF00.
module slave_status_osd (
 input wire clk,rst_n,de,
 input wire [9:0] x,y,
 input wire [23:0] rgb_in,
 input wire fullscreen,auto_enabled,master_ready,settings_valid,
 input wire [7:0] volume,
 input wire [1:0] brightness,period,
 output reg [23:0] rgb_out
);
 reg full_q,auto_q,ready_q,settings_q;
 reg [7:0] percent_q,bright_q,seconds_q;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin full_q<=0;auto_q<=0;ready_q<=0;settings_q<=0;percent_q<=0;bright_q<=0;seconds_q<=0;end
  else if(x==0 && y==524) begin
   full_q<=fullscreen;auto_q<=auto_enabled;ready_q<=master_ready;settings_q<=settings_valid && master_ready;
   percent_q<=fullscreen ? 8'd100 : !master_ready ? 8'd0 : (volume*16'd100+16'd127)/16'd255;
   case(brightness) 0:bright_q<=100;1:bright_q<=75;2:bright_q<=50;default:bright_q<=25;endcase
   case(period) 0:seconds_q<=2;1:seconds_q<=5;2:seconds_q<=10;default:seconds_q<=30;endcase
  end
 end
 function [34:0] glyph;
 input [7:0] ch;
 begin case(ch)
  8'd48: glyph=35'b11111100011000110001100011000111111;
  8'd49: glyph=35'b00100011000010000100001000010011111;
  8'd50: glyph=35'b11111000010000111111100001000011111;
  8'd51: glyph=35'b11111000010000111111000010000111111;
  8'd52: glyph=35'b10001100011000111111000010000100001;
  8'd53: glyph=35'b11111100001000011111000010000111111;
  8'd54: glyph=35'b11111100001000011111100011000111111;
  8'd55: glyph=35'b11111000010001000100010000100001000;
  8'd56: glyph=35'b11111100011000111111100011000111111;
  8'd57: glyph=35'b11111100011000111111000010000111111;
  8'd65: glyph=35'b01110100011000111111100011000110001;
  8'd67: glyph=35'b01111100001000010000100001000001111;
  8'd69: glyph=35'b11111100001000011110100001000011111;
  8'd70: glyph=35'b11111100001000011110100001000010000;
  8'd73: glyph=35'b01110001000010000100001000010001110;
  8'd76: glyph=35'b10000100001000010000100001000011111;
  8'd77: glyph=35'b10001110111010110101100011000110001;
  8'd78: glyph=35'b10001110011010110011100011000110001;
  8'd79: glyph=35'b01110100011000110001100011000101110;
  8'd80: glyph=35'b11110100011000111110100001000010000;
  8'd82: glyph=35'b11110100011000111110101001001010001;
  8'd83: glyph=35'b01111100001000001110000010000111110;
  8'd84: glyph=35'b11111001000010000100001000010000100;
  8'd85: glyph=35'b10001100011000110001100011000101110;
  8'd86: glyph=35'b10001100011000110001100010101000100;
  8'd58: glyph=35'b00000001000010000000001000010000000;
  8'd45: glyph=35'b00000000000000011111000000000000000;
  8'd37: glyph=35'b11001110100010001000101100011000000;
  8'd68: glyph=35'b11110100011000110001100011000111110;
  8'd75: glyph=35'b10001100101010011000101001001010001;
  8'd66:glyph=35'b11110100011000111110100011000111110;
  8'd71:glyph=35'b01111100001000010111100011000101111;
  8'd72:glyph=35'b10001100011000111111100011000110001;
  default:glyph=0;
 endcase end endfunction

 reg [159:0] text_line;
 reg [7:0] ch;
 reg [34:0] glyph_bits;
 integer row,col,dx,dy,gx,gy,value;
 always @* begin
  rgb_out=rgb_in;row=0;col=0;dx=0;dy=0;gx=0;gy=0;value=0;
  ch=32;glyph_bits=0;text_line=0;
  if(de && x>=8 && x<168 && y>=20 && y<60) begin
   dx=x-8;dy=y-20;row=dy/10;col=dx/8;gx=dx%8;gy=dy%10;
   case(row)
    0:text_line="BRIGHT: 100%        ";
    1:text_line="VOLUME: 100%        ";
    2:text_line="PERIOD: 30S         ";
    3:text_line=full_q ? "MODE: FULL VIDEO    " : !ready_q ? "MODE: PIP NO LINK   " : auto_q ? "MODE: PIP AUTO      " : "MODE: PIP MANUAL    ";
    default:text_line=0;
   endcase
   ch=text_line >> ((19-col)*8);
   if(row==0 || row==1) begin
    value=row==0 ? bright_q : percent_q;
    if(col==8)ch=value>=100 ? 8'd49 : 8'd32;
    if(col==9)ch=value>=10 ? 8'd48+(value/10)%10 : 8'd32;
    if(col==10)ch=8'd48+value%10;
    if(col==11)ch=8'd37;
    if(row==0 && !settings_q && col>=8 && col<=11)ch=col<10 ? 8'd45 : 8'd32;
   end
   if(row==2)begin
    if(col==8)ch=seconds_q>=10 ? 8'd48+seconds_q/10 : 8'd32;
    if(col==9)ch=8'd48+seconds_q%10;
    if(col==10)ch=8'd83;
    if(!settings_q && col>=8 && col<=10)ch=col<10 ? 8'd45 : 8'd32;
   end
   glyph_bits=glyph(ch);
   if(gx<5 && gy<7 && glyph_bits[34-gy*5-gx])rgb_out=24'hffff00;
  end
 end
endmodule
