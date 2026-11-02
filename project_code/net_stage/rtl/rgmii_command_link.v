`timescale 1ns/1ps
// Private EtherType link between the two boards, 1000 Mb/s RGMII only.
//
// Clocking follows the board's Anlogic UDP example (lab_ex_1):
//   * TXD/TX_CTL leave ODDRs clocked by tx_clk (125 MHz, 0 deg); TXC leaves
//     an ODDR clocked by tx_clk90 (same PLL, +90 deg), so TXC edges sit 2 ns
//     inside each nibble.
//   * RXC is used directly (global buffer, INDEL in the .adc) to sample
//     RXD/RX_CTL in IDDRs.
// IDDR q0 holds the RXC-rise sample (low nibble, RX_DV); q1 holds the
// preceding RXC-fall sample (high nibble, RX_DV^RX_ER). One register stage
// realigns them so that each byte carries its own RX_DV.
//
// Frames (byte offsets counted from the first byte after the SFD):
//   0-5 dst (00 00 00 00 00 id), 6-11 src, 12-13 EtherType, then
//   88B5 command, 64 bytes: 14-15 "V1", 16 cmd, 17 seq, 18 ~cmd, zero pad, FCS
//   88B7 picture line, 1302 bytes: 14-15 line number (0..479), 16-17 zero,
//        18.. 1280 bytes = 640 RGB565 pixels, little-endian 32-bit words
//        holding pixels 2k (bits 15:0) and 2k+1 (bits 31:16), FCS
//   88B8 status, 64 bytes: 14-16 = main-board audio state (blk_status,
//        bits 7:0 first), 17 zero, zero pad, FCS. Sent about every 1 ms.
// Commands have priority over stream blocks at frame boundaries.
//
// STREAM_TX: blocks come from media_stream_tx (blk_* port, tx_clk).
// STREAM_RX: payload bytes of 88B7 frames are presented on s_* in the
// rx_clk domain as they arrive, followed by s_commit (length, CRC and
// addresses good) or s_abort. The sink must discard data until s_commit.
// For 88B8, s_status holds the three status bytes when s_commit is set.
//
// Diagnostic LED outputs (sys_clk domain, each event stretched to STRETCH
// sys_clk cycles): led_rx_activity = any RX_DV burst, led_rx_good = valid
// command frame, led_rx_bad = RX_DV burst that failed every check,
// led_tx = frame transmitted, led_stream = valid stream frame received.
module rgmii_command_link #(
    parameter [7:0]  LOCAL_ID = 8'h01, PEER_ID = 8'h02,
    parameter [23:0] STRETCH  = 24'd2000000,
    parameter        STREAM_TX = 0,
    parameter        STREAM_RX = 0
)(
    input  wire sys_clk, rst_n,
    input  wire tx_clk, tx_clk90, rx_clk,
    input  wire [3:0] rx_d, input wire rx_ctl,
    output wire [3:0] tx_d, output wire tx_ctl, output wire tx_clk_o,
    input  wire send, input wire [7:0] send_cmd, send_seq,
    output wire send_ready,
    output reg  got_cmd, output reg [7:0] got_data, got_seq,
    output reg  [15:0] packet_count, output reg activity,
    output reg  led_rx_activity, led_rx_good, led_rx_bad, led_tx, led_stream,
    // Stream source (STREAM_TX), tx_clk domain
    input  wire        blk_ready, blk_audio,
    input  wire [8:0]  blk_line,
    input  wire [23:0] blk_status,
    output reg         blk_take, blk_done,
    output wire [8:0]  blk_raddr,
    input  wire [31:0] blk_rdata,
    // Stream sink (STREAM_RX), rx_clk domain
    output wire        s_rst_n,
    output reg         s_valid, s_audio,
    output reg  [7:0]  s_byte,
    output reg  [10:0] s_index,
    output reg  [8:0]  s_tag,
    output wire [23:0] s_status,
    output reg         s_commit, s_abort
);
    localparam [10:0] LINE_BYTES = 11'd1280, AUDIO_BYTES = 11'd46;

    function [31:0] crc_byte;
        input [31:0] old_crc;
        input [7:0] byte_in;
        reg [31:0] c;
        integer j;
        begin
            c=old_crc^byte_in;
            for (j=0;j<8;j=j+1)
                c=c[0] ? ((c>>1)^32'hEDB88320) : (c>>1);
            crc_byte=c;
        end
    endfunction

    // Asynchronous assert, synchronous release in each clock domain.
    reg [1:0] sys_rst_q, tx_rst_q, rx_rst_q;
    always @(posedge sys_clk or negedge rst_n)
        if (!rst_n) sys_rst_q<=0; else sys_rst_q<={sys_rst_q[0],1'b1};
    always @(posedge tx_clk or negedge rst_n)
        if (!rst_n) tx_rst_q<=0; else tx_rst_q<={tx_rst_q[0],1'b1};
    always @(posedge rx_clk or negedge rst_n)
        if (!rst_n) rx_rst_q<=0; else rx_rst_q<={rx_rst_q[0],1'b1};
    wire sys_rst_n=sys_rst_q[1], tx_rst_n=tx_rst_q[1], rx_rst_n=rx_rst_q[1];
    assign s_rst_n = rx_rst_n;

    // ------------------------------------------------------------------
    // Receive (rx_clk)
    // ------------------------------------------------------------------
    wire [3:0] rx_lo,rx_hi;
    wire rx_ctl_lo,rx_ctl_hi;
    genvar bit_i;
    generate for (bit_i=0;bit_i<4;bit_i=bit_i+1) begin: rx_lanes
        EG_LOGIC_IDDR #(.ASYNCRST("DISABLE"),.PIPEMODE("NONE")) u_ddr (
            .q0(rx_lo[bit_i]),.q1(rx_hi[bit_i]),
            .clk(rx_clk),.d(rx_d[bit_i]),.rst(1'b0));
    end endgenerate
    EG_LOGIC_IDDR #(.ASYNCRST("DISABLE"),.PIPEMODE("NONE")) u_rx_ctl (
        .q0(rx_ctl_lo),.q1(rx_ctl_hi),.clk(rx_clk),
        .d(rx_ctl),.rst(1'b0));

    reg [3:0] rx_lo_q;
    reg rx_dv_q;
    reg [7:0] rx_byte;
    reg rx_dv, rx_er;
    always @(posedge rx_clk or negedge rx_rst_n)
        if (!rx_rst_n) begin
            rx_lo_q<=0; rx_dv_q<=0; rx_byte<=0; rx_dv<=0; rx_er<=0;
        end else begin
            rx_lo_q<=rx_lo; rx_dv_q<=rx_ctl_lo;
            rx_byte<={rx_hi,rx_lo_q}; rx_dv<=rx_dv_q; rx_er<=rx_dv_q^rx_ctl_hi;
        end

    localparam [1:0] RX_IDLE=0, RX_PRE=1, RX_DATA=2, RX_DROP=3;
    reg [1:0] rx_state;
    reg [10:0] rx_count;
    reg [31:0] rx_crc;
    reg rx_err_seen;
    reg [7:0] dst,src,type_hi,type_lo,magic0,magic1,cmd,seq,inv_cmd;
    reg rx_good_toggle, rx_bad_toggle, rx_act_toggle, rx_stream_toggle;
    reg [7:0] rx_cmd_hold, rx_seq_hold;
    reg [15:0] rx_packet_count;
    wire rx_addr_ok = !rx_err_seen && rx_crc==32'hDEBB20E3 &&
        dst==LOCAL_ID && src==PEER_ID && type_hi==8'h88;
    wire rx_frame_ok = rx_state==RX_DATA && rx_addr_ok &&
        rx_count==11'd64 && type_lo==8'hB5 &&
        magic0==8'h56 && magic1==8'h31 && inv_cmd==(cmd^8'hFF);
    // Stream frames: type and addresses are known from byte 14 on.
    wire rx_is_stream = STREAM_RX && type_hi==8'h88 &&
        (type_lo==8'hB7 || type_lo==8'hB8) && dst==LOCAL_ID && src==PEER_ID;
    wire rx_is_audio = type_lo==8'hB8;
    wire [10:0] rx_pay_len = rx_is_audio ? AUDIO_BYTES : LINE_BYTES;
    // Offsets 14, 15, 16 = magic0, magic1, cmd = status bits 7:0, 15:8, 23:16.
    assign s_status = {cmd, magic1, magic0};
    wire rx_stream_ok = rx_state==RX_DATA && rx_is_stream && rx_addr_ok &&
        rx_count==rx_pay_len+11'd22;
    always @(posedge rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            rx_state<=RX_IDLE; rx_count<=0; rx_crc<=32'hFFFFFFFF; rx_err_seen<=0;
            dst<=0;src<=0;type_hi<=0;type_lo<=0;magic0<=0;magic1<=0;
            cmd<=0;seq<=0;inv_cmd<=0;
            rx_good_toggle<=0; rx_bad_toggle<=0; rx_act_toggle<=0; rx_stream_toggle<=0;
            rx_cmd_hold<=0; rx_seq_hold<=0; rx_packet_count<=0;
            s_valid<=0; s_audio<=0; s_byte<=0; s_index<=0; s_tag<=0;
            s_commit<=0; s_abort<=0;
        end else begin
            s_valid<=0; s_commit<=0; s_abort<=0;
            if (!rx_dv) begin
                if (rx_state!=RX_IDLE) begin
                    if (rx_frame_ok) begin
                        rx_cmd_hold<=cmd; rx_seq_hold<=seq;
                        rx_packet_count<=rx_packet_count+1'b1;
                        rx_good_toggle<=~rx_good_toggle;
                    end else if (rx_stream_ok) begin
                        rx_stream_toggle<=~rx_stream_toggle;
                    end else rx_bad_toggle<=~rx_bad_toggle;
                    if (rx_state==RX_DATA && rx_count>=11'd18 && rx_is_stream) begin
                        s_commit<=rx_stream_ok; s_abort<=!rx_stream_ok;
                    end
                end
                rx_state<=RX_IDLE;
            end else begin
                if (rx_er) rx_err_seen<=1;
                case (rx_state)
                    RX_IDLE: begin
                        rx_act_toggle<=~rx_act_toggle;
                        rx_err_seen<=rx_er;
                        rx_count<=0; rx_crc<=32'hFFFFFFFF;
                        // The PHY may shorten the preamble; hunt for the SFD.
                        rx_state<=rx_byte==8'hD5 ? RX_DATA :
                                  rx_byte==8'h55 ? RX_PRE : RX_DROP;
                    end
                    RX_PRE:
                        if (rx_byte==8'hD5) rx_state<=RX_DATA;
                        else if (rx_byte!=8'h55) rx_state<=RX_DROP;
                    RX_DATA: begin
                        if (rx_count!=11'h7FF) rx_count<=rx_count+1'b1;
                        rx_crc<=crc_byte(rx_crc,rx_byte);
                        case (rx_count)
                            5:dst<=rx_byte; 11:src<=rx_byte;
                            12:type_hi<=rx_byte; 13:type_lo<=rx_byte;
                            14:magic0<=rx_byte; 15:magic1<=rx_byte;
                            16:cmd<=rx_byte; 17:seq<=rx_byte; 18:inv_cmd<=rx_byte;
                        endcase
                        if (rx_is_stream && rx_count>=11'd18 &&
                            rx_count<rx_pay_len+11'd18) begin
                            s_valid<=1; s_byte<=rx_byte;
                            s_index<=rx_count-11'd18;
                            s_audio<=rx_is_audio; s_tag<={magic0[0],magic1};
                        end
                    end
                    default: ;
                endcase
            end
        end
    end

    // RX events into sys_clk. Hold registers change one rx_clk edge with
    // the toggle and stay stable for a whole frame, so they are read
    // directly after the three-stage toggle synchronizer.
    reg [2:0] good_meta, bad_meta, act_meta, strm_meta;
    reg [23:0] activity_timeout;
    reg [23:0] act_led_count, good_led_count, bad_led_count, tx_led_count, strm_led_count;
    reg [2:0] done_meta, txany_meta;
    reg txany_toggle;                       // tx_clk: any frame sent
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            good_meta<=0; bad_meta<=0; act_meta<=0; strm_meta<=0; txany_meta<=0;
            got_cmd<=0; got_data<=0; got_seq<=0; packet_count<=0;
            activity_timeout<=0; activity<=0;
            act_led_count<=0; good_led_count<=0; bad_led_count<=0; tx_led_count<=0;
            strm_led_count<=0;
            led_rx_activity<=0; led_rx_good<=0; led_rx_bad<=0; led_tx<=0; led_stream<=0;
        end else begin
            good_meta<={good_meta[1:0],rx_good_toggle};
            bad_meta<={bad_meta[1:0],rx_bad_toggle};
            act_meta<={act_meta[1:0],rx_act_toggle};
            strm_meta<={strm_meta[1:0],rx_stream_toggle};
            txany_meta<={txany_meta[1:0],txany_toggle};
            got_cmd<=0;
            if (activity_timeout!=0) activity_timeout<=activity_timeout-1'b1;
            activity<=activity_timeout!=0;
            if (good_meta[2]!=good_meta[1]) begin
                got_cmd<=1; got_data<=rx_cmd_hold; got_seq<=rx_seq_hold;
                packet_count<=rx_packet_count;
                activity_timeout<=24'd15000000;
            end
            act_led_count <= act_meta[2]!=act_meta[1] ? STRETCH :
                             act_led_count!=0 ? act_led_count-1'b1 : 24'd0;
            good_led_count<= good_meta[2]!=good_meta[1] ? STRETCH :
                             good_led_count!=0 ? good_led_count-1'b1 : 24'd0;
            bad_led_count <= bad_meta[2]!=bad_meta[1] ? STRETCH :
                             bad_led_count!=0 ? bad_led_count-1'b1 : 24'd0;
            tx_led_count  <= txany_meta[2]!=txany_meta[1] ? STRETCH :
                             tx_led_count!=0 ? tx_led_count-1'b1 : 24'd0;
            strm_led_count<= strm_meta[2]!=strm_meta[1] ? STRETCH :
                             strm_led_count!=0 ? strm_led_count-1'b1 : 24'd0;
            led_rx_activity<=act_led_count!=0;
            led_rx_good<=good_led_count!=0;
            led_rx_bad<=bad_led_count!=0;
            led_tx<=tx_led_count!=0;
            led_stream<=strm_led_count!=0;
        end
    end

    // ------------------------------------------------------------------
    // Send request (sys_clk) -> transmitter (tx_clk)
    // ------------------------------------------------------------------
    reg send_toggle;
    reg [7:0] command_hold,sequence_hold;
    reg done_toggle;
    assign send_ready=(send_toggle==done_meta[1]);
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            send_toggle<=0; done_meta<=0; command_hold<=0; sequence_hold<=0;
        end else begin
            done_meta<={done_meta[1:0],done_toggle};
            if (send && send_ready) begin
                command_hold<=send_cmd; sequence_hold<=send_seq;
                send_toggle<=~send_toggle;
            end
        end
    end

    // ------------------------------------------------------------------
    // Transmit (tx_clk). Stage A selects the byte for tx_index; stage B
    // updates the CRC and inserts the FCS. Both stages are full-cycle
    // paths into the ODDRs, which capture d0/d1 on the rising edge.
    // tx_index counts from the first preamble byte, i.e. it is the RX byte
    // offset plus 8.
    // ------------------------------------------------------------------
    reg [2:0] send_meta;
    reg send_seen;
    reg [7:0] tx_cmd,tx_seq;
    reg [8:0] tx_tag;
    reg [23:0] tx_status;
    reg [10:0] tx_index, tx_last, tx_fcs_first;
    reg tx_active, tx_stream, tx_audio;
    reg [3:0] gap;
    localparam [1:0] K_IDLE=0, K_PRE=1, K_DATA=2, K_FCS=3;
    reg [1:0] a_kind, a_fcs;
    reg [7:0] a_byte;
    reg [31:0] tx_crc;
    reg [7:0] tx_byte_q;
    reg tx_en_q;
    // Payload byte i is sent at tx_index 26+i; its word is addressed one
    // clock earlier because blk_rdata is a registered read.
    wire [10:0] pay_off = tx_index - 11'd25;
    assign blk_raddr = tx_index < 11'd25 ? 9'd0 : pay_off[10:2];
    wire [1:0] pay_sel = tx_index[1:0] + 2'd2;
    wire [7:0] pay_byte = pay_sel==0 ? blk_rdata[7:0] : pay_sel==1 ? blk_rdata[15:8] :
                          pay_sel==2 ? blk_rdata[23:16] : blk_rdata[31:24];
    reg [7:0] hdr_byte;
    always @* begin
        case (tx_index[4:0])
            5'd0,5'd1,5'd2,5'd3,5'd4,5'd5,5'd6: hdr_byte = 8'h55;
            5'd7:  hdr_byte = 8'hD5;
            5'd13: hdr_byte = PEER_ID;
            5'd19: hdr_byte = LOCAL_ID;
            5'd20: hdr_byte = 8'h88;
            5'd21: hdr_byte = !tx_stream ? 8'hB5 : tx_audio ? 8'hB8 : 8'hB7;
            5'd22: hdr_byte = tx_stream ? {7'd0, tx_tag[8]} : 8'h56;
            5'd23: hdr_byte = tx_stream ? tx_tag[7:0] : 8'h31;
            5'd24: hdr_byte = tx_stream ? 8'h00 : tx_cmd;
            5'd25: hdr_byte = tx_stream ? 8'h00 : tx_seq;
            default: hdr_byte = 8'h00;
        endcase
    end
    // Status frames carry blk_status in bytes 14..16 (tx_index 22..24).
    wire [7:0] st_byte = tx_index == 11'd22 ? tx_status[7:0] :
                         tx_index == 11'd23 ? tx_status[15:8] :
                         tx_index == 11'd24 ? tx_status[23:16] : 8'h00;
    wire [7:0] tx_byte_sel = (tx_stream && tx_audio) ? (tx_index < 11'd22 ? hdr_byte : st_byte) :
                             tx_index < 11'd26 ? hdr_byte :
                             tx_stream ? pay_byte :
                             tx_index == 11'd26 ? ~tx_cmd : 8'h00;
    always @(posedge tx_clk or negedge tx_rst_n) begin
        if (!tx_rst_n) begin
            send_meta<=0; send_seen<=0; done_toggle<=0; txany_toggle<=0;
            tx_cmd<=0; tx_seq<=0; tx_tag<=0; tx_status<=0; tx_index<=0; tx_last<=0; tx_fcs_first<=0;
            tx_active<=0; tx_stream<=0; tx_audio<=0; gap<=0;
            a_kind<=K_IDLE; a_fcs<=0; a_byte<=0;
            blk_take<=0; blk_done<=0;
        end else begin
            blk_take<=0; blk_done<=0;
            send_meta<={send_meta[1:0],send_toggle};
            if (gap!=0) gap<=gap-1'b1;
            if (tx_active) begin
                a_kind <= tx_index<11'd8 ? K_PRE : tx_index<tx_fcs_first ? K_DATA : K_FCS;
                a_byte <= tx_byte_sel;
                a_fcs  <= tx_index[1:0]-tx_fcs_first[1:0];
                if (tx_index==tx_last) begin
                    tx_active<=0; gap<=4'd12; txany_toggle<=~txany_toggle;
                    if (tx_stream) blk_done<=1;
                    else done_toggle<=send_seen;
                end else tx_index<=tx_index+1'b1;
            end else begin
                a_kind<=K_IDLE; a_byte<=0;
                if (gap==0 && send_meta[2]!=send_seen) begin
                    // command_hold/sequence_hold settled before the toggle.
                    send_seen<=send_meta[2];
                    tx_cmd<=command_hold; tx_seq<=sequence_hold;
                    tx_index<=0; tx_last<=11'd71; tx_fcs_first<=11'd68;
                    tx_stream<=0; tx_active<=1;
                end else if (STREAM_TX && gap==0 && blk_ready) begin
                    blk_take<=1;
                    tx_tag<=blk_line; tx_status<=blk_status; tx_audio<=blk_audio; tx_stream<=1;
                    tx_index<=0; tx_active<=1;
                    tx_last      <= blk_audio ? AUDIO_BYTES+11'd29 : LINE_BYTES+11'd29;
                    tx_fcs_first <= blk_audio ? AUDIO_BYTES+11'd26 : LINE_BYTES+11'd26;
                end
            end
        end
    end
    always @(posedge tx_clk or negedge tx_rst_n) begin
        if (!tx_rst_n) begin
            tx_crc<=32'hFFFFFFFF; tx_byte_q<=0; tx_en_q<=0;
        end else begin
            tx_en_q <= a_kind!=K_IDLE;
            case (a_kind)
                K_PRE:  begin tx_byte_q<=a_byte; tx_crc<=32'hFFFFFFFF; end
                K_DATA: begin tx_byte_q<=a_byte; tx_crc<=crc_byte(tx_crc,a_byte); end
                K_FCS:  case (a_fcs)
                            2'd0: tx_byte_q<=~tx_crc[7:0];
                            2'd1: tx_byte_q<=~tx_crc[15:8];
                            2'd2: tx_byte_q<=~tx_crc[23:16];
                            default: tx_byte_q<=~tx_crc[31:24];
                        endcase
                default: tx_byte_q<=0;
            endcase
        end
    end
    // Rising-edge half carries the low nibble, falling-edge half the high.
    generate for (bit_i=0;bit_i<4;bit_i=bit_i+1) begin: tx_lanes
        EG_LOGIC_ODDR #(.ASYNCRST("DISABLE")) u_ddr (
            .q(tx_d[bit_i]),.clk(tx_clk),.d0(tx_byte_q[bit_i]),
            .d1(tx_byte_q[bit_i+4]),.rst(1'b0));
    end endgenerate
    EG_LOGIC_ODDR #(.ASYNCRST("DISABLE")) u_tx_ctl (
        .q(tx_ctl),.clk(tx_clk),.d0(tx_en_q),.d1(tx_en_q),.rst(1'b0));
    EG_LOGIC_ODDR #(.ASYNCRST("DISABLE")) u_tx_clk (
        .q(tx_clk_o),.clk(tx_clk90),.d0(1'b1),.d1(1'b0),.rst(1'b0));
endmodule

// 50 MHz -> 125 MHz at 0 deg and 90 deg, the same PLL settings as
// pll_gen.v (clkc0/clkc4) in the lab_ex_1 UDP example for this board.
module rgmii_tx_pll (
    input  wire refclk, reset,
    output wire extlock, clk0_out, clk90_out
);
    wire clk0_buf;
    wire [2:0] unused_clk;
    EG_LOGIC_BUFG bufg_feedback (.i(clk0_buf), .o(clk0_out));
    EG_PHY_PLL #(
        .DPHASE_SOURCE("DISABLE"), .DYNCFG("DISABLE"), .FIN("50.000"),
        .FEEDBK_MODE("NORMAL"), .FEEDBK_PATH("CLKC0_EXT"),
        .STDBY_ENABLE("DISABLE"), .PLLRST_ENA("ENABLE"), .SYNC_ENABLE("DISABLE"),
        .GMC_GAIN(2), .ICP_CURRENT(9), .KVCO(2),
        .LPF_CAPACITOR(1), .LPF_RESISTOR(8),
        .REFCLK_DIV(2), .FBCLK_DIV(5),
        .CLKC0_ENABLE("ENABLE"), .CLKC0_DIV(8), .CLKC0_CPHASE(7), .CLKC0_FPHASE(0),
        .CLKC1_ENABLE("ENABLE"), .CLKC1_DIV(8), .CLKC1_CPHASE(1), .CLKC1_FPHASE(0)
    ) pll_inst (
        .refclk(refclk), .reset(reset), .stdby(1'b0), .extlock(extlock),
        .load_reg(1'b0), .psclk(1'b0), .psdown(1'b0), .psstep(1'b0),
        .psclksel(3'b000), .psdone(), .dclk(1'b0), .dcs(1'b0), .dwe(1'b0),
        .di(8'b00000000), .daddr(6'b000000), .do(),
        .fbclk(clk0_out), .clkc({unused_clk, clk90_out, clk0_buf})
    );
endmodule
