`timescale 1ns/1ps
// Slave link (VIDEO_TX=1) streams 2 frames x 150 sectors to the master link
// while commands and ACKs are exchanged. Checks every sector arrives once,
// in order, with good CRC, and that a command still gets its ACK.
module tb_rgmii_video_sectors;
    reg clk50=0, clk40=0, rst_n=0, clk125=0, clk125_90=0;
    always #10 clk50=~clk50;
    always #12.5 clk40=~clk40;
    initial begin #0.3; forever #4 clk125=~clk125; end
    initial begin #2.3; forever #4 clk125_90=~clk125_90; end

    wire [3:0] s_txd, m_txd; wire s_txctl, m_txctl, s_txc, m_txc;
    wire [3:0] m_rxd, s_rxd; wire m_rxctl, s_rxctl, m_rxc, s_rxc;
    assign #1.0 m_rxd = s_txd;  assign #1.0 m_rxctl = s_txctl; assign #1.0 m_rxc = s_txc;
    assign #1.0 s_rxd = m_txd;  assign #1.0 s_rxctl = m_txctl; assign #1.0 s_rxc = m_txc;

    // Byte source at SD rate: one byte per 32 clk40 (10 MHz SPI / 8 bits),
    // 512-byte sectors with a gap.
    reg vid_valid=0; reg [7:0] vid_byte=0, vid_sector=0, vid_frame=0; reg [8:0] vid_index=0;
    integer off=0, pace=0;
    always @(posedge clk40) begin
        vid_valid <= 0;
        if (rst_n && slave.vid_rst_n && off < 2*76800) begin
            pace <= pace + 1;
            if (pace % 32 == 0) begin
                vid_valid <= 1; vid_byte <= off*7 + off/512;
                vid_index <= off % 512; vid_sector <= (off % 76800)/512; vid_frame <= off/76800;
                off <= off + 1;
            end
        end
    end

    reg s_send=0, m_send=0;
    wire s_ready, m_ready, m_got, s_got;
    wire [7:0] m_gcmd, m_gseq, s_gcmd, s_gseq;
    wire [15:0] vsec_ok, vsec_seq_err; wire [7:0] vsec_frame;
    wire s_overrun;
    rgmii_command_link #(.LOCAL_ID(8'h02),.PEER_ID(8'h01),.STRETCH(24'd20),.VIDEO_TX(1)) slave (
        .sys_clk(clk50),.rst_n(rst_n),.tx_clk(clk125),.tx_clk90(clk125_90),
        .rx_clk(s_rxc),.rx_d(s_rxd),.rx_ctl(s_rxctl),
        .tx_d(s_txd),.tx_ctl(s_txctl),.tx_clk_o(s_txc),
        .send(s_send),.send_cmd(8'h01),.send_seq(8'h33),.send_ready(s_ready),
        .got_cmd(s_got),.got_data(s_gcmd),.got_seq(s_gseq),.packet_count(),.activity(),
        .led_rx_activity(),.led_rx_good(),.led_rx_bad(),.led_tx(),
        .vid_clk(clk40),.vid_valid(vid_valid),.vid_byte(vid_byte),
        .vid_sector(vid_sector),.vid_frame(vid_frame),.vid_index(vid_index),
        .vid_overrun(s_overrun),
        .vsec_ok(),.vsec_seq_err(),.vsec_frame(),.led_vsec());
    rgmii_command_link #(.LOCAL_ID(8'h01),.PEER_ID(8'h02),.STRETCH(24'd20)) master (
        .sys_clk(clk50),.rst_n(rst_n),.tx_clk(clk125),.tx_clk90(clk125_90),
        .rx_clk(m_rxc),.rx_d(m_rxd),.rx_ctl(m_rxctl),
        .tx_d(m_txd),.tx_ctl(m_txctl),.tx_clk_o(m_txc),
        .send(m_send),.send_cmd(8'hA5),.send_seq(m_gseq),.send_ready(m_ready),
        .got_cmd(m_got),.got_data(m_gcmd),.got_seq(m_gseq),.packet_count(),.activity(),
        .led_rx_activity(),.led_rx_good(),.led_rx_bad(m_bad),.led_tx(),
        .vid_clk(clk40),.vid_valid(1'b0),.vid_byte(8'd0),
        .vid_sector(8'd0),.vid_frame(8'd0),.vid_index(9'd0),.vid_overrun(),
        .vsec_ok(vsec_ok),.vsec_seq_err(vsec_seq_err),.vsec_frame(vsec_frame),.led_vsec());
    wire m_bad;
    always @(posedge clk50) m_send <= m_got;

    // Check payload of every received sector at the master's RX byte stream.
    integer bad_seen=0, payload_err=0, sec_seen=0;
    always @(posedge clk50) if (m_bad) bad_seen = bad_seen + 1;
    // Tap the master's RX bytes: payload index 0..511 is rx_count 18..529,
    // and byte value must match the generator for (frame, sector).
    integer pf, ps, pi, exp_off;
    always @(posedge master.rx_clk) if (master.rx_state==2 && master.rx_dv &&
                                         master.type_lo==8'hB6 &&
                                         master.rx_count>=18 && master.rx_count<530) begin
        pi = master.rx_count - 18;
        exp_off = master.magic0*76800 + master.magic1*512 + pi;
        if (master.rx_byte !== ((exp_off*7 + exp_off/512) & 8'hFF)) payload_err = payload_err + 1;
    end
    always @(master.rx_vsec_toggle) if (rst_n) sec_seen = sec_seen + 1;

    initial begin
        #200 rst_n = 1;
        #3000000;
        @(posedge clk50); s_send <= 1; @(posedge clk50); s_send <= 0;
        wait (off == 2*76800);
        #2000000;
        if (s_overrun) $fatal(1, "sector buffer overrun");
        if (vsec_ok != 300) $fatal(1, "vsec_ok=%0d (expected 300)", vsec_ok);
        if (vsec_seq_err != 0) $fatal(1, "sequence errors %0d", vsec_seq_err);
        if (vsec_frame != 1) $fatal(1, "last frame %0d", vsec_frame);
        if (bad_seen) $fatal(1, "bad frames at master");
        if (payload_err) $fatal(1, "%0d payload byte errors", payload_err);
        if (m_gcmd != 8'h01 || s_gcmd != 8'hA5 || s_gseq != 8'h33) $fatal(1, "command/ACK lost during video");
        $display("PASS: 300 video sectors in order with CRC; command+ACK interleaved");
        $finish;
    end
    initial begin #200000000 $fatal(1, "timeout off=%0d vsec_ok=%0d", off, vsec_ok); end
endmodule
