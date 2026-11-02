`timescale 1ns/1ps
// Two links back to back through a behavioural RTL8211E-like RGMII hop.
// Uses the Anlogic EG4 IDDR/ODDR simulation models. The PHY model passes
// TXD/TX_CTL across unchanged and delays RXC by RX_SKEW relative to RXD,
// like the PHY RXDLY strap. TX_SKEW checks that TXC from the 90 deg ODDR
// is centred in the TXD eye at the PHY input.
module tb_rgmii_command_link;
    parameter real RX_SKEW = 2.0;   // RXC delay after RXD (PHY RXDLY)
    reg clk50=0, rst_n=0;
    reg clk125=0, clk125_90=0;
    always #10 clk50=~clk50;
    initial begin
        #0.3;
        forever #4 clk125=~clk125;
    end
    initial begin
        #2.3;
        forever #4 clk125_90=~clk125_90;
    end

    wire [3:0] s_txd, m_txd;
    wire s_txctl, m_txctl, s_txc, m_txc;
    // PHY: RX direction regenerates the far-end stream on its own RXC.
    // Model as the far end's TXD with RXC = far TXC shifted to keep RXD
    // RX_SKEW ns ahead of RXC (data launched 2 ns before TXC already).
    wire [3:0] m_rxd; wire m_rxctl, m_rxc;
    wire [3:0] s_rxd; wire s_rxctl, s_rxc;
    assign #(1.0) m_rxd = s_txd;
    assign #(1.0) m_rxctl = s_txctl;
    assign #(1.0 + RX_SKEW - 2.0) m_rxc = s_txc;
    assign #(1.0) s_rxd = m_txd;
    assign #(1.0) s_rxctl = m_txctl;
    assign #(1.0 + RX_SKEW - 2.0) s_rxc = m_txc;

    // TX eye check at the PHY: TXD must be stable +/-1.0 ns around TXC.
    real last_d_change, last_c_edge;
    integer eye_errors=0;
    always @(s_txd or s_txctl) begin
        last_d_change=$realtime;
        if (rst_n && $realtime-last_c_edge<1.0 && $realtime>200) eye_errors=eye_errors+1;
    end
    always @(s_txc) begin
        last_c_edge=$realtime;
        if (rst_n && $realtime-last_d_change<1.0 && $realtime>200) eye_errors=eye_errors+1;
    end

    reg s_send=0, m_send=0;
    reg [7:0] s_cmd=8'h01, s_seq=8'h17;
    wire s_ready, m_ready, s_got, m_got;
    wire [7:0] s_gcmd, s_gseq, m_gcmd, m_gseq;
    wire [15:0] s_count, m_count;
    wire s_act, m_act;
    wire s_lrx, s_lgood, s_lbad, s_ltx, m_lrx, m_lgood, m_lbad, m_ltx;

    rgmii_command_link #(.LOCAL_ID(8'h02),.PEER_ID(8'h01),.STRETCH(24'd20),.STREAM_RX(1)) slave (
        .sys_clk(clk50),.rst_n(rst_n),.tx_clk(clk125),.tx_clk90(clk125_90),
        .rx_clk(s_rxc),.rx_d(s_rxd),.rx_ctl(s_rxctl),
        .tx_d(s_txd),.tx_ctl(s_txctl),.tx_clk_o(s_txc),
        .send(s_send),.send_cmd(s_cmd),.send_seq(s_seq),.send_ready(s_ready),
        .got_cmd(s_got),.got_data(s_gcmd),.got_seq(s_gseq),
        .packet_count(s_count),.activity(s_act),
        .led_rx_activity(s_lrx),.led_rx_good(s_lgood),.led_rx_bad(s_lbad),.led_tx(s_ltx),.led_stream(),
        .blk_ready(1'b0),.blk_audio(1'b0),.blk_line(9'd0),.blk_status(24'd0),.blk_take(),.blk_done(),.blk_raddr(),.blk_rdata(32'd0),
        .s_rst_n(),.s_valid(),.s_audio(),.s_byte(),.s_index(),.s_tag(),.s_status(),.s_commit(),.s_abort());
    rgmii_command_link #(.LOCAL_ID(8'h01),.PEER_ID(8'h02),.STRETCH(24'd20)) master (
        .sys_clk(clk50),.rst_n(rst_n),.tx_clk(clk125),.tx_clk90(clk125_90),
        .rx_clk(m_rxc),.rx_d(m_rxd),.rx_ctl(m_rxctl),
        .tx_d(m_txd),.tx_ctl(m_txctl),.tx_clk_o(m_txc),
        .send(m_send),.send_cmd(8'hA5),.send_seq(m_gseq),.send_ready(m_ready),
        .got_cmd(m_got),.got_data(m_gcmd),.got_seq(m_gseq),
        .packet_count(m_count),.activity(m_act),
        .led_rx_activity(m_lrx),.led_rx_good(m_lgood),.led_rx_bad(m_lbad),.led_tx(m_ltx),.led_stream(),
        .blk_ready(1'b0),.blk_audio(1'b0),.blk_line(9'd0),.blk_status(24'd0),.blk_take(),.blk_done(),.blk_raddr(),.blk_rdata(32'd0),
        .s_rst_n(),.s_valid(),.s_audio(),.s_byte(),.s_index(),.s_tag(),.s_status(),.s_commit(),.s_abort());

    // Master answers each valid command with an ACK, as net_command_policy.
    always @(posedge clk50) m_send <= m_got;

    integer m_bad_seen=0, s_bad_seen=0;
    always @(posedge clk50) begin
        if (m_lbad) m_bad_seen=m_bad_seen+1;
        if (s_lbad) s_bad_seen=s_bad_seen+1;
    end

    task send_cmd(input [7:0] c, input [7:0] q);
        begin
            @(posedge clk50); while(!s_ready) @(posedge clk50);
            s_cmd<=c; s_seq<=q; s_send<=1;
            @(posedge clk50); s_send<=0;
        end
    endtask

    initial begin
        #200 rst_n=1;
        #400;
        send_cmd(8'h01,8'h17);
        #3000;
        if (m_count!=1 || m_gcmd!=8'h01 || m_gseq!=8'h17)
            $fatal(1,"command not delivered: count=%0d cmd=%h seq=%h",m_count,m_gcmd,m_gseq);
        if (s_count!=1 || s_gcmd!=8'hA5 || s_gseq!=8'h17)
            $fatal(1,"ACK not delivered: count=%0d cmd=%h seq=%h",s_count,s_gcmd,s_gseq);
        send_cmd(8'h02,8'h18);
        #3000;
        if (m_count!=2 || m_gcmd!=8'h02 || m_gseq!=8'h18 || s_count!=2 || s_gseq!=8'h18)
            $fatal(1,"second exchange failed: m=%0d/%h/%h s=%0d/%h",m_count,m_gcmd,m_gseq,s_count,s_gseq);
        if (m_bad_seen || s_bad_seen) $fatal(1,"bad-frame indicator fired");
        if (eye_errors) $fatal(1,"TXD changed within 1 ns of TXC (%0d)",eye_errors);
        $display("PASS: command+ACK twice, CRC ok, TXC centred, RX_SKEW=%0.1f ns",RX_SKEW);
        $finish;
    end
    initial begin #20000 $fatal(1,"timeout"); end
endmodule
