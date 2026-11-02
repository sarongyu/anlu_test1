`timescale 1ns/1ps

// Master side of plan A: captures the finished 640x480 picture (after fade,
// OSD, waveform and subtitle) and hands it to the Ethernet link as line
// blocks, plus a small status block every 1 ms that carries the audio
// playback state. The slave board plays the arpeggio and the WAV files
// itself from that state, so no PCM crosses the link.
//
// Picture: every displayed line is packed as RGB565 into 320 32-bit words
// (two pixels per word, pixel 2k in bits 15:0) and published as one block
// tagged with the line number. Lines are produced every 32 us and leave in
// about 10.5 us, so a two-line ping-pong buffer is enough. If the link has
// not taken a line by the time the next one completes, the line is dropped
// and counted; the slave keeps the previous contents of that line.
//
// Status (status_in, pixel_clk domain, sampled every 1 ms):
//   [0] playback ready   [1] auto slideshow   [9:8] displayed image
//   [15:10] count of manual image commits (wraps)   [23:16] volume
//   [7] extended settings valid; [5:4] period ladder; [3:2] brightness ladder
module media_stream_tx #(
    parameter integer STATUS_PERIOD = 25000          // 1 ms at 25 MHz
)(
    // picture, pixel_clk domain (signals aligned with each other)
    input  wire        pix_clk, pix_rst_n,
    input  wire        vid_vs, vid_de,
    input  wire [23:0] vid_rgb,
    input  wire [23:0] status_in,
    // link side, tx_clk domain
    input  wire        tx_clk, tx_rst_n,
    output wire        blk_ready,      // a block is waiting
    output wire        blk_audio,      // 1 = status block, 0 = line block
    output wire [8:0]  blk_line,       // line number (line blocks)
    output wire [23:0] blk_status,     // status block contents
    input  wire        blk_take,       // pulse: link starts sending it
    input  wire [8:0]  blk_raddr,      // word address within the block
    output reg  [31:0] blk_rdata,      // registered read, one clock later
    input  wire        blk_done,       // pulse: block fully sent
    output reg  [15:0] lines_dropped
);
    // ------------------------------------------------------------------
    // Picture capture (pix_clk)
    // ------------------------------------------------------------------
    // Simple dual-port RAM with one registered read port, maps to BRAM.
    reg [31:0] line_buf [0:1023];      // two halves of 512 (320 used)
    reg [8:0]  wcol;                   // word column 0..319
    reg [15:0] lo_px;
    reg        px_phase;
    reg [8:0]  line_no;
    reg        lw_half;
    reg        de_q, vs_q;
    reg        line_toggle;            // pix: a line completed
    reg [8:0]  line_tag [0:1];
    wire [15:0] rgb565 = {vid_rgb[23:19], vid_rgb[15:10], vid_rgb[7:3]};
    always @(posedge pix_clk)
        if (vid_de && px_phase) line_buf[{lw_half, wcol}] <= {rgb565, lo_px};

    reg [2:0] lfree_meta;
    reg lfree_toggle;                  // tx: a line half was released
    reg lfree_seen, lf_half;
    reg [1:0] lfull;
    reg drop_toggle;                   // pix: a completed line was dropped
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            wcol <= 0; lo_px <= 0; px_phase <= 0; line_no <= 0; lw_half <= 0;
            de_q <= 0; vs_q <= 1; line_toggle <= 0; lfull <= 0;
            lfree_meta <= 0; lfree_seen <= 0; lf_half <= 0; drop_toggle <= 0;
            line_tag[0] <= 0; line_tag[1] <= 0;
        end else begin
            de_q <= vid_de; vs_q <= vid_vs;
            lfree_meta <= {lfree_meta[1:0], lfree_toggle};
            if (lfree_meta[2] != lfree_seen) begin
                lfree_seen <= lfree_meta[2];
                lfull[lf_half] <= 1'b0;       // halves are sent in fill order
                lf_half <= ~lf_half;
            end
            if (!vid_vs) line_no <= 0;
            if (vid_de) begin
                px_phase <= ~px_phase;
                if (!px_phase) lo_px <= rgb565;
                else wcol <= wcol + 1'b1;
            end
            if (de_q && !vid_de) begin        // end of an active line
                wcol <= 0; px_phase <= 0;
                if (!lfull[lw_half]) begin
                    line_tag[lw_half] <= line_no;
                    lfull[lw_half] <= 1'b1;
                    lw_half <= ~lw_half;
                    line_toggle <= ~line_toggle;
                end else drop_toggle <= ~drop_toggle;
                line_no <= line_no + 1'b1;
            end
        end
    end

    // ------------------------------------------------------------------
    // Status snapshot every STATUS_PERIOD (pix_clk)
    // ------------------------------------------------------------------
    reg [23:0] status_hold;
    reg        status_toggle;
    reg [15:0] st_count;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            status_hold <= 0; status_toggle <= 0; st_count <= 0;
        end else if (st_count == STATUS_PERIOD-1) begin
            st_count <= 0;
            status_hold <= status_in; status_toggle <= ~status_toggle;
        end else st_count <= st_count + 1'b1;
    end

    // ------------------------------------------------------------------
    // Link side (tx_clk)
    // ------------------------------------------------------------------
    reg [2:0] line_meta, st_meta, drop_meta;
    reg [1:0] lpend;
    reg       spend;
    reg lr_half;
    reg cur_status;
    reg busy;
    reg [23:0] st_tx;
    assign blk_ready  = !busy && (spend || lpend != 0);
    assign blk_audio  = spend;                 // status first: tiny, periodic
    assign blk_line   = line_tag[lr_half];     // stable: half is full
    assign blk_status = st_tx;
    always @(posedge tx_clk) blk_rdata <= line_buf[{lr_half, blk_raddr}];
    always @(posedge tx_clk or negedge tx_rst_n) begin
        if (!tx_rst_n) begin
            line_meta <= 0; st_meta <= 0; drop_meta <= 0; lpend <= 0; spend <= 0;
            lr_half <= 0; cur_status <= 0; busy <= 0; st_tx <= 0;
            lfree_toggle <= 0; lines_dropped <= 0;
        end else begin
            line_meta <= {line_meta[1:0], line_toggle};
            st_meta   <= {st_meta[1:0], status_toggle};
            drop_meta <= {drop_meta[1:0], drop_toggle};
            if (drop_meta[2] != drop_meta[1]) lines_dropped <= lines_dropped + 1'b1;
            lpend <= lpend + (line_meta[2] != line_meta[1])
                   - (blk_done && !cur_status);
            // status_hold is stable for 1 ms after its toggle.
            if (st_meta[2] != st_meta[1]) begin spend <= 1; st_tx <= status_hold; end
            else if (blk_done && cur_status) spend <= 0;
            if (blk_take) begin busy <= 1; cur_status <= blk_audio; end
            if (blk_done) begin
                busy <= 0;
                if (!cur_status) begin lr_half <= ~lr_half; lfree_toggle <= ~lfree_toggle; end
            end
        end
    end
endmodule
