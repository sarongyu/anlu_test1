`timescale 1ns/1ps

// Slave side of plan A: turns 88B7 picture lines and 88B8 status frames
// from rgmii_command_link (rx_clk domain) into
//   * 320-word line blocks for the SDRAM frame store (mem_clk domain), and
//   * the main board's audio-playback state (pix_clk domain), which drives
//     the slave's own audio executor.
//
// Lines: a 4-slot buffer. A slot is filled while a frame arrives and only
// handed over if the link reports s_commit; s_abort reuses the slot. If all
// four slots are waiting for SDRAM the incoming line is dropped (the frame
// store keeps the previous picture for that line).
//
// Status (one per 88B8 frame, every 1 ms), bytes 14..16 of the frame:
//   s_status[0]   playback ready      s_status[1]  auto slideshow
//   s_status[9:8] displayed image     s_status[15:10] manual-commit count
//   s_status[23:16] volume; [7] extended settings valid
//   [5:4] slideshow period (2/5/10/30 s), [3:2] brightness (100/75/50/25%)
// A change of the commit count produces one manual_image_commit pulse. If
// no status arrives for STATUS_TIMEOUT pixel clocks, playback_ready drops.
module media_stream_rx #(
    parameter integer STATUS_TIMEOUT = 2500000      // 100 ms at 25 MHz
)(
    // from rgmii_command_link, rx_clk domain
    input  wire        rx_clk, rx_rst_n,
    input  wire        s_valid, s_audio,
    input  wire [7:0]  s_byte,
    input  wire [10:0] s_index,
    input  wire [8:0]  s_tag,
    input  wire [23:0] s_status,
    input  wire        s_commit, s_abort,
    // line blocks, mem_clk domain
    input  wire        mem_clk, mem_rst_n,
    output wire        line_ready,
    output wire [8:0]  line_no,
    input  wire        line_take,         // pulse: start reading this slot
    input  wire [8:0]  line_raddr,
    output reg  [31:0] line_rdata,        // registered read
    input  wire        line_done,         // pulse: slot written to SDRAM
    // main-board audio state, pix_clk domain
    input  wire        pix_clk, pix_rst_n,
    output reg         m_ready, m_auto,
    output reg  [1:0]  m_media,
    output reg  [7:0]  m_volume,
    output reg [1:0]   m_brightness, m_period,
    output reg         m_settings_valid,
    output reg         m_commit,          // one-clock pulse
    output reg  [15:0] lines_dropped      // rx_clk domain
);
    // ------------------------------------------------------------------
    // Line slots
    // ------------------------------------------------------------------
    reg [31:0] lbuf [0:2047];          // 4 slots of 512 words (320 used)
    reg [1:0]  wslot;                  // rx: slot being filled
    reg [2:0]  lcount_rx;              // rx: slots committed and not yet freed
    reg [8:0]  slot_line [0:3];
    reg [23:0] wpack;
    reg        commit_toggle;          // rx -> mem, one per committed line
    reg [2:0]  free_meta;              // rx <- mem
    reg        free_toggle;            // mem
    reg        free_seen;
    reg        filling;
    wire [10:0] lw_addr = {wslot, 9'd0} + s_index[10:2];
    always @(posedge rx_clk)
        if (s_valid && !s_audio && filling && s_index[1:0] == 2'd3)
            lbuf[lw_addr] <= {s_byte, wpack};

    // Status hold register, published by toggle (frames are >= 1.7 us apart).
    reg [23:0] status_hold;
    reg        status_toggle;
    wire slot_freed = free_meta[2] != free_seen;
    always @(posedge rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            wslot <= 0; lcount_rx <= 0; wpack <= 0; commit_toggle <= 0;
            free_meta <= 0; free_seen <= 0; filling <= 0;
            slot_line[0] <= 0; slot_line[1] <= 0; slot_line[2] <= 0; slot_line[3] <= 0;
            status_hold <= 0; status_toggle <= 0; lines_dropped <= 0;
        end else begin
            free_meta <= {free_meta[1:0], free_toggle};
            if (slot_freed) free_seen <= free_meta[2];
            if (s_valid) begin
                if (s_index == 0) begin
                    // Keep a line only if it can be stored whole.
                    filling <= !s_audio && lcount_rx < 3'd4;
                    if (!s_audio && lcount_rx >= 3'd4) lines_dropped <= lines_dropped + 1'b1;
                end
                case (s_index[1:0])
                    2'd0: wpack[7:0]   <= s_byte;
                    2'd1: wpack[15:8]  <= s_byte;
                    2'd2: wpack[23:16] <= s_byte;
                    default: ;
                endcase
            end
            if (s_commit && s_audio) begin
                status_hold <= s_status; status_toggle <= ~status_toggle;
            end
            if (s_commit && filling) begin
                slot_line[wslot] <= s_tag;
                wslot <= wslot + 1'b1;
                commit_toggle <= ~commit_toggle;
            end
            if (s_commit || s_abort) filling <= 0;
            lcount_rx <= lcount_rx
                + ((s_commit && filling) ? 3'd1 : 3'd0)
                - (slot_freed ? 3'd1 : 3'd0);
        end
    end

    // ------------------------------------------------------------------
    // Line hand-over to the memory domain
    // ------------------------------------------------------------------
    reg [2:0] commit_meta;
    reg [2:0] lpend;
    reg [1:0] rslot;
    reg       rbusy;
    assign line_ready = lpend != 0 && !rbusy;
    assign line_no = slot_line[rslot];            // stable while committed
    always @(posedge mem_clk)
        line_rdata <= lbuf[{rslot, 9'd0} + line_raddr];
    always @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin
            commit_meta <= 0; lpend <= 0; rslot <= 0; rbusy <= 0; free_toggle <= 0;
        end else begin
            commit_meta <= {commit_meta[1:0], commit_toggle};
            lpend <= lpend + (commit_meta[2] != commit_meta[1]) - line_done;
            if (line_take) rbusy <= 1;
            if (line_done) begin
                rbusy <= 0; rslot <= rslot + 1'b1; free_toggle <= ~free_toggle;
            end
        end
    end

    // ------------------------------------------------------------------
    // Main-board audio state (pix_clk)
    // ------------------------------------------------------------------
    reg [2:0]  st_meta;
    reg [5:0]  commit_seen;
    reg        st_have;
    reg [21:0] st_age;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            st_meta <= 0; commit_seen <= 0; st_have <= 0; st_age <= 0;
            m_brightness <= 0; m_period <= 0; m_settings_valid <= 0;
            m_ready <= 0; m_auto <= 0; m_media <= 0; m_volume <= 0; m_commit <= 0;
        end else begin
            st_meta <= {st_meta[1:0], status_toggle};
            m_commit <= 0;
            if (st_meta[2] != st_meta[1]) begin
                // status_hold is stable since its toggle.
                st_age <= 0;
                m_settings_valid <= status_hold[7];
                m_period <= status_hold[5:4];
                m_brightness <= status_hold[3:2];
                m_auto <= status_hold[1];
                m_media <= status_hold[9:8];
                m_volume <= status_hold[23:16];
                m_ready <= status_hold[0];
                // The first status after (re)connection only sets the
                // reference count; it is not an image change.
                if (st_have && status_hold[15:10] != commit_seen) m_commit <= 1;
                commit_seen <= status_hold[15:10];
                st_have <= 1;
            end else if (st_age != STATUS_TIMEOUT) st_age <= st_age + 1'b1;
            else begin st_have <= 0; m_ready <= 0; m_settings_valid <= 0; end
        end
    end
endmodule
