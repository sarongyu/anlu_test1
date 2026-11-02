`timescale 1ns/1ps

module media_frame_loader #(
    parameter integer ADDR_WIDTH = 21,
    parameter integer IMAGE_WORDS = 307200,
    parameter integer IMAGE_WIDTH = 640,
    parameter integer FIFO_DEPTH = 512,
    parameter integer BURST_SIZE = 64,
    parameter integer STALL_TIMEOUT = 65535
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  load_start,
    input  wire [1:0]            image_index,
    input  wire                  pixel_valid,
    input  wire [23:0]           pixel_rgb,
    input  wire                  top_down,
    input  wire                  frame_commit,
    input  wire [7:0]            decode_error,
    output wire                  load_busy,
    output wire                  memory_request, memory_active,
    output reg                   load_done,
    output reg  [3:0]            image_complete,
    output reg  [7:0]            memory_error,
    input  wire                  controller_init_done,
    input  wire                  controller_busy,
    input  wire                  refresh_active,
    output reg                   app_wr_en,
    output reg  [ADDR_WIDTH-1:0] app_wr_addr,
    output reg  [3:0]            app_wr_mask,
    output reg  [31:0]           app_wr_data
);
    localparam ERR_NONE          = 8'h00;
    localparam ERR_NOT_ACTIVE    = 8'h41;
    localparam ERR_FIFO_OVERFLOW = 8'h42;
    localparam ERR_PIXEL_COUNT   = 8'h43;
    localparam ERR_DECODE        = 8'h44;
    localparam ERR_SHORT_FRAME   = 8'h45;
    localparam ERR_WRITE_STALL   = 8'h46;

    localparam integer PTR_WIDTH = (FIFO_DEPTH <= 2) ? 1 : $clog2(FIFO_DEPTH);
    localparam integer ROW_WIDTH = (IMAGE_WORDS / IMAGE_WIDTH <= 1) ? 1 :
                                   $clog2(IMAGE_WORDS / IMAGE_WIDTH + 1);
    localparam integer COL_WIDTH = (IMAGE_WIDTH <= 2) ? 1 : $clog2(IMAGE_WIDTH);
    localparam integer COUNT_WIDTH = (IMAGE_WORDS <= 1) ? 1 :
                                     $clog2(IMAGE_WORDS + 1);
    localparam [PTR_WIDTH:0] EFFECTIVE_BURST =
        (BURST_SIZE < FIFO_DEPTH) ? BURST_SIZE : FIFO_DEPTH;
    localparam [ADDR_WIDTH-1:0] BANK0_BASE = 0;
    localparam [ADDR_WIDTH-1:0] BANK1_BASE = IMAGE_WORDS;
    localparam [ADDR_WIDTH-1:0] BANK2_BASE = IMAGE_WORDS * 2;
    localparam [ADDR_WIDTH-1:0] BANK3_BASE = IMAGE_WORDS * 3;
    // Four 320x240 previews form one 640x480 frame after the full banks.
    localparam [ADDR_WIDTH-1:0] PREVIEW_BASE = IMAGE_WORDS * 4;

    reg active;
    reg [1:0] active_image;
    reg commit_seen;
    reg [COUNT_WIDTH-1:0] pixels_received;
    reg [COUNT_WIDTH-1:0] pixels_written;
    // Track the stream position as pixels leave the FIFO. Dividing a live
    // pixel counter by 640 costs substantial logic on this small FPGA.
    reg [ROW_WIDTH-1:0] stream_row;
    reg [COL_WIDTH-1:0] stream_col;
    reg [ADDR_WIDTH-1:0] row_base;
    reg [ADDR_WIDTH-1:0] preview_row_base;
    reg [31:0] stall_count;
    reg [31:0] fifo_mem [0:FIFO_DEPTH-1];
    reg [PTR_WIDTH-1:0] wr_ptr;
    reg [PTR_WIDTH-1:0] rd_ptr;
    reg [PTR_WIDTH:0] fifo_count;
    reg burst_active;
    reg [PTR_WIDTH:0] burst_remaining;
    // A full-image write burst must remain uninterrupted. Collect up to 32
    // selected pixels from each 64-word burst, then write a separate preview
    // burst after the SDRAM controller admits it.
    // Each full-image burst has at most 64 pixels, hence at most 32
    // checkerboard samples. Drain those before admitting the next burst.
    reg [ADDR_WIDTH-1:0] preview_addr [0:31];
    reg [31:0] preview_data [0:31];
    reg [4:0] preview_wp, preview_rp;
    reg [5:0] preview_count, preview_remaining;
    reg preview_burst;
    reg preview_drain_pending;
    wire preview_push = pop_allowed && (stream_row[0] == !top_down) && !stream_col[0] &&
                        preview_count < 32;
    wire preview_pop = active && preview_burst && preview_count != 0;
    wire pop_allowed;
    wire push_allowed;
    wire start_burst;
    wire [PTR_WIDTH:0] next_burst_words;

    assign load_busy = active;
    assign memory_request = active && controller_init_done && !burst_active &&
                            !preview_burst && !app_wr_en && !preview_drain_pending &&
                            (preview_count != 0 ||
                             (fifo_count != 0 && (fifo_count >= EFFECTIVE_BURST || commit_seen)));
    assign memory_active = burst_active || preview_burst ||
                           preview_drain_pending || app_wr_en;
    // The inherited controller consumes a continuous App_wr_en burst and
    // requires idle/refresh-safe admission for a NEW burst. Those status
    // outputs are not per-word ready signals: never split an active burst.
    assign start_burst = active && controller_init_done && !burst_active &&
                         !preview_burst && !preview_drain_pending &&
                        preview_count == 0 && !app_wr_en &&
                         !controller_busy && !refresh_active &&
                         fifo_count != 0 &&
                         (fifo_count >= EFFECTIVE_BURST || commit_seen);
    assign next_burst_words = (fifo_count >= EFFECTIVE_BURST) ?
                              EFFECTIVE_BURST : fifo_count;
    assign pop_allowed = active && controller_init_done && burst_active && fifo_count != 0;
    // Bottom-up BMP files (positive height) emit the bottom row first, each
    // row left-to-right. The write address is therefore mirrored per ROW
    // (display row d takes stream row H-1-d, columns stay in order); a whole
    // stream mirror would also flip columns left-to-right. top_down files
    // (negative height) are already in display order.
    wire [ADDR_WIDTH-1:0] preview_offset = PREVIEW_BASE + preview_row_base +
        (active_image[1] ? (IMAGE_WORDS / 2) : 0) +
        (active_image[0] ? (IMAGE_WIDTH / 2) : 0) + (stream_col >> 1);
    wire [ADDR_WIDTH-1:0] write_offset = top_down ? pixels_written :
                               row_base + stream_col;
    assign push_allowed = active && pixel_valid &&
                          (fifo_count < FIFO_DEPTH || pop_allowed);

    function [ADDR_WIDTH-1:0] bank_base;
        input [1:0] index;
        begin
            case (index)
                2'd0: bank_base = BANK0_BASE;
                2'd1: bank_base = BANK1_BASE;
                2'd2: bank_base = BANK2_BASE;
                default: bank_base = BANK3_BASE;
            endcase
        end
    endfunction

    task fail_load;
        input [7:0] code;
        begin
            memory_error <= code;
            active <= 1'b0;
            burst_active <= 1'b0;
            burst_remaining <= 0;
            preview_burst <= 1'b0;
            preview_remaining <= 0;
            preview_drain_pending <= 1'b0;
            load_done <= 1'b1;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active <= 1'b0;
            active_image <= 2'd0;
            commit_seen <= 1'b0;
            pixels_received <= 32'd0;
            pixels_written <= 32'd0;
            stream_row <= 32'd0;
            stream_col <= 32'd0;
            row_base <= IMAGE_WORDS - IMAGE_WIDTH;
            preview_row_base <= 0;
            stall_count <= 32'd0;
            wr_ptr <= {PTR_WIDTH{1'b0}};
            rd_ptr <= {PTR_WIDTH{1'b0}};
            fifo_count <= {(PTR_WIDTH+1){1'b0}};
            burst_active <= 1'b0;
            burst_remaining <= 0;
            preview_burst <= 1'b0;
            preview_wp <= 0;
            preview_rp <= 0;
            preview_count <= 0;
            preview_remaining <= 0;
            preview_drain_pending <= 1'b0;
            load_done <= 1'b0;
            image_complete <= 4'd0;
            memory_error <= ERR_NONE;
            app_wr_en <= 1'b0;
            app_wr_addr <= {ADDR_WIDTH{1'b0}};
            app_wr_mask <= 4'b0000;
            app_wr_data <= 32'd0;
        end else begin
            load_done <= 1'b0;
            app_wr_en <= 1'b0;

            if (load_start && !active) begin
                active <= 1'b1;
                active_image <= image_index;
                commit_seen <= 1'b0;
                pixels_received <= 32'd0;
                pixels_written <= 32'd0;
                stream_row <= 32'd0;
                stream_col <= 32'd0;
                row_base <= IMAGE_WORDS - IMAGE_WIDTH;
                preview_row_base <= top_down ? 0 :
                    (IMAGE_WORDS / IMAGE_WIDTH / 2 - 1) * IMAGE_WIDTH;
                stall_count <= 32'd0;
                wr_ptr <= {PTR_WIDTH{1'b0}};
                rd_ptr <= {PTR_WIDTH{1'b0}};
                fifo_count <= {(PTR_WIDTH+1){1'b0}};
                burst_active <= 1'b0;
                burst_remaining <= 0;
                preview_burst <= 1'b0;
                preview_wp <= 0;
                preview_rp <= 0;
                preview_count <= 0;
                preview_remaining <= 0;
                preview_drain_pending <= 1'b0;
                memory_error <= ERR_NONE;
                case (image_index)
                    2'd0: image_complete[0] <= 1'b0;
                    2'd1: image_complete[1] <= 1'b0;
                    2'd2: image_complete[2] <= 1'b0;
                    default: image_complete[3] <= 1'b0;
                endcase
            end

            if (active) begin
                if (decode_error != 0) begin
                    fail_load(ERR_DECODE);
                end else if (pixel_valid && pixels_received >= IMAGE_WORDS) begin
                    fail_load(ERR_PIXEL_COUNT);
                end else if (pixel_valid && !push_allowed) begin
                    fail_load(ERR_FIFO_OVERFLOW);
                end else begin
                    if (start_burst) begin
                        burst_active <= 1'b1;
                        burst_remaining <= next_burst_words;
                    end
                    if (!burst_active && !preview_burst && !preview_drain_pending &&
                        preview_count != 0 &&
                        preview_count != 0 &&
                        !app_wr_en && controller_init_done &&
                        !controller_busy && !refresh_active) begin
                        preview_burst <= 1'b1;
                        preview_remaining <= preview_count;
                    end

                    if (push_allowed) begin
                        fifo_mem[wr_ptr] <= {8'h00, pixel_rgb};
                        wr_ptr <= wr_ptr + 1'b1;
                        pixels_received <= pixels_received + 1'b1;
                    end

                    if (preview_pop) begin
                        app_wr_en <= 1'b1;
                        app_wr_addr <= preview_addr[preview_rp];
                        app_wr_data <= preview_data[preview_rp];
                        app_wr_mask <= 4'b0000;
                        preview_rp <= preview_rp + 1'b1;
                        if (preview_remaining == 1) begin
                            preview_burst <= 1'b0;
                            preview_remaining <= 0;
                            preview_drain_pending <= 1'b1;
                        end else preview_remaining <= preview_remaining - 1'b1;
                    end else if (pop_allowed) begin
                        app_wr_en <= 1'b1;
                        app_wr_addr <= bank_base(active_image) + write_offset[ADDR_WIDTH-1:0];
                        app_wr_data <= fifo_mem[rd_ptr];
                        app_wr_mask <= 4'b0000;
                        rd_ptr <= rd_ptr + 1'b1;
                        pixels_written <= pixels_written + 1'b1;
                        if (stream_col == IMAGE_WIDTH - 1) begin
                            stream_col <= 32'd0;
                            stream_row <= stream_row + 1'b1;
                            row_base <= row_base - IMAGE_WIDTH;
                            if (stream_row[0] == !top_down)
                                preview_row_base <= top_down ?
                                    preview_row_base + IMAGE_WIDTH :
                                    preview_row_base - IMAGE_WIDTH;
                        end else begin
                            stream_col <= stream_col + 1'b1;
                        end
                        stall_count <= 32'd0;
                        if (burst_remaining == 1) begin
                            burst_active <= 1'b0;
                            burst_remaining <= 0;
                        end else begin
                            burst_remaining <= burst_remaining - 1'b1;
                        end
                    end else if (fifo_count != 0 &&
                                 (commit_seen || fifo_count >= EFFECTIVE_BURST)) begin
                        if (stall_count == STALL_TIMEOUT)
                            fail_load(ERR_WRITE_STALL);
                        else
                            stall_count <= stall_count + 1'b1;
                    end else begin
                        stall_count <= 32'd0;
                    end
                    if (pop_allowed && (stream_row[0] == !top_down) && !stream_col[0] &&
                        preview_count == 32) begin
                        fail_load(ERR_FIFO_OVERFLOW);
                    end
                    if (preview_push) begin
                        preview_addr[preview_wp] <= preview_offset[ADDR_WIDTH-1:0];
                        preview_data[preview_wp] <= fifo_mem[rd_ptr];
                        preview_wp <= preview_wp + 1'b1;
                    end
                    case ({preview_push,preview_pop})
                        2'b10: preview_count <= preview_count + 1'b1;
                        2'b01: preview_count <= preview_count - 1'b1;
                        default: begin end
                    endcase
                    // controller_busy includes the arbiter's writer_blocked
                    // signal, which stays asserted while memory_active holds
                    // ownership. Waiting for it here deadlocks after a preview
                    // burst. Release ownership once our last registered write
                    // pulse has drained; the arbiter waits for the real SDRAM
                    // busy signal before admitting another owner.
                    if (preview_drain_pending && !app_wr_en)
                        preview_drain_pending <= 1'b0;

                    case ({push_allowed, pop_allowed})
                        2'b10: fifo_count <= fifo_count + 1'b1;
                        2'b01: fifo_count <= fifo_count - 1'b1;
                        default: fifo_count <= fifo_count;
                    endcase

                    if (frame_commit)
                        commit_seen <= 1'b1;

                    if (commit_seen && fifo_count == 0 && !pop_allowed &&
                        preview_count == 0 && !preview_burst &&
                        !preview_drain_pending && !app_wr_en &&
                        !burst_active) begin
                        if (pixels_received != IMAGE_WORDS || pixels_written != IMAGE_WORDS) begin
                            fail_load(ERR_SHORT_FRAME);
                        end else begin
                            case (active_image)
                                2'd0: image_complete[0] <= 1'b1;
                                2'd1: image_complete[1] <= 1'b1;
                                2'd2: image_complete[2] <= 1'b1;
                                default: image_complete[3] <= 1'b1;
                            endcase
                            active <= 1'b0;
                            load_done <= 1'b1;
                        end
                    end
                end
            end else if (pixel_valid) begin
                memory_error <= ERR_NOT_ACTIVE;
            end
        end
    end
endmodule
