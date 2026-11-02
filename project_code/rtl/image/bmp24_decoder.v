`timescale 1ns/1ps

module bmp24_decoder #(
    parameter integer EXPECTED_WIDTH = 640,
    parameter integer EXPECTED_HEIGHT = 480
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        file_start,
    input  wire [31:0] file_size,
    input  wire        file_valid,
    input  wire [7:0]  file_byte,
    input  wire [31:0] file_byte_offset,
    input  wire        file_end,
    input  wire [7:0]  upstream_error,
    output reg         header_accepted,
    output reg         pixel_valid,
    output reg  [23:0] pixel_rgb,
    output reg  [15:0] pixel_x,
    output reg  [15:0] pixel_y,
    output reg         frame_commit,
    output reg         bmp_done,
    output reg  [7:0]  bmp_error,
    output reg         top_down,
    output reg  [31:0] pixel_offset,
    output reg  [31:0] row_stride
);
    localparam ERR_NONE        = 8'h00;
    localparam ERR_SIGNATURE   = 8'h31;
    localparam ERR_DIMENSIONS  = 8'h32;
    localparam ERR_PLANES      = 8'h33;
    localparam ERR_BPP         = 8'h34;
    localparam ERR_COMPRESSION = 8'h35;
    localparam ERR_OFFSET      = 8'h36;
    localparam ERR_TRUNCATED   = 8'h37;
    localparam ERR_UPSTREAM    = 8'h38;

    // The validator only reads fields through BMP byte 33.
    reg [7:0] header [0:33];
    reg decode_enabled;
    reg [13:0] byte_in_row;
    reg [12:0] physical_row;
    reg [$clog2(EXPECTED_WIDTH*EXPECTED_HEIGHT+1)-1:0] pixels_received;
    reg [1:0] channel_phase;
    reg [7:0] blue_hold;
    reg [7:0] green_hold;
    reg [12:0] x_count;
    reg [$clog2(EXPECTED_WIDTH+1)-1:0] output_x;
    reg [$clog2(EXPECTED_HEIGHT+1)-1:0] output_y;
    reg [12:0] source_width;
    reg [12:0] source_height;
    reg [12:0] x_accum;
    reg [12:0] y_accum;
    reg row_selected;

    function [15:0] le16;
        input integer offset;
        begin
            le16 = {header[offset+1], header[offset]};
        end
    endfunction

    function [31:0] le32;
        input integer offset;
        begin
            le32 = {header[offset+3], header[offset+2], header[offset+1], header[offset]};
        end
    endfunction

    task validate_header;
        reg signed [31:0] parsed_height;
        reg [31:0] absolute_height;
        reg [31:0] parsed_offset;
        reg [31:0] parsed_stride;
        reg [31:0] required_end;
        begin
            parsed_height = le32(22);
            absolute_height = parsed_height[31] ? (~parsed_height + 1'b1) : parsed_height;
            parsed_offset = le32(10);
            parsed_stride = ((le32(18) * 3 + 3) / 4) * 4;
            required_end = parsed_offset + parsed_stride * absolute_height;
            if (header[0] != "B" || header[1] != "M") begin
                bmp_error <= ERR_SIGNATURE;
                decode_enabled <= 1'b0;
            end else if (le32(18) < EXPECTED_WIDTH || le32(18) > 4096 ||
                         absolute_height < EXPECTED_HEIGHT || absolute_height > 4096 ||
                         le32(14) < 40) begin
                bmp_error <= ERR_DIMENSIONS;
                decode_enabled <= 1'b0;
            end else if (le16(26) != 1) begin
                bmp_error <= ERR_PLANES;
                decode_enabled <= 1'b0;
            end else if (le16(28) != 24) begin
                bmp_error <= ERR_BPP;
                decode_enabled <= 1'b0;
            end else if (le32(30) != 0) begin
                bmp_error <= ERR_COMPRESSION;
                decode_enabled <= 1'b0;
            end else if (parsed_offset < 54 || parsed_offset >= file_size) begin
                bmp_error <= ERR_OFFSET;
                decode_enabled <= 1'b0;
            end else if (required_end > file_size || le32(2) < required_end || le32(2) > file_size) begin
                bmp_error <= ERR_TRUNCATED;
                decode_enabled <= 1'b0;
            end else begin
                header_accepted <= 1'b1;
                decode_enabled <= 1'b1;
                top_down <= parsed_height[31];
                pixel_offset <= parsed_offset;
                row_stride <= parsed_stride;
                source_width <= le32(18);
                source_height <= absolute_height;
                x_accum <= le32(18) - EXPECTED_WIDTH;
                y_accum <= 32'd0;
                row_selected <= 1'b1;
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            header_accepted <= 1'b0;
            pixel_valid <= 1'b0;
            pixel_rgb <= 24'd0;
            pixel_x <= 16'd0;
            pixel_y <= 16'd0;
            frame_commit <= 1'b0;
            bmp_done <= 1'b0;
            bmp_error <= ERR_NONE;
            top_down <= 1'b0;
            pixel_offset <= 32'd0;
            row_stride <= 32'd0;
            decode_enabled <= 1'b0;
            byte_in_row <= 32'd0;
            physical_row <= 32'd0;
            pixels_received <= 32'd0;
            channel_phase <= 2'd0;
            blue_hold <= 8'd0;
            green_hold <= 8'd0;
            x_count <= 16'd0;
            output_x <= 16'd0;
            output_y <= 16'd0;
            source_width <= 32'd0;
            source_height <= 32'd0;
            x_accum <= 32'd0;
            y_accum <= 32'd0;
            row_selected <= 1'b0;
        end else begin
            pixel_valid <= 1'b0;
            frame_commit <= 1'b0;
            bmp_done <= 1'b0;

            if (file_start) begin
                header_accepted <= 1'b0;
                bmp_error <= ERR_NONE;
                top_down <= 1'b0;
                pixel_offset <= 32'd0;
                row_stride <= 32'd0;
                decode_enabled <= 1'b0;
                byte_in_row <= 32'd0;
                physical_row <= 32'd0;
                pixels_received <= 32'd0;
                channel_phase <= 2'd0;
                x_count <= 16'd0;
                output_x <= 16'd0;
                output_y <= 16'd0;
                row_selected <= 1'b0;
            end

            if (file_valid && file_byte_offset < 34)
                header[file_byte_offset] <= file_byte;

            if (file_valid && file_byte_offset == 53)
                validate_header();

            if (file_valid && decode_enabled && file_byte_offset >= pixel_offset &&
                physical_row < source_height) begin
                if (byte_in_row < source_width * 3) begin
                    case (channel_phase)
                        2'd0: begin
                            blue_hold <= file_byte;
                            channel_phase <= 2'd1;
                        end
                        2'd1: begin
                            green_hold <= file_byte;
                            channel_phase <= 2'd2;
                        end
                        default: begin
                            if (x_count == 0 || x_accum + EXPECTED_WIDTH >= source_width) begin
                                if (row_selected) begin
                                    pixel_valid <= 1'b1;
                                    pixel_rgb <= {file_byte, green_hold, blue_hold};
                                    pixel_x <= output_x;
                                    pixel_y <= top_down ? output_y :
                                                       EXPECTED_HEIGHT - 1 - output_y;
                                    pixels_received <= pixels_received + 1'b1;
                                    output_x <= output_x + 1'b1;
                                end
                                x_accum <= (x_count == 0) ? 32'd0 :
                                           x_accum + EXPECTED_WIDTH - source_width;
                            end else begin
                                x_accum <= x_accum + EXPECTED_WIDTH;
                            end
                            x_count <= x_count + 1'b1;
                            channel_phase <= 2'd0;
                        end
                    endcase
                end

                if (byte_in_row == row_stride - 1'b1) begin
                    byte_in_row <= 32'd0;
                    physical_row <= physical_row + 1'b1;
                    channel_phase <= 2'd0;
                    x_count <= 16'd0;
                    output_x <= 16'd0;
                    x_accum <= source_width - EXPECTED_WIDTH;
                    if (y_accum + EXPECTED_HEIGHT >= source_height) begin
                        y_accum <= y_accum + EXPECTED_HEIGHT - source_height;
                        row_selected <= 1'b1;
                    end else begin
                        y_accum <= y_accum + EXPECTED_HEIGHT;
                        row_selected <= 1'b0;
                    end
                    if (row_selected)
                        output_y <= output_y + 1'b1;
                end else begin
                    byte_in_row <= byte_in_row + 1'b1;
                end
            end

            if (file_end) begin
                bmp_done <= 1'b1;
                decode_enabled <= 1'b0;
                if (upstream_error != 0) begin
                    bmp_error <= ERR_UPSTREAM;
                end else if (bmp_error == ERR_NONE && header_accepted &&
                             pixels_received == EXPECTED_WIDTH * EXPECTED_HEIGHT &&
                             physical_row == source_height) begin
                    frame_commit <= 1'b1;
                end else if (bmp_error == ERR_NONE) begin
                    bmp_error <= ERR_TRUNCATED;
                end
            end
        end
    end
endmodule
