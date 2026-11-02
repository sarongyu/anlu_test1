`timescale 1ns/1ps

// Two 128x128 RGB332 frames. The producer waits for the VSYNC swap
// before it is allowed to start the next frame. Each RAM has one write
// clock and one registered read clock.
module video_frame_buffer (
    input wire mem_clk, mem_rst_n,
    input wire byte_valid,
    input wire [7:0] byte_data,
    input wire [31:0] byte_offset,
    output wire sector_allow,
    output reg [31:0] frames_written,
    output reg frame_error,
    input wire pixel_clk, pixel_rst_n,
    input wire frame_start,
    input wire [14:0] pixel_address,
    output reg [7:0] pixel_index,
    output reg image_ready
);
    localparam FRAME_BYTES = 16384;
    localparam FRAME_TICKS = 8000000; // 5 fps on the 40 MHz memory clock.
    // Four 32x128 byte lanes per bank.
    reg [7:0] frame0_lane0 [0:4095], frame0_lane1 [0:4095];
    reg [7:0] frame0_lane2 [0:4095], frame0_lane3 [0:4095];
    reg [7:0] frame1_lane0 [0:4095], frame1_lane1 [0:4095];
    reg [7:0] frame1_lane2 [0:4095], frame1_lane3 [0:4095];
    reg [14:0] write_address;
    reg write_bank, ready_bank, ready_toggle, wait_swap;
    reg [22:0] frame_timer;
    reg frame_started;
    reg [31:0] expected_offset;
    reg ack_toggle;
    reg ack_meta, ack_sync, ack_seen;
    reg ready_meta, ready_sync, ready_seen, bank_meta, bank_sync;
    reg display_bank;
    reg [7:0] read00, read01, read02, read03;
    reg [7:0] read10, read11, read12, read13;
    reg [1:0] lane_q;
    reg bank_q;
    wire accept_byte = byte_valid && !wait_swap;
    wire [14:0] write_address_next = write_address + 1'b1;
    // The file streamer gates sector starts, not bytes inside a sector.
    // Never start a sector that could overrun the frame boundary.
    assign sector_allow = !wait_swap &&
        (write_address != 0 || !frame_started || frame_timer == 0);

    always @(posedge mem_clk) begin
        if (accept_byte && !write_bank && write_address[1:0] == 0) frame0_lane0[write_address[14:2]] <= byte_data;
        if (accept_byte && !write_bank && write_address[1:0] == 1) frame0_lane1[write_address[14:2]] <= byte_data;
        if (accept_byte && !write_bank && write_address[1:0] == 2) frame0_lane2[write_address[14:2]] <= byte_data;
        if (accept_byte && !write_bank && write_address[1:0] == 3) frame0_lane3[write_address[14:2]] <= byte_data;
        if (accept_byte && write_bank && write_address[1:0] == 0) frame1_lane0[write_address[14:2]] <= byte_data;
        if (accept_byte && write_bank && write_address[1:0] == 1) frame1_lane1[write_address[14:2]] <= byte_data;
        if (accept_byte && write_bank && write_address[1:0] == 2) frame1_lane2[write_address[14:2]] <= byte_data;
        if (accept_byte && write_bank && write_address[1:0] == 3) frame1_lane3[write_address[14:2]] <= byte_data;
    end
    always @(posedge pixel_clk) begin
        read00 <= frame0_lane0[pixel_address[14:2]];
        read01 <= frame0_lane1[pixel_address[14:2]];
        read02 <= frame0_lane2[pixel_address[14:2]];
        read03 <= frame0_lane3[pixel_address[14:2]];
        read10 <= frame1_lane0[pixel_address[14:2]];
        read11 <= frame1_lane1[pixel_address[14:2]];
        read12 <= frame1_lane2[pixel_address[14:2]];
        read13 <= frame1_lane3[pixel_address[14:2]];
        lane_q <= pixel_address[1:0];
        bank_q <= display_bank;
        pixel_index <= bank_q ?
            (lane_q == 0 ? read10 : lane_q == 1 ? read11 : lane_q == 2 ? read12 : read13) :
            (lane_q == 0 ? read00 : lane_q == 1 ? read01 : lane_q == 2 ? read02 : read03);
    end

    always @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin
            write_address <= 0; write_bank <= 0; ready_bank <= 0;
            ready_toggle <= 0; wait_swap <= 0; frame_timer <= 0; frame_started <= 0;
            frames_written <= 0; frame_error <= 0; expected_offset <= 0;
            ack_meta <= 0; ack_sync <= 0; ack_seen <= 0;
        end else begin
            ack_meta <= ack_toggle; ack_sync <= ack_meta;
            if (frame_timer != 0) frame_timer <= frame_timer - 1'b1;
            if (ack_sync != ack_seen) begin
                ack_seen <= ack_sync; wait_swap <= 0;
            end
            if (byte_valid && wait_swap) frame_error <= 1;
            if (accept_byte) begin
                if (write_address == 0) begin
                    frame_timer <= FRAME_TICKS - 1; frame_started <= 1;
                end
                if (byte_offset != expected_offset)
                    frame_error <= 1;
                expected_offset <= expected_offset + 1'b1;
                if (write_address == FRAME_BYTES-1) begin
                    write_address <= 0; ready_bank <= write_bank;
                    write_bank <= !write_bank; ready_toggle <= !ready_toggle;
                    wait_swap <= 1; frames_written <= frames_written + 1'b1;
                end else write_address <= write_address_next;
            end
        end
    end

    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            ready_meta <= 0; ready_sync <= 0; ready_seen <= 0;
            bank_meta <= 0; bank_sync <= 0; display_bank <= 0;
            ack_toggle <= 0; image_ready <= 0;
        end else begin
            ready_meta <= ready_toggle; ready_sync <= ready_meta;
            bank_meta <= ready_bank; bank_sync <= bank_meta;
            if (frame_start && ready_sync != ready_seen) begin
                ready_seen <= ready_sync; display_bank <= bank_sync;
                ack_toggle <= !ack_toggle; image_ready <= 1;
            end
        end
    end
endmodule
