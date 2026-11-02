`timescale 1ns/1ps

module media_player_control #(
    parameter [31:0] DEFAULT_PERIOD_CYCLES = 32'd250000000,
    parameter [31:0] MIN_PERIOD_CYCLES = 32'd50000000,
    parameter [31:0] MAX_PERIOD_CYCLES = 32'd3000000000,
    parameter integer RESTORE_ON_BOOT = 0
) (
    input  wire        sys_clk,
    input  wire        sys_rst_n,
    input  wire        next_event,
    input  wire        toggle_auto_event,
    input  wire        period_cfg_valid,
    input  wire [31:0] period_cfg_cycles,
    input  wire        auto_cfg_valid,
    input  wire        auto_cfg_enabled,
    input  wire        restore_valid,
    input  wire [1:0]  restore_index,
    input  wire        restore_ready,
    input  wire [3:0]  image_complete,
    output reg         auto_enabled,
    output wire        restore_done,
    output reg  [31:0] active_period_cycles,
    output reg  [1:0]  pending_index,
    output reg  [7:0]  control_error,
    output wire        request_busy,
    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire        frame_start,
    output reg  [1:0]  active_index,
    output reg manual_audio_commit,
    output reg auto_image_commit,
    output reg         index_commit
);
    localparam ERR_NONE = 8'h00;
    localparam ERR_PERIOD_RANGE = 8'h51;
    localparam ERR_REQUEST_BUSY = 8'h52;
    localparam ERR_NO_RESOURCE = 8'h53;

    reg manual_pending,manual_s1,manual_s2;
    reg automatic_pending, automatic_s1, automatic_s2;
    reg [31:0] auto_counter;
    reg request_toggle;
    reg ack_toggle_pix;
    reg ack_sync1, ack_sync2;
    reg [1:0] index_sync1, index_sync2;
    reg req_sync1, req_sync2;
    reg req_seen_pix;
    reg commit_pending_pix;
    reg restore_pending;
    reg restore_complete;
    reg restore_committed;

    assign request_busy = request_toggle != ack_sync2;
    assign restore_done = restore_complete && restore_committed &&
                          !restore_pending && !request_busy &&
                          (!RESTORE_ON_BOOT || image_complete[pending_index]);

    function [2:0] next_available;
        input [1:0] current;
        input [3:0] complete;
        integer step;
        reg [1:0] candidate;
        begin
            next_available = {1'b0, current};
            for (step = 1; step <= 4; step = step + 1) begin
                candidate = current + step[1:0];
                if (!next_available[2] && complete[candidate])
                    next_available = {1'b1, candidate};
            end
        end
    endfunction

    task issue_next_request;
        input automatic_request;
        reg [2:0] selected;
        begin
            selected = next_available(pending_index, image_complete);
            if (request_busy) begin
                control_error <= ERR_REQUEST_BUSY;
            end else if (!selected[2]) begin
                control_error <= ERR_NO_RESOURCE;
            end else begin
                pending_index <= selected[1:0];
                manual_pending <= next_event && !auto_enabled;
                automatic_pending <= automatic_request;
                request_toggle <= ~request_toggle;
                auto_counter <= 32'd0;
                // A successful issue retires any earlier sticky fault
                // (REQUEST_BUSY / NO_RESOURCE from boot-time presses), so the
                // OSD/UART error code reflects the latest outcome instead of
                // latching forever.
                control_error <= ERR_NONE;
            end
        end
    endtask

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            auto_enabled <= 1'b0; manual_pending<=0; automatic_pending<=0;
            active_period_cycles <= DEFAULT_PERIOD_CYCLES;
            pending_index <= 2'd0;
            control_error <= ERR_NONE;
            auto_counter <= 32'd0;
            restore_pending <= 1'b0;
            restore_complete <= !RESTORE_ON_BOOT;
            restore_committed <= !RESTORE_ON_BOOT;
            request_toggle <= 1'b0;
            ack_sync1 <= 1'b0;
            ack_sync2 <= 1'b0;
        end else begin
            ack_sync1 <= ack_toggle_pix;
            ack_sync2 <= ack_sync1;
            if (restore_complete && !restore_pending && !request_busy &&
                !restore_committed)
                restore_committed <= 1'b1;

            if (period_cfg_valid) begin
                if (period_cfg_cycles < MIN_PERIOD_CYCLES ||
                    period_cfg_cycles > MAX_PERIOD_CYCLES) begin
                    control_error <= ERR_PERIOD_RANGE;
                end else begin
                    active_period_cycles <= period_cfg_cycles;
                    auto_counter <= 32'd0;
                end
            end

            if (auto_cfg_valid) begin
                auto_enabled <= auto_cfg_enabled;
                auto_counter <= 32'd0;
            end
            if (restore_valid) begin
                restore_pending <= 1'b1;
                restore_complete <= 1'b0;
                restore_committed <= 1'b0;
                auto_counter <= 32'd0;
            end

            if (toggle_auto_event) begin
                auto_enabled <= ~auto_enabled;
                auto_counter <= 32'd0;
            end

            // Select a completed bank automatically when PIC1 is unavailable.
            // The normal request/ack and frame-boundary commit still apply.
            if (restore_pending && restore_ready && !request_busy) begin
                restore_pending <= 1'b0;
                restore_complete <= 1'b1;
                if (restore_index != pending_index) begin
                    pending_index <= restore_index;
                    request_toggle <= ~request_toggle;
                end else begin
                    restore_committed <= 1'b1;
                end
                auto_counter <= 32'd0;
            end else if (RESTORE_ON_BOOT &&
                         (!restore_complete || restore_pending ||
                          !restore_committed)) begin
                auto_counter <= 32'd0;
            end else if ((|image_complete) && !image_complete[pending_index] && !request_busy) begin
                issue_next_request(1'b0);
                manual_pending <= 1'b0;
            end else if (next_event) begin
                issue_next_request(1'b0);
            end else if (auto_enabled) begin
                if (request_busy)
                    auto_counter <= 32'd0;
                else if (auto_counter >= active_period_cycles - 1'b1)
                    issue_next_request(1'b1);
                else
                    auto_counter <= auto_counter + 1'b1;
            end else begin
                auto_counter <= 32'd0;
            end
        end
    end

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            index_sync1 <= 2'd0; manual_s1<=0;manual_s2<=0;manual_audio_commit<=0;
            automatic_s1<=0;automatic_s2<=0;auto_image_commit<=0;
            index_sync2 <= 2'd0;
            req_sync1 <= 1'b0;
            req_sync2 <= 1'b0;
            req_seen_pix <= 1'b0;
            commit_pending_pix <= 1'b0;
            active_index <= 2'd0;
            index_commit <= 1'b0;
            ack_toggle_pix <= 1'b0;
        end else begin
            index_sync1 <= pending_index;manual_s1<=manual_pending;manual_s2<=manual_s1;manual_audio_commit<=0;
            automatic_s1<=automatic_pending;automatic_s2<=automatic_s1;auto_image_commit<=0;
            index_sync2 <= index_sync1;
            req_sync1 <= request_toggle;
            req_sync2 <= req_sync1;
            index_commit <= 1'b0;

            // Sample the bank index at the frame_start commit, not when the
            // request toggle is detected. pending_index and request_toggle
            // launch on the same sys_clk edge through independent 2FF chains,
            // so index_sync2 can lag the toggle at detection time. By
            // frame_start the value has settled for a whole frame period and
            // cannot change again: issue_next_request is blocked by
            // request_busy until the ack sent at this very commit.
            if (req_sync2 != req_seen_pix && !commit_pending_pix) begin
                req_seen_pix <= req_sync2;
                commit_pending_pix <= 1'b1;
            end

            if (frame_start && commit_pending_pix) begin
                active_index <= index_sync2;
                index_commit <= 1'b1;manual_audio_commit<=manual_s2;auto_image_commit<=automatic_s2;
                commit_pending_pix <= 1'b0;
                ack_toggle_pix <= req_seen_pix;
            end
        end
    end
endmodule
