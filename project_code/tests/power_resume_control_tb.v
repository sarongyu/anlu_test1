`timescale 1ns/1ps
module power_resume_control_tb;
    reg sys_clk = 0, pix_clk = 0;
    always #5 sys_clk = ~sys_clk;
    always #7 pix_clk = ~pix_clk;
    reg rst_n = 0, restore_valid = 0, restore_ready = 0;
    reg [1:0] restore_index = 2'd2;
    reg [3:0] image_complete = 0;
    reg frame_start = 0, auto_cfg_valid = 0;
    wire [1:0] active_index, pending_index;
    wire auto_enabled, restore_done, request_busy;
    media_player_control #(.DEFAULT_PERIOD_CYCLES(32'd40),
        .MIN_PERIOD_CYCLES(32'd2), .MAX_PERIOD_CYCLES(32'd100),
        .RESTORE_ON_BOOT(1)) dut (
        .sys_clk(sys_clk), .sys_rst_n(rst_n), .next_event(1'b0),
        .toggle_auto_event(1'b0), .period_cfg_valid(1'b0),
        .period_cfg_cycles(32'd40), .auto_cfg_valid(auto_cfg_valid),
        .auto_cfg_enabled(1'b1), .restore_valid(restore_valid),
        .restore_index(restore_index), .restore_ready(restore_ready),
        .image_complete(image_complete), .auto_enabled(auto_enabled),
        .restore_done(restore_done), .active_period_cycles(),
        .pending_index(pending_index), .control_error(),
        .request_busy(request_busy), .pix_clk(pix_clk),
        .pix_rst_n(rst_n), .frame_start(frame_start),
        .active_index(active_index), .manual_audio_commit(),
        .auto_image_commit(), .index_commit());
    initial begin
        #33 rst_n = 1;
        @(negedge sys_clk); restore_valid = 1; auto_cfg_valid = 1;
        @(negedge sys_clk); restore_valid = 0; auto_cfg_valid = 0;
        image_complete = 4'b0001;
        repeat (15) @(negedge sys_clk);
        if (active_index !== 0 || pending_index !== 0 || restore_done)
            $fatal(1, "Advanced before saved image was ready");
        image_complete = 4'b0101; restore_ready = 1;
        repeat (6) @(negedge sys_clk);
        if (pending_index !== 2 || !request_busy || restore_done)
            $fatal(1, "Restore request was not issued");
        @(negedge pix_clk); frame_start = 1;
        @(negedge pix_clk); frame_start = 0;
        repeat (8) @(negedge sys_clk);
        if (active_index !== 2 || !restore_done || !auto_enabled)
            $fatal(1, "Saved PIC and auto mode did not resume");
        $display("PASS: waits for saved PIC and resumes auto mode");
        // Missing saved PIC: select PIC2 only after ingestion has finished.
        @(negedge sys_clk); rst_n=0; restore_ready=0;
        image_complete=0; restore_index=2'd1;
        repeat (3) @(negedge sys_clk); rst_n=1;
        @(negedge sys_clk); restore_valid=1;
        @(negedge sys_clk); restore_valid=0;
        image_complete=4'b0010;
        repeat (12) @(negedge sys_clk);
        if (restore_done || pending_index !== 0)
            $fatal(1,"Fallback happened before ingestion finished");
        restore_ready=1;
        repeat (6) @(negedge sys_clk);
        @(negedge pix_clk); frame_start=1;
        @(negedge pix_clk); frame_start=0;
        repeat (8) @(negedge sys_clk);
        if (active_index !== 1 || !restore_done)
            $fatal(1,"Missing PIC fallback failed");
        $display("PASS: missing saved PIC falls back after ingestion");
        $finish;
    end
endmodule
