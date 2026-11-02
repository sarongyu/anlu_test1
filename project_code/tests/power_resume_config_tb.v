`timescale 1ns/1ps
module power_resume_config_tb;
    reg clk=0, rst_n=0, load_start=0, save_start=0;
    always #5 clk=~clk;
    reg slot0_present=0, slot1_present=0;
    reg [31:0] slot0_version=0, slot0_period=0;
    reg [7:0] slot0_volume=0, slot0_mode=0;
    reg [15:0] slot0_crc=0;
    wire load_done, config_valid, record_ready;
    wire [7:0] active_mode, record_mode;
    wire [31:0] record_version, record_period;
    wire [7:0] record_volume;
    wire [15:0] record_crc;
    config_journal dut(.clk(clk),.rst_n(rst_n),.load_start(load_start),
        .slot0_present(slot0_present),.slot0_version(slot0_version),
        .slot0_period(slot0_period),.slot0_volume(slot0_volume),
        .slot0_mode(slot0_mode),.slot0_crc(slot0_crc),
        .slot1_present(slot1_present),.slot1_version(32'd0),
        .slot1_period(32'd0),.slot1_volume(8'd0),.slot1_mode(8'd0),
        .slot1_crc(16'd0),.load_done(load_done),.config_valid(config_valid),
        .active_version(),.active_period(),.active_volume(),
        .active_mode(active_mode),.save_start(save_start),
        .save_period(32'd5000),.save_volume(8'd255),
        .save_mode(8'b0000_1101),.record_ready(record_ready),
        .target_slot(),.record_version(record_version),
        .record_period(record_period),.record_volume(record_volume),
        .record_mode(record_mode),.record_crc(record_crc));
    initial begin
        #22 rst_n=1;
        @(negedge clk); save_start=1;
        @(negedge clk); save_start=0;
        slot0_present=1; slot0_version=record_version;
        slot0_period=record_period; slot0_volume=record_volume;
        slot0_mode=record_mode; slot0_crc=record_crc;
        @(negedge clk); load_start=1;
        @(negedge clk); load_start=0;
        if (!config_valid || active_mode !== 8'b0000_1101)
            $fatal(1,"New index/mode record did not load");
        $display("PASS: new format mode and PIC3 CRC record loaded");
        $finish;
    end
endmodule
