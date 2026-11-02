`timescale 1ns/1ps
// Behavioural SDHC card in SPI mode, enough for sd_spi_init and
// sd_spi_sector_reader: CMD0, CMD8, CMD55/ACMD41, CMD58, CMD17 with data
// token and CRC16. Mode 0: sample MOSI on SCLK rise, shift MISO on fall.
// Image: one byte per line in IMAGE ($readmemh), addressed by block.
module sd_card_model #(parameter IMAGE = "sd_image.hex", parameter integer BLOCKS = 512,
                       parameter integer READ_DELAY = 8)(
    input wire cs_n, sclk, mosi,
    output reg miso
);
    reg [7:0] mem [0:BLOCKS*512-1];
    initial $readmemh(IMAGE, mem);

    reg [7:0] rx_sh; reg [2:0] rx_bit;
    reg [7:0] cmd [0:5]; integer cmd_n;
    reg [7:0] out_q [0:600]; integer out_n, out_i;
    reg [7:0] tx_sh; reg [2:0] tx_bit; reg tx_active;
    reg app_cmd; integer acmd41_count;
    initial begin
        miso = 1; rx_bit = 0; cmd_n = 0; out_n = 0; out_i = 0;
        tx_active = 0; tx_bit = 0; tx_sh = 8'hff; app_cmd = 0; acmd41_count = 0;
    end
    function [15:0] crc16;
        input [15:0] c; input [7:0] d;
        integer k; reg [15:0] r;
        begin
            r = c;
            for (k = 7; k >= 0; k = k - 1)
                r = (r[15] ^ d[k]) ? ((r << 1) ^ 16'h1021) : (r << 1);
            crc16 = r;
        end
    endfunction
    task respond;
        integer k; reg [15:0] c; reg [31:0] arg; reg [5:0] idx;
        begin
            idx = cmd[0][5:0];
            arg = {cmd[1], cmd[2], cmd[3], cmd[4]};
            out_n = 0; out_i = 0;
            // one or two idle bytes before R1
            out_q[out_n] = 8'hff; out_n = out_n + 1;
            case (idx)
                6'd0: begin out_q[out_n] = 8'h01; out_n = out_n + 1; end
                6'd8: begin
                    out_q[out_n] = 8'h01; out_q[out_n+1] = 8'h00; out_q[out_n+2] = 8'h00;
                    out_q[out_n+3] = 8'h01; out_q[out_n+4] = 8'haa; out_n = out_n + 5;
                end
                6'd55: begin out_q[out_n] = 8'h01; out_n = out_n + 1; end
                6'd41: begin
                    acmd41_count = acmd41_count + 1;
                    out_q[out_n] = (acmd41_count >= 2) ? 8'h00 : 8'h01; out_n = out_n + 1;
                end
                6'd58: begin
                    out_q[out_n] = 8'h00; out_q[out_n+1] = 8'hc0; out_q[out_n+2] = 8'hff;
                    out_q[out_n+3] = 8'h80; out_q[out_n+4] = 8'h00; out_n = out_n + 5;
                end
                6'd17: begin
                    out_q[out_n] = 8'h00; out_n = out_n + 1;
                    for (k = 0; k < READ_DELAY; k = k + 1) begin out_q[out_n] = 8'hff; out_n = out_n + 1; end
                    out_q[out_n] = 8'hfe; out_n = out_n + 1;
                    c = 0;
                    for (k = 0; k < 512; k = k + 1) begin
                        out_q[out_n] = (arg < BLOCKS) ? mem[arg*512 + k] : 8'h00;
                        c = crc16(c, out_q[out_n]);
                        out_n = out_n + 1;
                    end
                    out_q[out_n] = c[15:8]; out_q[out_n+1] = c[7:0]; out_n = out_n + 2;
                end
                default: begin out_q[out_n] = 8'h04; out_n = out_n + 1; end
            endcase
            app_cmd = (idx == 6'd55);
        end
    endtask

    // Receive on rising edge
    always @(posedge sclk) if (!cs_n) begin
        rx_sh = {rx_sh[6:0], mosi};
        rx_bit = rx_bit + 1;
        if (rx_bit == 0) begin
            if (cmd_n == 0) begin
                if (rx_sh[7:6] == 2'b01) begin cmd[0] = rx_sh; cmd_n = 1; end
            end else begin
                cmd[cmd_n] = rx_sh; cmd_n = cmd_n + 1;
                if (cmd_n == 6) begin cmd_n = 0; respond; end
            end
        end
    end
    // Transmit. The host samples MISO when it raises SCLK, so the next bit
    // is put out after every falling edge (and before the first one).
    // tx_bit counts bits already sampled within the current byte.
    task load_next;
        begin
            if (out_i < out_n) begin tx_sh = out_q[out_i]; out_i = out_i + 1; end
            else tx_sh = 8'hff;
        end
    endtask
    always @(posedge sclk) if (!cs_n) tx_bit = tx_bit + 1;
    always @(negedge sclk or negedge cs_n or posedge cs_n) begin
        if (cs_n) begin miso = 1; tx_bit = 0; tx_active = 0; end
        else begin
            if (!tx_active || tx_bit == 0) begin load_next; tx_active = 1; end
            miso = tx_sh[7 - tx_bit];
        end
    end
    initial miso = 1;
endmodule
