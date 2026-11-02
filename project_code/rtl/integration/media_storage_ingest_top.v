`timescale 1ns/1ps

module media_storage_ingest_top #(
    parameter integer SPI_CLK_DIV = 4,
    parameter integer IMAGE_WIDTH = 640,
    parameter integer IMAGE_HEIGHT = 480,
    parameter integer ADDR_WIDTH = 21,
    parameter integer FIFO_DEPTH = 512,
    parameter integer STALL_TIMEOUT = 65535
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input wire audio_run,
    output wire audio_stopped,
    output wire audio_pcm_valid,
    output wire [32:0] audio_pcm_data,
    input wire audio_room_sector,
    output wire [7:0] audio_error,
    input  wire                  start,
    output wire                  busy,
    output reg                   done,
    output reg  [2:0]            error_stage,
    output reg  [7:0]            error_code,
    output reg  [7:0]            error_detail,
    output wire                  card_ready,
    output wire                  card_high_capacity,
    output wire [3:0]            resource_valid,
    output wire [3:0]            image_complete,
    input  wire                  controller_init_done,
    input  wire                  controller_busy,
    input  wire                  refresh_active,
    output wire                  memory_request, memory_active,
    output wire                  app_wr_en,
    output wire [ADDR_WIDTH-1:0] app_wr_addr,
    output wire [3:0]            app_wr_mask,
    output wire [31:0]           app_wr_data,
    output wire                  sd_cs_n,
    output wire                  sd_sclk,
    output wire                  sd_mosi,
    input  wire                  sd_miso
);
    localparam ST_IDLE = 2'd0;
    localparam ST_INIT = 2'd1;
    localparam ST_INGEST = 2'd2;
    localparam ERR_STAGE_NONE = 3'd0;
    localparam ERR_STAGE_INIT = 3'd1;
    localparam ERR_STAGE_INGEST = 3'd2;

    reg [1:0] state;
    reg init_start;
    reg ingest_start;
    wire init_done;
    wire [7:0] init_error;
    wire service_sector_req;
    wire [31:0] service_sector_lba;
    wire service_sector_busy;
    wire service_sector_done;
    wire service_sector_valid;
    wire [7:0] service_sector_byte;
    wire [8:0] service_sector_byte_index;
    wire [7:0] service_sector_error;
    wire ingest_done;
    wire [7:0] ingest_error;
    wire [7:0] ingest_detail_error;

    reg images_ready;
    wire audio_valid;wire [31:0] audio_cluster,audio_size,fat_start_lba,data_start_lba,max_cluster;
    wire [7:0] sectors_per_cluster;
    wire image_req,audio_req;wire [31:0] image_lba,audio_lba;
    wire [31:0] audio_pcm_word;wire audio_pcm_last;
    assign audio_pcm_data={audio_pcm_last,audio_pcm_word};
    assign service_sector_req=images_ready?audio_req:image_req;
    assign service_sector_lba=images_ready?audio_lba:image_lba;
    always @(posedge clk or negedge rst_n)begin
      if(!rst_n)images_ready<=0;
      else if(start)images_ready<=0;
      else if(ingest_done && (|image_complete))images_ready<=1;
    end
    wav_sd_stream u_wav(.clk(clk),.rst_n(rst_n),.run_request(audio_run),
      .storage_ready(images_ready),.audio_valid(audio_valid),.audio_cluster(audio_cluster),.audio_size(audio_size),
      .fat_start_lba(fat_start_lba),.data_start_lba(data_start_lba),.max_cluster(max_cluster),
      .sectors_per_cluster(sectors_per_cluster),.room_sector(audio_room_sector),
      .pcm_valid(audio_pcm_valid),.pcm_data(audio_pcm_word),.pcm_last(audio_pcm_last),
      .stopped(audio_stopped),.eof(),.error(audio_error),
      .reader_req(audio_req),.reader_lba(audio_lba),.reader_busy(service_sector_busy||!images_ready),
      .reader_valid(service_sector_valid&&images_ready),.reader_done(service_sector_done&&images_ready),
      .reader_byte(service_sector_byte),.reader_byte_index(service_sector_byte_index),.reader_error(service_sector_error));
    assign busy = state != ST_IDLE;

    sd_media_service #(.SPI_CLK_DIV(SPI_CLK_DIV)) u_sd_service (
        .clk(clk), .rst_n(rst_n), .init_start(init_start), .init_busy(),
        .init_done(init_done), .init_ready(card_ready), .init_error(init_error),
        .card_high_capacity(card_high_capacity), .card_ocr(), .owner_reader(),
        .sector_req(service_sector_req), .sector_lba(service_sector_lba),
        .sector_busy(service_sector_busy), .sector_done(service_sector_done),
        .sector_valid(service_sector_valid), .sector_byte(service_sector_byte),
        .sector_byte_index(service_sector_byte_index),
        .sector_error(service_sector_error), .sd_cs_n(sd_cs_n),
        .sd_sclk(sd_sclk), .sd_mosi(sd_mosi), .sd_miso(sd_miso)
    );

    media_ingest_frontend #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT),
        .ADDR_WIDTH(ADDR_WIDTH), .FIFO_DEPTH(FIFO_DEPTH),
        .STALL_TIMEOUT(STALL_TIMEOUT)
    ) u_ingest_frontend (
        .clk(clk), .rst_n(rst_n), .start(ingest_start), .busy(),
        .done(ingest_done), .ingest_error(ingest_error),
        .ingest_detail_error(ingest_detail_error), .current_image(),
        .resource_valid(resource_valid), .image_complete(image_complete),
        .audio_valid(audio_valid),.audio_cluster(audio_cluster),.audio_size(audio_size),
        .fat_start_lba(fat_start_lba),.data_start_lba(data_start_lba),.max_cluster(max_cluster),
        .sectors_per_cluster(sectors_per_cluster),
        .reader_req(image_req), .reader_lba(image_lba),
        .reader_busy(service_sector_busy), .reader_valid(service_sector_valid&&!images_ready),
        .reader_byte(service_sector_byte),
        .reader_byte_index(service_sector_byte_index),
        .reader_done(service_sector_done&&!images_ready), .reader_error(service_sector_error),
        .controller_init_done(controller_init_done),
        .controller_busy(controller_busy), .refresh_active(refresh_active),
        .memory_request(memory_request), .memory_active(memory_active), .app_wr_en(app_wr_en), .app_wr_addr(app_wr_addr),
        .app_wr_mask(app_wr_mask), .app_wr_data(app_wr_data)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            init_start <= 1'b0;
            ingest_start <= 1'b0;
            done <= 1'b0;
            error_stage <= ERR_STAGE_NONE;
            error_code <= 8'd0;
            error_detail <= 8'd0;
        end else begin
            init_start <= 1'b0;
            ingest_start <= 1'b0;
            done <= 1'b0;
            case (state)
                ST_IDLE: if (start) begin
                    error_stage <= ERR_STAGE_NONE;
                    error_code <= 8'd0;
                    error_detail <= 8'd0;
                    init_start <= 1'b1;
                    state <= ST_INIT;
                end
                ST_INIT: if (init_done) begin
                    if (!card_ready || init_error != 0) begin
                        error_stage <= ERR_STAGE_INIT;
                        error_code <= init_error;
                        error_detail <= init_error;
                        done <= 1'b1;
                        state <= ST_IDLE;
                    end else begin
                        ingest_start <= 1'b1;
                        state <= ST_INGEST;
                    end
                end
                ST_INGEST: if (ingest_done) begin
                    if (ingest_error != 0) begin
                        error_stage <= ERR_STAGE_INGEST;
                        error_code <= ingest_error;
                        error_detail <= ingest_detail_error;
                    end
                    done <= 1'b1;
                    state <= ST_IDLE;
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
