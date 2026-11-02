// Slaved-board keys and the main-board acknowledgement/mode controller.
module net_command_policy #(
    parameter SLAVE=0
)(
    input wire clk,rst_n,
    input wire key1_n,key2_n,
    input wire rx_pulse,input wire [7:0] rx_cmd,rx_seq,
    input wire tx_ready,
    output reg tx_send,output reg [7:0] tx_cmd,tx_seq,
    output reg fullscreen,output reg link_alive,
    output reg [7:0] last_sequence,output reg acknowledged
);
    localparam [7:0] FULL=8'h01,SMALL=8'h02,ACK=8'hA5;
    reg [1:0] key_sync1,key_sync2;
    reg [19:0] stable_count;
    reg [1:0] key_stable;
    reg [23:0] heartbeat_timer,retry_timer;
    reg [26:0] lost_timer;
    reg [7:0] command_sequence;
    reg pending;
    reg ack_pending;
    reg [7:0] ack_sequence;
    wire [1:0] keys={key2_n,key1_n};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_sync1<=2'b11;key_sync2<=2'b11;
            stable_count<=0;key_stable<=2'b11;
            heartbeat_timer<=0;retry_timer<=0;lost_timer<=0;
            command_sequence<=0;pending<=0;ack_pending<=0;ack_sequence<=0;
            tx_send<=0;tx_cmd<=SMALL;tx_seq<=0;
            fullscreen<=0;link_alive<=0;last_sequence<=0;
            acknowledged<=0;
        end else begin
            key_sync1<=keys;key_sync2<=key_sync1;
            tx_send<=0;
            if (key_sync2!=key_stable) begin
                if (stable_count==20'd1000000) begin
                    // Trigger once on press; avoid a second event on release.
                    if (SLAVE && key_stable[0]==1 && key_sync2[0]==0) begin
                        fullscreen<=1;command_sequence<=command_sequence+1'b1;
                        pending<=1;acknowledged<=0;retry_timer<=0;
                    end else if (SLAVE && key_stable[1]==1 && key_sync2[1]==0) begin
                        fullscreen<=0;command_sequence<=command_sequence+1'b1;
                        pending<=1;acknowledged<=0;retry_timer<=0;
                    end
                    key_stable<=key_sync2;stable_count<=0;
                end else stable_count<=stable_count+1'b1;
            end else stable_count<=0;

            if (heartbeat_timer==24'd14999999) heartbeat_timer<=0;
            else heartbeat_timer<=heartbeat_timer+1'b1;
            if (retry_timer!=0) retry_timer<=retry_timer-1'b1;
            if (lost_timer!=0) lost_timer<=lost_timer-1'b1;
            link_alive<=lost_timer!=0;
            if (rx_pulse) begin
                lost_timer<=27'd100000000;
                if (SLAVE) begin
                    if (rx_cmd==ACK && rx_seq==command_sequence) begin
                        acknowledged<=1;pending<=0;last_sequence<=rx_seq;
                    end
                end else if (rx_cmd==FULL || rx_cmd==SMALL) begin
                    fullscreen<=rx_cmd==FULL;
                    last_sequence<=rx_seq;
                    acknowledged<=1;
                    ack_pending<=1;ack_sequence<=rx_seq;
                end
            end
            if (tx_ready) begin
                if (SLAVE && (retry_timer==0 || heartbeat_timer==0)) begin
                    tx_send<=1;tx_cmd<=fullscreen ? FULL : SMALL;
                    tx_seq<=command_sequence;retry_timer<=24'd5000000;
                end else if (!SLAVE && ack_pending) begin
                    tx_send<=1;tx_cmd<=ACK;tx_seq<=ack_sequence;
                    ack_pending<=0;
                end
            end
        end
    end
endmodule
