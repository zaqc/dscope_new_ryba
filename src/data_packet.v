// ============================================================================
// Module Name:        data_packet
// File Path:          src/data_packet.v
// Description:        Prepends the 4-word System Header to the A-scan packet
//                     stream before routing to the UDP transmitter.
// ============================================================================

`timescale 1ns / 1ps

module data_packet (
    input  wire        sys_clk,
    input  wire        sys_rst_n,       // Synchronous active-low reset (from rst_sync)

    // System status flags (to be placed in the header)
    input  wire [31:0] i_sys_flags,

    // Stream interface from ascan_hub
    input  wire [31:0] i_hub_data,
    input  wire        i_hub_vld,
    output reg         o_hub_rdy,
    input  wire [15:0] i_hub_size,      // Input size in words (Metadata Header + Payload)
    input  wire        i_hub_ready,     // Signal that a complete frame is ready in the hub

    // Output packet stream (to UDP/MAC packetizer)
    output reg  [31:0] o_packet_data,
    output reg         o_packet_vld,
    input  wire        i_packet_rdy,
    output reg  [15:0] o_packet_len,    // Total packet size in 32-bit words
    output reg         o_packet_start   // 1-cycle pulse signaling start of frame
);

    // FSM States Definition
    localparam ST_IDLE    = 3'd0;
    localparam ST_START   = 3'd1;
    localparam ST_HDR_0   = 3'd2; // ASCN Magic word
    localparam ST_HDR_1   = 3'd3; // Frame Counter
    localparam ST_HDR_2   = 3'd4; // Total Words
    localparam ST_HDR_3   = 3'd5; // System Flags
    localparam ST_PAYLOAD = 3'd6; // Hub Passthrough

    reg [2:0]  r_state;
    reg [31:0] r_frame_counter;
    reg [15:0] r_hub_size;
    reg [15:0] r_total_words;
    reg [31:0] r_sys_flags;
    reg [15:0] r_word_cnt;
    reg        r_hub_ready_d1;

    // Edge-detection for the ready pulse
    wire w_hub_ready_trig = i_hub_ready && !r_hub_ready_d1;

    // Sequential State Transitions
    always @(posedge sys_clk) begin
        if (!sys_rst_n) begin
            r_state          <= ST_IDLE;
            r_frame_counter  <= 32'd0;
            r_hub_size       <= 16'd0;
            r_total_words    <= 16'd0;
            r_sys_flags      <= 32'd0;
            r_word_cnt       <= 16'd0;
            r_hub_ready_d1   <= 1'b0;
            o_packet_start   <= 1'b0;
            o_packet_len     <= 16'd0;
        end else begin
            o_packet_start   <= 1'b0; // Default 1-cycle pulse
            r_hub_ready_d1   <= i_hub_ready;

            case (r_state)
                ST_IDLE: begin
                    if (w_hub_ready_trig) begin
                        r_hub_size    <= i_hub_size;
                        r_total_words <= i_hub_size + 16'd4; // 4 words of System Header added
                        r_sys_flags   <= i_sys_flags;
                        r_state       <= ST_START;
                    end
                end

                ST_START: begin
                    o_packet_start <= 1'b1;
                    o_packet_len   <= r_total_words;
                    r_state        <= ST_HDR_0;
                end

                ST_HDR_0: begin
                    if (i_packet_rdy) begin
                        r_state    <= ST_HDR_1;
                    end
                end

                ST_HDR_1: begin
                    if (i_packet_rdy) begin
                        r_state    <= ST_HDR_2;
                    end
                end

                ST_HDR_2: begin
                    if (i_packet_rdy) begin
                        r_state    <= ST_HDR_3;
                    end
                end

                ST_HDR_3: begin
                    if (i_packet_rdy) begin
                        r_state    <= ST_PAYLOAD;
                        r_word_cnt <= 16'd0;
                    end
                end

                ST_PAYLOAD: begin
                    if (i_hub_vld && i_packet_rdy) begin
                        if (r_word_cnt == r_hub_size - 16'd1) begin
                            r_frame_counter <= r_frame_counter + 32'd1;
                            r_state         <= ST_IDLE;
                        end else begin
                            r_word_cnt      <= r_word_cnt + 16'd1;
                        end
                    end
                end

                default: r_state <= ST_IDLE;
            endcase
        end
    end

    // Combinational Output Generation (Muxing)
    always @(*) begin
        o_hub_rdy     = 1'b0;
        o_packet_vld  = 1'b0;
        o_packet_data = 32'd0;

        case (r_state)
            ST_HDR_0: begin
                o_packet_data = 32'h4153434E; // Magic word: "ASCN"
                o_packet_vld  = 1'b1;
            end

            ST_HDR_1: begin
                o_packet_data = r_frame_counter;
                o_packet_vld  = 1'b1;
            end

            ST_HDR_2: begin
                o_packet_data = {16'b0, r_total_words};
                o_packet_vld  = 1'b1;
            end

            ST_HDR_3: begin
                o_packet_data = r_sys_flags;
                o_packet_vld  = 1'b1;
            end

            ST_PAYLOAD: begin
                o_packet_data = i_hub_data;
                o_packet_vld  = i_hub_vld;
                o_hub_rdy     = i_packet_rdy;
            end

            default: ;
        endcase
    end

endmodule