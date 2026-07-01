// =========================================================================
// Global Project Module: ascan_hub (4-Channel Receiver Concentrator)
// Description: Instantiates 4 independent, parallel physical channels of
//              ascan pipelines. Aggregates data and block sizes from all
//              channels and serializes them into a single packet stream 
//              with a fixed header using a Step-First Packetizer FSM 
//              in sys_clk domain with purely synchronous active-low resets.
// =========================================================================

`default_nettype none

module ascan_hub #(
    parameter ADDR_WIDTH = 15 // Depth of each ping-pong buffer = 32768 words (128 KB)
)(
    // Clocks and synchronous resets (from central rst_sync)
    input  wire                    adc_clk,
    input  wire                    adc_rst_n,
    input  wire                    sys_clk,
    input  wire                    sys_rst_n,

    // Controls from central sync and sequencer (adc_clk domain)
    input  wire                    i_adc_sync,       // Global frame start
    input  wire                    i_sub_sync,       // Trigger start of subsequent sub-run step
    input  wire                    i_sub_last,       // Final step indicator of current sequence
    output wire [3:0]              o_adc_chan_ready, // Aggregated level readiness (connected to sequencer)

    // ADC inputs (adc_clk domain)
    input  wire signed [11:0]      i_adc_data_ch0,
    input  wire signed [11:0]      i_adc_data_ch1,
    input  wire signed [11:0]      i_adc_data_ch2,
    input  wire signed [11:0]      i_adc_data_ch3,

    // Channel 0 Configuration Parameters (captured on adc_clk triggers)
    input  wire [15:0]             i_ascan_ch0_n_samples,
    input  wire [7:0]              i_ascan_ch0_accum,
    input  wire [1:0]              i_ascan_ch0_accum_type,
    input  wire [15:0]             i_ascan_ch0_skip_ticks,

    // Channel 1 Configuration Parameters (captured on adc_clk triggers)
    input  wire [15:0]             i_ascan_ch1_n_samples,
    input  wire [7:0]              i_ascan_ch1_accum,
    input  wire [1:0]              i_ascan_ch1_accum_type,
    input  wire [15:0]             i_ascan_ch1_skip_ticks,

    // Channel 2 Configuration Parameters (captured on adc_clk triggers)
    input  wire [15:0]             i_ascan_ch2_n_samples,
    input  wire [7:0]              i_ascan_ch2_accum,
    input  wire [1:0]              i_ascan_ch2_accum_type,
    input  wire [15:0]             i_ascan_ch2_skip_ticks,

    // Channel 3 Configuration Parameters (captured on adc_clk triggers)
    input  wire [15:0]             i_ascan_ch3_n_samples,
    input  wire [7:0]              i_ascan_ch3_accum,
    input  wire [1:0]              i_ascan_ch3_accum_type,
    input  wire [15:0]             i_ascan_ch3_skip_ticks,

    // -------------------------------------------------------------------------
    // Joint Output Stream Interface (sys_clk domain)
    // -------------------------------------------------------------------------
    output wire [31:0]             o_out_data,
    output wire                    o_out_vld,
    input  wire                    i_out_rdy,
    output wire [15:0]             o_out_size,
    output wire                    o_data_ready
);

    // =========================================================================
    // Internal Wire Arrays for Generate Loop Mapping
    // =========================================================================

    // ADC inputs (adc_clk domain)
    wire signed [11:0]      adc_data_arr   [0:3];

    // Configuration Parameters (adc_clk domain)
    wire [15:0]             n_samples_arr  [0:3];
    wire [7:0]              accum_arr      [0:3];
    wire [1:0]              accum_type_arr [0:3];
    wire [15:0]             skip_ticks_arr [0:3];

    // Readout Stream Interfaces from individual channels (sys_clk domain)
    wire [31:0]             out_data_arr   [0:3];
    wire                    out_vld_arr    [0:3];
    wire                    out_rdy_arr    [0:3];
    wire [15:0]             out_size0_arr  [0:3];
    wire [15:0]             out_size1_arr  [0:3];
    wire [15:0]             out_size2_arr  [0:3];
    wire [15:0]             out_size3_arr  [0:3];
    wire [15:0]             out_size_arr   [0:3];
    wire                    data_ready_arr [0:3];

    // =========================================================================
    // Assigning Inputs to Arrays
    // =========================================================================
    assign adc_data_arr[0] = i_adc_data_ch0;
    assign adc_data_arr[1] = i_adc_data_ch1;
    assign adc_data_arr[2] = i_adc_data_ch2;
    assign adc_data_arr[3] = i_adc_data_ch3;

    assign n_samples_arr[0] = i_ascan_ch0_n_samples;
    assign n_samples_arr[1] = i_ascan_ch1_n_samples;
    assign n_samples_arr[2] = i_ascan_ch2_n_samples;
    assign n_samples_arr[3] = i_ascan_ch3_n_samples;

    assign accum_arr[0] = i_ascan_ch0_accum;
    assign accum_arr[1] = i_ascan_ch1_accum;
    assign accum_arr[2] = i_ascan_ch2_accum;
    assign accum_arr[3] = i_ascan_ch3_accum;

    assign accum_type_arr[0] = i_ascan_ch0_accum_type;
    assign accum_type_arr[1] = i_ascan_ch1_accum_type;
    assign accum_type_arr[2] = i_ascan_ch2_accum_type;
    assign accum_type_arr[3] = i_ascan_ch3_accum_type;

    assign skip_ticks_arr[0] = i_ascan_ch0_skip_ticks;
    assign skip_ticks_arr[1] = i_ascan_ch1_skip_ticks;
    assign skip_ticks_arr[2] = i_ascan_ch2_skip_ticks;
    assign skip_ticks_arr[3] = i_ascan_ch3_skip_ticks;


    // =========================================================================
    // Packetizer FSM & Multiplexer (sys_clk domain)
    // =========================================================================
    localparam STATE_IDLE        = 2'd0;
    localparam STATE_SEND_HEADER = 2'd1;
    localparam STATE_SEND_PAYLOAD= 2'd2;
    localparam STATE_FRAME_DONE  = 2'd3;

    reg [1:0]  state;
    reg [2:0]  header_cnt;
    reg [1:0]  curr_step;
    reg [1:0]  curr_chan;
    reg [15:0] word_cnt;
    reg [15:0] total_size_reg;
    reg        o_data_ready_reg;

    reg [31:0] r_out_data;
    reg        r_out_vld;

    // Check readiness of all active channels in the frame
    wire all_channels_ready = data_ready_arr[0] & data_ready_arr[1] & data_ready_arr[2] & data_ready_arr[3];

    // Combinatorial selection of current block size
    reg [15:0] curr_block_size;
    always @(*) begin
        case (curr_chan)
            2'd0: begin
                case (curr_step)
                    2'd0: curr_block_size = out_size0_arr[0];
                    2'd1: curr_block_size = out_size1_arr[0];
                    2'd2: curr_block_size = out_size2_arr[0];
                    2'd3: curr_block_size = out_size3_arr[0];
                endcase
            end
            2'd1: begin
                case (curr_step)
                    2'd0: curr_block_size = out_size0_arr[1];
                    2'd1: curr_block_size = out_size1_arr[1];
                    2'd2: curr_block_size = out_size2_arr[1];
                    2'd3: curr_block_size = out_size3_arr[1];
                endcase
            end
            2'd2: begin
                case (curr_step)
                    2'd0: curr_block_size = out_size0_arr[2];
                    2'd1: curr_block_size = out_size1_arr[2];
                    2'd2: curr_block_size = out_size2_arr[2];
                    2'd3: curr_block_size = out_size3_arr[2];
                endcase
            end
            2'd3: begin
                case (curr_step)
                    2'd0: curr_block_size = out_size0_arr[3];
                    2'd1: curr_block_size = out_size1_arr[3];
                    2'd2: curr_block_size = out_size2_arr[3];
                    2'd3: curr_block_size = out_size3_arr[3];
                endcase
            end
        endcase
    end

    // FSM Sequential Logic (with purely synchronous active-low reset)
    always @(posedge sys_clk) begin
        if (!sys_rst_n) begin
            state            <= STATE_IDLE;
            header_cnt       <= 3'd0;
            curr_step        <= 2'd0;
            curr_chan        <= 2'd0;
            word_cnt         <= 16'd0;
            total_size_reg   <= 16'd0;
            o_data_ready_reg <= 1'b0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    curr_step <= 2'd0;
                    curr_chan <= 2'd0;
                    word_cnt  <= 16'd0;
                    if (all_channels_ready) begin
                        o_data_ready_reg <= 1'b1;
                        // 8 header words + total payload size from all 4 channels
                        total_size_reg   <= 16'd8 + out_size_arr[0] + out_size_arr[1] + out_size_arr[2] + out_size_arr[3];
                        if (i_out_rdy) begin
                            state      <= STATE_SEND_HEADER;
                            header_cnt <= 3'd0;
                        end
                    end else begin
                        o_data_ready_reg <= 1'b0;
                    end
                end

                STATE_SEND_HEADER: begin
                    if (i_out_rdy) begin
                        if (header_cnt == 3'd7) begin
                            state    <= STATE_SEND_PAYLOAD;
                            word_cnt <= 16'd0;
                        end else begin
                            header_cnt <= header_cnt + 3'd1;
                        end
                    end
                end

                STATE_SEND_PAYLOAD: begin
                    if (curr_block_size == 16'd0) begin
                        // Skip block immediately if empty, advance FSM
                        if (curr_chan == 2'd3) begin
                            curr_chan <= 2'd0;
                            if (curr_step == 2'd3) begin
                                state <= STATE_FRAME_DONE;
                            end else begin
                                curr_step <= curr_step + 2'd1;
                            end
                        end else begin
                            curr_chan <= curr_chan + 2'd1;
                        end
                        word_cnt <= 16'd0;
                    end else if (out_vld_arr[curr_chan] && i_out_rdy) begin
                        if (word_cnt == curr_block_size - 16'd1) begin
                            // Current block complete, advance indices
                            if (curr_chan == 2'd3) begin
                                curr_chan <= 2'd0;
                                if (curr_step == 2'd3) begin
                                    state <= STATE_FRAME_DONE;
                                end else begin
                                    curr_step <= curr_step + 2'd1;
                                end
                            end else begin
                                curr_chan <= curr_chan + 2'd1;
                            end
                            word_cnt <= 16'd0;
                        end else begin
                            word_cnt <= word_cnt + 16'd1;
                        end
                    end
                end

                STATE_FRAME_DONE: begin
                    o_data_ready_reg <= 1'b0;
                    state            <= STATE_IDLE;
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end

    // Direct Read Handshake routing per channel
    assign out_rdy_arr[0] = (state == STATE_SEND_PAYLOAD && curr_chan == 2'd0 && curr_block_size > 0) ? i_out_rdy : 1'b0;
    assign out_rdy_arr[1] = (state == STATE_SEND_PAYLOAD && curr_chan == 2'd1 && curr_block_size > 0) ? i_out_rdy : 1'b0;
    assign out_rdy_arr[2] = (state == STATE_SEND_PAYLOAD && curr_chan == 2'd2 && curr_block_size > 0) ? i_out_rdy : 1'b0;
    assign out_rdy_arr[3] = (state == STATE_SEND_PAYLOAD && curr_chan == 2'd3 && curr_block_size > 0) ? i_out_rdy : 1'b0;

    // Output Data & Valid Multiplexer logic
    always @(*) begin
        case (state)
            STATE_SEND_HEADER: begin
                r_out_vld = 1'b1;
                case (header_cnt)
                    3'd0: r_out_data = {out_size1_arr[0], out_size0_arr[0]};
                    3'd1: r_out_data = {out_size3_arr[0], out_size2_arr[0]};
                    3'd2: r_out_data = {out_size1_arr[1], out_size0_arr[1]};
                    3'd3: r_out_data = {out_size3_arr[1], out_size2_arr[1]};
                    3'd4: r_out_data = {out_size1_arr[2], out_size0_arr[2]};
                    3'd5: r_out_data = {out_size3_arr[2], out_size2_arr[2]};
                    3'd6: r_out_data = {out_size1_arr[3], out_size0_arr[3]};
                    3'd7: r_out_data = {out_size3_arr[3], out_size2_arr[3]};
                    default: r_out_data = 32'd0;
                endcase
            end
            STATE_SEND_PAYLOAD: begin
                if (curr_block_size == 16'd0) begin
                    r_out_vld  = 1'b0;
                    r_out_data = 32'd0;
                end else begin
                    r_out_vld  = out_vld_arr[curr_chan];
                    r_out_data = out_data_arr[curr_chan];
                end
            end
            default: begin
                r_out_vld  = 1'b0;
                r_out_data = 32'd0;
            end
        endcase
    end

    // Drive final output ports
    assign o_out_data   = r_out_data;
    assign o_out_vld    = r_out_vld;
    assign o_out_size   = total_size_reg;
    assign o_data_ready = o_data_ready_reg;


    // =========================================================================
    // Generate Block for Channel Instances
    // =========================================================================
    generate
        genvar i;
        for (i = 0; i < 4; i = i + 1) begin : gen_ascan_channels
            ascan #(
                .ADDR_WIDTH (ADDR_WIDTH)
            ) u_ascan (
                .adc_clk      (adc_clk),
                .adc_rst_n    (adc_rst_n),
                .i_adc_sync   (i_adc_sync),
                .i_sub_sync   (i_sub_sync),
                .i_sub_last   (i_sub_last),
                .o_sub_done   (o_adc_chan_ready[i]),
                .i_adc_data   (adc_data_arr[i]),
                .i_n_samples  (n_samples_arr[i]),
                .i_accum      (accum_arr[i]),
                .i_accum_type (accum_type_arr[i]),
                .i_skip_ticks (skip_ticks_arr[i]),

                .sys_clk      (sys_clk),
                .sys_rst_n    (sys_rst_n),
                .o_out_data   (out_data_arr[i]),
                .o_out_vld    (out_vld_arr[i]),
                .i_out_rdy    (out_rdy_arr[i]),
                .o_out_size0  (out_size0_arr[i]),
                .o_out_size1  (out_size1_arr[i]),
                .o_out_size2  (out_size2_arr[i]),
                .o_out_size3  (out_size3_arr[i]),
                .o_out_size   (out_size_arr[i]),
                .o_data_ready (data_ready_arr[i])
            );
        end
    endgenerate

endmodule

`default_nettype wire