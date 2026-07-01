// =========================================================================
// Global Project Module: ascan_hub (4-Channel Receiver Concentrator)
// Description: Instantiates 4 independent, parallel physical channels of
//              ascan pipelines for handling 4 physical receivers using
//              a generate loop to reduce redundancy and prevent manual typos.
//              Converts individual ready statuses into a 4-bit bus for sequencer.
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
    // Readout Stream Interfaces (sys_clk domain)
    // -------------------------------------------------------------------------
    
    // Physical Channel 0 Readout Port
    output wire [31:0]             o_ch0_out_data,
    output wire                    o_ch0_out_vld,
    input  wire                    i_ch0_out_rdy,
    output wire [15:0]             o_ch0_out_size0,
    output wire [15:0]             o_ch0_out_size1,
    output wire [15:0]             o_ch0_out_size2,
    output wire [15:0]             o_ch0_out_size3,
    output wire [15:0]             o_ch0_out_size,
    output wire                    o_ch0_data_ready,

    // Physical Channel 1 Readout Port
    output wire [31:0]             o_ch1_out_data,
    output wire                    o_ch1_out_vld,
    input  wire                    i_ch1_out_rdy,
    output wire [15:0]             o_ch1_out_size0,
    output wire [15:0]             o_ch1_out_size1,
    output wire [15:0]             o_ch1_out_size2,
    output wire [15:0]             o_ch1_out_size3,
    output wire [15:0]             o_ch1_out_size,
    output wire                    o_ch1_data_ready,

    // Physical Channel 2 Readout Port
    output wire [31:0]             o_ch2_out_data,
    output wire                    o_ch2_out_vld,
    input  wire                    i_ch2_out_rdy,
    output wire [15:0]             o_ch2_out_size0,
    output wire [15:0]             o_ch2_out_size1,
    output wire [15:0]             o_ch2_out_size2,
    output wire [15:0]             o_ch2_out_size3,
    output wire [15:0]             o_ch2_out_size,
    output wire                    o_ch2_data_ready,

    // Physical Channel 3 Readout Port
    output wire [31:0]             o_ch3_out_data,
    output wire                    o_ch3_out_vld,
    input  wire                    i_ch3_out_rdy,
    output wire [15:0]             o_ch3_out_size0,
    output wire [15:0]             o_ch3_out_size1,
    output wire [15:0]             o_ch3_out_size2,
    output wire [15:0]             o_ch3_out_size3,
    output wire [15:0]             o_ch3_out_size,
    output wire                    o_ch3_data_ready
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

    // Readout Stream Interfaces (sys_clk domain)
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

    assign out_rdy_arr[0] = i_ch0_out_rdy;
    assign out_rdy_arr[1] = i_ch1_out_rdy;
    assign out_rdy_arr[2] = i_ch2_out_rdy;
    assign out_rdy_arr[3] = i_ch3_out_rdy;

    // =========================================================================
    // Assigning Arrays to Outputs
    // =========================================================================
    assign o_ch0_out_data = out_data_arr[0];
    assign o_ch1_out_data = out_data_arr[1];
    assign o_ch2_out_data = out_data_arr[2];
    assign o_ch3_out_data = out_data_arr[3];

    assign o_ch0_out_vld = out_vld_arr[0];
    assign o_ch1_out_vld = out_vld_arr[1];
    assign o_ch2_out_vld = out_vld_arr[2];
    assign o_ch3_out_vld = out_vld_arr[3];

    assign o_ch0_out_size0 = out_size0_arr[0];
    assign o_ch1_out_size0 = out_size0_arr[1];
    assign o_ch2_out_size0 = out_size0_arr[2];
    assign o_ch3_out_size0 = out_size0_arr[3];

    assign o_ch0_out_size1 = out_size1_arr[0];
    assign o_ch1_out_size1 = out_size1_arr[1];
    assign o_ch2_out_size1 = out_size1_arr[2];
    assign o_ch3_out_size1 = out_size1_arr[3];

    assign o_ch0_out_size2 = out_size2_arr[0];
    assign o_ch1_out_size2 = out_size2_arr[1];
    assign o_ch2_out_size2 = out_size2_arr[2];
    assign o_ch3_out_size2 = out_size2_arr[3];

    assign o_ch0_out_size3 = out_size3_arr[0];
    assign o_ch1_out_size3 = out_size3_arr[1];
    assign o_ch2_out_size3 = out_size3_arr[2];
    assign o_ch3_out_size3 = out_size3_arr[3];

    assign o_ch0_out_size = out_size_arr[0];
    assign o_ch1_out_size = out_size_arr[1];
    assign o_ch2_out_size = out_size_arr[2];
    assign o_ch3_out_size = out_size_arr[3];

    assign o_ch0_data_ready = data_ready_arr[0];
    assign o_ch1_data_ready = data_ready_arr[1];
    assign o_ch2_data_ready = data_ready_arr[2];
    assign o_ch3_data_ready = data_ready_arr[3];

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