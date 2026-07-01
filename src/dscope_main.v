// =========================================================================
// Global Project Module: dscope_main (Top-Level Controller)
// Description: Integrates centralized reset/sync generation, the central
//              sequencer, parametric configuration register space (param),
//              and 4-channel parallel capture pipelines (ascan_hub).
// =========================================================================

`default_nettype none

module dscope_main #(
    parameter ASCAN_ADDR_WIDTH = 15 // Depth of each ping-pong buffer = 32768 words (128 KB)
)(
    // Clock inputs (No i_ prefix as per naming conventions)
    input  wire                    sys_clk,   // 80 MHz (System Control & Readout)
    input  wire                    adc_clk,   // 65 MHz (ADC Capture & Packing)
    input  wire                    log_clk,   // 25 MHz (Logarithmic Receiver Capture)
    input  wire                    dac_clk,   // 50 MHz (DAC Gain Control/VRC)
    input  wire                    hi_clk,    // 250 MHz (High-speed Pulser)

    // External control signals
    input  wire                    rst_n,        // Global active-low asynchronous reset
    input  wire                    i_sys_sync,   // Master trigger input (sys_clk domain)

    // Synchronized Reset Outputs (Held for >= 8 cycles of their respective domain)
    output wire                    o_sys_rst_n,  // Synchronized to sys_clk
    output wire                    o_adc_rst_n,  // Synchronized to adc_clk
    output wire                    o_log_rst_n,  // Synchronized to log_clk
    output wire                    o_dac_rst_n,  // Synchronized to dac_clk
    output wire                    o_hi_rst_n,   // Synchronized to hi_clk

    // Synchronous Trigger Pulse Outputs (Exactly 1-cycle duration)
    output wire                    o_sys_sync,   // Synchronous to sys_clk
    output wire                    o_adc_sync,   // Synchronous to adc_clk
    output wire                    o_log_sync,   // Synchronous to log_clk
    output wire                    o_dac_sync,   // Synchronous to dac_clk
    output wire                    o_hi_sync,    // Synchronous to hi_clk

    // -------------------------------------------------------------------------
    // Command & Sequencer Configuration Interfaces (sys_clk domain)
    // -------------------------------------------------------------------------
    input  wire [31:0]             i_cmd_addr,   // Write command address
    input  wire [31:0]             i_cmd_data,   // Write command data
    input  wire                    i_cmd_vld,    // Write command valid strobe
    input  wire [1:0]              i_seq_count,  // Active scan steps (00: 1 step, ..., 11: 4 steps)

    // -------------------------------------------------------------------------
    // Physical ADC Inputs (adc_clk domain)
    // -------------------------------------------------------------------------
    input  wire signed [11:0]      i_adc_data_ch0,
    input  wire signed [11:0]      i_adc_data_ch1,
    input  wire signed [11:0]      i_adc_data_ch2,
    input  wire signed [11:0]      i_adc_data_ch3,

    // -------------------------------------------------------------------------
    // Physical Transmitter (Pulser) Outputs (hi_clk & sys_clk domains)
    // -------------------------------------------------------------------------
    output wire [15:0]             o_pulse_ch0_charge,
    output wire [15:0]             o_pulse_ch0_transfer,
    output wire [7:0]              o_pulse_ch0_strike,
    output wire [3:0]              o_pulse_ch0_gen_mask,

    output wire [15:0]             o_pulse_ch1_charge,
    output wire [15:0]             o_pulse_ch1_transfer,
    output wire [7:0]              o_pulse_ch1_strike,
    output wire [3:0]              o_pulse_ch1_gen_mask,

    output wire [15:0]             o_pulse_ch2_charge,
    output wire [15:0]             o_pulse_ch2_transfer,
    output wire [7:0]              o_pulse_ch2_strike,
    output wire [3:0]              o_pulse_ch2_gen_mask,

    output wire [15:0]             o_pulse_ch3_charge,
    output wire [15:0]             o_pulse_ch3_transfer,
    output wire [7:0]              o_pulse_ch3_strike,
    output wire [3:0]              o_pulse_ch3_gen_mask,

    // -------------------------------------------------------------------------
    // Physical Analog Gain Tuning (VRC/VGA) Outputs (sys_clk domain)
    // -------------------------------------------------------------------------
    output wire [10:0]             o_tune_ch0_start_amp,
    output wire [31:0]             o_tune_ch0_amp_one,
    output wire [31:0]             o_tune_ch0_amp_two,
    output wire [15:0]             o_tune_ch0_vrc_len,
    output wire [9:0]              o_tune_ch0_dac_min,
    output wire [9:0]              o_tune_ch0_dac_max,
    output wire [1:0]              o_tune_ch0_tune_mode,
    output wire [9:0]              o_tune_ch0_log_offset,

    output wire [10:0]             o_tune_ch1_start_amp,
    output wire [31:0]             o_tune_ch1_amp_one,
    output wire [31:0]             o_tune_ch1_amp_two,
    output wire [15:0]             o_tune_ch1_vrc_len,
    output wire [9:0]              o_tune_ch1_dac_min,
    output wire [9:0]              o_tune_ch1_dac_max,
    output wire [1:0]              o_tune_ch1_tune_mode,
    output wire [9:0]              o_tune_ch1_log_offset,

    output wire [10:0]             o_tune_ch2_start_amp,
    output wire [31:0]             o_tune_ch2_amp_one,
    output wire [31:0]             o_tune_ch2_amp_two,
    output wire [15:0]             o_tune_ch2_vrc_len,
    output wire [9:0]              o_tune_ch2_dac_min,
    output wire [9:0]              o_tune_ch2_dac_max,
    output wire [1:0]              o_tune_ch2_tune_mode,
    output wire [9:0]              o_tune_ch2_log_offset,

    output wire [10:0]             o_tune_ch3_start_amp,
    output wire [31:0]             o_tune_ch3_amp_one,
    output wire [31:0]             o_tune_ch3_amp_two,
    output wire [15:0]             o_tune_ch3_vrc_len,
    output wire [9:0]              o_tune_ch3_dac_min,
    output wire [9:0]              o_tune_ch3_dac_max,
    output wire [1:0]              o_tune_ch3_tune_mode,
    output wire [9:0]              o_tune_ch3_log_offset,

    // -------------------------------------------------------------------------
    // Physical PEP (Transducer Multiplexer) Controls (sys_clk domain)
    // -------------------------------------------------------------------------
    output wire [1:0]              o_ascan_ch0_pep_idx,
    output wire [1:0]              o_ascan_ch1_pep_idx,
    output wire [1:0]              o_ascan_ch2_pep_idx,
    output wire [1:0]              o_ascan_ch3_pep_idx,

    // -------------------------------------------------------------------------
    // Metadata Read Interface for Packetizer (sys_clk domain)
    // -------------------------------------------------------------------------
    input  wire [1:0]              i_packet_phy_ch,
    input  wire [1:0]              i_packet_vch,

    output wire [15:0]             o_sys_ascan_n_samples,
    output wire [7:0]              o_sys_ascan_accum,
    output wire [1:0]              o_sys_ascan_accum_type,
    output wire [15:0]             o_sys_ascan_drop_ticks,
    output wire [1:0]              o_sys_ascan_pep_idx,

    output wire [15:0]             o_sys_pulse_charge,
    output wire [15:0]             o_sys_pulse_transfer,
    output wire [7:0]              o_sys_pulse_strike,
    output wire [3:0]              o_sys_pulse_gen_mask,

    output wire [10:0]             o_sys_tune_start_amp,
    output wire [31:0]             o_sys_tune_amp_one,
    output wire [31:0]             o_sys_tune_amp_two,
    output wire [15:0]             o_sys_tune_vrc_len,
    output wire [9:0]              o_sys_tune_dac_min,
    output wire [9:0]              o_sys_tune_dac_max,
    output wire [1:0]              o_sys_tune_tune_mode,
    output wire [9:0]              o_sys_tune_log_offset,

    // -------------------------------------------------------------------------
    // Parallel Readout Stream Interfaces (sys_clk domain)
    // -------------------------------------------------------------------------
    
    // Channel 0 Readout Stream Port
    output wire [31:0]             o_ch0_out_data,
    output wire                    o_ch0_out_vld,
    input  wire                    i_ch0_out_rdy,
    output wire [15:0]             o_ch0_out_size0,
    output wire [15:0]             o_ch0_out_size1,
    output wire [15:0]             o_ch0_out_size2,
    output wire [15:0]             o_ch0_out_size3,
    output wire [15:0]             o_ch0_out_size,
    output wire                    o_ch0_data_ready,

    // Channel 1 Readout Stream Port
    output wire [31:0]             o_ch1_out_data,
    output wire                    o_ch1_out_vld,
    input  wire                    i_ch1_out_rdy,
    output wire [15:0]             o_ch1_out_size0,
    output wire [15:0]             o_ch1_out_size1,
    output wire [15:0]             o_ch1_out_size2,
    output wire [15:0]             o_ch1_out_size3,
    output wire [15:0]             o_ch1_out_size,
    output wire                    o_ch1_data_ready,

    // Channel 2 Readout Stream Port
    output wire [31:0]             o_ch2_out_data,
    output wire                    o_ch2_out_vld,
    input  wire                    i_ch2_out_rdy,
    output wire [15:0]             o_ch2_out_size0,
    output wire [15:0]             o_ch2_out_size1,
    output wire [15:0]             o_ch2_out_size2,
    output wire [15:0]             o_ch2_out_size3,
    output wire [15:0]             o_ch2_out_size,
    output wire                    o_ch2_data_ready,

    // Channel 3 Readout Stream Port
    output wire [31:0]             o_ch3_out_data,
    output wire                    o_ch3_out_vld,
    input  wire                    i_ch3_out_rdy,
    output wire [15:0]             o_ch3_out_size0,
    output wire [15:0]             o_ch3_out_size1,
    output wire [15:0]             o_ch3_out_size2,
    output wire [15:0]             o_ch3_out_size3,
    output wire [15:0]             o_ch3_out_size,
    output wire                    o_ch3_data_ready,

    // General Sequencer Status
    output wire                    o_seq_busy
);

    // =========================================================================
    // Internal Wires and Busses
    // =========================================================================
    wire [1:0] w_step_index;
    wire [3:0] w_adc_chan_ready;

    wire       w_adc_sub_sync;
    wire       w_adc_sub_last;

    // Ascan parameters routed from param to ascan_hub (captured on i_adc_sync)
    wire [15:0] w_ascan_ch0_n_samples;
    wire [7:0]  w_ascan_ch0_accum;
    wire [1:0]  w_ascan_ch0_accum_type;
    wire [15:0] w_ascan_ch0_drop_ticks;

    wire [15:0] w_ascan_ch1_n_samples;
    wire [7:0]  w_ascan_ch1_accum;
    wire [1:0]  w_ascan_ch1_accum_type;
    wire [15:0] w_ascan_ch1_drop_ticks;

    wire [15:0] w_ascan_ch2_n_samples;
    wire [7:0]  w_ascan_ch2_accum;
    wire [1:0]  w_ascan_ch2_accum_type;
    wire [15:0] w_ascan_ch2_drop_ticks;

    wire [15:0] w_ascan_ch3_n_samples;
    wire [7:0]  w_ascan_ch3_accum;
    wire [1:0]  w_ascan_ch3_accum_type;
    wire [15:0] w_ascan_ch3_drop_ticks;

    // =========================================================================
    // 1. Central Reset and Sync Generator Instantiation (rst_sync)
    // =========================================================================
    rst_sync u_rst_sync (
        .sys_clk      (sys_clk),
        .adc_clk      (adc_clk),
        .log_clk      (log_clk),
        .dac_clk      (dac_clk),
        .hi_clk       (hi_clk),
        .rst_n        (rst_n),
        .i_sys_sync   (i_sys_sync),
        
        .o_sys_rst_n  (o_sys_rst_n),
        .o_adc_rst_n  (o_adc_rst_n),
        .o_log_rst_n  (o_log_rst_n),
        .o_dac_rst_n  (o_dac_rst_n),
        .o_hi_rst_n   (o_hi_rst_n),
        
        .o_sys_sync   (o_sys_sync),
        .o_adc_sync   (o_adc_sync),
        .o_log_sync   (o_log_sync),
        .o_dac_sync   (o_dac_sync),
        .o_hi_sync    (o_hi_sync)
    );

    // =========================================================================
    // 2. Central Sequencer Instantiation
    // =========================================================================
    sequencer u_sequencer (
        .sys_clk          (sys_clk),
        .sys_rst_n        (o_sys_rst_n),
        .adc_clk          (adc_clk),
        .adc_rst_n        (o_adc_rst_n),
        .log_clk          (log_clk),
        .log_rst_n        (o_log_rst_n),
        .dac_clk          (dac_clk),
        .dac_rst_n        (o_dac_rst_n),
        .hi_clk           (hi_clk),
        .hi_rst_n         (o_hi_rst_n),

        .i_sys_sync       (o_sys_sync),
        .i_seq_count      (i_seq_count),
        .i_adc_chan_ready (w_adc_chan_ready),

        .o_sys_sub_sync   (),
        .o_adc_sub_sync   (w_adc_sub_sync),
        .o_log_sub_sync   (),
        .o_dac_sub_sync   (),
        .o_hi_sub_sync    (),

        .o_sys_sub_last   (),
        .o_adc_sub_last   (w_adc_sub_last),
        .o_log_sub_last   (),
        .o_dac_sub_last   (),
        .o_hi_sub_last    (),

        .o_step_index     (w_step_index),
        .o_seq_busy       (o_seq_busy)
    );

    // =========================================================================
    // 3. Central Parameter Register Manager Instantiation
    // =========================================================================
    param u_param (
        .sys_clk               (sys_clk),
        .sys_rst_n             (o_sys_rst_n),
        .i_sys_sync            (o_sys_sync),

        .adc_clk               (adc_clk),
        .adc_rst_n             (o_adc_rst_n),
        .i_adc_sync            (o_adc_sync),

        .hi_clk                (hi_clk),
        .hi_rst_n              (o_hi_rst_n),
        .i_hi_sync             (o_hi_sync),

        .i_sys_vch_sel         (w_step_index),

        .i_cmd_addr            (i_cmd_addr),
        .i_cmd_data            (i_cmd_data),
        .i_cmd_vld             (i_cmd_vld),

        // Outputs to Physical Ascan capture pipelines
        .o_ascan_ch0_n_samples (w_ascan_ch0_n_samples),
        .o_ascan_ch0_accum     (w_ascan_ch0_accum),
        .o_ascan_ch0_accum_type(w_ascan_ch0_accum_type),
        .o_ascan_ch0_drop_ticks(w_ascan_ch0_drop_ticks),

        .o_ascan_ch1_n_samples (w_ascan_ch1_n_samples),
        .o_ascan_ch1_accum     (w_ascan_ch1_accum),
        .o_ascan_ch1_accum_type(w_ascan_ch1_accum_type),
        .o_ascan_ch1_drop_ticks(w_ascan_ch1_drop_ticks),

        .o_ascan_ch2_n_samples (w_ascan_ch2_n_samples),
        .o_ascan_ch2_accum     (w_ascan_ch2_accum),
        .o_ascan_ch2_accum_type(w_ascan_ch2_accum_type),
        .o_ascan_ch2_drop_ticks(w_ascan_ch2_drop_ticks),

        .o_ascan_ch3_n_samples (w_ascan_ch3_n_samples),
        .o_ascan_ch3_accum     (w_ascan_ch3_accum),
        .o_ascan_ch3_accum_type(w_ascan_ch3_accum_type),
        .o_ascan_ch3_drop_ticks(w_ascan_ch3_drop_ticks),

        // Outputs for PEP analog multiplexers
        .o_ascan_ch0_pep_idx   (o_ascan_ch0_pep_idx),
        .o_ascan_ch1_pep_idx   (o_ascan_ch1_pep_idx),
        .o_ascan_ch2_pep_idx   (o_ascan_ch2_pep_idx),
        .o_ascan_ch3_pep_idx   (o_ascan_ch3_pep_idx),

        // Outputs for Pulser parameters (hi_clk domain)
        .o_pulse_ch0_charge    (o_pulse_ch0_charge),
        .o_pulse_ch0_transfer  (o_pulse_ch0_transfer),
        .o_pulse_ch0_strike    (o_pulse_ch0_strike),

        .o_pulse_ch1_charge    (o_pulse_ch1_charge),
        .o_pulse_ch1_transfer  (o_pulse_ch1_transfer),
        .o_pulse_ch1_strike    (o_pulse_ch1_strike),

        .o_pulse_ch2_charge    (o_pulse_ch2_charge),
        .o_pulse_ch2_transfer  (o_pulse_ch2_transfer),
        .o_pulse_ch2_strike    (o_pulse_ch2_strike),

        .o_pulse_ch3_charge    (o_pulse_ch3_charge),
        .o_pulse_ch3_transfer  (o_pulse_ch3_transfer),
        .o_pulse_ch3_strike    (o_pulse_ch3_strike),

        // Outputs for Pulser generator masks (sys_clk domain)
        .o_pulse_ch0_gen_mask  (o_pulse_ch0_gen_mask),
        .o_pulse_ch1_gen_mask  (o_pulse_ch1_gen_mask),
        .o_pulse_ch2_gen_mask  (o_pulse_ch2_gen_mask),
        .o_pulse_ch3_gen_mask  (o_pulse_ch3_gen_mask),

        // Outputs for VGA/VRC Tune Controllers (sys_clk domain)
        .o_tune_ch0_start_amp  (o_tune_ch0_start_amp),
        .o_tune_ch0_amp_one    (o_tune_ch0_amp_one),
        .o_tune_ch0_amp_two    (o_tune_ch0_amp_two),
        .o_tune_ch0_vrc_len    (o_tune_ch0_vrc_len),
        .o_tune_ch0_dac_min    (o_tune_ch0_dac_min),
        .o_tune_ch0_dac_max    (o_tune_ch0_dac_max),
        .o_tune_ch0_tune_mode  (o_tune_ch0_tune_mode),
        .o_tune_ch0_log_offset (o_tune_ch0_log_offset),

        .o_tune_ch1_start_amp  (o_tune_ch1_start_amp),
        .o_tune_ch1_amp_one    (o_tune_ch1_amp_one),
        .o_tune_ch1_amp_two    (o_tune_ch1_amp_two),
        .o_tune_ch1_vrc_len    (o_tune_ch1_vrc_len),
        .o_tune_ch1_dac_min    (o_tune_ch1_dac_min),
        .o_tune_ch1_dac_max    (o_tune_ch1_dac_max),
        .o_tune_ch1_tune_mode  (o_tune_ch1_tune_mode),
        .o_tune_ch1_log_offset (o_tune_ch1_log_offset),

        .o_tune_ch2_start_amp  (o_tune_ch2_start_amp),
        .o_tune_ch2_amp_one    (o_tune_ch2_amp_one),
        .o_tune_ch2_amp_two    (o_tune_ch2_amp_two),
        .o_tune_ch2_vrc_len    (o_tune_ch2_vrc_len),
        .o_tune_ch2_dac_min    (o_tune_ch2_dac_min),
        .o_tune_ch2_dac_max    (o_tune_ch2_dac_max),
        .o_tune_ch2_tune_mode  (o_tune_ch2_tune_mode),
        .o_tune_ch2_log_offset (o_tune_ch2_log_offset),

        .o_tune_ch3_start_amp  (o_tune_ch3_start_amp),
        .o_tune_ch3_amp_one    (o_tune_ch3_amp_one),
        .o_tune_ch3_amp_two    (o_tune_ch3_amp_two),
        .o_tune_ch3_vrc_len    (o_tune_ch3_vrc_len),
        .o_tune_ch3_dac_min    (o_tune_ch3_dac_min),
        .o_tune_ch3_dac_max    (o_tune_ch3_dac_max),
        .o_tune_ch3_tune_mode  (o_tune_ch3_tune_mode),
        .o_tune_ch3_log_offset (o_tune_ch3_log_offset),

        // Two-coordinate Metadata Readout Interface for Packetizer
        .i_packet_phy_ch       (i_packet_phy_ch),
        .i_packet_vch          (i_packet_vch),

        .o_sys_ascan_n_samples (o_sys_ascan_n_samples),
        .o_sys_ascan_accum     (o_sys_ascan_accum),
        .o_sys_ascan_accum_type(o_sys_ascan_accum_type),
        .o_sys_ascan_drop_ticks(o_sys_ascan_drop_ticks),
        .o_sys_ascan_pep_idx   (o_sys_ascan_pep_idx),

        .o_sys_pulse_charge    (o_sys_pulse_charge),
        .o_sys_pulse_transfer  (o_sys_pulse_transfer),
        .o_sys_pulse_strike    (o_sys_pulse_strike),
        .o_sys_pulse_gen_mask  (o_sys_pulse_gen_mask),

        .o_sys_tune_start_amp  (o_sys_tune_start_amp),
        .o_sys_tune_amp_one    (o_sys_tune_amp_one),
        .o_sys_tune_amp_two    (o_sys_tune_amp_two),
        .o_sys_tune_vrc_len    (o_sys_tune_vrc_len),
        .o_sys_tune_dac_min    (o_sys_tune_dac_min),
        .o_sys_tune_dac_max    (o_sys_tune_dac_max),
        .o_sys_tune_tune_mode  (o_sys_tune_tune_mode),
        .o_sys_tune_log_offset (o_sys_tune_log_offset)
    );

    // =========================================================================
    // 3. 4-Channel Receiver Concentrator Instantiation (ascan_hub)
    // =========================================================================
    ascan_hub #(
        .ADDR_WIDTH          (ASCAN_ADDR_WIDTH)
    ) u_ascan_hub (
        // Clocks and synchronous resets
        .adc_clk             (adc_clk),
        .adc_rst_n           (o_adc_rst_n),
        .sys_clk             (sys_clk),
        .sys_rst_n           (o_sys_rst_n),

        // Controls from central sync and sequencer (adc_clk domain)
        .i_adc_sync          (o_adc_sync),
        .i_sub_sync          (w_adc_sub_sync),
        .i_sub_last          (w_adc_sub_last),
        .o_adc_chan_ready    (w_adc_chan_ready),

        // Physical ADC-1 Input Feeds
        .i_adc_data_ch0      (i_adc_data_ch0),
        .i_adc_data_ch1      (i_adc_data_ch1),
        .i_adc_data_ch2      (i_adc_data_ch2),
        .i_adc_data_ch3      (i_adc_data_ch3),

        // Active parameters mapped to channels
        .i_ascan_ch0_n_samples  (w_ascan_ch0_n_samples),
        .i_ascan_ch0_accum      (w_ascan_ch0_accum),
        .i_ascan_ch0_accum_type (w_ascan_ch0_accum_type),
        .i_ascan_ch0_skip_ticks (w_ascan_ch0_drop_ticks),

        .i_ascan_ch1_n_samples  (w_ascan_ch1_n_samples),
        .i_ascan_ch1_accum      (w_ascan_ch1_accum),
        .i_ascan_ch1_accum_type (w_ascan_ch1_accum_type),
        .i_ascan_ch1_skip_ticks (w_ascan_ch1_drop_ticks),

        .i_ascan_ch2_n_samples  (w_ascan_ch2_n_samples),
        .i_ascan_ch2_accum      (w_ascan_ch2_accum),
        .i_ascan_ch2_accum_type (w_ascan_ch2_accum_type),
        .i_ascan_ch2_skip_ticks (w_ascan_ch2_drop_ticks),

        .i_ascan_ch3_n_samples  (w_ascan_ch3_n_samples),
        .i_ascan_ch3_accum      (w_ascan_ch3_accum),
        .i_ascan_ch3_accum_type (w_ascan_ch3_accum_type),
        .i_ascan_ch3_skip_ticks (w_ascan_ch3_drop_ticks),

        // Readout Streaming Busses
        .o_ch0_out_data      (o_ch0_out_data),
        .o_ch0_out_vld       (o_ch0_out_vld),
        .i_ch0_out_rdy       (i_ch0_out_rdy),
        .o_ch0_out_size0     (o_ch0_out_size0),
        .o_ch0_out_size1     (o_ch0_out_size1),
        .o_ch0_out_size2     (o_ch0_out_size2),
        .o_ch0_out_size3     (o_ch0_out_size3),
        .o_ch0_out_size      (o_ch0_out_size),
        .o_ch0_data_ready    (o_ch0_data_ready),

        .o_ch1_out_data      (o_ch1_out_data),
        .o_ch1_out_vld       (o_ch1_out_vld),
        .i_ch1_out_rdy       (i_ch1_out_rdy),
        .o_ch1_out_size0     (o_ch1_out_size0),
        .o_ch1_out_size1     (o_ch1_out_size1),
        .o_ch1_out_size2     (o_ch1_out_size2),
        .o_ch1_out_size3     (o_ch1_out_size3),
        .o_ch1_out_size      (o_ch1_out_size),
        .o_ch1_data_ready    (o_ch1_data_ready),

        .o_ch2_out_data      (o_ch2_out_data),
        .o_ch2_out_vld       (o_ch2_out_vld),
        .i_ch2_out_rdy       (i_ch2_out_rdy),
        .o_ch2_out_size0     (o_ch2_out_size0),
        .o_ch2_out_size1     (o_ch2_out_size1),
        .o_ch2_out_size2     (o_ch2_out_size2),
        .o_ch2_out_size3     (o_ch2_out_size3),
        .o_ch2_out_size      (o_ch2_out_size),
        .o_ch2_data_ready    (o_ch2_data_ready),

        .o_ch3_out_data      (o_ch3_out_data),
        .o_ch3_out_vld       (o_ch3_out_vld),
        .i_ch3_out_rdy       (i_ch3_out_rdy),
        .o_ch3_out_size0     (o_ch3_out_size0),
        .o_ch3_out_size1     (o_ch3_out_size1),
        .o_ch3_out_size2     (o_ch3_out_size2),
        .o_ch3_out_size3     (o_ch3_out_size3),
        .o_ch3_out_size      (o_ch3_out_size),
        .o_ch3_data_ready    (o_ch3_data_ready)
    );

endmodule

`default_nettype wire