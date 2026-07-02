// =========================================================================
// Global Project Module: dscope_main (Top-Level Controller)
// Description: Integrates centralized reset/sync generation, the central
//              sequencer, parametric configuration register space (param),
//              4-channel parallel capture pipelines (ascan_hub),
//              real-time VRC/TGC curve calculators (tune) with SPI DACs,
//              and a downstream packetizer (data_packet) for prepending
//              system headers.
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
    // Physical DAC SPI Interfaces (dac_clk domain)
    // -------------------------------------------------------------------------
    output wire [3:0]              o_dac_one_sclk,      // SPI SCLK for VGA Stage-1 DACs
    output wire [3:0]              o_dac_one_din,       // SPI DIN for VGA Stage-1 DACs
    output wire [3:0]              o_dac_one_sync_n,    // SPI CS_N for VGA Stage-1 DACs

    output wire [3:0]              o_dac_two_sclk,      // SPI SCLK for VGA Stage-2 DACs
    output wire [3:0]              o_dac_two_din,       // SPI DIN for VGA Stage-2 DACs
    output wire [3:0]              o_dac_two_sync_n,    // SPI CS_N for VGA Stage-2 DACs

    output wire [3:0]              o_dac_offset_sclk,   // SPI SCLK for Log Offset DACs
    output wire [3:0]              o_dac_offset_din,    // SPI DIN for Log Offset DACs
    output wire [3:0]              o_dac_offset_sync_n, // SPI CS_N for Log Offset DACs

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
    output wire [15:0]             o_sys_tune_drop_ticks,
    output wire [31:0]             o_sys_tune_amp_one,
    output wire [31:0]             o_sys_tune_amp_two,
    output wire [15:0]             o_sys_tune_vrc_len,
    output wire [9:0]              o_sys_tune_dac_min,
    output wire [9:0]              o_sys_tune_dac_max,
    output wire [1:0]              o_sys_tune_tune_mode,
    output wire [9:0]              o_sys_tune_log_offset,

    // -------------------------------------------------------------------------
    // Joint Readout Stream Interface (sys_clk domain)
    // -------------------------------------------------------------------------
    input  wire [31:0]             i_sys_flags,    // System flags for packet header
    output wire [31:0]             o_out_data,     // Merged packet stream data (Header + Payload)
    output wire                    o_out_vld,      // Merged packet validity
    input  wire                    i_out_rdy,      // Downstream backpressure readiness
    output wire [15:0]             o_out_size,     // Total packet size in 32-bit words
    output wire                    o_packet_start, // 1-cycle pulse signaling start of frame
    output wire                    o_data_ready,   // High when complete 4-channel butterfly buffer is ready for readout

    // General Sequencer Status
    output wire                    o_seq_busy
);

    // =========================================================================
    // Internal Wires and Busses
    // =========================================================================
    wire [1:0] w_step_index;
    wire [3:0] w_adc_chan_ready;

    // Sub-sync trigger lines per clock domain
    wire       w_adc_sub_sync;
    wire       w_adc_sub_last;
    wire       w_dac_sub_sync;
    wire       w_hi_sub_sync;

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

    // Internal arrays for VGA/VRC parameters (dac_clk domain)
    wire [10:0] w_tune_start_amp   [0:3];
    wire [15:0] w_tune_drop_ticks  [0:3];
    wire [31:0] w_tune_amp_one     [0:3];
    wire [31:0] w_tune_amp_two     [0:3];
    wire [15:0] w_tune_vrc_len     [0:3];
    wire [9:0]  w_tune_dac_min     [0:3];
    wire [9:0]  w_tune_dac_max     [0:3];
    wire [1:0]  w_tune_tune_mode   [0:3];
    wire [9:0]  w_tune_log_offset  [0:3];

    // Real-time calculated DAC codes (dac_clk domain)
    wire [9:0]  w_dac_one_code     [0:3];
    wire [9:0]  w_dac_two_code     [0:3];
    wire [9:0]  w_dac_offset_code  [0:3];
    wire        w_dac_data_vld     [0:3];

    // Hub readout stream connections before formatting
    wire [31:0] w_hub_data;
    wire        w_hub_vld;
    wire        w_hub_rdy;
    wire [15:0] w_hub_size;
    wire        w_hub_ready;

    // Direct assignment of buffer status signal to top boundary
    assign o_data_ready = w_hub_ready;

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
    // 2. Central Sequencer Instantiation (with sub-sync outputs mapped)
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
        .o_dac_sub_sync   (w_dac_sub_sync), // Real-time sub-trigger in dac_clk domain
        .o_hi_sub_sync    (w_hi_sub_sync),  // Real-time sub-trigger in hi_clk domain

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
    //    (Updated to latch execution parameters per-sub-trigger step)
    // =========================================================================
    param u_param (
        .sys_clk               (sys_clk),
        .sys_rst_n             (o_sys_rst_n),
        .i_sys_sync            (o_sys_sync),

        .adc_clk               (adc_clk),
        .adc_rst_n             (o_adc_rst_n),
        .i_adc_sync            (w_adc_sub_sync), // Sync to step sub-sync triggers

        .dac_clk               (dac_clk),
        .dac_rst_n             (o_dac_rst_n),
        .i_dac_sync            (w_dac_sub_sync), // Sync to step sub-sync triggers

        .hi_clk                (hi_clk),
        .hi_rst_n              (o_hi_rst_n),
        .i_hi_sync             (w_hi_sub_sync),  // Sync to step sub-sync triggers

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

        // Outputs for VGA/VRC Tune Controllers (dac_clk domain)
        .o_tune_ch0_start_amp  (w_tune_start_amp[0]),
        .o_tune_ch0_drop_ticks (w_tune_drop_ticks[0]),
        .o_tune_ch0_amp_one    (w_tune_amp_one[0]),
        .o_tune_ch0_amp_two    (w_tune_amp_two[0]),
        .o_tune_ch0_vrc_len    (w_tune_vrc_len[0]),
        .o_tune_ch0_dac_min    (w_tune_dac_min[0]),
        .o_tune_ch0_dac_max    (w_tune_dac_max[0]),
        .o_tune_ch0_tune_mode  (w_tune_tune_mode[0]),
        .o_tune_ch0_log_offset (w_tune_log_offset[0]),

        .o_tune_ch1_start_amp  (w_tune_start_amp[1]),
        .o_tune_ch1_drop_ticks (w_tune_drop_ticks[1]),
        .o_tune_ch1_amp_one    (w_tune_amp_one[1]),
        .o_tune_ch1_amp_two    (w_tune_amp_two[1]),
        .o_tune_ch1_vrc_len    (w_tune_vrc_len[1]),
        .o_tune_ch1_dac_min    (w_tune_dac_min[1]),
        .o_tune_ch1_dac_max    (w_tune_dac_max[1]),
        .o_tune_ch1_tune_mode  (w_tune_tune_mode[1]),
        .o_tune_ch1_log_offset (w_tune_log_offset[1]),

        .o_tune_ch2_start_amp  (w_tune_start_amp[2]),
        .o_tune_ch2_drop_ticks (w_tune_drop_ticks[2]),
        .o_tune_ch2_amp_one    (w_tune_amp_one[2]),
        .o_tune_ch2_amp_two    (w_tune_amp_two[2]),
        .o_tune_ch2_vrc_len    (w_tune_vrc_len[2]),
        .o_tune_ch2_dac_min    (w_tune_dac_min[2]),
        .o_tune_ch2_dac_max    (w_tune_dac_max[2]),
        .o_tune_ch2_tune_mode  (w_tune_tune_mode[2]),
        .o_tune_ch2_log_offset (w_tune_log_offset[2]),

        .o_tune_ch3_start_amp  (w_tune_start_amp[3]),
        .o_tune_ch3_drop_ticks (w_tune_drop_ticks[3]),
        .o_tune_ch3_amp_one    (w_tune_amp_one[3]),
        .o_tune_ch3_amp_two    (w_tune_amp_two[3]),
        .o_tune_ch3_vrc_len    (w_tune_vrc_len[3]),
        .o_tune_ch3_dac_min    (w_tune_dac_min[3]),
        .o_tune_ch3_dac_max    (w_tune_dac_max[3]),
        .o_tune_ch3_tune_mode  (w_tune_tune_mode[3]),
        .o_tune_ch3_log_offset (w_tune_log_offset[3]),

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
        .o_sys_tune_drop_ticks (o_sys_tune_drop_ticks),
        .o_sys_tune_amp_one    (o_sys_tune_amp_one),
        .o_sys_tune_amp_two    (o_sys_tune_amp_two),
        .o_sys_tune_vrc_len    (o_sys_tune_vrc_len),
        .o_sys_tune_dac_min    (o_sys_tune_dac_min),
        .o_sys_tune_dac_max    (o_sys_tune_dac_max),
        .o_sys_tune_tune_mode  (o_sys_tune_tune_mode),
        .o_sys_tune_log_offset (o_sys_tune_log_offset)
    );

    // =========================================================================
    // 4. 4-Channel Receiver Concentrator Instantiation (ascan_hub)
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

        // Internal Output Streaming Interface (Routes into data_packet)
        .o_out_data          (w_hub_data),
        .o_out_vld           (w_hub_vld),
        .i_out_rdy           (w_hub_rdy),
        .o_out_size          (w_hub_size),
        .o_data_ready        (w_hub_ready)
    );

    // =========================================================================
    // 5. 4-Channel VGA/TGC Curve Generators and SPI DAC Control Hub
    // =========================================================================
    genvar ch;
    generate
        for (ch = 0; ch < 4; ch = ch + 1) begin : gen_tgc_channels
            
            // 5.1 Real-Time Time-Gain-Compensation (TGC/VRC) sweep calculator
            //     (Triggered on every sub-sync step in dac_clk domain)
            tune #(
                .GAIN_FRAC_WIDTH (16)
            ) u_tune (
                .dac_clk        (dac_clk),
                .dac_rst_n      (o_dac_rst_n),
                .i_dac_sync     (w_dac_sub_sync), // Triggered per-sub-trigger
                
                .i_start_amp    (w_tune_start_amp[ch]),
                .i_drop_ticks   (w_tune_drop_ticks[ch]),
                .i_amp_one      (w_tune_amp_one[ch]),
                .i_amp_two      (w_tune_amp_two[ch]),
                .i_vrc_len      (w_tune_vrc_len[ch]),
                .i_dac_min      (w_tune_dac_min[ch]),
                .i_dac_max      (w_tune_dac_max[ch]),
                .i_tune_mode    (w_tune_tune_mode[ch]),
                .i_log_offset   (w_tune_log_offset[ch]),

                .o_dac_one      (w_dac_one_code[ch]),
                .o_dac_two      (w_dac_two_code[ch]),
                .o_dac_offset   (w_dac_offset_code[ch]),
                .o_dac_data_vld (w_dac_data_vld[ch])
            );

            // 5.2 Dynamic update throttling state registers
            reg [9:0] r_prev_dac_one;
            reg       r_dac_one_wr;
            wire      w_dac_one_busy;

            reg [9:0] r_prev_dac_two;
            reg       r_dac_two_wr;
            wire      w_dac_two_busy;

            reg [9:0] r_prev_dac_offset;
            reg       r_dac_offset_wr;
            wire      w_dac_offset_busy;

            always @(posedge dac_clk) begin
                if (!o_dac_rst_n) begin
                    r_prev_dac_one    <= 10'd0;
                    r_dac_one_wr      <= 1'b0;
                    r_prev_dac_two    <= 10'd0;
                    r_dac_two_wr      <= 1'b0;
                    r_prev_dac_offset <= 10'd0;
                    r_dac_offset_wr   <= 1'b0;
                end else begin
                    // Throttled VGA Stage 1 DAC Transmitter
                    if (w_dac_data_vld[ch] && (w_dac_one_code[ch] != r_prev_dac_one) && !w_dac_one_busy && !r_dac_one_wr) begin
                        r_dac_one_wr   <= 1'b1;
                        r_prev_dac_one <= w_dac_one_code[ch];
                    end else begin
                        r_dac_one_wr   <= 1'b0;
                    end

                    // Throttled VGA Stage 2 DAC Transmitter
                    if (w_dac_data_vld[ch] && (w_dac_two_code[ch] != r_prev_dac_two) && !w_dac_two_busy && !r_dac_two_wr) begin
                        r_dac_two_wr   <= 1'b1;
                        r_prev_dac_two <= w_dac_two_code[ch];
                    end else begin
                        r_dac_two_wr   <= 1'b0;
                    end

                    // Throttled Log Offset Calibration DAC Transmitter
                    if (w_dac_data_vld[ch] && (w_dac_offset_code[ch] != r_prev_dac_offset) && !w_dac_offset_busy && !r_dac_offset_wr) begin
                        r_dac_offset_wr   <= 1'b1;
                        r_prev_dac_offset <= w_dac_offset_code[ch];
                    end else begin
                        r_dac_offset_wr   <= 1'b0;
                    end
                end
            end

            // 5.3 Physical TI DAC101S101 SPI Transmitters
            spi_dac101s101 u_spi_dac_one (
                .dac_clk    (dac_clk),
                .dac_rst_n  (o_dac_rst_n),
                .i_dac_data (r_prev_dac_one),
                .i_dac_wr   (r_dac_one_wr),
                .o_dac_busy (w_dac_one_busy),
                .o_sclk     (o_dac_one_sclk[ch]),
                .o_din      (o_dac_one_din[ch]),
                .o_sync_n   (o_dac_one_sync_n[ch])
            );

            spi_dac101s101 u_spi_dac_two (
                .dac_clk    (dac_clk),
                .dac_rst_n  (o_dac_rst_n),
                .i_dac_data (r_prev_dac_two),
                .i_dac_wr   (r_dac_two_wr),
                .o_dac_busy (w_dac_two_busy),
                .o_sclk     (o_dac_two_sclk[ch]),
                .o_din      (o_dac_two_din[ch]),
                .o_sync_n   (o_dac_two_sync_n[ch])
            );

            spi_dac101s101 u_spi_dac_offset (
                .dac_clk    (dac_clk),
                .dac_rst_n  (o_dac_rst_n),
                .i_dac_data (r_prev_dac_offset),
                .i_dac_wr   (r_dac_offset_wr),
                .o_dac_busy (w_dac_offset_busy),
                .o_sclk     (o_dac_offset_sclk[ch]),
                .o_din      (o_dac_offset_din[ch]),
                .o_sync_n   (o_dac_offset_sync_n[ch])
            );
        end
    endgenerate

    // =========================================================================
    // 6. Data Packetizer (Prepends System Header to Frame Stream)
    // =========================================================================
    data_packet u_data_packet (
        .sys_clk        (sys_clk),
        .sys_rst_n      (o_sys_rst_n),
        
        // System status flags
        .i_sys_flags    (i_sys_flags),
        
        // Stream interface from ascan_hub
        .i_hub_data     (w_hub_data),
        .i_hub_vld      (w_hub_vld),
        .o_hub_rdy      (w_hub_rdy),
        .i_hub_size     (w_hub_size),
        .i_hub_ready    (w_hub_ready),
        
        // Output packet stream (Connected directly to top-level outputs)
        .o_packet_data  (o_out_data),
        .o_packet_vld   (o_out_vld),
        .i_packet_rdy   (i_out_rdy),
        .o_packet_len   (o_out_size),
        .o_packet_start (o_packet_start)
    );

endmodule

`default_nettype wire