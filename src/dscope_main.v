// =========================================================================
// Global Project Module: dscope_main (Top-Level Controller)
// Description: Integrates centralized reset/sync generation and 
//              the A-scan capture & buffering pipelines.
// =========================================================================

`default_nettype none

module dscope_main #(
    parameter ASCAN_ADDR_WIDTH = 15 // Depth of each ping-pong buffer = 2^15 words
)(
    // Clock inputs (No i_ prefix as per naming conventions)
    input  wire                    sys_clk,   // 80 MHz
    input  wire                    adc_clk,   // 65 MHz
    input  wire                    log_clk,   // 25 MHz
    input  wire                    dac_clk,   // 50 MHz
    input  wire                    hi_clk,    // 250 MHz

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

    // ADC-1 Input Interface (adc_clk domain)
    input  wire signed [11:0]      i_adc_data,

    // A-Scan Configuration Parameters (captured on o_adc_sync trigger)
    input  wire [15:0]             i_n_samples,  // Total number of samples to capture
    input  wire [7:0]              i_accum,      // Accumulation length (0 or 1 means no accumulation)
    input  wire [1:0]              i_accum_type, // 00: Peak, 01: Average, 10: Decimation
    input  wire [15:0]             i_skip_ticks, // Delay in ticks before starting registration

    // Streaming Output Interface (sys_clk domain)
    output wire [31:0]             o_out_data,   // Streamed data words
    output wire                    o_out_vld,    // Stream data validity
    input  wire                    i_out_rdy,    // Backpressure readiness from receiver
    output wire [15:0]             o_out_size,   // Size of the packet ready for readout
    output wire                    o_data_ready  // Indicator that a complete buffer is ready for readout
);

    // =========================================================================
    // 1. Central Reset and Sync Generator Instantiation
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
    // 2. A-Scan Capture and Buffer Module Instantiation
    // =========================================================================
    ascan #(
        .ADDR_WIDTH   (ASCAN_ADDR_WIDTH)
    ) u_ascan (
        // ADC Clock Domain Connections
        .adc_clk      (adc_clk),
        .adc_rst_n    (o_adc_rst_n),
        .i_adc_sync   (o_adc_sync),
        .i_adc_data   (i_adc_data),

        // Latching Configuration Parameters
        .i_n_samples  (i_n_samples),
        .i_accum      (i_accum),
        .i_accum_type (i_accum_type),
        .i_skip_ticks (i_skip_ticks),

        // System Clock Domain Connections
        .sys_clk      (sys_clk),
        .sys_rst_n    (o_sys_rst_n),

        // Output Stream Bus (Direct Mapping to Top-Level Outputs)
        .o_out_data   (o_out_data),
        .o_out_vld    (o_out_vld),
        .i_out_rdy    (i_out_rdy),
        .o_out_size   (o_out_size),
        .o_data_ready (o_data_ready)
    );

endmodule

`default_nettype wire