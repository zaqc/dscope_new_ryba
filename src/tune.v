// =========================================================================
// Ultrasound Defectoscope - TGC / VRC (Tune) Module
// File: src/tune.v
// Description: Implements Time-Gain Compensation curve calculation
//              and VGA/LOG stage DAC output mapping.
// =========================================================================

`timescale 1ns / 1ps

module tune #(
    parameter GAIN_FRAC_WIDTH = 16     // Width of fractional part for fixed-point
)(
    // Clock and synchronous resets
    input  wire        dac_clk,        // DAC TGC clock 50 MHz
    input  wire        dac_rst_n,      // Synchronous reset active-low (dac_clk domain)

    // Synchronization trigger
    input  wire        i_dac_sync,     // Start sweep trigger (1-cycle pulse in dac_clk domain)

    // Configuration parameters (synchronous to dac_clk)
    input  wire [10:0] i_start_amp,    // Start total gain (0..2047 in DAC LSBs)
    input  wire [15:0] i_drop_ticks,   // Delay time before ramp starts (ticks)
    input  wire [31:0] i_amp_one,      // Step increment for 1st segment (Q_GAIN_FRAC_WIDTH)
    input  wire [31:0] i_amp_two,      // Step increment for 2nd segment (Q_GAIN_FRAC_WIDTH)
    input  wire [15:0] i_vrc_len,      // Duration of 1st segment (ticks)
    input  wire [9:0]  i_dac_min,      // Lower clamp limit for DACs
    input  wire [9:0]  i_dac_max,      // Upper clamp limit for DACs
    input  wire [1:0]  i_tune_mode,    // 00 = Sequential, 01 = Parallel
    input  wire [9:0]  i_log_offset,   // Log amp static offset calibration

    // Output interfaces to DAC SPI Transmitters
    output reg  [9:0]  o_dac_one,      // First VGA stage DAC code
    output reg  [9:0]  o_dac_two,      // Second VGA stage DAC code
    output reg  [9:0]  o_dac_offset,   // Log amp offset DAC code
    output reg         o_dac_data_vld  // Strobe indicating updated outputs are valid
);

    // =========================================================================
    // Registers and Internal Signals
    // =========================================================================
    reg [31:0] r_time_cnt;             // Time sweep counter
    reg [31:0] r_gain_acc;             // 32-bit fixed-point gain accumulator

    // Overflow protected additions for the accumulator
    wire [32:0] w_next_gain_one = {1'b0, r_gain_acc} + {1'b0, i_amp_one};
    wire [32:0] w_next_gain_two = {1'b0, r_gain_acc} + {1'b0, i_amp_two};

    // Extract integer gain component
    wire [31:0] w_gain_int_raw = r_gain_acc >> GAIN_FRAC_WIDTH;
    wire [11:0] w_gain_int     = w_gain_int_raw[11:0];

    // Transition boundary calculation
    wire [31:0] w_phase1_end = {16'b0, i_drop_ticks};
    wire [31:0] w_phase2_end = {16'b0, i_drop_ticks} + {16'b0, i_vrc_len};

    // =========================================================================
    // Sweep Accumulation Logic
    // =========================================================================
    always @(posedge dac_clk) begin
        if (!dac_rst_n) begin
            r_time_cnt <= 32'd0;
            r_gain_acc <= 32'd0;
        end else if (i_dac_sync) begin
            r_time_cnt <= 32'd0;
            r_gain_acc <= { {32-11{1'b0}}, i_start_amp } << GAIN_FRAC_WIDTH;
        end else begin
            // Saturate time counter to prevent roll-over issues
            if (r_time_cnt != 32'hFFFFFFFF) begin
                r_time_cnt <= r_time_cnt + 1'b1;
            end

            // Segment state machine based on elapsed cycles
            if (r_time_cnt <= w_phase1_end) begin
                r_gain_acc <= r_gain_acc; // Phase 1: Delay plateau
            end else if (r_time_cnt <= w_phase2_end) begin
                // Phase 2: First slope segment (with overflow protection)
                r_gain_acc <= w_next_gain_one[32] ? 32'hFFFFFFFF : w_next_gain_one[31:0];
            end else begin
                // Phase 3: Second slope segment (with overflow protection)
                r_gain_acc <= w_next_gain_two[32] ? 32'hFFFFFFFF : w_next_gain_two[31:0];
            end
        end
    end

    // =========================================================================
    // Sequential Mode Calculations
    // =========================================================================
    reg [11:0] r_dac_one_seq_raw;
    reg [11:0] r_dac_two_seq_raw;

    always @(*) begin
        if (w_gain_int < i_dac_max) begin
            r_dac_one_seq_raw = w_gain_int;
            r_dac_two_seq_raw = i_dac_min;
        end else begin
            r_dac_one_seq_raw = i_dac_max;
            r_dac_two_seq_raw = w_gain_int - i_dac_max + i_dac_min;
        end
    end

    // Clamping for sequential mode channels
    wire [9:0] w_dac_one_seq_clamp = (r_dac_one_seq_raw < i_dac_min) ? i_dac_min :
                                     (r_dac_one_seq_raw > i_dac_max) ? i_dac_max :
                                     r_dac_one_seq_raw[9:0];

    wire [9:0] w_dac_two_seq_clamp = (r_dac_two_seq_raw < i_dac_min) ? i_dac_min :
                                     (r_dac_two_seq_raw > i_dac_max) ? i_dac_max :
                                     r_dac_two_seq_raw[9:0];

    // =========================================================================
    // Parallel Mode Calculations
    // =========================================================================
    wire [10:0] w_gain_div2 = w_gain_int >> 1;

    wire [9:0] w_dac_par_clamp = (w_gain_div2 < i_dac_min) ? i_dac_min :
                                 (w_gain_div2 > i_dac_max) ? i_dac_max :
                                 w_gain_div2[9:0];

    // =========================================================================
    // Output Registers Drive
    // =========================================================================
    always @(posedge dac_clk) begin
        if (!dac_rst_n) begin
            o_dac_one      <= 10'd0;
            o_dac_two      <= 10'd0;
            o_dac_offset   <= 10'd0;
            o_dac_data_vld <= 1'b0;
        end else begin
            o_dac_offset   <= i_log_offset;
            o_dac_data_vld <= 1'b1; // Signal outputs are stable/valid

            case (i_tune_mode)
                2'b00: begin // Sequential Mode
                    o_dac_one <= w_dac_one_seq_clamp;
                    o_dac_two <= w_dac_two_seq_clamp;
                end
                2'b01: begin // Parallel Mode
                    o_dac_one <= w_dac_par_clamp;
                    o_dac_two <= w_dac_par_clamp;
                end
                default: begin // Default to Sequential Mode
                    o_dac_one <= w_dac_one_seq_clamp;
                    o_dac_two <= w_dac_two_seq_clamp;
                end
            endcase
        end
    end

endmodule