// =========================================================================
// Module: rst_sync
// Description: Generates synchronous active-low resets (minimum 8 cycles)
//              and synchronous single-cycle trigger pulses for 5 clock domains.
//              Fully compliant with the 16-Channel UT Defectoscope specifications.
// =========================================================================

`timescale 1ns / 1ps

module rst_sync (
    // Clock inputs (No i_ prefix as per naming conventions)
    input  wire sys_clk,
    input  wire adc_clk,
    input  wire log_clk,
    input  wire dac_clk,
    input  wire hi_clk,

    // Global control inputs
    input  wire rst_n,        // Active-low external reset (No i_ prefix)
    input  wire i_sys_sync,   // Master trigger input (synchronous to sys_clk)

    // Synchronous active-low reset outputs (held for >= 8 cycles)
    output wire o_sys_rst_n,
    output wire o_adc_rst_n,
    output wire o_log_rst_n,
    output wire o_dac_rst_n,
    output wire o_hi_rst_n,

    // Synchronous trigger pulse outputs (exactly 1 cycle duration)
    output wire o_sys_sync,
    output wire o_adc_sync,
    output wire o_log_sync,
    output wire o_dac_sync,
    output wire o_hi_sync
);

    // =========================================================================
    // 1. Reset Generators Instantiation (Synchronous Active-Low Resets)
    // =========================================================================
    rst_gen u_sys_rst_gen ( .clk(sys_clk), .rst_n(rst_n), .o_rst_n(o_sys_rst_n) );
    rst_gen u_adc_rst_gen ( .clk(adc_clk), .rst_n(rst_n), .o_rst_n(o_adc_rst_n) );
    rst_gen u_log_rst_gen ( .clk(log_clk), .rst_n(rst_n), .o_rst_n(o_log_rst_n) );
    rst_gen u_dac_rst_gen ( .clk(dac_clk), .rst_n(rst_n), .o_rst_n(o_dac_rst_n) );
    rst_gen u_hi_rst_gen  ( .clk(hi_clk),  .rst_n(rst_n), .o_rst_n(o_hi_rst_n)  );

    // =========================================================================
    // 2. Edge Detector in System Domain (sys_clk)
    // =========================================================================
    reg sys_sync_reg = 1'b0;

    always @(posedge sys_clk) begin
        if (!o_sys_rst_n) begin
            sys_sync_reg <= 1'b0;
        end else begin
            sys_sync_reg <= i_sys_sync;
        end
    end
    
    // Generate 1-cycle pulse on the rising edge of i_sys_sync
    assign o_sys_sync = i_sys_sync && !sys_sync_reg;

    // =========================================================================
    // 3. Clock Domain Crossing for Trigger Pulses (CDC Toggle-Synchronizers)
    // =========================================================================
    
    // CDC: sys_clk -> adc_clk
    cdc_pulse_sync u_adc_pulse_sync (
        .src_clk     (sys_clk),
        .src_rst_n   (o_sys_rst_n),
        .i_src_pulse (o_sys_sync),
        .dst_clk     (adc_clk),
        .dst_rst_n   (o_adc_rst_n),
        .o_dst_pulse (o_adc_sync)
    );

    // CDC: sys_clk -> log_clk
    cdc_pulse_sync u_log_pulse_sync (
        .src_clk     (sys_clk),
        .src_rst_n   (o_sys_rst_n),
        .i_src_pulse (o_sys_sync),
        .dst_clk     (log_clk),
        .dst_rst_n   (o_log_rst_n),
        .o_dst_pulse (o_log_sync)
    );

    // CDC: sys_clk -> dac_clk
    cdc_pulse_sync u_dac_pulse_sync (
        .src_clk     (sys_clk),
        .src_rst_n   (o_sys_rst_n),
        .i_src_pulse (o_sys_sync),
        .dst_clk     (dac_clk),
        .dst_rst_n   (o_dac_rst_n),
        .o_dst_pulse (o_dac_sync)
    );

    // CDC: sys_clk -> hi_clk
    cdc_pulse_sync u_hi_pulse_sync (
        .src_clk     (sys_clk),
        .src_rst_n   (o_sys_rst_n),
        .i_src_pulse (o_sys_sync),
        .dst_clk     (hi_clk),
        .dst_rst_n   (o_hi_rst_n),
        .o_dst_pulse (o_hi_sync)
    );

endmodule


// =========================================================================
// Helper Module: rst_gen
// Description: Implements an 8-bit shift register initialized to 0.
//              Generates a synchronous active-low reset held for 8 cycles.
// =========================================================================
module rst_gen (
    input  wire clk,
    input  wire rst_n,
    output wire o_rst_n
);
    reg [7:0] rst_reg = 8'h00;

    always @(posedge clk) begin
        if (!rst_n) begin
            rst_reg <= 8'h00;
        end else begin
            rst_reg <= {rst_reg[6:0], 1'b1};
        end
    end

    assign o_rst_n = rst_reg[7];

endmodule


// =========================================================================
// Helper Module: cdc_pulse_sync
// Description: Robust Clock Domain Crossing (CDC) for pulse signals.
//              Converts pulse to toggle in source domain, synchronizes 
//              level, and reconstructs 1-cycle pulse in destination domain.
//              Uses active-low synchronous resets.
// =========================================================================
module cdc_pulse_sync (
    input  wire src_clk,
    input  wire src_rst_n,
    input  wire i_src_pulse,
    input  wire dst_clk,
    input  wire dst_rst_n,
    output wire o_dst_pulse
);
    reg src_toggle = 1'b0;

    // Toggle level generation in the source clock domain
    always @(posedge src_clk) begin
        if (!src_rst_n) begin
            src_toggle <= 1'b0;
        end else if (i_src_pulse) begin
            src_toggle <= !src_toggle;
        end
    end

    // 3-stage synchronizer shift-register in the destination domain
    reg [2:0] dst_sync_reg = 3'b000;

    always @(posedge dst_clk) begin
        if (!dst_rst_n) begin
            dst_sync_reg <= 3'b000;
        end else begin
            dst_sync_reg <= {dst_sync_reg[1:0], src_toggle};
        end
    end

    // Reconstruct edge/pulse in destination clock domain via XOR
    assign o_dst_pulse = dst_sync_reg[1] ^ dst_sync_reg[2];

endmodule