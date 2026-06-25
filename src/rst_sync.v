// =========================================================================
// Module: rst_sync
// Description: Generates synchronous active-high resets (minimum 8 cycles)
//              and synchronous single-cycle trigger pulses for 5 clock domains.
//              Input trigger 'i_sys_sync' is synchronous to 'sys_clk'.
// =========================================================================

`timescale 1ns / 1ps

module rst_sync (
    // Clock inputs (no i_ prefix)
    input  wire sys_clk,
    input  wire adc_clk,
    input  wire log_clk,
    input  wire dac_clk,
    input  wire hi_clk,

    // Global control inputs
    input  wire rst_n,        // Active-low external reset
    input  wire i_sys_sync,   // Master trigger input (synchronous to sys_clk)

    // Synchronous active-high reset outputs (held for >= 8 cycles)
    output wire o_sys_rst,
    output wire o_adc_rst,
    output wire o_log_rst,
    output wire o_dac_rst,
    output wire o_hi_rst,

    // Synchronous trigger pulse outputs (exactly 1 cycle duration)
    output wire o_sys_sync,
    output wire o_adc_sync,
    output wire o_log_sync,
    output wire o_dac_sync,
    output wire o_hi_sync
);

    // =========================================================================
    // 1. Reset Generators Instantiation
    // =========================================================================
    rst_gen u_sys_rst_gen ( .clk(sys_clk), .rst_n(rst_n), .rst_out(o_sys_rst) );
    rst_gen u_adc_rst_gen ( .clk(adc_clk), .rst_n(rst_n), .rst_out(o_adc_rst) );
    rst_gen u_log_rst_gen ( .clk(log_clk), .rst_n(rst_n), .rst_out(o_log_rst) );
    rst_gen u_dac_rst_gen ( .clk(dac_clk), .rst_n(rst_n), .rst_out(o_dac_rst) );
    rst_gen u_hi_rst_gen  ( .clk(hi_clk),  .rst_n(rst_n), .rst_out(o_hi_rst)  );

    // =========================================================================
    // 2. Source Domain Edge Detector (sys_clk)
    // =========================================================================
    reg sys_trig_d = 1'b0;

    always @(posedge sys_clk) begin
        if (o_sys_rst) begin
            sys_trig_d <= 1'b0;
        end else begin
            sys_trig_d <= i_sys_sync;
        end
    end
    
    // Generate 1-cycle pulse in sys_clk domain on rising edge of i_sys_sync
    assign o_sys_sync = i_sys_sync && !sys_trig_d;

    // =========================================================================
    // 3. Clock Domain Crossing for Trigger Pulses
    // =========================================================================
    
    // CDC: sys_clk -> adc_clk
    cdc_pulse_sync u_adc_pulse_sync (
        .src_clk   (sys_clk),
        .src_rst   (o_sys_rst),
        .src_pulse (o_sys_sync),
        .dst_clk   (adc_clk),
        .dst_rst   (o_adc_rst),
        .dst_pulse (o_adc_sync)
    );

    // CDC: sys_clk -> log_clk
    cdc_pulse_sync u_log_pulse_sync (
        .src_clk   (sys_clk),
        .src_rst   (o_sys_rst),
        .src_pulse (o_sys_sync),
        .dst_clk   (log_clk),
        .dst_rst   (o_log_rst),
        .dst_pulse (o_log_sync)
    );

    // CDC: sys_clk -> dac_clk
    cdc_pulse_sync u_dac_pulse_sync (
        .src_clk   (sys_clk),
        .src_rst   (o_sys_rst),
        .src_pulse (o_sys_sync),
        .dst_clk   (dac_clk),
        .dst_rst   (o_dac_rst),
        .dst_pulse (o_dac_sync)
    );

    // CDC: sys_clk -> hi_clk
    cdc_pulse_sync u_hi_pulse_sync (
        .src_clk   (sys_clk),
        .src_rst   (o_sys_rst),
        .src_pulse (o_sys_sync),
        .dst_clk   (hi_clk),
        .dst_rst   (o_hi_rst),
        .dst_pulse (o_hi_sync)
    );

endmodule


// =========================================================================
// Helper Module: rst_gen
// Description: Implements an 8-bit shift register to safely deassert
//              reset synchronously with the target clock domain.
// =========================================================================
module rst_gen (
    input  wire clk,
    input  wire rst_n,
    output wire rst_out
);
    reg [7:0] rst_reg = 8'hFF;

    always @(posedge clk) begin
        if (!rst_n) begin
            rst_reg <= 8'hFF;
        end else begin
            rst_reg <= {rst_reg[6:0], 1'b0};
        end
    end

    assign rst_out = rst_reg[7];

endmodule


// =========================================================================
// Helper Module: cdc_pulse_sync
// Description: Robust Clock Domain Crossing (CDC) for pulse signals.
//              Converts pulse to toggle in source domain, synchronizes 
//              level, and reconstructs 1-cycle pulse in destination domain.
// =========================================================================
module cdc_pulse_sync (
    input  wire src_clk,
    input  wire src_rst,
    input  wire src_pulse,
    input  wire dst_clk,
    input  wire dst_rst,
    output wire dst_pulse
);
    reg src_toggle = 1'b0;

    // Toggle level generation in the source clock domain
    always @(posedge src_clk) begin
        if (src_rst) begin
            src_toggle <= 1'b0;
        end else if (src_pulse) begin
            src_toggle <= !src_toggle;
        end
    end

    // 3-stage synchronizer shift-register in the destination domain
    reg [2:0] dst_sync_reg = 3'b000;

    always @(posedge dst_clk) begin
        if (dst_rst) begin
            dst_sync_reg <= 3'b000;
        end else begin
            dst_sync_reg <= {dst_sync_reg[1:0], src_toggle};
        end
    end

    // Reconstruct edge/pulse in destination clock domain
    assign dst_pulse = dst_sync_reg[1] ^ dst_sync_reg[2];

endmodule