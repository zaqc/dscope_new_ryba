// =========================================================================
// Ultrasound Defectoscope Project
// Module: spi_dac101s101
// Description: SPI Controller for 10-bit TI DAC101S101
//              Generates Time-Varying Gain (TGC) voltages.
//              Operates in dac_clk (50 MHz) domain.
// =========================================================================

module spi_dac101s101 (
    input            dac_clk,      // 50 MHz DAC clock
    input            dac_rst_n,    // Synchronous active-low reset (from rst_sync)
    
    // Control / Data Interface
    input [9:0]      i_dac_data,   // 10-bit input value to be written to DAC
    input            i_dac_wr,     // Write strobe (1 dac_clk pulse) to initiate transaction
    output           o_dac_busy,   // High when transaction is in progress
    
    // Physical DAC Interface (SPI)
    output           o_sclk,       // SPI Serial Clock (12.5 MHz)
    output           o_din,        // SPI Serial Data input (MOSI)
    output           o_sync_n      // SPI Sync / Chip Select (Active Low)
);

    // State machine parameter declarations
    localparam [2:0] ST_IDLE      = 3'd0,
                     ST_PREPARE   = 3'd1,
                     ST_TRANSMIT  = 3'd2,
                     ST_DONE      = 3'd3;

    // Internal registers
    reg [2:0]  r_state;
    reg [1:0]  r_clk_div;     // Clock divider for SCLK (50 MHz / 4 = 12.5 MHz)
    reg [3:0]  r_bit_cnt;     // Tracks 16 bits of transmission (0 to 15)
    reg [15:0] r_shift_reg;   // 16-bit transmit shift register
    
    reg        r_sclk;
    reg        r_din;
    reg        r_sync_n;
    reg        r_busy;

    // Output assignments
    assign o_sclk     = r_sclk;
    assign o_din      = r_din;
    assign o_sync_n   = r_sync_n;
    assign o_dac_busy = r_busy;

    // Main sequential block
    always @(posedge dac_clk) begin
        if (!dac_rst_n) begin
            r_state     <= ST_IDLE;
            r_clk_div   <= 2'd0;
            r_bit_cnt   <= 4'd0;
            r_shift_reg <= 16'd0;
            r_sclk      <= 1'b0;
            r_din       <= 1'b0;
            r_sync_n    <= 1'b1;
            r_busy      <= 1'b0;
        end else begin
            case (r_state)
                
                // Wait for write command
                ST_IDLE: begin
                    r_sclk    <= 1'b0;
                    r_din     <= 1'b0;
                    r_sync_n  <= 1'b1;
                    r_busy    <= 1'b0;
                    r_clk_div <= 2'd0;
                    
                    if (i_dac_wr) begin
                        // Format: 2'b00 (Normal mode) + 10-bit data + 4'b0000 (Don't care)
                        r_shift_reg <= {2'b00, i_dac_data, 4'b0000};
                        r_din       <= 1'b0; // First bit (DB15) is 0
                        r_busy      <= 1'b1;
                        r_state     <= ST_PREPARE;
                    end
                end

                // Hardware setup delay (SYNC_N low to SCLK falling setup time)
                ST_PREPARE: begin
                    r_sync_n <= 1'b0;
                    // Wait 4 dac_clk cycles (80ns) before starting SCLK
                    if (r_clk_div == 2'd3) begin
                        r_clk_div <= 2'd0;
                        r_bit_cnt <= 4'd0;
                        r_state   <= ST_TRANSMIT;
                    end else begin
                        r_clk_div <= r_clk_div + 1'b1;
                    end
                end

                // Clock out 16 bits of data
                ST_TRANSMIT: begin
                    r_clk_div <= r_clk_div + 1'b1;
                    
                    if (r_clk_div == 2'd0) begin
                        r_sclk <= 1'b1; // Rising edge of SCLK
                        // Update DIN on rising edge (except for first bit which was placed early)
                        if (r_bit_cnt != 4'd0) begin
                            r_shift_reg <= {r_shift_reg[14:0], 1'b0};
                            r_din       <= r_shift_reg[14];
                        end
                    end 
                    else if (r_clk_div == 2'd2) begin
                        r_sclk <= 1'b0; // Falling edge of SCLK (DAC samples DIN here)
                    end 
                    else if (r_clk_div == 2'd3) begin
                        r_clk_div <= 2'd0;
                        if (r_bit_cnt == 4'd15) begin
                            r_state <= ST_DONE;
                        end else begin
                            r_bit_cnt <= r_bit_cnt + 1'b1;
                        end
                    end
                end

                // Finish frame, raise SYNC_N and satisfy minimum idle high duration
                ST_DONE: begin
                    r_sync_n <= 1'b1;
                    r_din    <= 1'b0;
                    r_sclk   <= 1'b0;
                    
                    // Hold SYNC_N high for 4 cycles (80ns) to meet DAC t_s_high spec
                    if (r_clk_div == 2'd3) begin
                        r_busy  <= 1'b0;
                        r_state <= ST_IDLE;
                    end else begin
                        r_clk_div <= r_clk_div + 1'b1;
                    end
                end

                default: r_state <= ST_IDLE;
            endcase
        end
    end

endmodule