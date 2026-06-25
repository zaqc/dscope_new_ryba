`timescale 1ns / 1ps

module ascan_tb;

    // -------------------------------------------------------------------------
    // 1. Clock and Reset Generation
    // -------------------------------------------------------------------------
    reg adc_clk = 0;
    reg sys_clk = 0;
    
    // 65 MHz clock: Period = ~15.385 ns (7.692 ns half-period)
    always #7.692 adc_clk = ~adc_clk;
    
    // 80 MHz clock: Period = 12.5 ns (6.25 ns half-period)
    always #6.25 sys_clk = ~sys_clk;

    // Synchronous active-low resets
    reg adc_rst_n = 0;
    reg sys_rst_n = 0;

    initial begin
        #100;
        @(posedge adc_clk);
        adc_rst_n <= 1'b1;
        @(posedge sys_clk);
        sys_rst_n <= 1'b1;
    end

    // -------------------------------------------------------------------------
    // 2. Stimulus and Interface Signals
    // -------------------------------------------------------------------------
    reg         i_adc_sync = 0;
    reg [11:0]  i_adc_data = 0;
    reg [15:0]  i_n_samples = 0;
    reg [7:0]   i_accum = 0;
    reg [1:0]   i_accum_type = 0;
    reg [15:0]  i_skip_ticks = 0;

    wire [31:0] o_out_data;
    wire        o_out_vld;
    wire        i_out_rdy;
    wire [15:0] o_out_size;
    wire        o_data_ready;

    // -------------------------------------------------------------------------
    // 3. Module Under Test (MUT) Instantiation
    // -------------------------------------------------------------------------
    ascan #(
        .ADDR_WIDTH(10) // Use smaller address width for faster simulation
    ) u_ascan (
        .adc_clk      (adc_clk),
        .adc_rst_n    (adc_rst_n),
        .i_adc_sync   (i_adc_sync),
        .i_adc_data   (i_adc_data),
        
        .i_n_samples  (i_n_samples),
        .i_accum      (i_accum),
        .i_accum_type (i_accum_type),
        .i_skip_ticks (i_skip_ticks),

        .sys_clk      (sys_clk),
        .sys_rst_n    (sys_rst_n),

        .o_out_data   (o_out_data),
        .o_out_vld    (o_out_vld),
        .i_out_rdy    (i_out_rdy),
        .o_out_size   (o_out_size),
        .o_data_ready (o_data_ready)
    );

    // -------------------------------------------------------------------------
    // 4. Predictable ADC Test Data Source
    // -------------------------------------------------------------------------
    reg signed [11:0] adc_val = 12'sh000;
    always @(posedge adc_clk) begin
        if (!adc_rst_n) begin
            adc_val    <= 12'sh00A; // start at 10
            i_adc_data <= 12'sh000;
        end else begin
            i_adc_data <= adc_val;
            // Alternating pattern: +10, -20, +30, -40, +50...
            if (adc_val > 12'sh000) begin
                adc_val <= -(adc_val + 12'sd10);
            end else begin
                adc_val <= -adc_val + 12'sd10;
            end
            
            // Boundary wrap-around
            if (adc_val > 12'sd1000 || adc_val < -12'sd1000) begin
                adc_val <= 12'sd10;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 5. Output Stream Interface (AXI-Stream emulation with backpressure)
    // -------------------------------------------------------------------------
    reg rdy_reg = 0;
    assign i_out_rdy = rdy_reg;

    always @(posedge sys_clk) begin
        if (!sys_rst_n) begin
            rdy_reg <= 1'b0;
        end else begin
            // Simulates randomized receiver backpressure (80% probability of being ready)
            rdy_reg <= ($random % 10) < 8;
        end
    end

    // Track and log output data packets
    integer rx_word_idx = 0;
    always @(posedge sys_clk) begin
        if (sys_rst_n && o_out_vld && i_out_rdy) begin
            $display("[SYS_CLK @ %0t ns] Out Word[%0d]: 32'h%h (Bits: %b)", 
                     $time, rx_word_idx, o_out_data, o_out_data);
            rx_word_idx = rx_word_idx + 1;
        end
    end

    // Monitor bank readiness and transition logs
    always @(posedge sys_clk) begin
        if (o_data_ready) begin
            $display("[SYS_CLK @ %0t ns] o_data_ready is HIGH, Out Frame Size: %d words", $time, o_out_size);
        end
    end

    // -------------------------------------------------------------------------
    // 6. Test Sequences
    // -------------------------------------------------------------------------
    task trigger_measurement(
        input [15:0] samples,
        input [7:0]  accum,
        input [1:0]  accum_type,
        input [15:0] skip
    );
        begin
            $display("\n--- Starting Capture Command ---");
            $display("Samples: %0d, Accumulation Rate: %0d, Mode: %0d, Skip Ticks: %0d", 
                      samples, accum, accum_type, skip);
            
            @(posedge adc_clk);
            i_n_samples  <= samples;
            i_accum      <= accum;
            i_accum_type <= accum_type;
            i_skip_ticks <= skip;
            i_adc_sync   <= 1'b1;
            
            @(posedge adc_clk);
            i_adc_sync   <= 1'b0;
        end
    endtask

    initial begin
        // Output waveform configuration
`ifdef VCD_FILE
        $dumpfile(`VCD_FILE);
`else
        $dumpfile("ascan_tb.vcd");
`endif
        $dumpvars(0, ascan_tb);

        // Wait for resets to clear (resets are active-low, wait for high)
        wait(adc_rst_n == 1'b1 && sys_rst_n == 1'b1);
        #100;

        // ---------------------------------------------------------
        // TEST CASE 1: Peak Detector (Mode 0)
        // 16 samples, group accumulation of 2 (creates 8 output points)
        // 8 points x 12 bits = 96 bits -> 3 complete 32-bit packed words.
        // No skip ticks.
        // ---------------------------------------------------------
        rx_word_idx = 0;
        trigger_measurement(16'd16, 8'd2, 2'b00, 16'd0);
        
        // Wait until system clock domain flags that data is ready and transmitted
        wait(o_data_ready == 1'b1);
        wait(rx_word_idx == 3);
        #200;

        // ---------------------------------------------------------
        // TEST CASE 2: Integrator / Average (Mode 1)
        // 12 samples, group accumulation of 3 (creates 4 output points)
        // 4 points x 12 bits = 48 bits -> 2 packed words (second is padded).
        // Skip ticks: 4 cycles
        // ---------------------------------------------------------
        rx_word_idx = 0;
        trigger_measurement(16'd12, 8'd3, 2'b01, 16'd4);
        
        wait(o_data_ready == 1'b1);
        wait(rx_word_idx == 2);
        #200;

        // ---------------------------------------------------------
        // TEST CASE 3: Decimation (Mode 2)
        // 16 samples, decimation rate of 4 (creates 4 output points)
        // 4 points x 12 bits = 48 bits -> 2 packed words.
        // Skip ticks: 2 cycles
        // ---------------------------------------------------------
        rx_word_idx = 0;
        trigger_measurement(16'd16, 8'd4, 2'b10, 16'd2);
        
        wait(o_data_ready == 1'b1);
        wait(rx_word_idx == 2);
        #300;

        $display("\n==================================================");
        $display("   AScan Simulation Success / Verification Ended  ");
        $display("==================================================");
        $finish;
    end

endmodule