// =========================================================================
// Global Project Module: ascan (A-Scan Processor & Buffer)
// =========================================================================

`default_nettype none

module ascan #(
    parameter ADDR_WIDTH = 15 // Depth of each ping-pong buffer = 32768 words (128 KB)
)(
    // ADC clock domain (65 MHz)
    input  wire                    adc_clk,
    input  wire                    adc_rst,
    input  wire                    i_adc_sync,
    input  wire signed [11:0]      i_adc_data,

    // Configuration parameters (captured on i_adc_sync)
    input  wire [15:0]             i_n_samples,
    input  wire [7:0]              i_accum,
    input  wire [1:0]              i_accum_type,
    input  wire [15:0]             i_skip_ticks,

    // System clock domain (80 MHz)
    input  wire                    sys_clk,
    input  wire                    sys_rst,

    // Stream Output Interface (sys_clk domain)
    output wire [31:0]             o_out_data,
    output wire                    o_out_vld,
    input  wire                    i_out_rdy,
    output reg  [15:0]             o_out_size,
    output reg                     o_data_ready
);

    // -------------------------------------------------------------------------
    // 1. Accumulator Submodule Instantiation
    // -------------------------------------------------------------------------
    wire [11:0] accum_data;
    wire        accum_vld;
    wire        accum_last;

    ascan_accum u_accum (
        .clk          (adc_clk),
        .rst          (adc_rst),
        .i_sync       (i_adc_sync),
        .i_data       (i_adc_data),
        .i_n_samples  (i_n_samples),
        .i_accum      (i_accum),
        .i_accum_type (i_accum_type),
        .i_skip_ticks (i_skip_ticks),
        .o_data       (accum_data),
        .o_vld        (accum_vld),
        .o_last       (accum_last)
    );

    // -------------------------------------------------------------------------
    // 2. Packer Submodule Instantiation
    // -------------------------------------------------------------------------
    wire [31:0] packed_word;
    wire        packed_word_vld;
    wire        packed_frame_done;

    ascan_packer u_packer (
        .clk          (adc_clk),
        .rst          (adc_rst),
        .i_data       (accum_data),
        .i_vld        (accum_vld),
        .i_last       (accum_last),
        .o_word       (packed_word),
        .o_word_vld   (packed_word_vld),
        .o_frame_done (packed_frame_done)
    );

    // -------------------------------------------------------------------------
    // 3. Ping-Pong Buffers ("Butterfly" Buffer) and CDC Handshake Logic
    // -------------------------------------------------------------------------
    reg                    write_bank;     // 0 or 1 (adc_clk domain)
    reg [ADDR_WIDTH-1:0]   waddr;          // Write address pointer
    reg [15:0]             frame_size_0;   // Captured size of Bank 0
    reg [15:0]             frame_size_1;   // Captured size of Bank 1

    reg                    bank_0_avail;   // Bank 0 contains complete valid frame (adc_clk domain)
    reg                    bank_1_avail;   // Bank 1 contains complete valid frame (adc_clk domain)

    wire                   bank_0_clear_adc; // Cleared status synced back to adc_clk
    wire                   bank_1_clear_adc;

    // CDC: Synchronize clear signals from sys_clk to adc_clk domain
    ascan_sync #(.WIDTH(1)) u_sync_clear_0 (
        .clk   (adc_clk),
        .rst   (adc_rst),
        .i_sig (bank_0_clear_sys),
        .o_sig (bank_0_clear_adc)
    );

    ascan_sync #(.WIDTH(1)) u_sync_clear_1 (
        .clk   (adc_clk),
        .rst   (adc_rst),
        .i_sig (bank_1_clear_sys),
        .o_sig (bank_1_clear_adc)
    );

    // Writing Control Logic (adc_clk domain)
    wire ram_we_0 = (write_bank == 1'b0) && packed_word_vld;
    wire ram_we_1 = (write_bank == 1'b1) && packed_word_vld;

    always @(posedge adc_clk or posedge adc_rst) begin
        if (adc_rst) begin
            waddr        <= 0;
            write_bank   <= 1'b0;
            bank_0_avail <= 1'b0;
            bank_1_avail <= 1'b0;
            frame_size_0 <= 16'd0;
            frame_size_1 <= 16'd0;
        end else begin
            // Lower availability flag once sys_clk domain finished reading and cleared it
            if (bank_0_clear_adc) begin
                bank_0_avail <= 1'b0;
            end
            if (bank_1_clear_adc) begin
                bank_1_avail <= 1'b0;
            end

            // Increment write address on new word
            if (packed_word_vld) begin
                waddr <= waddr + 1'b1;
            end

            // Complete frame written, commit size and switch bank
            if (packed_frame_done) begin
                if (write_bank == 1'b0) begin
                    frame_size_0 <= waddr + (packed_word_vld ? 1'b1 : 1'b0);
                    bank_0_avail <= 1'b1;
                    write_bank   <= 1'b1;
                end else begin
                    frame_size_1 <= waddr + (packed_word_vld ? 1'b1 : 1'b0);
                    bank_1_avail <= 1'b1;
                    write_bank   <= 1'b0;
                end
                waddr <= 0;
            end
        end
    end

    // RAM Instances mapped to external dp_ram ports
    wire [ADDR_WIDTH-1:0] raddr;
    wire [31:0]           rdata_0;
    wire [31:0]           rdata_1;

    dp_ram #(
        .DATA_WIDTH(32),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_ram_bank_0 (
        .clk_a  (adc_clk),
        .addr_a (waddr),
        .we_a   (ram_we_0),
        .d_a    (packed_word),
        .q_a    (), // Unused on write-only port

        .clk_b  (sys_clk),
        .addr_b (raddr),
        .we_b   (1'b0),
        .d_b    (32'd0),
        .q_b    (rdata_0)
    );

    dp_ram #(
        .DATA_WIDTH(32),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_ram_bank_1 (
        .clk_a  (adc_clk),
        .addr_a (waddr),
        .we_a   (ram_we_1),
        .d_a    (packed_word),
        .q_a    (), // Unused on write-only port

        .clk_b  (sys_clk),
        .addr_b (raddr),
        .we_b   (1'b0),
        .d_b    (32'd0),
        .q_b    (rdata_1)
    );

    // -------------------------------------------------------------------------
    // 4. System Domain Processing and Readout Logic (sys_clk)
    // -------------------------------------------------------------------------
    wire bank_0_avail_sys;
    wire bank_1_avail_sys;

    // CDC: Synchronize available signals from adc_clk to sys_clk domain
    ascan_sync #(.WIDTH(1)) u_sync_avail_0 (
        .clk   (sys_clk),
        .rst   (sys_rst),
        .i_sig (bank_0_avail),
        .o_sig (bank_0_avail_sys)
    );

    ascan_sync #(.WIDTH(1)) u_sync_avail_1 (
        .clk   (sys_clk),
        .rst   (sys_rst),
        .i_sig (bank_1_avail),
        .o_sig (bank_1_avail_sys)
    );

    reg bank_0_clear_sys;
    reg bank_1_clear_sys;

    // FSM States for reading
    localparam R_IDLE  = 2'd0;
    localparam R_READ  = 2'd1;
    localparam R_CLEAR = 2'd2;

    reg [1:0]  read_state;
    reg        read_bank;
    reg [15:0] read_size;
    reg [15:0] raddr_cnt;
    reg [15:0] pop_cnt;

    // Read address assigned to RAMs
    reg [ADDR_WIDTH-1:0] raddr_reg;
    assign raddr = raddr_reg;

    // Select source data from active bank
    wire [31:0] rdata = (read_bank == 1'b0) ? rdata_0 : rdata_1;

    // Read FIFO to match AXI-Stream backpressure with 0-bubble performance
    reg [31:0] fifo_mem [3:0];
    reg [1:0]  fifo_wptr;
    reg [1:0]  fifo_rptr;
    reg [2:0]  fifo_cnt;

    wire       fifo_full = (fifo_cnt == 3'd4);
    wire       fifo_empty = (fifo_cnt == 3'd0);

    reg        ram_read_en;
    reg        ram_read_val;

    always @(posedge sys_clk or posedge sys_rst) begin
        if (sys_rst) begin
            read_state       <= R_IDLE;
            read_bank        <= 1'b0;
            read_size        <= 16'd0;
            raddr_reg        <= 0;
            raddr_cnt        <= 0;
            pop_cnt          <= 0;
            bank_0_clear_sys <= 1'b0;
            bank_1_clear_sys <= 1'b0;
            o_out_size       <= 16'd0;
            o_data_ready     <= 1'b0;
            ram_read_en      <= 1'b0;
            ram_read_val     <= 1'b0;
        end else begin
            ram_read_en  <= 1'b0;
            ram_read_val <= ram_read_en;

            case (read_state)
                R_IDLE: begin
                    o_data_ready <= 1'b0;
                    if (bank_0_avail_sys && !bank_0_clear_sys) begin
                        read_bank    <= 1'b0;
                        read_size    <= frame_size_0;
                        o_out_size   <= frame_size_0;
                        o_data_ready <= 1'b1;
                        raddr_reg    <= 0;
                        raddr_cnt    <= 0;
                        pop_cnt      <= 0;
                        read_state   <= R_READ;
                    end else if (bank_1_avail_sys && !bank_1_clear_sys) begin
                        read_bank    <= 1'b1;
                        read_size    <= frame_size_1;
                        o_out_size   <= frame_size_1;
                        o_data_ready <= 1'b1;
                        raddr_reg    <= 0;
                        raddr_cnt    <= 0;
                        pop_cnt      <= 0;
                        read_state   <= R_READ;
                    end
                end

                R_READ: begin
                    o_data_ready <= 1'b1;
                    // Read from RAM if there is space in output FIFO and more words are left
                    if ((raddr_cnt < read_size) && (fifo_cnt + ram_read_en < 3'd4)) begin
                        ram_read_en <= 1'b1;
                        raddr_reg   <= raddr_reg + 1'b1;
                        raddr_cnt   <= raddr_cnt + 1'b1;
                    end

                    // Count popped items from output FIFO
                    if (o_out_vld && i_out_rdy) begin
                        pop_cnt <= pop_cnt + 1'b1;
                    end

                    // Finished transfer
                    if (pop_cnt == read_size) begin
                        o_data_ready <= 1'b0;
                        if (read_bank == 1'b0) begin
                            bank_0_clear_sys <= 1'b1;
                        end else begin
                            bank_1_clear_sys <= 1'b1;
                        end
                        read_state <= R_CLEAR;
                    end
                end

                R_CLEAR: begin
                    // Wait for acknowledgment from write clock domain
                    if (read_bank == 1'b0 && !bank_0_avail_sys) begin
                        bank_0_clear_sys <= 1'b0;
                        read_state       <= R_IDLE;
                    end else if (read_bank == 1'b1 && !bank_1_avail_sys) begin
                        bank_1_clear_sys <= 1'b0;
                        read_state       <= R_IDLE;
                    end
                end
            endcase
        end
    end

    // Output FIFO Controller (Sync)
    wire fifo_push = ram_read_val;
    wire fifo_pop  = o_out_vld && i_out_rdy;

    always @(posedge sys_clk or posedge sys_rst) begin
        if (sys_rst) begin
            fifo_wptr <= 2'd0;
            fifo_rptr <= 2'd0;
            fifo_cnt  <= 3'd0;
        end else begin
            if (fifo_push && !fifo_pop) begin
                fifo_mem[fifo_wptr] <= rdata;
                fifo_wptr           <= fifo_wptr + 1'b1;
                fifo_cnt            <= fifo_cnt + 1'b1;
            end else if (!fifo_push && fifo_pop) begin
                fifo_rptr           <= fifo_rptr + 1'b1;
                fifo_cnt            <= fifo_cnt - 1'b1;
            end else if (fifo_push && fifo_pop) begin
                fifo_mem[fifo_wptr] <= rdata;
                fifo_wptr           <= fifo_wptr + 1'b1;
                fifo_rptr           <= fifo_rptr + 1'b1;
            end
        end
    end

    assign o_out_vld  = !fifo_empty;
    assign o_out_data = fifo_mem[fifo_rptr];

endmodule


// =========================================================================
// Auxiliary Module: Accumulator and Detector (ascan_accum)
// =========================================================================

module ascan_accum (
    input  wire                    clk,
    input  wire                    rst,
    input  wire                    i_sync,
    input  wire signed [11:0]      i_data,

    // Runtime Parameters (latched on i_sync)
    input  wire [15:0]             i_n_samples,
    input  wire [7:0]              i_accum,
    input  wire [1:0]              i_accum_type,
    input  wire [15:0]             i_skip_ticks,

    // Outputs to Packer
    output reg  [11:0]             o_data,
    output reg                     o_vld,
    output reg                     o_last
);

    // Latch configuration on start sync
    reg [15:0] r_n_samples;
    reg [7:0]  r_accum;
    reg [1:0]  r_accum_type;
    reg [15:0] r_skip_ticks;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            r_n_samples  <= 16'd0;
            r_accum      <= 8'd1;
            r_accum_type <= 2'b00;
            r_skip_ticks <= 16'd0;
        end else if (i_sync) begin
            r_n_samples  <= i_n_samples;
            r_accum      <= (i_accum == 8'd0) ? 8'd1 : i_accum;
            r_accum_type <= i_accum_type;
            r_skip_ticks <= i_skip_ticks;
        end
    end

    // Convert signed ADC inputs to absolute unsigned values (12-bit, max value 2048)
    reg [11:0] abs_data;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            abs_data <= 12'd0;
        end else begin
            if (i_data[11]) begin
                // abs(-2048) = 2048. Max value perfectly fits in unsigned 12 bits [0..4095]
                if (i_data == 12'sh800)
                    abs_data <= 12'd2048;
                else
                    abs_data <= -i_data;
            end else begin
                abs_data <= i_data;
            end
        end
    end

    // Match start triggers with absolute data pipeline latency
    reg sync_reg;
    always @(posedge clk or posedge rst) begin
        if (rst) sync_reg <= 1'b0;
        else     sync_reg <= i_sync;
    end

    // FSM States
    localparam S_IDLE    = 2'd0;
    localparam S_SKIP    = 2'd1;
    localparam S_CAPTURE = 2'd2;

    reg [1:0]  state;
    reg [15:0] skip_cnt;
    reg [15:0] sample_cnt;
    reg [7:0]  accum_cnt;

    // Accumulation registers
    reg [11:0] max_reg;
    reg [19:0] sum_reg;
    reg [11:0] dec_reg;

    reg        group_done_raw;
    reg        group_last_raw;

    // Boundary conditions
    wire is_last_of_group = (accum_cnt == r_accum - 1'b1) || (sample_cnt == r_n_samples - 1'b1);
    wire is_last_of_frame = (sample_cnt == r_n_samples - 1'b1);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state          <= S_IDLE;
            skip_cnt       <= 16'd0;
            sample_cnt     <= 16'd0;
            accum_cnt      <= 8'd0;
            max_reg        <= 12'd0;
            sum_reg        <= 20'd0;
            dec_reg        <= 12'd0;
            group_done_raw <= 1'b0;
            group_last_raw <= 1'b0;
        end else begin
            group_done_raw <= 1'b0;
            group_last_raw <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (sync_reg) begin
                        if (r_skip_ticks == 16'd0) begin
                            state <= S_CAPTURE;
                        end else begin
                            state    <= S_SKIP;
                            skip_cnt <= r_skip_ticks - 1'b1;
                        end
                        sample_cnt <= 16'd0;
                        accum_cnt  <= 8'd0;
                    end
                end

                S_SKIP: begin
                    skip_cnt <= skip_cnt - 1'b1;
                    if (skip_cnt == 16'd0) begin
                        state <= S_CAPTURE;
                    end
                end

                S_CAPTURE: begin
                    // Store / update data in accumulation registers
                    if (accum_cnt == 8'd0) begin
                        max_reg <= abs_data;
                        sum_reg <= {8'd0, abs_data};
                        dec_reg <= abs_data;
                    end else begin
                        if (abs_data > max_reg) begin
                            max_reg <= abs_data;
                        end
                        sum_reg <= sum_reg + abs_data;
                    end

                    sample_cnt <= sample_cnt + 1'b1;
                    accum_cnt  <= accum_cnt + 1'b1;

                    if (is_last_of_group) begin
                        group_done_raw <= 1'b1;
                        accum_cnt      <= 8'd0;
                        if (is_last_of_frame) begin
                            group_last_raw <= 1'b1;
                            state          <= S_IDLE;
                        end
                    end
                end
            endcase
        end
    end

    // Pipeline Stage 2: Combinational decision aligned with Divider delay
    wire [7:0] divisor = (r_accum == 8'd0) ? 8'd1 : r_accum;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            o_vld  <= 1'b0;
            o_last <= 1'b0;
            o_data <= 12'd0;
        end else begin
            o_vld  <= group_done_raw;
            o_last <= group_last_raw;

            if (group_done_raw) begin
                case (r_accum_type)
                    2'b00: begin // Peak Detector
                        o_data <= (accum_cnt == 8'd0) ? abs_data : ((abs_data > max_reg) ? abs_data : max_reg);
                    end
                    2'b01: begin // Integrator (Average)
                        if (accum_cnt == 8'd0) begin
                            o_data <= abs_data;
                        end else begin
                            o_data <= (sum_reg + abs_data) / divisor;
                        end
                    end
                    2'b10: begin // Decimation
                        o_data <= (accum_cnt == 8'd0) ? abs_data : dec_reg;
                    end
                    default: begin
                        o_data <= (accum_cnt == 8'd0) ? abs_data : dec_reg;
                    end
                endcase
            end
        end
    end

endmodule


// =========================================================================
// Auxiliary Module: Bit Packer (ascan_packer)
// =========================================================================

module ascan_packer (
    input  wire                    clk,
    input  wire                    rst,
    input  wire [11:0]             i_data,
    input  wire                    i_vld,
    input  wire                    i_last,

    output reg  [31:0]             o_word,
    output reg                     o_word_vld,
    output reg                     o_frame_done
);

    reg [63:0] bit_buf;
    reg [5:0]  bit_cnt;
    reg        flush_active;
    reg [31:0] flush_buf;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            bit_buf      <= 64'd0;
            bit_cnt      <= 6'd0;
            o_word       <= 32'd0;
            o_word_vld   <= 1'b0;
            o_frame_done <= 1'b0;
            flush_active <= 1'b0;
            flush_buf    <= 32'd0;
        end else begin
            o_word_vld   <= 1'b0;
            o_frame_done <= 1'b0;

            if (i_vld) begin
                reg [63:0] next_bit_buf;
                reg [5:0]  next_bit_cnt;

                // Append incoming 12-bit sample to current bit position
                next_bit_buf = bit_buf | ({{52{1'b0}}, i_data} << bit_cnt);
                next_bit_cnt = bit_cnt + 6'd12;

                if (next_bit_cnt >= 6'd32) begin
                    o_word       <= next_bit_buf[31:0];
                    o_word_vld   <= 1'b1;
                    next_bit_buf = next_bit_buf >> 32;
                    next_bit_cnt = next_bit_cnt - 6'd32;

                    if (i_last) begin
                        if (next_bit_cnt > 0) begin
                            // Extra bits remain, need one extra cycle (flush) to empty them
                            flush_active <= 1'b1;
                            flush_buf    <= next_bit_buf[31:0];
                        end else begin
                            o_frame_done <= 1'b1;
                        end
                    end
                end else begin
                    // Less than 32 bits, but frame ended: output zero-padded partial word
                    if (i_last) begin
                        o_word       <= next_bit_buf[31:0];
                        o_word_vld   <= 1'b1;
                        o_frame_done <= 1'b1;
                        next_bit_buf = 0;
                        next_bit_cnt = 0;
                    end
                end

                bit_buf <= next_bit_buf;
                bit_cnt <= next_bit_cnt;

            end else if (flush_active) begin
                // Flush cycle to write the final leftover word
                o_word       <= flush_buf;
                o_word_vld   <= 1'b1;
                o_frame_done <= 1'b1;
                flush_active <= 1'b0;
                bit_buf      <= 64'd0;
                bit_cnt      <= 6'd0;
            end
        end
    end

endmodule
