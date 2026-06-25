// =========================================================================
// Dual Clock FIFO Wrapper with TESTMODE support.
// Emulates Quartus dcfifo for Icarus Verilog, or uses real IP in synthesis.
// =========================================================================

`timescale 1ns / 1ps

module dc_fifo #(
    parameter lpm_width     = 16,
    parameter lpm_numwords  = 256,
    parameter lpm_widthu    = 8,
    parameter lpm_showahead = "OFF"
) (
    input  wire [lpm_width-1:0]  data,
    input  wire                  rdclk,
    input  wire                  rdreq,
    input  wire                  wrclk,
    input  wire                  wrreq,
    input  wire                  aclr,
    output wire [lpm_width-1:0]  q,
    output wire                  rdempty,
    output wire                  rdfull,
    output wire                  wrempty,
    output wire                  wrfull,
    output wire [lpm_widthu-1:0] rdusedw,
    output wire [lpm_widthu-1:0] wrusedw
);

`ifdef TESTMODE

    // ---------------------------------------------------------------------
    // Emulation Mode (used for Icarus Verilog testing)
    // ---------------------------------------------------------------------

    // Memory Array
    reg [lpm_width-1:0] mem [0:lpm_numwords-1];

    // Pointers & Gray Code Synchronization
    reg [lpm_widthu:0] wr_ptr;
    reg [lpm_widthu:0] rd_ptr;

    reg [lpm_widthu:0] wr_ptr_gray;
    reg [lpm_widthu:0] rd_ptr_gray;

    // Synchronizers
    reg [lpm_widthu:0] wr_ptr_gray_rd1, wr_ptr_gray_rd2;
    reg [lpm_widthu:0] rd_ptr_gray_wr1, rd_ptr_gray_wr2;

    // Gray code conversion functions
    function [lpm_widthu:0] bin2gray;
        input [lpm_widthu:0] bin;
        begin
            bin2gray = bin ^ (bin >> 1);
        end
    endfunction

    function [lpm_widthu:0] gray2bin;
        input [lpm_widthu:0] gray;
        reg [lpm_widthu:0] bin;
        integer i;
        begin
            bin[lpm_widthu] = gray[lpm_widthu];
            for (i = lpm_widthu-1; i >= 0; i = i - 1) begin
                bin[i] = bin[i+1] ^ gray[i];
            end
            gray2bin = bin;
        end
    endfunction

    // Write Clock Domain Logic
    always @(posedge wrclk or posedge aclr) begin
        if (aclr) begin
            wr_ptr          <= 0;
            wr_ptr_gray     <= 0;
            rd_ptr_gray_wr1 <= 0;
            rd_ptr_gray_wr2 <= 0;
        end else begin
            if (wrreq && !wrfull) begin
                mem[wr_ptr[lpm_widthu-1:0]] <= data;
                wr_ptr                      <= wr_ptr + 1'b1;
                wr_ptr_gray                 <= bin2gray(wr_ptr + 1'b1);
            end else begin
                wr_ptr_gray                 <= bin2gray(wr_ptr);
            end
            // Synchronization of Read Pointer to Write Domain
            rd_ptr_gray_wr1 <= rd_ptr_gray;
            rd_ptr_gray_wr2 <= rd_ptr_gray_wr1;
        end
    end

    // Read Clock Domain Logic
    always @(posedge rdclk or posedge aclr) begin
        if (aclr) begin
            rd_ptr          <= 0;
            rd_ptr_gray     <= 0;
            wr_ptr_gray_rd1 <= 0;
            wr_ptr_gray_rd2 <= 0;
        end else begin
            if (rdreq && !rdempty) begin
                rd_ptr          <= rd_ptr + 1'b1;
                rd_ptr_gray     <= bin2gray(rd_ptr + 1'b1);
            end else begin
                rd_ptr_gray     <= bin2gray(rd_ptr);
            end
            // Synchronization of Write Pointer to Read Domain
            wr_ptr_gray_rd1 <= wr_ptr_gray;
            wr_ptr_gray_rd2 <= wr_ptr_gray_rd1;
        end
    end

    // Status and Flag Calculations
    wire [lpm_widthu:0] rd_ptr_sync_bin = gray2bin(rd_ptr_gray_wr2);
    wire [lpm_widthu:0] wr_ptr_sync_bin = gray2bin(wr_ptr_gray_rd2);

    wire [lpm_widthu:0] wrusedw_full = wr_ptr - rd_ptr_sync_bin;
    wire [lpm_widthu:0] rdusedw_full = wr_ptr_sync_bin - rd_ptr;

    assign wrusedw = wrusedw_full[lpm_widthu-1:0];
    assign rdusedw = rdusedw_full[lpm_widthu-1:0];

    assign wrfull  = (wrusedw_full >= lpm_numwords);
    assign wrempty = (wrusedw_full == 0);

    assign rdempty = (rdusedw_full == 0);
    assign rdfull  = (rdusedw_full >= lpm_numwords);

    // Output Generation (Normal vs Show-Ahead Mode)
    reg [lpm_width-1:0] q_normal;
    wire [lpm_width-1:0] q_showahead = mem[rd_ptr[lpm_widthu-1:0]];

    always @(posedge rdclk or posedge aclr) begin
        if (aclr) begin
            q_normal <= {lpm_width{1'b0}};
        end else if (rdreq && !rdempty) begin
            q_normal <= mem[rd_ptr[lpm_widthu-1:0]];
        end
    end

    assign q = (lpm_showahead == "ON") ? q_showahead : q_normal;

`else

    // ---------------------------------------------------------------------
    // Hardware Mode (Uses standard Intel Quartus IP)
    // ---------------------------------------------------------------------
    dcfifo #(
        .lpm_width          (lpm_width),
        .lpm_numwords       (lpm_numwords),
        .lpm_widthu         (lpm_widthu),
        .lpm_showahead      (lpm_showahead),
        .use_eab            ("ON"),
        .overflow_checking  ("ON"),
        .underflow_checking ("ON")
    ) dcfifo_component (
        .data    (data),
        .rdclk   (rdclk),
        .rdreq   (rdreq),
        .wrclk   (wrclk),
        .wrreq   (wrreq),
        .aclr    (aclr),
        .q       (q),
        .rdempty (rdempty),
        .rdfull  (rdfull),
        .wrempty (wrempty),
        .wrfull  (wrfull),
        .rdusedw (rdusedw),
        .wrusedw (wrusedw)
    );

`endif

endmodule