// =========================================================================
// Модуль: dp_ram
// Описание: Двухпортовое ОЗУ с независимыми тактовыми доменами.
//           При TESTMODE используется эмуляция для Icarus Verilog.
//           В режиме синтеза используется мегафункция altsyncram (Quartus).
// =========================================================================

`timescale 1ns / 1ps

module dp_ram #(
    parameter DATA_WIDTH = 32,                  // Разрядность данных
    parameter ADDR_WIDTH = 10                   // Глубина адресного пространства (2^ADDR_WIDTH слов)
)(
    // Порт A (обычно используется для записи данных АЦП)
    input                       clk_a,
    input   [ADDR_WIDTH-1:0]    addr_a,
    input                       we_a,
    input   [DATA_WIDTH-1:0]    d_a,
    output  [DATA_WIDTH-1:0]    q_a,

    // Порт B (обычно используется для чтения системной шиной)
    input                       clk_b,
    input   [ADDR_WIDTH-1:0]    addr_b,
    input                       we_b,
    input   [DATA_WIDTH-1:0]    d_b,
    output  [DATA_WIDTH-1:0]    q_b
);

`ifdef TESTMODE

    // =========================================================================
    // ЭМУЛЯЦИЯ ДЛЯ ТЕСТИРОВАНИЯ (Icarus Verilog)
    // =========================================================================
    
    localparam RAM_DEPTH = 1 << ADDR_WIDTH;
    reg [DATA_WIDTH-1:0] ram [RAM_DEPTH-1:0];

    reg [DATA_WIDTH-1:0] q_a_reg;
    reg [DATA_WIDTH-1:0] q_b_reg;

    assign q_a = q_a_reg;
    assign q_b = q_b_reg;

    // Инициализация памяти нулями для симулятора
    integer i;
    initial begin
        for (i = 0; i < RAM_DEPTH; i = i + 1) begin
            ram[i] = {DATA_WIDTH{1'b0}};
        end
    end

    // Управление Портом A
    always @(posedge clk_a) begin
        if (we_a) begin
            ram[addr_a] <= d_a;
        end
        q_a_reg <= ram[addr_a];
    end

    // Управление Портом B
    always @(posedge clk_b) begin
        if (we_b) begin
            ram[addr_b] <= d_b;
        end
        q_b_reg <= ram[addr_b];
    end

`else

    // =========================================================================
    // СИНТЕЗИРУЕМЫЙ БЛОК ДЛЯ QUARTUS (Intel FPGA IP)
    // =========================================================================
    
    altsyncram #(
        .address_reg_b              ("CLOCK1"),
        .clock_enable_input_a       ("BYPASS"),
        .clock_enable_input_b       ("BYPASS"),
        .clock_enable_output_a      ("BYPASS"),
        .clock_enable_output_b      ("BYPASS"),
        .indata_reg_b               ("CLOCK1"),
        .intended_device_family     ("Cyclone IV E"),
        .lpm_type                   ("altsyncram"),
        .numwords_a                 (1 << ADDR_WIDTH),
        .numwords_b                 (1 << ADDR_WIDTH),
        .operation_mode             ("BIDIR_DUAL_PORT"),
        .outdata_aclr_a             ("NONE"),
        .outdata_aclr_b             ("NONE"),
        .outdata_reg_a              ("CLOCK0"), // Задержка чтения - 1 такт для соответствия эмулятору
        .outdata_reg_b              ("CLOCK1"), // Задержка чтения - 1 такт для соответствия эмулятору
        .power_up_uninitialized     ("FALSE"),
        .read_during_write_mode_port_a("NEW_DATA_NO_NBE_READ"),
        .read_during_write_mode_port_b("NEW_DATA_NO_NBE_READ"),
        .widthad_a                  (ADDR_WIDTH),
        .widthad_b                  (ADDR_WIDTH),
        .width_a                    (DATA_WIDTH),
        .width_b                    (DATA_WIDTH),
        .wrcontrol_wraddress_reg_b  ("CLOCK1")
    ) altsyncram_component (
        .address_a                  (addr_a),
        .address_b                  (addr_b),
        .clock0                     (clk_a),
        .clock1                     (clk_b),
        .data_a                     (d_a),
        .data_b                     (d_b),
        .wren_a                     (we_a),
        .wren_b                     (we_b),
        .q_a                        (q_a),
        .q_b                        (q_b),
        .aclr0                      (1'b0),
        .aclr1                      (1'b0),
        .addressstall_a             (1'b0),
        .addressstall_b             (1'b0),
        .byteena_a                  (1'b1),
        .byteena_b                  (1'b1),
        .clocken0                   (1'b1),
        .clocken1                   (1'b1),
        .clocken2                   (1'b1),
        .clocken3                   (1'b1),
        .eccstatus                  ()
    );

`endif

endmodule