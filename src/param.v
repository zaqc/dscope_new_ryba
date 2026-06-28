// =============================================================================
// Модуль: param
// Путь: src/param.v
// Описание: Модуль приема конфигурационных параметров по шине управления,
//           декодирования, буферизации в теневых регистрах для 16 логических каналов
//           и безопасной коммутации на 4 физических тракта с использованием
//           двухбанковой Ping-Pong буферизации.
// =============================================================================

`timescale 1ns / 1ps

// =============================================================================
// Дефайны начальных значений параметров по умолчанию
// =============================================================================
// Группа: ascan (накопление и выдача АЦП)
`define PARAM_DEFAULT_ASCAN_N_SAMPLES   16'd512
`define PARAM_DEFAULT_ASCAN_ACCUM       8'd1
`define PARAM_DEFAULT_ASCAN_ACCUM_TYPE  2'd0
`define PARAM_DEFAULT_ASCAN_DROP_TICKS  16'd0

// Группа: pulse (параметры генератора импульсов)
`define PARAM_DEFAULT_PULSE_CHARGE      16'd100
`define PARAM_DEFAULT_PULSE_TRANSFER    16'd50
`define PARAM_DEFAULT_PULSE_STRIKE      8'd10

// Группа: tune (параметры настройки тракта и ВАРУ)
`define PARAM_DEFAULT_TUNE_START_AMP    11'd500
`define PARAM_DEFAULT_TUNE_AMP_ONE      32'd0
`define PARAM_DEFAULT_TUNE_AMP_TWO      32'd0
`define PARAM_DEFAULT_TUNE_VRC_LEN      16'd0
`define PARAM_DEFAULT_TUNE_DAC_MIN      10'd0
`define PARAM_DEFAULT_TUNE_DAC_MAX      10'd1023
`define PARAM_DEFAULT_TUNE_TUNE_MODE    2'd0
`define PARAM_DEFAULT_TUNE_LOG_OFFSET   10'd0

module param (
    // Системные сигналы (домен управления)
    input  wire        sys_clk,
    input  wire        sys_rst_n,      // Синхронный сброс домена sys_clk (o_sys_rst_n из rst_sync)
    input  wire        i_sys_sync,     // Импульс запуска в sys_clk (o_sys_sync из rst_sync)

    // Домены синхронизации CDC
    input  wire        adc_clk,
    input  wire        adc_rst_n,
    input  wire        i_adc_sync,     // Импульс запуска в adc_clk (o_adc_sync из rst_sync)

    input  wire        hi_clk,
    input  wire        hi_rst_n,
    input  wire        i_hi_sync,      // Импульс запуска в hi_clk (o_hi_sync из rst_sync)

    // Индексы коммутации логических каналов (0..15) на физические тракты (0..3)
    // Задаются в домене sys_clk и фиксируются по i_sys_sync
    input  wire [3:0]  i_sys_ch_sel_ch0,
    input  wire [3:0]  i_sys_ch_sel_ch1,
    input  wire [3:0]  i_sys_ch_sel_ch2,
    input  wire [3:0]  i_sys_ch_sel_ch3,

    // Входной интерфейс команд (домен sys_clk)
    input  wire [31:0] i_cmd_addr,     // [31:24] - ID модуля, [23:16] - ID параметра, [15:12] - логический канал (0..15)
    input  wire [31:0] i_cmd_data,     // Значение параметра
    input  wire        i_cmd_vld,      // Строб валидности команды

    // ==========================================
    // ВЫХОДЫ НА ФИЗИЧЕСКИЕ ИСПОЛНИТЕЛЬНЫЕ ТРАКТЫ
    // ==========================================

    // Физические модули 1: ascan (домен adc_clk, 4 параллельных приемника)
    output reg  [15:0] o_ascan_ch0_n_samples,
    output reg  [7:0]  o_ascan_ch0_accum,
    output reg  [1:0]  o_ascan_ch0_accum_type,
    output reg  [15:0] o_ascan_ch0_drop_ticks,

    output reg  [15:0] o_ascan_ch1_n_samples,
    output reg  [7:0]  o_ascan_ch1_accum,
    output reg  [1:0]  o_ascan_ch1_accum_type,
    output reg  [15:0] o_ascan_ch1_drop_ticks,

    output reg  [15:0] o_ascan_ch2_n_samples,
    output reg  [7:0]  o_ascan_ch2_accum,
    output reg  [1:0]  o_ascan_ch2_accum_type,
    output reg  [15:0] o_ascan_ch2_drop_ticks,

    output reg  [15:0] o_ascan_ch3_n_samples,
    output reg  [7:0]  o_ascan_ch3_accum,
    output reg  [1:0]  o_ascan_ch3_accum_type,
    output reg  [15:0] o_ascan_ch3_drop_ticks,

    // Физические модули 2: pulse (домен hi_clk, 4 параллельных генератора)
    output reg  [15:0] o_pulse_ch0_charge,
    output reg  [15:0] o_pulse_ch0_transfer,
    output reg  [7:0]  o_pulse_ch0_strike,

    output reg  [15:0] o_pulse_ch1_charge,
    output reg  [15:0] o_pulse_ch1_transfer,
    output reg  [7:0]  o_pulse_ch1_strike,

    output reg  [15:0] o_pulse_ch2_charge,
    output reg  [15:0] o_pulse_ch2_transfer,
    output reg  [7:0]  o_pulse_ch2_strike,

    output reg  [15:0] o_pulse_ch3_charge,
    output reg  [15:0] o_pulse_ch3_transfer,
    output reg  [7:0]  o_pulse_ch3_strike,

    // Физические модули 3: tune (домен sys_clk, 4 параллельных тракта усиления)
    output reg  [10:0] o_tune_ch0_start_amp,
    output reg  [31:0] o_tune_ch0_amp_one,
    output reg  [31:0] o_tune_ch0_amp_two,
    output reg  [15:0] o_tune_ch0_vrc_len,
    output reg  [9:0]  o_tune_ch0_dac_min,
    output reg  [9:0]  o_tune_ch0_dac_max,
    output reg  [1:0]  o_tune_ch0_tune_mode,
    output reg  [9:0]  o_tune_ch0_log_offset,

    output reg  [10:0] o_tune_ch1_start_amp,
    output reg  [31:0] o_tune_ch1_amp_one,
    output reg  [31:0] o_tune_ch1_amp_two,
    output reg  [15:0] o_tune_ch1_vrc_len,
    output reg  [9:0]  o_tune_ch1_dac_min,
    output reg  [9:0]  o_tune_ch1_dac_max,
    output reg  [1:0]  o_tune_ch1_tune_mode,
    output reg  [9:0]  o_tune_ch1_log_offset,

    output reg  [10:0] o_tune_ch2_start_amp,
    output reg  [31:0] o_tune_ch2_amp_one,
    output reg  [31:0] o_tune_ch2_amp_two,
    output reg  [15:0] o_tune_ch2_vrc_len,
    output reg  [9:0]  o_tune_ch2_dac_min,
    output reg  [9:0]  o_tune_ch2_dac_max,
    output reg  [1:0]  o_tune_ch2_tune_mode,
    output reg  [9:0]  o_tune_ch2_log_offset,

    output reg  [10:0] o_tune_ch3_start_amp,
    output reg  [31:0] o_tune_ch3_amp_one,
    output reg  [31:0] o_tune_ch3_amp_two,
    output reg  [15:0] o_tune_ch3_vrc_len,
    output reg  [9:0]  o_tune_ch3_dac_min,
    output reg  [9:0]  o_tune_ch3_dac_max,
    output reg  [1:0]  o_tune_ch3_tune_mode,
    output reg  [9:0]  o_tune_ch3_log_offset,

    // ==========================================
    // ИНТЕРФЕЙС ЧТЕНИЯ МЕТАДАННЫХ ДЛЯ ВЫХОДНОГО ПАКЕТА
    // ==========================================
    // Позволяет последовательно вычитать параметры любого из 16 каналов предыдущего измерения
    input  wire [3:0]  i_packet_ch_idx,   // Индекс запрашиваемого логического канала (0..15)

    // Группа ascan (для выбранного i_packet_ch_idx из замороженного банка)
    output reg  [15:0] o_sys_ascan_n_samples,
    output reg  [7:0]  o_sys_ascan_accum,
    output reg  [1:0]  o_sys_ascan_accum_type,
    output reg  [15:0] o_sys_ascan_drop_ticks,

    // Группа pulse (для выбранного i_packet_ch_idx из замороженного банка)
    output reg  [15:0] o_sys_pulse_charge,
    output reg  [15:0] o_sys_pulse_transfer,
    output reg  [7:0]  o_sys_pulse_strike,

    // Группа tune (для выбранного i_packet_ch_idx из замороженного банка)
    output reg  [10:0] o_sys_tune_start_amp,
    output reg  [31:0] o_sys_tune_amp_one,
    output reg  [31:0] o_sys_tune_amp_two,
    output reg  [15:0] o_sys_tune_vrc_len,
    output reg  [9:0]  o_sys_tune_dac_min,
    output reg  [9:0]  o_sys_tune_dac_max,
    output reg  [1:0]  o_sys_tune_tune_mode,
    output reg  [9:0]  o_sys_tune_log_offset
);

    // Вспомогательная переменная для циклов инициализации массивов
    integer i;

    // =========================================================================
    // 1. Теневые регистры (Shadow / Holding Registers) в домене sys_clk (16 логических каналов)
    // =========================================================================
    reg [15:0] hold_ascan_n_samples  [0:15];
    reg [7:0]  hold_ascan_accum      [0:15];
    reg [1:0]  hold_ascan_accum_type [0:15];
    reg [15:0] hold_ascan_drop_ticks [0:15];

    reg [15:0] hold_pulse_charge     [0:15];
    reg [15:0] hold_pulse_transfer   [0:15];
    reg [7:0]  hold_pulse_strike     [0:15];

    reg [10:0] hold_tune_start_amp   [0:15];
    reg [31:0] hold_tune_amp_one     [0:15];
    reg [31:0] hold_tune_amp_two     [0:15];
    reg [15:0] hold_tune_vrc_len     [0:15];
    reg [9:0]  hold_tune_dac_min     [0:15];
    reg [9:0]  hold_tune_dac_max     [0:15];
    reg [1:0]  hold_tune_tune_mode   [0:15];
    reg [9:0]  hold_tune_log_offset  [0:15];

    // Выделение логического канала из адреса команды
    wire [3:0] w_cmd_ch_idx = i_cmd_addr[15:12];

    // Декодирование адреса командной шины и запись в теневые регистры
    always @(posedge sys_clk) begin
        if (!sys_rst_n) begin
            for (i = 0; i < 16; i = i + 1) begin
                hold_ascan_n_samples[i]   <= `PARAM_DEFAULT_ASCAN_N_SAMPLES;
                hold_ascan_accum[i]       <= `PARAM_DEFAULT_ASCAN_ACCUM;
                hold_ascan_accum_type[i]  <= `PARAM_DEFAULT_ASCAN_ACCUM_TYPE;
                hold_ascan_drop_ticks[i]  <= `PARAM_DEFAULT_ASCAN_DROP_TICKS;

                hold_pulse_charge[i]      <= `PARAM_DEFAULT_PULSE_CHARGE;
                hold_pulse_transfer[i]    <= `PARAM_DEFAULT_PULSE_TRANSFER;
                hold_pulse_strike[i]      <= `PARAM_DEFAULT_PULSE_STRIKE;

                hold_tune_start_amp[i]    <= `PARAM_DEFAULT_TUNE_START_AMP;
                hold_tune_amp_one[i]      <= `PARAM_DEFAULT_TUNE_AMP_ONE;
                hold_tune_amp_two[i]      <= `PARAM_DEFAULT_TUNE_AMP_TWO;
                hold_tune_vrc_len[i]      <= `PARAM_DEFAULT_TUNE_VRC_LEN;
                hold_tune_dac_min[i]      <= `PARAM_DEFAULT_TUNE_DAC_MIN;
                hold_tune_dac_max[i]      <= `PARAM_DEFAULT_TUNE_DAC_MAX;
                hold_tune_tune_mode[i]    <= `PARAM_DEFAULT_TUNE_TUNE_MODE;
                hold_tune_log_offset[i]   <= `PARAM_DEFAULT_TUNE_LOG_OFFSET;
            end
        end else if (i_cmd_vld) begin
            case (i_cmd_addr[31:24]) // Выбор модуля
                8'h01: begin // ascan
                    case (i_cmd_addr[23:16])
                        8'h01: hold_ascan_n_samples[w_cmd_ch_idx]  <= i_cmd_data[15:0];
                        8'h02: hold_ascan_accum[w_cmd_ch_idx]      <= i_cmd_data[7:0];
                        8'h03: hold_ascan_accum_type[w_cmd_ch_idx] <= i_cmd_data[1:0];
                        8'h04: hold_ascan_drop_ticks[w_cmd_ch_idx] <= i_cmd_data[15:0];
                        default: ;
                    endcase
                end

                8'h02: begin // pulse
                    case (i_cmd_addr[23:16])
                        8'h01: hold_pulse_charge[w_cmd_ch_idx]     <= i_cmd_data[15:0];
                        8'h02: hold_pulse_transfer[w_cmd_ch_idx]   <= i_cmd_data[15:0];
                        8'h03: hold_pulse_strike[w_cmd_ch_idx]     <= i_cmd_data[7:0];
                        default: ;
                    endcase
                end

                8'h03: begin // tune
                    case (i_cmd_addr[23:16])
                        8'h01: hold_tune_start_amp[w_cmd_ch_idx]   <= i_cmd_data[10:0];
                        8'h02: hold_tune_amp_one[w_cmd_ch_idx]     <= i_cmd_data[31:0];
                        8'h03: hold_tune_amp_two[w_cmd_ch_idx]     <= i_cmd_data[31:0];
                        8'h04: hold_tune_vrc_len[w_cmd_ch_idx]     <= i_cmd_data[15:0];
                        8'h05: begin
                            hold_tune_dac_min[w_cmd_ch_idx]        <= i_cmd_data[9:0];
                            hold_tune_dac_max[w_cmd_ch_idx]        <= i_cmd_data[19:10];
                            hold_tune_tune_mode[w_cmd_ch_idx]      <= i_cmd_data[21:20];
                        end
                        8'h06: hold_tune_log_offset[w_cmd_ch_idx]  <= i_cmd_data[9:0];
                        default: ;
                    endcase
                end

                default: ;
            endcase
        end
    end

    // =========================================================================
    // 2. Двойная буферизация (Ping-Pong) активных регистров (Двумерные массивы [Банк][Канал])
    // =========================================================================
    reg r_wr_ptr; // Указатель записи/активного банка (в домене sys_clk)

    reg [15:0] bank_ascan_n_samples  [0:1][0:15];
    reg [7:0]  bank_ascan_accum      [0:1][0:15];
    reg [1:0]  bank_ascan_accum_type [0:1][0:15];
    reg [15:0] bank_ascan_drop_ticks [0:1][0:15];

    reg [15:0] bank_pulse_charge     [0:1][0:15];
    reg [15:0] bank_pulse_transfer   [0:1][0:15];
    reg [7:0]  bank_pulse_strike     [0:1][0:15];

    reg [10:0] bank_tune_start_amp   [0:1][0:15];
    reg [31:0] bank_tune_amp_one     [0:1][0:15];
    reg [31:0] bank_tune_amp_two     [0:1][0:15];
    reg [15:0] bank_tune_vrc_len     [0:1][0:15];
    reg [9:0]  bank_tune_dac_min     [0:1][0:15];
    reg [9:0]  bank_tune_dac_max     [0:1][0:15];
    reg [1:0]  bank_tune_tune_mode   [0:1][0:15];
    reg [9:0]  bank_tune_log_offset  [0:1][0:15];

    // Фиксированные индексы коммутации логических каналов на физические тракты (sys_clk)
    reg [3:0] r_sys_ch_sel_ch0;
    reg [3:0] r_sys_ch_sel_ch1;
    reg [3:0] r_sys_ch_sel_ch2;
    reg [3:0] r_sys_ch_sel_ch3;

    // Перенос из теневых регистров в целевой банк и фиксация каналов по i_sys_sync
    always @(posedge sys_clk) begin
        if (!sys_rst_n) begin
            r_wr_ptr         <= 1'b0;
            r_sys_ch_sel_ch0 <= 4'd0;
            r_sys_ch_sel_ch1 <= 4'd1;
            r_sys_ch_sel_ch2 <= 4'd2;
            r_sys_ch_sel_ch3 <= 4'd3;

            for (i = 0; i < 16; i = i + 1) begin
                // Банк 0
                bank_ascan_n_samples[0][i]  <= `PARAM_DEFAULT_ASCAN_N_SAMPLES;
                bank_ascan_accum[0][i]      <= `PARAM_DEFAULT_ASCAN_ACCUM;
                bank_ascan_accum_type[0][i] <= `PARAM_DEFAULT_ASCAN_ACCUM_TYPE;
                bank_ascan_drop_ticks[0][i] <= `PARAM_DEFAULT_ASCAN_DROP_TICKS;

                bank_pulse_charge[0][i]     <= `PARAM_DEFAULT_PULSE_CHARGE;
                bank_pulse_transfer[0][i]   <= `PARAM_DEFAULT_PULSE_TRANSFER;
                bank_pulse_strike[0][i]     <= `PARAM_DEFAULT_PULSE_STRIKE;

                bank_tune_start_amp[0][i]   <= `PARAM_DEFAULT_TUNE_START_AMP;
                bank_tune_amp_one[0][i]     <= `PARAM_DEFAULT_TUNE_AMP_ONE;
                bank_tune_amp_two[0][i]     <= `PARAM_DEFAULT_TUNE_AMP_TWO;
                bank_tune_vrc_len[0][i]     <= `PARAM_DEFAULT_TUNE_VRC_LEN;
                bank_tune_dac_min[0][i]     <= `PARAM_DEFAULT_TUNE_DAC_MIN;
                bank_tune_dac_max[0][i]     <= `PARAM_DEFAULT_TUNE_DAC_MAX;
                bank_tune_tune_mode[0][i]   <= `PARAM_DEFAULT_TUNE_TUNE_MODE;
                bank_tune_log_offset[0][i]  <= `PARAM_DEFAULT_TUNE_LOG_OFFSET;

                // Банк 1
                bank_ascan_n_samples[1][i]  <= `PARAM_DEFAULT_ASCAN_N_SAMPLES;
                bank_ascan_accum[1][i]      <= `PARAM_DEFAULT_ASCAN_ACCUM;
                bank_ascan_accum_type[1][i] <= `PARAM_DEFAULT_ASCAN_ACCUM_TYPE;
                bank_ascan_drop_ticks[1][i] <= `PARAM_DEFAULT_ASCAN_DROP_TICKS;

                bank_pulse_charge[1][i]     <= `PARAM_DEFAULT_PULSE_CHARGE;
                bank_pulse_transfer[1][i]   <= `PARAM_DEFAULT_PULSE_TRANSFER;
                bank_pulse_strike[1][i]     <= `PARAM_DEFAULT_PULSE_STRIKE;

                bank_tune_start_amp[1][i]   <= `PARAM_DEFAULT_TUNE_START_AMP;
                bank_tune_amp_one[1][i]     <= `PARAM_DEFAULT_TUNE_AMP_ONE;
                bank_tune_amp_two[1][i]     <= `PARAM_DEFAULT_TUNE_AMP_TWO;
                bank_tune_vrc_len[1][i]     <= `PARAM_DEFAULT_TUNE_VRC_LEN;
                bank_tune_dac_min[1][i]     <= `PARAM_DEFAULT_TUNE_DAC_MIN;
                bank_tune_dac_max[1][i]     <= `PARAM_DEFAULT_TUNE_DAC_MAX;
                bank_tune_tune_mode[1][i]   <= `PARAM_DEFAULT_TUNE_TUNE_MODE;
                bank_tune_log_offset[1][i]  <= `PARAM_DEFAULT_TUNE_LOG_OFFSET;
            end
        end else if (i_sys_sync) begin
            // Инвертирование указателя записи
            r_wr_ptr <= ~r_wr_ptr;

            // Фиксация индексов коммутации логических каналов для текущего кадра
            r_sys_ch_sel_ch0 <= i_sys_ch_sel_ch0;
            r_sys_ch_sel_ch1 <= i_sys_ch_sel_ch1;
            r_sys_ch_sel_ch2 <= i_sys_ch_sel_ch2;
            r_sys_ch_sel_ch3 <= i_sys_ch_sel_ch3;

            // Копируем все параметры из теневых регистров в банк r_wr_ptr (до изменения)
            for (i = 0; i < 16; i = i + 1) begin
                bank_ascan_n_samples[r_wr_ptr][i]  <= hold_ascan_n_samples[i];
                bank_ascan_accum[r_wr_ptr][i]      <= hold_ascan_accum[i];
                bank_ascan_accum_type[r_wr_ptr][i] <= hold_ascan_accum_type[i];
                bank_ascan_drop_ticks[r_wr_ptr][i] <= hold_ascan_drop_ticks[i];

                bank_pulse_charge[r_wr_ptr][i]     <= hold_pulse_charge[i];
                bank_pulse_transfer[r_wr_ptr][i]   <= hold_pulse_transfer[i];
                bank_pulse_strike[r_wr_ptr][i]     <= hold_pulse_strike[i];

                bank_tune_start_amp[r_wr_ptr][i]   <= hold_tune_start_amp[i];
                bank_tune_amp_one[r_wr_ptr][i]     <= hold_tune_amp_one[i];
                bank_tune_amp_two[r_wr_ptr][i]     <= hold_tune_amp_two[i];
                bank_tune_vrc_len[r_wr_ptr][i]     <= hold_tune_vrc_len[i];
                bank_tune_dac_min[r_wr_ptr][i]     <= hold_tune_dac_min[i];
                bank_tune_dac_max[r_wr_ptr][i]     <= hold_tune_dac_max[i];
                bank_tune_tune_mode[r_wr_ptr][i]   <= hold_tune_tune_mode[i];
                bank_tune_log_offset[r_wr_ptr][i]  <= hold_tune_log_offset[i];
            end
        end
    end

    // =========================================================================
    // 3. Формирование селекторов коммутации
    // =========================================================================
    // Т.к. r_wr_ptr инвертируется по i_sys_sync, то после спада i_sys_sync:
    // - Текущий (запущенный) банк исполнения - это банк, куда мы только что записали.
    //   Он соответствует старому значению r_wr_ptr, т.е. ~r_wr_ptr.
    // - Предыдущий ("замороженный") банк метаданных для выдачи в пакет результатов -
    //   это стабильный старый банк, который сейчас равен новому значению r_wr_ptr.
    wire active_sel = ~r_wr_ptr; // Селектор для активного исполнения
    wire sys_sel    = r_wr_ptr;  // Селектор для "замороженного" пакета результатов

    // =========================================================================
    // 4. Физический исполнительный тракт 3: tune (домен sys_clk, 4 канала)
    // =========================================================================
    always @(posedge sys_clk) begin
        if (!sys_rst_n) begin
            // Сброс канала 0
            o_tune_ch0_start_amp  <= `PARAM_DEFAULT_TUNE_START_AMP;
            o_tune_ch0_amp_one    <= `PARAM_DEFAULT_TUNE_AMP_ONE;
            o_tune_ch0_amp_two    <= `PARAM_DEFAULT_TUNE_AMP_TWO;
            o_tune_ch0_vrc_len    <= `PARAM_DEFAULT_TUNE_VRC_LEN;
            o_tune_ch0_dac_min    <= `PARAM_DEFAULT_TUNE_DAC_MIN;
            o_tune_ch0_dac_max    <= `PARAM_DEFAULT_TUNE_DAC_MAX;
            o_tune_ch0_tune_mode  <= `PARAM_DEFAULT_TUNE_TUNE_MODE;
            o_tune_ch0_log_offset <= `PARAM_DEFAULT_TUNE_LOG_OFFSET;
            // Сброс канала 1
            o_tune_ch1_start_amp  <= `PARAM_DEFAULT_TUNE_START_AMP;
            o_tune_ch1_amp_one    <= `PARAM_DEFAULT_TUNE_AMP_ONE;
            o_tune_ch1_amp_two    <= `PARAM_DEFAULT_TUNE_AMP_TWO;
            o_tune_ch1_vrc_len    <= `PARAM_DEFAULT_TUNE_VRC_LEN;
            o_tune_ch1_dac_min    <= `PARAM_DEFAULT_TUNE_DAC_MIN;
            o_tune_ch1_dac_max    <= `PARAM_DEFAULT_TUNE_DAC_MAX;
            o_tune_ch1_tune_mode  <= `PARAM_DEFAULT_TUNE_TUNE_MODE;
            o_tune_ch1_log_offset <= `PARAM_DEFAULT_TUNE_LOG_OFFSET;
            // Сброс канала 2
            o_tune_ch2_start_amp  <= `PARAM_DEFAULT_TUNE_START_AMP;
            o_tune_ch2_amp_one    <= `PARAM_DEFAULT_TUNE_AMP_ONE;
            o_tune_ch2_amp_two    <= `PARAM_DEFAULT_TUNE_AMP_TWO;
            o_tune_ch2_vrc_len    <= `PARAM_DEFAULT_TUNE_VRC_LEN;
            o_tune_ch2_dac_min    <= `PARAM_DEFAULT_TUNE_DAC_MIN;
            o_tune_ch2_dac_max    <= `PARAM_DEFAULT_TUNE_DAC_MAX;
            o_tune_ch2_tune_mode  <= `PARAM_DEFAULT_TUNE_TUNE_MODE;
            o_tune_ch2_log_offset <= `PARAM_DEFAULT_TUNE_LOG_OFFSET;
            // Сброс канала 3
            o_tune_ch3_start_amp  <= `PARAM_DEFAULT_TUNE_START_AMP;
            o_tune_ch3_amp_one    <= `PARAM_DEFAULT_TUNE_AMP_ONE;
            o_tune_ch3_amp_two    <= `PARAM_DEFAULT_TUNE_AMP_TWO;
            o_tune_ch3_vrc_len    <= `PARAM_DEFAULT_TUNE_VRC_LEN;
            o_tune_ch3_dac_min    <= `PARAM_DEFAULT_TUNE_DAC_MIN;
            o_tune_ch3_dac_max    <= `PARAM_DEFAULT_TUNE_DAC_MAX;
            o_tune_ch3_tune_mode  <= `PARAM_DEFAULT_TUNE_TUNE_MODE;
            o_tune_ch3_log_offset <= `PARAM_DEFAULT_TUNE_LOG_OFFSET;
        end else begin
            // Тракт ch0
            o_tune_ch0_start_amp  <= bank_tune_start_amp [active_sel][r_sys_ch_sel_ch0];
            o_tune_ch0_amp_one    <= bank_tune_amp_one   [active_sel][r_sys_ch_sel_ch0];
            o_tune_ch0_amp_two    <= bank_tune_amp_two   [active_sel][r_sys_ch_sel_ch0];
            o_tune_ch0_vrc_len    <= bank_tune_vrc_len   [active_sel][r_sys_ch_sel_ch0];
            o_tune_ch0_dac_min    <= bank_tune_dac_min   [active_sel][r_sys_ch_sel_ch0];
            o_tune_ch0_dac_max    <= bank_tune_dac_max   [active_sel][r_sys_ch_sel_ch0];
            o_tune_ch0_tune_mode  <= bank_tune_tune_mode [active_sel][r_sys_ch_sel_ch0];
            o_tune_ch0_log_offset <= bank_tune_log_offset[active_sel][r_sys_ch_sel_ch0];
            // Тракт ch1
            o_tune_ch1_start_amp  <= bank_tune_start_amp [active_sel][r_sys_ch_sel_ch1];
            o_tune_ch1_amp_one    <= bank_tune_amp_one   [active_sel][r_sys_ch_sel_ch1];
            o_tune_ch1_amp_two    <= bank_tune_amp_two   [active_sel][r_sys_ch_sel_ch1];
            o_tune_ch1_vrc_len    <= bank_tune_vrc_len   [active_sel][r_sys_ch_sel_ch1];
            o_tune_ch1_dac_min    <= bank_tune_dac_min   [active_sel][r_sys_ch_sel_ch1];
            o_tune_ch1_dac_max    <= bank_tune_dac_max   [active_sel][r_sys_ch_sel_ch1];
            o_tune_ch1_tune_mode  <= bank_tune_tune_mode [active_sel][r_sys_ch_sel_ch1];
            o_tune_ch1_log_offset <= bank_tune_log_offset[active_sel][r_sys_ch_sel_ch1];
            // Тракт ch2
            o_tune_ch2_start_amp  <= bank_tune_start_amp [active_sel][r_sys_ch_sel_ch2];
            o_tune_ch2_amp_one    <= bank_tune_amp_one   [active_sel][r_sys_ch_sel_ch2];
            o_tune_ch2_amp_two    <= bank_tune_amp_two   [active_sel][r_sys_ch_sel_ch2];
            o_tune_ch2_vrc_len    <= bank_tune_vrc_len   [active_sel][r_sys_ch_sel_ch2];
            o_tune_ch2_dac_min    <= bank_tune_dac_min   [active_sel][r_sys_ch_sel_ch2];
            o_tune_ch2_dac_max    <= bank_tune_dac_max   [active_sel][r_sys_ch_sel_ch2];
            o_tune_ch2_tune_mode  <= bank_tune_tune_mode [active_sel][r_sys_ch_sel_ch2];
            o_tune_ch2_log_offset <= bank_tune_log_offset[active_sel][r_sys_ch_sel_ch2];
            // Тракт ch3
            o_tune_ch3_start_amp  <= bank_tune_start_amp [active_sel][r_sys_ch_sel_ch3];
            o_tune_ch3_amp_one    <= bank_tune_amp_one   [active_sel][r_sys_ch_sel_ch3];
            o_tune_ch3_amp_two    <= bank_tune_amp_two   [active_sel][r_sys_ch_sel_ch3];
            o_tune_ch3_vrc_len    <= bank_tune_vrc_len   [active_sel][r_sys_ch_sel_ch3];
            o_tune_ch3_dac_min    <= bank_tune_dac_min   [active_sel][r_sys_ch_sel_ch3];
            o_tune_ch3_dac_max    <= bank_tune_dac_max   [active_sel][r_sys_ch_sel_ch3];
            o_tune_ch3_tune_mode  <= bank_tune_tune_mode [active_sel][r_sys_ch_sel_ch3];
            o_tune_ch3_log_offset <= bank_tune_log_offset[active_sel][r_sys_ch_sel_ch3];
        end
    end

    // =========================================================================
    // 5. Физический исполнительный тракт 1: ascan (CDC, домен adc_clk, 4 канала)
    // =========================================================================
    always @(posedge adc_clk) begin
        if (!adc_rst_n) begin
            // Сброс канала 0
            o_ascan_ch0_n_samples   <= `PARAM_DEFAULT_ASCAN_N_SAMPLES;
            o_ascan_ch0_accum       <= `PARAM_DEFAULT_ASCAN_ACCUM;
            o_ascan_ch0_accum_type  <= `PARAM_DEFAULT_ASCAN_ACCUM_TYPE;
            o_ascan_ch0_drop_ticks  <= `PARAM_DEFAULT_ASCAN_DROP_TICKS;
            // Сброс канала 1
            o_ascan_ch1_n_samples   <= `PARAM_DEFAULT_ASCAN_N_SAMPLES;
            o_ascan_ch1_accum       <= `PARAM_DEFAULT_ASCAN_ACCUM;
            o_ascan_ch1_accum_type  <= `PARAM_DEFAULT_ASCAN_ACCUM_TYPE;
            o_ascan_ch1_drop_ticks  <= `PARAM_DEFAULT_ASCAN_DROP_TICKS;
            // Сброс канала 2
            o_ascan_ch2_n_samples   <= `PARAM_DEFAULT_ASCAN_N_SAMPLES;
            o_ascan_ch2_accum       <= `PARAM_DEFAULT_ASCAN_ACCUM;
            o_ascan_ch2_accum_type  <= `PARAM_DEFAULT_ASCAN_ACCUM_TYPE;
            o_ascan_ch2_drop_ticks  <= `PARAM_DEFAULT_ASCAN_DROP_TICKS;
            // Сброс канала 3
            o_ascan_ch3_n_samples   <= `PARAM_DEFAULT_ASCAN_N_SAMPLES;
            o_ascan_ch3_accum       <= `PARAM_DEFAULT_ASCAN_ACCUM;
            o_ascan_ch3_accum_type  <= `PARAM_DEFAULT_ASCAN_ACCUM_TYPE;
            o_ascan_ch3_drop_ticks  <= `PARAM_DEFAULT_ASCAN_DROP_TICKS;
        end else if (i_adc_sync) begin
            // Безопасное защелкивание стабильных квазистатических данных из sys_clk домена
            // Тракт ch0
            o_ascan_ch0_n_samples   <= bank_ascan_n_samples [active_sel][r_sys_ch_sel_ch0];
            o_ascan_ch0_accum       <= bank_ascan_accum     [active_sel][r_sys_ch_sel_ch0];
            o_ascan_ch0_accum_type  <= bank_ascan_accum_type[active_sel][r_sys_ch_sel_ch0];
            o_ascan_ch0_drop_ticks  <= bank_ascan_drop_ticks[active_sel][r_sys_ch_sel_ch0];
            // Тракт ch1
            o_ascan_ch1_n_samples   <= bank_ascan_n_samples [active_sel][r_sys_ch_sel_ch1];
            o_ascan_ch1_accum       <= bank_ascan_accum     [active_sel][r_sys_ch_sel_ch1];
            o_ascan_ch1_accum_type  <= bank_ascan_accum_type[active_sel][r_sys_ch_sel_ch1];
            o_ascan_ch1_drop_ticks  <= bank_ascan_drop_ticks[active_sel][r_sys_ch_sel_ch1];
            // Тракт ch2
            o_ascan_ch2_n_samples   <= bank_ascan_n_samples [active_sel][r_sys_ch_sel_ch2];
            o_ascan_ch2_accum       <= bank_ascan_accum     [active_sel][r_sys_ch_sel_ch2];
            o_ascan_ch2_accum_type  <= bank_ascan_accum_type[active_sel][r_sys_ch_sel_ch2];
            o_ascan_ch2_drop_ticks  <= bank_ascan_drop_ticks[active_sel][r_sys_ch_sel_ch2];
            // Тракт ch3
            o_ascan_ch3_n_samples   <= bank_ascan_n_samples [active_sel][r_sys_ch_sel_ch3];
            o_ascan_ch3_accum       <= bank_ascan_accum     [active_sel][r_sys_ch_sel_ch3];
            o_ascan_ch3_accum_type  <= bank_ascan_accum_type[active_sel][r_sys_ch_sel_ch3];
            o_ascan_ch3_drop_ticks  <= bank_ascan_drop_ticks[active_sel][r_sys_ch_sel_ch3];
        end
    end

    // =========================================================================
    // 6. Физический исполнительный тракт 2: pulse (CDC, домен hi_clk, 4 канала)
    // =========================================================================
    always @(posedge hi_clk) begin
        if (!hi_rst_n) begin
            // Сброс канала 0
            o_pulse_ch0_charge    <= `PARAM_DEFAULT_PULSE_CHARGE;
            o_pulse_ch0_transfer  <= `PARAM_DEFAULT_PULSE_TRANSFER;
            o_pulse_ch0_strike    <= `PARAM_DEFAULT_PULSE_STRIKE;
            // Сброс канала 1
            o_pulse_ch1_charge    <= `PARAM_DEFAULT_PULSE_CHARGE;
            o_pulse_ch1_transfer  <= `PARAM_DEFAULT_PULSE_TRANSFER;
            o_pulse_ch1_strike    <= `PARAM_DEFAULT_PULSE_STRIKE;
            // Сброс канала 2
            o_pulse_ch2_charge    <= `PARAM_DEFAULT_PULSE_CHARGE;
            o_pulse_ch2_transfer  <= `PARAM_DEFAULT_PULSE_TRANSFER;
            o_pulse_ch2_strike    <= `PARAM_DEFAULT_PULSE_STRIKE;
            // Сброс канала 3
            o_pulse_ch3_charge    <= `PARAM_DEFAULT_PULSE_CHARGE;
            o_pulse_ch3_transfer  <= `PARAM_DEFAULT_PULSE_TRANSFER;
            o_pulse_ch3_strike    <= `PARAM_DEFAULT_PULSE_STRIKE;
        end else if (i_hi_sync) begin
            // Безопасное защелкивание стабильных квазистатических данных из sys_clk домена
            // Тракт ch0
            o_pulse_ch0_charge    <= bank_pulse_charge   [active_sel][r_sys_ch_sel_ch0];
            o_pulse_ch0_transfer  <= bank_pulse_transfer [active_sel][r_sys_ch_sel_ch0];
            o_pulse_ch0_strike    <= bank_pulse_strike   [active_sel][r_sys_ch_sel_ch0];
            // Тракт ch1
            o_pulse_ch1_charge    <= bank_pulse_charge   [active_sel][r_sys_ch_sel_ch1];
            o_pulse_ch1_transfer  <= bank_pulse_transfer [active_sel][r_sys_ch_sel_ch1];
            o_pulse_ch1_strike    <= bank_pulse_strike   [active_sel][r_sys_ch_sel_ch1];
            // Тракт ch2
            o_pulse_ch2_charge    <= bank_pulse_charge   [active_sel][r_sys_ch_sel_ch2];
            o_pulse_ch2_transfer  <= bank_pulse_transfer [active_sel][r_sys_ch_sel_ch2];
            o_pulse_ch2_strike    <= bank_pulse_strike   [active_sel][r_sys_ch_sel_ch2];
            // Тракт ch3
            o_pulse_ch3_charge    <= bank_pulse_charge   [active_sel][r_sys_ch_sel_ch3];
            o_pulse_ch3_transfer  <= bank_pulse_transfer [active_sel][r_sys_ch_sel_ch3];
            o_pulse_ch3_strike    <= bank_pulse_strike   [active_sel][r_sys_ch_sel_ch3];
        end
    end

    // =========================================================================
    // 7. Порты пакетного вывода (Домен sys_clk, чтение из "замороженного" банка)
    // =========================================================================
    always @(posedge sys_clk) begin
        if (!sys_rst_n) begin
            o_sys_ascan_n_samples  <= `PARAM_DEFAULT_ASCAN_N_SAMPLES;
            o_sys_ascan_accum      <= `PARAM_DEFAULT_ASCAN_ACCUM;
            o_sys_ascan_accum_type <= `PARAM_DEFAULT_ASCAN_ACCUM_TYPE;
            o_sys_ascan_drop_ticks <= `PARAM_DEFAULT_ASCAN_DROP_TICKS;

            o_sys_pulse_charge     <= `PARAM_DEFAULT_PULSE_CHARGE;
            o_sys_pulse_transfer   <= `PARAM_DEFAULT_PULSE_TRANSFER;
            o_sys_pulse_strike     <= `PARAM_DEFAULT_PULSE_STRIKE;

            o_sys_tune_start_amp   <= `PARAM_DEFAULT_TUNE_START_AMP;
            o_sys_tune_amp_one     <= `PARAM_DEFAULT_TUNE_AMP_ONE;
            o_sys_tune_amp_two     <= `PARAM_DEFAULT_TUNE_AMP_TWO;
            o_sys_tune_vrc_len     <= `PARAM_DEFAULT_TUNE_VRC_LEN;
            o_sys_tune_dac_min     <= `PARAM_DEFAULT_TUNE_DAC_MIN;
            o_sys_tune_dac_max     <= `PARAM_DEFAULT_TUNE_DAC_MAX;
            o_sys_tune_tune_mode   <= `PARAM_DEFAULT_TUNE_TUNE_MODE;
            o_sys_tune_log_offset  <= `PARAM_DEFAULT_TUNE_LOG_OFFSET;
        end else begin
            // Чтение параметров логического канала i_packet_ch_idx из стабильного ("замороженного") банка sys_sel
            o_sys_ascan_n_samples  <= bank_ascan_n_samples [sys_sel][i_packet_ch_idx];
            o_sys_ascan_accum      <= bank_ascan_accum     [sys_sel][i_packet_ch_idx];
            o_sys_ascan_accum_type <= bank_ascan_accum_type[sys_sel][i_packet_ch_idx];
            o_sys_ascan_drop_ticks <= bank_ascan_drop_ticks[sys_sel][i_packet_ch_idx];

            o_sys_pulse_charge     <= bank_pulse_charge    [sys_sel][i_packet_ch_idx];
            o_sys_pulse_transfer   <= bank_pulse_transfer  [sys_sel][i_packet_ch_idx];
            o_sys_pulse_strike     <= bank_pulse_strike    [sys_sel][i_packet_ch_idx];

            o_sys_tune_start_amp   <= bank_tune_start_amp  [sys_sel][i_packet_ch_idx];
            o_sys_tune_amp_one     <= bank_tune_amp_one    [sys_sel][i_packet_ch_idx];
            o_sys_tune_amp_two     <= bank_tune_amp_two    [sys_sel][i_packet_ch_idx];
            o_sys_tune_vrc_len     <= bank_tune_vrc_len    [sys_sel][i_packet_ch_idx];
            o_sys_tune_dac_min     <= bank_tune_dac_min    [sys_sel][i_packet_ch_idx];
            o_sys_tune_dac_max     <= bank_tune_dac_max    [sys_sel][i_packet_ch_idx];
            o_sys_tune_tune_mode   <= bank_tune_tune_mode  [sys_sel][i_packet_ch_idx];
            o_sys_tune_log_offset  <= bank_tune_log_offset [sys_sel][i_packet_ch_idx];
        end
    end

endmodule