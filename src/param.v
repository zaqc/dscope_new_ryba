// =============================================================================
// Модуль: param
// Путь: src/param.v
// Описание: Модуль приема конфигурационных параметров по шине управления,
//           декодирования, буферизации в теневых регистрах для сетки 4 физ. канала
//           на 4 виртуальных цикла, и безопасной коммутации с использованием
//           двухбанковой Ping-Pong буферизации на плоских (1D) массивах.
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
`define PARAM_DEFAULT_ASCAN_PEP_IDX     2'd0

// Группа: pulse (параметры генератора импульсов)
`define PARAM_DEFAULT_PULSE_CHARGE      16'd100
`define PARAM_DEFAULT_PULSE_TRANSFER    16'd50
`define PARAM_DEFAULT_PULSE_STRIKE      8'd10
`define PARAM_DEFAULT_PULSE_GEN_MASK    4'd0

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
    input  wire        i_sys_sync,     // Импульс запуска в sys_clk (o_sys_sync из rst_sync, конец общего цикла)

    // Домены синхронизации CDC
    input  wire        adc_clk,
    input  wire        adc_rst_n,
    input  wire        i_adc_sync,     // Импульс запуска в adc_clk (o_adc_sync из rst_sync)

    input  wire        hi_clk,
    input  wire        hi_rst_n,
    input  wire        i_hi_sync,      // Импульс запуска в hi_clk (o_hi_sync из rst_sync)

    // Номер текущего виртуального цикла/субзапуска (0..3), выполняемого в данный момент.
    // Изменяется динамически в измерительном цикле, коммутируя параметры физических каналов на лету.
    input  wire [1:0]  i_sys_vch_sel,

    // Входной интерфейс команд (домен sys_clk)
    // Поля адреса i_cmd_addr: 
    // [31:24] - ID модуля, [23:16] - ID параметра, [15:14] - физ. канал (0..3), [13:12] - виртуальный цикл (0..3)
    input  wire [31:0] i_cmd_addr,     
    input  wire [31:0] i_cmd_data,     // Значение параметра
    input  wire        i_cmd_vld,      // Строб валидности команды

    // ==========================================
    // ВЫХОДЫ НА ФИЗИЧЕСКИЕ ИСПОЛНИТЕЛЬНЫЕ ТРАКТЫ
    // (Реагируют на динамическое изменение i_sys_vch_sel)
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

    // Физические коммутаторы приемников (домен sys_clk, управление аналоговыми ключами ПЭП 1:4)
    output reg  [1:0]  o_ascan_ch0_pep_idx,     // Выбор ПЭП для приемника ch0
    output reg  [1:0]  o_ascan_ch1_pep_idx,     // Выбор ПЭП для приемника ch1
    output reg  [1:0]  o_ascan_ch2_pep_idx,     // Выбор ПЭП для приемника ch2
    output reg  [1:0]  o_ascan_ch3_pep_idx,     // Выбор ПЭП для приемника ch3

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

    // Физические коммутаторы генераторов (домен sys_clk, маскирование выходов запуска)
    output reg  [3:0]  o_pulse_ch0_gen_mask,    // Маска подключения генераторов для ch0
    output reg  [3:0]  o_pulse_ch1_gen_mask,    // Маска подключения генераторов для ch1
    output reg  [3:0]  o_pulse_ch2_gen_mask,    // Маска подключения генераторов для ch2
    output reg  [3:0]  o_pulse_ch3_gen_mask,    // Маска подключения генераторов для ch3

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
    // Двухкомпонентный адрес чтения сохраненного банка предыдущего измерения
    input  wire [1:0]  i_packet_phy_ch,  // Запрашиваемый физический канал (0..3)
    input  wire [1:0]  i_packet_vch,     // Запрашиваемый виртуальный цикл (0..3)

    // Группа ascan (для выбранных координат из замороженного банка)
    output reg  [15:0] o_sys_ascan_n_samples,
    output reg  [7:0]  o_sys_ascan_accum,
    output reg  [1:0]  o_sys_ascan_accum_type,
    output reg  [15:0] o_sys_ascan_drop_ticks,
    output reg  [1:0]  o_sys_ascan_pep_idx,     // Вычитанный индекс коммутатора ПЭП

    // Группа pulse (для выбранных координат из замороженного банка)
    output reg  [15:0] o_sys_pulse_charge,
    output reg  [15:0] o_sys_pulse_transfer,
    output reg  [7:0]  o_sys_pulse_strike,
    output reg  [3:0]  o_sys_pulse_gen_mask,    // Вычитанная маска генераторов

    // Группа tune (для выбранных координат из замороженного банка)
    output reg  [10:0] o_sys_tune_start_amp,
    output reg  [31:0] o_sys_tune_amp_one,
    output reg  [31:0] o_sys_tune_amp_two,
    output reg  [15:0] o_sys_tune_vrc_len,
    output reg  [9:0]  o_sys_tune_dac_min,
    output reg  [9:0]  o_sys_tune_dac_max,
    output reg  [1:0]  o_sys_tune_tune_mode,
    output reg  [9:0]  o_sys_tune_log_offset
);

    // Вспомогательная переменная для линейных циклов
    integer i;

    // =========================================================================
    // 1. Теневые регистры (Shadow / Holding Registers) в домене sys_clk [0:15]
    //    Адресация: {Физический_Канал[1:0], Виртуальный_Цикл[1:0]}
    // =========================================================================
    reg [15:0] hold_ascan_n_samples  [0:15];
    reg [7:0]  hold_ascan_accum      [0:15];
    reg [1:0]  hold_ascan_accum_type [0:15];
    reg [15:0] hold_ascan_drop_ticks [0:15];
    reg [1:0]  hold_ascan_pep_idx    [0:15];

    reg [15:0] hold_pulse_charge     [0:15];
    reg [15:0] hold_pulse_transfer   [0:15];
    reg [7:0]  hold_pulse_strike     [0:15];
    reg [3:0]  hold_pulse_gen_mask   [0:15];

    reg [10:0] hold_tune_start_amp   [0:15];
    reg [31:0] hold_tune_amp_one     [0:15];
    reg [31:0] hold_tune_amp_two     [0:15];
    reg [15:0] hold_tune_vrc_len     [0:15];
    reg [9:0]  hold_tune_dac_min     [0:15];
    reg [9:0]  hold_tune_dac_max     [0:15];
    reg [1:0]  hold_tune_tune_mode   [0:15];
    reg [9:0]  hold_tune_log_offset  [0:15];

    // Вычисление одномерного адреса для записи команды
    // hold_idx = {phy_ch[1:0], vch[1:0]} -> i_cmd_addr[15:12]
    wire [3:0] w_cmd_idx = i_cmd_addr[15:12];

    // Декодирование адреса командной шины и запись в теневые регистры
    always @(posedge sys_clk) begin
        if (!sys_rst_n) begin
            for (i = 0; i < 16; i = i + 1) begin
                hold_ascan_n_samples[i]   <= `PARAM_DEFAULT_ASCAN_N_SAMPLES;
                hold_ascan_accum[i]       <= `PARAM_DEFAULT_ASCAN_ACCUM;
                hold_ascan_accum_type[i]  <= `PARAM_DEFAULT_ASCAN_ACCUM_TYPE;
                hold_ascan_drop_ticks[i]  <= `PARAM_DEFAULT_ASCAN_DROP_TICKS;
                hold_ascan_pep_idx[i]     <= `PARAM_DEFAULT_ASCAN_PEP_IDX;

                hold_pulse_charge[i]      <= `PARAM_DEFAULT_PULSE_CHARGE;
                hold_pulse_transfer[i]    <= `PARAM_DEFAULT_PULSE_TRANSFER;
                hold_pulse_strike[i]      <= `PARAM_DEFAULT_PULSE_STRIKE;
                hold_pulse_gen_mask[i]    <= `PARAM_DEFAULT_PULSE_GEN_MASK;

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
                        8'h01: hold_ascan_n_samples[w_cmd_idx]  <= i_cmd_data[15:0];
                        8'h02: hold_ascan_accum[w_cmd_idx]      <= i_cmd_data[7:0];
                        8'h03: hold_ascan_accum_type[w_cmd_idx] <= i_cmd_data[1:0];
                        8'h04: hold_ascan_drop_ticks[w_cmd_idx] <= i_cmd_data[15:0];
                        8'h05: hold_ascan_pep_idx[w_cmd_idx]    <= i_cmd_data[1:0];
                        default: ;
                    endcase
                end

                8'h02: begin // pulse
                    case (i_cmd_addr[23:16])
                        8'h01: hold_pulse_charge[w_cmd_idx]     <= i_cmd_data[15:0];
                        8'h02: hold_pulse_transfer[w_cmd_idx]   <= i_cmd_data[15:0];
                        8'h03: hold_pulse_strike[w_cmd_idx]     <= i_cmd_data[7:0];
                        8'h04: hold_pulse_gen_mask[w_cmd_idx]   <= i_cmd_data[3:0];
                        default: ;
                    endcase
                end

                8'h03: begin // tune
                    case (i_cmd_addr[23:16])
                        8'h01: hold_tune_start_amp[w_cmd_idx]   <= i_cmd_data[10:0];
                        8'h02: hold_tune_amp_one[w_cmd_idx]     <= i_cmd_data[31:0];
                        8'h03: hold_tune_amp_two[w_cmd_idx]     <= i_cmd_data[31:0];
                        8'h04: hold_tune_vrc_len[w_cmd_idx]     <= i_cmd_data[15:0];
                        8'h05: begin
                            hold_tune_dac_min[w_cmd_idx]        <= i_cmd_data[9:0];
                            hold_tune_dac_max[w_cmd_idx]        <= i_cmd_data[19:10];
                            hold_tune_tune_mode[w_cmd_idx]      <= i_cmd_data[21:20];
                        end
                        8'h06: hold_tune_log_offset[w_cmd_idx]  <= i_cmd_data[9:0];
                        default: ;
                    endcase
                end

                default: ;
            endcase
        end
    end

    // =========================================================================
    // 2. Двойная буферизация (Ping-Pong) активных регистров [0:31]
    //    Адресация: {Указатель_Банка[0], Физический_Канал[1:0], Виртуальный_Цикл[1:0]}
    // =========================================================================
    reg r_wr_ptr; // Указатель записи/активного банка (в домене sys_clk)

    reg [15:0] bank_ascan_n_samples  [0:31];
    reg [7:0]  bank_ascan_accum      [0:31];
    reg [1:0]  bank_ascan_accum_type [0:31];
    reg [15:0] bank_ascan_drop_ticks [0:31];
    reg [1:0]  bank_ascan_pep_idx    [0:31];

    reg [15:0] bank_pulse_charge     [0:31];
    reg [15:0] bank_pulse_transfer   [0:31];
    reg [7:0]  bank_pulse_strike     [0:31];
    reg [3:0]  bank_pulse_gen_mask   [0:31];

    reg [10:0] bank_tune_start_amp   [0:31];
    reg [31:0] bank_tune_amp_one     [0:31];
    reg [31:0] bank_tune_amp_two     [0:31];
    reg [15:0] bank_tune_vrc_len     [0:31];
    reg [9:0]  bank_tune_dac_min     [0:31];
    reg [9:0]  bank_tune_dac_max     [0:31];
    reg [1:0]  bank_tune_tune_mode   [0:31];
    reg [9:0]  bank_tune_log_offset  [0:31];

    // Копирование параметров из теневых регистров в целевой рабочий банк по i_sys_sync
    always @(posedge sys_clk) begin
        if (!sys_rst_n) begin
            r_wr_ptr <= 1'b0;
            for (i = 0; i < 32; i = i + 1) begin
                bank_ascan_n_samples[i]  <= `PARAM_DEFAULT_ASCAN_N_SAMPLES;
                bank_ascan_accum[i]      <= `PARAM_DEFAULT_ASCAN_ACCUM;
                bank_ascan_accum_type[i] <= `PARAM_DEFAULT_ASCAN_ACCUM_TYPE;
                bank_ascan_drop_ticks[i] <= `PARAM_DEFAULT_ASCAN_DROP_TICKS;
                bank_ascan_pep_idx[i]    <= `PARAM_DEFAULT_ASCAN_PEP_IDX;

                bank_pulse_charge[i]     <= `PARAM_DEFAULT_PULSE_CHARGE;
                bank_pulse_transfer[i]   <= `PARAM_DEFAULT_PULSE_TRANSFER;
                bank_pulse_strike[i]     <= `PARAM_DEFAULT_PULSE_STRIKE;
                bank_pulse_gen_mask[i]   <= `PARAM_DEFAULT_PULSE_GEN_MASK;

                bank_tune_start_amp[i]   <= `PARAM_DEFAULT_TUNE_START_AMP;
                bank_tune_amp_one[i]     <= `PARAM_DEFAULT_TUNE_AMP_ONE;
                bank_tune_amp_two[i]     <= `PARAM_DEFAULT_TUNE_AMP_TWO;
                bank_tune_vrc_len[i]     <= `PARAM_DEFAULT_TUNE_VRC_LEN;
                bank_tune_dac_min[i]     <= `PARAM_DEFAULT_TUNE_DAC_MIN;
                bank_tune_dac_max[i]     <= `PARAM_DEFAULT_TUNE_DAC_MAX;
                bank_tune_tune_mode[i]   <= `PARAM_DEFAULT_TUNE_TUNE_MODE;
                bank_tune_log_offset[i]  <= `PARAM_DEFAULT_TUNE_LOG_OFFSET;
            end
        end else if (i_sys_sync) begin
            // Инвертирование указателя записи для следующего такта измерения
            r_wr_ptr <= ~r_wr_ptr;

            // Копируем все параметры из теневых регистров в банк r_wr_ptr (до изменения указателя)
            // bank_idx = {r_wr_ptr, i[3:0]}
            for (i = 0; i < 16; i = i + 1) begin
                bank_ascan_n_samples[{r_wr_ptr, i[3:0]}]  <= hold_ascan_n_samples[i];
                bank_ascan_accum[{r_wr_ptr, i[3:0]}]      <= hold_ascan_accum[i];
                bank_ascan_accum_type[{r_wr_ptr, i[3:0]}] <= hold_ascan_accum_type[i];
                bank_ascan_drop_ticks[{r_wr_ptr, i[3:0]}] <= hold_ascan_drop_ticks[i];
                bank_ascan_pep_idx[{r_wr_ptr, i[3:0]}]    <= hold_ascan_pep_idx[i];

                bank_pulse_charge[{r_wr_ptr, i[3:0]}]     <= hold_pulse_charge[i];
                bank_pulse_transfer[{r_wr_ptr, i[3:0]}]   <= hold_pulse_transfer[i];
                bank_pulse_strike[{r_wr_ptr, i[3:0]}]     <= hold_pulse_strike[i];
                bank_pulse_gen_mask[{r_wr_ptr, i[3:0]}]   <= hold_pulse_gen_mask[i];

                bank_tune_start_amp[{r_wr_ptr, i[3:0]}]   <= hold_tune_start_amp[i];
                bank_tune_amp_one[{r_wr_ptr, i[3:0]}]     <= hold_tune_amp_one[i];
                bank_tune_amp_two[{r_wr_ptr, i[3:0]}]     <= hold_tune_amp_two[i];
                bank_tune_vrc_len[{r_wr_ptr, i[3:0]}]     <= hold_tune_vrc_len[i];
                bank_tune_dac_min[{r_wr_ptr, i[3:0]}]     <= hold_tune_dac_min[i];
                bank_tune_dac_max[{r_wr_ptr, i[3:0]}]     <= hold_tune_dac_max[i];
                bank_tune_tune_mode[{r_wr_ptr, i[3:0]}]   <= hold_tune_tune_mode[i];
                bank_tune_log_offset[{r_wr_ptr, i[3:0]}]  <= hold_tune_log_offset[i];
            end
        end
    end

    // =========================================================================
    // 3. Формирование селекторов коммутации и индексов доступа к массивам
    // =========================================================================
    // Т.к. r_wr_ptr инвертируется по i_sys_sync, то после спада i_sys_sync:
    // - Текущие (новые) исполняемые параметры находятся в банке ~r_wr_ptr (active_sel).
    // - Предыдущие параметры (для пакета результатов) находятся в банке r_wr_ptr (sys_sel).
    wire active_sel = ~r_wr_ptr; // Селектор для текущего физического исполнения (новые параметры)
    wire sys_sel    = r_wr_ptr;  // Селектор для метаданных пакета результатов (замороженный банк)

    // Одномерные индексы доступа для 4-х физических каналов исполнения
    wire [4:0] active_idx_ch0 = {active_sel, 2'd0, i_sys_vch_sel};
    wire [4:0] active_idx_ch1 = {active_sel, 2'd1, i_sys_vch_sel};
    wire [4:0] active_idx_ch2 = {active_sel, 2'd2, i_sys_vch_sel};
    wire [4:0] active_idx_ch3 = {active_sel, 2'd3, i_sys_vch_sel};

    // Одномерный индекс доступа для чтения пакета результатов предыдущего измерения
    wire [4:0] packet_read_idx = {sys_sel, i_packet_phy_ch, i_packet_vch};

    // =========================================================================
    // 4. Формирование стабильных мультиплексированных сигналов в домене sys_clk
    //    для безопасного переноса через CDC по синкоимпульсам
    // =========================================================================
    wire [15:0] sys_ascan_n_samples  [0:3];
    wire [7:0]  sys_ascan_accum      [0:3];
    wire [1:0]  sys_ascan_accum_type [0:3];
    wire [15:0] sys_ascan_drop_ticks [0:3];

    wire [15:0] sys_pulse_charge     [0:3];
    wire [15:0] sys_pulse_transfer   [0:3];
    wire [7:0]  sys_pulse_strike     [0:3];

    genvar ch;
    generate
        for (ch = 0; ch < 4; ch = ch + 1) begin : gen_sys_mux
            // Вычисление локального индекса канала внутри генератора
            wire [4:0] active_ch_idx = {active_sel, ch[1:0], i_sys_vch_sel};

            assign sys_ascan_n_samples[ch]  = bank_ascan_n_samples [active_ch_idx];
            assign sys_ascan_accum[ch]      = bank_ascan_accum     [active_ch_idx];
            assign sys_ascan_accum_type[ch] = bank_ascan_accum_type[active_ch_idx];
            assign sys_ascan_drop_ticks[ch] = bank_ascan_drop_ticks[active_ch_idx];

            assign sys_pulse_charge[ch]     = bank_pulse_charge    [active_ch_idx];
            assign sys_pulse_transfer[ch]   = bank_pulse_transfer  [active_ch_idx];
            assign sys_pulse_strike[ch]     = bank_pulse_strike    [active_ch_idx];
        end
    endgenerate

    // =========================================================================
    // 5. Физические выходы в домене sys_clk (Реагируют на изменение i_sys_vch_sel)
    // =========================================================================
    always @(posedge sys_clk) begin
        if (!sys_rst_n) begin
            // Сброс физических коммутаторов
            o_ascan_ch0_pep_idx   <= `PARAM_DEFAULT_ASCAN_PEP_IDX;
            o_ascan_ch1_pep_idx   <= `PARAM_DEFAULT_ASCAN_PEP_IDX;
            o_ascan_ch2_pep_idx   <= `PARAM_DEFAULT_ASCAN_PEP_IDX;
            o_ascan_ch3_pep_idx   <= `PARAM_DEFAULT_ASCAN_PEP_IDX;

            o_pulse_ch0_gen_mask  <= `PARAM_DEFAULT_PULSE_GEN_MASK;
            o_pulse_ch1_gen_mask  <= `PARAM_DEFAULT_PULSE_GEN_MASK;
            o_pulse_ch2_gen_mask  <= `PARAM_DEFAULT_PULSE_GEN_MASK;
            o_pulse_ch3_gen_mask  <= `PARAM_DEFAULT_PULSE_GEN_MASK;

            // Сброс тракта tune ch0
            o_tune_ch0_start_amp  <= `PARAM_DEFAULT_TUNE_START_AMP;
            o_tune_ch0_amp_one    <= `PARAM_DEFAULT_TUNE_AMP_ONE;
            o_tune_ch0_amp_two    <= `PARAM_DEFAULT_TUNE_AMP_TWO;
            o_tune_ch0_vrc_len    <= `PARAM_DEFAULT_TUNE_VRC_LEN;
            o_tune_ch0_dac_min    <= `PARAM_DEFAULT_TUNE_DAC_MIN;
            o_tune_ch0_dac_max    <= `PARAM_DEFAULT_TUNE_DAC_MAX;
            o_tune_ch0_tune_mode  <= `PARAM_DEFAULT_TUNE_TUNE_MODE;
            o_tune_ch0_log_offset <= `PARAM_DEFAULT_TUNE_LOG_OFFSET;

            // Сброс тракта tune ch1
            o_tune_ch1_start_amp  <= `PARAM_DEFAULT_TUNE_START_AMP;
            o_tune_ch1_amp_one    <= `PARAM_DEFAULT_TUNE_AMP_ONE;
            o_tune_ch1_amp_two    <= `PARAM_DEFAULT_TUNE_AMP_TWO;
            o_tune_ch1_vrc_len    <= `PARAM_DEFAULT_TUNE_VRC_LEN;
            o_tune_ch1_dac_min    <= `PARAM_DEFAULT_TUNE_DAC_MIN;
            o_tune_ch1_dac_max    <= `PARAM_DEFAULT_TUNE_DAC_MAX;
            o_tune_ch1_tune_mode  <= `PARAM_DEFAULT_TUNE_TUNE_MODE;
            o_tune_ch1_log_offset <= `PARAM_DEFAULT_TUNE_LOG_OFFSET;

            // Сброс тракта tune ch2
            o_tune_ch2_start_amp  <= `PARAM_DEFAULT_TUNE_START_AMP;
            o_tune_ch2_amp_one    <= `PARAM_DEFAULT_TUNE_AMP_ONE;
            o_tune_ch2_amp_two    <= `PARAM_DEFAULT_TUNE_AMP_TWO;
            o_tune_ch2_vrc_len    <= `PARAM_DEFAULT_TUNE_VRC_LEN;
            o_tune_ch2_dac_min    <= `PARAM_DEFAULT_TUNE_DAC_MIN;
            o_tune_ch2_dac_max    <= `PARAM_DEFAULT_TUNE_DAC_MAX;
            o_tune_ch2_tune_mode  <= `PARAM_DEFAULT_TUNE_TUNE_MODE;
            o_tune_ch2_log_offset <= `PARAM_DEFAULT_TUNE_LOG_OFFSET;

            // Сброс тракта tune ch3
            o_tune_ch3_start_amp  <= `PARAM_DEFAULT_TUNE_START_AMP;
            o_tune_ch3_amp_one    <= `PARAM_DEFAULT_TUNE_AMP_ONE;
            o_tune_ch3_amp_two    <= `PARAM_DEFAULT_TUNE_AMP_TWO;
            o_tune_ch3_vrc_len    <= `PARAM_DEFAULT_TUNE_VRC_LEN;
            o_tune_ch3_dac_min    <= `PARAM_DEFAULT_TUNE_DAC_MIN;
            o_tune_ch3_dac_max    <= `PARAM_DEFAULT_TUNE_DAC_MAX;
            o_tune_ch3_tune_mode  <= `PARAM_DEFAULT_TUNE_TUNE_MODE;
            o_tune_ch3_log_offset <= `PARAM_DEFAULT_TUNE_LOG_OFFSET;
        end else begin
            // Коммутаторы физических трактов (sys_clk)
            o_ascan_ch0_pep_idx   <= bank_ascan_pep_idx[active_idx_ch0];
            o_ascan_ch1_pep_idx   <= bank_ascan_pep_idx[active_idx_ch1];
            o_ascan_ch2_pep_idx   <= bank_ascan_pep_idx[active_idx_ch2];
            o_ascan_ch3_pep_idx   <= bank_ascan_pep_idx[active_idx_ch3];

            o_pulse_ch0_gen_mask  <= bank_pulse_gen_mask[active_idx_ch0];
            o_pulse_ch1_gen_mask  <= bank_pulse_gen_mask[active_idx_ch1];
            o_pulse_ch2_gen_mask  <= bank_pulse_gen_mask[active_idx_ch2];
            o_pulse_ch3_gen_mask  <= bank_pulse_gen_mask[active_idx_ch3];

            // Тракт tune ch0
            o_tune_ch0_start_amp  <= bank_tune_start_amp [active_idx_ch0];
            o_tune_ch0_amp_one    <= bank_tune_amp_one   [active_idx_ch0];
            o_tune_ch0_amp_two    <= bank_tune_amp_two   [active_idx_ch0];
            o_tune_ch0_vrc_len    <= bank_tune_vrc_len   [active_idx_ch0];
            o_tune_ch0_dac_min    <= bank_tune_dac_min   [active_idx_ch0];
            o_tune_ch0_dac_max    <= bank_tune_dac_max   [active_idx_ch0];
            o_tune_ch0_tune_mode  <= bank_tune_tune_mode [active_idx_ch0];
            o_tune_ch0_log_offset <= bank_tune_log_offset[active_idx_ch0];

            // Тракт tune ch1
            o_tune_ch1_start_amp  <= bank_tune_start_amp [active_idx_ch1];
            o_tune_ch1_amp_one    <= bank_tune_amp_one   [active_idx_ch1];
            o_tune_ch1_amp_two    <= bank_tune_amp_two   [active_idx_ch1];
            o_tune_ch1_vrc_len    <= bank_tune_vrc_len   [active_idx_ch1];
            o_tune_ch1_dac_min    <= bank_tune_dac_min   [active_idx_ch1];
            o_tune_ch1_dac_max    <= bank_tune_dac_max   [active_idx_ch1];
            o_tune_ch1_tune_mode  <= bank_tune_tune_mode [active_idx_ch1];
            o_tune_ch1_log_offset <= bank_tune_log_offset[active_idx_ch1];

            // Тракт tune ch2
            o_tune_ch2_start_amp  <= bank_tune_start_amp [active_idx_ch2];
            o_tune_ch2_amp_one    <= bank_tune_amp_one   [active_idx_ch2];
            o_tune_ch2_amp_two    <= bank_tune_amp_two   [active_idx_ch2];
            o_tune_ch2_vrc_len    <= bank_tune_vrc_len   [active_idx_ch2];
            o_tune_ch2_dac_min    <= bank_tune_dac_min   [active_idx_ch2];
            o_tune_ch2_dac_max    <= bank_tune_dac_max   [active_idx_ch2];
            o_tune_ch2_tune_mode  <= bank_tune_tune_mode [active_idx_ch2];
            o_tune_ch2_log_offset <= bank_tune_log_offset[active_idx_ch2];

            // Тракт tune ch3
            o_tune_ch3_start_amp  <= bank_tune_start_amp [active_idx_ch3];
            o_tune_ch3_amp_one    <= bank_tune_amp_one   [active_idx_ch3];
            o_tune_ch3_amp_two    <= bank_tune_amp_two   [active_idx_ch3];
            o_tune_ch3_vrc_len    <= bank_tune_vrc_len   [active_idx_ch3];
            o_tune_ch3_dac_min    <= bank_tune_dac_min   [active_idx_ch3];
            o_tune_ch3_dac_max    <= bank_tune_dac_max   [active_idx_ch3];
            o_tune_ch3_tune_mode  <= bank_tune_tune_mode [active_idx_ch3];
            o_tune_ch3_log_offset <= bank_tune_log_offset[active_idx_ch3];
        end
    end

    // =========================================================================
    // 6. Междоменный перенос (CDC) в домен АЦП (adc_clk) по сигналу i_adc_sync
    // =========================================================================
    always @(posedge adc_clk) begin
        if (!adc_rst_n) begin
            // Сброс каналов
            o_ascan_ch0_n_samples   <= `PARAM_DEFAULT_ASCAN_N_SAMPLES;
            o_ascan_ch0_accum       <= `PARAM_DEFAULT_ASCAN_ACCUM;
            o_ascan_ch0_accum_type  <= `PARAM_DEFAULT_ASCAN_ACCUM_TYPE;
            o_ascan_ch0_drop_ticks  <= `PARAM_DEFAULT_ASCAN_DROP_TICKS;

            o_ascan_ch1_n_samples   <= `PARAM_DEFAULT_ASCAN_N_SAMPLES;
            o_ascan_ch1_accum       <= `PARAM_DEFAULT_ASCAN_ACCUM;
            o_ascan_ch1_accum_type  <= `PARAM_DEFAULT_ASCAN_ACCUM_TYPE;
            o_ascan_ch1_drop_ticks  <= `PARAM_DEFAULT_ASCAN_DROP_TICKS;

            o_ascan_ch2_n_samples   <= `PARAM_DEFAULT_ASCAN_N_SAMPLES;
            o_ascan_ch2_accum       <= `PARAM_DEFAULT_ASCAN_ACCUM;
            o_ascan_ch2_accum_type  <= `PARAM_DEFAULT_ASCAN_ACCUM_TYPE;
            o_ascan_ch2_drop_ticks  <= `PARAM_DEFAULT_ASCAN_DROP_TICKS;

            o_ascan_ch3_n_samples   <= `PARAM_DEFAULT_ASCAN_N_SAMPLES;
            o_ascan_ch3_accum       <= `PARAM_DEFAULT_ASCAN_ACCUM;
            o_ascan_ch3_accum_type  <= `PARAM_DEFAULT_ASCAN_ACCUM_TYPE;
            o_ascan_ch3_drop_ticks  <= `PARAM_DEFAULT_ASCAN_DROP_TICKS;
        end else if (i_adc_sync) begin
            // Защелкивание стабильных шин из домена sys_clk
            o_ascan_ch0_n_samples   <= sys_ascan_n_samples[0];
            o_ascan_ch0_accum       <= sys_ascan_accum[0];
            o_ascan_ch0_accum_type  <= sys_ascan_accum_type[0];
            o_ascan_ch0_drop_ticks  <= sys_ascan_drop_ticks[0];

            o_ascan_ch1_n_samples   <= sys_ascan_n_samples[1];
            o_ascan_ch1_accum       <= sys_ascan_accum[1];
            o_ascan_ch1_accum_type  <= sys_ascan_accum_type[1];
            o_ascan_ch1_drop_ticks  <= sys_ascan_drop_ticks[1];

            o_ascan_ch2_n_samples   <= sys_ascan_n_samples[2];
            o_ascan_ch2_accum       <= sys_ascan_accum[2];
            o_ascan_ch2_accum_type  <= sys_ascan_accum_type[2];
            o_ascan_ch2_drop_ticks  <= sys_ascan_drop_ticks[2];

            o_ascan_ch3_n_samples   <= sys_ascan_n_samples[3];
            o_ascan_ch3_accum       <= sys_ascan_accum[3];
            o_ascan_ch3_accum_type  <= sys_ascan_accum_type[3];
            o_ascan_ch3_drop_ticks  <= sys_ascan_drop_ticks[3];
        end
    end

    // =========================================================================
    // 7. Междоменный перенос (CDC) в домен pulse (hi_clk) по сигналу i_hi_sync
    // =========================================================================
    always @(posedge hi_clk) begin
        if (!hi_rst_n) begin
            // Сброс каналов
            o_pulse_ch0_charge      <= `PARAM_DEFAULT_PULSE_CHARGE;
            o_pulse_ch0_transfer    <= `PARAM_DEFAULT_PULSE_TRANSFER;
            o_pulse_ch0_strike      <= `PARAM_DEFAULT_PULSE_STRIKE;

            o_pulse_ch1_charge      <= `PARAM_DEFAULT_PULSE_CHARGE;
            o_pulse_ch1_transfer    <= `PARAM_DEFAULT_PULSE_TRANSFER;
            o_pulse_ch1_strike      <= `PARAM_DEFAULT_PULSE_STRIKE;

            o_pulse_ch2_charge      <= `PARAM_DEFAULT_PULSE_CHARGE;
            o_pulse_ch2_transfer    <= `PARAM_DEFAULT_PULSE_TRANSFER;
            o_pulse_ch2_strike      <= `PARAM_DEFAULT_PULSE_STRIKE;

            o_pulse_ch3_charge      <= `PARAM_DEFAULT_PULSE_CHARGE;
            o_pulse_ch3_transfer    <= `PARAM_DEFAULT_PULSE_TRANSFER;
            o_pulse_ch3_strike      <= `PARAM_DEFAULT_PULSE_STRIKE;
        end else if (i_hi_sync) begin
            // Защелкивание стабильных шин из домена sys_clk
            o_pulse_ch0_charge      <= sys_pulse_charge[0];
            o_pulse_ch0_transfer    <= sys_pulse_transfer[0];
            o_pulse_ch0_strike      <= sys_pulse_strike[0];

            o_pulse_ch1_charge      <= sys_pulse_charge[1];
            o_pulse_ch1_transfer    <= sys_pulse_transfer[1];
            o_pulse_ch1_strike      <= sys_pulse_strike[1];

            o_pulse_ch2_charge      <= sys_pulse_charge[2];
            o_pulse_ch2_transfer    <= sys_pulse_transfer[2];
            o_pulse_ch2_strike      <= sys_pulse_strike[2];

            o_pulse_ch3_charge      <= sys_pulse_charge[3];
            o_pulse_ch3_transfer    <= sys_pulse_transfer[3];
            o_pulse_ch3_strike      <= sys_pulse_strike[3];
        end
    end

    // =========================================================================
    // 8. Порты пакетного вывода (Домен sys_clk, чтение из "замороженного" банка)
    // =========================================================================
    always @(posedge sys_clk) begin
        if (!sys_rst_n) begin
            o_sys_ascan_n_samples  <= `PARAM_DEFAULT_ASCAN_N_SAMPLES;
            o_sys_ascan_accum      <= `PARAM_DEFAULT_ASCAN_ACCUM;
            o_sys_ascan_accum_type <= `PARAM_DEFAULT_ASCAN_ACCUM_TYPE;
            o_sys_ascan_drop_ticks <= `PARAM_DEFAULT_ASCAN_DROP_TICKS;
            o_sys_ascan_pep_idx    <= `PARAM_DEFAULT_ASCAN_PEP_IDX;

            o_sys_pulse_charge     <= `PARAM_DEFAULT_PULSE_CHARGE;
            o_sys_pulse_transfer   <= `PARAM_DEFAULT_PULSE_TRANSFER;
            o_sys_pulse_strike     <= `PARAM_DEFAULT_PULSE_STRIKE;
            o_sys_pulse_gen_mask   <= `PARAM_DEFAULT_PULSE_GEN_MASK;

            o_sys_tune_start_amp   <= `PARAM_DEFAULT_TUNE_START_AMP;
            o_sys_tune_amp_one     <= `PARAM_DEFAULT_TUNE_AMP_ONE;
            o_sys_tune_amp_two     <= `PARAM_DEFAULT_TUNE_AMP_TWO;
            o_sys_tune_vrc_len     <= `PARAM_DEFAULT_TUNE_VRC_LEN;
            o_sys_tune_dac_min     <= `PARAM_DEFAULT_TUNE_DAC_MIN;
            o_sys_tune_dac_max     <= `PARAM_DEFAULT_TUNE_DAC_MAX;
            o_sys_tune_tune_mode   <= `PARAM_DEFAULT_TUNE_TUNE_MODE;
            o_sys_tune_log_offset  <= `PARAM_DEFAULT_TUNE_LOG_OFFSET;
        end else begin
            // Чтение параметров выбранной пары координат [Физический][Виртуальный] из стабильного ("замороженного") банка sys_sel
            o_sys_ascan_n_samples  <= bank_ascan_n_samples [packet_read_idx];
            o_sys_ascan_accum      <= bank_ascan_accum     [packet_read_idx];
            o_sys_ascan_accum_type <= bank_ascan_accum_type[packet_read_idx];
            o_sys_ascan_drop_ticks <= bank_ascan_drop_ticks[packet_read_idx];
            o_sys_ascan_pep_idx    <= bank_ascan_pep_idx   [packet_read_idx];

            o_sys_pulse_charge     <= bank_pulse_charge    [packet_read_idx];
            o_sys_pulse_transfer   <= bank_pulse_transfer  [packet_read_idx];
            o_sys_pulse_strike     <= bank_pulse_strike    [packet_read_idx];
            o_sys_pulse_gen_mask   <= bank_pulse_gen_mask  [packet_read_idx];

            o_sys_tune_start_amp   <= bank_tune_start_amp  [packet_read_idx];
            o_sys_tune_amp_one     <= bank_tune_amp_one    [packet_read_idx];
            o_sys_tune_amp_two     <= bank_tune_amp_two    [packet_read_idx];
            o_sys_tune_vrc_len     <= bank_tune_vrc_len    [packet_read_idx];
            o_sys_tune_dac_min     <= bank_tune_dac_min    [packet_read_idx];
            o_sys_tune_dac_max     <= bank_tune_dac_max    [packet_read_idx];
            o_sys_tune_tune_mode   <= bank_tune_tune_mode  [packet_read_idx];
            o_sys_tune_log_offset  <= bank_tune_log_offset [packet_read_idx];
        end
    end

endmodule