// =========================================================================
// Модуль Sequencer (Последователь циклов зондирования)
// Соответствует техническому описанию desc/sequencer.md
// =========================================================================

`timescale 1ns / 1ps

module sequencer (
    // Системные интерфейсы, тактирование и сбросы (синхронные, активный низкий)
    input  wire        sys_clk,
    input  wire        sys_rst_n,
    
    input  wire        adc_clk,
    input  wire        adc_rst_n,
    
    input  wire        log_clk,
    input  wire        log_rst_n,
    
    input  wire        dac_clk,
    input  wire        dac_rst_n,
    
    input  wire        hi_clk,
    input  wire        hi_rst_n,

    // Управляющие интерфейсы
    input  wire        i_sys_sync,        // Сигнал глобального запуска кадра (sys_clk)
    input  wire [1:0]  i_seq_count,       // Количество активных шагов в кадре (sys_clk)
    input  wire [3:0]  i_adc_chan_ready,  // Сигналы готовности от ascan 0..3 (adc_clk)

    // Выходные синхроимпульсы запуска (трансляция sub_sync)
    output wire        o_sys_sub_sync,    // Домен sys_clk (1 такт sys_clk)
    output wire        o_adc_sub_sync,    // Домен adc_clk (1 такт adc_clk)
    output wire        o_log_sub_sync,    // Домен log_clk (1 такт log_clk)
    output wire        o_dac_sub_sync,    // Домен dac_clk (1 такт dac_clk)
    output wire        o_hi_sub_sync,     // Домен hi_clk  (1 такт hi_clk)

    // Выходные синхроимпульсы признака последнего субцикла (трансляция sub_last)
    output wire        o_sys_sub_last,    // Домен sys_clk (1 такт sys_clk)
    output wire        o_adc_sub_last,    // Домен adc_clk (1 такт adc_clk)
    output wire        o_log_sub_last,    // Домен log_clk (1 такт log_clk)
    output wire        o_dac_sub_last,    // Домен dac_clk (1 такт dac_clk)
    output wire        o_hi_sub_last,     // Домен hi_clk  (1 такт hi_clk)

    // Выходные сигналы адресации и состояния (sys_clk)
    output reg  [1:0]  o_step_index,      // Индекс текущего шага [0..3]
    output wire        o_seq_busy         // Флаг выполнения кадра сканирования
);

    // =========================================================================
    // Локальные параметры и состояния FSM
    // =========================================================================
    localparam ST_IDLE       = 3'd0; // Ожидание запуска
    localparam ST_INIT       = 3'd1; // Инициализация кадра (пауза 1 такт)
    localparam ST_SETTLE     = 3'd2; // Стабилизация аналоговых ключей (пауза 2 такта)
    localparam ST_TRIGGER    = 3'd3; // Генерация импульсов запуска
    localparam ST_WAIT_READY = 3'd4; // Ожидание готовности каналов приема

    reg [2:0] state;
    reg [1:0] step_index;
    reg [1:0] settle_cnt;
    reg [1:0] seq_count_latched;
    reg       seq_busy_reg;
    reg       sys_sub_sync_reg;
    reg       sys_sub_last_reg;

    // Сдвинутые сигналы готовности, безопасные для домена sys_clk
    wire [3:0] synced_chan_ready;

    // =========================================================================
    // Конечный автомат управления (FSM) в системном домене sys_clk
    // =========================================================================
    always @(posedge sys_clk) begin
        if (!sys_rst_n) begin
            state             <= ST_IDLE;
            step_index        <= 2'b00;
            settle_cnt        <= 2'b00;
            seq_count_latched <= 2'b00;
            seq_busy_reg      <= 1'b0;
            sys_sub_sync_reg  <= 1'b0;
            sys_sub_last_reg  <= 1'b0;
            o_step_index      <= 2'b00;
        end else begin
            case (state)
                ST_IDLE: begin
                    seq_busy_reg     <= 1'b0;
                    sys_sub_sync_reg <= 1'b0;
                    sys_sub_last_reg <= 1'b0;
                    step_index       <= 2'b00;
                    
                    if (i_sys_sync) begin
                        seq_count_latched <= i_seq_count;
                        state             <= ST_INIT;
                    end
                end

                ST_INIT: begin
                    seq_busy_reg     <= 1'b1;
                    step_index       <= 2'b00;
                    settle_cnt       <= 2'b00;
                    // Гарантированный 1 такт ожидания
                    state            <= ST_SETTLE;
                end

                ST_SETTLE: begin
                    o_step_index <= step_index;
                    // Пауза 2 такта sys_clk (0 и 1)
                    if (settle_cnt == 2'd1) begin
                        state <= ST_TRIGGER;
                    end else begin
                        settle_cnt <= settle_cnt + 1'b1;
                    end
                end

                ST_TRIGGER: begin
                    sys_sub_sync_reg <= 1'b1;
                    sys_sub_last_reg <= (step_index == seq_count_latched);
                    state            <= ST_WAIT_READY;
                end

                ST_WAIT_READY: begin
                    // Сброс пусковых сигналов на следующем такте
                    sys_sub_sync_reg <= 1'b0;
                    sys_sub_last_reg <= 1'b0;
                    
                    // Ожидаем установки всех флагов готовности в 1
                    if (synced_chan_ready == 4'b1111) begin
                        if (step_index == seq_count_latched) begin
                            state <= ST_IDLE;
                        end else begin
                            step_index <= step_index + 1'b1;
                            settle_cnt <= 2'b00;
                            state      <= ST_SETTLE;
                        end
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

    // Назначение системных выходов состояния
    assign o_seq_busy     = seq_busy_reg;
    assign o_sys_sub_sync = sys_sub_sync_reg;
    assign o_sys_sub_last = sys_sub_last_reg;

    // =========================================================================
    // Синхронизация флагов готовности из домена adc_clk в домен sys_clk
    // =========================================================================
    sequencer_level_sync #(
        .WIDTH(4)
    ) sync_adc_ready_inst (
        .clk_dst   (sys_clk),
        .rst_dst_n (sys_rst_n),
        .level_src (i_adc_chan_ready),
        .level_dst (synced_chan_ready)
    );

    // =========================================================================
    // Трансляция импульсов sub_sync и sub_last в целевые тактовые домены
    // =========================================================================

    // --- Домен adc_clk (65 MHz) ---
    sequencer_pulse_sync sync_adc_sync (
        .clk_src   (sys_clk), .rst_src_n (sys_rst_n), .pulse_src (o_sys_sub_sync),
        .clk_dst   (adc_clk), .rst_dst_n (adc_rst_n), .pulse_dst (o_adc_sub_sync)
    );
    sequencer_pulse_sync sync_adc_last (
        .clk_src   (sys_clk), .rst_src_n (sys_rst_n), .pulse_src (o_sys_sub_last),
        .clk_dst   (adc_clk), .rst_dst_n (adc_rst_n), .pulse_dst (o_adc_sub_last)
    );

    // --- Домен log_clk (25 MHz) ---
    sequencer_pulse_sync sync_log_sync (
        .clk_src   (sys_clk), .rst_src_n (sys_rst_n), .pulse_src (o_sys_sub_sync),
        .clk_dst   (log_clk), .rst_dst_n (log_rst_n), .pulse_dst (o_log_sub_sync)
    );
    sequencer_pulse_sync sync_log_last (
        .clk_src   (sys_clk), .rst_src_n (sys_rst_n), .pulse_src (o_sys_sub_last),
        .clk_dst   (log_clk), .rst_dst_n (log_rst_n), .pulse_dst (o_log_sub_last)
    );

    // --- Домен dac_clk (50 MHz) ---
    sequencer_pulse_sync sync_dac_sync (
        .clk_src   (sys_clk), .rst_src_n (sys_rst_n), .pulse_src (o_sys_sub_sync),
        .clk_dst   (dac_clk), .rst_dst_n (dac_rst_n), .pulse_dst (o_dac_sub_sync)
    );
    sequencer_pulse_sync sync_dac_last (
        .clk_src   (sys_clk), .rst_src_n (sys_rst_n), .pulse_src (o_sys_sub_last),
        .clk_dst   (dac_clk), .rst_dst_n (dac_rst_n), .pulse_dst (o_dac_sub_last)
    );

    // --- Домен hi_clk (250 MHz) ---
    sequencer_pulse_sync sync_hi_sync (
        .clk_src   (sys_clk), .rst_src_n (sys_rst_n), .pulse_src (o_sys_sub_sync),
        .clk_dst   (hi_clk),  .rst_dst_n (hi_rst_n),  .pulse_dst (o_hi_sub_sync)
    );
    sequencer_pulse_sync sync_hi_last (
        .clk_src   (sys_clk), .rst_src_n (sys_rst_n), .pulse_src (o_sys_sub_last),
        .clk_dst   (hi_clk),  .rst_dst_n (hi_rst_n),  .pulse_dst (o_hi_sub_last)
    );

endmodule


// =========================================================================
// Вспомогательный модуль: Безопасный перенос импульса между доменами (CDC)
// Метод: Toggle-генератор -> 3-ступенчатый синхронизатор -> Детектор фронта
// =========================================================================
module sequencer_pulse_sync (
    input  wire clk_src,
    input  wire rst_src_n,
    input  wire pulse_src,
    input  wire clk_dst,
    input  wire rst_dst_n,
    output wire pulse_dst
);
    reg toggle_src;
    
    // Toggle-генератор в домене источника
    always @(posedge clk_src) begin
        if (!rst_src_n) begin
            toggle_src <= 1'b0;
        end else if (pulse_src) begin
            toggle_src <= ~toggle_src;
        end
    end

    // 3-ступенчатый синхронизатор уровня в целевом домене
    reg [2:0] sync_dst;
    always @(posedge clk_dst) begin
        if (!rst_dst_n) begin
            sync_dst <= 3'b000;
        end else begin
            sync_dst <= {sync_dst[1:0], toggle_src};
        end
    end

    // Детектор изменения уровня (XOR) восстанавливает импульс длительностью 1 такт
    assign pulse_dst = sync_dst[2] ^ sync_dst[1];

endmodule


// =========================================================================
// Вспомогательный модуль: Трехтриггерный синхронизатор уровня (Level Sync)
// Минимизирует эффекты метастабильности при переходе сигналов состояния
// =========================================================================
module sequencer_level_sync #(
    parameter WIDTH = 1
)(
    input  wire             clk_dst,
    input  wire             rst_dst_n,
    input  wire [WIDTH-1:0] level_src,
    output reg  [WIDTH-1:0] level_dst
);
    reg [WIDTH-1:0] sync_stage0;
    reg [WIDTH-1:0] sync_stage1;
    reg [WIDTH-1:0] sync_stage2;

    always @(posedge clk_dst) begin
        if (!rst_dst_n) begin
            sync_stage0 <= {WIDTH{1'b0}};
            sync_stage1 <= {WIDTH{1'b0}};
            sync_stage2 <= {WIDTH{1'b0}};
            level_dst   <= {WIDTH{1'b0}};
        end else begin
            sync_stage0 <= level_src;
            sync_stage1 <= sync_stage0;
            sync_stage2 <= sync_stage1;
            level_dst   <= sync_stage2;
        end
    end

endmodule