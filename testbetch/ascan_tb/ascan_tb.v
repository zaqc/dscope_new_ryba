`timescale 1ns / 1ps

module ascan_tb;

    // -------------------------------------------------------------------------
    // 1. Генерация тактовых частот и сброса
    // -------------------------------------------------------------------------
    reg adc_clk = 0;
    reg sys_clk = 0;
    
    // Частота АЦП 65 МГц: период ~15.385 нс (полупериод 7.692 нс)
    always #7.692 adc_clk = ~adc_clk;
    
    // Системная частота 80 МГц: период 12.5 нс (полупериод 6.25 нс)
    always #6.25 sys_clk = ~sys_clk;

    // Синхронные сбросы активным низким уровнем
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
    // Ограничение времени симуляции (Simulation Watchdog / Timeout)
    // -------------------------------------------------------------------------
    initial begin
        #300000; // Предельный интервал симуляции
        $display("[ERROR] Simulation Watchdog Timeout!");
        $finish;
    end

    // -------------------------------------------------------------------------
    // 2. Сигналы интерфейса модуля ascan
    // -------------------------------------------------------------------------
    reg         i_adc_sync = 0;
    reg         i_sub_sync = 0;
    reg         i_sub_last = 0;
    wire        o_sub_done;
    reg signed [11:0] i_adc_data = 0;

    reg [15:0]  i_n_samples = 0;
    reg [7:0]   i_accum = 0;
    reg [1:0]   i_accum_type = 0;
    reg [15:0]  i_skip_ticks = 0;

    wire [31:0] o_out_data;
    wire        o_out_vld;
    wire        i_out_rdy;
    wire [15:0] o_out_size0;
    wire [15:0] o_out_size1;
    wire [15:0] o_out_size2;
    wire [15:0] o_out_size3;
    wire [15:0] o_out_size;
    wire        o_data_ready;

    // -------------------------------------------------------------------------
    // 3. Инстанцирование тестируемого модуля (MUT)
    // -------------------------------------------------------------------------
    ascan #(
        .ADDR_WIDTH(10) // Уменьшенный размер буфера для симуляции
    ) u_ascan (
        .adc_clk      (adc_clk),
        .adc_rst_n    (adc_rst_n),
        .i_adc_sync   (i_adc_sync),
        .i_sub_sync   (i_sub_sync),
        .i_sub_last   (i_sub_last),
        .o_sub_done   (o_sub_done),
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
        .o_out_size0  (o_out_size0),
        .o_out_size1  (o_out_size1),
        .o_out_size2  (o_out_size2),
        .o_out_size3  (o_out_size3),
        .o_out_size   (o_out_size),
        .o_data_ready (o_data_ready)
    );

    // -------------------------------------------------------------------------
    // 4. Память симулятора и массивы верификации
    // -------------------------------------------------------------------------
    reg signed [11:0] test_inputs[0:999];
    reg [11:0]        expected_array[0:999]; // Индексация: [run_idx * 100 + point_idx]
    reg [31:0]        rx_frame_buffer[0:999];
    
    // Переменные для верификации и расчетов
    integer           points_per_run[0:3];
    integer           words_per_run[0:3];
    integer           rx_word_cnt;

    // Вспомогательная функция вычисления абсолютного значения
    function [11:0] get_abs;
        input signed [11:0] val;
        begin
            if (val[11]) begin
                if (val == 12'sh800)
                    get_abs = 12'd2048;
                else
                    get_abs = -val;
            end else begin
                get_abs = val;
            end
        end
    endfunction

    // Генерация входных тестовых векторов заполнения
    integer init_idx;
    initial begin
        for (init_idx = 0; init_idx < 1000; init_idx = init_idx + 1) begin
            if (init_idx % 7 == 0)       test_inputs[init_idx] = -12'sd2048;
            else if (init_idx % 7 == 1)  test_inputs[init_idx] = 12'sd150;
            else if (init_idx % 7 == 2)  test_inputs[init_idx] = -12'sd500;
            else if (init_idx % 7 == 3)  test_inputs[init_idx] = 12'sd0;
            else if (init_idx % 7 == 4)  test_inputs[init_idx] = -12'sd10;
            else if (init_idx % 7 == 5)  test_inputs[init_idx] = 12'sd1000;
            else                         test_inputs[init_idx] = -12'sd300;
        end
    end

    // -------------------------------------------------------------------------
    // 5. Потоковый интерфейс чтения (AXI-Stream с обратным давлением)
    // -------------------------------------------------------------------------
    reg rdy_reg = 0;
    assign i_out_rdy = rdy_reg;

    // -------------------------------------------------------------------------
    // 6. Задача запуска субнакопления (Sub-Run)
    // -------------------------------------------------------------------------
    task run_sub_step(
        input integer run_idx,
        input is_first,
        input is_last,
        input [15:0] samples,
        input [7:0]  accum,
        input [1:0]  accum_type,
        input [15:0] skip,
        input integer start_pattern_offset
    );
        integer i;
        integer total_ticks;
        integer p, j, start_idx, end_idx;
        reg [19:0] g_sum;
        reg [11:0] g_max;
        reg [11:0] g_first;
        reg [11:0] abs_val;
        begin
            total_ticks = skip + samples + 10; // Запас тактов для конвейера
            
            // 1. Вычисление эталона для данного субцикла
            points_per_run[run_idx] = (samples + accum - 1) / accum;
            words_per_run[run_idx]  = ((points_per_run[run_idx] * 12) + 31) / 32;

            for (p = 0; p < points_per_run[run_idx]; p = p + 1) begin
                start_idx = start_pattern_offset + skip + p * accum;
                end_idx = start_idx + accum - 1;
                if (end_idx >= start_pattern_offset + skip + samples) begin
                    end_idx = start_pattern_offset + skip + samples - 1;
                end
                
                g_sum = 0;
                g_max = 0;
                g_first = get_abs(test_inputs[start_idx]);
                
                for (j = start_idx; j <= end_idx; j = j + 1) begin
                    abs_val = get_abs(test_inputs[j]);
                    g_sum = g_sum + abs_val;
                    if (abs_val > g_max || j == start_idx) begin
                        g_max = abs_val;
                    end
                end
                
                case (accum_type)
                    2'b00: expected_array[run_idx * 100 + p] = g_max;
                    2'b01: expected_array[run_idx * 100 + p] = g_sum / accum;
                    2'b10: expected_array[run_idx * 100 + p] = g_first;
                    default: expected_array[run_idx * 100 + p] = g_first;
                endcase
            end

            $display("[TB]  -> Запуск шага %0d: Samples=%0d, Accum=%0d, Type=%0d, Skip=%0d. Ожидаем точек: %0d", 
                     run_idx, samples, accum, accum_type, skip, points_per_run[run_idx]);

            // 2. Генерация управляющих сигналов запуска
            @(posedge adc_clk);
            i_n_samples  <= samples;
            i_accum      <= accum;
            i_accum_type <= accum_type;
            i_skip_ticks <= skip;
            i_sub_last   <= is_last;
            
            if (is_first) begin
                i_adc_sync <= 1'b1;
            end else begin
                i_sub_sync <= 1'b1;
            end
            i_adc_data <= test_inputs[start_pattern_offset];
            
            @(posedge adc_clk);
            i_adc_sync <= 1'b0;
            i_sub_sync <= 1'b0;

            // 3. Передача потока АЦП
            for (i = 1; i < total_ticks; i = i + 1) begin
                i_adc_data <= test_inputs[start_pattern_offset + i];
                @(posedge adc_clk);
            end
            i_adc_data <= 12'sd0;

            // 4. Ожидание завершения субнакопления (o_sub_done)
            if (!o_sub_done) begin
                @(posedge o_sub_done);
            end
            repeat(10) @(posedge adc_clk);
        end
    endtask

    // -------------------------------------------------------------------------
    // 7. Чтение кадра и разбор битового потока на выходе
    // -------------------------------------------------------------------------
    task read_and_verify_frame(input integer num_runs);
        integer total_words;
        integer run_idx;
        integer item_idx;
        integer bit_offset;
        integer word_idx;
        integer base_word;
        reg [11:0] unpacked_val;
        reg [63:0] temp_bitstream;
        integer match_errors;
        begin
            total_words = 0;
            for (run_idx = 0; run_idx < num_runs; run_idx = run_idx + 1) begin
                total_words = total_words + words_per_run[run_idx];
            end

            // Ожидаем поднятия флага готовности буфера
            wait(o_data_ready == 1'b1);
            #5;

            // Сравнение общих размеров
            if (o_out_size !== total_words) begin
                $display("[TB] [ERROR] Несовпадение размера кадра! Получено: %0d, Ожидалось: %0d", o_out_size, total_words);
                $finish;
            end

            // Чтение данных по AXI-Stream с эмуляцией backpressure
            rx_word_cnt = 0;
            while (rx_word_cnt < total_words) begin
                @(negedge sys_clk);
                rdy_reg = ($random % 10) < 8; // Готов в 80% тактов
                
                @(posedge sys_clk);
                if (o_out_vld && i_out_rdy) begin
                    rx_frame_buffer[rx_word_cnt] = o_out_data;
                    rx_word_cnt = rx_word_cnt + 1;
                end
            end
            @(negedge sys_clk);
            rdy_reg = 1'b0;
            wait(o_data_ready == 1'b0); // Убеждаемся, что готовность сброшена

            // Побитовая распаковка и поочередное сравнение с математическим эталоном
            match_errors = 0;
            base_word = 0;
            $display("[TB] --- Верификация Кадра (Субзапусков: %0d) ---", num_runs);
            
            for (run_idx = 0; run_idx < num_runs; run_idx = run_idx + 1) begin
                $display("  [Шаг %0d] Распаковка точек...", run_idx);
                for (item_idx = 0; item_idx < points_per_run[run_idx]; item_idx = item_idx + 1) begin
                    // Вычисляем смещение относительно начала текущего субзапуска
                    bit_offset = item_idx * 12;
                    word_idx = base_word + (bit_offset / 32);
                    bit_offset = bit_offset % 32;

                    // Чтение 64-битного скользящего окна
                    temp_bitstream = {32'd0, rx_frame_buffer[word_idx]};
                    if (bit_offset + 12 > 32) begin
                        temp_bitstream = temp_bitstream | ({32'd0, rx_frame_buffer[word_idx + 1]} << 32);
                    end

                    unpacked_val = (temp_bitstream >> bit_offset) & 12'hFFF;

                    if (unpacked_val === expected_array[run_idx * 100 + item_idx]) begin
                        $display("    [MATCH] Точка [%2d]: %0d", item_idx, unpacked_val);
                    end else begin
                        $display("    [ERROR] Точка [%2d]: Получено: %0d, Ожидалось: %0d <--- ОШИБКА!", 
                                 item_idx, unpacked_val, expected_array[run_idx * 100 + item_idx]);
                        match_errors = match_errors + 1;
                    end
                end
                // Переход к следующей 32-битной границе (с учетом Flush)
                base_word = base_word + words_per_run[run_idx];
            end

            if (match_errors == 0) begin
                $display("[TB] РЕЗУЛЬТАТ: Кадр верифицирован успешно!\n");
            end else begin
                $display("[TB] РЕЗУЛЬТАТ: Найдено %0d ошибок при проверке кадра!\n", match_errors);
                $finish;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // 8. Основной процесс верификации
    // -------------------------------------------------------------------------
    initial begin
`ifdef VCD_FILE
        $dumpfile(`VCD_FILE);
`else
        $dumpfile("ascan_tb.vcd");
`endif
        $dumpvars(0, ascan_tb);

        // Ждем снятия системных сбросов
        wait(adc_rst_n == 1'b1 && sys_rst_n == 1'b1);
        #100;

        // =====================================================================
        // ТЕСТ 1: Однократная регистрация в кадре (Стандартный одиночный запуск)
        // =====================================================================
        $display("\n=== СЦЕНАРИЙ 1: Одиночный запуск кадра (1 субцикл) ===");
        run_sub_step(0, 1'b1, 1'b1, 16'd16, 8'd2, 2'b00, 16'd0, 0);
        read_and_verify_frame(1);
        #200;

        // =====================================================================
        // ТЕСТ 2: Двукратная последовательная регистрация (Multi-capture x2)
        // =====================================================================
        $display("\n=== СЦЕНАРИЙ 2: Двукратная последовательная регистрация (2 субцикла) ===");
        
        // Шаг 0
        run_sub_step(0, 1'b1, 1'b0, 16'd16, 8'd2, 2'b00, 16'd4, 100);

        // Шаг 1
        run_sub_step(1, 1'b0, 1'b1, 16'd12, 8'd3, 2'b01, 16'd0, 200);
        
        read_and_verify_frame(2);
        #200;

        // =====================================================================
        // ТЕСТ 3: Полная 4-шаговая конфигурация (Multi-capture x4)
        // =====================================================================
        $display("\n=== СЦЕНАРИЙ 3: Полная последовательность (4 субцикла) ===");
        
        // Шаг 0
        run_sub_step(0, 1'b1, 1'b0, 16'd10, 8'd1, 2'b10, 16'd2, 300);

        // Шаг 1
        run_sub_step(1, 1'b0, 1'b0, 16'd20, 8'd4, 2'b00, 16'd1, 400);

        // Шаг 2
        run_sub_step(2, 1'b0, 1'b0, 16'd15, 8'd3, 2'b01, 16'd0, 500);

        // Шаг 3
        run_sub_step(3, 1'b0, 1'b1, 16'd8, 8'd2, 2'b10, 16'd3, 600);
        
        read_and_verify_frame(4);
        #200;

        $display("======================================================================");
        $display("     ВСЕ ТЕСТЫ ASCAN_TB ВЫПОЛНЕНЫ С ПОЛНЫМ СОВПАДЕНИЕМ СИГНАЛОВ      ");
        $display("======================================================================");
        $finish;
    end

endmodule