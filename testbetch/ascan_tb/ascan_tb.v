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
    // 2. Сигналы интерфейса модуля ascan
    // -------------------------------------------------------------------------
    reg         i_adc_sync = 0;
    reg signed [11:0] i_adc_data = 0;
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
    // 3. Инстанцирование тестируемого модуля (MUT)
    // -------------------------------------------------------------------------
    ascan #(
        .ADDR_WIDTH(10) // Уменьшенный размер буфера для ускорения симуляции
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
    // 4. Память симулятора и управляющие переменные
    // -------------------------------------------------------------------------
    reg signed [11:0] test_inputs[0:999];
    reg [11:0]        expected_array[0:999];
    reg [31:0]        rx_frame_buffer[0:999];
    
    integer           num_points;
    integer           expected_words;
    integer           p, j, start_idx, end_idx;
    reg [19:0]        g_sum;
    reg [11:0]        g_max;
    reg [11:0]        g_first;
    reg [11:0]        abs_val;

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

    // -------------------------------------------------------------------------
    // 5. Потоковый интерфейс чтения (AXI-Stream с обратным давлением)
    // -------------------------------------------------------------------------
    reg rdy_reg = 0;
    assign i_out_rdy = rdy_reg;

    // -------------------------------------------------------------------------
    // 6. Задача автоматического запуска измерения и генерации воздействий
    // -------------------------------------------------------------------------
    task trigger_measurement(
        input [15:0] samples,
        input [7:0]  accum,
        input [1:0]  accum_type,
        input [15:0] skip
    );
        integer i;
        integer total_feed;
        begin
            total_feed = skip + samples + 15;
            
            // 1. Наполняем тестовые входные векторы знаковыми амплитудами
            for (i = 0; i < total_feed; i = i + 1) begin
                if (i % 7 == 0)       test_inputs[i] = -12'sd2048; // Предельное отрицательное значение
                else if (i % 7 == 1)  test_inputs[i] = 12'sd150;
                else if (i % 7 == 2)  test_inputs[i] = -12'sd500;
                else if (i % 7 == 3)  test_inputs[i] = 12'sd0;
                else if (i % 7 == 4)  test_inputs[i] = -12'sd10;
                else if (i % 7 == 5)  test_inputs[i] = 12'sd1000;
                else                  test_inputs[i] = -12'sd300;
            end

            // 2. Вычисляем эталонные значения А-скана (математическая модель)
            num_points = (samples + accum - 1) / accum;
            
            for (p = 0; p < num_points; p = p + 1) begin
                start_idx = skip + p * accum;
                end_idx = start_idx + accum - 1;
                if (end_idx >= skip + samples) begin
                    end_idx = skip + samples - 1;
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
                    2'b00: expected_array[p] = g_max;                  // Пиковый детектор
                    2'b01: expected_array[p] = g_sum / accum;          // Интегратор
                    2'b10: expected_array[p] = g_first;                // Обычная децимация
                    default: expected_array[p] = g_first;
                endcase
            end

            // 3. Вычисляем ожидаемое количество 32-битных слов при бесшовной упаковке 12-битных точек
            expected_words = ((num_points * 12) + 31) / 32;

            $display("[TB] ----------------------------------------------------------------");
            $display("[TB] ТЕСТ: Samples=%0d, Accum=%0d, Type=%0d, Skip_ticks=%0d", samples, accum, accum_type, skip);
            $display("[TB] Ожидаем точек А-скана: %0d, ожидаем слов 32-бит: %0d", num_points, expected_words);
            $display("[TB] ----------------------------------------------------------------");

            // 4. Подаем строб запуска в домен АЦП
            @(posedge adc_clk);
            i_n_samples  <= samples;
            i_accum      <= accum;
            i_accum_type <= accum_type;
            i_skip_ticks <= skip;
            i_adc_sync   <= 1'b1;
            i_adc_data   <= test_inputs[0];
            
            @(posedge adc_clk);
            i_adc_sync   <= 1'b0;
            
            // 5. Подаем отсчеты АЦП такт за тактом
            for (i = 1; i < total_feed; i = i + 1) begin
                i_adc_data <= test_inputs[i];
                @(posedge adc_clk);
            end
            
            i_adc_data <= 12'sd0;
        end
    endtask

    // -------------------------------------------------------------------------
    // 7. Задача разбора битового потока и сравнения с эталоном
    // -------------------------------------------------------------------------
    task unpack_and_verify(input integer num_points);
        integer item_idx;
        integer bit_offset;
        integer word_idx;
        reg [11:0] unpacked_val;
        reg [63:0] temp_bitstream;
        integer match_errors;
        begin
            match_errors = 0;
            $display("[TB] Распаковка плотного 12-битного потока...");
            
            for (item_idx = 0; item_idx < num_points; item_idx = item_idx + 1) begin
                // Вычисляем сквозное битовое смещение
                bit_offset = item_idx * 12;
                word_idx = bit_offset / 32;
                bit_offset = bit_offset % 32;
                
                // Читаем скользящее 64-битное окно для бесшовного извлечения пересечений
                temp_bitstream = {32'd0, rx_frame_buffer[word_idx]};
                if (bit_offset + 12 > 32) begin
                    temp_bitstream = temp_bitstream | ({32'd0, rx_frame_buffer[word_idx + 1]} << 32);
                end
                
                // Извлекаем LSB-выровненную 12-битную точку без лишних бит
                unpacked_val = (temp_bitstream >> bit_offset) & 12'hFFF;
                
                if (unpacked_val === expected_array[item_idx]) begin
                    $display("  [MATCH] Точка [%2d]: Получено %0d, Ожидалось %0d", item_idx, unpacked_val, expected_array[item_idx]);
                end else begin
                    $display("  [ERROR] Точка [%2d]: Получено %0d, Ожидалось %0d <--- НЕСОВПАДЕНИЕ!", item_idx, unpacked_val, expected_array[item_idx]);
                    match_errors = match_errors + 1;
                end
            end
            
            if (match_errors == 0) begin
                $display("[TB] РЕЗУЛЬТАТ: Успешное совпадение 100%%\n");
            end else begin
                $display("[TB] РЕЗУЛЬТАТ: ОБНАРУЖЕНО %0d ОШИБОК!\n", match_errors);
                $finish;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // 8. Основной процесс верификации
    // -------------------------------------------------------------------------
    integer rx_word_cnt;

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
        // ТЕСТ 1: Пиковый детектор (Mode 0), без skip_ticks, кратный кадр
        // =====================================================================
        trigger_measurement(16'd16, 8'd2, 2'b00, 16'd0);
        
        // Ожидаем готовности буфера
        wait(o_data_ready == 1'b1);
        #10;
        
        // Верифицируем размер готового кадра
        if (o_out_size !== expected_words) begin
            $display("[TB] ОШИБКА: Размер выходного кадра %0d не совпадает с ожидаемым %0d!", o_out_size, expected_words);
            $finish;
        end

        // Читаем по потоковой шине с рандомизированным i_out_rdy на спаде такта
        rx_word_cnt = 0;
        while (rx_word_cnt < expected_words) begin
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
        wait(o_data_ready == 1'b0); // Убеждаемся, что флаг готовности снят после вычитки
        #100;
        unpack_and_verify(num_points);

        // =====================================================================
        // ТЕСТ 2: Интегратор / Среднее (Mode 1), со skip_ticks
        // =====================================================================
        trigger_measurement(16'd24, 8'd3, 2'b01, 16'd8);
        
        wait(o_data_ready == 1'b1);
        #10;
        if (o_out_size !== expected_words) begin
            $display("[TB] ОШИБКА: Размер выходного кадра %0d не совпадает с ожидаемым %0d!", o_out_size, expected_words);
            $finish;
        end

        rx_word_cnt = 0;
        while (rx_word_cnt < expected_words) begin
            @(negedge sys_clk);
            rdy_reg = ($random % 10) < 8;
            
            @(posedge sys_clk);
            if (o_out_vld && i_out_rdy) begin
                rx_frame_buffer[rx_word_cnt] = o_out_data;
                rx_word_cnt = rx_word_cnt + 1;
            end
        end
        @(negedge sys_clk);
        rdy_reg = 1'b0;
        wait(o_data_ready == 1'b0);
        #100;
        unpack_and_verify(num_points);

        // =====================================================================
        // ТЕСТ 3: Обычная децимация (Mode 2), некратные границы окна и кадра
        // =====================================================================
        trigger_measurement(16'd13, 8'd5, 2'b10, 16'd2);
        
        wait(o_data_ready == 1'b1);
        #10;
        if (o_out_size !== expected_words) begin
            $display("[TB] ОШИБКА: Размер выходного кадра %0d не совпадает с ожидаемым %0d!", o_out_size, expected_words);
            $finish;
        end

        rx_word_cnt = 0;
        while (rx_word_cnt < expected_words) begin
            @(negedge sys_clk);
            rdy_reg = ($random % 10) < 8;
            
            @(posedge sys_clk);
            if (o_out_vld && i_out_rdy) begin
                rx_frame_buffer[rx_word_cnt] = o_out_data;
                rx_word_cnt = rx_word_cnt + 1;
            end
        end
        @(negedge sys_clk);
        rdy_reg = 1'b0;
        wait(o_data_ready == 1'b0);
        #100;
        unpack_and_verify(num_points);

        // =====================================================================
        // ТЕСТ 4: Пиковый детектор (Mode 0), не кратный 8 остаток точек
        // =====================================================================
        trigger_measurement(16'd17, 8'd2, 2'b00, 16'd4);
        
        wait(o_data_ready == 1'b1);
        #10;
        if (o_out_size !== expected_words) begin
            $display("[TB] ОШИБКА: Размер выходного кадра %0d не совпадает с ожидаемым %0d!", o_out_size, expected_words);
            $finish;
        end

        rx_word_cnt = 0;
        while (rx_word_cnt < expected_words) begin
            @(negedge sys_clk);
            rdy_reg = ($random % 10) < 8;
            
            @(posedge sys_clk);
            if (o_out_vld && i_out_rdy) begin
                rx_frame_buffer[rx_word_cnt] = o_out_data;
                rx_word_cnt = rx_word_cnt + 1;
            end
        end
        @(negedge sys_clk);
        rdy_reg = 1'b0;
        wait(o_data_ready == 1'b0);
        #100;
        unpack_and_verify(num_points);

        // =====================================================================
        // ТЕСТ 5: Пропускной режим без децимации (accum = 1)
        // =====================================================================
        trigger_measurement(16'd6, 8'd1, 2'b10, 16'd0);
        
        wait(o_data_ready == 1'b1);
        #10;
        if (o_out_size !== expected_words) begin
            $display("[TB] ОШИБКА: Размер выходного кадра %0d не совпадает с ожидаемым %0d!", o_out_size, expected_words);
            $finish;
        end

        rx_word_cnt = 0;
        while (rx_word_cnt < expected_words) begin
            @(negedge sys_clk);
            rdy_reg = ($random % 10) < 8;
            
            @(posedge sys_clk);
            if (o_out_vld && i_out_rdy) begin
                rx_frame_buffer[rx_word_cnt] = o_out_data;
                rx_word_cnt = rx_word_cnt + 1;
            end
        end
        @(negedge sys_clk);
        rdy_reg = 1'b0;
        wait(o_data_ready == 1'b0);
        #100;
        unpack_and_verify(num_points);

        // =====================================================================
        // ТЕСТ 6: Экстремальный граничный случай (кадр из 1 отсчета)
        // =====================================================================
        trigger_measurement(16'd1, 8'd1, 2'b01, 16'd1);
        
        wait(o_data_ready == 1'b1);
        #10;
        if (o_out_size !== expected_words) begin
            $display("[TB] ОШИБКА: Размер выходного кадра %0d не совпадает с ожидаемым %0d!", o_out_size, expected_words);
            $finish;
        end

        rx_word_cnt = 0;
        while (rx_word_cnt < expected_words) begin
            @(negedge sys_clk);
            rdy_reg = ($random % 10) < 8;
            
            @(posedge sys_clk);
            if (o_out_vld && i_out_rdy) begin
                rx_frame_buffer[rx_word_cnt] = o_out_data;
                rx_word_cnt = rx_word_cnt + 1;
            end
        end
        @(negedge sys_clk);
        rdy_reg = 1'b0;
        wait(o_data_ready == 1'b0);
        #100;
        unpack_and_verify(num_points);

        $display("======================================================================");
        $display("       ВСЕ СЦЕНАРИИ ТЕСТБЕНЧА ASCAN УСПЕШНО И ПОЛНОСТЬЮ ВЕРИФИЦИРОВАНЫ ");
        $display("======================================================================");
        $finish;
    end

endmodule