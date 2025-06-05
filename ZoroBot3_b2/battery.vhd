
-- battery.vhd
-- Este módulo gestiona la lectura y filtrado del voltaje de la batería,
-- y controla los LEDs para mostrar el nivel de la batería.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all; -- Para round() en constantes

library work;
use work.robot_types_pkg.all; -- Para constantes como AUX_BATTERY_ID, ADC_LSB_VHDL, etc.

entity battery is
    port (
        i_clk                       : in  std_logic;                                 -- Reloj del sistema
        i_rst_n                     : in  std_logic;                                 -- Reset asíncrono activo-bajo
        i_get_clock_ticks           : in  unsigned(31 downto 0);                     -- Contador de milisegundos del sistema

        -- Entradas para update_battery_voltage
        i_aux_battery_raw           : in  unsigned(ADC_BITS-1 downto 0);             -- get_aux_raw(AUX_BATTERY_ID) - Lectura raw del ADC para la batería

        -- Triggers y salidas para show_battery_level
        i_show_battery_level_trigger: in  std_logic;                                 -- Pulso para activar show_battery_level()

        -- Salidas/Triggers para funciones externas
        o_set_leds_battery_level    : out unsigned(7 downto 0);                      -- set_leds_battery_level(battery_level)
        o_set_leds_battery_level_en : out std_logic;                                 -- Enable para set_leds_battery_level
        o_all_leds_clear_trigger    : out std_logic;                                 -- all_leds_clear() trigger

        -- Salidas de estado (equivalentes a get_battery_voltage, get_battery_high_limit_voltage)
        o_battery_voltage           : out unsigned(23 downto 0);                     -- Voltaje actual de la batería (Fixed-point: MV_FP fractional bits)
        o_battery_high_limit_voltage: out unsigned(23 downto 0)                      -- Límite alto de voltaje de la batería (Fixed-point: MV_FP fractional bits)
    );
end entity battery;

architecture rtl of battery is

    -- Señales internas (equivalentes a variables 'static' en C)
    signal s_battery_full_voltage   : unsigned(23 downto 0) := BATTERY_3S_LOW_LIMIT_VOLTAGE_MV_FP; -- Fixed-point
    signal s_battery_voltage        : unsigned(23 downto 0) := (others => '0'); -- Fixed-point

    -- FSM para show_battery_level
    type t_battery_fsm_state is (
        S_IDLE,
        S_CALCULATE_LEVEL,
        S_SHOW_LEVEL_DELAY,
        S_CLEAR_LEDS
    );
    signal s_battery_fsm_state      : t_battery_fsm_state := S_IDLE;

    -- Señales para el manejador de retardo no bloqueante
    signal s_delay_start_ms         : unsigned(31 downto 0);
    signal s_delay_duration_ms      : unsigned(31 downto 0);
    signal s_delay_done_pulse       : std_logic;

    -- Variables para cálculos intermedios (ej. battery_level)
    signal s_calculated_battery_level : unsigned(7 downto 0); -- 0-100 range

begin

    -- Asignación de salidas
    o_battery_voltage            <= s_battery_voltage;
    o_battery_high_limit_voltage <= s_battery_full_voltage;

    --- Lógica de retardo no bloqueante ---
    -- Este proceso implementa un retardo genérico basado en los ticks del reloj en milisegundos.
    process (i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            s_delay_done_pulse <= '0';
        elsif rising_edge(i_clk) then
            s_delay_done_pulse <= '0'; -- Limpiar el pulso en cada ciclo

            if s_delay_duration_ms /= (others => '0') then -- El retardo está activo
                -- Comprobar si el tiempo ha transcurrido, manejando el desbordamiento del contador
                if i_get_clock_ticks >= s_delay_start_ms then
                    if (i_get_clock_ticks - s_delay_start_ms) >= s_delay_duration_ms then
                        s_delay_done_pulse <= '1';
                        s_delay_duration_ms <= (others => '0'); -- Marcar retardo como finalizado
                    end if;
                else -- Desbordamiento del contador (i_get_clock_ticks ha vuelto a 0)
                    if (to_unsigned(2**32-1, 32) - s_delay_start_ms) + i_get_clock_ticks >= s_delay_duration_ms then
                        s_delay_done_pulse <= '1';
                        s_delay_duration_ms <= (others => '0');
                    end if;
                end if;
            end if;
        end if;
    end process;


    --- Proceso para update_battery_voltage() ---
    -- Se asume que este proceso es llamado periódicamente o activado por un trigger.
    -- Aquí se activa en cada flanco de reloj, pero un trigger de un timer es más realista.
    -- Para simular el comportamiento C, se ejecuta en cada ciclo de reloj.
    process (i_clk, i_rst_n)
        -- Declaración de una variable para el voltaje raw calculado
        variable v_voltage_raw_fp : unsigned(23 downto 0); -- Acomoda (ADC_BITS + ADC_LSB_FP + VOLT_DIV_FACTOR_FP - BATTERY_VOLTAGE_FP - 1)
        variable v_temp_mult      : unsigned(ADC_BITS + ADC_LSB_FP + VOLT_DIV_FACTOR_FP -1 downto 0); -- Temporary for multiplication result
    begin
        if i_rst_n = '0' then
            s_battery_voltage <= (others => '0');
        elsif rising_edge(i_clk) then
            -- float voltage = get_aux_raw(AUX_BATTERY_ID) * ADC_LSB * VOLT_DIV_FACTOR;
            -- Fixed-point: voltage = (i_aux_battery_raw * ADC_LSB_VHDL * VOLT_DIV_FACTOR_VHDL) >> (ADC_LSB_FP + VOLT_DIV_FACTOR_FP - BATTERY_VOLTAGE_FP)
            -- Multiplicación de i_aux_battery_raw (12 bits) * ADC_LSB_VHDL (20 bits) * VOLT_DIV_FACTOR_VHDL (16 bits)
            -- Se necesita un ancho suficiente para el resultado de la multiplicación.
            -- Max bits: (12-1) + (20-1) + (16-1) + 1 = 11 + 19 + 15 + 1 = 46 bits.
            -- Para este caso específico: ADC_BITS + ADC_LSB_FP + VOLT_DIV_FACTOR_FP = 12 + 8 + 8 = 28 bits para el resultado de i_aux_battery_raw * ADC_LSB_VHDL * VOLT_DIV_FACTOR_VHDL antes de ajustar la escala.
            -- La escala final deseada es BATTERY_VOLTAGE_FP.
            -- v_voltage_raw_fp := (i_aux_battery_raw * ADC_LSB_VHDL * VOLT_DIV_FACTOR_VHDL) / (2**(ADC_LSB_FP + VOLT_DIV_FACTOR_FP - BATTERY_VOLTAGE_FP)); -- División es lenta
            -- Mejor con shifteo:
            v_temp_mult := (resize(i_aux_battery_raw, i_aux_battery_raw'length + ADC_LSB_VHDL'length) * ADC_LSB_VHDL);
            v_temp_mult := (resize(v_temp_mult, v_temp_mult'length + VOLT_DIV_FACTOR_VHDL'length) * VOLT_DIV_FACTOR_VHDL);
            -- Ajustar a la escala de BATTERY_VOLTAGE_FP bits fraccionarios
            v_voltage_raw_fp := resize(v_temp_mult(v_temp_mult'left downto (ADC_LSB_FP + VOLT_DIV_FACTOR_FP - BATTERY_VOLTAGE_FP)), 24);


            -- if (battery_voltage == 0) {
            if s_battery_voltage = (others => '0') then
                s_battery_voltage <= v_voltage_raw_fp;
            else
                -- battery_voltage = BATTERY_VOLTAGE_LOW_PASS_FILTER_ALPHA * voltage + (1 - BATTERY_VOLTAGE_LOW_PASS_FILTER_ALPHA) * battery_voltage;
                -- Fixed-point: (alpha * voltage) >> LPF_ALPHA_FP + ((1-alpha) * battery_voltage) >> LPF_ALPHA_FP
                s_battery_voltage <= resize(((v_voltage_raw_fp * BATTERY_LPF_ALPHA_VHDL) +
                                             (s_battery_voltage * BATTERY_LPF_ONE_MINUS_ALPHA_VHDL))
                                            sll (-BATTERY_LPF_ALPHA_FP), 24);
                -- Shifteo a la derecha por BATTERY_LPF_ALPHA_FP bits para deshacer la multiplicación por 2^BATTERY_LPF_ALPHA_FP
            end if;
        end if;
    end process;

    --- FSM para show_battery_level() ---
    process (i_clk, i_rst_n)
        -- Variables para cálculos intermedios
        variable v_voltage          : unsigned(23 downto 0);
        variable v_battery_level_calc : unsigned(7 downto 0);

        -- Fixed-point map variables
        variable v_val_norm         : unsigned(23+BATTERY_VOLTAGE_FP+1 downto 0); -- Sufficient width for intermediate map calculations
        variable v_map_low_in       : unsigned(23 downto 0);
        variable v_map_high_in      : unsigned(23 downto 0);
        variable v_map_low_out      : unsigned(7 downto 0); -- 0
        variable v_map_high_out     : unsigned(7 downto 0); -- 100
        variable v_range_in         : signed(24 downto 0);  -- For difference (high_in - low_in)
        variable v_result_shifted   : signed(24+8+1 downto 0); -- Max bits for intermediate product in map (v_val_norm * v_map_high_out)

    begin
        if i_rst_n = '0' then
            s_battery_fsm_state      <= S_IDLE;
            o_set_leds_battery_level <= (others => '0');
            o_set_leds_battery_level_en <= '0';
            o_all_leds_clear_trigger <= '0';
        elsif rising_edge(i_clk) then
            -- Limpiar pulsos de salida por defecto en cada ciclo
            o_set_leds_battery_level_en <= '0';
            o_all_leds_clear_trigger <= '0';

            case s_battery_fsm_state is
                when S_IDLE =>
                    if i_show_battery_level_trigger = '1' then
                        s_battery_fsm_state <= S_CALCULATE_LEVEL;
                    end if;

                when S_CALCULATE_LEVEL =>
                    v_voltage := s_battery_voltage; -- Obtener el voltaje actual

                    -- if (voltage >= BATTERY_3S_LOW_LIMIT_VOLTAGE && voltage <= BATTERY_3S_HIGH_LIMIT_VOLTAGE)
                    if v_voltage >= BATTERY_3S_LOW_LIMIT_VOLTAGE_MV_FP and
                       v_voltage <= BATTERY_3S_HIGH_LIMIT_VOLTAGE_MV_FP then
                        s_battery_full_voltage <= BATTERY_3S_HIGH_LIMIT_VOLTAGE_MV_FP;
                        v_map_low_in  := BATTERY_3S_LOW_LIMIT_VOLTAGE_MV_FP;
                        v_map_high_in := BATTERY_3S_HIGH_LIMIT_VOLTAGE_MV_FP;
                        v_map_low_out := (others => '0'); -- 0
                        v_map_high_out:= to_unsigned(100, 8); -- 100
                    -- } else if (voltage >= BATTERY_2S_LOW_LIMIT_VOLTAGE)
                    elsif v_voltage >= BATTERY_2S_LOW_LIMIT_VOLTAGE_MV_FP then -- Asume que ya no está en rango 3S
                        s_battery_full_voltage <= BATTERY_2S_HIGH_LIMIT_VOLTAGE_MV_FP;
                        v_map_low_in  := BATTERY_2S_LOW_LIMIT_VOLTAGE_MV_FP;
                        v_map_high_in := BATTERY_2S_HIGH_LIMIT_VOLTAGE_MV_FP;
                        v_map_low_out := (others => '0'); -- 0
                        v_map_high_out:= to_unsigned(100, 8); -- 100
                    else
                        -- Fuera de rango conocido, establecer a 0%
                        s_battery_full_voltage <= (others => '0'); -- O un valor predeterminado seguro
                        v_map_low_in  := (others => '0');
                        v_map_high_in := (others => '0');
                        v_map_low_out := (others => '0');
                        v_map_high_out:= (others => '0');
                    end if;

                    -- Implementación del 'map'
                    -- map(value, fromLow, fromHigh, toLow, toHigh)
                    -- result = (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
                    -- Aquí, toLow siempre es 0, y toHigh es 100.
                    -- battery_level = (voltage - fromLow) * 100 / (fromHigh - fromLow);

                    if v_map_high_in > v_map_low_in then
                        v_range_in := signed(v_map_high_in) - signed(v_map_low_in); -- (fromHigh - fromLow)
                        -- (voltage - fromLow)
                        v_val_norm := resize(v_voltage, v_voltage'length+1) - resize(v_map_low_in, v_map_low_in'length+1);
                        -- Multiplicar por toHigh (100) y luego dividir por v_range_in
                        v_result_shifted := resize(v_val_norm, v_val_norm'length + v_map_high_out'length) * signed(resize(v_map_high_out, v_map_high_out'length+1));
                        -- División entera en VHDL
                        v_battery_level_calc := to_unsigned(to_integer(v_result_shifted / v_range_in), 8);
                    else
                        v_battery_level_calc := (others => '0'); -- Evitar división por cero
                    end if;

                    -- Clamp the result to 0-100 range
                    if v_battery_level_calc > to_unsigned(100, 8) then
                        s_calculated_battery_level <= to_unsigned(100, 8);
                    elsif v_battery_level_calc < to_unsigned(0, 8) then
                        s_calculated_battery_level <= to_unsigned(0, 8);
                    else
                        s_calculated_battery_level <= v_battery_level_calc;
                    end if;

                    s_delay_start_ms   <= i_get_clock_ticks;
                    s_delay_duration_ms <= to_unsigned(750, 32); -- while (get_clock_ticks() < ticksInicio + 750)
                    s_battery_fsm_state <= S_SHOW_LEVEL_DELAY;

                when S_SHOW_LEVEL_DELAY =>
                    o_set_leds_battery_level_en <= '1';
                    o_set_leds_battery_level    <= s_calculated_battery_level;

                    if s_delay_done_pulse = '1' then
                        s_battery_fsm_state <= S_CLEAR_LEDS;
                    end if;

                when S_CLEAR_LEDS =>
                    o_all_leds_clear_trigger <= '1';
                    s_battery_fsm_state <= S_IDLE;

                when others =>
                    s_battery_fsm_state <= S_IDLE;
            end case;
        end if;
    end process;

end architecture rtl;
