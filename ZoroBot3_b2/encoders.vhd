
-- encoders.vhd
-- Este módulo VHDL implementa el procesamiento de datos de los encoders del motor.
-- Calcula las diferencias de ticks, distancias (micrómetros, milímetros)
-- y velocidades lineales/angulares utilizando aritmética de punto fijo.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Incluye el paquete de tipos y constantes personalizados,
-- que contiene las definiciones de punto fijo y las constantes de los encoders.
use work.robot_types_pkg.all;

entity encoders_controller is
    port (
        i_clk                       : in  std_logic;                                -- Reloj del sistema
        i_reset                     : in  std_logic;                                -- Reset síncrono (activo alto)

        -- Entradas de los contadores de los timers (simula timer_get_counter)
        i_timer_left_counter        : in  unsigned(15 downto 0);                    -- Contador raw del encoder izquierdo (uint16_t en C)
        i_timer_right_counter       : in  unsigned(15 downto 0);                    -- Contador raw del encoder derecho (uint16_t en C)

        -- Comandos de control (pulsos, activos en alto por un ciclo de reloj)
        i_update_readings_en        : in  std_logic;                                -- Pulso para activar update_encoder_readings()
        i_reset_avg_en              : in  std_logic;                                -- Pulso para activar reset_encoder_avg()

        -- Salidas de ticks
        o_left_total_ticks          : out signed(31 downto 0);                      -- left_total_ticks (int32_t)
        o_right_total_ticks         : out signed(31 downto 0);                      -- right_total_ticks (int32_t)

        -- Salidas de distancia (punto fijo, en micrómetros y milímetros)
        o_left_micrometers_fixed    : out signed(31 downto 0);                      -- left_micrometers (int32_t convertido a fixed-point)
        o_right_micrometers_fixed   : out signed(31 downto 0);                      -- right_micrometers (int32_t convertido a fixed-point)
        o_avg_micrometers_fixed     : out signed(31 downto 0);                      -- avg_micrometers (float convertido a fixed-point)
        o_left_millimeters_fixed    : out signed(31 downto 0);                      -- left_millimeters (int32_t convertido a fixed-point)
        o_right_millimeters_fixed   : out signed(31 downto 0);                      -- right_millimeters (int32_t convertido a fixed-point)
        o_avg_millimeters_fixed     : out signed(31 downto 0);                      -- avg_millimeters (float convertido a fixed-point)

        -- Salidas de velocidad (punto fijo, en mm/s y rad/s)
        o_left_speed_fixed          : out T_FIXED_POINT;                            -- left_speed (float convertido a fixed-point)
        o_right_speed_fixed         : out T_FIXED_POINT;                            -- right_speed (float convertido a fixed-point)
        o_avg_speed_fixed           : out T_FIXED_POINT;                            -- avg_speed (float convertido a fixed-point)
        o_angular_speed_fixed       : out T_FIXED_POINT                             -- angular_speed (float convertido a fixed-point)
    );
end entity encoders_controller;

architecture Behavioral of encoders_controller is

    -- =========================================================================
    -- SEÑALES INTERNAS (equivalentes a variables 'static volatile' en C)
    -- =========================================================================
    signal s_left_diff_ticks   : signed(31 downto 0) := (others => '0');
    signal s_right_diff_ticks  : signed(31 downto 0) := (others => '0');

    signal s_left_total_ticks  : signed(31 downto 0) := (others => '0');
    signal s_right_total_ticks : signed(31 downto 0) := (others => '0');

    signal s_left_micrometers_fixed  : signed(31 downto 0) := (others => '0'); -- Stored as fixed-point
    signal s_right_micrometers_fixed : signed(31 downto 0) := (others => '0'); -- Stored as fixed-point

    signal s_left_millimeters_fixed  : signed(31 downto 0) := (others => '0'); -- Stored as fixed-point
    signal s_right_millimeters_fixed : signed(31 downto 0) := (others => '0'); -- Stored as fixed-point

    signal s_left_speed_fixed    : T_FIXED_POINT := (others => '0');
    signal s_right_speed_fixed   : T_FIXED_POINT := (others => '0');
    signal s_angular_speed_fixed : T_FIXED_POINT := (others => '0');

    -- Variables estáticas para almacenar el último tick leído
    signal s_last_left_ticks   : unsigned(15 downto 0) := (others => '0');
    signal s_last_right_ticks  : unsigned(15 downto 0) := (others => '0');

    -- =========================================================================
    -- FUNCIONES AUXILIARES
    -- =========================================================================

    -- max_likelihood_counter_diff: Implementa la lógica de C para manejar desbordamientos.
    -- Retorna la diferencia de ticks como un valor signed(31 downto 0).
    function max_likelihood_counter_diff_fp(now_val, before_val : unsigned(15 downto 0)) return signed(31 downto 0) is
        variable forward_diff_u : unsigned(15 downto 0);
        variable backward_diff_u : unsigned(15 downto 0);
    begin
        forward_diff_u := now_val - before_val; -- This will wrap around correctly for unsigned types
        backward_diff_u := before_val - now_val; -- This will wrap around correctly for unsigned types

        if forward_diff_u > backward_diff_u then
            return -(signed(resize(backward_diff_u, 32)));
        else
            return signed(resize(forward_diff_u, 32));
        end if;
    end function;

    -- Fixed-point division (A / B)
    function fixed_div(A, B : T_FIXED_POINT) return T_FIXED_POINT is
    begin
        -- For division, to maintain precision, multiply by C_FIXED_POINT_SCALE before dividing.
        -- If B is zero, return 0 to avoid division by zero errors.
        if B = 0 then return (others => '0'); end if;
        return resize((A * C_FIXED_POINT_SCALE) / B, A'length);
    end function;

    -- Fixed-point multiplication (A * B)
    function fixed_mul(A, B : T_FIXED_POINT) return T_FIXED_POINT is
    begin
        -- For multiplication, multiply and then divide by C_FIXED_POINT_SCALE to account for double scaling.
        return resize((A * B) / C_FIXED_POINT_SCALE, A'length);
    end function;


begin

    -- =========================================================================
    -- PROCESO PRINCIPAL DE ACTUALIZACIÓN DE LECTURAS DE ENCODERS
    -- =========================================================================
    process (i_clk)
        -- Declaración de variables internas para el proceso (equivalente a variables locales)
        variable v_left_ticks  : unsigned(15 downto 0);
        variable v_right_ticks : unsigned(15 downto 0);

        variable v_temp_left_micrometers_fixed  : signed(31 downto 0);
        variable v_temp_right_micrometers_fixed : signed(31 downto 0);

        variable v_temp_left_millimeters_fixed  : signed(31 downto 0);
        variable v_temp_right_millimeters_fixed : signed(31 downto 0);

        variable v_temp_left_speed_fixed  : T_FIXED_POINT;
        variable v_temp_right_speed_fixed : T_FIXED_POINT;

        variable v_temp_angular_speed_fixed : T_FIXED_POINT;

    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                s_left_diff_ticks   <= (others => '0');
                s_right_diff_ticks  <= (others => '0');
                s_left_total_ticks  <= (others => '0');
                s_right_total_ticks <= (others => '0');
                s_left_micrometers_fixed  <= (others => '0');
                s_right_micrometers_fixed <= (others => '0');
                s_left_millimeters_fixed  <= (others => '0');
                s_right_millimeters_fixed <= (others => '0');
                s_left_speed_fixed    <= (others => '0');
                s_right_speed_fixed   <= (others => '0');
                s_angular_speed_fixed <= (others => '0');
                s_last_left_ticks   <= (others => '0');
                s_last_right_ticks  <= (others => '0');
            else
                -- Lógica para reset_encoder_avg()
                if i_reset_avg_en = '1' then
                    s_left_micrometers_fixed  <= (others => '0');
                    s_right_micrometers_fixed <= (others => '0');
                    s_left_millimeters_fixed  <= (others => '0');
                    s_right_millimeters_fixed <= (others => '0');
                    -- Note: C code resets avg_micrometers/millimeters, but these are derived.
                    -- Speeds are not reset here, they are recalculated.
                end if;

                -- Lógica para update_encoder_readings()
                if i_update_readings_en = '1' then
                    -- Read raw encoder counters (simulated by inputs)
                    v_left_ticks  := i_timer_left_counter;
                    v_right_ticks := i_timer_right_counter;

                    -- Calculate diff ticks (accounting for overflow)
                    -- Note: C code has `left_diff_ticks = -max_likelihood_counter_diff(left_ticks, last_left_ticks);`
                    -- The negative sign on left_diff_ticks suggests one encoder might be reversed.
                    s_left_diff_ticks  <= -max_likelihood_counter_diff_fp(v_left_ticks, s_last_left_ticks);
                    s_right_diff_ticks <= max_likelihood_counter_diff_fp(v_right_ticks, s_last_right_ticks);

                    -- Update total ticks
                    s_left_total_ticks  <= s_left_total_ticks + s_left_diff_ticks;
                    s_right_total_ticks <= s_right_total_ticks + s_right_diff_ticks;

                    -- Calculate distance travelled (micrometers)
                    -- left_micrometers = (int32_t)(left_total_ticks * MICROMETERS_PER_TICK);
                    -- Fixed-point: (s_left_total_ticks * C_MICROMETERS_PER_TICK_FP) / C_FIXED_POINT_SCALE
                    v_temp_left_micrometers_fixed  := fixed_mul(s_left_total_ticks, C_MICROMETERS_PER_TICK_FP);
                    v_temp_right_micrometers_fixed := fixed_mul(s_right_total_ticks, C_MICROMETERS_PER_TICK_FP);
                    s_left_micrometers_fixed  <= v_temp_left_micrometers_fixed;
                    s_right_micrometers_fixed <= v_temp_right_micrometers_fixed;

                    -- Calculate distance travelled (millimeters)
                    -- left_millimeters = left_micrometers / MICROMETERS_PER_MILLIMETER;
                    -- Fixed-point: s_left_micrometers_fixed / C_MICROMETERS_PER_MILLIMETER (integer division or fixed-point div)
                    s_left_millimeters_fixed  <= s_left_micrometers_fixed  / to_fixed(C_MICROMETERS_PER_MILLIMETER);
                    s_right_millimeters_fixed <= s_right_micrometers_fixed / to_fixed(C_MICROMETERS_PER_MILLIMETER);


                    -- Calculate speed (mm/s)
                    -- left_speed = left_diff_ticks * (MICROMETERS_PER_TICK / MICROMETERS_PER_MILLIMETER) * SYSTICK_FREQUENCY_HZ;
                    -- Re-arrange to avoid intermediate float division in C
                    -- left_speed = left_diff_ticks * MICROMETERS_PER_TICK * SYSTICK_FREQUENCY_HZ / MICROMETERS_PER_MILLIMETER;
                    -- Fixed-point: s_left_diff_ticks * C_MICROMETERS_PER_TICK_FP * C_SYSTICK_FREQUENCY_HZ_FP / C_MICROMETERS_PER_MILLIMETER_FP
                    -- Use direct constants where possible.
                    v_temp_left_speed_fixed  := fixed_mul(fixed_mul(s_left_diff_ticks, C_MICROMETERS_PER_TICK_FP), to_fixed(C_SYSTICK_FREQUENCY_HZ)) / to_fixed(C_MICROMETERS_PER_MILLIMETER);
                    v_temp_right_speed_fixed := fixed_mul(fixed_mul(s_right_diff_ticks, C_MICROMETERS_PER_TICK_FP), to_fixed(C_SYSTICK_FREQUENCY_HZ)) / to_fixed(C_MICROMETERS_PER_MILLIMETER);
                    s_left_speed_fixed  <= v_temp_left_speed_fixed;
                    s_right_speed_fixed <= v_temp_right_speed_fixed;


                    -- Calculate angular speed (rad/s)
                    -- angular_speed = (float)((right_speed - left_speed) / MILLIMETERS_PER_METER) / ((float)WHEELS_SEPARATION / (float)MILLIMETERS_PER_METER);
                    -- This simplifies to: angular_speed = (right_speed - left_speed) / WHEELS_SEPARATION; (as MILLIMETERS_PER_METER cancels out)
                    -- Fixed-point: (s_right_speed_fixed - s_left_speed_fixed) / C_WHEELS_SEPARATION_MM_FP
                    v_temp_angular_speed_fixed := fixed_div((s_right_speed_fixed - s_left_speed_fixed), C_WHEELS_SEPARATION_MM_FP);
                    s_angular_speed_fixed <= v_temp_angular_speed_fixed;

                    -- Update static variables for next reading
                    s_last_left_ticks  <= v_left_ticks;
                    s_last_right_ticks <= v_right_ticks;
                end if;
            end if;
        end if;
    end process;


    -- =========================================================================
    -- ASIGNACIONES DE SALIDA
    -- =========================================================================
    o_left_total_ticks          <= s_left_total_ticks;
    o_right_total_ticks         <= s_right_total_ticks;

    o_left_micrometers_fixed    <= s_left_micrometers_fixed;
    o_right_micrometers_fixed   <= s_right_micrometers_fixed;
    o_avg_micrometers_fixed     <= (s_left_micrometers_fixed + s_right_micrometers_fixed) / to_fixed(2);

    o_left_millimeters_fixed    <= s_left_millimeters_fixed;
    o_right_millimeters_fixed   <= s_right_millimeters_fixed;
    o_avg_millimeters_fixed     <= (s_left_millimeters_fixed + s_right_millimeters_fixed) / to_fixed(2);

    o_left_speed_fixed          <= s_left_speed_fixed;
    o_right_speed_fixed         <= s_right_speed_fixed;
    o_avg_speed_fixed           <= (s_left_speed_fixed + s_right_speed_fixed) / to_fixed(2);
    o_angular_speed_fixed       <= s_angular_speed_fixed;

end architecture Behavioral;
