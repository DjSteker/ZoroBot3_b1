
-- motors.vhd
-- Este módulo VHDL gestiona el control de los motores, la detección de saturación
-- y el control de velocidad del ventilador.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all; -- Solo para cálculos de constantes en tiempo de diseño/síntesis

-- Incluye el paquete de tipos personalizados donde se definen enumeraciones y records
use work.robot_types_pkg.all;

entity motors_controller is
    port (
        i_clk                     : in  std_logic;                                -- Reloj del sistema
        i_reset                   : in  std_logic;                                -- Reset síncrono (activo alto)

        -- Comandos de control desde el nivel superior (ej. 'move.vhd')
        i_set_motors_enable_en    : in  std_logic;                                -- Pulso para habilitar/deshabilitar motores
        i_motors_enable_val       : in  std_logic;                                -- Valor para habilitar (1) o deshabilitar (0)
        i_set_motors_speed_en     : in  std_logic;                                -- Pulso para establecer la velocidad de los motores (float en C)
        i_motors_speed_left_val   : in  signed(15 downto 0);                      -- Velocidad motor izquierdo (escalado, -1000 a 1000 en C)
        i_motors_speed_right_val  : in  signed(15 downto 0);                      -- Velocidad motor derecho (escalado)
        i_set_motors_brake_en     : in  std_logic;                                -- Pulso para activar el freno de los motores
        i_set_motors_pwm_en       : in  std_logic;                                -- Pulso para establecer el PWM de los motores (int32_t en C)
        i_motors_pwm_left_val     : in  signed(15 downto 0);                      -- Valor de PWM motor izquierdo
        i_motors_pwm_right_val    : in  signed(15 downto 0);                      -- Valor de PWM motor derecho
        i_set_fan_speed_en        : in  std_logic;                                -- Pulso para establecer la velocidad del ventilador (uint8_t en C)
        i_fan_speed_val           : in  natural range 0 to 100;                   -- Velocidad del ventilador (0-100)
        i_reset_motors_saturated_en : in std_logic;                               -- Pulso para resetear el estado de saturación

        -- Entradas de estado y sensores
        i_get_clock_ticks         : in  natural;                                  -- Ticks del reloj (para tiempo de saturación)
        i_get_encoder_angular_speed : in signed(15 downto 0);                     -- Velocidad angular del encoder (escalado)

        -- Control de habilitación de chequeo de saturación (desde 'move.vhd')
        i_set_check_motors_saturated_enabled_en : in std_logic;
        i_check_motors_saturated_enabled_val    : in std_logic;

        -- Salidas para GPIOs y Timers (abstracción del hardware subyacente)
        o_gpio_motor_enable       : out std_logic;                                -- Habilitación global de motores (GPIOB, GPIO15)
        o_tim8_oc1_val            : out natural;                                  -- PWM OC1 (Motor D - DIR)
        o_tim8_oc2_val            : out natural;                                  -- PWM OC2 (Motor D - PWM)
        o_tim8_oc3_val            : out natural;                                  -- PWM OC3 (Motor I - DIR)
        o_tim8_oc4_val            : out natural;                                  -- PWM OC4 (Motor I - PWM)
        o_tim1_oc1_val            : out natural;                                  -- PWM OC1 (Ventilador)

        -- Salidas de estado para el nivel superior
        o_is_motor_saturated      : out std_logic;                                -- Flag de saturación de motores
        o_motors_saturated_ms     : out natural                                   -- Tiempo en ms desde el inicio de la saturación
    );
end entity motors_controller;

architecture Behavioral of motors_controller is

    -- =========================================================================
    -- CONSTANTES (del código C)
    -- Se recomienda mover estas a un paquete de constantes global si se usan
    -- en múltiples módulos VHDL.
    -- =========================================================================
    constant C_MAX_MOTOR_SATURATION_COUNT       : natural := 10;
    constant C_MAX_MOTOR_ANGULAR_SATURATION_COUNT : natural := 10;
    constant C_MOTORES_MAX_PWM                  : natural := 1000; -- Max PWM value (ej. 1000 para 1000 puntos de resolución)
    constant C_MOTORES_SATURATION_PWM           : natural := 950;  -- PWM threshold for saturation (ej. 95% de MAX_PWM)
    constant C_MOTORES_SATURATION_ANGULAR_SPEED : natural := 5000; -- Angular speed threshold for saturation (escalado, ej. 5000 para 0.05 rad/s * 100000)
    constant C_LEDS_MAX_PWM                     : natural := 1000; -- Max PWM for LEDs/Fan

    -- Factor de escala para el mapeo de velocidades de motor (-1000 a 1000) a PWM (0 a MOTORES_MAX_PWM)
    -- La función 'map' de Arduino se traduce a una operación aritmética en VHDL.
    -- Aquí, 1000 es la velocidad máxima, y MOTORES_MAX_PWM es el valor máximo del contador del timer.
    constant C_SPEED_TO_PWM_SCALE_FACTOR : natural := C_MOTORES_MAX_PWM / 1000;

    -- =========================================================================
    -- SEÑALES INTERNAS (equivalentes a variables 'static' en C)
    -- =========================================================================
    signal s_motors_saturated           : std_logic := '0';
    signal s_motors_saturated_ms        : natural   := 0;
    signal s_left_motor_saturation_count : natural   := 0;
    signal s_right_motor_saturation_count: natural   := 0;
    signal s_angular_speed_saturation_count : natural := 0;

    signal s_check_motors_saturated_enabled : std_logic := '1'; -- Default true as in C

    -- Registros para las salidas de PWM
    signal r_tim8_oc1_val            : natural := C_MOTORES_MAX_PWM;
    signal r_tim8_oc2_val            : natural := C_MOTORES_MAX_PWM;
    signal r_tim8_oc3_val            : natural := C_MOTORES_MAX_PWM;
    signal r_tim8_oc4_val            : natural := C_MOTORES_MAX_PWM;
    signal r_tim1_oc1_val            : natural := 0;

    -- Funciones internas para simular las de C
    -- Función para simular 'map(value, fromLow, fromHigh, toLow, toHigh)'
    function map_func (
        value    : signed;
        fromLow  : signed;
        fromHigh : signed;
        toLow    : natural;
        toHigh   : natural
    ) return natural is
        variable result : natural;
    begin
        -- Evitar división por cero
        if (fromHigh - fromLow) = 0 then
            return toLow; -- O manejar error
        end if;
        -- (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow
        -- Para simplificar, asumimos fromLow = 0 como en el uso de set_motors_speed
        -- y que el 'value' ya está en el rango [0, fromHigh]
        result := (to_natural(abs(value) * to_signed(toHigh - toLow, value'length + (toHigh-toLow)'length))) / (to_natural(abs(fromHigh - fromLow)));
        result := result + toLow;
        return result;
    end function;

    -- Versión para valores naturales (para fan_speed)
    function map_func_nat (
        value    : natural;
        fromLow  : natural;
        fromHigh : natural;
        toLow    : natural;
        toHigh   : natural
    ) return natural is
        variable result : natural;
    begin
        if (fromHigh - fromLow) = 0 then
            return toLow;
        end if;
        result := ((value - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow);
        result := result + toLow;
        return result;
    end function;

begin

    -- =========================================================================
    -- PROCESO PRINCIPAL DE LÓGICA DE CONTROL (síncrono)
    -- =========================================================================
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                -- Reset de todas las señales internas
                s_motors_saturated            <= '0';
                s_motors_saturated_ms         <= 0;
                s_left_motor_saturation_count <= 0;
                s_right_motor_saturation_count <= 0;
                s_angular_speed_saturation_count <= 0;
                s_check_motors_saturated_enabled <= '1'; -- Default true
                o_gpio_motor_enable           <= '0';    -- Motores deshabilitados por defecto
                r_tim8_oc1_val <= C_MOTORES_MAX_PWM;
                r_tim8_oc2_val <= C_MOTORES_MAX_PWM;
                r_tim8_oc3_val <= C_MOTORES_MAX_PWM;
                r_tim8_oc4_val <= C_MOTORES_MAX_PWM;
                r_tim1_oc1_val <= 0;
            else
                -- Lógica para set_check_motors_saturated_enabled
                if i_set_check_motors_saturated_enabled_en = '1' then
                    s_check_motors_saturated_enabled <= i_check_motors_saturated_enabled_val;
                end if;

                -- Lógica para set_motors_enable
                if i_set_motors_enable_en = '1' then
                    o_gpio_motor_enable <= i_motors_enable_val;
                end if;

                -- Lógica para set_motors_brake
                if i_set_motors_brake_en = '1' then
                    r_tim8_oc1_val <= C_MOTORES_MAX_PWM;
                    r_tim8_oc2_val <= C_MOTORES_MAX_PWM;
                    r_tim8_oc3_val <= C_MOTORES_MAX_PWM;
                    r_tim8_oc4_val <= C_MOTORES_MAX_PWM;
                end if;

                -- Lógica para set_motors_speed (float en C, aquí signed fixed-point)
                -- Nota: Esta implementación de 'set_motors_speed' y 'set_motors_pwm'
                -- debería ser mutuamente exclusiva o tener prioridad.
                -- Aquí asumo que 'set_motors_pwm' es la que realmente establece los valores finales.
                -- Si 'set_motors_speed' debe usarse, necesitaría un FSM para su duración.
                -- El original de C es una función de seteo, no una operación de bucle.
                if i_set_motors_speed_en = '1' then
                    -- Esto calcula el ocI/ocD (offset/compare value)
                    -- y luego llama a set_motors_pwm internamente.
                    -- Para VHDL, o se hace aquí la lógica completa de set_motors_pwm,
                    -- o se convierte i_motors_speed_left_val/right_val a los pwm_left/right directamente.
                    -- Opto por que set_motors_pwm sea el punto de control final.
                    -- Por lo tanto, esta sección se encarga del 'map' y la dirección.
                    -- Luego, se pasaría al proceso que implementa 'set_motors_pwm'.
                    -- Sin embargo, el C original no llama a set_motors_pwm desde set_motors_speed,
                    -- sino que usa timer_set_oc_value directamente.
                    -- Replicamos esa lógica.
                    declare
                        v_ocI_abs : natural;
                        v_ocD_abs : natural;
                    begin
                        -- map(abs(velI), 0, 1000, 0, MOTORES_MAX_PWM);
                        v_ocI_abs := map_func_nat(to_natural(abs(i_motors_speed_left_val)), 0, 1000, 0, C_MOTORES_MAX_PWM);
                        v_ocD_abs := map_func_nat(to_natural(abs(i_motors_speed_right_val)), 0, 1000, 0, C_MOTORES_MAX_PWM);

                        if i_motors_speed_left_val > 0 then
                            r_tim8_oc4_val <= C_MOTORES_MAX_PWM - v_ocI_abs;
                            r_tim8_oc3_val <= C_MOTORES_MAX_PWM;
                        else
                            r_tim8_oc3_val <= C_MOTORES_MAX_PWM - v_ocI_abs;
                            r_tim8_oc4_val <= C_MOTORES_MAX_PWM;
                        end if;

                        if i_motors_speed_right_val > 0 then
                            r_tim8_oc2_val <= C_MOTORES_MAX_PWM - v_ocD_abs;
                            r_tim8_oc1_val <= C_MOTORES_MAX_PWM;
                        else
                            r_tim8_oc1_val <= C_MOTORES_MAX_PWM - v_ocD_abs;
                            r_tim8_oc2_val <= C_MOTORES_MAX_PWM;
                        end if;
                    end declare;
                end if;

                -- Lógica para set_motors_pwm
                -- Esta es la función principal que actualiza los valores de PWM y saturación.
                if i_set_motors_pwm_en = '1' then
                    declare
                        v_pwm_left  : signed(15 downto 0) := i_motors_pwm_left_val;
                        v_pwm_right : signed(15 downto 0) := i_motors_pwm_right_val;
                    begin
                        -- Limitar PWM
                        if v_pwm_left > C_MOTORES_MAX_PWM then
                            v_pwm_left := to_signed(C_MOTORES_MAX_PWM, v_pwm_left'length);
                        elsif v_pwm_left < -C_MOTORES_MAX_PWM then
                            v_pwm_left := to_signed(-C_MOTORES_MAX_PWM, v_pwm_left'length);
                        end if;

                        if v_pwm_right > C_MOTORES_MAX_PWM then
                            v_pwm_right := to_signed(C_MOTORES_MAX_PWM, v_pwm_right'length);
                        elsif v_pwm_right < -C_MOTORES_MAX_PWM then
                            v_pwm_right := to_signed(-C_MOTORES_MAX_PWM, v_pwm_right'length);
                        end if;

                        -- Detección de saturación del motor izquierdo
                        if to_natural(abs(v_pwm_left)) >= C_MOTORES_SATURATION_PWM then
                            s_left_motor_saturation_count <= s_left_motor_saturation_count + 1;
                        else
                            s_left_motor_saturation_count <= 0;
                        end if;

                        -- Actualizar PWM para motor izquierdo
                        if v_pwm_left >= 0 then
                            r_tim8_oc4_val <= C_MOTORES_MAX_PWM - to_natural(abs(v_pwm_left));
                            r_tim8_oc3_val <= C_MOTORES_MAX_PWM;
                        else
                            r_tim8_oc3_val <= C_MOTORES_MAX_PWM - to_natural(abs(v_pwm_left));
                            r_tim8_oc4_val <= C_MOTORES_MAX_PWM;
                        end if;

                        -- Detección de saturación del motor derecho
                        if to_natural(abs(v_pwm_right)) >= C_MOTORES_SATURATION_PWM then
                            s_right_motor_saturation_count <= s_right_motor_saturation_count + 1;
                        else
                            s_right_motor_saturation_count <= 0;
                        end if;

                        -- Actualizar PWM para motor derecho
                        if v_pwm_right >= 0 then
                            r_tim8_oc2_val <= C_MOTORES_MAX_PWM - to_natural(abs(v_pwm_right));
                            r_tim8_oc1_val <= C_MOTORES_MAX_PWM;
                        else
                            r_tim8_oc1_val <= C_MOTORES_MAX_PWM - to_natural(abs(v_pwm_right));
                            r_tim8_oc2_val <= C_MOTORES_MAX_PWM;
                        end if;

                        -- Detección de saturación por velocidad angular
                        if to_natural(abs(i_get_encoder_angular_speed)) >= C_MOTORES_SATURATION_ANGULAR_SPEED then
                            s_angular_speed_saturation_count <= s_angular_speed_saturation_count + 1;
                        else
                            s_angular_speed_saturation_count <= 0;
                        end if;
                    end declare;
                end if;

                -- Lógica para set_fan_speed
                if i_set_fan_speed_en = '1' then
                    declare
                        v_ocF : natural;
                    begin
                        if i_fan_speed_val /= 0 then
                            v_ocF := map_func_nat(i_fan_speed_val, 0, 100, 0, C_LEDS_MAX_PWM);
                        else
                            v_ocF := 0;
                        end if;
                        r_tim1_oc1_val <= v_ocF;
                    end declare;
                end if;

                -- Lógica para reset_motors_saturated
                if i_reset_motors_saturated_en = '1' then
                    s_motors_saturated            <= '0';
                    s_motors_saturated_ms         <= 0;
                    s_left_motor_saturation_count <= 0;
                    s_right_motor_saturation_count <= 0;
                    s_angular_speed_saturation_count <= 0;
                    s_check_motors_saturated_enabled <= '1';
                end if;

                -- Lógica para check_motors_saturated (siempre activa por ser static void)
                -- Se ejecuta en cada ciclo de reloj.
                if s_check_motors_saturated_enabled = '1' and (
                   s_left_motor_saturation_count >= C_MAX_MOTOR_SATURATION_COUNT or
                   s_right_motor_saturation_count >= C_MAX_MOTOR_SATURATION_COUNT or
                   s_angular_speed_saturation_count >= C_MAX_MOTOR_ANGULAR_SATURATION_COUNT
                ) then
                    if s_motors_saturated = '0' then -- Si no estaba saturado antes
                        s_motors_saturated_ms <= i_get_clock_ticks;
                    end if;
                    s_motors_saturated <= '1';
                else
                    s_motors_saturated <= '0';
                end if;

            end if; -- end if i_reset
        end if; -- end if rising_edge
    end process;


    -- =========================================================================
    -- ASIGNACIONES DE SALIDA (Combinacional)
    -- =========================================================================
    o_tim8_oc1_val            <= r_tim8_oc1_val;
    o_tim8_oc2_val            <= r_tim8_oc2_val;
    o_tim8_oc3_val            <= r_tim8_oc3_val;
    o_tim8_oc4_val            <= r_tim8_oc4_val;
    o_tim1_oc1_val            <= r_tim1_oc1_val;

    -- Salidas de estado
    o_is_motor_saturated      <= s_motors_saturated;
    o_motors_saturated_ms     <= s_motors_saturated_ms;

end architecture Behavioral;
