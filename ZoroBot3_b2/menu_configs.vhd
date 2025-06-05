
-- menu_configs.vhd
-- Este módulo VHDL gestiona la interfaz del menú de configuración,
-- incluyendo la selección de modo (calibración/depuración) y la selección de valores.
-- Controla los LEDs de estado e información y activa rutinas externas.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Incluye el paquete de tipos personalizados donde se definen enumeraciones y records
use work.robot_types_pkg.all;

entity menu_configs_controller is
    port (
        i_clk                     : in  std_logic;                                -- Reloj del sistema
        i_reset                   : in  std_logic;                                -- Reset síncrono (activo alto)

        -- Entradas de botones (activas en alto)
        i_menu_mode_btn           : in  std_logic;                                -- Botón para cambiar de modo/confirmar
        i_menu_up_btn             : in  std_logic;                                -- Botón para aumentar el valor
        i_menu_down_btn           : in  std_logic;                                -- Botón para disminuir el valor

        -- Entrada de tiempo para delays y detección de pulsación larga
        i_get_clock_ticks         : in  natural;                                  -- Ticks del reloj en ms

        -- Entrada de estado de depuración (desde un módulo externo)
        i_is_debug_enabled        : in  std_logic;

        -- Comandos para resetear la configuración del menú
        i_menu_config_reset_values_en : in  std_logic;                            -- Pulso para resetear valueConfig
        i_menu_config_reset_mode_en   : in  std_logic;                            -- Pulso para resetear modeConfig

        -- Salidas para controlar LEDs de estado
        o_set_status_led_en       : out std_logic;                                -- Pulso para set_status_led
        o_status_led_val          : out std_logic;                                -- Valor para set_status_led (true/false)
        o_warning_status_led_en   : out std_logic;                                -- Pulso para warning_status_led
        o_warning_status_led_freq : out natural;                                  -- Frecuencia para warning_status_led (ms)
        o_set_rgb_color_en        : out std_logic;                                -- Pulso para set_RGB_color
        o_rgb_r                   : out natural range 0 to 255;                   -- Componente R
        o_rgb_g                   : out natural range 0 to 255;                   -- Componente G
        o_rgb_b                   : out natural range 0 to 255;                   -- Componente B

        -- Salidas para controlar LEDs de información
        o_clear_info_leds_en      : out std_logic;                                -- Pulso para clear_info_leds
        o_set_info_led_en         : out std_logic;                                -- Pulso para set_info_led
        o_info_led_idx            : out natural;                                  -- Índice del LED de información
        o_info_led_val            : out std_logic;                                -- Valor del LED de información (on/off)
        o_set_leds_wave_en        : out std_logic;                                -- Pulso para set_leds_wave
        o_leds_wave_freq          : out natural;                                  -- Frecuencia para leds_wave
        o_set_leds_side_sensors_en: out std_logic;                                -- Pulso para set_leds_side_sensors
        o_leds_side_sensors_freq  : out natural;                                  -- Frecuencia para leds_side_sensors
        o_set_leds_front_sensors_en: out std_logic;                               -- Pulso para set_leds_front_sensors
        o_leds_front_sensors_freq : out natural;                                  -- Frecuencia para leds_front_sensors
        o_set_leds_blink_en       : out std_logic;                                -- Pulso para set_leds_blink
        o_leds_blink_freq         : out natural;                                  -- Frecuencia para leds_blink

        -- Salidas para activar rutinas de calibración y depuración
        o_calibrate_from_config_en : out std_logic;                               -- Pulso para calibrate_from_config
        o_calibrate_from_config_val: out natural;                                  -- Valor de calibración
        o_debug_from_config_en    : out std_logic;                                -- Pulso para debug_from_config
        o_debug_from_config_val   : out natural;                                  -- Valor de depuración

        -- Salida de estado del menú de configuración
        o_menu_config_exit        : out std_logic                                 -- Indica si el menú de configuración debe salirse
    );
end entity menu_configs_controller;

architecture Behavioral of menu_configs_controller is

    -- =========================================================================
    -- ESTADOS DE LA MÁQUINA DE ESTADOS PRINCIPAL (menu_config_handler)
    -- =========================================================================
    type menu_fsm_state_type is (
        STATE_HANDLE_MODES_AND_VALUES,    -- Estado inicial y continuo de manejo de LEDs
        STATE_MODE_BTN_PRESS_WAIT_RELEASE, -- Espera a que se suelte el botón de modo
        STATE_MODE_BTN_LONG_PRESS_CHECK,   -- Verifica pulsación larga del botón de modo
        STATE_MODE_BTN_SHORT_PRESS_DETECTED, -- Pulsación corta de botón de modo
        STATE_MODE_BTN_LONG_PRESS_DETECTED,  -- Pulsación larga de botón de modo
        STATE_UP_BTN_PRESS_WAIT_RELEASE,   -- Espera a que se suelte el botón UP
        STATE_DOWN_BTN_PRESS_WAIT_RELEASE, -- Espera a que se suelte el botón DOWN
        STATE_DELAY_AFTER_BTN_PRESS        -- Estado para la función 'delay(50)'
    );
    signal current_menu_state : menu_fsm_state_type := STATE_HANDLE_MODES_AND_VALUES;
    signal next_menu_state    : menu_fsm_state_type;

    -- =========================================================================
    -- SEÑALES INTERNAS (equivalentes a variables 'static' en C)
    -- =========================================================================
    signal s_mode_config               : menu_config_mode_type := MODE_CALIBRATION;
    signal s_value_config              : value_config_array_t  := (others => 0); -- Inicializa todos los valores a 0

    -- Señales para debouncing de botones
    signal s_menu_mode_btn_debounced   : std_logic := '0';
    signal s_menu_up_btn_debounced     : std_logic := '0';
    signal s_menu_down_btn_debounced   : std_logic := '0';

    -- Señales para detección de flancos (rising_edge)
    signal s_menu_mode_btn_rising_edge : std_logic := '0';
    signal s_menu_up_btn_rising_edge   : std_logic := '0';
    signal s_menu_down_btn_rising_edge : std_logic := '0';

    -- Temporizadores para pulsación larga y delays
    signal s_mode_btn_press_start_ms : natural := 0;
    signal s_delay_timer_start_ms    : natural := 0;
    signal s_long_press_threshold_ms : natural := 200; -- umbral de pulsación larga en ms
    signal s_short_delay_ms          : natural := 50;  -- duración del delay en ms

    -- Señales de pulso para las salidas (activas por un ciclo de reloj)
    signal s_o_set_status_led_en_pulse      : std_logic := '0';
    signal s_o_warning_status_led_en_pulse  : std_logic := '0';
    signal s_o_set_rgb_color_en_pulse       : std_logic := '0';
    signal s_o_clear_info_leds_en_pulse     : std_logic := '0';
    signal s_o_set_info_led_en_pulse        : std_logic := '0';
    signal s_o_set_leds_wave_en_pulse       : std_logic := '0';
    signal s_o_set_leds_side_sensors_en_pulse : std_logic := '0';
    signal s_o_set_leds_front_sensors_en_pulse : std_logic := '0';
    signal s_o_set_leds_blink_en_pulse      : std_logic := '0';
    signal s_o_calibrate_from_config_en_pulse : std_logic := '0';
    signal s_o_debug_from_config_en_pulse   : std_logic := '0';
    signal s_o_menu_config_exit_pulse       : std_logic := '0';

    -- Señales para los valores de salida, que se actualizarán por la lógica de control
    signal s_o_status_led_val               : std_logic := '0';
    signal s_o_warning_status_led_freq      : natural   := 0;
    signal s_o_rgb_r                        : natural range 0 to 255 := 0;
    signal s_o_rgb_g                        : natural range 0 to 255 := 0;
    signal s_o_rgb_b                        : natural range 0 to 255 := 0;
    signal s_o_info_led_idx                 : natural   := 0;
    signal s_o_info_led_val                 : std_logic := '0';
    signal s_o_leds_wave_freq               : natural   := 0;
    signal s_o_leds_side_sensors_freq       : natural   := 0;
    signal s_o_leds_front_sensors_freq      : natural   := 0;
    signal s_o_leds_blink_freq              : natural   := 0;
    signal s_o_calibrate_from_config_val    : natural   := 0;
    signal s_o_debug_from_config_val        : natural   := 0;

begin

    -- =========================================================================
    -- LÓGICA DE DEBOUNCING DE BOTONES Y DETECCIÓN DE FLANCOS
    -- Cada botón tiene su propia lógica de debouncing.
    -- =========================================================================
    -- Debouncing para i_menu_mode_btn
    process (i_clk)
        constant DEBOUNCE_DELAY_CLKS : natural := 10; -- Ejemplo: 10 ciclos de reloj para debouncing
        variable count_mode_btn : natural range 0 to DEBOUNCE_DELAY_CLKS := 0;
        variable prev_mode_btn  : std_logic := '0';
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                count_mode_btn <= 0;
                prev_mode_btn  := '0';
                s_menu_mode_btn_debounced <= '0';
                s_menu_mode_btn_rising_edge <= '0';
            else
                s_menu_mode_btn_rising_edge <= '0'; -- Borrar pulso en cada ciclo

                if i_menu_mode_btn /= prev_mode_btn then
                    count_mode_btn := 0; -- Resetear contador si cambia el estado
                else
                    if count_mode_btn < DEBOUNCE_DELAY_CLKS then
                        count_mode_btn := count_mode_btn + 1;
                    end if;
                end if;

                prev_mode_btn := i_menu_mode_btn;

                if count_mode_btn = DEBOUNCE_DELAY_CLKS then
                    if s_menu_mode_btn_debounced = '0' and i_menu_mode_btn = '1' then
                        s_menu_mode_btn_rising_edge <= '1'; -- Detectar flanco de subida debounced
                    end if;
                    s_menu_mode_btn_debounced <= i_menu_mode_btn;
                end if;
            end if;
        end if;
    end process;

    -- Lógica similar para s_menu_up_btn_debounced y s_menu_down_btn_debounced
    -- (Omitido para brevedad, replicar la estructura anterior para cada botón)
    process (i_clk)
        constant DEBOUNCE_DELAY_CLKS : natural := 10;
        variable count_up_btn : natural range 0 to DEBOUNCE_DELAY_CLKS := 0;
        variable prev_up_btn  : std_logic := '0';
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                count_up_btn <= 0;
                prev_up_btn  := '0';
                s_menu_up_btn_debounced <= '0';
                s_menu_up_btn_rising_edge <= '0';
            else
                s_menu_up_btn_rising_edge <= '0';
                if i_menu_up_btn /= prev_up_btn then
                    count_up_btn := 0;
                else
                    if count_up_btn < DEBOUNCE_DELAY_CLKS then
                        count_up_btn := count_up_btn + 1;
                    end if;
                end if;
                prev_up_btn := i_menu_up_btn;
                if count_up_btn = DEBOUNCE_DELAY_CLKS then
                    if s_menu_up_btn_debounced = '0' and i_menu_up_btn = '1' then
                        s_menu_up_btn_rising_edge <= '1';
                    end if;
                    s_menu_up_btn_debounced <= i_menu_up_btn;
                end if;
            end if;
        end if;
    end process;

    process (i_clk)
        constant DEBOUNCE_DELAY_CLKS : natural := 10;
        variable count_down_btn : natural range 0 to DEBOUNCE_DELAY_CLKS := 0;
        variable prev_down_btn  : std_logic := '0';
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                count_down_btn <= 0;
                prev_down_btn  := '0';
                s_menu_down_btn_debounced <= '0';
                s_menu_down_btn_rising_edge <= '0';
            else
                s_menu_down_btn_rising_edge <= '0';
                if i_menu_down_btn /= prev_down_btn then
                    count_down_btn := 0;
                else
                    if count_down_btn < DEBOUNCE_DELAY_CLKS then
                        count_down_btn := count_down_btn + 1;
                    end if;
                end if;
                prev_down_btn := i_menu_down_btn;
                if count_down_btn = DEBOUNCE_DELAY_CLKS then
                    if s_menu_down_btn_debounced = '0' and i_menu_down_btn = '1' then
                        s_menu_down_btn_rising_edge <= '1';
                    end if;
                    s_menu_down_btn_debounced <= i_menu_down_btn;
                end if;
            end if;
        end if;
    end process;


    -- =========================================================================
    -- PROCESO PRINCIPAL DE LA MÁQUINA DE ESTADOS (menu_config_handler)
    -- =========================================================================
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                current_menu_state <= STATE_HANDLE_MODES_AND_VALUES;
                s_mode_config      <= MODE_CALIBRATION;
                s_value_config     <= (others => 0);
                s_o_menu_config_exit_pulse <= '0';
                s_delay_timer_start_ms <= 0;
                s_mode_btn_press_start_ms <= 0;

                -- Resetear salidas de pulso
                s_o_set_status_led_en_pulse <= '0';
                s_o_warning_status_led_en_pulse <= '0';
                s_o_set_rgb_color_en_pulse <= '0';
                s_o_clear_info_leds_en_pulse <= '0';
                s_o_set_info_led_en_pulse <= '0';
                s_o_set_leds_wave_en_pulse <= '0';
                s_o_set_leds_side_sensors_en_pulse <= '0';
                s_o_set_leds_front_sensors_en_pulse <= '0';
                s_o_set_leds_blink_en_pulse <= '0';
                s_o_calibrate_from_config_en_pulse <= '0';
                s_o_debug_from_config_en_pulse <= '0';

            else
                current_menu_state <= next_menu_state;
                -- Borrar pulsos en cada ciclo
                s_o_set_status_led_en_pulse <= '0';
                s_o_warning_status_led_en_pulse <= '0';
                s_o_set_rgb_color_en_pulse <= '0';
                s_o_clear_info_leds_en_pulse <= '0';
                s_o_set_info_led_en_pulse <= '0';
                s_o_set_leds_wave_en_pulse <= '0';
                s_o_set_leds_side_sensors_en_pulse <= '0';
                s_o_set_leds_front_sensors_en_pulse <= '0';
                s_o_set_leds_blink_en_pulse <= '0';
                s_o_calibrate_from_config_en_pulse <= '0';
                s_o_debug_from_config_en_pulse <= '0';
                s_o_menu_config_exit_pulse <= '0';

                -- Lógica para menu_config_reset_values
                if i_menu_config_reset_values_en = '1' then
                    s_value_config(MODE_CALIBRATION) <= 0;
                    s_value_config(MODE_DEBUG) <= 0;
                end if;

                -- Lógica para menu_config_reset_mode
                if i_menu_config_reset_mode_en = '1' then
                    s_mode_config <= MODE_CALIBRATION;
                end if;

            end if;
        end if;
    end process;

    process (current_menu_state, s_mode_config, s_value_config,
             s_menu_mode_btn_debounced, s_menu_mode_btn_rising_edge,
             s_menu_up_btn_debounced, s_menu_up_btn_rising_edge,
             s_menu_down_btn_debounced, s_menu_down_btn_rising_edge,
             i_get_clock_ticks, s_mode_btn_press_start_ms,
             i_is_debug_enabled, s_delay_timer_start_ms,
             s_long_press_threshold_ms, s_short_delay_ms)
    begin
        next_menu_state <= current_menu_state;

        -- Reset all LED control outputs by default (will be set in handle_menu_config_value_logic and _mode_logic)
        s_o_set_status_led_en_pulse      <= '0';
        s_o_warning_status_led_en_pulse  <= '0';
        s_o_set_rgb_color_en_pulse       <= '0';
        s_o_clear_info_leds_en_pulse     <= '0';
        s_o_set_info_led_en_pulse        <= '0';
        s_o_set_leds_wave_en_pulse       <= '0';
        s_o_set_leds_side_sensors_en_pulse <= '0';
        s_o_set_leds_front_sensors_en_pulse <= '0';
        s_o_set_leds_blink_en_pulse      <= '0';
        s_o_calibrate_from_config_en_pulse <= '0';
        s_o_debug_from_config_en_pulse   <= '0';
        s_o_menu_config_exit_pulse       <= '0';

        -- Default values for non-pulse outputs (combinational, from states)
        s_o_status_led_val               <= '0';
        s_o_warning_status_led_freq      <= 0;
        s_o_rgb_r                        <= 0;
        s_o_rgb_g                        <= 0;
        s_o_rgb_b                        <= 0;
        s_o_info_led_idx                 <= 0;
        s_o_info_led_val                 <= '0';
        s_o_leds_wave_freq               <= 0;
        s_o_leds_side_sensors_freq       <= 0;
        s_o_leds_front_sensors_freq      <= 0;
        s_o_leds_blink_freq              <= 0;
        s_o_calibrate_from_config_val    <= 0;
        s_o_debug_from_config_val        <= 0;

        case current_menu_state is
            when STATE_HANDLE_MODES_AND_VALUES =>
                -- Lógica handle_menu_config_mode
                case s_mode_config is
                    when MODE_CALIBRATION =>
                        s_o_warning_status_led_en_pulse <= '1';
                        s_o_warning_status_led_freq <= 125;
                    when MODE_DEBUG =>
                        s_o_set_status_led_en_pulse <= '1';
                        s_o_status_led_val <= '1';
                    when others => null;
                end case;

                -- Lógica handle_menu_config_value
                case s_mode_config is
                    when MODE_CALIBRATION =>
                        s_o_set_rgb_color_en_pulse <= '1'; s_o_rgb_r <= 0; s_o_rgb_g <= 0; s_o_rgb_b <= 0;
                        case to_calibrate_value_type(s_value_config(s_mode_config)) is
                            when CALIBRATE_NONE =>
                                s_o_clear_info_leds_en_pulse <= '1';
                            when CALIBRATE_GYRO_Z =>
                                s_o_set_leds_wave_en_pulse <= '1'; s_o_leds_wave_freq <= 120;
                            when CALIBRATE_SIDE_SENSORS_OFFSET =>
                                s_o_set_leds_side_sensors_en_pulse <= '1'; s_o_leds_side_sensors_freq <= 120;
                            when CALIBRATE_FRONT_SENSORS =>
                                s_o_set_leds_front_sensors_en_pulse <= '1'; s_o_leds_front_sensors_freq <= 120;
                            when CALIBRATE_STORE_EEPROM =>
                                s_o_set_leds_blink_en_pulse <= '1'; s_o_leds_blink_freq <= 250;
                            when others => null; -- Should not happen if enum is exhaustive
                        end case;
                        s_o_calibrate_from_config_en_pulse <= '1'; s_o_calibrate_from_config_val <= s_value_config(s_mode_config);

                    when MODE_DEBUG =>
                        for i in 1 to C_NUM_VALUES_DEBUG loop
                            -- set_info_led(i - 1, i == valueConfig[modeConfig]);
                            -- Esto requiere una FSM para iterar o un componente de control de LEDs.
                            -- Aquí, asumimos que set_info_led_en habilita y el índice/valor se pasan.
                            if i = s_value_config(s_mode_config) then
                                s_o_set_info_led_en_pulse <= '1'; s_o_info_led_idx <= i-1; s_o_info_led_val <= '1';
                            else
                                -- Asumimos que los LEDs que no coinciden se apagan o se gestionan externamente
                                -- Podríamos emitir un 'clear_info_leds' antes de setear el que corresponde.
                                null; -- Para un control más fino, cada LED necesitaría su propio 'set_info_led_en' y 'set_info_led_val'
                            end if;
                        end loop;
                        s_o_debug_from_config_en_pulse <= '1'; s_o_debug_from_config_val <= s_value_config(s_mode_config);
                    when others => null;
                end case;

                -- Comprueba cambios del modo de configuración (i_menu_mode_btn)
                if s_value_config(s_mode_config) = 0 and s_menu_mode_btn_rising_edge = '1' then
                    s_mode_btn_press_start_ms <= i_get_clock_ticks;
                    next_menu_state <= STATE_MODE_BTN_PRESS_WAIT_RELEASE;
                elsif s_menu_up_btn_rising_edge = '1' and not (s_mode_config = MODE_DEBUG and i_is_debug_enabled = '1') then
                    -- Comprueba aumento de valor de configuración
                    s_value_config(s_mode_config) <= s_value_config(s_mode_config) + 1;
                    case s_mode_config is
                        when MODE_CALIBRATION =>
                            if s_value_config(s_mode_config) > C_NUM_VALUES_CALIBRATION then
                                s_value_config(s_mode_config) <= 0;
                            end if;
                        when MODE_DEBUG =>
                            if s_value_config(s_mode_config) > C_NUM_VALUES_DEBUG then
                                s_value_config(s_mode_config) <= 0;
                            end if;
                        when others => null;
                    end case;
                    s_o_clear_info_leds_en_pulse <= '1'; -- clear_info_leds()
                    s_delay_timer_start_ms <= i_get_clock_ticks;
                    next_menu_state <= STATE_DELAY_AFTER_BTN_PRESS;

                elsif s_menu_down_btn_rising_edge = '1' and not (s_mode_config = MODE_DEBUG and i_is_debug_enabled = '1') then
                    -- Comprueba descenso de valor de configuración
                    if s_value_config(s_mode_config) > 0 then
                        s_value_config(s_mode_config) <= s_value_config(s_mode_config) - 1;
                    end if;
                    s_o_clear_info_leds_en_pulse <= '1'; -- clear_info_leds()
                    s_delay_timer_start_ms <= i_get_clock_ticks;
                    next_menu_state <= STATE_DELAY_AFTER_BTN_PRESS;
                end if;

            when STATE_MODE_BTN_PRESS_WAIT_RELEASE =>
                -- Lógica handle_menu_config_mode/value mientras se mantiene pulsado
                case s_mode_config is
                    when MODE_CALIBRATION =>
                        s_o_warning_status_led_en_pulse <= '1';
                        s_o_warning_status_led_freq <= 125;
                    when MODE_DEBUG =>
                        s_o_set_status_led_en_pulse <= '1';
                        s_o_status_led_val <= '1';
                    when others => null;
                end case;
                -- handle_menu_config_value() - llamada continua
                -- (la lógica combinacional asociada a valueConfig ya lo hace)
                s_o_calibrate_from_config_en_pulse <= '1'; s_o_calibrate_from_config_val <= s_value_config(s_mode_config);
                s_o_debug_from_config_en_pulse <= '1'; s_o_debug_from_config_val <= s_value_config(s_mode_config);

                if s_menu_mode_btn_debounced = '0' then -- Si se ha soltado el botón
                    if (i_get_clock_ticks - s_mode_btn_press_start_ms) >= s_long_press_threshold_ms then
                        next_menu_state <= STATE_MODE_BTN_LONG_PRESS_DETECTED;
                    else
                        next_menu_state <= STATE_MODE_BTN_SHORT_PRESS_DETECTED;
                    end if;
                elsif (i_get_clock_ticks - s_mode_btn_press_start_ms) >= s_long_press_threshold_ms then
                    -- Lógica del "while (get_menu_mode_btn()) { ... }" para pulsación larga
                    s_o_warning_status_led_en_pulse <= '1'; s_o_warning_status_led_freq <= 50;
                else
                    s_o_warning_status_led_en_pulse <= '1'; s_o_warning_status_led_freq <= 125; -- handle_menu_config_mode() normal
                end if;

            when STATE_MODE_BTN_SHORT_PRESS_DETECTED =>
                s_mode_config <= menu_config_mode_type((to_natural(s_mode_config) + 1) mod C_NUM_MODES);
                s_delay_timer_start_ms <= i_get_clock_ticks;
                next_menu_state <= STATE_DELAY_AFTER_BTN_PRESS;

            when STATE_MODE_BTN_LONG_PRESS_DETECTED =>
                s_o_menu_config_exit_pulse <= '1'; -- return true;
                next_menu_state <= STATE_HANDLE_MODES_AND_VALUES; -- Vuelve al inicio o a un estado de reposo

            when STATE_UP_BTN_PRESS_WAIT_RELEASE =>
                -- Lógica handle_menu_config_value/mode mientras se mantiene pulsado
                -- (replicar del estado STATE_HANDLE_MODES_AND_VALUES o modular)
                -- Aquí simplemente esperamos la liberación del botón
                if s_menu_up_btn_debounced = '0' then
                    s_delay_timer_start_ms <= i_get_clock_ticks;
                    next_menu_state <= STATE_DELAY_AFTER_BTN_PRESS;
                end if;

            when STATE_DOWN_BTN_PRESS_WAIT_RELEASE =>
                -- Lógica handle_menu_config_value/mode mientras se mantiene pulsado
                -- (replicar del estado STATE_HANDLE_MODES_AND_VALUES o modular)
                -- Aquí simplemente esperamos la liberación del botón
                if s_menu_down_btn_debounced = '0' then
                    s_delay_timer_start_ms <= i_get_clock_ticks;
                    next_menu_state <= STATE_DELAY_AFTER_BTN_PRESS;
                end if;

            when STATE_DELAY_AFTER_BTN_PRESS =>
                if (i_get_clock_ticks - s_delay_timer_start_ms) >= s_short_delay_ms then
                    next_menu_state <= STATE_HANDLE_MODES_AND_VALUES;
                end if;

            when others =>
                next_menu_state <= STATE_HANDLE_MODES_AND_VALUES; -- Default safety
        end case;
    end process;


    -- =========================================================================
    -- ASIGNACIONES DE SALIDA (Combinacional)
    -- Los valores de salida se toman de las señales de pulso y las señales de valor
    -- =========================================================================
    o_set_status_led_en       <= s_o_set_status_led_en_pulse;
    o_status_led_val          <= s_o_status_led_val;
    o_warning_status_led_en   <= s_o_warning_status_led_en_pulse;
    o_warning_status_led_freq <= s_o_warning_status_led_freq;
    o_set_rgb_color_en        <= s_o_set_rgb_color_en_pulse;
    o_rgb_r                   <= s_o_rgb_r;
    o_rgb_g                   <= s_o_rgb_g;
    o_rgb_b                   <= s_o_rgb_b;
    o_clear_info_leds_en      <= s_o_clear_info_leds_en_pulse;
    o_set_info_led_en         <= s_o_set_info_led_en_pulse;
    o_info_led_idx            <= s_o_info_led_idx;
    o_info_led_val            <= s_o_info_led_val;
    o_set_leds_wave_en        <= s_o_set_leds_wave_en_pulse;
    o_leds_wave_freq          <= s_o_leds_wave_freq;
    o_set_leds_side_sensors_en<= s_o_set_leds_side_sensors_en_pulse;
    o_leds_side_sensors_freq  <= s_o_leds_side_sensors_freq;
    o_set_leds_front_sensors_en <= s_o_set_leds_front_sensors_en_pulse;
    o_leds_front_sensors_freq <= s_o_leds_front_sensors_freq;
    o_set_leds_blink_en       <= s_o_set_leds_blink_en_pulse;
    o_leds_blink_freq         <= s_o_leds_blink_freq;
    o_calibrate_from_config_en <= s_o_calibrate_from_config_en_pulse;
    o_calibrate_from_config_val <= s_o_calibrate_from_config_val;
    o_debug_from_config_en    <= s_o_debug_from_config_en_pulse;
    o_debug_from_config_val   <= s_o_debug_from_config_val;
    o_menu_config_exit        <= s_o_menu_config_exit_pulse;

end architecture Behavioral;
