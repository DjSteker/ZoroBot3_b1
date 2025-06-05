
-- calibrations.vhd
-- Este módulo VHDL gestiona las rutinas de calibración del robot.
-- Se encarga de la interacción del usuario a través del botón de modo de menú
-- y de coordinar las funciones de calibración de los sensores y el giroscopio,
-- así como el guardado de datos en la EEPROM.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.calibrations_pkg.all; -- Paquete de constantes y tipos para calibración

entity calibrations is
    port (
        i_clk                       : in  std_logic;                                 -- Reloj del sistema
        i_rst_n                     : in  std_logic;                                 -- Reset asíncrono activo-bajo
        i_get_clock_ticks           : in  unsigned(31 downto 0);                     -- Contador de milisegundos del sistema

        -- Entradas de control y estado
        i_get_menu_mode_btn         : in  std_logic;                                 -- get_menu_mode_btn() - Botón de modo de menú
        i_get_debug_btn             : in  std_logic;                                 -- get_debug_btn() - Botón de depuración
        i_calibrate_config_type     : in  t_calibrate_type;                          -- calibrate_from_config(type) input
        i_calibrate_manual_trigger  : in  std_logic;                                 -- Trigger para calibrate_manual_distances()
        i_set_debug_btn_ack         : in  std_logic;                                 -- Acknowledge for set_debug_btn(false)

        -- Entradas de datos de los sensores para calibrate_manual_distances
        i_get_sensor_raw_filter_sl  : in  signed(15 downto 0);                       -- get_sensor_raw_filter(SENSOR_SIDE_LEFT_WALL_ID)
        i_get_sensor_raw_filter_fl  : in  signed(15 downto 0);                       -- get_sensor_raw_filter(SENSOR_FRONT_LEFT_WALL_ID)
        i_get_sensor_raw_filter_fr  : in  signed(15 downto 0);                       -- get_sensor_raw_filter(SENSOR_FRONT_RIGHT_WALL_ID)
        i_get_sensor_raw_filter_sr  : in  signed(15 downto 0);                       -- get_sensor_raw_filter(SENSOR_SIDE_RIGHT_WALL_ID)

        -- Salidas/Triggers para funciones externas
        o_clear_info_leds_trigger   : out std_logic;                                 -- clear_info_leds() trigger
        o_set_RGB_color_en          : out std_logic;                                 -- set_RGB_color(r,g,b) enable
        o_set_RGB_color_r           : out unsigned(7 downto 0);                      -- RGB Red component
        o_set_RGB_color_g           : out unsigned(7 downto 0);                      -- RGB Green component
        o_set_RGB_color_b           : out unsigned(7 downto 0);                      -- RGB Blue component
        o_lsm6dsr_gyro_z_calibration_trigger: out std_logic;                         -- lsm6dsr_gyro_z_calibration() trigger
        o_side_sensors_calibration_trigger  : out std_logic;                         -- side_sensors_calibration() trigger
        o_front_sensors_calibration_trigger : out std_logic;                         -- front_sensors_calibration() trigger
        o_eeprom_save_trigger       : out std_logic;                                 -- eeprom_save() trigger
        o_menu_config_reset_values_trigger: out std_logic;                           -- menu_config_reset_values() trigger
        o_set_debug_btn_en          : out std_logic;                                 -- set_debug_btn(false) enable
        o_set_debug_btn_val         : out std_logic;                                 -- set_debug_btn(false) value

        -- Interfaz de impresión (printf)
        o_print_trigger             : out std_logic;                                 -- Pulso para activar una impresión
        o_print_msg_id              : out natural;                                   -- ID del formato del mensaje
        o_print_data                : out t_print_data_array;                        -- Valores de datos a imprimir

        -- Salida de estado interno
        o_calibration_enabled       : out std_logic                                  -- is_calibration_enabled()
    );
end entity calibrations;

architecture rtl of calibrations is

    -- Señales internas (equivalentes a variables 'static' en C)
    signal s_calibration_enabled     : boolean := false;
    signal s_current_calibrate_type  : t_calibrate_type := CALIBRATE_NONE;

    -- Estados de la FSM principal para calibrate_from_config
    type t_cal_fsm_state is (
        S_IDLE,
        S_CHECK_CAL_ACTIVE_PRESS,      -- Esperando la pulsación del botón de modo
        S_CHECK_CAL_ACTIVE_RELEASE,    -- Esperando la liberación del botón de modo
        S_UPDATE_CAL_ENABLED,          -- Actualiza el estado de calibración habilitada
        S_INIT_CALIBRATION,            -- Inicialización común (LEDs, delay)
        S_DELAY_AFTER_INIT,            -- Retardo después de la inicialización
        S_PERFORM_CALIBRATION,         -- Realiza la calibración específica
        S_CALIBRATION_DONE_DELAY,      -- Retardo final después de la calibración
        S_FINAL_STATE                  -- Estado final de la calibración
    );
    signal s_cal_state                 : t_cal_fsm_state := S_IDLE;

    -- Estados de la FSM para calibrate_manual_distances
    type t_manual_cal_fsm_state is (
        S_MANUAL_IDLE,
        S_MANUAL_DELAY_500MS,
        S_MANUAL_PRINT_DATA,
        S_MANUAL_RESET_BTN
    );
    signal s_manual_cal_state          : t_manual_cal_fsm_state := S_MANUAL_IDLE;

    -- Señales para el manejador de retardo no bloqueante
    signal s_delay_start_ms            : unsigned(31 downto 0);
    signal s_delay_duration_ms         : unsigned(31 downto 0);
    signal s_delay_done_pulse          : std_logic;

    -- Señales locales para pulsos de salida (para manejar lógicas de un solo ciclo)
    signal r_clear_info_leds_trigger   : std_logic := '0';
    signal r_set_RGB_color_en          : std_logic := '0';
    signal r_lsm6dsr_gyro_z_calibration_trigger: std_logic := '0';
    signal r_side_sensors_calibration_trigger  : std_logic := '0';
    signal r_front_sensors_calibration_trigger : std_logic := '0';
    signal r_eeprom_save_trigger       : std_logic := '0';
    signal r_menu_config_reset_values_trigger: std_logic := '0';
    signal r_set_debug_btn_en          : std_logic := '0';
    signal r_print_trigger             : std_logic := '0';


begin

    -- Asignación de salidas
    o_calibration_enabled <= '1' when s_calibration_enabled else '0';
    o_clear_info_leds_trigger <= r_clear_info_leds_trigger;
    o_set_RGB_color_en <= r_set_RGB_color_en;
    o_lsm6dsr_gyro_z_calibration_trigger <= r_lsm6dsr_gyro_z_calibration_trigger;
    o_side_sensors_calibration_trigger <= r_side_sensors_calibration_trigger;
    o_front_sensors_calibration_trigger <= r_front_sensors_calibration_trigger;
    o_eeprom_save_trigger <= r_eeprom_save_trigger;
    o_menu_config_reset_values_trigger <= r_menu_config_reset_values_trigger;
    o_set_debug_btn_en <= r_set_debug_btn_en;
    o_set_debug_btn_val <= '0'; -- Always set to false for set_debug_btn(false)
    o_print_trigger <= r_print_trigger;


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


    --- FSM principal para calibrate_from_config ---
    process (i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            s_cal_state                 <= S_IDLE;
            s_calibration_enabled       <= false;
            s_current_calibrate_type    <= CALIBRATE_NONE;
            r_clear_info_leds_trigger   <= '0';
            r_set_RGB_color_en          <= '0';
            o_set_RGB_color_r           <= (others => '0');
            o_set_RGB_color_g           <= (others => '0');
            o_set_RGB_color_b           <= (others => '0');
            r_lsm6dsr_gyro_z_calibration_trigger <= '0';
            r_side_sensors_calibration_trigger   <= '0';
            r_front_sensors_calibration_trigger  <= '0';
            r_eeprom_save_trigger       <= '0';
            r_menu_config_reset_values_trigger <= '0';
        elsif rising_edge(i_clk) then
            -- Limpiar pulsos de salida por defecto en cada ciclo
            r_clear_info_leds_trigger   <= '0';
            r_set_RGB_color_en          <= '0';
            r_lsm6dsr_gyro_z_calibration_trigger <= '0';
            r_side_sensors_calibration_trigger   <= '0';
            r_front_sensors_calibration_trigger  <= '0';
            r_eeprom_save_trigger       <= '0';
            r_menu_config_reset_values_trigger <= '0';
            o_set_RGB_color_r           <= (others => '0'); -- Por defecto a 0
            o_set_RGB_color_g           <= (others => '0');
            o_set_RGB_color_b           <= (others => '0');

            case s_cal_state is
                when S_IDLE =>
                    if i_calibrate_config_type /= CALIBRATE_NONE then
                        s_current_calibrate_type <= i_calibrate_config_type;
                        s_cal_state <= S_CHECK_CAL_ACTIVE_PRESS;
                    else
                        s_calibration_enabled <= false; -- Si no se especifica tipo, deshabilitar calibración
                    end if;

                when S_CHECK_CAL_ACTIVE_PRESS =>
                    -- check_calibration_active() parte 1: Esperar pulsación
                    if i_get_menu_mode_btn = '1' then
                        s_cal_state <= S_CHECK_CAL_ACTIVE_RELEASE;
                    else
                        -- Si no hay pulsación, y ya estaba habilitado, continuar con la calibración.
                        -- Si no estaba habilitado, esperar el botón.
                        if s_calibration_enabled = true then
                             s_cal_state <= S_INIT_CALIBRATION;
                        end if;
                    end if;

                when S_CHECK_CAL_ACTIVE_RELEASE =>
                    -- check_calibration_active() parte 2: Esperar liberación
                    if i_get_menu_mode_btn = '0' then
                        s_cal_state <= S_UPDATE_CAL_ENABLED;
                    end if;

                when S_UPDATE_CAL_ENABLED =>
                    -- check_calibration_active() parte 3: Alternar estado
                    s_calibration_enabled <= not s_calibration_enabled;
                    -- Si se deshabilita, reiniciar valores de menú
                    if s_calibration_enabled = false then
                        r_menu_config_reset_values_trigger <= '1';
                        s_cal_state <= S_IDLE;
                    else
                        -- Si se habilita o sigue habilitado, pasar a la inicialización
                        s_cal_state <= S_INIT_CALIBRATION;
                    end if;

                when S_INIT_CALIBRATION =>
                    if s_calibration_enabled = true then -- Solo si la calibración está habilitada
                        r_clear_info_leds_trigger <= '1'; -- clear_info_leds()
                        r_set_RGB_color_en <= '1';        -- set_RGB_color(0, 125, 0)
                        o_set_RGB_color_g <= to_unsigned(125, 8);
                        s_delay_start_ms <= i_get_clock_ticks;
                        s_delay_duration_ms <= to_unsigned(2000, 32); -- delay(2000)
                        s_cal_state <= S_DELAY_AFTER_INIT;
                    else
                        s_cal_state <= S_IDLE; -- Volver a IDLE si se deshabilitó
                    end if;

                when S_DELAY_AFTER_INIT =>
                    if s_delay_done_pulse = '1' then
                        s_cal_state <= S_PERFORM_CALIBRATION;
                    end if;

                when S_PERFORM_CALIBRATION =>
                    -- switch (type)
                    case s_current_calibrate_type is
                        when CALIBRATE_GYRO_Z =>
                            r_lsm6dsr_gyro_z_calibration_trigger <= '1';
                        when CALIBRATE_SIDE_SENSORS_OFFSET =>
                            r_side_sensors_calibration_trigger <= '1';
                        when CALIBRATE_FRONT_SENSORS =>
                            r_front_sensors_calibration_trigger <= '1';
                        when CALIBRATE_STORE_EEPROM =>
                            r_eeprom_save_trigger <= '1';
                        when others => null; -- CALIBRATE_NONE o cualquier otro, no hace nada aquí
                    end case;
                    -- Después de activar el trigger, pasar al estado final
                    s_cal_state <= S_FINAL_STATE;

                when S_FINAL_STATE =>
                    r_set_RGB_color_en <= '1'; -- set_RGB_color(0, 0, 0)
                    s_calibration_enabled <= false;
                    r_menu_config_reset_values_trigger <= '1';
                    s_cal_state <= S_IDLE;

                when others =>
                    s_cal_state <= S_IDLE; -- Estado de fallback
            end case;
        end if;
    end process;


    --- FSM para calibrate_manual_distances ---
    process (i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            s_manual_cal_state      <= S_MANUAL_IDLE;
            r_set_debug_btn_en      <= '0';
            r_print_trigger         <= '0';
            o_print_msg_id          <= 0;
            o_print_data            <= (others => (others => '0'));
        elsif rising_edge(i_clk) then
            r_set_debug_btn_en      <= '0';
            r_print_trigger         <= '0';
            o_print_msg_id          <= 0;
            o_print_data            <= (others => (others => '0'));

            case s_manual_cal_state is
                when S_MANUAL_IDLE =>
                    if i_calibrate_manual_trigger = '1' then -- Se activa si el botón de debug es pulsado
                        s_manual_cal_state <= S_MANUAL_DELAY_500MS;
                        s_delay_start_ms <= i_get_clock_ticks;
                        s_delay_duration_ms <= to_unsigned(500, 32); -- delay(500)
                    end if;

                when S_MANUAL_DELAY_500MS =>
                    if s_delay_done_pulse = '1' then
                        s_manual_cal_state <= S_MANUAL_PRINT_DATA;
                    end if;

                when S_MANUAL_PRINT_DATA =>
                    r_print_trigger <= '1';
                    o_print_msg_id <= MSG_ID_MANUAL_DISTANCES_FMT;
                    o_print_data(0) <= i_get_sensor_raw_filter_sl;
                    o_print_data(1) <= i_get_sensor_raw_filter_fl;
                    o_print_data(2) <= i_get_sensor_raw_filter_fr;
                    o_print_data(3) <= i_get_sensor_raw_filter_sr;
                    s_manual_cal_state <= S_MANUAL_RESET_BTN;

                when S_MANUAL_RESET_BTN =>
                    r_set_debug_btn_en <= '1'; -- set_debug_btn(false)
                    -- Esperar el ACK de que el botón de debug se ha reseteado externamente
                    if i_set_debug_btn_ack = '1' then -- Necesitamos un ACK para saber que se ha procesado
                        s_manual_cal_state <= S_MANUAL_IDLE;
                    end if;

                when others =>
                    s_manual_cal_state <= S_MANUAL_IDLE;
            end case;
        end if;
    end process;

end architecture rtl;
