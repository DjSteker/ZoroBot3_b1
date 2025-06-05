
-- main.vhd
-- Este es el módulo VHDL de nivel superior que integra y orquesta
-- todos los demás módulos del sistema del robot, simulando la lógica de main.c.
--
-- Aquí se instancian los módulos VHDL generados previamente:
-- - battery.vhd
-- - buttons.vhd
-- - control.vhd
-- - calibrations.vhd
-- - eeprom.vhd
-- - task_scheduler.vhd
--
-- Y se incluyen PLACEHOLDERS para los módulos no traducidos (e.g., encoders, sensors, leds, etc.),
-- con interfaces de puerto supuestas basadas en las llamadas a funciones de C.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.float_pkg.all; -- Para interacción con el módulo de control que usa floats

library work;
use work.robot_types_pkg.all; -- Para constantes y tipos globales
use work.calibrations_pkg.all; -- Para tipos específicos de calibración (e.g., t_calibrate_type)
use work.debug_pkg.all;       -- Para tipos específicos de depuración (e.g., t_debug_type)
use work.control_pkg.all;     -- Para tipos específicos de control (e.g., t_kinematics)

entity main is
    port (
        i_clk                : in  std_logic;                                -- Reloj principal del sistema
        i_rst_n              : in  std_logic;                                -- Reset asíncrono activo-bajo

        -- Entradas ADC/GPIO Raw desde el hardware físico (ej. de un módulo ADC o un pin GPIO)
        i_adc_battery_raw    : in  unsigned(ADC_BITS-1 downto 0);            -- Corresponde a get_aux_raw(AUX_BATTERY_ID)
        i_adc_current_left_raw: in unsigned(ADC_BITS-1 downto 0);            -- Corresponde a get_aux_raw(AUX_CURRENT_LEFT_ID)
        i_adc_current_right_raw: in unsigned(ADC_BITS-1 downto 0);           -- Corresponde a get_aux_raw(AUX_CURRENT_RIGHT_ID)
        i_adc_menu_btn_raw   : in  unsigned(ADC_BITS-1 downto 0);            -- Corresponde a get_aux_raw(AUX_MENU_BTN_ID)
        i_encoder_left_count : in  signed(31 downto 0);                      -- Contador raw del encoder izquierdo
        i_encoder_right_count: in  signed(31 downto 0);                      -- Contador raw del encoder derecho
        i_lsm6dsr_gyro_z_raw : in  signed(15 downto 0);                      -- Lectura raw del giroscopio Z
        i_sensor_front_left_raw_0: in signed(15 downto 0);                   -- Lectura raw del sensor FL (background)
        i_sensor_front_left_raw_1: in signed(15 downto 0);                   -- Lectura raw del sensor FL (with light)
        i_sensor_front_right_raw_0: in signed(15 downto 0);                  -- Lectura raw del sensor FR (background)
        i_sensor_front_right_raw_1: in signed(15 downto 0);                  -- Lectura raw del sensor FR (with light)
        i_sensor_side_left_raw_0: in signed(15 downto 0);                    -- Lectura raw del sensor SL (background)
        i_sensor_side_left_raw_1: in signed(15 downto 0);                    -- Lectura raw del sensor SL (with light)
        i_sensor_side_right_raw_0: in signed(15 downto 0);                   -- Lectura raw del sensor SR (background)
        i_sensor_side_right_raw_1: in signed(15 downto 0);                   -- Lectura raw del sensor SR (with light)
        i_usart_rx_data      : in  std_logic_vector(7 downto 0);             -- Datos de RX de USART
        i_usart_rx_valid     : in  std_logic;                                -- Válido de RX de USART

        -- Salidas PWM y GPIO para el hardware físico
        o_motor_left_pwm     : out signed(15 downto 0);                      -- PWM para motor izquierdo
        o_motor_right_pwm    : out signed(15 downto 0);                      -- PWM para motor derecho
        o_motor_enable_out   : out std_logic;                                -- Habilitar motores
        o_motor_brake_out    : out std_logic;                                -- Freno de motores
        o_fan_speed_pwm      : out signed(15 downto 0);                      -- PWM para ventilador
        o_led_status         : out std_logic;                                -- LED de estado
        o_leds_battery_level_out: out unsigned(7 downto 0);                  -- Nivel de batería para LEDs
        o_leds_battery_level_en_out: out std_logic;                          -- Habilitar LEDs de batería
        o_info_leds_clear_out: out std_logic;                                -- Limpiar LEDs de información
        o_rgb_r_out          : out unsigned(7 downto 0);                     -- Componente R del LED RGB
        o_rgb_g_out          : out unsigned(7 downto 0);                     -- Componente G del LED RGB
        o_rgb_b_out          : out unsigned(7 downto 0);                     -- Componente B del LED RGB
        o_rgb_color_en_out   : out std_logic;                                -- Habilitar LED RGB
        o_gpio_b13_out       : out std_logic;                                -- GPIO_B13 (para pulso en control_loop)
        o_gpio_b15_out       : out std_logic;                                -- GPIO_B15 (para pulso en keep_z_angle debug)
        o_usart_tx_data      : out std_logic_vector(7 downto 0);             -- Datos de TX de USART
        o_usart_tx_valid     : out std_logic                                 -- Válido de TX de USART
    );
end entity main;

architecture rtl of main is

    -- =========================================================================
    -- Señales Globales del Sistema (equivalentes a variables globales en C)
    -- =========================================================================
    signal s_global_ms_ticks        : unsigned(31 downto 0); -- get_clock_ticks()
    signal s_global_us_ticks        : unsigned(31 downto 0); -- get_us_counter() (for task_scheduler)
    signal s_control_loop_trigger   : std_logic := '0';      -- Trigger para control_loop (e.g., 1ms)
    signal s_sys_tick_trigger       : std_logic := '0';      -- Trigger para sys_tick_handler (e.g., 1ms)
    signal s_aux_raw_val            : unsigned(ADC_BITS-1 downto 0); -- Valor actual del ADC auxiliar
    signal s_aux_raw_channel_sel    : natural range 0 to AUX_MENU_BTN_ID := 0; -- Canal ADC a leer
    signal s_get_aux_raw_trigger    : std_logic := '0'; -- Trigger para el módulo aux_input_controller

    -- =========================================================================
    -- Señales para interconexión entre módulos
    -- (Los nombres siguen la convención o_modulename_signal y i_modulename_signal)
    -- =========================================================================

    -- battery.vhd
    signal s_batt_voltage               : unsigned(23 downto 0);
    signal s_batt_high_limit_voltage    : unsigned(23 downto 0);
    signal s_batt_update_trigger        : std_logic := '0';

    -- buttons.vhd
    signal s_btn_menu_up                : std_logic;
    signal s_btn_menu_down              : std_logic;
    signal s_btn_menu_mode              : std_logic;
    signal s_btn_debug                  : std_logic;
    signal s_set_debug_btn_en           : std_logic := '0';
    signal s_set_debug_btn_val          : std_logic := '0';

    -- control.vhd
    signal s_is_race_started            : std_logic;
    signal s_is_front_sensors_correction_enabled: std_logic;
    signal s_ideal_linear_speed         : signed(31 downto 0);
    signal s_ideal_angular_speed        : float32;
    signal s_check_start_run_result     : signed(7 downto 0);
    signal s_set_motors_pwm_en_ctrl     : std_logic;
    signal s_set_motors_pwm_left_ctrl   : signed(15 downto 0);
    signal s_set_motors_pwm_right_ctrl  : signed(15 downto 0);
    signal s_set_motors_enable_en_ctrl  : std_logic;
    signal s_set_motors_enable_val_ctrl : std_logic;
    signal s_set_motors_brake_trigger_ctrl: std_logic;
    signal s_set_fan_speed_en_ctrl      : std_logic;
    signal s_set_fan_speed_val_ctrl     : signed(15 downto 0);
    signal s_set_rgb_color_en_ctrl      : std_logic;
    signal s_set_rgb_color_r_ctrl       : unsigned(7 downto 0);
    signal s_set_rgb_color_g_ctrl       : unsigned(7 downto 0);
    signal s_set_rgb_color_b_ctrl       : unsigned(7 downto 0);
    signal s_blink_rgb_color_trigger_ctrl: std_logic;
    signal s_blink_rgb_color_r_ctrl     : unsigned(7 downto 0);
    signal s_blink_rgb_color_g_ctrl     : unsigned(7 downto 0);
    signal s_blink_rgb_color_b_ctrl     : unsigned(7 downto 0);
    signal s_blink_rgb_color_ms_ctrl    : unsigned(31 downto 0);
    signal s_menu_reset_trigger_ctrl    : std_logic;
    signal s_menu_run_reset_trigger_ctrl: std_logic;
    signal s_reset_motors_saturated_trigger_ctrl: std_logic;
    signal s_reset_encoder_avg_trigger_ctrl: std_logic;
    signal s_reset_control_all_trigger_ctrl: std_logic;
    signal s_gpio_13_toggle_ctrl        : std_logic;
    signal s_gpio_15_set_ctrl           : std_logic;
    -- Signals to pass to control module for external function calls
    signal s_ctrl_kinematics            : t_kinematics;
    signal s_ctrl_get_encoder_left_speed: float32;
    signal s_ctrl_get_encoder_right_speed: float32;
    signal s_ctrl_lsm6dsr_get_gyro_z_radps: float32;
    signal s_ctrl_get_sensor_distance_fl: signed(15 downto 0);
    signal s_ctrl_get_sensor_distance_fr: signed(15 downto 0);
    signal s_ctrl_get_side_sensors_error: float32;
    signal s_ctrl_get_front_sensors_angle_error: float32;
    signal s_ctrl_get_front_sensors_diagonal_error: float32;
    signal s_ctrl_is_debug_enabled      : std_logic;
    signal s_ctrl_is_motor_saturated    : std_logic;
    signal s_ctrl_get_motors_saturated_ms: unsigned(31 downto 0);
    signal s_ctrl_get_encoder_avg_millimeters: float32;
    signal s_ctrl_get_wall_lost_toggle_state: std_logic;
    signal s_ctrl_get_cell_change_toggle_state: std_logic;
    signal s_set_target_linear_speed_en_ctrl : std_logic := '0';
    signal s_target_linear_speed_val_ctrl    : signed(31 downto 0) := (others => '0');
    signal s_set_ideal_angular_speed_en_ctrl : std_logic := '0';
    signal s_ideal_angular_speed_val_ctrl    : float32 := to_float32(0.0);
    signal s_set_target_fan_speed_en_ctrl    : std_logic := '0';
    signal s_target_fan_speed_val_ctrl       : signed(31 downto 0) := (others => '0');
    signal s_target_fan_speed_ms_ctrl        : unsigned(31 downto 0) := (others => '0');
    signal s_set_race_started_en_ctrl        : std_logic := '0';
    signal s_set_race_started_val_ctrl       : std_logic := '0';
    signal s_set_control_debug_en_ctrl       : std_logic := '0';
    signal s_set_control_debug_val_ctrl      : std_logic := '0';
    signal s_set_side_sensors_close_correction_en_ctrl: std_logic := '0';
    signal s_set_side_sensors_close_correction_val_ctrl: std_logic := '0';
    signal s_set_side_sensors_far_correction_en_ctrl: std_logic := '0';
    signal s_set_side_sensors_far_correction_val_ctrl: std_logic := '0';
    signal s_set_front_sensors_correction_en_ctrl: std_logic := '0';
    signal s_set_front_sensors_correction_val_ctrl: std_logic := '0';
    signal s_set_front_sensors_diagonal_correction_en_ctrl: std_logic := '0';
    signal s_set_front_sensors_diagonal_correction_val_ctrl: std_logic := '0';
    signal s_disable_sensors_correction_trigger_ctrl: std_logic := '0';
    signal s_reset_control_errors_trigger_ctrl: std_logic := '0';
    signal s_reset_control_speed_trigger_ctrl: std_logic := '0';
    signal s_keep_z_angle_trigger_ctrl: std_logic := '0';


    -- calibrations.vhd
    signal s_cal_clear_info_leds_trigger: std_logic;
    signal s_cal_set_rgb_color_en   : std_logic;
    signal s_cal_set_rgb_color_r    : unsigned(7 downto 0);
    signal s_cal_set_rgb_color_g    : unsigned(7 downto 0);
    signal s_cal_set_rgb_color_b    : unsigned(7 downto 0);
    signal s_cal_lsm6dsr_gyro_z_calibration_trigger: std_logic;
    signal s_cal_side_sensors_calibration_trigger: std_logic;
    signal s_cal_front_sensors_calibration_trigger: std_logic;
    signal s_cal_eeprom_save_trigger: std_logic;
    signal s_cal_menu_config_reset_values_trigger: std_logic;
    signal s_cal_set_debug_btn_en   : std_logic;
    signal s_cal_set_debug_btn_val  : std_logic;
    signal s_cal_print_trigger      : std_logic;
    signal s_cal_print_msg_id       : natural;
    signal s_cal_print_data         : t_print_data_array;
    signal s_calibration_enabled    : std_logic;
    -- Signals to pass to calibrations module
    signal s_cal_get_sensor_raw_filter_sl: signed(15 downto 0);
    signal s_cal_get_sensor_raw_filter_fl: signed(15 downto 0);
    signal s_cal_get_sensor_raw_filter_fr: signed(15 downto 0);
    signal s_cal_get_sensor_raw_filter_sr: signed(15 downto 0);
    signal s_calibrate_config_type  : t_calibrate_type := CALIBRATE_NONE; -- from menu
    signal s_calibrate_manual_trigger: std_logic := '0'; -- from debug_btn
    signal s_set_debug_btn_ack_cal  : std_logic := '0'; -- ACK for set_debug_btn

    -- debug.vhd
    signal s_debug_macroarray_print_trigger: std_logic;
    signal s_debug_floodfill_maze_print_trigger: std_logic;
    signal s_debug_menu_config_reset_values_trigger: std_logic;
    signal s_debug_set_sensors_enabled_en: std_logic;
    signal s_debug_set_sensors_enabled_val: std_logic;
    signal s_debug_set_motors_enable_en: std_logic;
    signal s_debug_set_motors_enable_val: std_logic;
    signal s_debug_set_motors_speed_en: std_logic;
    signal s_debug_set_motors_speed_left: signed(15 downto 0);
    signal s_debug_set_motors_speed_right: signed(15 downto 0);
    signal s_debug_reset_control_all_trigger: std_logic;
    signal s_debug_keep_z_angle_trigger: std_logic;
    signal s_debug_set_fan_speed_en: std_logic;
    signal s_debug_set_fan_speed_val: signed(15 downto 0);
    signal s_debug_set_rgb_color_en: std_logic;
    signal s_debug_set_rgb_color_r: unsigned(7 downto 0);
    signal s_debug_set_rgb_color_g: unsigned(7 downto 0);
    signal s_debug_set_rgb_color_b: unsigned(7 downto 0);
    signal s_debug_print_trigger  : std_logic;
    signal s_debug_print_msg_id   : natural;
    signal s_debug_print_data     : t_print_data_array;
    signal s_debug_enabled        : std_logic;
    -- Signals to pass to debug module
    signal s_debug_config_type    : t_debug_type := DEBUG_NONE; -- from menu
    signal s_debug_main_trigger   : std_logic := '0'; -- from setup for main loop

    -- eeprom.vhd
    signal s_eeprom_load_trigger        : std_logic := '0';
    signal s_eeprom_save_trigger        : std_logic; -- from calibrations
    signal s_eeprom_clear_trigger       : std_logic := '0';
    signal s_eeprom_restore_trigger     : std_logic := '0';
    signal s_eeprom_backup_trigger      : std_logic := '0';
    signal s_eeprom_set_data_en         : std_logic := '0';
    signal s_eeprom_set_data_index      : natural range 0 to DATA_LENGTH - 1 := 0;
    signal s_eeprom_set_data_value      : signed(15 downto 0) := (others => '0');
    signal s_eeprom_set_data_write_en   : std_logic := '0';
    signal s_eeprom_read_data_index     : natural range 0 to DATA_LENGTH - 1 := 0;
    signal s_eeprom_read_data_value     : signed(15 downto 0);
    signal s_eeprom_lsm6dsr_load_eeprom_en : std_logic;
    signal s_eeprom_sensors_load_eeprom_en : std_logic;
    signal s_eeprom_floodfill_load_maze_en : std_logic;
    signal s_eeprom_menu_run_load_values_en: std_logic;
    signal s_eeprom_rc5_load_eeprom_en  : std_logic;
    signal s_eeprom_set_status_led_en   : std_logic;
    signal s_eeprom_status_led_val      : std_logic;
    signal s_eeprom_toggle_status_led_en: std_logic;

    -- task_scheduler.vhd
    constant C_SYS_TICK_TASKS_COUNT : natural := 6; -- Number of tasks in sys_tick_handler
    signal s_scheduler_set_interval : std_logic_vector(C_SYS_TICK_TASKS_COUNT-1 downto 0);
    signal s_scheduler_interval     : unsigned(31 downto 0);
    signal s_scheduler_timer_idx    : natural range 0 to C_SYS_TICK_TASKS_COUNT-1;
    signal s_scheduler_reset_timer  : std_logic_vector(C_SYS_TICK_TASKS_COUNT-1 downto 0);
    signal s_scheduler_enable_timer : std_logic_vector(C_SYS_TICK_TASKS_COUNT-1 downto 0);
    signal s_scheduler_disable_timer: std_logic_vector(C_SYS_TICK_TASKS_COUNT-1 downto 0);
    signal s_scheduler_task_trigger : std_logic_vector(C_SYS_TICK_TASKS_COUNT-1 downto 0);

    -- Task IDs for sys_tick_handler
    constant TASK_ID_ENCODERS_UPDATE    : natural := 0;
    constant TASK_ID_SENSORS_UPDATE     : natural := 1;
    constant TASK_ID_BATTERY_UPDATE     : natural := 2;
    constant TASK_ID_LEDS_CHECK         : natural := 3;
    constant TASK_ID_BUTTONS_CHECK      : natural := 4;
    constant TASK_ID_LSM6DSR_UPDATE     : natural := 5;
    constant TASK_ID_CONTROL_LOOP       : natural := 6; -- Periodic trigger for control_loop (e.g. 1ms)

    -- =========================================================================
    -- PLACEHOLDER MODULES (Interfaces assumed based on C code)
    -- Estos módulos necesitarán ser implementados por separado.
    -- =========================================================================

    -- aux_input_controller.vhd (for get_aux_raw)
    signal s_aux_input_raw_out      : unsigned(ADC_BITS-1 downto 0);

    -- encoder_controller.vhd
    signal s_enc_update_trigger         : std_logic := '0';
    signal s_enc_left_speed             : float32;
    signal s_enc_right_speed            : float32;
    signal s_enc_reset_avg_trigger      : std_logic := '0';
    signal s_enc_left_millimeters       : signed(31 downto 0);
    signal s_enc_right_millimeters      : signed(31 downto 0);
    signal s_enc_avg_millimeters        : float32;

    -- sensor_controller.vhd
    signal s_sens_update_magics_trigger : std_logic := '0';
    signal s_sens_enabled               : std_logic;
    signal s_sens_set_enabled_en        : std_logic := '0';
    signal s_sens_set_enabled_val       : std_logic := '0';
    signal s_sens_raw_sl_0              : signed(15 downto 0);
    signal s_sens_raw_sl_1              : signed(15 downto 0);
    signal s_sens_raw_filter_sl         : signed(15 downto 0);
    signal s_sens_raw_fl_0              : signed(15 downto 0);
    signal s_sens_raw_fl_1              : signed(15 downto 0);
    signal s_sens_raw_filter_fl         : signed(15 downto 0);
    signal s_sens_raw_fr_0              : signed(15 downto 0);
    signal s_sens_raw_fr_1              : signed(15 downto 0);
    signal s_sens_raw_filter_fr         : signed(15 downto 0);
    signal s_sens_raw_sr_0              : signed(15 downto 0);
    signal s_sens_raw_sr_1              : signed(15 downto 0);
    signal s_sens_raw_filter_sr         : signed(15 downto 0);
    signal s_sens_distance_sl           : signed(15 downto 0);
    signal s_sens_distance_fl           : signed(15 downto 0);
    signal s_sens_distance_fr           : signed(15 downto 0);
    signal s_sens_distance_sr           : signed(15 downto 0);
    signal s_sens_side_sensors_error    : float32;
    signal s_sens_front_sensors_angle_error: float32;
    signal s_sens_front_sensors_diagonal_error: float32;
    signal s_sens_wall_lost_toggle_state: std_logic;
    signal s_sens_cell_change_toggle_state: std_logic;
    signal s_sens_side_calibration_trigger: std_logic := '0';
    signal s_sens_front_calibration_trigger: std_logic := '0';

    -- leds_controller.vhd
    signal s_leds_check_while_trigger   : std_logic := '0';
    signal s_led_status_val             : std_logic;
    signal s_led_status_en              : std_logic := '0';
    signal s_led_status_toggle_en       : std_logic := '0';
    signal s_warning_status_led_trigger : std_logic := '0';
    signal s_warning_status_led_val     : unsigned(7 downto 0);
    signal s_set_leds_wave_trigger      : std_logic := '0';
    signal s_set_leds_wave_val          : unsigned(7 downto 0);
    signal s_set_rgb_color_r_leds       : unsigned(7 downto 0);
    signal s_set_rgb_color_g_leds       : unsigned(7 downto 0);
    signal s_set_rgb_color_b_leds       : unsigned(7 downto 0);
    signal s_set_rgb_color_en_leds      : std_logic := '0';
    signal s_delay_ms_leds              : unsigned(31 downto 0);
    signal s_set_rgb_rainbow_trigger    : std_logic := '0';
    signal s_blink_rgb_color_r_leds     : unsigned(7 downto 0);
    signal s_blink_rgb_color_g_leds     : unsigned(7 downto 0);
    signal s_blink_rgb_color_b_leds     : unsigned(7 downto 0);
    signal s_blink_rgb_color_ms_leds    : unsigned(31 downto 0);
    signal s_blink_rgb_color_trigger_leds: std_logic := '0';

    -- lsm6dsr_controller.vhd
    signal s_lsm6dsr_update_trigger     : std_logic := '0';
    signal s_lsm6dsr_who_am_i_out       : std_logic_vector(7 downto 0);
    signal s_lsm6dsr_gyro_z_raw_out     : signed(15 downto 0);
    signal s_lsm6dsr_gyro_z_radps_out   : float32;
    signal s_lsm6dsr_gyro_z_dps_out     : float32;
    signal s_lsm6dsr_gyro_z_degrees_out : float32;
    signal s_lsm6dsr_gyro_z_calibration_trigger: std_logic := '0';

    -- macroarray_controller.vhd
    signal s_macroarray_print_trigger   : std_logic := '0';
    signal s_macroarray_store_trigger   : std_logic := '0';
    signal s_macroarray_store_group     : natural;
    signal s_macroarray_store_labels_mask: std_logic_vector(10 downto 0); -- Adjusted width for 0b000110001
    signal s_macroarray_store_labels    : integer;
    signal s_macroarray_store_length    : natural;
    signal s_macroarray_store_data      : t_print_data_array; -- Re-using print_data_array for macroarray data

    -- menu_controller.vhd
    signal s_menu_handler_trigger       : std_logic := '0';
    signal s_menu_reset_trigger_menu    : std_logic := '0';
    signal s_menu_run_can_start_out     : std_logic;
    signal s_menu_run_get_explore_algorithm_out: explore_algorithm_type;
    signal s_menu_run_reset_trigger_menu: std_logic := '0';
    signal s_menu_config_reset_values_trigger_menu: std_logic := '0';
    signal s_menu_get_menu_mode_btn_in  : std_logic;
    signal s_menu_get_menu_up_btn_in    : std_logic;
    signal s_menu_get_menu_down_btn_in  : std_logic;
    signal s_menu_is_race_started_in    : std_logic;
    signal s_menu_cal_config_type_out   : t_calibrate_type;
    signal s_menu_debug_config_type_out : t_debug_type;


    -- motors_controller.vhd
    signal s_motors_enable_motors       : std_logic := '0';
    signal s_motors_speed_en            : std_logic := '0';
    signal s_motors_speed_left          : signed(15 downto 0);
    signal s_motors_speed_right         : signed(15 downto 0);
    signal s_motors_pwm_en              : std_logic := '0';
    signal s_motors_pwm_left            : signed(15 downto 0);
    signal s_motors_pwm_right           : signed(15 downto 0);
    signal s_motors_brake_trigger       : std_logic := '0';
    signal s_motors_reset_saturated_trigger: std_logic := '0';
    signal s_motors_is_saturated        : std_logic;
    signal s_motors_saturated_ms        : unsigned(31 downto 0);

    -- floodfill_controller.vhd
    signal s_floodfill_start_run_trigger    : std_logic := '0';
    signal s_floodfill_start_explore_trigger: std_logic := '0';
    signal s_floodfill_loop_trigger     : std_logic := '0';
    signal s_floodfill_load_maze_trigger: std_logic := '0';
    signal s_floodfill_maze_print_trigger: std_logic := '0';

    -- handwall_controller.vhd
    signal s_handwall_use_left_hand_trigger: std_logic := '0';
    signal s_handwall_use_right_hand_trigger: std_logic := '0';
    signal s_handwall_start_trigger     : std_logic := '0';
    signal s_handwall_loop_trigger      : std_logic := '0';

    -- timetrial_controller.vhd
    signal s_timetrial_start_trigger    : std_logic := '0';
    signal s_timetrial_loop_trigger     : std_logic := '0';

    -- rc5_controller.vhd
    signal s_rc5_load_eeprom_trigger    : std_logic := '0';

    -- setup_controller.vhd
    signal s_setup_trigger              : std_logic := '0';
    signal s_setup_done                 : std_logic; -- Indicates setup() is complete

    -- usart_tx_controller.vhd (for printf)
    signal s_usart_print_trigger        : std_logic := '0';
    signal s_usart_print_msg_id         : natural;
    signal s_usart_print_data           : t_print_data_array;
    signal s_usart_tx_ready_to_send     : std_logic; -- From UART module, indicates it's ready for new data

    -- gpio_controller.vhd
    signal s_gpio_set_en                : std_logic := '0';
    signal s_gpio_clear_en              : std_logic := '0';
    signal s_gpio_id                    : natural;

    -- =========================================================================
    -- Main Loop FSM
    -- =========================================================================
    type t_main_fsm_state is (
        S_INIT,                      -- Initial setup phase (main.c setup())
        S_INIT_EEPROM_LOAD,          -- Call eeprom_load()
        S_INIT_SHOW_BATTERY_LEVEL,   -- Call show_battery_level()
        S_INIT_PRINT_AUX_RAW,        -- First printf in main
        S_MAIN_LOOP_IDLE,            -- Enter the main while(1) loop
        S_MAIN_LOOP_NOT_STARTED,     -- !is_race_started() branch
        S_MAIN_LOOP_RACE_STARTED,    -- is_race_started() branch
        S_DELAY_200MS_SENSORS_ENABLE, -- delay(200) for set_sensors_enabled
        S_MAIN_LOOP_DEBUG_ZONE,      -- Placeholder for the commented-out debug zone
        S_DELAY_DEBUG_LOOP           -- Delay inside the debug loop
    );
    signal s_main_state             : t_main_fsm_state := S_INIT;
    signal s_main_delay_start_ms    : unsigned(31 downto 0);
    signal s_main_delay_duration_ms : unsigned(31 downto 0);
    signal s_main_delay_done_pulse  : std_logic;

    -- =========================================================================
    -- Millisecond and Microsecond Counters
    -- =========================================================================
    -- Assuming a 100 MHz clock (10 ns period)
    constant C_CLOCK_HZ         : natural := 100_000_000;
    constant C_MS_COUNTER_LIMIT : natural := C_CLOCK_HZ / 1000; -- Ticks per millisecond
    constant C_US_COUNTER_LIMIT : natural := C_CLOCK_HZ / 1_000_000; -- Ticks per microsecond

    signal r_ms_tick_counter    : natural range 0 to C_MS_COUNTER_LIMIT-1 := 0;
    signal r_us_tick_counter    : natural range 0 to C_US_COUNTER_LIMIT-1 := 0;


begin

    -- =========================================================================
    -- Instantiation of Modules
    -- =========================================================================

    -- Global Millisecond Counter
    process (i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            r_ms_tick_counter <= 0;
            s_global_ms_ticks <= (others => '0');
            s_sys_tick_trigger <= '0';
        elsif rising_edge(i_clk) then
            s_sys_tick_trigger <= '0'; -- Clear trigger each cycle
            if r_ms_tick_counter = C_MS_COUNTER_LIMIT - 1 then
                r_ms_tick_counter <= 0;
                s_global_ms_ticks <= s_global_ms_ticks + 1; -- Increment global ms counter
                s_sys_tick_trigger <= '1'; -- 1ms tick
            else
                r_ms_tick_counter <= r_ms_tick_counter + 1;
            end if;
        end if;
    end process;

    -- Global Microsecond Counter (for task_scheduler and control/delay_us needs)
    process (i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            r_us_tick_counter <= 0;
            s_global_us_ticks <= (others => '0');
        elsif rising_edge(i_clk) then
            if r_us_tick_counter = C_US_COUNTER_LIMIT - 1 then
                r_us_tick_counter <= 0;
                s_global_us_ticks <= s_global_us_ticks + 1; -- Increment global us counter
            else
                r_us_tick_counter <= r_us_tick_counter + 1;
            end if;
        end if;
    end process;

    -- Non-blocking Delay for Main FSM (similar to what's in other modules)
    process (i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            s_main_delay_done_pulse <= '0';
        elsif rising_edge(i_clk) then
            s_main_delay_done_pulse <= '0'; -- Clear pulse each cycle

            if s_main_delay_duration_ms /= (others => '0') then -- Delay is active
                if s_global_ms_ticks >= s_main_delay_start_ms then
                    if (s_global_ms_ticks - s_main_delay_start_ms) >= s_main_delay_duration_ms then
                        s_main_delay_done_pulse <= '1';
                        s_main_delay_duration_ms <= (others => '0'); -- Mark delay as finished
                    end if;
                else -- Overflow condition
                    if (to_unsigned(2**32-1, 32) - s_main_delay_start_ms) + s_global_ms_ticks >= s_main_delay_duration_ms then
                        s_main_delay_done_pulse <= '1';
                        s_main_delay_duration_ms <= (others => '0');
                    end if;
                end if;
            end if;
        end if;
    end process;


    -- Battery Module
    U_BATTERY : entity work.battery
        port map (
            i_clk                       => i_clk,
            i_rst_n                     => i_rst_n,
            i_get_clock_ticks           => s_global_ms_ticks,
            i_aux_battery_raw           => i_adc_battery_raw, -- Connected to physical ADC input
            i_show_battery_level_trigger=> '0', -- Controlled by main FSM
            o_set_leds_battery_level    => o_leds_battery_level_out,
            o_set_leds_battery_level_en => o_leds_battery_level_en_out,
            o_all_leds_clear_trigger    => o_info_leds_clear_out, -- Re-use for all_leds_clear
            o_battery_voltage           => s_batt_voltage,
            o_battery_high_limit_voltage=> s_batt_high_limit_voltage
        );

    -- Buttons Module
    U_BUTTONS : entity work.buttons
        port map (
            i_clk             => i_clk,
            i_rst_n           => i_rst_n,
            i_get_clock_ticks => s_global_ms_ticks,
            i_btn_analog_raw  => i_adc_menu_btn_raw, -- Connected to physical ADC input
            i_set_debug_btn_en=> s_set_debug_btn_en,
            i_set_debug_btn_val=> s_set_debug_btn_val,
            o_get_menu_up_btn   => s_btn_menu_up,
            o_get_menu_down_btn => s_btn_menu_down,
            o_get_menu_mode_btn => s_btn_menu_mode,
            o_get_debug_btn     => s_btn_debug
        );

    -- Control Module
    U_CONTROL : entity work.control
        port map (
            i_clk                       => i_clk,
            i_rst_n                     => i_rst_n,
            i_get_clock_ticks           => s_global_ms_ticks,
            i_control_loop_trigger      => s_control_loop_trigger, -- Triggered periodically by scheduler
            i_get_battery_voltage       => std_logic_vector_to_float32(resize(s_batt_voltage, 32)), -- Fixed-point to Float conversion. Por ajustar: explicit conversion needed.
            i_get_battery_high_limit_voltage => std_logic_vector_to_float32(resize(s_batt_high_limit_voltage, 32)), -- Por ajustar
            i_kinematics                => s_ctrl_kinematics, -- From menu or config
            i_get_encoder_left_speed    => s_ctrl_get_encoder_left_speed, -- From encoder module
            i_get_encoder_right_speed   => s_ctrl_get_encoder_right_speed, -- From encoder module
            i_lsm6dsr_get_gyro_z_radps  => s_ctrl_lsm6dsr_get_gyro_z_radps, -- From LSM6DSR module
            i_get_sensor_distance_fl    => s_sens_distance_fl, -- From sensors module
            i_get_sensor_distance_fr    => s_sens_distance_fr, -- From sensors module
            i_get_side_sensors_error    => s_sens_side_sensors_error, -- From sensors module
            i_get_front_sensors_angle_error => s_sens_front_sensors_angle_error, -- From sensors module
            i_get_front_sensors_diagonal_error => s_sens_front_sensors_diagonal_error, -- From sensors module
            i_is_debug_enabled          => s_debug_enabled, -- From debug module
            i_is_motor_saturated        => s_motors_is_saturated, -- From motors module
            i_get_motors_saturated_ms   => s_motors_saturated_ms, -- From motors module
            i_get_encoder_avg_millimeters => s_enc_avg_millimeters, -- From encoders module
            i_get_wall_lost_toggle_state => s_sens_wall_lost_toggle_state, -- From sensors module
            i_get_cell_change_toggle_state => s_sens_cell_change_toggle_state, -- From sensors module
            i_set_target_linear_speed_en => s_set_target_linear_speed_en_ctrl,
            i_target_linear_speed_val    => s_target_linear_speed_val_ctrl,
            i_set_ideal_angular_speed_en => s_set_ideal_angular_speed_en_ctrl,
            i_ideal_angular_speed_val    => s_ideal_angular_speed_val_ctrl,
            i_set_target_fan_speed_en    => s_set_target_fan_speed_en_ctrl,
            i_target_fan_speed_val       => s_target_fan_speed_val_ctrl,
            i_target_fan_speed_ms        => s_target_fan_speed_ms_ctrl,
            i_set_race_started_en        => s_set_race_started_en_ctrl,
            i_set_race_started_val       => s_set_race_started_val_ctrl,
            i_set_control_debug_en       => s_set_control_debug_en_ctrl,
            i_set_control_debug_val      => s_set_control_debug_val_ctrl,
            i_set_side_sensors_close_correction_en => s_set_side_sensors_close_correction_en_ctrl,
            i_set_side_sensors_close_correction_val => s_set_side_sensors_close_correction_val_ctrl,
            i_set_side_sensors_far_correction_en => s_set_side_sensors_far_correction_en_ctrl,
            i_set_side_sensors_far_correction_val => s_set_side_sensors_far_correction_val_ctrl,
            i_set_front_sensors_correction_en => s_set_front_sensors_correction_en_ctrl,
            i_set_front_sensors_correction_val => s_set_front_sensors_correction_val_ctrl,
            i_set_front_sensors_diagonal_correction_en => s_set_front_sensors_diagonal_correction_en_ctrl,
            i_set_front_sensors_diagonal_correction_val => s_set_front_sensors_diagonal_correction_val_ctrl,
            i_disable_sensors_correction_trigger => s_disable_sensors_correction_trigger_ctrl,
            i_reset_control_errors_trigger => s_reset_control_errors_trigger_ctrl,
            i_reset_control_speed_trigger => s_reset_control_speed_trigger_ctrl,
            i_reset_control_all_trigger => s_reset_control_all_trigger_ctrl,
            i_keep_z_angle_trigger      => s_keep_z_angle_trigger_ctrl,
            o_set_motors_pwm_en         => s_set_motors_pwm_en_ctrl,
            o_set_motors_pwm_left       => s_set_motors_pwm_left_ctrl,
            o_set_motors_pwm_right      => s_set_motors_pwm_right_ctrl,
            o_set_motors_enable_en      => s_set_motors_enable_en_ctrl,
            o_set_motors_enable_val     => s_set_motors_enable_val_ctrl,
            o_set_motors_brake_trigger  => s_set_motors_brake_trigger_ctrl,
            o_set_fan_speed_en          => s_set_fan_speed_en_ctrl,
            o_set_fan_speed_val         => s_set_fan_speed_val_ctrl,
            o_set_RGB_color_en          => s_set_rgb_color_en_ctrl,
            o_set_RGB_color_r           => s_set_rgb_color_r_ctrl,
            o_set_RGB_color_g           => s_set_rgb_color_g_ctrl,
            o_set_RGB_color_b           => s_set_rgb_color_b_ctrl,
            o_blink_RGB_color_trigger   => s_blink_rgb_color_trigger_ctrl,
            o_blink_RGB_color_r         => s_blink_rgb_color_r_ctrl,
            o_blink_RGB_color_g         => s_blink_rgb_color_g_ctrl,
            o_blink_RGB_color_b         => s_blink_rgb_color_b_ctrl,
            o_blink_RGB_color_ms        => s_blink_rgb_color_ms_ctrl,
            o_menu_reset_trigger        => s_menu_reset_trigger_ctrl,
            o_menu_run_reset_trigger    => s_menu_run_reset_trigger_ctrl,
            o_reset_motors_saturated_trigger => s_reset_motors_saturated_trigger_ctrl,
            o_reset_encoder_avg_trigger => s_reset_encoder_avg_trigger_ctrl,
            o_gpio_13_toggle            => s_gpio_13_toggle_ctrl,
            o_gpio_15_set               => s_gpio_15_set_ctrl,
            o_is_race_started           => s_is_race_started,
            o_is_front_sensors_correction_enabled => s_is_front_sensors_correction_enabled,
            o_ideal_linear_speed        => s_ideal_linear_speed,
            o_ideal_angular_speed       => s_ideal_angular_speed,
            o_check_start_run_result    => s_check_start_run_result
        );

    -- Calibrations Module
    U_CALIBRATIONS : entity work.calibrations
        port map (
            i_clk                       => i_clk,
            i_rst_n                     => i_rst_n,
            i_get_clock_ticks           => s_global_ms_ticks,
            i_get_menu_mode_btn         => s_btn_menu_mode, -- From buttons module
            i_get_debug_btn             => s_btn_debug,     -- From buttons module
            i_calibrate_config_type     => s_menu_cal_config_type_out, -- From menu module
            i_calibrate_manual_trigger  => '0', -- Temporarily unused (can be pulsed by a debug option if needed)
            i_set_debug_btn_ack         => s_set_debug_btn_ack_cal, -- Acknowledge from debug_btn logic
            i_get_sensor_raw_filter_sl  => s_sens_raw_filter_sl,
            i_get_sensor_raw_filter_fl  => s_sens_raw_filter_fl,
            i_get_sensor_raw_filter_fr  => s_sens_raw_filter_fr,
            i_get_sensor_raw_filter_sr  => s_sens_raw_filter_sr,
            o_clear_info_leds_trigger   => s_cal_clear_info_leds_trigger,
            o_set_RGB_color_en          => s_cal_set_rgb_color_en,
            o_set_RGB_color_r           => s_cal_set_rgb_color_r,
            o_set_RGB_color_g           => s_cal_set_rgb_color_g,
            o_set_RGB_color_b           => s_cal_set_rgb_color_b,
            o_lsm6dsr_gyro_z_calibration_trigger => s_cal_lsm6dsr_gyro_z_calibration_trigger,
            o_side_sensors_calibration_trigger => s_cal_side_sensors_calibration_trigger,
            o_front_sensors_calibration_trigger => s_cal_front_sensors_calibration_trigger,
            o_eeprom_save_trigger       => s_cal_eeprom_save_trigger,
            o_menu_config_reset_values_trigger => s_cal_menu_config_reset_values_trigger,
            o_set_debug_btn_en          => s_cal_set_debug_btn_en,
            o_set_debug_btn_val         => s_cal_set_debug_btn_val,
            o_print_trigger             => s_cal_print_trigger,
            o_print_msg_id              => s_cal_print_msg_id,
            o_print_data                => s_cal_print_data,
            o_calibration_enabled       => s_calibration_enabled
        );

    -- Debug Module
    U_DEBUG : entity work.debug
        port map (
            i_clk                       => i_clk,
            i_rst_n                     => i_rst_n,
            i_get_clock_ticks           => s_global_ms_ticks,
            i_menu_mode_btn             => s_btn_menu_mode, -- From buttons module
            i_debug_config_type         => s_menu_debug_config_type_out, -- From menu module
            i_debug_main_trigger        => s_debug_main_trigger,
            o_macroarray_print_trigger  => s_debug_macroarray_print_trigger,
            o_floodfill_maze_print_trigger => s_debug_floodfill_maze_print_trigger,
            o_menu_config_reset_values_trigger => s_debug_menu_config_reset_values_trigger,
            o_set_sensors_enabled_en    => s_debug_set_sensors_enabled_en,
            o_set_sensors_enabled_val   => s_debug_set_sensors_enabled_val,
            o_set_motors_enable_en      => s_debug_set_motors_enable_en,
            o_set_motors_enable_val     => s_debug_set_motors_enable_val,
            o_set_motors_speed_en       => s_debug_set_motors_speed_en,
            o_set_motors_speed_left     => s_debug_set_motors_speed_left,
            o_set_motors_speed_right    => s_debug_set_motors_speed_right,
            o_reset_control_all_trigger => s_debug_reset_control_all_trigger,
            o_keep_z_angle_trigger      => s_debug_keep_z_angle_trigger,
            o_set_fan_speed_en          => s_debug_set_fan_speed_en,
            o_set_fan_speed_val         => s_debug_set_fan_speed_val,
            o_set_RGB_color_en          => s_debug_set_rgb_color_en,
            o_set_RGB_color_r           => s_debug_set_rgb_color_r,
            o_set_RGB_color_g           => s_debug_set_rgb_color_g,
            o_set_RGB_color_b           => s_debug_set_rgb_color_b,
            o_print_trigger             => s_debug_print_trigger,
            o_print_msg_id              => s_debug_print_msg_id,
            o_print_data                => s_debug_print_data,
            o_debug_enabled             => s_debug_enabled,
            -- Inputs for Debug module to get sensor/aux data
            i_get_sensor_raw_sl_0       => s_sens_raw_sl_0,
            i_get_sensor_raw_sl_1       => s_sens_raw_sl_1,
            i_get_sensor_raw_filter_sl  => s_sens_raw_filter_sl,
            i_get_sensor_raw_fl_0       => s_sens_raw_fl_0,
            i_get_sensor_raw_fl_1       => s_sens_raw_fl_1,
            i_get_sensor_raw_filter_fl  => s_sens_raw_filter_fl,
            i_get_sensor_raw_fr_0       => s_sens_raw_fr_0,
            i_get_sensor_raw_fr_1       => s_sens_raw_fr_1,
            i_get_sensor_raw_filter_fr  => s_sens_raw_filter_fr,
            i_get_sensor_raw_sr_0       => s_sens_raw_sr_0,
            i_get_sensor_raw_sr_1       => s_sens_raw_sr_1,
            i_get_sensor_raw_filter_sr  => s_sens_raw_filter_sr,
            i_get_sensor_distance_sl    => s_sens_distance_sl,
            i_get_sensor_distance_fl    => s_sens_distance_fl,
            i_get_sensor_distance_fr    => s_sens_distance_fr,
            i_get_sensor_distance_sr    => s_sens_distance_sr,
            i_get_aux_battery           => signed(resize(s_aux_input_raw_out, 16)), -- Por ajustar: Ensure correct mapping of AUX_BATTERY_ID
            i_get_aux_current_left      => signed(resize(s_aux_input_raw_out, 16)), -- Por ajustar
            i_get_aux_current_right     => signed(resize(s_aux_input_raw_out, 16)), -- Por ajustar
            i_get_aux_menu_btn          => signed(resize(s_aux_input_raw_out, 16))  -- Por ajustar
        );


    -- EEPROM Module
    U_EEPROM : entity work.eeprom_controller
        port map (
            i_clk                       => i_clk,
            i_reset                     => not i_rst_n, -- Active-low reset converted to active-high
            i_get_clock_ticks           => s_global_ms_ticks,
            i_save_en                   => s_eeprom_save_trigger, -- From calibrations module
            i_load_en                   => s_eeprom_load_trigger,
            i_clear_en                  => s_eeprom_clear_trigger,
            i_restore_en                => s_eeprom_restore_trigger,
            i_backup_en                 => s_eeprom_backup_trigger,
            i_set_data_en               => s_eeprom_set_data_en,
            i_set_data_index            => s_eeprom_set_data_index,
            i_set_data_value            => s_eeprom_set_data_value,
            i_set_data_write_en         => s_eeprom_set_data_write_en,
            i_read_data_index           => s_eeprom_read_data_index,
            o_read_data_value           => s_eeprom_read_data_value,
            o_lsm6dsr_load_eeprom_en    => s_eeprom_lsm6dsr_load_eeprom_en,
            o_sensors_load_eeprom_en    => s_eeprom_sensors_load_eeprom_en,
            o_floodfill_load_maze_en    => s_eeprom_floodfill_load_maze_en,
            o_menu_run_load_values_en   => s_eeprom_menu_run_load_values_en,
            o_rc5_load_eeprom_en        => s_eeprom_rc5_load_eeprom_en,
            o_set_status_led_en         => s_eeprom_set_status_led_en,
            o_status_led_val            => s_eeprom_status_led_val,
            o_toggle_status_led_en      => s_eeprom_toggle_status_led_en
        );

    -- Task Scheduler Module (for sys_tick_handler functions and control_loop)
    U_TASK_SCHEDULER : entity work.task_scheduler
        generic map (
            G_NUM_TIMERS => C_SYS_TICK_TASKS_COUNT + 1 -- +1 for control_loop
        )
        port map (
            i_clk         => i_clk,
            i_rst_n       => i_rst_n,
            i_micros_tick => s_global_us_ticks, -- Use microsecond tick for scheduler
            i_set_interval => s_scheduler_set_interval,
            i_interval    => s_scheduler_interval,
            i_timer_idx   => s_scheduler_timer_idx,
            i_reset_timer => s_scheduler_reset_timer,
            i_enable_timer => s_scheduler_enable_timer,
            i_disable_timer => s_scheduler_disable_timer,
            o_task_trigger => s_scheduler_task_trigger
        );

    -- =========================================================================
    -- PLACEHOLDER MODULE INSTANTIATIONS
    -- (These entities need to be created with the specified ports)
    -- =========================================================================

    -- AUX Input Controller (Handles ADC multiplexing for get_aux_raw)
    -- This module would take i_adc_battery_raw, i_adc_current_left_raw, etc.,
    -- and based on s_aux_raw_channel_sel, output s_aux_input_raw_out.
    -- For simplicity here, we'll directly connect raw inputs, but a proper AUX controller
    -- with multiplexing would be needed if multiple AUX IDs share one ADC.
    -- Here, we'll assume a direct connection for simplicity, as only AUX_BATTERY_ID
    -- is used in the main printf. For other AUX_IDs, a proper AUX controller is mandatory.
    -- Por hacer: Implement aux_input_controller.vhd to handle multiple aux_raw inputs.
    process (s_aux_raw_channel_sel, i_adc_battery_raw, i_adc_current_left_raw, i_adc_current_right_raw, i_adc_menu_btn_raw)
    begin
        case s_aux_raw_channel_sel is
            when AUX_BATTERY_ID    => s_aux_input_raw_out <= i_adc_battery_raw;
            when AUX_CURRENT_LEFT_ID => s_aux_input_raw_out <= i_adc_current_left_raw;
            when AUX_CURRENT_RIGHT_ID => s_aux_input_raw_out <= i_adc_current_right_raw;
            when AUX_MENU_BTN_ID   => s_aux_input_raw_out <= i_adc_menu_btn_raw;
            when others            => s_aux_input_raw_out <= (others => '0'); -- Default or error
        end case;
    end process;


    -- Encoder Controller
    U_ENCODER_CONTROLLER : entity work.encoder_controller_placeholder
        port map (
            i_clk                 => i_clk,
            i_rst_n               => i_rst_n,
            i_update_trigger      => s_enc_update_trigger, -- From task scheduler
            i_encoder_left_count  => i_encoder_left_count,
            i_encoder_right_count => i_encoder_right_count,
            i_reset_avg_trigger   => s_reset_encoder_avg_trigger_ctrl, -- From control module
            o_left_speed          => s_ctrl_get_encoder_left_speed,
            o_right_speed         => s_ctrl_get_encoder_right_speed,
            o_left_millimeters    => s_enc_left_millimeters,
            o_right_millimeters   => s_enc_right_millimeters,
            o_avg_millimeters     => s_enc_avg_millimeters
        );

    -- Sensor Controller
    U_SENSOR_CONTROLLER : entity work.sensor_controller_placeholder
        port map (
            i_clk                         => i_clk,
            i_rst_n                       => i_rst_n,
            i_update_magics_trigger       => s_sens_update_magics_trigger, -- From task scheduler
            i_set_enabled_en              => s_sens_set_enabled_en, -- From main FSM / debug
            i_set_enabled_val             => s_sens_set_enabled_val, -- From main FSM / debug
            i_side_calibration_trigger    => s_cal_side_sensors_calibration_trigger, -- From calibrations
            i_front_calibration_trigger   => s_cal_front_sensors_calibration_trigger, -- From calibrations
            i_sensor_front_left_raw_0     => i_sensor_front_left_raw_0,
            i_sensor_front_left_raw_1     => i_sensor_front_left_raw_1,
            i_sensor_front_right_raw_0    => i_sensor_front_right_raw_0,
            i_sensor_front_right_raw_1    => i_sensor_front_right_raw_1,
            i_sensor_side_left_raw_0      => i_sensor_side_left_raw_0,
            i_sensor_side_left_raw_1      => i_sensor_side_left_raw_1,
            i_sensor_side_right_raw_0     => i_sensor_side_right_raw_0,
            i_sensor_side_right_raw_1     => i_sensor_side_right_raw_1,
            o_sensors_enabled             => s_sens_enabled,
            o_sensor_raw_sl_0             => s_sens_raw_sl_0,
            o_sensor_raw_sl_1             => s_sens_raw_sl_1,
            o_sensor_raw_filter_sl        => s_sens_raw_filter_sl,
            o_sensor_raw_fl_0             => s_sens_raw_fl_0,
            o_sensor_raw_fl_1             => s_sens_raw_fl_1,
            o_sensor_raw_filter_fl        => s_sens_raw_filter_fl,
            o_sensor_raw_fr_0             => s_sens_raw_fr_0,
            o_sensor_raw_fr_1             => s_sens_raw_fr_1,
            o_sensor_raw_filter_fr        => s_sens_raw_filter_fr,
            o_sensor_raw_sr_0             => s_sens_raw_sr_0,
            o_sensor_raw_sr_1             => s_sens_raw_sr_1,
            o_sensor_raw_filter_sr        => s_sens_raw_filter_sr,
            o_sensor_distance_sl          => s_sens_distance_sl,
            o_sensor_distance_fl          => s_sens_distance_fl,
            o_sensor_distance_fr          => s_sens_distance_fr,
            o_sensor_distance_sr          => s_sens_distance_sr,
            o_side_sensors_error          => s_sens_side_sensors_error,
            o_front_sensors_angle_error   => s_sens_front_sensors_angle_error,
            o_front_sensors_diagonal_error => s_sens_front_sensors_diagonal_error,
            o_wall_lost_toggle_state      => s_sens_wall_lost_toggle_state,
            o_cell_change_toggle_state    => s_sens_cell_change_toggle_state
        );

    -- LED Controller
    U_LED_CONTROLLER : entity work.led_controller_placeholder
        port map (
            i_clk                 => i_clk,
            i_rst_n               => i_rst_n,
            i_get_clock_ticks     => s_global_ms_ticks,
            i_check_while_trigger => s_leds_check_while_trigger, -- From task scheduler
            i_set_battery_level   => o_leds_battery_level_out, -- From battery module
            i_set_battery_level_en=> o_leds_battery_level_en_out, -- From battery module
            i_all_leds_clear_trigger => o_info_leds_clear_out, -- From battery/calibrations
            i_status_led_en       => s_eeprom_set_status_led_en, -- From eeprom module
            i_status_led_val      => s_eeprom_status_led_val,    -- From eeprom module
            i_toggle_status_led_en => s_eeprom_toggle_status_led_en, -- From eeprom module
            i_warning_status_led_trigger => s_warning_status_led_trigger,
            i_warning_status_led_val => s_warning_status_led_val,
            i_set_leds_wave_trigger => s_set_leds_wave_trigger,
            i_set_leds_wave_val   => s_set_leds_wave_val,
            i_set_rgb_color_en_in => s_set_rgb_color_en_leds,
            i_set_rgb_color_r_in  => s_set_rgb_color_r_leds,
            i_set_rgb_color_g_in  => s_set_rgb_color_g_leds,
            i_set_rgb_color_b_in  => s_set_rgb_color_b_leds,
            i_blink_rgb_color_trigger_in => s_blink_rgb_color_trigger_leds,
            i_blink_rgb_color_r_in => s_blink_rgb_color_r_leds,
            i_blink_rgb_color_g_in => s_blink_rgb_color_g_leds,
            i_blink_rgb_color_b_in => s_blink_rgb_color_b_leds,
            i_blink_rgb_color_ms_in => s_blink_rgb_color_ms_leds,
            i_set_rgb_rainbow_trigger => s_set_rgb_rainbow_trigger,
            o_led_status_out      => o_led_status -- Physical LED output
        );

    -- LSM6DSR Controller
    U_LSM6DSR_CONTROLLER : entity work.lsm6dsr_controller_placeholder
        port map (
            i_clk                 => i_clk,
            i_rst_n               => i_rst_n,
            i_update_trigger      => s_lsm6dsr_update_trigger, -- From task scheduler
            i_gyro_z_raw_in       => i_lsm6dsr_gyro_z_raw,
            i_calibration_trigger => s_cal_lsm6dsr_gyro_z_calibration_trigger, -- From calibrations module
            i_load_eeprom_trigger => s_eeprom_lsm6dsr_load_eeprom_en, -- From eeprom module
            o_who_am_i            => s_lsm6dsr_who_am_i_out,
            o_gyro_z_raw          => s_lsm6dsr_gyro_z_raw_out,
            o_gyro_z_radps        => s_ctrl_lsm6dsr_get_gyro_z_radps,
            o_gyro_z_dps          => s_lsm6dsr_gyro_z_dps_out,
            o_gyro_z_degrees      => s_lsm6dsr_gyro_z_degrees_out
        );

    -- Macroarray Controller
    U_MACROARRAY_CONTROLLER : entity work.macroarray_controller_placeholder
        port map (
            i_clk               => i_clk,
            i_rst_n             => i_rst_n,
            i_print_trigger     => s_debug_macroarray_print_trigger, -- From debug module
            i_store_trigger     => s_macroarray_store_trigger,
            i_store_group       => s_macroarray_store_group,
            i_store_labels_mask => s_macroarray_store_labels_mask,
            i_store_labels      => s_macroarray_store_labels,
            i_store_length      => s_macroarray_store_length,
            i_store_data        => s_macroarray_store_data
        );

    -- Menu Controller
    U_MENU_CONTROLLER : entity work.menu_controller_placeholder
        port map (
            i_clk                       => i_clk,
            i_rst_n                     => i_rst_n,
            i_handler_trigger           => s_menu_handler_trigger, -- Triggered by main FSM
            i_reset_trigger             => s_menu_reset_trigger_ctrl, -- From control module
            i_run_reset_trigger         => s_menu_run_reset_trigger_ctrl, -- From control module
            i_config_reset_values_trigger => s_cal_menu_config_reset_values_trigger or s_debug_menu_config_reset_values_trigger, -- From calibrations/debug
            i_get_menu_up_btn           => s_btn_menu_up,   -- From buttons module
            i_get_menu_down_btn         => s_btn_menu_down, -- From buttons module
            i_get_menu_mode_btn         => s_btn_menu_mode, -- From buttons module
            i_is_race_started           => s_is_race_started, -- From control module
            i_load_values_en            => s_eeprom_menu_run_load_values_en, -- From eeprom module
            o_run_can_start             => s_menu_run_can_start_out,
            o_run_get_explore_algorithm => s_menu_run_get_explore_algorithm_out,
            o_cal_config_type           => s_menu_cal_config_type_out,
            o_debug_config_type         => s_menu_debug_config_type_out,
            o_kinematics_data           => s_ctrl_kinematics -- To control module
        );

    -- Motor Controller
    U_MOTOR_CONTROLLER : entity work.motors_controller_placeholder
        port map (
            i_clk                       => i_clk,
            i_rst_n                     => i_rst_n,
            i_set_enable_en             => s_set_motors_enable_en_ctrl or s_debug_set_motors_enable_en, -- From control/debug
            i_set_enable_val            => s_set_motors_enable_val_ctrl or s_debug_set_motors_enable_val, -- From control/debug
            i_set_speed_en              => s_debug_set_motors_speed_en, -- From debug
            i_set_speed_left            => s_debug_set_motors_speed_left, -- From debug
            i_set_speed_right           => s_debug_set_motors_speed_right, -- From debug
            i_set_pwm_en                => s_set_motors_pwm_en_ctrl, -- From control
            i_set_pwm_left              => s_set_motors_pwm_left_ctrl, -- From control
            i_set_pwm_right             => s_set_motors_pwm_right_ctrl, -- From control
            i_set_brake_trigger         => s_set_motors_brake_trigger_ctrl, -- From control
            i_reset_saturated_trigger   => s_reset_motors_saturated_trigger_ctrl, -- From control
            o_is_saturated              => s_motors_is_saturated,
            o_saturated_ms              => s_motors_saturated_ms,
            o_motor_left_pwm_out        => o_motor_left_pwm, -- Physical PWM output
            o_motor_right_pwm_out       => o_motor_right_pwm, -- Physical PWM output
            o_motor_enable_out          => o_motor_enable_out, -- Physical enable output
            o_motor_brake_out           => o_motor_brake_out -- Physical brake output
        );

    -- Fan Controller (part of Motors or separate PWM module)
    -- This could be part of motor_controller, or a simple PWM generator.
    -- For now, connect directly to output, assuming PWM generation outside this module.
    -- Por hacer: Implement fan_controller_placeholder.vhd
    process(i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            o_fan_speed_pwm <= (others => '0');
        elsif rising_edge(i_clk) then
            if s_set_fan_speed_en_ctrl = '1' then
                o_fan_speed_pwm <= s_set_fan_speed_val_ctrl;
            elsif s_debug_set_fan_speed_en = '1' then
                o_fan_speed_pwm <= s_debug_set_fan_speed_val;
            end if;
        end if;
    end process;


    -- Floodfill Controller
    U_FLOODFILL_CONTROLLER : entity work.floodfill_controller_placeholder
        port map (
            i_clk                 => i_clk,
            i_rst_n               => i_rst_n,
            i_start_run_trigger   => s_floodfill_start_run_trigger,
            i_start_explore_trigger => s_floodfill_start_explore_trigger,
            i_loop_trigger        => s_floodfill_loop_trigger,
            i_load_maze_trigger   => s_eeprom_floodfill_load_maze_en, -- From eeprom module
            i_maze_print_trigger  => s_debug_floodfill_maze_print_trigger
        );

    -- Handwall Controller
    U_HANDWALL_CONTROLLER : entity work.handwall_controller_placeholder
        port map (
            i_clk                 => i_clk,
            i_rst_n               => i_rst_n,
            i_use_left_hand_trigger => s_handwall_use_left_hand_trigger,
            i_use_right_hand_trigger => s_handwall_use_right_hand_trigger,
            i_start_trigger       => s_handwall_start_trigger,
            i_loop_trigger        => s_handwall_loop_trigger
        );

    -- Timetrial Controller
    U_TIMETRIAL_CONTROLLER : entity work.timetrial_controller_placeholder
        port map (
            i_clk                 => i_clk,
            i_rst_n               => i_rst_n,
            i_start_trigger       => s_timetrial_start_trigger,
            i_loop_trigger        => s_timetrial_loop_trigger
        );

    -- RC5 Controller
    U_RC5_CONTROLLER : entity work.rc5_controller_placeholder
        port map (
            i_clk                 => i_clk,
            i_rst_n               => i_rst_n,
            i_load_eeprom_trigger => s_eeprom_rc5_load_eeprom_en
        );

    -- Setup Controller
    U_SETUP_CONTROLLER : entity work.setup_controller_placeholder
        port map (
            i_clk           => i_clk,
            i_rst_n         => i_rst_n,
            i_setup_trigger => s_setup_trigger, -- Triggered by main FSM
            o_setup_done    => s_setup_done
        );

    -- USART TX Controller (for printf)
    U_USART_TX_CONTROLLER : entity work.usart_tx_controller_placeholder
        port map (
            i_clk                 => i_clk,
            i_rst_n               => i_rst_n,
            i_print_trigger_cal   => s_cal_print_trigger, -- From calibrations module
            i_print_msg_id_cal    => s_cal_print_msg_id,
            i_print_data_cal      => s_cal_print_data,
            i_print_trigger_debug => s_debug_print_trigger, -- From debug module
            i_print_msg_id_debug  => s_debug_print_msg_id,
            i_print_data_debug    => s_debug_print_data,
            i_print_trigger_main  => s_usart_print_trigger, -- From main FSM
            i_print_msg_id_main   => s_usart_print_msg_id,
            i_print_data_main     => s_usart_print_data,
            o_usart_tx_data       => o_usart_tx_data, -- Physical UART TX output
            o_usart_tx_valid      => o_usart_tx_valid, -- Physical UART TX valid
            o_ready_to_send       => s_usart_tx_ready_to_send -- Indicates if UART TX is ready
        );

    -- GPIO Controller
    U_GPIO_CONTROLLER : entity work.gpio_controller_placeholder
        port map (
            i_clk       => i_clk,
            i_rst_n     => i_rst_n,
            i_gpio_set_en => s_gpio_set_en,
            i_gpio_clear_en => s_gpio_clear_en,
            i_gpio_id   => s_gpio_id,
            o_gpio_b13  => o_gpio_b13_out, -- Physical GPIO output
            o_gpio_b15  => o_gpio_b15_out  -- Physical GPIO output
        );

    -- =========================================================================
    -- Connections from internal modules to common outputs
    -- =========================================================================

    -- Set RGB Color
    process (i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            o_rgb_color_en_out <= '0';
            o_rgb_r_out <= (others => '0');
            o_rgb_g_out <= (others => '0');
            o_rgb_b_out <= (others => '0');
        elsif rising_edge(i_clk) then
            o_rgb_color_en_out <= '0'; -- Clear enable by default

            -- Priority for RGB colors: Debug > Calibrations > Control
            if s_debug_set_rgb_color_en = '1' then
                o_rgb_color_en_out <= '1';
                o_rgb_r_out <= s_debug_set_rgb_color_r;
                o_rgb_g_out <= s_debug_set_rgb_color_g;
                o_rgb_b_out <= s_debug_set_rgb_color_b;
            elsif s_cal_set_rgb_color_en = '1' then
                o_rgb_color_en_out <= '1';
                o_rgb_r_out <= s_cal_set_rgb_color_r;
                o_rgb_g_out <= s_cal_set_rgb_color_g;
                o_rgb_b_out <= s_cal_set_rgb_color_b;
            elsif s_set_rgb_color_en_ctrl = '1' then
                o_rgb_color_en_out <= '1';
                o_rgb_r_out <= s_set_rgb_color_r_ctrl;
                o_rgb_g_out <= s_set_rgb_color_g_ctrl;
                o_rgb_b_out <= s_set_rgb_color_b_ctrl;
            else
                -- If no module is setting RGB, set to off
                o_rgb_color_en_out <= '1'; -- Keep enabled to output 0,0,0
                o_rgb_r_out <= (others => '0');
                o_rgb_g_out <= (others => '0');
                o_rgb_b_out <= (others => '0');
            end if;
        end if;
    end process;


    -- Blink RGB Color
    process (i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            s_blink_rgb_color_trigger_leds <= '0';
            s_blink_rgb_color_r_leds <= (others => '0');
            s_blink_rgb_color_g_leds <= (others => '0');
            s_blink_rgb_color_b_leds <= (others => '0');
            s_blink_rgb_color_ms_leds <= (others => '0');
        elsif rising_edge(i_clk) then
            s_blink_rgb_color_trigger_leds <= '0'; -- Clear trigger by default

            if s_blink_rgb_color_trigger_ctrl = '1' then
                s_blink_rgb_color_trigger_leds <= '1';
                s_blink_rgb_color_r_leds <= s_blink_rgb_color_r_ctrl;
                s_blink_rgb_color_g_leds <= s_blink_rgb_color_g_ctrl;
                s_blink_rgb_color_b_leds <= s_blink_rgb_color_b_ctrl;
                s_blink_rgb_color_ms_leds <= s_blink_rgb_color_ms_ctrl;
            end if;
        end if;
    end process;

    -- Clear Info LEDs trigger (OR together from sources)
    o_info_leds_clear_out <= s_cal_clear_info_leds_trigger; -- Add other sources if they appear

    -- Set Debug Button (OR together from sources)
    s_set_debug_btn_en <= s_cal_set_debug_btn_en;
    s_set_debug_btn_val <= s_cal_set_debug_btn_val;
    s_set_debug_btn_ack_cal <= s_set_debug_btn_en and s_set_debug_btn_val = '0'; -- Acknowledge when buttons module processes 'false'

    -- Reset Control All trigger (OR together from sources)
    s_reset_control_all_trigger_ctrl <= '0'; -- Default to low
    if s_set_race_started_en_ctrl = '1' then -- When set_race_started is called
        s_reset_control_all_trigger_ctrl <= '1';
    elsif s_debug_reset_control_all_trigger = '1' then -- From debug module
        s_reset_control_all_trigger_ctrl <= '1';
    end if;


    -- =========================================================================
    -- Main FSM (Simulates main() and sys_tick_handler)
    -- =========================================================================
    process (i_clk, i_rst_n)
        -- Local variables for main loop state
        variable v_explore_alg : explore_algorithm_type;
        variable v_sensor_id   : signed(7 downto 0);
    begin
        if i_rst_n = '0' then
            s_main_state <= S_INIT;
            s_setup_trigger <= '0';
            s_eeprom_load_trigger <= '0';
            i_show_battery_level_trigger <= '0';
            s_usart_print_trigger <= '0';
            s_usart_print_msg_id <= 0;
            s_usart_print_data <= (others => (others => '0'));
            s_menu_handler_trigger <= '0';
            s_sens_set_enabled_en <= '0';
            s_sens_set_enabled_val <= '0';
            s_debug_main_trigger <= '0'; -- Ensure debug isn't active on startup
            s_set_race_started_en_ctrl <= '0';
            s_set_race_race_started_val_ctrl <= '0';
            s_floodfill_start_run_trigger <= '0';
            s_floodfill_start_explore_trigger <= '0';
            s_handwall_use_left_hand_trigger <= '0';
            s_handwall_use_right_hand_trigger <= '0';
            s_handwall_start_trigger <= '0';
            s_timetrial_start_trigger <= '0';
            s_handwall_loop_trigger <= '0';
            s_floodfill_loop_trigger <= '0';
            s_timetrial_loop_trigger <= '0';

            -- Reset task scheduler configuration
            s_scheduler_set_interval <= (others => '0');
            s_scheduler_reset_timer <= (others => '0');
            s_scheduler_enable_timer <= (others => '0');
            s_scheduler_disable_timer <= (others => '0');

        elsif rising_edge(i_clk) then
            -- Clear one-shot triggers at the beginning of each cycle
            s_setup_trigger <= '0';
            s_eeprom_load_trigger <= '0';
            i_show_battery_level_trigger <= '0';
            s_usart_print_trigger <= '0';
            s_menu_handler_trigger <= '0';
            s_sens_set_enabled_en <= '0';
            s_set_race_started_en_ctrl <= '0';
            s_floodfill_start_run_trigger <= '0';
            s_floodfill_start_explore_trigger <= '0';
            s_handwall_use_left_hand_trigger <= '0';
            s_handwall_use_right_hand_trigger <= '0';
            s_handwall_start_trigger <= '0';
            s_timetrial_start_trigger <= '0';
            s_handwall_loop_trigger <= '0';
            s_floodfill_loop_trigger <= '0';
            s_timetrial_loop_trigger <= '0';
            s_control_loop_trigger <= '0';
            s_keep_z_angle_trigger_ctrl <= '0';
            s_set_target_linear_speed_en_ctrl <= '0';
            s_set_ideal_angular_speed_en_ctrl <= '0';
            s_set_target_fan_speed_en_ctrl <= '0';
            s_set_control_debug_en_ctrl <= '0';
            s_set_side_sensors_close_correction_en_ctrl <= '0';
            s_set_side_sensors_far_correction_en_ctrl <= '0';
            s_set_front_sensors_correction_en_ctrl <= '0';
            s_set_front_sensors_diagonal_correction_en_ctrl <= '0';
            s_disable_sensors_correction_trigger_ctrl <= '0';
            s_reset_control_errors_trigger_ctrl <= '0';
            s_reset_control_speed_trigger_ctrl <= '0';


            -- Update task scheduler configuration (one-time setup)
            -- These pulses should only be active during initial setup.
            s_scheduler_set_interval <= (others => '0');
            s_scheduler_reset_timer <= (others => '0');
            s_scheduler_enable_timer <= (others => '0');
            s_scheduler_disable_timer <= (others => '0');


            -- --- SysTick Handler periodic calls (managed by Task Scheduler) ---
            -- This simulates the sys_tick_handler() interrupt.
            -- Each bit in s_scheduler_task_trigger corresponds to a function call.
            if s_sys_tick_trigger = '1' then -- Activated at 1ms intervals
                s_enc_update_trigger <= s_scheduler_task_trigger(TASK_ID_ENCODERS_UPDATE);
                s_sens_update_magics_trigger <= s_scheduler_task_trigger(TASK_ID_SENSORS_UPDATE);
                s_batt_update_trigger <= s_scheduler_task_trigger(TASK_ID_BATTERY_UPDATE);
                s_leds_check_while_trigger <= s_scheduler_task_trigger(TASK_ID_LEDS_CHECK);
                -- check_buttons() is implicitly updated by buttons module on every clk cycle
                s_lsm6dsr_update_trigger <= s_scheduler_task_trigger(TASK_ID_LSM6DSR_UPDATE);
                -- control_loop is also a periodic task
                s_control_loop_trigger <= s_scheduler_task_trigger(TASK_ID_CONTROL_LOOP);
            end if;

            -- --- Main FSM Logic ---
            case s_main_state is
                when S_INIT =>
                    s_setup_trigger <= '1'; -- Call setup()
                    s_main_state <= S_INIT_EEPROM_LOAD; -- Next state immediately (assuming setup is fast or handled by its own FSM)
                    s_usart_print_trigger <= '1';
                    s_usart_print_msg_id <= MSG_ID_HINVERNADERO_INIT_MAIN;
                    s_usart_print_data <= (others => (others => '0'));

                    -- Configure Task Scheduler for sys_tick_handler tasks (one-time setup)
                    s_scheduler_set_interval(TASK_ID_ENCODERS_UPDATE) <= '1'; s_scheduler_interval <= to_unsigned(1000, 32); s_scheduler_timer_idx <= TASK_ID_ENCODERS_UPDATE; -- 1ms
                    s_scheduler_set_interval(TASK_ID_SENSORS_UPDATE) <= '1'; s_scheduler_interval <= to_unsigned(1000, 32); s_scheduler_timer_idx <= TASK_ID_SENSORS_UPDATE;   -- 1ms
                    s_scheduler_set_interval(TASK_ID_BATTERY_UPDATE) <= '1'; s_scheduler_interval <= to_unsigned(1000, 32); s_scheduler_timer_idx <= TASK_ID_BATTERY_UPDATE;   -- 1ms
                    s_scheduler_set_interval(TASK_ID_LEDS_CHECK) <= '1'; s_scheduler_interval <= to_unsigned(1000, 32); s_scheduler_timer_idx <= TASK_ID_LEDS_CHECK;       -- 1ms
                    s_scheduler_set_interval(TASK_ID_BUTTONS_CHECK) <= '1'; s_scheduler_interval <= to_unsigned(1000, 32); s_scheduler_timer_idx <= TASK_ID_BUTTONS_CHECK;   -- 1ms
                    s_scheduler_set_interval(TASK_ID_LSM6DSR_UPDATE) <= '1'; s_scheduler_interval <= to_unsigned(1000, 32); s_scheduler_timer_idx <= TASK_ID_LSM6DSR_UPDATE; -- 1ms
                    s_scheduler_set_interval(TASK_ID_CONTROL_LOOP) <= '1'; s_scheduler_interval <= to_unsigned(1000, 32); s_scheduler_timer_idx <= TASK_ID_CONTROL_LOOP;   -- 1ms

                    -- Reset all tasks to start immediately
                    s_scheduler_reset_timer <= (others => '1');
                    s_scheduler_enable_timer <= (others => '1');


                when S_INIT_EEPROM_LOAD =>
                    -- Por hacer: Wait for setup_done if setup module has a longer running process
                    -- Currently, we proceed immediately after triggering setup.
                    s_eeprom_load_trigger <= '1'; -- Call eeprom_load()
                    s_main_state <= S_INIT_SHOW_BATTERY_LEVEL;

                when S_INIT_SHOW_BATTERY_LEVEL =>
                    -- Por hacer: Wait for eeprom_load to finish if it's a multi-cycle process.
                    i_show_battery_level_trigger <= '1'; -- Call show_battery_level()
                    s_main_state <= S_INIT_PRINT_AUX_RAW;

                when S_INIT_PRINT_AUX_RAW =>
                    -- Por hacer: Wait for show_battery_level to finish.
                    -- printf("BA: %4d CI: %4d CD: %4d BO: %4d\n", get_aux_raw(AUX_BATTERY_ID), get_aux_raw(AUX_CURRENT_LEFT_ID), get_aux_raw(AUX_CURRENT_RIGHT_ID), get_aux_raw(AUX_MENU_BTN_ID));
                    if s_usart_tx_ready_to_send = '1' then
                        s_usart_print_trigger <= '1';
                        s_usart_print_msg_id <= MSG_ID_AUX_RAW_FMT;
                        -- These need to be from their respective AUX_IDs
                        s_usart_print_data(0) <= signed(i_adc_battery_raw); -- get_aux_raw(AUX_BATTERY_ID)
                        s_usart_print_data(1) <= signed(i_adc_current_left_raw); -- get_aux_raw(AUX_CURRENT_LEFT_ID)
                        s_usart_print_data(2) <= signed(i_adc_current_right_raw); -- get_aux_raw(AUX_CURRENT_RIGHT_ID)
                        s_usart_print_data(3) <= signed(i_adc_menu_btn_raw); -- get_aux_raw(AUX_MENU_BTN_ID)
                        s_main_state <= S_MAIN_LOOP_IDLE;
                    end if;

                when S_MAIN_LOOP_IDLE =>
                    -- This is the entry point for the main while(1) loop.
                    -- The robot's main operational loop.
                    s_main_state <= S_MAIN_LOOP_NOT_STARTED; -- Decide next based on is_race_started()

                when S_MAIN_LOOP_NOT_STARTED => -- !is_race_started() branch
                    if s_is_race_started = '1' then
                        s_main_state <= S_MAIN_LOOP_RACE_STARTED;
                    else
                        s_menu_handler_trigger <= '1'; -- menu_handler()

                        if s_menu_run_can_start_out = '1' then
                            if s_sens_enabled = '0' then -- !get_sensors_enabled()
                                s_sens_set_enabled_en <= '1';
                                s_sens_set_enabled_val <= '1'; -- set_sensors_enabled(true)
                                s_main_delay_start_ms <= s_global_ms_ticks;
                                s_main_delay_duration_ms <= to_unsigned(200, 32); -- delay(200)
                                s_main_state <= S_DELAY_200MS_SENSORS_ENABLE;
                            else
                                s_sens_set_enabled_en <= '1';
                                s_sens_set_enabled_val <= '1'; -- set_sensors_enabled(true)
                                -- Already enabled, no delay
                                -- check_start_run(); if needed, transition to next state
                                s_main_state <= S_MAIN_LOOP_NOT_STARTED; -- Stay in this loop for continuous check
                                -- Directly proceed to check_start_run logic if menu_run_can_start()
                                v_sensor_id := s_check_start_run_result; -- Result from control module
                                if s_is_race_started = '1' then
                                    v_explore_alg := s_menu_run_get_explore_algorithm_out;
                                    case v_explore_alg is
                                        when EXPLORE_HANDWALL =>
                                            case v_sensor_id is
                                                when SENSOR_FRONT_LEFT_WALL_ID => s_handwall_use_left_hand_trigger <= '1'; s_handwall_start_trigger <= '1';
                                                when SENSOR_FRONT_RIGHT_WALL_ID => s_handwall_use_right_hand_trigger <= '1'; s_handwall_start_trigger <= '1';
                                                when others => null;
                                            end case;
                                        when EXPLORE_FLOODFILL =>
                                            case v_sensor_id is
                                                when SENSOR_FRONT_LEFT_WALL_ID => s_floodfill_start_run_trigger <= '1';
                                                when SENSOR_FRONT_RIGHT_WALL_ID => s_floodfill_start_explore_trigger <= '1';
                                                when others => null;
                                            end case;
                                        when EXPLORE_TIME_TRIAL =>
                                            s_timetrial_start_trigger <= '1';
                                        when others =>
                                            s_set_race_started_en_ctrl <= '1';
                                            s_set_race_started_val_ctrl <= '0'; -- false
                                    end case;
                                    -- After starting an algorithm, go to race started loop
                                    s_main_state <= S_MAIN_LOOP_RACE_STARTED;
                                else
                                    -- If race not started, continue checking
                                    s_main_state <= S_MAIN_LOOP_NOT_STARTED;
                                end if;
                            end if;
                        else
                            s_sens_set_enabled_en <= '1';
                            s_sens_set_enabled_val <= '0'; -- set_sensors_enabled(false)
                            s_main_state <= S_MAIN_LOOP_NOT_STARTED; -- Stay in this loop
                        end if;
                    end if;

                when S_DELAY_200MS_SENSORS_ENABLE =>
                    if s_main_delay_done_pulse = '1' then
                        -- After delay, continue with check_start_run logic
                        v_sensor_id := s_check_start_run_result; -- Result from control module
                        if s_is_race_started = '1' then
                            v_explore_alg := s_menu_run_get_explore_algorithm_out;
                            case v_explore_alg is
                                when EXPLORE_HANDWALL =>
                                    case v_sensor_id is
                                        when SENSOR_FRONT_LEFT_WALL_ID => s_handwall_use_left_hand_trigger <= '1'; s_handwall_start_trigger <= '1';
                                        when SENSOR_FRONT_RIGHT_WALL_ID => s_handwall_use_right_hand_trigger <= '1'; s_handwall_start_trigger <= '1';
                                        when others => null;
                                    end case;
                                when EXPLORE_FLOODFILL =>
                                    case v_sensor_id is
                                        when SENSOR_FRONT_LEFT_WALL_ID => s_floodfill_start_run_trigger <= '1';
                                        when SENSOR_FRONT_RIGHT_WALL_ID => s_floodfill_start_explore_trigger <= '1';
                                        when others => null;
                                    end case;
                                when EXPLORE_TIME_TRIAL =>
                                    s_timetrial_start_trigger <= '1';
                                when others =>
                                    s_set_race_started_en_ctrl <= '1';
                                    s_set_race_started_val_ctrl <= '0'; -- false
                            end case;
                            s_main_state <= S_MAIN_LOOP_RACE_STARTED;
                        else
                            s_main_state <= S_MAIN_LOOP_NOT_STARTED;
                        end if;
                    end if;


                when S_MAIN_LOOP_RACE_STARTED => -- is_race_started() branch
                    if s_is_race_started = '0' then
                        s_main_state <= S_MAIN_LOOP_NOT_STARTED; -- Race ended, go back to not started
                    else
                        v_explore_alg := s_menu_run_get_explore_algorithm_out;
                        case v_explore_alg is
                            when EXPLORE_HANDWALL =>
                                s_handwall_loop_trigger <= '1';
                            when EXPLORE_FLOODFILL =>
                                s_floodfill_loop_trigger <= '1';
                            when EXPLORE_TIME_TRIAL =>
                                s_timetrial_loop_trigger <= '1';
                            when others =>
                                s_set_race_started_en_ctrl <= '1';
                                s_set_race_started_val_ctrl <= '0'; -- false
                        end case;
                        s_main_state <= S_MAIN_LOOP_RACE_STARTED; -- Continue looping
                    end if;

                when S_MAIN_LOOP_DEBUG_ZONE =>
                    -- This state represents the commented-out debug code in main.c.
                    -- By default, it just transitions back to the main loop if not triggered.
                    -- To activate specific debug prints or demos, you would need
                    -- to add control logic (e.g., from a button or UART command)
                    -- to transition to specific sub-states here.
                    s_gpio_set_en <= '1'; s_gpio_id <= GPIOB_15_ID; -- gpio_set(GPIOB, GPIO15)
                    s_main_state <= S_MAIN_LOOP_DEBUG_ZONE; -- Loop in this state

                    -- Example: if an external debug trigger is asserted, activate specific debug
                    -- if i_external_debug_trigger = '1' and s_usart_tx_ready_to_send = '1' then
                    --     s_usart_print_trigger <= '1';
                    --     s_usart_print_msg_id <= MSG_ID_MPU_FMT;
                    --     s_usart_print_data(0) <= signed(s_lsm6dsr_who_am_i_out); -- MPU_WHO_AM_I
                    --     s_main_delay_start_ms <= s_global_ms_ticks;
                    --     s_main_delay_duration_ms <= to_unsigned(500, 32);
                    --     s_main_state <= S_DELAY_DEBUG_LOOP;
                    -- end if;

                when S_DELAY_DEBUG_LOOP =>
                    if s_main_delay_done_pulse = '1' then
                        s_main_state <= S_MAIN_LOOP_DEBUG_ZONE;
                    end if;

                when others =>
                    s_main_state <= S_INIT; -- Fallback to initial state
            end case;
        end if;
    end process;


    -- =========================================================================
    -- Placeholder Entity Declarations (These entities need to be defined in separate .vhd files)
    -- =========================================================================

    component encoder_controller_placeholder is
        port (
            i_clk                 : in  std_logic;
            i_rst_n               : in  std_logic;
            i_update_trigger      : in  std_logic;
            i_encoder_left_count  : in  signed(31 downto 0);
            i_encoder_right_count : in  signed(31 downto 0);
            i_reset_avg_trigger   : in  std_logic;
            o_left_speed          : out float32;
            o_right_speed         : out float32;
            o_left_millimeters    : out signed(31 downto 0);
            o_right_millimeters   : out signed(31 downto 0);
            o_avg_millimeters     : out float32
        );
    end component;

    component sensor_controller_placeholder is
        port (
            i_clk                         : in  std_logic;
            i_rst_n                       : in  std_logic;
            i_update_magics_trigger       : in  std_logic;
            i_set_enabled_en              : in  std_logic;
            i_set_enabled_val             : in  std_logic;
            i_side_calibration_trigger    : in  std_logic;
            i_front_calibration_trigger   : in  std_logic;
            i_sensor_front_left_raw_0     : in  signed(15 downto 0);
            i_sensor_front_left_raw_1     : in  signed(15 downto 0);
            i_sensor_front_right_raw_0    : in  signed(15 downto 0);
            i_sensor_front_right_raw_1    : in  signed(15 downto 0);
            i_sensor_side_left_raw_0      : in  signed(15 downto 0);
            i_sensor_side_left_raw_1      : in  signed(15 downto 0);
            i_sensor_side_right_raw_0     : in  signed(15 downto 0);
            i_sensor_side_right_raw_1     : in  signed(15 downto 0);
            o_sensors_enabled             : out std_logic;
            o_sensor_raw_sl_0             : out signed(15 downto 0);
            o_sensor_raw_sl_1             : out signed(15 downto 0);
            o_sensor_raw_filter_sl        : out signed(15 downto 0);
            o_sensor_raw_fl_0             : out signed(15 downto 0);
            o_sensor_raw_fl_1             : out signed(15 downto 0);
            o_sensor_raw_filter_fl        : out signed(15 downto 0);
            o_sensor_raw_fr_0             : out signed(15 downto 0);
            o_sensor_raw_fr_1             : out signed(15 downto 0);
            o_sensor_raw_filter_fr        : out signed(15 downto 0);
            o_sensor_raw_sr_0             : out signed(15 downto 0);
            o_sensor_raw_sr_1             : out signed(15 downto 0);
            o_sensor_raw_filter_sr        : out signed(15 downto 0);
            o_sensor_distance_sl          : out signed(15 downto 0);
            o_sensor_distance_fl          : out signed(15 downto 0);
            o_sensor_distance_fr          : out signed(15 downto 0);
            o_sensor_distance_sr          : out signed(15 downto 0);
            o_side_sensors_error          : out float32;
            o_front_sensors_angle_error   : out float32;
            o_front_sensors_diagonal_error: out float32;
            o_wall_lost_toggle_state      : out std_logic;
            o_cell_change_toggle_state    : out std_logic
        );
    end component;

    component leds_controller_placeholder is
        port (
            i_clk                 : in  std_logic;
            i_rst_n               : in  std_logic;
            i_get_clock_ticks     : in  unsigned(31 downto 0);
            i_check_while_trigger : in  std_logic;
            i_set_battery_level   : in  unsigned(7 downto 0);
            i_set_battery_level_en: in  std_logic;
            i_all_leds_clear_trigger: in std_logic;
            i_status_led_en       : in  std_logic;
            i_status_led_val      : in  std_logic;
            i_toggle_status_led_en: in  std_logic;
            i_warning_status_led_trigger : in std_logic;
            i_warning_status_led_val     : in unsigned(7 downto 0);
            i_set_leds_wave_trigger      : in std_logic;
            i_set_leds_wave_val          : in unsigned(7 downto 0);
            i_set_rgb_color_en_in : in  std_logic;
            i_set_rgb_color_r_in  : in  unsigned(7 downto 0);
            i_set_rgb_color_g_in  : in  unsigned(7 downto 0);
            i_set_rgb_color_b_in  : in  unsigned(7 downto 0);
            i_blink_rgb_color_trigger_in : in std_logic;
            i_blink_rgb_color_r_in : in unsigned(7 downto 0);
            i_blink_rgb_color_g_in : in unsigned(7 downto 0);
            i_blink_rgb_color_b_in : in unsigned(7 downto 0);
            i_blink_rgb_color_ms_in : in unsigned(31 downto 0);
            i_set_rgb_rainbow_trigger : in std_logic;
            o_led_status_out      : out std_logic
        );
    end component;

    component lsm6dsr_controller_placeholder is
        port (
            i_clk                 : in  std_logic;
            i_rst_n               : in  std_logic;
            i_update_trigger      : in  std_logic;
            i_gyro_z_raw_in       : in  signed(15 downto 0);
            i_calibration_trigger : in  std_logic;
            i_load_eeprom_trigger : in  std_logic;
            o_who_am_i            : out std_logic_vector(7 downto 0);
            o_gyro_z_raw          : out signed(15 downto 0);
            o_gyro_z_radps        : out float32;
            o_gyro_z_dps          : out float32;
            o_gyro_z_degrees      : out float32
        );
    end component;

    component macroarray_controller_placeholder is
        port (
            i_clk               : in  std_logic;
            i_rst_n             : in  std_logic;
            i_print_trigger     : in  std_logic;
            i_store_trigger     : in  std_logic;
            i_store_group       : in  natural;
            i_store_labels_mask : in  std_logic_vector(10 downto 0);
            i_store_labels      : in  integer;
            i_store_length      : in  natural;
            i_store_data        : in  t_print_data_array
        );
    end component;

    component menu_controller_placeholder is
        port (
            i_clk                       : in  std_logic;
            i_rst_n                     : in  std_logic;
            i_handler_trigger           : in  std_logic;
            i_reset_trigger             : in  std_logic;
            i_run_reset_trigger         : in  std_logic;
            i_config_reset_values_trigger : in std_logic;
            i_get_menu_up_btn           : in  std_logic;
            i_get_menu_down_btn         : in  std_logic;
            i_get_menu_mode_btn         : in  std_logic;
            i_is_race_started           : in  std_logic;
            i_load_values_en            : in  std_logic;
            o_run_can_start             : out std_logic;
            o_run_get_explore_algorithm : out explore_algorithm_type;
            o_cal_config_type           : out t_calibrate_type;
            o_debug_config_type         : out t_debug_type;
            o_kinematics_data           : out t_kinematics
        );
    end component;

    component motors_controller_placeholder is
        port (
            i_clk                 : in  std_logic;
            i_rst_n               : in  std_logic;
            i_set_enable_en       : in  std_logic;
            i_set_enable_val      : in  std_logic;
            i_set_speed_en        : in  std_logic;
            i_set_speed_left      : in  signed(15 downto 0);
            i_set_speed_right     : in  signed(15 downto 0);
            i_set_pwm_en          : in  std_logic;
            i_set_pwm_left        : in  signed(15 downto 0);
            i_set_pwm_right       : in  signed(15 downto 0);
            i_set_brake_trigger   : in  std_logic;
            i_reset_saturated_trigger : in std_logic;
            o_is_saturated        : out std_logic;
            o_saturated_ms        : out unsigned(31 downto 0);
            o_motor_left_pwm_out  : out signed(15 downto 0);
            o_motor_right_pwm_out : out signed(15 downto 0);
            o_motor_enable_out    : out std_logic;
            o_motor_brake_out     : out std_logic
        );
    end component;

    component floodfill_controller_placeholder is
        port (
            i_clk                 : in  std_logic;
            i_rst_n               : in  std_logic;
            i_start_run_trigger   : in  std_logic;
            i_start_explore_trigger : in  std_logic;
            i_loop_trigger        : in  std_logic;
            i_load_maze_trigger   : in  std_logic;
            i_maze_print_trigger  : in  std_logic
        );
    end component;

    component handwall_controller_placeholder is
        port (
            i_clk                 : in  std_logic;
            i_rst_n               : in  std_logic;
            i_use_left_hand_trigger : in std_logic;
            i_use_right_hand_trigger : in std_logic;
            i_start_trigger       : in  std_logic;
            i_loop_trigger        : in  std_logic
        );
    end component;

    component timetrial_controller_placeholder is
        port (
            i_clk                 : in  std_logic;
            i_rst_n               : in  std_logic;
            i_start_trigger       : in  std_logic;
            i_loop_trigger        : in  std_logic
        );
    end component;

    component rc5_controller_placeholder is
        port (
            i_clk                 : in  std_logic;
            i_rst_n               : in  std_logic;
            i_load_eeprom_trigger : in  std_logic
        );
    end component;

    component setup_controller_placeholder is
        port (
            i_clk           : in  std_logic;
            i_rst_n         : in  std_logic;
            i_setup_trigger : in  std_logic;
            o_setup_done    : out std_logic
        );
    end component;

    component usart_tx_controller_placeholder is
        port (
            i_clk                 : in  std_logic;
            i_rst_n               : in  std_logic;
            i_print_trigger_cal   : in  std_logic;
            i_print_msg_id_cal    : in  natural;
            i_print_data_cal      : in  t_print_data_array;
            i_print_trigger_debug : in  std_logic;
            i_print_msg_id_debug  : in  natural;
            i_print_data_debug    : in  t_print_data_array;
            i_print_trigger_main  : in  std_logic;
            i_print_msg_id_main   : in  natural;
            i_print_data_main     : in  t_print_data_array;
            o_usart_tx_data       : out std_logic_vector(7 downto 0);
            o_usart_tx_valid      : out std_logic;
            o_ready_to_send       : out std_logic
        );
    end component;

    component gpio_controller_placeholder is
        port (
            i_clk       : in  std_logic;
            i_rst_n     : in  std_logic;
            i_gpio_set_en : in  std_logic;
            i_gpio_clear_en : in  std_logic;
            i_gpio_id   : in  natural;
            o_gpio_b13  : out std_logic;
            o_gpio_b15  : out std_logic
        );
    end component;

end architecture rtl;
