

-- move.vhd

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all; -- Solo para cálculo de constantes en tiempo de diseño/síntesis

-- =============================================================================
-- PAQUETE DE TIPOS COMUNES (robot_types_pkg.vhd)
-- Se recomienda definir estos tipos en un archivo .vhd separado para reutilización
-- y luego incluirlo con 'use work.robot_types_pkg.all;'
-- =============================================================================
-- type movement is (
--     MOVE_NONE, MOVE_HOME, MOVE_START, MOVE_END, MOVE_FRONT, MOVE_LEFT,
--     MOVE_RIGHT, MOVE_LEFT_90, MOVE_RIGHT_90, MOVE_LEFT_180, MOVE_RIGHT_180,
--     MOVE_DIAGONAL, MOVE_LEFT_TO_45, MOVE_RIGHT_TO_45, MOVE_LEFT_TO_135,
--     MOVE_RIGHT_TO_135, MOVE_LEFT_45_TO_45, MOVE_RIGHT_45_TO_45,
--     MOVE_LEFT_FROM_45, MOVE_RIGHT_FROM_45, MOVE_LEFT_FROM_45_180,
--     MOVE_RIGHT_FROM_45_180, MOVE_BACK, MOVE_BACK_WALL, MOVE_BACK_STOP
-- );
--
-- type speed_strategy is (SPEED_HAKI, SPEED_NORMAL, -- ... otras velocidades);
--
-- type accel_params is record
--     accel_soft : natural;
--     accel_hard : natural;
--     break_accel: natural; -- 'break_accel' es un 'float' en C, aquí natural escalado
-- end record;
--
-- type turn_params is record
--     start       : natural; -- 'float' en C, aquí natural escalado
--     end_turn    : natural; -- 'float' en C, aquí natural escalado
--     arc         : natural; -- 'float' en C, aquí natural escalado
--     linear_speed: natural; -- 'int32_t' en C, aquí natural
--     max_angular_speed: natural; -- 'float' en C, aquí natural escalado para rad/s
--     sign        : signed(0 to 0); -- 'int8_t' en C, -1 o 1
--     -- Podrías necesitar bits para los offsets:
--     -- start_offset_mm : signed(X downto 0);
--     -- end_offset_mm   : signed(X downto 0);
-- end record;
--
-- type kinematics_data is record
--     linear_speed : natural;
--     linear_accel : accel_params;
--     fan_speed    : natural;
--     turns        : array(movement range MOVE_LEFT to MOVE_RIGHT_FROM_45_180) of turn_params; -- O un rango específico
-- end record;
--
-- type walls_status is record
--     front : std_logic;
--     left  : std_logic;
--     right : std_logic;
-- end record;
--
-- type kinematics_settings_array_t is array(speed_strategy) of kinematics_data;
-- =============================================================================

-- Asumo que los tipos anteriores están definidos en un paquete 'robot_types_pkg'
use work.robot_types_pkg.all; -- Incluye tus tipos personalizados

-- =============================================================================
-- CONSTANTES GLOBALES (Necesitan ser definidas consistentemente, ej. en un paquete)
-- Los valores 'float' de C se traducen a 'natural' o 'signed' escalados (punto fijo).
-- =============================================================================
constant C_MICROMETERS_PER_MILLIMETER : natural := 1000;
constant C_PI                         : natural := 314159; -- Ejemplo de PI * 100000 (fixed-point)
constant C_SENSING_POINT_DISTANCE     : natural := 40;   -- Valor en mm
constant C_CELL_DIMENSION             : natural := 180;  -- Valor en mm
constant C_ROBOT_BACK_LENGTH          : natural := 50;   -- Valor en mm
constant C_WALL_WIDTH                 : natural := 12;   -- Valor en mm
constant C_MIDDLE_MAZE_DISTANCE       : natural := 75;   -- Valor en mm
constant C_WALL_LOSS_TO_SENSING_POINT_DISTANCE : natural := 60; -- Valor en mm
constant C_ROBOT_FRONT_LENGTH         : natural := 50;  -- Valor en mm
constant C_CELL_DIAGONAL              : natural := 254; -- Aprox sqrt(180^2 + 180^2) = 254.55, escala de 1000 para punto fijo

-- Factor de escala global para números flotantes convertidos a enteros
-- Ej: Si un float era 1.23, con C_FIXED_POINT_SCALE = 1000, sería 1230.
constant C_FIXED_POINT_SCALE : natural := 1000;

-- Definir las configuraciones de cinemática (equivalente a kinematics_settings[] en C)
-- Esto sería una constante VHDL de tipo kinematics_settings_array_t.
-- Los valores 'float' deben ser convertidos a 'natural' o 'signed' escalados.
-- Ejemplo:
-- constant C_KINEMATICS_SETTINGS : kinematics_settings_array_t := (
--     SPEED_HAKI => (
--         linear_speed => 300,
--         linear_accel => (accel_soft => 1000, accel_hard => 2000, break_accel => 5000), -- Escalado
--         fan_speed    => 255,
--         turns        => (
--             MOVE_LEFT_90 => (start => 500, end_turn => 500, arc => 800, linear_speed => 200, max_angular_speed => 1000, sign => "1"), -- Max angular speed escalado (rad/s * 1000)
--             -- ... resto de los movimientos
--             others => (start => 0, end_turn => 0, arc => 0, linear_speed => 0, max_angular_speed => 0, sign => "0")
--         )
--     ),
--     -- ... resto de las estrategias de velocidad
--     others => (linear_speed => 0, linear_accel => (others => 0), fan_speed => 0, turns => (others => (others => '0')))
-- );

-- Definición de la dirección de los GPIOs de los emisores (ej. de setup_gpio)
-- Estas son asignaciones de constantes que reflejan la configuración de pines en C.
-- Se usarán para controlar las salidas o_gpio_emitter_out en 'sensors.vhd'
-- constant GPIOA_EMITTER_0_IDX : natural := 0; -- Para SENSOR_FRONT_LEFT_WALL_ID
-- constant GPIOA_EMITTER_1_IDX : natural := 1; -- Para SENSOR_SIDE_RIGHT_WALL_ID
-- constant GPIOA_EMITTER_2_IDX : natural := 2; -- Para SENSOR_SIDE_LEFT_WALL_ID
-- constant GPIOA_EMITTER_3_IDX : natural := 3; -- Para SENSOR_FRONT_RIGHT_WALL_ID


--------------------------------------------------------------------------------
-- Entidad: robot_movement_controller
-- Descripción: Este módulo VHDL gestiona todos los movimientos de alto nivel
--              del robot, desde la navegación en el laberinto hasta los giros
--              y las correcciones basadas en sensores.
--              Es la traducción de 'move.c' a hardware FSMs.
--
-- Entradas:
--   i_clk                    : Reloj principal del sistema
--   i_reset                  : Reset síncrono (activo alto)
--   i_start_movement_cmd     : Pulso para iniciar un movimiento (desde 'move' en C)
--   i_movement_type_cmd      : Tipo de movimiento a ejecutar (desde 'move' en C)
--   i_run_sequence_cmd       : Pulso para iniciar una secuencia de movimientos
--   i_movement_sequence      : Array de tipos de movimiento para la secuencia
--   i_current_kinematics     : Parámetros de cinemática actuales
--   i_encoder_avg_micrometers: Distancia media de encoder en micrómetros
--   i_encoder_avg_millimeters: Distancia media de encoder en milímetros
--   i_walls_status           : Estado actual de detección de paredes (front, left, right)
--   i_sensor_distance        : Array de distancias de sensores (natural, escalado)
--   i_front_wall_distance    : Distancia media a pared frontal
--   i_is_race_started        : Flag si la carrera está iniciada
--   i_get_clock_ticks        : Contador de ticks de reloj (para delays en ms)
--   i_lsm6dsr_gyro_z_degrees : Ángulo Z del giroscopio (en grados, float/fixed-point)
--   i_menu_run_get_speed     : Estrategia de velocidad actual del menú
--   i_menu_run_can_start     : Flag del menú para poder iniciar
--
-- Salidas:
--   o_movement_done_pulse      : Pulso cuando un movimiento ha terminado
--   o_set_target_linear_speed_en : Habilita el control de velocidad lineal
--   o_target_linear_speed_val  : Valor de velocidad lineal (natural)
--   o_set_ideal_angular_speed_en : Habilita el control de velocidad angular ideal
--   o_ideal_angular_speed_val  : Valor de velocidad angular ideal (signed fixed-point)
--   o_set_front_sensors_corr_en : Habilita corrección de sensores frontales
--   o_front_sensors_corr_val   : Valor de habilitación (std_logic)
--   o_set_front_sensors_diag_corr_en : Habilita corrección diagonal de sensores frontales
--   o_front_sensors_diag_corr_val: Valor de habilitación (std_logic)
--   o_set_side_sensors_close_corr_en : Habilita corrección de sensores laterales cercanos
--   o_side_sensors_close_corr_val: Valor de habilitación (std_logic)
--   o_set_side_sensors_far_corr_en : Habilita corrección de sensores laterales lejanos
--   o_side_sensors_far_corr_val: Valor de habilitación (std_logic)
--   o_disable_sensors_correction_en: Pulso para deshabilitar todas las correcciones
--   o_set_rgb_color_en         : Habilita el control de color RGB
--   o_rgb_r, o_rgb_g, o_rgb_b  : Componentes de color RGB
--   o_toggle_status_led_en     : Pulso para alternar LED de estado
--   o_set_check_motors_saturated_enabled_en : Habilita chequeo de motores saturados
--   o_check_motors_saturated_enabled_val : Valor de habilitación (std_logic)
--   o_reset_control_errors_en  : Pulso para resetear errores de control
--   o_set_starting_position_en : Pulso para establecer posición inicial
--   o_set_race_started_flag_en : Habilita flag de carrera iniciada
--   o_set_race_started_flag_val: Valor para flag de carrera iniciada
--   o_set_debug_btn_en         : Habilita botón de depuración
--   o_set_debug_btn_val        : Valor para botón de depuración
--   o_move_arc_turn_params     : Parámetros para giro en arco
--   o_move_arc_turn_en         : Pulso para iniciar giro en arco
--   o_move_inplace_turn_type   : Tipo de giro in-place
--   o_move_inplace_turn_en     : Pulso para iniciar giro in-place
--   o_lsm6dsr_set_gyro_z_degrees_en : Habilita ajuste de ángulo Z del giroscopio
--   o_lsm6dsr_set_gyro_z_degrees_val: Valor de ángulo Z (float/fixed-point)
--------------------------------------------------------------------------------
entity robot_movement_controller is
    generic (
        -- Puedes añadir generics para constantes como ROBOT_BACK_LENGTH, WALL_WIDTH etc.
        -- si quieres que sean configurables en síntesis.
    );
    port (
        i_clk                    : in  std_logic;
        i_reset                  : in  std_logic;

        -- Comandos de movimiento de alto nivel
        i_start_movement_cmd     : in  std_logic;
        i_movement_type_cmd      : in  movement;
        i_run_sequence_cmd       : in  std_logic;
        i_movement_sequence      : in  movement_sequence_array_t; -- Definir este tipo en robot_types_pkg

        -- Entradas del módulo de cinemática/configuración
        i_current_kinematics     : in  kinematics_data;
        i_kinematics_settings_rom: in  kinematics_settings_array_t; -- Pasa la ROM de settings

        -- Entradas de los módulos de sensores
        i_encoder_avg_micrometers: in  natural;
        i_encoder_avg_millimeters: in  natural;
        i_walls_status           : in  walls_status;
        i_sensor_distance        : in  sensor_distance_array_t; -- Array de distancias de sensores (natural)
        i_front_wall_distance    : in  natural; -- Distancia media a pared frontal

        -- Entradas de otros módulos de control
        i_is_race_started        : in  std_logic;
        i_get_clock_ticks        : in  natural; -- Contador de ticks de reloj
        i_lsm6dsr_gyro_z_degrees : in  signed(15 downto 0); -- Ángulo Z del giroscopio (fixed-point)
        i_menu_run_get_speed     : in  speed_strategy; -- Estrategia de velocidad actual
        i_menu_run_can_start     : in  std_logic;

        -- Salidas de control para el módulo de movimiento
        o_movement_done_pulse      : out std_logic;

        -- Salidas para Motor/Kinemática
        o_set_target_linear_speed_en : out std_logic;
        o_target_linear_speed_val  : out natural;
        o_set_ideal_angular_speed_en : out std_logic;
        o_ideal_angular_speed_val  : out signed(15 downto 0); -- Fixed-point para rad/s
        o_move_arc_turn_en         : out std_logic;
        o_move_arc_turn_params     : out turn_params;
        o_move_inplace_turn_en     : out std_logic;
        o_move_inplace_turn_type   : out movement; -- Para move_inplace_turn
        o_lsm6dsr_set_gyro_z_degrees_en : out std_logic;
        o_lsm6dsr_set_gyro_z_degrees_val: out signed(15 downto 0); -- Fixed-point para grados

        -- Salidas para corrección de sensores
        o_set_front_sensors_corr_en : out std_logic;
        o_front_sensors_corr_val   : out std_logic;
        o_set_front_sensors_diag_corr_en : out std_logic;
        o_front_sensors_diag_corr_val: out std_logic;
        o_set_side_sensors_close_corr_en : out std_logic;
        o_side_sensors_close_corr_val: out std_logic;
        o_set_side_sensors_far_corr_en : out std_logic;
        o_side_sensors_far_corr_val: out std_logic;
        o_disable_sensors_correction_en: out std_logic;

        -- Salidas para LEDs y debugging
        o_set_rgb_color_en         : out std_logic;
        o_rgb_r                    : out natural range 0 to 255;
        o_rgb_g                    : out natural range 0 to 255;
        o_rgb_b                    : out natural range 0 to 255;
        o_toggle_status_led_en     : out std_logic;
        o_set_check_motors_saturated_enabled_en : out std_logic;
        o_check_motors_saturated_enabled_val : out std_logic;
        o_reset_control_errors_en  : out std_logic;
        o_set_starting_position_en : out std_logic;
        o_set_race_started_flag_en : out std_logic;
        o_set_race_started_flag_val: out std_logic;
        o_set_debug_btn_en         : out std_logic;
        o_set_debug_btn_val        : out std_logic
    );
end entity robot_movement_controller;

architecture Behavioral of robot_movement_controller is

    -- =========================================================================
    -- ESTADOS DE LA MÁQUINA DE ESTADOS PRINCIPAL (para 'move_run_sequence')
    -- =========================================================================
    type main_fsm_state_type is (
        STATE_IDLE,
        STATE_PROCESS_SEQUENCE_ITEM,
        STATE_HANDLE_STRAIGHT_SEGMENT,
        STATE_WAIT_STRAIGHT_DONE,
        STATE_HANDLE_DIAGONAL_SEGMENT,
        STATE_WAIT_DIAGONAL_DONE,
        STATE_HANDLE_TURN_SEGMENT,
        STATE_WAIT_TURN_DONE,
        STATE_HANDLE_BACK_SEGMENT,
        STATE_WAIT_BACK_DONE,
        STATE_DONE_SEQUENCE
    );
    signal current_main_state : main_fsm_state_type := STATE_IDLE;
    signal next_main_state    : main_fsm_state_type;

    -- =========================================================================
    -- ESTADOS DE LAS SUB-MÁQUINAS DE ESTADOS (para 'move_straight', 'move_arc_turn', etc.)
    -- Cada una de estas podría ser una entidad VHDL separada si la lógica es compleja.
    -- Aquí las definimos conceptualmente como estados dentro de este mismo archivo.
    -- =========================================================================
    -- FSM para move_straight (y move_straight_until_front_distance)
    type straight_fsm_state_type is (
        STRAIGHT_IDLE,
        STRAIGHT_INIT,
        STRAIGHT_SET_SPEED,
        STRAIGHT_CHECK_WALL_LOSS,
        STRAIGHT_WAIT_MOVE_DONE,
        STRAIGHT_DECEL_WAIT,
        STRAIGHT_STOP_WAIT,
        STRAIGHT_DONE
    );
    signal current_straight_state : straight_fsm_state_type := STRAIGHT_IDLE;
    signal next_straight_state    : straight_fsm_state_type;
    signal s_straight_active      : std_logic := '0'; -- Activa la FSM de movimiento recto

    -- FSM para move_arc_turn
    type arc_turn_fsm_state_type is (
        ARC_TURN_IDLE,
        ARC_TURN_INIT,
        ARC_TURN_ACCEL,
        ARC_TURN_MAX_SPEED,
        ARC_TURN_DECEL,
        ARC_TURN_STOP_WAIT,
        ARC_TURN_DONE
    );
    signal current_arc_turn_state : arc_turn_fsm_state_type := ARC_TURN_IDLE;
    signal next_arc_turn_state    : arc_turn_fsm_state_type;
    signal s_arc_turn_active      : std_logic := '0'; -- Activa la FSM de giro en arco

    -- FSM para move_inplace_turn
    type inplace_turn_fsm_state_type is (
        INPLACE_IDLE,
        INPLACE_INIT,
        INPLACE_ACCEL,
        INPLACE_MAX_SPEED,
        INPLACE_DECEL,
        INPLACE_STOP,
        INPLACE_DONE
    );
    signal current_inplace_turn_state : inplace_turn_fsm_state_type := INPLACE_IDLE;
    signal next_inplace_turn_state    : inplace_turn_fsm_state_type;
    signal s_inplace_turn_active      : std_logic := '0'; -- Activa la FSM de giro in-place

    -- =========================================================================
    -- VARIABLES INTERNAS (equivalentes a 'static' en C)
    -- =========================================================================
    signal s_kinematics             : kinematics_data;
    signal s_current_cell_start_mm          : signed(31 downto 0) := (others => '0'); -- int32_t
    signal s_current_cell_wall_lost         : std_logic := '0'; -- bool
    signal s_current_cell_absolute_start_mm : signed(31 downto 0) := (others => '0'); -- int32_t

    signal s_wall_lost_toggle_state : std_logic := '0'; -- bool
    signal s_cell_change_toggle_state : std_logic := '0'; -- bool

    -- Variables para 'move_run_sequence'
    signal s_seq_distance           : natural := 0; -- float en C
    signal s_seq_start_offset       : natural := 0; -- float en C
    signal s_seq_end_offset         : natural := 0; -- float en C
    signal s_seq_running_diagonal   : std_logic := '0'; -- bool
    signal s_seq_straight_has_begin : std_logic := '0'; -- bool
    signal s_seq_straight_cells     : natural := 0; -- uint16_t
    signal s_seq_index              : natural range 0 to MAZE_CELLS + 3 := 0; -- Índice de la secuencia
    signal s_seq_turn_params_current : turn_params; -- Parámetros de giro

    -- Variables para move_straight / run_straight
    signal s_move_straight_distance_target : natural := 0;
    signal s_move_straight_speed_target    : signed(15 downto 0);
    signal s_move_straight_check_wall_loss : std_logic;
    signal s_move_straight_stop_at_end     : std_logic;
    signal s_straight_current_distance_micrometers : natural;
    signal s_straight_stop_distance_calc   : natural;
    signal s_straight_initial_walls        : walls_status;

    -- Variables para run_straight específica
    signal s_run_straight_distance_param : natural;
    signal s_run_straight_start_offset_param : natural;
    signal s_run_straight_end_offset_param : natural;
    signal s_run_straight_cells_param : natural;
    signal s_run_straight_has_begin_param : std_logic;
    signal s_run_straight_speed_param : natural;
    signal s_run_straight_final_speed_param : natural;
    signal s_run_straight_current_cell : natural;
    signal s_run_straight_current_cell_dist_left : natural;
    signal s_run_straight_cell_walls : walls_status;
    signal s_run_straight_slow_distance_calc : natural;

    -- Variables para move_arc_turn
    signal s_arc_turn_start_micrometers : natural;
    signal s_arc_turn_travelled_mm      : natural;
    signal s_arc_turn_angular_speed     : signed(15 downto 0); -- Fixed-point
    signal s_arc_turn_factor            : natural; -- Fixed-point (0-1000 for 0.0-1.0)

    -- Variables para move_inplace_turn
    signal s_inplace_turn_ms_start      : natural;
    signal s_inplace_turn_ms_current    : natural;
    signal s_inplace_turn_angular_speed : signed(15 downto 0); -- Fixed-point
    signal s_inplace_turn_sign          : signed(0 to 0);

    -- Variables para move_inplace_angle
    signal s_inplace_angle_target_degrees : signed(15 downto 0); -- Fixed-point degrees
    signal s_inplace_angle_rads_target    : signed(15 downto 0); -- Fixed-point rads

    -- Variables para calc_straight_to_speed_distance
    signal s_calc_from_speed_sq : natural;
    signal s_calc_to_speed_sq   : natural;
    signal s_calc_denominator   : natural;
    signal s_calc_distance_mm   : natural;

    -- Señales de pulso para las salidas de los módulos (se activan un ciclo)
    signal s_o_movement_done_pulse      : std_logic := '0';
    signal s_o_set_target_linear_speed_en_pulse : std_logic := '0';
    signal s_o_set_ideal_angular_speed_en_pulse : std_logic := '0';
    signal s_o_set_front_sensors_corr_en_pulse : std_logic := '0';
    signal s_o_set_front_sensors_diag_corr_en_pulse : std_logic := '0';
    signal s_o_set_side_sensors_close_corr_en_pulse : std_logic := '0';
    signal s_o_set_side_sensors_far_corr_en_pulse : std_logic := '0';
    signal s_o_disable_sensors_correction_en_pulse: std_logic := '0';
    signal s_o_set_rgb_color_en_pulse   : std_logic := '0';
    signal s_o_toggle_status_led_en_pulse : std_logic := '0';
    signal s_o_set_check_motors_saturated_enabled_en_pulse : std_logic := '0';
    signal s_o_reset_control_errors_en_pulse : std_logic := '0';
    signal s_o_set_starting_position_en_pulse : std_logic := '0';
    signal s_o_set_race_started_flag_en_pulse : std_logic := '0';
    signal s_o_set_debug_btn_en_pulse   : std_logic := '0';
    signal s_o_move_arc_turn_en_pulse   : std_logic := '0';
    signal s_o_move_inplace_turn_en_pulse : std_logic := '0';
    signal s_o_lsm6dsr_set_gyro_z_degrees_en_pulse : std_logic := '0';

begin

    -- =========================================================================
    -- PROCESO PRINCIPAL DE LA MÁQUINA DE ESTADOS (Secuenciador de movimientos)
    -- Equivalente a 'move_run_sequence' y 'move' en C
    -- =========================================================================
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                current_main_state <= STATE_IDLE;
                s_seq_index        <= 0;
                -- Resetear todas las variables de secuencia
                s_seq_distance           <= 0;
                s_seq_start_offset       <= 0;
                s_seq_end_offset         <= 0;
                s_seq_running_diagonal   <= '0';
                s_seq_straight_has_begin <= '0';
                s_seq_straight_cells     <= 0;
                s_o_movement_done_pulse  <= '0';
                s_arc_turn_active        <= '0';
                s_inplace_turn_active    <= '0';
                s_straight_active        <= '0';
                o_movement_done_pulse    <= '0'; -- Asegurar que el pulso se resetea
            else
                current_main_state <= next_main_state;
                -- Borrar pulsos de salida en cada ciclo
                s_o_movement_done_pulse <= '0';
                s_o_set_target_linear_speed_en_pulse <= '0';
                s_o_set_ideal_angular_speed_en_pulse <= '0';
                s_o_set_front_sensors_corr_en_pulse <= '0';
                s_o_set_front_sensors_diag_corr_en_pulse <= '0';
                s_o_set_side_sensors_close_corr_en_pulse <= '0';
                s_o_set_side_sensors_far_corr_en_pulse <= '0';
                s_o_disable_sensors_correction_en_pulse <= '0';
                s_o_set_rgb_color_en_pulse <= '0';
                s_o_toggle_status_led_en_pulse <= '0';
                s_o_set_check_motors_saturated_enabled_en_pulse <= '0';
                s_o_reset_control_errors_en_pulse <= '0';
                s_o_set_starting_position_en_pulse <= '0';
                s_o_set_race_started_flag_en_pulse <= '0';
                s_o_set_debug_btn_en_pulse <= '0';
                s_o_move_arc_turn_en_pulse <= '0';
                s_o_move_inplace_turn_en_pulse <= '0';
                s_o_lsm6dsr_set_gyro_z_degrees_en_pulse <= '0';
            end if;
        end if;
    end process;

    process (current_main_state, i_start_movement_cmd, i_run_sequence_cmd, s_seq_index,
             i_movement_sequence, s_seq_distance, s_seq_end_offset, s_seq_straight_cells,
             s_seq_running_diagonal, s_seq_straight_has_begin, i_current_kinematics,
             s_straight_active, s_arc_turn_active, s_inplace_turn_active,
             current_straight_state, current_arc_turn_state, current_inplace_turn_state,
             i_movement_type_cmd, i_menu_run_get_speed, i_menu_run_can_start,
             s_seq_turn_params_current
             )
    begin
        -- Comandos por defecto (no hacer nada)
        next_main_state <= current_main_state;

        -- Lógica de control para los pulsos (deben ser un ciclo de reloj de duración)
        o_movement_done_pulse      <= s_o_movement_done_pulse;
        o_set_target_linear_speed_en <= s_o_set_target_linear_speed_en_pulse;
        o_set_ideal_angular_speed_en <= s_o_set_ideal_angular_speed_en_pulse;
        o_set_front_sensors_corr_en <= s_o_set_front_sensors_corr_en_pulse;
        o_front_sensors_corr_val   <= '0'; -- Valor por defecto
        o_set_front_sensors_diag_corr_en <= s_o_set_front_sensors_diag_corr_en_pulse;
        o_front_sensors_diag_corr_val <= '0'; -- Valor por defecto
        o_set_side_sensors_close_corr_en <= s_o_set_side_sensors_close_corr_en_pulse;
        o_side_sensors_close_corr_val <= '0'; -- Valor por defecto
        o_set_side_sensors_far_corr_en <= s_o_set_side_sensors_far_corr_en_pulse;
        o_side_sensors_far_corr_val <= '0'; -- Valor por defecto
        o_disable_sensors_correction_en <= s_o_disable_sensors_correction_en_pulse;
        o_set_rgb_color_en         <= s_o_set_rgb_color_en_pulse;
        o_rgb_r <= 0; o_rgb_g <= 0; o_rgb_b <= 0; -- Valores por defecto
        o_toggle_status_led_en     <= s_o_toggle_status_led_en_pulse;
        o_set_check_motors_saturated_enabled_en <= s_o_set_check_motors_saturated_enabled_en_pulse;
        o_check_motors_saturated_enabled_val <= '0'; -- Valor por defecto
        o_reset_control_errors_en  <= s_o_reset_control_errors_en_pulse;
        o_set_starting_position_en <= s_o_set_starting_position_en_pulse;
        o_set_race_started_flag_en <= s_o_set_race_started_flag_en_pulse;
        o_set_race_started_flag_val <= '0'; -- Valor por defecto
        o_set_debug_btn_en         <= s_o_set_debug_btn_en_pulse;
        o_set_debug_btn_val        <= '0'; -- Valor por defecto
        o_move_arc_turn_en         <= s_o_move_arc_turn_en_pulse;
        o_move_inplace_turn_en     <= s_o_move_inplace_turn_en_pulse;
        o_lsm6dsr_set_gyro_z_degrees_en <= s_o_lsm6dsr_set_gyro_z_degrees_en_pulse;

        -- Desactivar todas las sub-FSMs por defecto
        s_straight_active <= '0';
        s_arc_turn_active <= '0';
        s_inplace_turn_active <= '0';

        case current_main_state is
            when STATE_IDLE =>
                if i_run_sequence_cmd = '1' then
                    next_main_state <= STATE_PROCESS_SEQUENCE_ITEM;
                    s_seq_index <= 0;
                    s_seq_distance <= 0;
                    s_seq_start_offset <= 0;
                    s_seq_end_offset <= 0;
                    s_seq_running_diagonal <= '0';
                    s_seq_straight_has_begin <= '0';
                    o_set_check_motors_saturated_enabled_en_pulse <= '1';
                    o_check_motors_saturated_enabled_val <= '1';
                elsif i_start_movement_cmd = '1' then
                    -- Esto activaría un movimiento individual (similar a 'move' en C)
                    -- Se necesita una FSM secundaria para manejar cada 'case' de 'move'
                    -- Por simplicidad, solo mostramos el manejo de 'move_home' y 'move_front' aquí.
                    case i_movement_type_cmd is
                        when MOVE_HOME =>
                            -- Configuración de sensores para move_home
                            o_set_front_sensors_corr_en_pulse <= '1'; o_front_sensors_corr_val <= '0';
                            o_set_front_sensors_diag_corr_en_pulse <= '1'; o_front_sensors_diag_corr_val <= '0';
                            o_set_side_sensors_close_corr_en_pulse <= '1'; o_side_sensors_close_corr_val <= '1';
                            o_set_side_sensors_far_corr_en_pulse <= '1'; o_side_sensors_far_corr_val <= '0';
                            -- Si walls.front y (no left o no right)
                            if i_walls_status.front = '1' and (i_walls_status.left = '0' or i_walls_status.right = '0') then
                                o_set_side_sensors_close_corr_en_pulse <= '1'; o_side_sensors_close_corr_val <= '0';
                            end if;
                            -- Iniciar move_straight_until_front_distance, luego move_inplace_turn, etc.
                            -- Esto requiere una FSM para 'move_home' o sub-estados complejos.
                            -- Ejemplo: next_main_state <= STATE_MOVE_HOME_STEP1;
                            null; -- Placeholder
                        when MOVE_FRONT =>
                             -- Lógica de 'move_front' de C
                            o_set_front_sensors_corr_en_pulse <= '1'; o_front_sensors_corr_val <= '0';
                            o_set_front_sensors_diag_corr_en_pulse <= '1'; o_front_sensors_diag_corr_val <= '0';
                            if i_walls_status.left = '1' or i_walls_status.right = '1' then
                                o_set_side_sensors_close_corr_en_pulse <= '1'; o_side_sensors_close_corr_val <= '1';
                                o_set_side_sensors_far_corr_en_pulse <= '1'; o_side_sensors_far_corr_val <= '1';
                            else
                                o_set_side_sensors_close_corr_en_pulse <= '1'; o_side_sensors_close_corr_val <= '0';
                                o_set_side_sensors_far_corr_en_pulse <= '1'; o_side_sensors_far_corr_val <= '0';
                            end if;
                            -- Iniciar move_straight, luego enter_next_cell
                            -- next_main_state <= STATE_MOVE_FRONT_STEP1;
                            null; -- Placeholder
                        when others => null; -- Otros movimientos
                    end case;
                end if;


            when STATE_PROCESS_SEQUENCE_ITEM =>
                -- Verifica si la secuencia ha terminado (o un elemento inválido)
                if s_seq_index >= MAZE_CELLS + 3 or i_movement_sequence(s_seq_index) = MOVE_NONE then
                    next_main_state <= STATE_DONE_SEQUENCE;
                else
                    case i_movement_sequence(s_seq_index) is
                        when MOVE_START =>
                            s_seq_distance           <= s_seq_distance + (C_CELL_DIMENSION - (C_ROBOT_BACK_LENGTH + C_WALL_WIDTH / 2));
                            s_seq_straight_cells     <= s_seq_straight_cells + 1;
                            s_seq_straight_has_begin <= '1';
                            s_seq_index <= s_seq_index + 1;
                            next_main_state <= STATE_PROCESS_SEQUENCE_ITEM; -- Sigue procesando
                        when MOVE_FRONT =>
                            s_seq_distance           <= s_seq_distance + C_CELL_DIMENSION;
                            s_seq_straight_cells     <= s_seq_straight_cells + 1;
                            s_seq_index <= s_seq_index + 1;
                            next_main_state <= STATE_PROCESS_SEQUENCE_ITEM;
                        when MOVE_DIAGONAL =>
                            s_seq_running_diagonal <= '1';
                            s_seq_distance         <= s_seq_distance + C_CELL_DIAGONAL;
                            s_seq_straight_cells   <= s_seq_straight_cells + 1;
                            s_seq_index <= s_seq_index + 1;
                            next_main_state <= STATE_PROCESS_SEQUENCE_ITEM;
                        when MOVE_HOME =>
                            if s_seq_distance > 0 then
                                if s_seq_running_diagonal = '1' then
                                    -- Iniciar run_diagonal
                                    s_straight_active <= '1';
                                    current_straight_state <= STRAIGHT_INIT; -- Reutilizar FSM de straight para diagonal
                                    s_move_straight_distance_target <= s_seq_distance;
                                    s_move_straight_speed_target <= to_signed(i_current_kinematics.linear_speed, 16);
                                    s_move_straight_stop_at_end <= '1'; -- Se detiene al final
                                    s_move_straight_check_wall_loss <= '0'; -- No aplica aquí
                                    next_main_state <= STATE_WAIT_DIAGONAL_DONE;
                                else
                                    -- Iniciar run_straight
                                    s_straight_active <= '1';
                                    current_straight_state <= STRAIGHT_INIT;
                                    s_move_straight_distance_target <= s_seq_distance;
                                    s_move_straight_speed_target <= to_signed(i_current_kinematics.linear_speed, 16);
                                    s_move_straight_stop_at_end <= '1'; -- Se detiene al final
                                    s_move_straight_check_wall_loss <= '0'; -- No aplica aquí
                                    next_main_state <= STATE_WAIT_STRAIGHT_DONE;
                                end if;
                            else
                                -- Mover a casa directamente (move_home)
                                -- Esto requiere una FSM específica para 'move_home'
                                -- next_main_state <= STATE_MOVE_HOME_FSM_START;
                                null; -- Placeholder
                            end if;
                            s_seq_distance           <= 0;
                            s_seq_start_offset       <= 0;
                            s_seq_end_offset         <= 0;
                            s_seq_straight_cells     <= 0;
                            s_seq_straight_has_begin <= '0';
                            s_seq_running_diagonal   <= '0';
                            s_seq_index <= s_seq_index + 1; -- Sigue al siguiente elemento
                        when MOVE_LEFT | MOVE_RIGHT | MOVE_LEFT_90 | MOVE_RIGHT_90 | MOVE_LEFT_180 | MOVE_RIGHT_180 |
                             MOVE_LEFT_TO_45 | MOVE_RIGHT_TO_45 | MOVE_LEFT_TO_135 | MOVE_RIGHT_TO_135 |
                             MOVE_LEFT_45_TO_45 | MOVE_RIGHT_45_TO_45 | MOVE_LEFT_FROM_45 | MOVE_RIGHT_FROM_45 |
                             MOVE_LEFT_FROM_45_180 | MOVE_RIGHT_FROM_45_180 =>

                            s_seq_turn_params_current <= i_kinematics_settings_rom(i_menu_run_get_speed).turns(i_movement_sequence(s_seq_index));
                            s_seq_end_offset <= s_seq_turn_params_current.end_turn; -- Ajusta el offset final si 'start' es negativo

                            -- Lógica de ajuste de velocidad basada en 'calc_straight_to_speed_distance'
                            -- Esto puede ser un bucle en C, pero en VHDL se pre-calcula o se hace en un ciclo.
                            -- Aquí asumimos que los 'turns' ya tienen los parámetros correctos.
                            -- Si necesitas adaptar 'turn_params' dinámicamente, requiere una FSM aquí.

                            -- Resetea el offset para giros de 180 si no le sigue un MOVE_FRONT
                            if (i_movement_sequence(s_seq_index) = MOVE_LEFT_180 or i_movement_sequence(s_seq_index) = MOVE_RIGHT_180) and
                               ((s_seq_index + 1) < (MAZE_CELLS + 3)) and (i_movement_sequence(s_seq_index + 1) /= MOVE_FRONT) then
                                s_seq_end_offset <= 0; -- Reset if no straight follows
                            end if;

                            if s_seq_distance > 0 or s_seq_end_offset > 0 then
                                if s_seq_running_diagonal = '1' then
                                    -- Iniciar run_diagonal
                                    s_straight_active <= '1';
                                    current_straight_state <= STRAIGHT_INIT;
                                    s_move_straight_distance_target <= s_seq_distance;
                                    s_move_straight_speed_target <= to_signed(i_current_kinematics.linear_speed, 16);
                                    s_move_straight_stop_at_end <= '0'; -- No detiene completamente
                                    s_move_straight_check_wall_loss <= '0';
                                    next_main_state <= STATE_WAIT_DIAGONAL_DONE;
                                else
                                    -- Iniciar run_straight
                                    s_straight_active <= '1';
                                    current_straight_state <= STRAIGHT_INIT;
                                    s_move_straight_distance_target <= s_seq_distance;
                                    s_move_straight_speed_target <= to_signed(i_current_kinematics.linear_speed, 16);
                                    s_move_straight_stop_at_end <= '0'; -- No detiene completamente
                                    s_move_straight_check_wall_loss <= '0';
                                    next_main_state <= STATE_WAIT_STRAIGHT_DONE;
                                end if;
                            else -- Si no hubo movimiento recto previo, pasa directo al giro
                                next_main_state <= STATE_HANDLE_TURN_SEGMENT;
                            end if;
                            -- Después del recto, la distancia para el siguiente giro es 'turn.end'
                            if s_seq_turn_params_current.end_turn < 0 then
                                s_seq_distance <= abs(s_seq_turn_params_current.end_turn);
                                s_seq_start_offset <= abs(s_seq_turn_params_current.end_turn);
                            else
                                s_seq_distance <= 0;
                            end if;
                            s_seq_end_offset         <= 0;
                            s_seq_straight_cells     <= 0;
                            s_seq_straight_has_begin <= '0';
                            s_seq_running_diagonal   <= '0';

                            -- Resetear offset para giros de 180 si no le sigue un MOVE_FRONT
                            if (i_movement_sequence(s_seq_index) = MOVE_LEFT_180 or i_movement_sequence(s_seq_index) = MOVE_RIGHT_180) and
                               ((s_seq_index + 1) < (MAZE_CELLS + 3)) and (i_movement_sequence(s_seq_index + 1) /= MOVE_FRONT) then
                                s_seq_distance <= 0;
                            end if;

                        when MOVE_BACK | MOVE_BACK_WALL | MOVE_BACK_STOP =>
                            -- Esto requiere una FSM para 'move_back' o sub-estados complejos.
                            -- next_main_state <= STATE_MOVE_BACK_FSM_START;
                            null; -- Placeholder
                            s_seq_index <= s_seq_index + 1; -- Sigue al siguiente elemento
                            next_main_state <= STATE_PROCESS_SEQUENCE_ITEM;

                        when others =>
                            next_main_state <= STATE_DONE_SEQUENCE; -- Termina la secuencia si es un movimiento desconocido
                    end case;
                end if;

            when STATE_WAIT_STRAIGHT_DONE =>
                if current_straight_state = STRAIGHT_DONE then
                    next_main_state <= STATE_HANDLE_TURN_SEGMENT; -- Después de recto, normalmente viene un giro
                    s_straight_active <= '0'; -- Desactiva la FSM de movimiento recto
                end if;

            when STATE_WAIT_DIAGONAL_DONE =>
                if current_straight_state = STRAIGHT_DONE then
                    next_main_state <= STATE_HANDLE_TURN_SEGMENT; -- Después de diagonal, normalmente viene un giro
                    s_straight_active <= '0'; -- Desactiva la FSM de movimiento recto
                end if;

            when STATE_HANDLE_TURN_SEGMENT =>
                -- Iniciar run_side
                s_arc_turn_active <= '1';
                current_arc_turn_state <= ARC_TURN_INIT;
                o_move_arc_turn_params <= s_seq_turn_params_current; -- Pasa los parámetros de giro
                s_o_move_arc_turn_en_pulse <= '1'; -- Pulso para iniciar giro en arco
                next_main_state <= STATE_WAIT_TURN_DONE;

            when STATE_WAIT_TURN_DONE =>
                if current_arc_turn_state = ARC_TURN_DONE then
                    next_main_state <= STATE_PROCESS_SEQUENCE_ITEM; -- Vuelve a procesar el siguiente elemento
                    s_arc_turn_active <= '0'; -- Desactiva la FSM de giro en arco
                    s_seq_index <= s_seq_index + 1;
                end if;

            when STATE_DONE_SEQUENCE =>
                o_movement_done_pulse <= '1'; -- Señal de fin de secuencia
                next_main_state <= STATE_IDLE;
                o_set_check_motors_saturated_enabled_en_pulse <= '1';
                o_check_motors_saturated_enabled_val <= '0'; -- Deshabilita el chequeo
                o_set_race_started_flag_en_pulse <= '1';
                o_set_race_started_flag_val <= '0'; -- Termina la carrera
                -- resetear todas las variables de estado de secuencia
                s_seq_distance           <= 0;
                s_seq_start_offset       <= 0;
                s_seq_end_offset         <= 0;
                s_seq_running_diagonal   <= '0';
                s_seq_straight_has_begin <= '0';
                s_seq_straight_cells     <= 0;
                s_seq_index              <= 0;

            -- =================================================================
            -- Manejo de Movimientos Individuales (similares a funciones de C)
            -- Estas FSMs serían para 'move_straight', 'move_arc_turn', etc.
            -- Son llamadas por la FSM principal o por 'i_start_movement_cmd'.
            -- =================================================================
            -- FSM para move_straight (y run_straight/run_diagonal)
            when others => null; -- Evita warnings si no todas las transiciones están cubiertas.
        end case;
    end process;


    -- =========================================================================
    -- LÓGICA DE LAS SUB-MÁQUINAS DE ESTADOS (Ejemplo para 'move_straight')
    -- Esto puede ser en procesos separados o entidades instanciadas.
    -- =========================================================================
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                current_straight_state <= STRAIGHT_IDLE;
                s_o_movement_done_pulse <= '0';
            elsif s_straight_active = '1' then -- Solo si la FSM está activa
                current_straight_state <= next_straight_state;
            else
                current_straight_state <= STRAIGHT_IDLE; -- Asegurarse de que esté en IDLE si no está activa
                s_o_movement_done_pulse <= '0'; -- Asegurarse de que el pulso esté apagado
            end if;
        end if;
    end process;

    process (current_straight_state, i_is_race_started, i_encoder_avg_micrometers,
             s_move_straight_distance_target, s_move_straight_speed_target,
             s_move_straight_check_wall_loss, s_move_straight_stop_at_end,
             s_straight_current_distance_micrometers, s_straight_stop_distance_calc,
             s_straight_initial_walls, i_walls_status, i_sensor_distance)
    begin
        next_straight_state <= current_straight_state;
        -- Defaults para salidas de esta FSM
        o_set_target_linear_speed_en <= '0'; o_target_linear_speed_val <= 0;
        o_set_ideal_angular_speed_en <= '0'; o_ideal_angular_speed_val <= (others => '0');
        o_set_rgb_color_en <= '0'; o_rgb_r <= 0; o_rgb_g <= 0; o_rgb_b <= 0;
        o_disable_sensors_correction_en <= '0';

        case current_straight_state is
            when STRAIGHT_IDLE =>
                -- Espera la activación desde la FSM principal
                null; -- No hace nada

            when STRAIGHT_INIT =>
                -- Inicializa el movimiento recto
                o_set_ideal_angular_speed_en <= '1'; o_ideal_angular_speed_val <= (others => '0'); -- 0.0
                o_set_target_linear_speed_en <= '1'; o_target_linear_speed_val <= to_natural(s_move_straight_speed_target);
                s_straight_current_distance_micrometers <= i_encoder_avg_micrometers;
                s_straight_initial_walls <= i_walls_status; -- Captura el estado inicial de las paredes
                next_straight_state <= STRAIGHT_CHECK_WALL_LOSS;

            when STRAIGHT_CHECK_WALL_LOSS =>
                if i_is_race_started = '0' then
                    next_straight_state <= STRAIGHT_DONE; -- Si la carrera se detiene, termina
                elsif s_move_straight_check_wall_loss = '1' and check_wall_loss_correction_logic(s_straight_initial_walls, i_walls_status, s_current_cell_wall_lost) = '1' then
                    -- Lógica de corrección de pérdida de pared
                    s_straight_current_distance_micrometers <= i_encoder_avg_micrometers;
                    s_move_straight_distance_target <= C_WALL_LOSS_TO_SENSING_POINT_DISTANCE;
                    o_set_rgb_color_en <= '1'; o_rgb_r <= 0; o_rgb_g <= 255; o_rgb_b <= 0; -- Color verde (while)
                    next_straight_state <= STRAIGHT_SET_SPEED; -- Reinicia la velocidad si es necesario
                else
                    next_straight_state <= STRAIGHT_SET_SPEED;
                end if;

            when STRAIGHT_SET_SPEED =>
                -- Cálculos para la parada si 'stop' es verdadero
                if s_move_straight_stop_at_end = '1' then
                    -- calc_straight_to_speed_distance(get_ideal_linear_speed(), 0)
                    s_straight_stop_distance_calc <= calc_straight_to_speed_distance_logic(to_natural(o_target_linear_speed_val), 0, i_current_kinematics.linear_accel.break_accel);
                else
                    s_straight_stop_distance_calc <= 0;
                end if;
                next_straight_state <= STRAIGHT_WAIT_MOVE_DONE;

            when STRAIGHT_WAIT_MOVE_DONE =>
                -- Comprobación de distancia (simulando el while loop)
                if i_is_race_started = '0' then
                    next_straight_state <= STRAIGHT_DONE; -- Si la carrera se detiene, termina
                elsif to_natural(i_encoder_avg_micrometers) >= s_straight_current_distance_micrometers +
                                                              (s_move_straight_distance_target - s_straight_stop_distance_calc) * C_MICROMETERS_PER_MILLIMETER then
                    -- Distancia alcanzada
                    if s_move_straight_stop_at_end = '1' then
                        next_straight_state <= STRAIGHT_STOP_WAIT;
                    else
                        next_straight_state <= STRAIGHT_DONE;
                    end if;
                end if;
                -- Continuar verificando la pérdida de pared
                next_straight_state <= STRAIGHT_CHECK_WALL_LOSS; -- Vuelve a check_wall_loss para re-evaluar

            when STRAIGHT_STOP_WAIT =>
                -- Asegura que el robot se detiene completamente
                o_set_target_linear_speed_en <= '1'; o_target_linear_speed_val <= 0;
                o_set_ideal_angular_speed_en <= '1'; o_ideal_angular_speed_val <= (others => '0');
                if i_is_race_started = '0' or o_target_linear_speed_val = 0 then -- Asumiendo que o_target_linear_speed_val se actualiza por el controlador de motores
                    -- y que el error angular también se reduce a cero si está habilitado.
                    next_straight_state <= STRAIGHT_DONE;
                end if;

            when STRAIGHT_DONE =>
                s_o_movement_done_pulse <= '1'; -- Pulso de finalización
                next_straight_state <= STRAIGHT_IDLE; -- Vuelve a IDLE
        end case;
    end process;


    -- =========================================================================
    -- LÓGICA DE LAS FUNCIONES INTERNAS DE C (combinacional o con registros)
    -- =========================================================================

    -- Función: calc_straight_to_speed_distance
    -- Los valores de entrada/salida son enteros escalados (fixed-point).
    s_calc_from_speed_sq <= to_natural(abs(to_signed(calc_straight_to_speed_distance_val_from_speed_logic(to_natural(o_target_linear_speed_val)), 16))); -- asume velocidad es natural
    s_calc_to_speed_sq   <= 0; -- to_speed es 0 en el caso de stop

    -- El denominador (2 * break_accel) también debe ser escalado
    s_calc_denominator <= 2 * i_current_kinematics.linear_accel.break_accel; -- Asume break_accel ya escalado

    -- Cálculo de la distancia (fixed-point division)
    -- Asumo que la división de 'natural' devuelve un 'natural' (entero),
    -- y que el resultado está en la misma escala que 'distance' (mm).
    -- La división real en VHDL requiere un IP core o una FSM de división.
    s_calc_distance_mm <= (s_calc_from_speed_sq - s_calc_to_speed_sq) / s_calc_denominator;

    -- Función: enter_next_cell
    -- Esto se activa como una secuencia de acciones en una FSM cuando se requiere.
    -- Aquí solo definimos el comportamiento de las variables que se cambian.
    -- Este proceso será parte de la FSM principal o de 'run_straight'.
    --
    -- Cuando se active la secuencia de 'enter_next_cell':
    -- current_cell_start_mm <= -C_SENSING_POINT_DISTANCE;
    -- current_cell_absolute_start_mm <= to_signed(i_encoder_avg_millimeters, 32);
    -- current_cell_wall_lost <= '0';
    -- o_set_rgb_color_en <= '1'; o_rgb_r <= 255; o_rgb_g <= 0; o_rgb_b <= 0; -- set_RGB_color_while(255, 0, 0, 33)
    -- o_toggle_status_led_en <= '1';
    -- s_cell_change_toggle_state <= not s_cell_change_toggle_state;


    -- Función: check_wall_loss_correction
    -- Lógica combinacional (o en un proceso para lectura de entradas)
    function check_wall_loss_correction_logic (
        initial_walls : walls_status;
        current_walls : walls_status;
        current_cell_wall_lost_flag : std_logic
    ) return std_logic is
        variable v_wall_lost : std_logic := '0';
    begin
        if current_cell_wall_lost_flag = '1' then
            return '0';
        end if;
        if initial_walls.left = '1' and current_walls.left = '0' then
            v_wall_lost := '1';
        elsif initial_walls.right = '1' and current_walls.right = '0' then
            v_wall_lost := '1';
        end if;
        -- TODO: La lógica comentada de C para count_check_walls_left/right requiere contadores y FSM.
        return v_wall_lost;
    end function;

    -- Función: configure_kinematics
    -- Asignación directa de la cinemática seleccionada
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                s_kinematics <= (others => (others => '0'));
            elsif o_configure_kinematics_en = '1' then -- Asumo que hay un pulso para configurar
                s_kinematics <= i_kinematics_settings_rom(i_menu_run_get_speed);
            end if;
        end if;
    end process;
    -- o_configure_kinematics_en debería ser una salida de este módulo, activada cuando se llama.

    -- Función: get_kinematics
    -- Solo es el acceso a la señal 's_kinematics'
    o_target_linear_speed_val <= s_kinematics.linear_speed; -- Ejemplo de uso en las salidas

    -- Funciones 'get_floodfill_XXX'
    o_set_speed <= i_kinematics_settings_rom(SPEED_HAKI).turns(MOVE_LEFT_90).linear_speed;
    -- ... y las otras asignaciones directas de las constantes.


    -- Asignación de salidas de pulsos
    -- Los pulsos se gestionan en el proceso principal y se asignan a las salidas
    -- o_set_target_linear_speed_en, o_set_ideal_angular_speed_en, etc.
    -- Los valores (o_target_linear_speed_val, o_ideal_angular_speed_val, etc.)
    -- se asignan dentro de las FSMs o la lógica combinacional.

    -- Asignación de outputs que no son pulsos
    o_target_linear_speed_val  <= to_natural(s_move_straight_speed_target); -- Ejemplo de conexión
    o_ideal_angular_speed_val  <= s_arc_turn_angular_speed; -- Ejemplo de conexión

    -- Asegurar que los toggle states se mantengan
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                s_wall_lost_toggle_state <= '0';
                s_cell_change_toggle_state <= '0';
            else
                if s_o_toggle_status_led_en_pulse = '1' then
                    -- Esta lógica es manejada por el módulo de LED
                    null;
                end if;
            end if;
        end if;
    end process;
    o_toggle_status_led_en <= s_o_toggle_status_led_en_pulse;
    o_set_rgb_color_en <= s_o_set_rgb_color_en_pulse;

    -- Conexión de salidas de estado a los puertos
    o_front_sensors_corr_val   <= '0'; -- Será controlado por FSMs
    o_front_sensors_diag_corr_val <= '0';
    o_side_sensors_close_corr_val <= '0';
    o_side_sensors_far_corr_val <= '0';
    o_check_motors_saturated_enabled_val <= '0';
    o_set_race_started_flag_val <= '0';
    o_set_debug_btn_val <= '0';


end architecture Behavioral;
