
-- floodfill.vhd
-- Este módulo es el controlador principal (top-level orchestrator)
-- para el algoritmo Floodfill en el robot. Coordina la exploración del laberinto
-- y la ejecución de la carrera, instanciando y controlando módulos auxiliares
-- para tareas específicas.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.robot_types_pkg.all; -- Paquete de tipos y constantes personalizados

entity floodfill_controller is
    port (
        i_clk                       : in  std_logic;                                -- Reloj del sistema
        i_reset                     : in  std_logic;                                -- Reset síncrono (activo alto)
        i_get_clock_ticks           : in  natural;                                  -- Ticks del reloj en ms (desde un timer global)

        -- Entradas de comando
        i_start_explore_en          : in  std_logic;                                -- Pulso para iniciar modo exploración
        i_start_run_en              : in  std_logic;                                -- Pulso para iniciar modo carrera
        i_loop_en                   : in  std_logic;                                -- Pulso periódico para el bucle principal (Explore/Run)

        -- Estado actual del robot (sensores)
        i_walls_status              : in  walls_status;                             -- Estado de los sensores de pared (front, left, right)
        i_current_position          : in  natural range 0 to MAZE_CELLS - 1;        -- Posición actual del robot (desde encoders/odometría)
        i_current_direction         : in  compass_direction;                        -- Dirección actual del robot (desde IMU/odometría)
        i_is_race_started           : in  std_logic;                                -- Estado de la carrera (desde Race Manager)
        i_get_kinematics_fan_speed  : in  natural range 0 to 100;                   -- Velocidad del ventilador (desde Kinematics/menu)
        i_menu_run_speed_strategy   : in  speed_strategy;                           -- Estrategia de velocidad actual del menú

        -- Salidas para controlar otros módulos del robot
        o_configure_kinematics_en   : out std_logic;                                -- Pulso para configurar cinemática
        o_configure_kinematics_speed: out speed_strategy;                           -- Velocidad a configurar
        o_clear_info_leds_en        : out std_logic;                                -- Pulso para limpiar LEDs
        o_set_rgb_color_en          : out std_logic;                                -- Pulso para establecer color RGB
        o_rgb_r_val                 : out natural range 0 to 255;
        o_rgb_g_val                 : out natural range 0 to 255;
        o_rgb_b_val                 : out natural range 0 to 255;
        o_set_rgb_color_while_en    : out std_logic;                                -- Pulso para establecer color RGB temporal
        o_rgb_r_while_val           : out natural range 0 to 255;
        o_rgb_g_while_val           : out natural range 0 to 255;
        o_rgb_b_while_val           : out natural range 0 to 255;
        o_rgb_while_duration_ms     : out natural;
        o_set_target_fan_speed_en   : out std_logic;                                -- Pulso para velocidad del ventilador
        o_target_fan_speed_val      : out natural range 0 to 100;
        o_target_fan_speed_duration_ms : out natural;
        o_move_en                   : out std_logic;                                -- Pulso para movimiento
        o_move_type                 : out movement;                                 -- Tipo de movimiento
        o_set_race_started_en       : out std_logic;                                -- Pulso para Race Manager
        o_race_started_val          : out std_logic;
        o_set_target_linear_speed_en : out std_logic;                               -- Pulso para velocidad lineal
        o_target_linear_speed_val   : out natural;
        o_set_ideal_angular_speed_en : out std_logic;                               -- Pulso para velocidad angular
        o_ideal_angular_speed_val   : out signed(31 downto 0);
        o_warning_status_led_en     : out std_logic;                                -- Pulso para LED de advertencia
        o_warning_status_led_freq   : out natural;
        o_set_status_led_en         : out std_logic;                                -- Pulso para LED de estado (on/off)
        o_status_led_val            : out std_logic;

        -- Salidas de EEPROM (para guardar/cargar laberinto)
        o_eeprom_set_data_en        : out std_logic;
        o_eeprom_data_index         : out natural;
        o_eeprom_data_array         : out T_MAZE_ARRAY; -- O un tipo que coincida con int16_t[]
        o_eeprom_data_length        : out natural;
        o_eeprom_save_en            : out std_logic;
        i_eeprom_data_loaded        : in  T_MAZE_ARRAY; -- Datos cargados desde EEPROM

        -- Debug outputs (optional)
        o_debug_current_state       : out natural;
        o_debug_floodfill_val       : out T_FIXED_POINT
    );
end entity floodfill_controller;

architecture Behavioral of floodfill_controller is

    -- =========================================================================
    -- ESTADOS DE LA MÁQUINA DE ESTADOS PRINCIPAL (Top-Level)
    -- =========================================================================
    type main_fsm_state_type is (
        STATE_IDLE,                 -- Esperando comando
        STATE_COMMON_INIT_START,    -- Inicialización común a Explore/Run
        STATE_EXPLORE_INIT_MAZE,    -- Inicialización de laberinto (Explore)
        STATE_EXPLORE_MAIN_LOOP,    -- Bucle principal de exploración
        STATE_EXPLORE_CELL_VISITED, -- Celda visitada, re-calc floodfill
        STATE_EXPLORE_MOVE_TO_TARGET, -- Moviéndose a celda objetivo
        STATE_EXPLORE_GOAL_REACHED_FRONT, -- Celdas meta: moverse adelante
        STATE_EXPLORE_GOAL_REACHED_BACK, -- Celdas meta: moverse atras y guardar
        STATE_EXPLORE_FINISHED,     -- Exploración completa
        STATE_RUN_INIT_BUILD_SEQ,   -- Inicialización de carrera: construir secuencia
        STATE_RUN_MAIN_LOOP,        -- Bucle principal de carrera
        STATE_RUN_FINISHED,         -- Carrera completa
        STATE_ERROR_STOP,           -- Estado de error o parada forzada
        STATE_DELAY_ACTIVE          -- Estado para gestionar delays (p.ej. 500ms inicial)
    );
    signal current_state : main_fsm_state_type := STATE_IDLE;
    signal next_state    : main_fsm_state_type;

    -- =========================================================================
    -- SEÑALES INTERNAS (equivalentes a variables 'static' en C)
    -- =========================================================================
    signal s_start_ms          : natural := 0;
    signal s_time_limit        : natural := 0;
    signal s_race_mode         : std_logic := '0'; -- '1' = run, '0' = explore

    -- Variables de estado del laberinto (mapeadas a Block RAMs o Distributed RAM)
    signal s_maze_data         : T_MAZE_ARRAY;
    signal s_floodfill_data    : T_FLOODFILL_ARRAY;

    -- Datos de la cola y pilas
    signal s_cells_queue       : cells_queue_t;
    signal s_target_cells      : cells_stack_t;
    signal s_goal_cells        : cells_stack_t;

    -- Secuencias de movimiento
    signal s_run_sequence_movements : T_RUN_SEQUENCE_MOVEMENTS_ARRAY;
    signal s_run_sequence_idx       : natural range 0 to MAZE_CELLS + 2;

    -- Pesos de Floodfill (desde floodfill_weights.vhd)
    signal s_straight_weights       : T_CELL_WEIGHT_ARRAY;
    signal s_straight_weights_count : natural;
    signal s_diagonal_weights       : T_CELL_WEIGHT_ARRAY;
    signal s_diagonal_weights_count : natural;

    -- Señales para controlar la instanciación de módulos
    -- (Estas señales serían entradas/salidas a los sub-módulos)
    signal s_maze_logic_update_en       : std_logic := '0';
    signal s_maze_logic_wall_exists     : std_logic := '0';
    signal s_maze_logic_is_visited      : std_logic := '0';
    signal s_maze_logic_is_goal         : std_logic := '0';
    signal s_floodfill_core_calc_en     : std_logic := '0';
    signal s_floodfill_core_calc_done   : std_logic := '0';
    signal s_floodfill_core_next_step   : step_direction;
    signal s_run_seq_gen_build_en       : std_logic := '0';
    signal s_run_seq_gen_smooth_en      : std_logic := '0';
    signal s_run_seq_gen_done           : std_logic := '0';


    -- Señales para el delay
    signal s_delay_start_ms    : natural := 0;
    signal s_delay_duration_ms : natural := 0;
    signal s_delay_done_pulse  : std_logic := '0';

    -- Pulses for one-shot outputs (will be reset each clock cycle)
    signal s_o_configure_kinematics_en_pulse   : std_logic := '0';
    signal s_o_clear_info_leds_en_pulse        : std_logic := '0';
    signal s_o_set_rgb_color_en_pulse          : std_logic := '0';
    signal s_o_set_rgb_color_while_en_pulse    : std_logic := '0';
    signal s_o_set_target_fan_speed_en_pulse   : std_logic := '0';
    signal s_o_move_en_pulse                   : std_logic := '0';
    signal s_o_set_race_started_en_pulse       : std_logic := '0';
    signal s_o_set_target_linear_speed_en_pulse: std_logic := '0';
    signal s_o_set_ideal_angular_speed_en_pulse: std_logic := '0';
    signal s_o_warning_status_led_en_pulse     : std_logic := '0';
    signal s_o_set_status_led_en_pulse         : std_logic := '0';
    signal s_o_eeprom_set_data_en_pulse        : std_logic := '0';
    signal s_o_eeprom_save_en_pulse            : std_logic := '0';


begin

    -- =========================================================================
    -- GESTIÓN DEL RELOJ Y RESET
    -- =========================================================================
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                current_state <= STATE_IDLE;
                s_race_mode <= '0';
                s_start_ms <= 0;
                s_time_limit <= 0;
                s_run_sequence_idx <= 0;

                -- Reset all pulse outputs
                s_o_configure_kinematics_en_pulse   <= '0';
                s_o_clear_info_leds_en_pulse        <= '0';
                s_o_set_rgb_color_en_pulse          <= '0';
                s_o_set_rgb_color_while_en_pulse    <= '0';
                s_o_set_target_fan_speed_en_pulse   <= '0';
                s_o_move_en_pulse                   <= '0';
                s_o_set_race_started_en_pulse       <= '0';
                s_o_set_target_linear_speed_en_pulse <= '0';
                s_o_set_ideal_angular_speed_en_pulse <= '0';
                s_o_warning_status_led_en_pulse     <= '0';
                s_o_set_status_led_en_pulse         <= '0';
                s_o_eeprom_set_data_en_pulse        <= '0';
                s_o_eeprom_save_en_pulse            <= '0';
                o_race_started_val                  <= '0';
                o_status_led_val                    <= '0';
                o_move_type                         <= MOVE_NONE;
                o_configure_kinematics_speed        <= SPEED_EXPLORE; -- Default

            else
                current_state <= next_state;

                -- Reset all pulse outputs at the start of each clock cycle
                s_o_configure_kinematics_en_pulse   <= '0';
                s_o_clear_info_leds_en_pulse        <= '0';
                s_o_set_rgb_color_en_pulse          <= '0';
                s_o_set_rgb_color_while_en_pulse    <= '0';
                s_o_set_target_fan_speed_en_pulse   <= '0';
                s_o_move_en_pulse                   <= '0';
                s_o_set_race_started_en_pulse       <= '0';
                s_o_set_target_linear_speed_en_pulse <= '0';
                s_o_set_ideal_angular_speed_en_pulse <= '0';
                s_o_warning_status_led_en_pulse     <= '0';
                s_o_set_status_led_en_pulse         <= '0';
                s_o_eeprom_set_data_en_pulse        <= '0';
                s_o_eeprom_save_en_pulse            <= '0';

                -- Delay manager (non-blocking)
                if s_delay_duration_ms /= 0 and (i_get_clock_ticks - s_delay_start_ms) >= s_delay_duration_ms then
                    s_delay_duration_ms <= 0;
                    s_delay_done_pulse <= '1';
                else
                    s_delay_done_pulse <= '0';
                end if;

            end if;
        end if;
    end process;

    -- =========================================================================
    -- LÓGICA DE TRANSICIÓN DE ESTADOS Y ACCIONES COMBINACIONALES
    -- =========================================================================
    process (current_state, i_start_explore_en, i_start_run_en, i_loop_en,
             s_delay_done_pulse, i_get_clock_ticks, s_time_limit, s_start_ms,
             s_race_mode, i_menu_run_speed_strategy, i_get_kinematics_fan_speed,
             i_walls_status, i_current_position, i_current_direction, i_is_race_started,
             s_floodfill_core_calc_done, s_floodfill_core_next_step, s_run_seq_gen_done,
             s_maze_data, s_floodfill_data, s_run_sequence_movements, s_run_sequence_idx)
    begin
        next_state <= current_state;

        -- Default outputs (will be overwritten if a state needs to assert them)
        o_rgb_r_val <= 0; o_rgb_g_val <= 0; o_rgb_b_val <= 0;
        o_rgb_r_while_val <= 0; o_rgb_g_while_val <= 0; o_rgb_b_while_val <= 0; o_rgb_while_duration_ms <= 0;
        o_target_fan_speed_val <= 0; o_target_fan_speed_duration_ms <= 0;
        o_race_started_val <= '0';
        o_status_led_val <= '0';
        o_target_linear_speed_val <= 0;
        o_ideal_angular_speed_val <= (others => '0');
        o_warning_status_led_freq <= 0;
        o_eeprom_data_index <= 0; o_eeprom_data_length <= 0; o_eeprom_data_array <= (others => (others => '0'));

        -- Default control signals for sub-modules
        s_maze_logic_update_en <= '0';
        s_floodfill_core_calc_en <= '0';
        s_run_seq_gen_build_en <= '0';
        s_run_seq_gen_smooth_en <= '0';
        s_run_sequence_idx <= s_run_sequence_idx; -- Keep current index

        o_debug_current_state <= to_natural(unsigned(to_signed(current_state'pos, 8)));
        o_debug_floodfill_val <= s_floodfill_data(i_current_position);


        case current_state is
            when STATE_IDLE =>
                if i_start_explore_en = '1' then
                    s_race_mode <= '0'; -- Exploration mode
                    next_state <= STATE_COMMON_INIT_START;
                elsif i_start_run_en = '1' then
                    s_race_mode <= '1'; -- Race mode
                    next_state <= STATE_COMMON_INIT_START;
                elsif i_loop_en = '1' then -- If already in a mode (e.g. from previous run)
                    if s_race_mode = '1' then
                        next_state <= STATE_RUN_MAIN_LOOP;
                    else
                        next_state <= STATE_EXPLORE_MAIN_LOOP;
                    end if;
                end if;

            when STATE_COMMON_INIT_START =>
                s_o_configure_kinematics_en_pulse <= '1';
                if s_race_mode = '1' then
                    o_configure_kinematics_speed <= i_menu_run_speed_strategy;
                else
                    o_configure_kinematics_speed <= SPEED_EXPLORE;
                end if;
                s_o_clear_info_leds_en_pulse <= '1';
                s_o_set_rgb_color_en_pulse <= '1'; -- set_RGB_color(0, 0, 0)
                o_rgb_r_val <= 0; o_rgb_g_val <= 0; o_rgb_b_val <= 0;
                s_o_set_target_fan_speed_en_pulse <= '1';
                o_target_fan_speed_val <= i_get_kinematics_fan_speed;
                o_target_fan_speed_duration_ms <= 400;
                s_delay_start_ms <= i_get_clock_ticks;
                s_delay_duration_ms <= 500;
                next_state <= STATE_DELAY_ACTIVE;

            when STATE_DELAY_ACTIVE =>
                if s_delay_done_pulse = '1' then
                    if s_race_mode = '1' then
                        next_state <= STATE_RUN_INIT_BUILD_SEQ;
                    else
                        next_state <= STATE_EXPLORE_INIT_MAZE;
                    end if;
                end if;

            -- ===============================================================
            -- Explore Mode
            -- ===============================================================
            when STATE_EXPLORE_INIT_MAZE =>
                -- Initialize maze (all cells to 0, borders to wall bits)
                -- This should be handled by Maze_Memory_Interface / Maze_Logic_Controller
                -- For simplicity here, we assume maze is cleared or set.
                -- initialize_maze(); -- Needs a pulse to Maze_Logic_Controller
                s_maze_data <= (others => (others => '0')); -- Clear maze for example
                for i in 0 to MAZE_COLUMNS-1 loop
                    s_maze_data(i) <= s_maze_data(i) or SOUTH_BIT;
                    s_maze_data((MAZE_ROWS - 1) * MAZE_COLUMNS + i) <= s_maze_data((MAZE_ROWS - 1) * MAZE_COLUMNS + i) or NORTH_BIT;
                end loop;
                for i in 0 to MAZE_ROWS-1 loop
                    s_maze_data(i * MAZE_COLUMNS) <= s_maze_data(i * MAZE_COLUMNS) or WEST_BIT;
                    s_maze_data((MAZE_COLUMNS - 1) + i * MAZE_COLUMNS) <= s_maze_data((MAZE_COLUMNS - 1) + i * MAZE_COLUMNS) or EAST_BIT;
                end loop;

                -- set_initial_state() / set_goals_from_maze() / set_goal_as_target()
                -- These would be handled by Queue/Stack modules and Maze_Logic_Controller
                -- Assuming goals are known and target_cells is set for start.
                s_target_cells.size <= 1;
                s_target_cells.stack(0) <= 0; -- Target cell 0 (start)

                -- update_walls(get_walls()) -- Requires interaction with Maze_Logic_Controller and sensor input
                s_maze_logic_update_en <= '1'; -- Pulse to update maze with initial walls

                s_start_ms <= i_get_clock_ticks; -- Capture start time
                s_o_move_en_pulse <= '1';
                o_move_type <= MOVE_START;
                next_state <= STATE_EXPLORE_MOVE_TO_TARGET; -- Move and then loop

            when STATE_EXPLORE_MAIN_LOOP =>
                if i_is_race_started = '0' then -- Check global race status (can be stopped externally)
                    next_state <= STATE_EXPLORE_FINISHED;
                elsif s_time_limit > 0 and (i_get_clock_ticks - s_start_ms) >= s_time_limit then
                    next_state <= STATE_ERROR_STOP; -- Time limit exceeded
                elsif i_loop_en = '1' then -- Proceed with exploration loop
                    -- go_to_target()
                    s_floodfill_core_calc_en <= '1'; -- Trigger Floodfill calculation
                    next_state <= STATE_EXPLORE_MOVE_TO_TARGET;
                else
                    next_state <= STATE_EXPLORE_MAIN_LOOP; -- Wait for loop_en
                end if;

            when STATE_EXPLORE_MOVE_TO_TARGET =>
                if s_floodfill_core_calc_done = '1' then -- Floodfill calculation done
                    -- Check if current cell is visited. If not, update walls.
                    -- This check needs a dedicated state or sub-module interaction.
                    -- Assuming logic: if not s_maze_logic_is_visited then update_walls(i_walls_status)
                    if s_maze_logic_is_visited = '0' then
                        s_maze_logic_update_en <= '1'; -- Update maze with current walls
                        s_floodfill_core_calc_en <= '1'; -- Recalculate floodfill after wall update
                        next_state <= STATE_EXPLORE_MOVE_TO_TARGET; -- Wait for next calc done
                    else
                        -- Get next step from floodfill
                        s_o_set_rgb_color_while_en_pulse <= '1'; -- set_RGB_color_while(255, 255, 0, 33)
                        o_rgb_r_while_val <= 255; o_rgb_g_while_val <= 255; o_rgb_b_while_val <= 0; o_rgb_while_duration_ms <= 33;
                        s_o_move_en_pulse <= '1';
                        case s_floodfill_core_next_step is
                            when FRONT => o_move_type <= MOVE_FRONT;
                            when LEFT  => o_move_type <= MOVE_LEFT;
                            when RIGHT => o_move_type <= MOVE_RIGHT;
                            when BACK  =>
                                -- if walls.front then move(MOVE_BACK_WALL) else move(MOVE_BACK)
                                -- This logic would need a wall status check.
                                if i_walls_status.front = '1' then
                                    o_move_type <= MOVE_BACK_WALL;
                                else
                                    o_move_type <= MOVE_BACK;
                                end if;
                            when others => -- Error or default stop
                                next_state <= STATE_ERROR_STOP;
                        end case;

                        -- update_position(next_step) -- Update current_position and current_direction internally (or via module)
                        -- This would be handled by the top-level FSM or a small dedicated module
                        -- For simplicity, let's assume current_position and current_direction are updated by odometry/IMU and provided as inputs
                        -- So, we just transition to next state if move is done.
                        if s_floodfill_data(i_current_position) = to_fixed_signed(0) then -- Reached target (value 0)
                            if s_maze_logic_is_goal = '1' then -- Check if current cell is a goal
                                next_state <= STATE_EXPLORE_GOAL_REACHED_FRONT;
                            else
                                next_state <= STATE_EXPLORE_MAIN_LOOP; -- Target reached, find next interesting cell
                            end if;
                        else
                             next_state <= STATE_EXPLORE_MAIN_LOOP; -- Continue moving in the loop
                        end if;
                    end if;
                end if;

            when STATE_EXPLORE_GOAL_REACHED_FRONT =>
                s_o_move_en_pulse <= '1'; o_move_type <= MOVE_FRONT;
                s_maze_logic_update_en <= '1'; -- Update walls after move
                next_state <= STATE_EXPLORE_GOAL_REACHED_BACK;

            when STATE_EXPLORE_GOAL_REACHED_BACK =>
                s_o_move_en_pulse <= '1'; o_move_type <= MOVE_BACK_STOP;
                s_o_eeprom_set_data_en_pulse <= '1';
                o_eeprom_data_index <= DATA_INDEX_MAZE;
                o_eeprom_data_array <= s_maze_data; -- Pass the entire maze array
                o_eeprom_data_length <= MAZE_CELLS;
                s_o_eeprom_save_en_pulse <= '1';
                next_state <= STATE_EXPLORE_FINISHED;

            when STATE_EXPLORE_FINISHED =>
                -- Finalization for exploration
                s_o_set_target_linear_speed_en_pulse <= '1'; o_target_linear_speed_val <= 0;
                s_o_set_ideal_angular_speed_en_pulse <= '1'; o_ideal_angular_speed_val <= (others => '0');
                s_o_set_target_fan_speed_en_pulse <= '1'; o_target_fan_speed_val <= 0; o_target_fan_speed_duration_ms <= 400;
                -- Warning LED for 2s
                s_o_warning_status_led_en_pulse <= '1'; o_warning_status_led_freq <= 50;
                s_delay_start_ms <= i_get_clock_ticks; s_delay_duration_ms <= 2000;
                s_o_set_rgb_color_en_pulse <= '1'; o_rgb_r_val <= 255; o_rgb_g_val <= 255; o_rgb_b_val <= 0;
                s_o_set_status_led_en_pulse <= '1'; o_status_led_val <= '0';
                s_o_set_race_started_en_pulse <= '1'; o_race_started_val <= '0';
                next_state <= STATE_IDLE; -- Go back to idle after finishing

            -- ===============================================================
            -- Run Mode
            -- ===============================================================
            when STATE_RUN_INIT_BUILD_SEQ =>
                s_run_seq_gen_build_en <= '1'; -- Trigger sequence building
                next_state <= STATE_RUN_MAIN_LOOP; -- Proceed to execute loop

            when STATE_RUN_MAIN_LOOP =>
                if i_is_race_started = '0' then
                    next_state <= STATE_RUN_FINISHED; -- Race stopped
                elsif s_run_sequence_idx < (MAZE_CELLS + 3) and s_run_sequence_movements(s_run_sequence_idx) /= MOVE_NONE then
                    s_o_move_en_pulse <= '1';
                    o_move_type <= s_run_sequence_movements(s_run_sequence_idx);
                    s_run_sequence_idx <= s_run_sequence_idx + 1;
                    next_state <= STATE_RUN_MAIN_LOOP; -- Continue executing sequence
                else
                    next_state <= STATE_RUN_FINISHED; -- Sequence finished
                end if;

            when STATE_RUN_FINISHED =>
                s_o_set_target_linear_speed_en_pulse <= '1'; o_target_linear_speed_val <= 0;
                s_o_set_ideal_angular_speed_en_pulse <= '1'; o_ideal_angular_speed_val <= (others => '0');
                s_o_set_target_fan_speed_en_pulse <= '1'; o_target_fan_speed_val <= 0; o_target_fan_speed_duration_ms <= 400;
                s_o_set_rgb_color_while_en_pulse <= '1'; o_rgb_r_while_val <= 255; o_rgb_g_while_val <= 0; o_rgb_b_while_val <= 0; o_rgb_while_duration_ms <= 33;
                s_delay_start_ms <= i_get_clock_ticks; s_delay_duration_ms <= 1000; -- 1s warning
                s_o_set_status_led_en_pulse <= '1'; o_status_led_val <= '0';
                s_o_set_race_started_en_pulse <= '1'; o_race_started_val <= '0';
                next_state <= STATE_IDLE;

            when STATE_ERROR_STOP =>
                s_o_set_target_linear_speed_en_pulse <= '1'; o_target_linear_speed_val <= 0;
                s_o_set_ideal_angular_speed_en_pulse <= '1'; o_ideal_angular_speed_val <= (others => '0');
                s_o_warning_status_led_en_pulse <= '1'; o_warning_status_led_freq <= 50; -- Constant warning
                -- This state loops indefinitely until reset or external intervention
                next_state <= STATE_ERROR_STOP;

            when others =>
                next_state <= STATE_IDLE;
        end case;
    end process;


    -- =========================================================================
    -- ASIGNACIONES DE SALIDA (Combinacional)
    -- =========================================================================
    o_configure_kinematics_en   <= s_o_configure_kinematics_en_pulse;
    o_clear_info_leds_en        <= s_o_clear_info_leds_en_pulse;
    o_set_rgb_color_en          <= s_o_set_rgb_color_en_pulse;
    o_set_rgb_color_while_en    <= s_o_set_rgb_color_while_en_pulse;
    o_set_target_fan_speed_en   <= s_o_set_target_fan_speed_en_pulse;
    o_move_en                   <= s_o_move_en_pulse;
    o_set_race_started_en       <= s_o_set_race_started_en_pulse;
    o_set_target_linear_speed_en <= s_o_set_target_linear_speed_en_pulse;
    o_set_ideal_angular_speed_en <= s_o_set_ideal_angular_speed_en_pulse;
    o_warning_status_led_en     <= s_o_warning_status_led_en_pulse;
    o_set_status_led_en         <= s_o_set_status_led_en_pulse;
    o_eeprom_set_data_en        <= s_o_eeprom_set_data_en_pulse;
    o_eeprom_save_en            <= s_o_eeprom_save_en_pulse;


    -- =========================================================================
    -- INSTANCIACIÓN DE SUB-MÓDULOS (EJEMPLO)
    -- =========================================================================

    -- Componente para la gestión de memoria del laberinto (s_maze_data, s_floodfill_data)
    -- Este módulo se encargaría de leer/escribir en las BRAMs o RAMs distribuidas.
    -- Instancia: maze_memory_interface_inst : entity work.maze_memory_interface
    -- port map (
    --     i_clk => i_clk,
    --     i_reset => i_reset,
    --     i_read_addr_maze => ...,
    --     o_read_data_maze => s_maze_data_read,
    --     i_write_en_maze => ...,
    --     i_write_addr_maze => ...,
    --     i_write_data_maze => ...,
    --     -- Similar para floodfill_data
    --     o_maze_array => s_maze_data,          -- Podría ser una salida o acceder directamente
    --     o_floodfill_array => s_floodfill_data -- Podría ser una salida o acceder directamente
    -- );

    -- Componente para la lógica de pared y celda (wall_exists, visited, goal)
    -- Instancia: maze_logic_inst : entity work.maze_logic_controller
    -- port map (
    --     i_clk => i_clk,
    --     i_reset => i_reset,
    --     i_current_position => i_current_position,
    --     i_current_direction => i_current_direction,
    --     i_walls_status => i_walls_status,
    --     i_maze_data => s_maze_data, -- Acceso a la memoria del laberinto
    --     i_goal_cells => s_goal_cells,
    --     o_wall_exists_front => ..., o_wall_exists_left => ..., o_wall_exists_right => ...,
    --     o_current_cell_is_visited => s_maze_logic_is_visited,
    --     o_current_cell_is_goal => s_maze_logic_is_goal,
    --     o_update_maze_en => s_maze_logic_update_en, -- Pulso para actualizar paredes
    --     o_updated_maze_data => ... -- Escribir de vuelta a maze_memory_interface
    -- );

    -- Componente principal de cálculo del Floodfill (update_floodfill)
    -- Este módulo contendría la lógica principal de la cola, y utilizaría floodfill_weights.vhd
    -- Instancia: floodfill_core_inst : entity work.floodfill_core_algorithm
    -- port map (
    --     i_clk => i_clk,
    --     i_reset => i_reset,
    --     i_calc_en => s_floodfill_core_calc_en,
    --     i_maze_data => s_maze_data, -- Acceso a la memoria del laberinto
    --     i_floodfill_data_in => s_floodfill_data, -- Entrada de la tabla de Floodfill
    --     o_floodfill_data_out => s_floodfill_data, -- Salida de la tabla de Floodfill (para escribir de vuelta)
    --     i_target_cells => s_target_cells,
    --     i_straight_weights => s_straight_weights, -- Desde floodfill_weights_inst
    --     i_straight_weights_count => s_straight_weights_count,
    --     i_diagonal_weights => s_diagonal_weights, -- Desde floodfill_weights_inst
    --     i_diagonal_weights_count => s_diagonal_weights_count,
    --     o_calc_done => s_floodfill_core_calc_done,
    --     o_next_step => s_floodfill_core_next_step
    -- );

    -- Componente para el cálculo de pesos (floodfill_weights.vhd)
    -- Este módulo se encargaría de calcular las tablas de pesos straight_weights y diagonal_weights.
    -- Instancia: floodfill_weights_inst : entity work.floodfill_weights
    -- port map (
    --     i_clk => i_clk,
    --     i_reset => i_reset,
    --     i_calc_en => '1', -- Podría calcularse una vez al inicio o bajo demanda
    --     i_distance => to_fixed(180), -- Ejemplo, 180.0f
    --     i_init_speed => ...,
    --     i_max_speed => ...,
    --     i_accel => ...,
    --     i_cells_to_max_speed => ...,
    --     o_weights_out => s_straight_weights,
    --     o_calc_done => open -- No es crítico si se calcula una vez
    -- );
    -- Similar para diagonal_weights.

    -- Componente para la generación de la secuencia de carrera (build_run_sequence, smooth_run_sequence)
    -- Instancia: run_sequence_generator_inst : entity work.run_sequence_generator
    -- port map (
    --     i_clk => i_clk,
    --     i_reset => i_reset,
    --     i_build_en => s_run_seq_gen_build_en,
    --     i_smooth_en => s_run_seq_gen_smooth_en,
    --     i_maze_data => s_maze_data,
    --     i_floodfill_data => s_floodfill_data,
    --     i_goal_cells => s_goal_cells,
    --     i_speed_strategy => i_menu_run_speed_strategy,
    --     o_run_sequence_movements => s_run_sequence_movements,
    --     o_gen_done => s_run_seq_gen_done
    -- );


end architecture Behavioral;
