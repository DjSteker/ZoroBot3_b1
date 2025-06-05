
-- handwall.vhd
-- Este módulo VHDL implementa la lógica de seguimiento de paredes
-- para el robot, traduciendo la funcionalidad del archivo C 'handwall.c'.
-- Gestiona la inicialización de la secuencia, la toma de decisiones
-- basada en los sensores de pared y el control de los actuadores y LEDs.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Incluye el paquete de tipos personalizados donde se definen enumeraciones y records
use work.robot_types_pkg.all;

entity handwall_controller is
    port (
        i_clk                       : in  std_logic;                                -- Reloj del sistema
        i_reset                     : in  std_logic;                                -- Reset síncrono (activo alto)
        i_get_clock_ticks           : in  natural;                                  -- Ticks del reloj en ms

        -- Comandos de configuración
        i_use_left_hand_en          : in  std_logic;                                -- Pulso para handwall_use_left_hand()
        i_use_right_hand_en         : in  std_logic;                                -- Pulso para handwall_use_right_hand()
        i_set_time_limit_en         : in  std_logic;                                -- Pulso para handwall_set_time_limit()
        i_set_time_limit_ms         : in  natural;                                  -- Valor de tiempo límite en ms

        -- Comando para iniciar la secuencia de seguimiento de pared
        i_handwall_start_en         : in  std_logic;                                -- Pulso para handwall_start()
        i_handwall_loop_en          : in  std_logic;                                -- Pulso periódico para handwall_loop()

        -- Entradas de estado de otros módulos
        i_walls_status              : in  walls_status;                             -- Estado de los sensores de pared (front, left, right)
        i_menu_run_speed_strategy   : in  speed_strategy;                           -- Estrategia de velocidad actual del menú (para configure_kinematics)

        -- Salidas para controlar otros módulos
        o_configure_kinematics_en   : out std_logic;                                -- Pulso para configure_kinematics()
        o_configure_kinematics_speed: out speed_strategy;                           -- Velocidad a configurar
        o_clear_info_leds_en        : out std_logic;                                -- Pulso para clear_info_leds()
        o_set_rgb_color_en          : out std_logic;                                -- Pulso para set_RGB_color()
        o_rgb_r_val                 : out natural range 0 to 255;                   -- Valor R para set_RGB_color()
        o_rgb_g_val                 : out natural range 0 to 255;                   -- Valor G para set_RGB_color()
        o_rgb_b_val                 : out natural range 0 to 255;                   -- Valor B para set_RGB_color()
        o_set_rgb_color_while_en    : out std_logic;                                -- Pulso para set_RGB_color_while()
        o_rgb_r_while_val           : out natural range 0 to 255;                   -- Valor R para set_RGB_color_while()
        o_rgb_g_while_val           : out natural range 0 to 255;                   -- Valor G para set_RGB_color_while()
        o_rgb_b_while_val           : out natural range 0 to 255;                   -- Valor B para set_RGB_color_while()
        o_rgb_while_duration_ms     : out natural;                                  -- Duración en ms para set_RGB_color_while()
        o_set_target_fan_speed_en   : out std_logic;                                -- Pulso para set_target_fan_speed()
        o_target_fan_speed_val      : out natural range 0 to 100;                   -- Velocidad del ventilador
        o_target_fan_speed_duration_ms : out natural;                               -- Duración de la rampa del ventilador
        o_move_en                   : out std_logic;                                -- Pulso para move()
        o_move_type                 : out movement;                                 -- Tipo de movimiento para move()
        o_set_race_started_en       : out std_logic;                                -- Pulso para set_race_started()
        o_race_started_val          : out std_logic;                                -- Valor para set_race_started()
        o_set_target_linear_speed_en : out std_logic;                               -- Pulso para set_target_linear_speed()
        o_target_linear_speed_val   : out natural;                                  -- Velocidad lineal a configurar
        o_set_ideal_angular_speed_en : out std_logic;                               -- Pulso para set_ideal_angular_speed()
        o_ideal_angular_speed_val   : out signed(31 downto 0);                      -- Velocidad angular a configurar (escalado)
        o_warning_status_led_en     : out std_logic;                                -- Pulso para warning_status_led()
        o_warning_status_led_freq   : out natural                                   -- Frecuencia para warning_status_led()
    );
end entity handwall_controller;

architecture Behavioral of handwall_controller is

    -- =========================================================================
    -- ESTADOS DE LA MÁQUINA DE ESTADOS PRINCIPAL
    -- =========================================================================
    type handwall_fsm_state_type is (
        STATE_IDLE,                 -- Esperando un comando de inicio
        STATE_START_INIT,           -- handwall_start: configure_kinematics, clear_leds, set_RGB
        STATE_START_SET_FAN_SPEED,  -- handwall_start: set_target_fan_speed
        STATE_START_DELAY,          -- handwall_start: delay(500)
        STATE_START_MOVE_START,     -- handwall_start: move(MOVE_START)
        STATE_LOOP_CHECK_TIME,      -- handwall_loop: Check time limit
        STATE_LOOP_SET_RGB_WHILE,   -- handwall_loop: set_RGB_color_while
        STATE_LOOP_DECIDE_MOVE,     -- handwall_loop: Logic for wall following
        STATE_LOOP_IDLE_MOTION      -- handwall_loop: Default motion (stop/warning)
    );
    signal current_state : handwall_fsm_state_type := STATE_IDLE;
    signal next_state    : handwall_fsm_state_type;

    -- =========================================================================
    -- SEÑALES INTERNAS (equivalentes a variables 'static' en C)
    -- =========================================================================
    signal s_use_left_hand : std_logic := '1'; -- True (1) por defecto
    signal s_start_ms      : natural   := 0;
    signal s_time_limit    : natural   := 0;

    -- Variables para delays (reemplaza delay())
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

begin

    -- =========================================================================
    -- PROCESO PRINCIPAL DE LA MÁQUINA DE ESTADOS
    -- =========================================================================
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                current_state <= STATE_IDLE;
                s_use_left_hand <= '1';
                s_start_ms      <= 0;
                s_time_limit    <= 0;
                s_delay_start_ms <= 0;
                s_delay_duration_ms <= 0;
                s_delay_done_pulse <= '0';

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
                o_race_started_val                  <= '0'; -- Default to not started
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


                -- Lógica para handwall_use_left_hand() y handwall_use_right_hand()
                if i_use_left_hand_en = '1' then
                    s_use_left_hand <= '1';
                elsif i_use_right_hand_en = '1' then
                    s_use_left_hand <= '0';
                end if;

                -- Lógica para handwall_set_time_limit()
                if i_set_time_limit_en = '1' then
                    s_time_limit <= i_set_time_limit_ms;
                end if;

                -- Lógica de delay no bloqueante
                if s_delay_duration_ms /= 0 and (i_get_clock_ticks - s_delay_start_ms) >= s_delay_duration_ms then
                    s_delay_duration_ms <= 0; -- Reinicia la duración, indicando que el delay ha terminado
                    s_delay_done_pulse <= '1'; -- Pulso para indicar que el delay está hecho
                else
                    s_delay_done_pulse <= '0';
                end if;

            end if;
        end if;
    end process;


    -- =========================================================================
    -- LÓGICA DE TRANSICIÓN DE ESTADOS Y ACCIONES COMBINACIONALES
    -- =========================================================================
    process (current_state, i_handwall_start_en, i_handwall_loop_en, i_get_clock_ticks, s_start_ms,
             s_time_limit, i_menu_run_speed_strategy, s_delay_done_pulse, i_walls_status,
             s_use_left_hand)
    begin
        next_state <= current_state;

        -- Default values for non-pulse outputs (will be overwritten if pulse is '1')
        o_configure_kinematics_speed <= SPEED_EXPLORE; -- Default or dummy value
        o_rgb_r_val <= 0; o_rgb_g_val <= 0; o_rgb_b_val <= 0;
        o_rgb_r_while_val <= 0; o_rgb_g_while_val <= 0; o_rgb_b_while_val <= 0; o_rgb_while_duration_ms <= 0;
        o_target_fan_speed_val <= 0; o_target_fan_speed_duration_ms <= 0;
        o_move_type <= MOVE_NONE;
        o_race_started_val <= '0'; -- Explicitly set to '0' by default
        o_target_linear_speed_val <= 0;
        o_ideal_angular_speed_val <= (others => '0');
        o_warning_status_led_freq <= 0;

        case current_state is
            when STATE_IDLE =>
                if i_handwall_start_en = '1' then
                    s_start_ms <= i_get_clock_ticks; -- capture start time
                    next_state <= STATE_START_INIT;
                elsif i_handwall_loop_en = '1' then -- Allows for external triggering of loop without full start
                    next_state <= STATE_LOOP_CHECK_TIME;
                end if;

            -- ===============================================================
            -- handwall_start() sequence
            -- ===============================================================
            when STATE_START_INIT =>
                s_o_configure_kinematics_en_pulse <= '1';
                o_configure_kinematics_speed <= i_menu_run_speed_strategy;
                s_o_clear_info_leds_en_pulse <= '1';
                s_o_set_rgb_color_en_pulse <= '1';
                o_rgb_r_val <= 0; o_rgb_g_val <= 0; o_rgb_b_val <= 0;
                next_state <= STATE_START_SET_FAN_SPEED;

            when STATE_START_SET_FAN_SPEED =>
                -- fan_speed from C_KINEMATICS_SETTINGS for the current speed_strategy
                -- Need a way to read C_KINEMATICS_SETTINGS (e.g., from a separate ROM or a function)
                -- For simplicity, let's assume get_kinematics().fan_speed is available here
                -- or that C_KINEMATICS_SETTINGS is accessible by index.
                s_o_set_target_fan_speed_en_pulse <= '1';
                o_target_fan_speed_val <= C_KINEMATICS_SETTINGS(i_menu_run_speed_strategy).fan_speed;
                o_target_fan_speed_duration_ms <= 400;
                s_delay_start_ms <= i_get_clock_ticks;
                s_delay_duration_ms <= 500; -- delay(500)
                next_state <= STATE_START_DELAY;

            when STATE_START_DELAY =>
                if s_delay_done_pulse = '1' then
                    next_state <= STATE_START_MOVE_START;
                end if;

            when STATE_START_MOVE_START =>
                s_o_move_en_pulse <= '1';
                o_move_type <= MOVE_START;
                next_state <= STATE_IDLE; -- Return to idle, loop will be triggered externally

            -- ===============================================================
            -- handwall_loop() sequence (triggered by i_handwall_loop_en)
            -- ===============================================================
            when STATE_LOOP_CHECK_TIME =>
                -- Check time limit
                if s_time_limit > 0 and (i_get_clock_ticks - s_start_ms) >= s_time_limit then
                    s_o_set_race_started_en_pulse <= '1';
                    o_race_started_val <= '0'; -- set_race_started(false)
                    next_state <= STATE_IDLE; -- Exit loop
                else
                    next_state <= STATE_LOOP_SET_RGB_WHILE;
                end if;

            when STATE_LOOP_SET_RGB_WHILE =>
                s_o_set_rgb_color_while_en_pulse <= '1';
                o_rgb_r_while_val <= 255; o_rgb_g_while_val <= 255; o_rgb_b_while_val <= 0;
                o_rgb_while_duration_ms <= 20;
                next_state <= STATE_LOOP_DECIDE_MOVE;

            when STATE_LOOP_DECIDE_MOVE =>
                -- Logic for wall following (if/else if cascade)
                if (s_use_left_hand = '1' and i_walls_status.left = '0') or
                   (s_use_left_hand = '0' and i_walls_status.right = '1' and i_walls_status.left = '0') then
                    s_o_move_en_pulse <= '1';
                    o_move_type <= MOVE_LEFT;
                    next_state <= STATE_IDLE; -- Go back to idle, wait for next loop trigger
                elsif (s_use_left_hand = '0' and i_walls_status.right = '0') or
                      (s_use_left_hand = '1' and i_walls_status.left = '1' and i_walls_status.right = '0') then
                    s_o_move_en_pulse <= '1';
                    o_move_type <= MOVE_RIGHT;
                    next_state <= STATE_IDLE;
                elsif i_walls_status.front = '0' then
                    s_o_move_en_pulse <= '1';
                    o_move_type <= MOVE_FRONT;
                    next_state <= STATE_IDLE;
                elsif i_walls_status.front = '1' and i_walls_status.left = '1' and i_walls_status.right = '1' then
                    s_o_move_en_pulse <= '1';
                    o_move_type <= MOVE_BACK_WALL;
                    next_state <= STATE_IDLE;
                else
                    next_state <= STATE_LOOP_IDLE_MOTION;
                end if;

            when STATE_LOOP_IDLE_MOTION =>
                s_o_set_target_linear_speed_en_pulse <= '1';
                o_target_linear_speed_val <= 0;
                s_o_set_ideal_angular_speed_en_pulse <= '1';
                o_ideal_angular_speed_val <= (others => '0'); -- 0 (scaled)
                s_o_warning_status_led_en_pulse <= '1';
                o_warning_status_led_freq <= 50;
                next_state <= STATE_IDLE; -- Return to idle, wait for next loop trigger

            when others =>
                next_state <= STATE_IDLE; -- Safety
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

end architecture Behavioral;
