
-- control.vhd
-- VHDL translation of control.h logic

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.float_pkg.all; -- For floating-point operations

library work;
use work.control_pkg.all;
-- You might also need constants from debug_pkg if you're using it, e.g.:
-- use work.debug_pkg.all;

entity control is
    port (
        i_clk                     : in  std_logic;                                 -- System clock
        i_rst_n                   : in  std_logic;                                 -- Asynchronous active-low reset
        i_get_clock_ticks         : in  unsigned(31 downto 0);                     -- get_clock_ticks() - Millisecond tick counter
        i_control_loop_trigger    : in  std_logic;                                 -- Pulse at CONTROL_FREQUENCY_HZ to run control_loop

        -- Inputs from other modules (replacing C function calls)
        i_get_battery_voltage         : in  float32;                             -- get_battery_voltage()
        i_get_battery_high_limit_voltage: in float32;                            -- get_battery_high_limit_voltage()
        i_kinematics                  : in  t_kinematics;                        -- get_kinematics()
        i_get_encoder_left_speed      : in  float32;                             -- get_encoder_left_speed()
        i_get_encoder_right_speed     : in  float32;                             -- get_encoder_right_speed()
        i_lsm6dsr_get_gyro_z_radps    : in  float32;                             -- lsm6dsr_get_gyro_z_radps()
        i_get_sensor_distance_fl      : in  signed(15 downto 0);                 -- get_sensor_distance(SENSOR_FRONT_LEFT_WALL_ID)
        i_get_sensor_distance_fr      : in  signed(15 downto 0);                 -- get_sensor_distance(SENSOR_FRONT_RIGHT_WALL_ID)
        i_get_side_sensors_error      : in  float32;                             -- get_side_sensors_error()
        i_get_front_sensors_angle_error: in float32;                             -- get_front_sensors_angle_error()
        i_get_front_sensors_diagonal_error: in float32;                          -- get_front_sensors_diagonal_error()
        i_is_debug_enabled            : in  std_logic;                           -- is_debug_enabled()
        i_is_motor_saturated          : in  std_logic;                           -- is_motor_saturated()
        i_get_motors_saturated_ms     : in  unsigned(31 downto 0);               -- get_motors_saturated_ms()
        i_get_encoder_avg_millimeters : in  float32;                             -- get_encoder_avg_millimeters()
        i_get_wall_lost_toggle_state  : in  std_logic;                           -- get_wall_lost_toggle_state()
        i_get_cell_change_toggle_state: in  std_logic;                           -- get_cell_change_toggle_state()

        -- Control Inputs (from higher-level logic)
        i_set_target_linear_speed_en : in  std_logic;                           -- Enable for set_target_linear_speed()
        i_target_linear_speed_val    : in  signed(31 downto 0);                 -- Value for set_target_linear_speed()
        i_set_ideal_angular_speed_en : in  std_logic;                           -- Enable for set_ideal_angular_speed()
        i_ideal_angular_speed_val    : in  float32;                             -- Value for set_ideal_angular_speed()
        i_set_target_fan_speed_en    : in  std_logic;                           -- Enable for set_target_fan_speed()
        i_target_fan_speed_val       : in  signed(31 downto 0);                 -- Value for set_target_fan_speed()
        i_target_fan_speed_ms        : in  unsigned(31 downto 0);               -- ms for set_target_fan_speed()

        i_set_race_started_en        : in  std_logic;                           -- Enable for set_race_started()
        i_set_race_started_val       : in  std_logic;                           -- Value for set_race_started()
        i_set_control_debug_en       : in  std_logic;                           -- Enable for set_control_debug()
        i_set_control_debug_val      : in  std_logic;                           -- Value for set_control_debug()

        i_set_side_sensors_close_correction_en: in std_logic;                   -- Enable for set_side_sensors_close_correction()
        i_set_side_sensors_close_correction_val: in std_logic;                  -- Value for set_side_sensors_close_correction()
        i_set_side_sensors_far_correction_en: in std_logic;                     -- Enable for set_side_sensors_far_correction()
        i_set_side_sensors_far_correction_val: in std_logic;                    -- Value for set_side_sensors_far_correction()
        i_set_front_sensors_correction_en: in std_logic;                        -- Enable for set_front_sensors_correction()
        i_set_front_sensors_correction_val: in std_logic;                       -- Value for set_front_sensors_correction()
        i_set_front_sensors_diagonal_correction_en: in std_logic;               -- Enable for set_front_sensors_diagonal_correction()
        i_set_front_sensors_diagonal_correction_val: in std_logic;              -- Value for set_front_sensors_diagonal_correction()
        i_disable_sensors_correction_trigger: in std_logic;                      -- Trigger for disable_sensors_correction()
        i_reset_control_errors_trigger: in  std_logic;                          -- Trigger for reset_control_errors()
        i_reset_control_speed_trigger: in  std_logic;                           -- Trigger for reset_control_speed()
        i_reset_control_all_trigger  : in  std_logic;                           -- Trigger for reset_control_all()
        i_keep_z_angle_trigger       : in  std_logic;                           -- Trigger for keep_z_angle()


        -- Outputs to other modules (replacing C function calls)
        o_set_motors_pwm_en           : out std_logic;                           -- set_motors_pwm(pwm_left, pwm_right) enable
        o_set_motors_pwm_left         : out signed(15 downto 0);                 -- set_motors_pwm left value
        o_set_motors_pwm_right        : out signed(15 downto 0);                 -- set_motors_pwm right value
        o_set_motors_enable_en        : out std_logic;                           -- set_motors_enable(state) enable
        o_set_motors_enable_val       : out std_logic;                           -- set_motors_enable(state) value
        o_set_motors_brake_trigger    : out std_logic;                           -- set_motors_brake() trigger
        o_set_fan_speed_en            : out std_logic;                           -- set_fan_speed(speed) enable
        o_set_fan_speed_val           : out signed(15 downto 0);                 -- set_fan_speed value (percentage_to_fan_pwm result)

        o_set_RGB_color_en            : out std_logic;                           -- set_RGB_color(r,g,b) enable
        o_set_RGB_color_r             : out unsigned(7 downto 0);
        o_set_RGB_color_g             : out unsigned(7 downto 0);
        o_set_RGB_color_b             : out unsigned(7 downto 0);
        o_blink_RGB_color_trigger     : out std_logic;                           -- blink_RGB_color(r,g,b,ms) trigger
        o_blink_RGB_color_r           : out unsigned(7 downto 0);
        o_blink_RGB_color_g           : out unsigned(7 downto 0);
        o_blink_RGB_color_b           : out unsigned(7 downto 0);
        o_blink_RGB_color_ms          : out unsigned(31 downto 0);

        o_menu_reset_trigger          : out std_logic;                           -- menu_reset() trigger
        o_menu_run_reset_trigger      : out std_logic;                           -- menu_run_reset() trigger
        o_reset_motors_saturated_trigger: out std_logic;                         -- reset_motors_saturated() trigger
        o_reset_encoder_avg_trigger   : out std_logic;                           -- reset_encoder_avg() trigger
        o_gpio_13_toggle              : out std_logic;                           -- For gpio_set/clear (GPIOB, GPIO13)
        o_gpio_15_set                 : out std_logic;                           -- For gpio_set(GPIOB, GPIO15)

        -- Outputs for internal state (read by other modules or for debug)
        o_is_race_started             : out std_logic;                           -- is_race_started()
        o_is_front_sensors_correction_enabled: out std_logic;                    -- is_front_sensors_correction_enabled()
        o_ideal_linear_speed          : out signed(31 downto 0);                 -- get_ideal_linear_speed()
        o_ideal_angular_speed         : out float32;                             -- get_ideal_angular_speed()
        o_check_start_run_result      : out signed(7 downto 0)                   -- check_start_run() return value
    );
end entity control;

architecture rtl of control is

    -- Internal signals (static volatile variables in C)
    signal s_race_started                     : boolean := false;
    signal s_race_finish_ms                   : unsigned(31 downto 0) := (others => '0');
    signal s_sensor_front_left_start_ms       : unsigned(31 downto 0) := (others => '0');
    signal s_sensor_front_right_start_ms      : unsigned(31 downto 0) := (others => '0');
    signal s_control_debug                    : boolean := false;

    signal s_target_linear_speed              : signed(31 downto 0) := (others => '0');
    signal s_ideal_linear_speed               : signed(31 downto 0) := (others => '0');
    signal s_ideal_angular_speed              : float32 := to_float32(0.0);

    signal s_target_fan_speed                 : signed(31 downto 0) := (others => '0');
    signal s_ideal_fan_speed                  : float32 := to_float32(0.0);
    signal s_fan_speed_accel                  : float32 := to_float32(0.0);

    signal s_linear_error                     : float32 := to_float32(0.0);
    signal s_last_linear_error                : float32 := to_float32(0.0);

    signal s_angular_error                    : float32 := to_float32(0.0);
    signal s_last_angular_error               : float32 := to_float32(0.0);

    signal s_side_sensors_close_correction_enabled : boolean := false;
    signal s_side_sensors_far_correction_enabled   : boolean := false;
    signal s_front_sensors_correction_enabled      : boolean := false;
    signal s_front_sensors_diagonal_correction_enabled: boolean := false;

    signal s_side_sensors_error               : float32 := to_float32(0.0);
    signal s_last_side_sensors_error          : float32 := to_float32(0.0);
    signal s_sum_side_sensors_error           : float32 := to_float32(0.0);

    signal s_front_sensors_error              : float32 := to_float32(0.0);
    signal s_sum_front_sensors_error          : float32 := to_float32(0.0);

    signal s_front_sensors_diagonal_error     : float32 := to_float32(0.0);
    signal s_last_front_sensors_diagonal_error: float32 := to_float32(0.0);
    signal s_sum_front_sensors_diagonal_error : float32 := to_float32(0.0);

    signal s_voltage_left                     : float32 := to_float32(0.0);
    signal s_voltage_right                    : float32 := to_float32(0.0);
    signal s_pwm_left                         : signed(15 downto 0) := (others => '0');
    signal s_pwm_right                        : signed(15 downto 0) := (others => '0');

    -- Internal signals for output pulses (to avoid combinatorial loops and manage one-shot triggers)
    signal r_set_motors_pwm_en              : std_logic := '0';
    signal r_set_motors_enable_en           : std_logic := '0';
    signal r_set_motors_enable_val          : std_logic := '0';
    signal r_set_motors_brake_trigger       : std_logic := '0';
    signal r_set_fan_speed_en               : std_logic := '0';
    signal r_set_RGB_color_en               : std_logic := '0';
    signal r_blink_RGB_color_trigger        : std_logic := '0';
    signal r_menu_reset_trigger             : std_logic := '0';
    signal r_menu_run_reset_trigger         : std_logic := '0';
    signal r_reset_motors_saturated_trigger : std_logic := '0';
    signal r_reset_encoder_avg_trigger      : std_logic := '0';
    signal r_reset_control_all_trigger      : std_logic := '0';
    signal r_gpio_13_toggle_reg             : std_logic := '0';
    signal r_gpio_15_set_reg                : std_logic := '0';

    -- FSM for check_start_run() delays
    type t_check_start_run_state is (S_CHECK_START_IDLE, S_CHECK_START_DELAY_RGB_ON, S_CHECK_START_DELAY_RGB_OFF, S_CHECK_START_DONE);
    signal s_check_start_run_state : t_check_start_run_state := S_CHECK_START_IDLE;
    signal s_check_start_run_delay_start_ms : unsigned(31 downto 0) := (others => '0');
    signal s_check_start_run_delay_duration_ms : unsigned(31 downto 0) := (others => '0');
    signal s_check_start_run_delay_done_pulse : std_logic := '0';
    signal s_check_start_run_return_sensor_idx : signed(7 downto 0) := to_signed(-1, 8);


begin

    -- Output assignments
    o_is_race_started                      <= '1' when s_race_started else '0';
    o_is_front_sensors_correction_enabled  <= '1' when s_front_sensors_correction_enabled else '0';
    o_ideal_linear_speed                   <= s_ideal_linear_speed;
    o_ideal_angular_speed                  <= s_ideal_angular_speed;

    o_set_motors_pwm_en                    <= r_set_motors_pwm_en;
    o_set_motors_pwm_left                  <= s_pwm_left;
    o_set_motors_pwm_right                 <= s_pwm_right;
    o_set_motors_enable_en                 <= r_set_motors_enable_en;
    o_set_motors_enable_val                <= r_set_motors_enable_val;
    o_set_motors_brake_trigger             <= r_set_motors_brake_trigger;
    o_set_fan_speed_en                     <= r_set_fan_speed_en;
    o_set_fan_speed_val                    <= signed(float32_to_std_logic_vector(s_ideal_fan_speed)(15 downto 0)); -- Cast to signed (por ajustar)

    o_set_RGB_color_en                     <= r_set_RGB_color_en;
    o_set_RGB_color_r                      <= (others => '0'); -- R=0 for set_RGB_color(0, 50, 0) and (0,0,0)
    o_set_RGB_color_g                      <= to_unsigned(50, 8); -- G=50 for (0,50,0), set to 0 elsewhere by default
    o_set_RGB_color_b                      <= (others => '0'); -- B=0

    o_blink_RGB_color_trigger              <= r_blink_RGB_color_trigger;
    o_blink_RGB_color_r                    <= to_unsigned(512/2, 8); -- Assuming 512 is >255, scale down to 8-bit. Por ajustar.
    o_blink_RGB_color_g                    <= (others => '0');
    o_blink_RGB_color_b                    <= (others => '0');
    o_blink_RGB_color_ms                   <= to_unsigned(50, 32);

    o_menu_reset_trigger                   <= r_menu_reset_trigger;
    o_menu_run_reset_trigger               <= r_menu_run_reset_trigger;
    o_reset_motors_saturated_trigger       <= r_reset_motors_saturated_trigger;
    o_reset_encoder_avg_trigger            <= r_reset_encoder_avg_trigger;
    o_reset_control_all_trigger            <= r_reset_control_all_trigger;
    o_gpio_13_toggle                       <= r_gpio_13_toggle_reg;
    o_gpio_15_set                          <= r_gpio_15_set_reg;

    o_check_start_run_result               <= s_check_start_run_return_sensor_idx;

    --- Simple Delay Logic ---
    -- This process implements a simple non-blocking delay for check_start_run
    -- and similar one-shot delays.
    process (i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            s_check_start_run_delay_done_pulse <= '0';
        elsif rising_edge(i_clk) then
            s_check_start_run_delay_done_pulse <= '0'; -- Clear pulse each cycle

            if s_check_start_run_delay_start_ms /= (others => '0') then -- Check if delay is 'active' (not reset to 0)
                -- Handle normal operation and overflow
                if i_get_clock_ticks >= s_check_start_run_delay_start_ms then
                    if (i_get_clock_ticks - s_check_start_run_delay_start_ms) >= s_check_start_run_delay_duration_ms then
                        s_check_start_run_delay_done_pulse <= '1';
                        s_check_start_run_delay_start_ms   <= (others => '0'); -- Mark delay as finished
                    end if;
                else -- Overflow condition: i_get_clock_ticks < s_check_start_run_delay_start_ms
                    if (to_unsigned(2**32-1, 32) - s_check_start_run_delay_start_ms) + i_get_clock_ticks >= s_check_start_run_delay_duration_ms then
                        s_check_start_run_delay_done_pulse <= '1';
                        s_check_start_run_delay_start_ms   <= (others => '0');
                    end if;
                end if;
            end if;
        end if;
    end process;


    --- Core Control Logic FSM ---
    process (i_clk, i_rst_n)
        -- Local signals for operations (to handle float conversions or temporary values)
        -- No need for local float vars here, as float_pkg allows direct operations on signals.
    begin
        if i_rst_n = '0' then
            -- Reset all internal state variables
            s_race_started                     <= false;
            s_race_finish_ms                   <= (others => '0');
            s_sensor_front_left_start_ms       <= (others => '0');
            s_sensor_front_right_start_ms      <= (others => '0');
            s_control_debug                    <= false;
            s_target_linear_speed              <= (others => '0');
            s_ideal_linear_speed               <= (others => '0');
            s_ideal_angular_speed              <= to_float32(0.0);
            s_target_fan_speed                 <= (others => '0');
            s_ideal_fan_speed                  <= to_float32(0.0);
            s_fan_speed_accel                  <= to_float32(0.0);
            s_linear_error                     <= to_float32(0.0);
            s_last_linear_error                <= to_float32(0.0);
            s_angular_error                    <= to_float32(0.0);
            s_last_angular_error               <= to_float32(0.0);
            s_side_sensors_close_correction_enabled <= false;
            s_side_sensors_far_correction_enabled   <= false;
            s_front_sensors_correction_enabled      <= false;
            s_front_sensors_diagonal_correction_enabled <= false;
            s_side_sensors_error               <= to_float32(0.0);
            s_last_side_sensors_error          <= to_float32(0.0);
            s_sum_side_sensors_error           <= to_float32(0.0);
            s_front_sensors_error              <= to_float32(0.0);
            s_sum_front_sensors_error          <= to_float32(0.0);
            s_front_sensors_diagonal_error     <= to_float32(0.0);
            s_last_front_sensors_diagonal_error <= to_float32(0.0);
            s_sum_front_sensors_diagonal_error <= to_float32(0.0);
            s_voltage_left                     <= to_float32(0.0);
            s_voltage_right                    <= to_float32(0.0);
            s_pwm_left                         <= (others => '0');
            s_pwm_right                        <= (others => '0');

            -- Reset all output triggers/enables
            r_set_motors_pwm_en              <= '0';
            r_set_motors_enable_en           <= '0';
            r_set_motors_enable_val          <= '0';
            r_set_motors_brake_trigger       <= '0';
            r_set_fan_speed_en               <= '0';
            r_set_RGB_color_en               <= '0';
            r_blink_RGB_color_trigger        <= '0';
            r_menu_reset_trigger             <= '0';
            r_menu_run_reset_trigger         <= '0';
            r_reset_motors_saturated_trigger <= '0';
            r_reset_encoder_avg_trigger      <= '0';
            r_reset_control_all_trigger      <= '0';
            r_gpio_13_toggle_reg             <= '0';
            r_gpio_15_set_reg                <= '0';

        elsif rising_edge(i_clk) then
            -- Clear one-shot pulses by default for each cycle
            r_set_motors_pwm_en              <= '0';
            r_set_motors_enable_en           <= '0';
            r_set_motors_brake_trigger       <= '0';
            r_set_fan_speed_en               <= '0';
            r_set_RGB_color_en               <= '0'; -- Clear unless explicitly set
            r_blink_RGB_color_trigger        <= '0';
            r_menu_reset_trigger             <= '0';
            r_menu_run_reset_trigger         <= '0';
            r_reset_motors_saturated_trigger <= '0';
            r_reset_encoder_avg_trigger      <= '0';
            r_reset_control_all_trigger      <= '0';
            r_gpio_13_toggle_reg             <= '0'; -- Clear toggle if it was set
            r_gpio_15_set_reg                <= '0'; -- Clear if it was set

            -- --- External Set/Reset Function Calls ---
            -- set_target_linear_speed()
            if i_set_target_linear_speed_en = '1' then
                s_target_linear_speed <= i_target_linear_speed_val;
            end if;
            -- set_ideal_angular_speed()
            if i_set_ideal_angular_speed_en = '1' then
                s_ideal_angular_speed <= i_ideal_angular_speed_val;
            end if;
            -- set_target_fan_speed()
            if i_set_target_fan_speed_en = '1' then
                s_target_fan_speed <= i_target_fan_speed_val;
                -- C code: fan_speed_accel = (fan_speed - ideal_fan_speed) * CONTROL_FREQUENCY_HZ / ms;
                -- Por ajustar: ensure division by zero isn't an issue for ms=0
                -- Por hacer: Implement safe division for ms=0
                if i_target_fan_speed_ms /= to_unsigned(0, 32) then
                    s_fan_speed_accel <= (to_float32(s_target_fan_speed) - s_ideal_fan_speed) * to_float32(CONTROL_FREQUENCY_HZ) / to_float32(i_target_fan_speed_ms);
                else
                    s_fan_speed_accel <= to_float32(0.0); -- No acceleration if ms is 0
                end if;
            end if;
            -- set_race_started()
            if i_set_race_started_en = '1' then
                if i_set_race_started_val = '1' then -- set_race_started(true)
                    r_reset_control_all_trigger <= '1'; -- reset_control_all();
                    s_race_started <= true;
                else -- set_race_started(false)
                    r_reset_control_all_trigger <= '1'; -- reset_control_all();
                    s_race_started <= false;
                    r_menu_reset_trigger <= '1'; -- menu_reset();
                    s_race_finish_ms <= i_get_clock_ticks;
                end if;
            end if;
            -- set_control_debug()
            if i_set_control_debug_en = '1' then
                s_control_debug <= (i_set_control_debug_val = '1');
            end if;

            -- Sensor correction enable/disable
            if i_set_side_sensors_close_correction_en = '1' then
                s_side_sensors_close_correction_enabled <= (i_set_side_sensors_close_correction_val = '1');
            end if;
            if i_set_side_sensors_far_correction_en = '1' then
                s_side_sensors_far_correction_enabled <= (i_set_side_sensors_far_correction_val = '1');
            end if;
            if i_set_front_sensors_correction_en = '1' then
                s_front_sensors_correction_enabled <= (i_set_front_sensors_correction_val = '1');
            end if;
            if i_set_front_sensors_diagonal_correction_en = '1' then
                s_front_sensors_diagonal_correction_enabled <= (i_set_front_sensors_diagonal_correction_val = '1');
            end if;
            -- disable_sensors_correction()
            if i_disable_sensors_correction_trigger = '1' then
                s_front_sensors_correction_enabled <= false;
                s_front_sensors_diagonal_correction_enabled <= false;
                s_side_sensors_close_correction_enabled <= false;
                s_side_sensors_far_correction_enabled <= false;
            end if;

            -- reset_control_errors()
            if i_reset_control_errors_trigger = '1' then
                s_sum_side_sensors_error <= to_float32(0.0);
                s_last_side_sensors_error <= to_float32(0.0);
                s_sum_front_sensors_error <= to_float32(0.0);
                s_sum_front_sensors_diagonal_error <= to_float32(0.0);
                s_linear_error <= to_float32(0.0);
                s_angular_error <= to_float32(0.0);
                s_last_linear_error <= to_float32(0.0);
                s_last_angular_error <= to_float32(0.0);
            end if;
            -- reset_control_speed()
            if i_reset_control_speed_trigger = '1' then
                s_target_linear_speed <= (others => '0');
                s_ideal_linear_speed <= (others => '0');
                s_ideal_angular_speed <= to_float32(0.0);
                s_voltage_left <= to_float32(0.0);
                s_voltage_right <= to_float32(0.0);
                s_pwm_left <= (others => '0');
                s_pwm_right <= (others => '0');
            end if;
            -- reset_control_all()
            if i_reset_control_all_trigger = '1' then
                -- This will trigger the above two resets too, if chained appropriately
                r_reset_control_errors_trigger <= '1'; -- This is an immediate trigger
                r_reset_control_speed_trigger <= '1';  -- This is an immediate trigger
                r_reset_motors_saturated_trigger <= '1';
                r_reset_encoder_avg_trigger <= '1';
            end if;


            -- --- check_start_run() FSM ---
            case s_check_start_run_state is
                when S_CHECK_START_IDLE =>
                    s_check_start_run_return_sensor_idx <= to_signed(-1, 8); -- Default return value
                    -- check_start_run logic for setting start_ms
                    if i_get_sensor_distance_fl <= SENSOR_FRONT_DETECTION_START then
                        if s_sensor_front_left_start_ms = 0 and s_sensor_front_right_start_ms = 0 then
                            s_sensor_front_left_start_ms <= i_get_clock_ticks;
                        end if;
                    else
                        s_sensor_front_left_start_ms <= (others => '0');
                    end if;

                    if i_get_sensor_distance_fr <= SENSOR_FRONT_DETECTION_START then
                        if s_sensor_front_left_start_ms = 0 and s_sensor_front_right_start_ms = 0 then
                            s_sensor_front_right_start_ms <= i_get_clock_ticks;
                        end if;
                    else
                        s_sensor_front_right_start_ms <= (others => '0');
                    end if;

                    if (s_sensor_front_left_start_ms /= (others => '0') and (i_get_clock_ticks - s_sensor_front_left_start_ms) >= SENSOR_START_MIN_MS) or
                       (s_sensor_front_right_start_ms /= (others => '0') and (i_get_clock_ticks - s_sensor_front_right_start_ms) >= SENSOR_START_MIN_MS) then
                        s_check_start_run_state <= S_CHECK_START_DELAY_RGB_ON;
                        r_set_RGB_color_en <= '1';
                        o_set_RGB_color_r <= (others => '0');
                        o_set_RGB_color_g <= to_unsigned(50, 8);
                        o_set_RGB_color_b <= (others => '0');
                        s_check_start_run_delay_start_ms <= i_get_clock_ticks;
                        s_check_start_run_delay_duration_ms <= to_unsigned(1000, 32); -- delay(1000)
                    end if;

                when S_CHECK_START_DELAY_RGB_ON =>
                    if s_check_start_run_delay_done_pulse = '1' then
                        r_set_RGB_color_en <= '1';
                        o_set_RGB_color_r <= (others => '0');
                        o_set_RGB_color_g <= (others => '0');
                        o_set_RGB_color_b <= (others => '0');
                        s_check_start_run_state <= S_CHECK_START_DELAY_RGB_OFF;
                        s_check_start_run_delay_start_ms <= i_get_clock_ticks; -- Reset delay for next phase
                        s_check_start_run_delay_duration_ms <= to_unsigned(1, 32); -- Small delay to ensure RGB is off before proceeding
                    end if;

                when S_CHECK_START_DELAY_RGB_OFF =>
                    if s_check_start_run_delay_done_pulse = '1' then
                        r_set_race_started_en <= '1';
                        r_set_race_started_val <= '1'; -- set_race_started(true)

                        if s_sensor_front_left_start_ms /= (others => '0') and (i_get_clock_ticks - s_sensor_front_left_start_ms) >= SENSOR_START_MIN_MS then
                            s_check_start_run_return_sensor_idx <= to_signed(SENSOR_FRONT_LEFT_WALL_ID, 8);
                        else
                            s_check_start_run_return_sensor_idx <= to_signed(SENSOR_FRONT_RIGHT_WALL_ID, 8);
                        end if;
                        s_sensor_front_left_start_ms <= (others => '0');
                        s_sensor_front_right_start_ms <= (others => '0');
                        r_menu_run_reset_trigger <= '1'; -- menu_run_reset();
                        s_check_start_run_state <= S_CHECK_START_DONE; -- Done, will return to IDLE next cycle
                    end if;

                when S_CHECK_START_DONE =>
                    s_check_start_run_state <= S_CHECK_START_IDLE; -- Return to idle state for next check
            end case;


            -- --- control_loop() main logic (triggered by i_control_loop_trigger) ---
            if i_control_loop_trigger = '1' then
                -- C code: gpio_set(GPIOB, GPIO13); delay_us(100); gpio_clear(GPIOB, GPIO13);
                -- Por hacer: Implement precise delay_us (100us) for this GPIO pulse.
                -- For now, just a single cycle pulse on i_clk is a rough approximation.
                r_gpio_13_toggle_reg <= '1'; -- Set for one cycle
                -- Next cycle it will be cleared by default

                if i_is_debug_enabled = '1' then
                    -- C code: return; (implies no further control loop action)
                    null;
                elsif i_is_motor_saturated = '1' and s_race_started = true then
                    r_set_motors_pwm_en <= '1';
                    s_pwm_left <= (others => '0');
                    s_pwm_right <= (others => '0');
                    r_set_fan_speed_en <= '1';
                    o_set_fan_speed_val <= to_signed(0, 16);
                    if i_get_clock_ticks >= i_get_motors_saturated_ms and (i_get_clock_ticks - i_get_motors_saturated_ms) < 3000 then
                        r_blink_RGB_color_trigger <= '1';
                        o_blink_RGB_color_r <= to_unsigned(512/2, 8); -- Por ajustar: 512 is out of 8-bit range, scaled down
                        o_blink_RGB_color_g <= (others => '0');
                        o_blink_RGB_color_b <= (others => '0');
                        o_blink_RGB_color_ms <= to_unsigned(50, 32);
                    else
                        r_set_RGB_color_en <= '1';
                        o_set_RGB_color_r <= (others => '0');
                        o_set_RGB_color_g <= (others => '0');
                        o_set_RGB_color_b <= (others => '0');
                        r_set_race_started_en <= '1';
                        r_set_race_started_val <= '0'; -- set_race_started(false);
                    end if;
                elsif s_race_started = false then
                    if s_race_finish_ms /= (others => '0') and i_get_clock_ticks >= s_race_finish_ms and (i_get_clock_ticks - s_race_finish_ms) <= 3000 then
                        r_set_motors_brake_trigger <= '1';
                    else
                        r_set_motors_pwm_en <= '1';
                        s_pwm_left <= (others => '0');
                        s_pwm_right <= (others => '0');
                        r_set_motors_enable_en <= '1';
                        r_set_motors_enable_val <= '0'; -- false
                    end if;
                    r_set_fan_speed_en <= '1';
                    o_set_fan_speed_val <= to_signed(0, 16);
                else -- s_race_started = true
                    r_set_motors_enable_en <= '1';
                    r_set_motors_enable_val <= '1'; -- true

                    -- update_ideal_linear_speed()
                    -- Por ajustar: Need a proper fixed-point or float representation for kinematics.
                    if s_ideal_linear_speed < s_target_linear_speed then
                        -- int16_t accel = get_kinematics().linear_accel.accel_soft;
                        -- if (get_kinematics().linear_accel.speed_hard == 0 || ideal_linear_speed < get_kinematics().linear_accel.speed_hard) {
                        --     accel = get_kinematics().linear_accel.accel_hard;
                        -- }
                        -- ideal_linear_speed += accel / CONTROL_FREQUENCY_HZ;
                        -- Por hacer: Implement kinematics acceleration logic carefully with fixed-point if not using floats for speed
                        if i_kinematics.linear_accel.speed_hard = to_signed(0, 16) or s_ideal_linear_speed < i_kinematics.linear_accel.speed_hard then
                            s_ideal_linear_speed <= s_ideal_linear_speed + i_kinematics.linear_accel.accel_hard / to_signed(CONTROL_FREQUENCY_HZ, 16);
                        else
                            s_ideal_linear_speed <= s_ideal_linear_speed + i_kinematics.linear_accel.accel_soft / to_signed(CONTROL_FREQUENCY_HZ, 16);
                        end if;
                        if s_ideal_linear_speed > s_target_linear_speed then
                            s_ideal_linear_speed <= s_target_linear_speed;
                        end if;
                    elsif s_ideal_linear_speed > s_target_linear_speed then
                        -- ideal_linear_speed -= get_kinematics().linear_accel.break_accel / CONTROL_FREQUENCY_HZ;
                        s_ideal_linear_speed <= s_ideal_linear_speed - i_kinematics.linear_accel.break_accel / to_signed(CONTROL_FREQUENCY_HZ, 16);
                        if s_ideal_linear_speed < s_target_linear_speed then
                            s_ideal_linear_speed <= s_target_linear_speed;
                        end if;
                    end if;

                    -- update_fan_speed()
                    -- Por ajustar: Ensure `fan_speed_accel / CONTROL_FREQUENCY_HZ` is handled with float arithmetic
                    if s_ideal_fan_speed < to_float32(s_target_fan_speed) then
                        s_ideal_fan_speed <= s_ideal_fan_speed + s_fan_speed_accel / to_float32(CONTROL_FREQUENCY_HZ);
                        if s_ideal_fan_speed > to_float32(s_target_fan_speed) then
                            s_ideal_fan_speed <= to_float32(s_target_fan_speed);
                        end if;
                    elsif s_ideal_fan_speed > to_float32(s_target_fan_speed) then
                        s_ideal_fan_speed <= s_ideal_fan_speed + s_fan_speed_accel / to_float32(CONTROL_FREQUENCY_HZ); -- C code had + here, should be -
                        -- Por ajustar: C code has + here, assuming it's a typo and should be - for deceleration.
                        -- Assuming it should be: s_ideal_fan_speed <= s_ideal_fan_speed - s_fan_speed_accel / to_float32(CONTROL_FREQUENCY_HZ);
                        if s_ideal_fan_speed < to_float32(s_target_fan_speed) then
                            s_ideal_fan_speed <= to_float32(s_target_fan_speed);
                        end if;
                    end if;
                    r_set_fan_speed_en <= '1';
                    o_set_fan_speed_val <= signed(float32_to_std_logic_vector(s_ideal_fan_speed)(15 downto 0)); -- Por ajustar: correct conversion

                    -- PID Calculations
                    s_last_linear_error <= s_linear_error;
                    -- Por ajustar: Conversion between signed(31 downto 0) and float32.
                    s_linear_error <= s_linear_error + (to_float32(s_ideal_linear_speed) - ((i_get_encoder_left_speed + i_get_encoder_right_speed) / to_float32(2.0)));

                    s_last_angular_error <= s_angular_error;
                    s_angular_error <= s_angular_error + (s_ideal_angular_speed - (-i_lsm6dsr_get_gyro_z_radps)); -- minus sign for gyro

                    s_side_sensors_error <= to_float32(0.0);
                    -- C code: if (side_sensors_close_correction_enabled) { side_sensors_error += get_side_sensors_close_error(); sum_side_sensors_error += side_sensors_error; }
                    -- C code: if (side_sensors_far_correction_enabled) { side_sensors_error += get_side_sensors_far_error(); sum_side_sensors_error += side_sensors_error; }
                    -- The translated C code below simplifies to just get_side_sensors_error() for both close/far.
                    if s_side_sensors_close_correction_enabled = true or s_side_sensors_far_correction_enabled = true then
                        s_side_sensors_error <= s_side_sensors_error + i_get_side_sensors_error;
                        s_sum_side_sensors_error <= s_sum_side_sensors_error + s_side_sensors_error;
                    end if;

                    if s_side_sensors_close_correction_enabled = false and s_side_sensors_far_correction_enabled = false then
                        s_sum_side_sensors_error <= to_float32(0.0);
                        s_last_side_sensors_error <= to_float32(0.0);
                    end if;

                    s_front_sensors_error <= to_float32(0.0);
                    if s_front_sensors_correction_enabled = true then
                        s_front_sensors_error <= i_get_front_sensors_angle_error;
                        s_sum_front_sensors_error <= s_sum_front_sensors_error + s_front_sensors_error;
                    end if;

                    s_front_sensors_diagonal_error <= to_float32(0.0);
                    if s_front_sensors_diagonal_correction_enabled = true then
                        s_front_sensors_diagonal_error <= i_get_front_sensors_diagonal_error;
                        s_sum_front_sensors_diagonal_error <= s_sum_front_sensors_diagonal_error + s_front_sensors_diagonal_error;
                        -- Por hacer: RGB color toggle for diagonal error (commented out in C)
                        -- if (s_front_sensors_diagonal_error /= to_float32(0.0)) then
                        --     r_set_RGB_color_en <= '1'; o_set_RGB_color_r <= to_unsigned(255, 8); o_set_RGB_color_g <= (others => '0'); o_set_RGB_color_b <= (others => '0');
                        -- else
                        --     r_set_RGB_color_en <= '1'; o_set_RGB_color_r <= (others => '0'); o_set_RGB_color_g <= to_unsigned(255, 8); o_set_RGB_color_b <= (others => '0');
                        -- end if;
                    else
                        s_front_sensors_diagonal_error <= to_float32(0.0);
                        s_sum_front_sensors_diagonal_error <= to_float32(0.0);
                        s_last_front_sensors_diagonal_error <= to_float32(0.0);
                    end if;

                    s_voltage_left <=
                        (KP_LINEAR * s_linear_error + KD_LINEAR * (s_linear_error - s_last_linear_error)) +
                        (KP_ANGULAR * s_angular_error + KD_ANGULAR * (s_angular_error - s_last_angular_error) +
                         KP_SIDE_SENSORS * s_side_sensors_error + KI_SIDE_SENSORS * s_sum_side_sensors_error + KD_SIDE_SENSORS * (s_side_sensors_error - s_last_side_sensors_error) +
                         KP_FRONT_SENSORS * s_front_sensors_error + KI_FRONT_SENSORS * s_sum_front_sensors_error +
                         KP_FRONT_DIAGONAL_SENSORS * s_front_sensors_diagonal_error + KI_FRONT_DIAGONAL_SENSORS * s_sum_front_sensors_diagonal_error + KD_FRONT_DIAGONAL_SENSORS * (s_front_sensors_diagonal_error - s_last_front_sensors_diagonal_error));

                    s_voltage_right <=
                        (KP_LINEAR * s_linear_error + KD_LINEAR * (s_linear_error - s_last_linear_error)) -
                        (KP_ANGULAR * s_angular_error + KD_ANGULAR * (s_angular_error - s_last_angular_error) +
                         KP_SIDE_SENSORS * s_side_sensors_error + KI_SIDE_SENSORS * s_sum_side_sensors_error + KD_SIDE_SENSORS * (s_side_sensors_error - s_last_side_sensors_error) +
                         KP_FRONT_SENSORS * s_front_sensors_error + KI_FRONT_SENSORS * s_sum_front_sensors_error +
                         KP_FRONT_DIAGONAL_SENSORS * s_front_sensors_diagonal_error + KI_FRONT_DIAGONAL_SENSORS * s_sum_front_sensors_diagonal_error + KD_FRONT_DIAGONAL_SENSORS * (s_front_sensors_diagonal_error - s_last_front_sensors_diagonal_error));

                    s_last_side_sensors_error <= s_side_sensors_error;
                    s_last_front_sensors_diagonal_error <= s_front_sensors_diagonal_error;

                    -- C code: if (get_ideal_linear_speed() > 0) { angular_voltage *= get_ideal_linear_speed() / 500.0f; }
                    -- Por hacer: Implement optional angular_voltage scaling with linear speed if needed.

                    -- voltage_to_motor_pwm()
                    -- Por ajustar: Floating point division. Ensure get_battery_voltage() is not zero.
                    if i_get_battery_voltage /= to_float32(0.0) then
                        s_pwm_left  <= to_signed(round(float32_to_float(s_voltage_left / (i_get_battery_voltage * to_float32(1.0/8.0))) * float32_to_float(to_float32(MOTORES_MAX_PWM))), 16); -- Assumed 8.0 is a constant here. Por ajustar.
                        s_pwm_right <= to_signed(round(float32_to_float(s_voltage_right / (i_get_battery_voltage * to_float32(1.0/8.0))) * float32_to_float(to_float32(MOTORES_MAX_PWM))), 16); -- Por ajustar.
                    else
                        s_pwm_left <= (others => '0');
                        s_pwm_right <= (others => '0');
                    end if;

                    r_set_motors_pwm_en <= '1';

                    -- C code: macroarray_store calls
                    -- Por hacer: Implement macroarray_store if needed. Requires a dedicated macroarray module.
                    -- It's commented out in the C code, so it remains commented out here.
                end if;
            end if; -- end if i_control_loop_trigger

            -- --- keep_z_angle() (triggered externally) ---
            if i_keep_z_angle_trigger = '1' then
                s_last_linear_error <= s_linear_error;
                s_linear_error <= s_linear_error + (to_float32(s_ideal_linear_speed) - ((i_get_encoder_left_speed + i_get_encoder_right_speed) / to_float32(2.0)));

                s_last_angular_error <= s_angular_error;
                s_angular_error <= s_angular_error + (s_ideal_angular_speed - (-i_lsm6dsr_get_gyro_z_radps));

                s_voltage_left <= (KP_LINEAR * s_linear_error + KD_LINEAR * (s_linear_error - s_last_linear_error)) +
                                  (KP_ANGULAR * s_angular_error + KD_ANGULAR * (s_angular_error - s_last_angular_error));
                s_voltage_right <= (KP_LINEAR * s_linear_error + KD_LINEAR * (s_linear_error - s_last_linear_error)) -
                                   (KP_ANGULAR * s_angular_error + KD_ANGULAR * (s_angular_error - s_last_angular_error));

                -- voltage_to_motor_pwm() logic repeated
                if i_get_battery_voltage /= to_float32(0.0) then
                    s_pwm_left  <= to_signed(round(float32_to_float(s_voltage_left / (i_get_battery_voltage * to_float32(1.0/8.0))) * float32_to_float(to_float32(MOTORES_MAX_PWM))), 16); -- Por ajustar
                    s_pwm_right <= to_signed(round(float32_to_float(s_voltage_right / (i_get_battery_voltage * to_float32(1.0/8.0))) * float32_to_float(to_float32(MOTORES_MAX_PWM))), 16); -- Por ajustar
                else
                    s_pwm_left <= (others => '0');
                    s_pwm_right <= (others => '0');
                end if;

                r_gpio_15_set_reg <= '1'; -- gpio_set(GPIOB, GPIO15)
                r_set_motors_pwm_en <= '1';
            end if;

        end if; -- rising_edge(i_clk)
    end process;

    --- Combinatorial Logic (for functions like voltage_to_motor_pwm which are simple calculations) ---
    -- Note: voltage_to_motor_pwm and percentage_to_fan_pwm results are used in the main process.
    -- These are now effectively part of the main process where they are called.
    -- However, if you need them as standalone functions or to be driven by external inputs
    -- outside the main control_loop, they would be separate concurrent statements or processes.

    -- percentage_to_fan_pwm()
    -- Por ajustar: Floating point division. Ensure get_battery_voltage() is not zero.
    -- This calculation is moved inside the main process when setting o_set_fan_speed_val.
    -- The C code's constrain function (min/max) is applied there.
    -- C code: return percentage > 0 ? (int32_t)constrain((get_battery_high_limit_voltage() / get_battery_voltage()) * percentage, percentage, 100.0f) : 0;
    -- Por hacer: Implement constrain logic if not handled by target_fan_speed and ideal_fan_speed updates.

end architecture rtl;
