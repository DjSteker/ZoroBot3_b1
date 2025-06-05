
-- debug_pkg.vhd
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package debug_pkg is
    -- Define sensor IDs (adjust these values based on your actual system)
    constant SENSOR_SIDE_LEFT_WALL_ID  : natural := 0;
    constant SENSOR_FRONT_LEFT_WALL_ID : natural := 1;
    constant SENSOR_FRONT_RIGHT_WALL_ID: natural := 2;
    constant SENSOR_SIDE_RIGHT_WALL_ID : natural := 3;

    -- Define AUX IDs
    constant AUX_BATTERY_ID     : natural := 0;
    constant AUX_CURRENT_LEFT_ID: natural := 1;
    constant AUX_CURRENT_RIGHT_ID: natural := 2;
    constant AUX_MENU_BTN_ID    : natural := 3;

    -- Debug types from config
    type t_debug_type is (
        DEBUG_NONE,
        DEBUG_MACROARRAY,
        DEBUG_TYPE_SENSORS_RAW,
        DEBUG_TYPE_SENSORS_DISTANCES,
        DEBUG_FLOODFILL_MAZE,
        DEBUG_MOTORS_CURRENT,
        DEBUG_GYRO_DEMO,
        DEBUG_FAN_DEMO
    );

    -- Print format for sensor data (adjust width as needed)
    constant C_SENSOR_PRINT_WIDTH : natural := 4;
    constant C_AUX_PRINT_WIDTH    : natural := 4;

    -- Define a generic type for sensor data or print data if needed
    -- For printf, we'll send integer values and a format string ID.
    -- Assuming up to 4 values per print line for simplicity, if more needed, adjust array size.
    type t_print_data_array is array (0 to 3) of signed(15 downto 0); -- Assuming sensor values fit in 16 bits
    constant C_NUM_PRINT_VALUES : natural := 4; -- Max values per print line

    -- Print Message IDs (define IDs for each printf format string)
    constant MSG_ID_SENSORS_RAW_FMT     : natural := 0;
    constant MSG_ID_SENSORS_DIST_FMT    : natural := 1;
    constant MSG_ID_MOTORS_CURR_FMT     : natural := 2;
    constant MSG_ID_HINVERNADERO_INIT   : natural := 3;
    constant MSG_ID_INIT_DONE           : natural := 4;
    -- Add more as needed for other printf calls in your system

end package debug_pkg;


















-- debug.vhd
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.debug_pkg.all;

entity debug is
    port (
        i_clk             : in  std_logic;                                      -- System clock
        i_rst_n           : in  std_logic;                                      -- Asynchronous active-low reset
        i_get_clock_ticks : in  unsigned(31 downto 0);                          -- Millisecond tick counter

        -- Inputs for external functions
        i_menu_mode_btn   : in  std_logic;                                      -- get_menu_mode_btn()
        i_debug_config_type : in  t_debug_type;                                 -- debug_from_config(type) input
        i_debug_main_trigger: in  std_logic;                                    -- debug_from_main() trigger pulse

        -- Outputs/Triggers for external functions
        o_macroarray_print_trigger  : out std_logic;                            -- macroarray_print() trigger
        o_floodfill_maze_print_trigger: out std_logic;                          -- floodfill_maze_print() trigger
        o_menu_config_reset_values_trigger: out std_logic;                      -- menu_config_reset_values() trigger
        o_set_sensors_enabled_en    : out std_logic;                            -- set_sensors_enabled(true/false) enable
        o_set_sensors_enabled_val   : out std_logic;                            -- set_sensors_enabled(true/false) value

        o_set_motors_enable_en      : out std_logic;                            -- set_motors_enable(true/false) enable
        o_set_motors_enable_val     : out std_logic;                            -- set_motors_enable(true/false) value
        o_set_motors_speed_en       : out std_logic;                            -- set_motors_speed(left, right) enable
        o_set_motors_speed_left     : out signed(15 downto 0);                  -- set_motors_speed left value
        o_set_motors_speed_right    : out signed(15 downto 0);                  -- set_motors_speed right value

        o_reset_control_all_trigger : out std_logic;                            -- reset_control_all() trigger
        o_keep_z_angle_trigger      : out std_logic;                            -- keep_z_angle() trigger
        o_set_fan_speed_en          : out std_logic;                            -- set_fan_speed(speed) enable
        o_set_fan_speed_val         : out signed(15 downto 0);                  -- set_fan_speed speed value

        o_set_RGB_color_en          : out std_logic;                            -- set_RGB_color(r, g, b) enable
        o_set_RGB_color_r           : out unsigned(7 downto 0);                 -- RGB Red component
        o_set_RGB_color_g           : out unsigned(7 downto 0);                 -- RGB Green component
        o_set_RGB_color_b           : out unsigned(7 downto 0);                 -- RGB Blue component

        -- Sensor inputs
        i_get_sensor_raw_sl_0     : in  signed(15 downto 0);                    -- get_sensor_raw(SENSOR_SIDE_LEFT_WALL_ID, 0)
        i_get_sensor_raw_sl_1     : in  signed(15 downto 0);                    -- get_sensor_raw(SENSOR_SIDE_LEFT_WALL_ID, 1)
        i_get_sensor_raw_filter_sl: in  signed(15 downto 0);                    -- get_sensor_raw_filter(SENSOR_SIDE_LEFT_WALL_ID)
        i_get_sensor_raw_fl_0     : in  signed(15 downto 0);                    -- get_sensor_raw(SENSOR_FRONT_LEFT_WALL_ID, 0)
        i_get_sensor_raw_fl_1     : in  signed(15 downto 0);                    -- get_sensor_raw(SENSOR_FRONT_LEFT_WALL_ID, 1)
        i_get_sensor_raw_filter_fl: in  signed(15 downto 0);                    -- get_sensor_raw_filter(SENSOR_FRONT_LEFT_WALL_ID)
        i_get_sensor_raw_fr_0     : in  signed(15 downto 0);                    -- get_sensor_raw(SENSOR_FRONT_RIGHT_WALL_ID, 0)
        i_get_sensor_raw_fr_1     : in  signed(15 downto 0);                    -- get_sensor_raw(SENSOR_FRONT_RIGHT_WALL_ID, 1)
        i_get_sensor_raw_filter_fr: in  signed(15 downto 0);                    -- get_sensor_raw_filter(SENSOR_FRONT_RIGHT_WALL_ID)
        i_get_sensor_raw_sr_0     : in  signed(15 downto 0);                    -- get_sensor_raw(SENSOR_SIDE_RIGHT_WALL_ID, 0)
        i_get_sensor_raw_sr_1     : in  signed(15 downto 0);                    -- get_sensor_raw(SENSOR_SIDE_RIGHT_WALL_ID, 1)
        i_get_sensor_raw_filter_sr: in  signed(15 downto 0);                    -- get_sensor_raw_filter(SENSOR_SIDE_RIGHT_WALL_ID)

        i_get_sensor_distance_sl  : in  signed(15 downto 0);                    -- get_sensor_distance(SENSOR_SIDE_LEFT_WALL_ID)
        i_get_sensor_distance_fl  : in  signed(15 downto 0);                    -- get_sensor_distance(SENSOR_FRONT_LEFT_WALL_ID)
        i_get_sensor_distance_fr  : in  signed(15 downto 0);                    -- get_sensor_distance(SENSOR_FRONT_RIGHT_WALL_ID)
        i_get_sensor_distance_sr  : in  signed(15 downto 0);                    -- get_sensor_distance(SENSOR_SIDE_RIGHT_WALL_ID)

        -- AUX inputs
        i_get_aux_battery       : in  signed(15 downto 0);                      -- get_aux_raw(AUX_BATTERY_ID)
        i_get_aux_current_left  : in  signed(15 downto 0);                      -- get_aux_raw(AUX_CURRENT_LEFT_ID)
        i_get_aux_current_right : in  signed(15 downto 0);                      -- get_aux_raw(AUX_CURRENT_RIGHT_ID)
        i_get_aux_menu_btn      : in  signed(15 downto 0);                      -- get_aux_raw(AUX_MENU_BTN_ID)

        -- Printf interface
        o_print_trigger         : out std_logic;                                -- Pulse to trigger a print
        o_print_msg_id          : out natural;                                  -- ID of the message format
        o_print_data            : out t_print_data_array;                       -- Data values to print
        o_debug_enabled         : out std_logic                                 -- is_debug_enabled()
    );
end entity debug;

architecture rtl of debug is

    -- Internal states
    signal s_debug_enabled     : boolean := false;
    signal s_last_print_debug  : unsigned(31 downto 0) := (others => '0');
    signal s_last_keep_z_angle : unsigned(31 downto 0) := (others => '0');

    -- State machine for main debug loop
    type t_debug_main_state is (
        S_IDLE,
        S_CHECK_DEBUG_ACTIVE,
        S_WAIT_MENU_BTN_RELEASE,
        S_UPDATE_DEBUG_STATE,
        S_DEBUG_MACROARRAY,
        S_DEBUG_SENSORS_RAW,
        S_DEBUG_SENSORS_DISTANCES,
        S_DEBUG_FLOODFILL_MAZE,
        S_DEBUG_MOTORS_CURRENT,
        S_DEBUG_GYRO_DEMO_START,
        S_DEBUG_GYRO_DEMO_LOOP,
        S_DEBUG_FAN_DEMO_START,
        S_DEBUG_FAN_DEMO_LOOP,
        S_DELAY_1000MS, -- For gyro_demo and fan_demo initial delay
        S_DELAY_KEEP_Z_ANGLE -- For 1ms delay in keep_z_angle loop
    );
    signal s_debug_state         : t_debug_main_state := S_IDLE;
    signal s_current_debug_type  : t_debug_type := DEBUG_NONE;

    -- Delay counter
    signal s_delay_start_ms      : unsigned(31 downto 0);
    signal s_delay_duration_ms   : unsigned(31 downto 0);
    signal s_delay_done_pulse    : std_logic;

    -- Local signals for output pulses (to avoid combinatorial loops with state updates)
    signal r_macroarray_print_trigger   : std_logic := '0';
    signal r_floodfill_maze_print_trigger : std_logic := '0';
    signal r_menu_config_reset_values_trigger: std_logic := '0';
    signal r_set_sensors_enabled_en     : std_logic := '0';
    signal r_set_sensors_enabled_val    : std_logic := '0';
    signal r_set_motors_enable_en       : std_logic := '0';
    signal r_set_motors_enable_val      : std_logic := '0';
    signal r_set_motors_speed_en        : std_logic := '0';
    signal r_reset_control_all_trigger  : std_logic := '0';
    signal r_keep_z_angle_trigger       : std_logic := '0';
    signal r_set_fan_speed_en           : std_logic := '0';
    signal r_set_RGB_color_en           : std_logic := '0';
    signal r_print_trigger              : std_logic := '0';

begin

    o_debug_enabled <= '1' when s_debug_enabled else '0';

    -- Assign outputs from internal registers/signals
    o_macroarray_print_trigger      <= r_macroarray_print_trigger;
    o_floodfill_maze_print_trigger  <= r_floodfill_maze_print_trigger;
    o_menu_config_reset_values_trigger <= r_menu_config_reset_values_trigger;
    o_set_sensors_enabled_en        <= r_set_sensors_enabled_en;
    o_set_sensors_enabled_val       <= r_set_sensors_enabled_val;
    o_set_motors_enable_en          <= r_set_motors_enable_en;
    o_set_motors_enable_val         <= r_set_motors_enable_val;
    o_set_motors_speed_en           <= r_set_motors_speed_en;
    o_reset_control_all_trigger     <= r_reset_control_all_trigger;
    o_keep_z_angle_trigger          <= r_keep_z_angle_trigger;
    o_set_fan_speed_en              <= r_set_fan_speed_en;
    o_set_RGB_color_en              <= r_set_RGB_color_en;
    o_print_trigger                 <= r_print_trigger;

    -- Set constant motor speeds/RGB colors
    o_set_motors_speed_left         <= to_signed(150, 16);
    o_set_motors_speed_right        <= to_signed(150, 16);
    o_set_RGB_color_r               <= (others => '0');
    o_set_RGB_color_g               <= to_unsigned(50, 8);
    o_set_RGB_color_b               <= (others => '0');
    o_set_fan_speed_val             <= to_signed(50, 16); -- For debug_fan_demo

    --- Delay Logic ---
    -- This process implements a simple non-blocking delay based on clock ticks
    process (i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            s_delay_done_pulse <= '0';
        elsif rising_edge(i_clk) then
            s_delay_done_pulse <= '0'; -- Clear pulse each cycle

            -- Check if delay is active and time has elapsed
            if s_delay_start_ms /= (others => '0') then -- Check if delay is 'active' (not reset to 0)
                if i_get_clock_ticks >= s_delay_start_ms and (i_get_clock_ticks - s_delay_start_ms) >= s_delay_duration_ms then
                    s_delay_done_pulse <= '1';
                    s_delay_start_ms   <= (others => '0'); -- Mark delay as finished
                elsif i_get_clock_ticks < s_delay_start_ms then -- Handle overflow
                    if (C_MAX_UINT32_VAL - s_delay_start_ms) + i_get_clock_ticks >= s_delay_duration_ms then
                        s_delay_done_pulse <= '1';
                        s_delay_start_ms   <= (others => '0');
                    end if;
                end if;
            end if;
        end if;
    end process;

    --- Main Debug FSM ---
    process (i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            s_debug_enabled             <= false;
            s_last_print_debug          <= (others => '0');
            s_last_keep_z_angle         <= (others => '0');
            s_debug_state               <= S_IDLE;
            s_current_debug_type        <= DEBUG_NONE;
            s_delay_start_ms            <= (others => '0');
            s_delay_duration_ms         <= (others => '0');

            -- Reset all output triggers
            r_macroarray_print_trigger   <= '0';
            r_floodfill_maze_print_trigger <= '0';
            r_menu_config_reset_values_trigger <= '0';
            r_set_sensors_enabled_en     <= '0';
            r_set_sensors_enabled_val    <= '0';
            r_set_motors_enable_en       <= '0';
            r_set_motors_enable_val      <= '0';
            r_set_motors_speed_en        <= '0';
            r_reset_control_all_trigger  <= '0';
            r_keep_z_angle_trigger       <= '0';
            r_set_fan_speed_en           <= '0';
            r_set_RGB_color_en           <= '0';
            r_print_trigger              <= '0';
            o_print_msg_id               <= 0;
            o_print_data                 <= (others => (others => '0'));

        elsif rising_edge(i_clk) then
            -- Clear one-shot pulses by default
            r_macroarray_print_trigger   <= '0';
            r_floodfill_maze_print_trigger <= '0';
            r_menu_config_reset_values_trigger <= '0';
            r_set_sensors_enabled_en     <= '0';
            r_set_motors_enable_en       <= '0';
            r_set_motors_speed_en        <= '0';
            r_reset_control_all_trigger  <= '0';
            r_keep_z_angle_trigger       <= '0';
            r_set_fan_speed_en           <= '0';
            r_set_RGB_color_en           <= '0';
            r_print_trigger              <= '0';
            o_print_msg_id               <= 0; -- Default ID
            o_print_data                 <= (others => (others => '0')); -- Clear data

            -- check_debug_active logic (always active to monitor button)
            case s_debug_state is
                when S_IDLE =>
                    -- Check for debug_from_config or debug_from_main calls
                    if i_debug_main_trigger = '1' then
                        s_debug_enabled <= true;
                        r_set_sensors_enabled_en <= '1';
                        r_set_sensors_enabled_val <= '1';
                        s_current_debug_type <= i_debug_config_type; -- Or a specific type if debug_from_main implies one
                        s_debug_state <= S_CHECK_DEBUG_ACTIVE;
                    elsif i_debug_config_type /= DEBUG_NONE then
                        -- debug_from_config(type)
                        s_current_debug_type <= i_debug_config_type;
                        s_debug_state <= S_CHECK_DEBUG_ACTIVE;
                    else
                        s_debug_enabled <= false;
                        r_set_RGB_color_en <= '1';
                        o_set_RGB_color_r <= (others => '0');
                        o_set_RGB_color_g <= (others => '0');
                        o_set_RGB_color_b <= (others => '0');
                    end if;

                when S_CHECK_DEBUG_ACTIVE =>
                    if i_menu_mode_btn = '1' then
                        s_debug_state <= S_WAIT_MENU_BTN_RELEASE;
                    else
                        -- If debug_enabled is false, jump to S_IDLE
                        if s_debug_enabled = false then
                            s_debug_state <= S_IDLE;
                        else
                            -- Debug is enabled, proceed to execute current debug type
                            r_set_sensors_enabled_en <= '1';
                            r_set_sensors_enabled_val <= '1';
                            r_set_RGB_color_en <= '1';
                            o_set_RGB_color_r <= (others => '0');
                            o_set_RGB_color_g <= to_unsigned(50, 8);
                            o_set_RGB_color_b <= (others => '0');

                            case s_current_debug_type is
                                when DEBUG_MACROARRAY =>      s_debug_state <= S_DEBUG_MACROARRAY;
                                when DEBUG_TYPE_SENSORS_RAW => s_debug_state <= S_DEBUG_SENSORS_RAW;
                                when DEBUG_TYPE_SENSORS_DISTANCES => s_debug_state <= S_DEBUG_SENSORS_DISTANCES;
                                when DEBUG_FLOODFILL_MAZE =>  s_debug_state <= S_DEBUG_FLOODFILL_MAZE;
                                when DEBUG_MOTORS_CURRENT =>  s_debug_state <= S_DEBUG_MOTORS_CURRENT;
                                when DEBUG_GYRO_DEMO =>       s_debug_state <= S_DEBUG_GYRO_DEMO_START;
                                when DEBUG_FAN_DEMO =>        s_debug_state <= S_DEBUG_FAN_DEMO_START;
                                when others =>
                                    s_debug_enabled <= false;
                                    s_debug_state <= S_IDLE;
                            end case;
                        end if;
                    end if;

                when S_WAIT_MENU_BTN_RELEASE =>
                    if i_menu_mode_btn = '0' then
                        s_debug_state <= S_UPDATE_DEBUG_STATE;
                    end if;

                when S_UPDATE_DEBUG_STATE =>
                    s_debug_enabled <= not s_debug_enabled;
                    if s_debug_enabled = false then
                        r_menu_config_reset_values_trigger <= '1';
                        s_debug_state <= S_IDLE;
                    else
                        -- Stay in check_debug_active state to re-evaluate after button press
                        s_debug_state <= S_CHECK_DEBUG_ACTIVE;
                    end if;

                -- Individual Debug Modes (C code logic)
                when S_DEBUG_MACROARRAY =>
                    r_macroarray_print_trigger <= '1';
                    s_debug_enabled <= false;
                    r_menu_config_reset_values_trigger <= '1';
                    s_debug_state <= S_IDLE;

                when S_DEBUG_SENSORS_RAW =>
                    if i_get_clock_ticks >= s_last_print_debug and (i_get_clock_ticks - s_last_print_debug) >= 50 then
                        r_print_trigger <= '1';
                        o_print_msg_id <= MSG_ID_SENSORS_RAW_FMT;
                        o_print_data(0) <= i_get_sensor_raw_sl_1;
                        o_print_data(1) <= i_get_sensor_raw_sl_0;
                        o_print_data(2) <= i_get_sensor_raw_filter_sl;
                        -- Add more values as needed, might need multiple print calls or larger array
                        s_last_print_debug <= i_get_clock_ticks;
                    end if;
                    s_debug_state <= S_CHECK_DEBUG_ACTIVE; -- Always re-check active for continuous modes

                when S_DEBUG_SENSORS_DISTANCES =>
                    if i_get_clock_ticks >= s_last_print_debug and (i_get_clock_ticks - s_last_print_debug) >= 50 then
                        r_print_trigger <= '1';
                        o_print_msg_id <= MSG_ID_SENSORS_DIST_FMT;
                        o_print_data(0) <= i_get_sensor_distance_sl;
                        o_print_data(1) <= i_get_sensor_distance_fl;
                        o_print_data(2) <= i_get_sensor_distance_fr;
                        o_print_data(3) <= i_get_sensor_distance_sr;
                        s_last_print_debug <= i_get_clock_ticks;
                    end if;
                    s_debug_state <= S_CHECK_DEBUG_ACTIVE;

                when S_DEBUG_FLOODFILL_MAZE =>
                    r_floodfill_maze_print_trigger <= '1';
                    s_debug_enabled <= false;
                    r_menu_config_reset_values_trigger <= '1';
                    s_debug_state <= S_IDLE;

                when S_DEBUG_MOTORS_CURRENT =>
                    if i_get_clock_ticks >= s_last_print_debug and (i_get_clock_ticks - s_last_print_debug) >= 50 then
                        r_set_motors_enable_en <= '1';
                        r_set_motors_enable_val <= '1'; -- true
                        r_set_motors_speed_en <= '1';

                        r_print_trigger <= '1';
                        o_print_msg_id <= MSG_ID_MOTORS_CURR_FMT;
                        o_print_data(0) <= i_get_aux_battery;
                        o_print_data(1) <= i_get_aux_current_left;
                        o_print_data(2) <= i_get_aux_current_right;
                        o_print_data(3) <= i_get_aux_menu_btn;
                        s_last_print_debug <= i_get_clock_ticks;
                    end if;
                    s_debug_state <= S_CHECK_DEBUG_ACTIVE;

                when S_DEBUG_GYRO_DEMO_START =>
                    r_reset_control_all_trigger <= '1';
                    s_delay_start_ms <= i_get_clock_ticks;
                    s_delay_duration_ms <= to_unsigned(1000, 32); -- delay(1000)
                    s_debug_state <= S_DELAY_1000MS;

                when S_DELAY_1000MS =>
                    if s_delay_done_pulse = '1' then
                        -- Check which demo called this delay
                        if s_current_debug_type = DEBUG_GYRO_DEMO then
                            s_debug_state <= S_DEBUG_GYRO_DEMO_LOOP;
                        elsif s_current_debug_type = DEBUG_FAN_DEMO then
                            s_debug_state <= S_DEBUG_FAN_DEMO_LOOP;
                        else
                            s_debug_state <= S_IDLE; -- Should not happen
                        end if;
                    end if;

                when S_DEBUG_GYRO_DEMO_LOOP =>
                    -- This loop continues as long as debug_enabled is true
                    if s_debug_enabled = false then
                        r_set_fan_speed_en <= '1';
                        o_set_fan_speed_val <= to_signed(0, 16);
                        r_reset_control_all_trigger <= '1';
                        s_debug_state <= S_IDLE;
                    else
                        if i_get_clock_ticks >= s_last_keep_z_angle and (i_get_clock_ticks - s_last_keep_z_angle) >= 1 then
                            r_keep_z_angle_trigger <= '1';
                            s_last_keep_z_angle <= i_get_clock_ticks;
                        end if;
                        s_debug_state <= S_CHECK_DEBUG_ACTIVE; -- Always re-check for button press
                    end if;

                when S_DEBUG_FAN_DEMO_START =>
                    r_reset_control_all_trigger <= '1';
                    s_delay_start_ms <= i_get_clock_ticks;
                    s_delay_duration_ms <= to_unsigned(1000, 32); -- delay(1000)
                    s_debug_state <= S_DELAY_1000MS;
                
                when S_DEBUG_FAN_DEMO_LOOP =>
                    r_set_fan_speed_en <= '1';
                    o_set_fan_speed_val <= to_signed(50, 16);
                    if s_debug_enabled = false then
                        r_set_fan_speed_en <= '1';
                        o_set_fan_speed_val <= to_signed(0, 16);
                        r_reset_control_all_trigger <= '1';
                        s_debug_state <= S_IDLE;
                    else
                        if i_get_clock_ticks >= s_last_keep_z_angle and (i_get_clock_ticks - s_last_keep_z_angle) >= 1 then
                            r_keep_z_angle_trigger <= '1';
                            s_last_keep_z_angle <= i_get_clock_ticks;
                        end if;
                        s_debug_state <= S_CHECK_DEBUG_ACTIVE; -- Always re-check for button press
                    end if;

                when others =>
                    s_debug_state <= S_IDLE; -- Default to safe state
            end case;
        end if;
    end process;

end architecture rtl;
