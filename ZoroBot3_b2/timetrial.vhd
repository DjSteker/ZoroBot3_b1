
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Define constants for maze and robot dimensions (assuming fixed-point or integer scaling)
-- These should match the C definitions after conversion (e.g., in millimeters/1000)
constant C_CELL_DIMENSION    : natural := 180; -- Example: 180 units (e.g., mm)
constant C_ROBOT_BACK_LENGTH : natural := 30;  -- Example: 30 units
constant C_WALL_WIDTH        : natural := 12;  -- Example: 12 units

-- Define a type for robot kinematics parameters
-- This would be a record or separate signals in VHDL
type kinematics_data_t is record
    fan_speed          : natural;
    linear_speed       : natural;
    -- Placeholder for turn parameters. Assuming all speeds are 'natural' (unsigned integers)
    turn_right_90_start_speed : natural;
    turn_right_90_end_speed   : natural;
    turn_right_90_linear_speed: natural;
end record;

-- Define types for maze types (from maze_logic_module)
type maze_type_t is (MAZE_HOME, MAZE_COMPETITION);

--------------------------------------------------------------------------------
-- Entity: timetrial_controller
-- Description: VHDL FSM to sequence the robot's actions for a time trial.
--              It manages high-level orchestration, interacting with sub-modules.
--
-- Inputs:
--   i_clk             : Master clock signal
--   i_reset           : Asynchronous reset signal
--   i_start_timetrial : Pulse to initiate the time trial (replaces timetrial_start())
--   i_current_maze_rows   : Input from maze_logic_module (replaces maze_get_rows())
--   i_current_maze_columns: Input from maze_logic_module (replaces maze_get_columns())
--   i_kinematics_data : Input from kinematics configuration module
--   i_movement_done   : A pulse/flag from the motor controller indicating movement complete
--   i_timer_done      : A pulse/flag from the TramasMicros2_timer module indicating delay complete
--
-- Outputs:
--   o_configure_kinematics_en : Enable signal for kinematics configuration
--   o_set_speed             : Speed value for kinematics configuration
--   o_clear_info_leds_en    : Enable signal for LED clear
--   o_set_RGB_color_en      : Enable signal for RGB LED setting
--   o_RGB_R, o_RGB_G, o_RGB_B: RGB color values
--   o_set_target_fan_speed_en: Enable for fan speed setter
--   o_target_fan_speed      : Fan speed value
--   o_target_fan_accel_time : Fan acceleration time
--   o_timer_set_interval_en : Enable to set timer interval
--   o_timer_interval_us     : Interval value for timer (microseconds)
--   o_run_straight_en       : Enable for straight movement module
--   o_run_straight_dist_val : Distance for straight movement (in its internal unit)
--   o_run_straight_start_speed: Start speed for straight
--   o_run_straight_end_speed: End speed for straight
--   o_run_straight_cells_num: Number of cells for straight
--   o_run_straight_accel_en : Acceleration enable for straight
--   o_run_straight_lin_speed: Linear speed for straight
--   o_run_straight_turn_lin_speed: Turn linear speed for straight
--   o_run_side_en           : Enable for side movement module
--   o_run_side_type         : Type of side movement (e.g., MOVE_RIGHT_90)
--   o_run_side_turn_start_speed: Specific turn parameters
--   o_run_side_turn_end_speed  :
--   o_run_side_turn_linear_speed:
--   o_move_back_stop_en     : Enable for final stop
--   o_set_race_started_flag : Sets/clears a 'race started' flag
--------------------------------------------------------------------------------
entity timetrial_controller is
    port (
        i_clk             : in  std_logic;
        i_reset           : in  std_logic;
        i_start_timetrial : in  std_logic;
        i_current_maze_rows : in natural;
        i_current_maze_columns: in natural;
        i_kinematics_data : in kinematics_data_t;
        i_movement_done   : in  std_logic; -- From motor control/distance module
        i_timer_done      : in  std_logic; -- From TramasMicros2_timer module

        o_configure_kinematics_en : out std_logic;
        o_set_speed             : out natural;
        o_clear_info_leds_en    : out std_logic;
        o_set_RGB_color_en      : out std_logic;
        o_RGB_R                 : out std_logic_vector(7 downto 0);
        o_RGB_G                 : out std_logic_vector(7 downto 0);
        o_RGB_B                 : out std_logic_vector(7 downto 0);
        o_set_target_fan_speed_en: out std_logic;
        o_target_fan_speed      : out natural;
        o_target_fan_accel_time : out natural;
        o_timer_set_interval_en : out std_logic;
        o_timer_interval_us     : out natural;
        o_run_straight_en       : out std_logic;
        o_run_straight_dist_val : out natural;
        o_run_straight_start_speed: out natural;
        o_run_straight_end_speed: out natural;
        o_run_straight_cells_num: out natural;
        o_run_straight_accel_en : out std_logic;
        o_run_straight_lin_speed: out natural;
        o_run_straight_turn_lin_speed: out natural;
        o_run_side_en           : out std_logic;
        o_run_side_type         : out std_logic_vector(1 downto 0);
        o_run_side_turn_start_speed: out natural;
        o_run_side_turn_end_speed  : out natural;
        o_run_side_turn_linear_speed: out natural;
        o_move_back_stop_en     : out std_logic;
        o_set_race_started_flag : out std_logic
    );
end entity timetrial_controller;

architecture Behavioral of timetrial_controller is

    -- Define states for the FSM
    type state_type is (
        STATE_IDLE,
        STATE_START_INIT,
        STATE_CONFIG_KINEMATICS,
        STATE_CLEAR_LEDS,
        STATE_SET_RGB,
        STATE_SET_FAN_SPEED,
        STATE_SETUP_DELAY_TIMER, -- New state to configure the delay timer
        STATE_WAIT_DELAY_TIMER,  -- New state to wait for the delay timer
        STATE_MOVE_STRAIGHT_1,
        STATE_TURN_SIDE_1,
        STATE_MOVE_STRAIGHT_2,
        STATE_TURN_SIDE_2,
        STATE_MOVE_STRAIGHT_3,
        STATE_TURN_SIDE_3,
        STATE_MOVE_STRAIGHT_4,
        STATE_TURN_SIDE_4,
        STATE_FINAL_STOP,
        STATE_RESET_RACE_FLAG,
        STATE_DONE
    );

    signal current_state : state_type;
    signal next_state    : state_type;

    -- Constants for specific movement types (mapping C enums to VHDL bits)
    constant C_MOVE_RIGHT_90 : std_logic_vector(1 downto 0) := "01";
    constant C_MOVE_BACK_STOP: std_logic_vector(1 downto 0) := "00"; -- Example, adjust as needed

    -- Calculations for distances (combinatorial or part of state logic)
    -- Assuming WALL_WIDTH / 2.0f is handled by fixed-point scaling in C or as a constant.
    -- If CELL_DIMENSION, ROBOT_BACK_LENGTH, WALL_WIDTH are integers, division by 2 for WALL_WIDTH
    -- implies integer division. For more precision, scale all units (e.g., *1000 for mm).
    signal s_straight_dist_1 : natural;
    signal s_straight_dist_2_3_4 : natural;

begin

    -- Combinatorial logic for distances
    -- The C code has (WALL_WIDTH / 2.0f). If WALL_WIDTH is integer, this implies 0.5.
    -- For VHDL using 'natural', we can approximate (e.g., WALL_WIDTH / 2 for integer division)
    -- or use scaled integers if true floating point precision is critical.
    s_straight_dist_1 <= C_CELL_DIMENSION * (i_current_maze_rows - 1) - (C_ROBOT_BACK_LENGTH + C_WALL_WIDTH / 2);
    s_straight_dist_2_3_4 <= C_CELL_DIMENSION * (i_current_maze_columns - 2);

    -- State register process
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                current_state <= STATE_IDLE;
            else
                current_state <= next_state;
            end if;
        end if;
    end process;

    -- Next state and output logic (FSM logic)
    process (current_state, i_start_timetrial, i_movement_done, i_timer_done,
             i_kinematics_data, i_current_maze_rows, i_current_maze_columns,
             s_straight_dist_1, s_straight_dist_2_3_4)
    begin
        -- Default outputs (active low or no action for pulses)
        o_configure_kinematics_en <= '0';
        o_set_speed             <= 0;
        o_clear_info_leds_en    <= '0';
        o_set_RGB_color_en      <= '0';
        o_RGB_R                 <= (others => '0');
        o_RGB_G                 <= (others => '0');
        o_RGB_B                 <= (others => '0');
        o_set_target_fan_speed_en <= '0';
        o_target_fan_speed      <= 0;
        o_target_fan_accel_time <= 0;
        o_timer_set_interval_en <= '0';
        o_timer_interval_us     <= 0;
        o_run_straight_en       <= '0';
        o_run_straight_dist_val <= 0;
        o_run_straight_start_speed <= 0;
        o_run_straight_end_speed   <= 0;
        o_run_straight_cells_num   <= 0;
        o_run_straight_accel_en    <= '0';
        o_run_straight_lin_speed   <= 0;
        o_run_straight_turn_lin_speed <= 0;
        o_run_side_en           <= '0';
        o_run_side_type         <= (others => '0');
        o_run_side_turn_start_speed <= 0;
        o_run_side_turn_end_speed   <= 0;
        o_run_side_turn_linear_speed<= 0;
        o_move_back_stop_en     <= '0';
        o_set_race_started_flag <= '0'; -- Default to race not started

        -- Default next state
        next_state <= current_state;

        case current_state is
            when STATE_IDLE =>
                if i_start_timetrial = '1' then
                    next_state <= STATE_START_INIT;
                    o_set_race_started_flag <= '1'; -- Race starts here conceptually
                end if;

            -- timetrial_start() sequence
            when STATE_START_INIT =>
                o_configure_kinematics_en <= '1';
                o_set_speed <= i_kinematics_data.linear_speed;
                next_state <= STATE_CONFIG_KINEMATICS;

            when STATE_CONFIG_KINEMATICS =>
                -- This state is for a single clock cycle to assert the enable.
                -- Move immediately to next state.
                next_state <= STATE_CLEAR_LEDS;

            when STATE_CLEAR_LEDS =>
                o_clear_info_leds_en <= '1';
                next_state <= STATE_SET_RGB;

            when STATE_SET_RGB =>
                o_set_RGB_color_en <= '1';
                o_RGB_R <= (others => '0'); -- 0,0,0
                o_RGB_G <= (others => '0');
                o_RGB_B <= (others => '0');
                next_state <= STATE_SET_FAN_SPEED;

            when STATE_SET_FAN_SPEED =>
                o_set_target_fan_speed_en <= '1';
                o_target_fan_speed      <= i_kinematics_data.fan_speed;
                o_target_fan_accel_time <= 400;
                next_state <= STATE_SETUP_DELAY_TIMER; -- Transition to setup timer

            when STATE_SETUP_DELAY_TIMER =>
                o_timer_set_interval_en <= '1'; -- Tell the timer to set a new interval
                o_timer_interval_us     <= 500 * 1000; -- 500 ms in microseconds
                next_state <= STATE_WAIT_DELAY_TIMER; -- Wait for timer to complete

            when STATE_WAIT_DELAY_TIMER =>
                if i_timer_done = '1' then -- Wait for timer to assert its done signal
                    next_state <= STATE_MOVE_STRAIGHT_1;
                end if;

            -- timetrial_loop() sequence - each movement is a state
            when STATE_MOVE_STRAIGHT_1 =>
                o_run_straight_en       <= '1';
                o_run_straight_dist_val <= s_straight_dist_1;
                o_run_straight_start_speed <= i_kinematics_data.turn_right_90_start_speed;
                o_run_straight_end_speed   <= i_kinematics_data.turn_right_90_start_speed;
                o_run_straight_cells_num   <= i_current_maze_rows - 1;
                o_run_straight_accel_en    <= '1'; -- true
                o_run_straight_lin_speed   <= i_kinematics_data.linear_speed;
                o_run_straight_turn_lin_speed <= i_kinematics_data.turn_right_90_linear_speed;
                if i_movement_done = '1' then
                    next_state <= STATE_TURN_SIDE_1;
                end if;

            when STATE_TURN_SIDE_1 =>
                o_run_side_en           <= '1';
                o_run_side_type         <= C_MOVE_RIGHT_90;
                o_run_side_turn_start_speed <= i_kinematics_data.turn_right_90_start_speed;
                o_run_side_turn_end_speed   <= i_kinematics_data.turn_right_90_end_speed;
                o_run_side_turn_linear_speed<= i_kinematics_data.turn_right_90_linear_speed;
                if i_movement_done = '1' then
                    next_state <= STATE_MOVE_STRAIGHT_2;
                end if;

            when STATE_MOVE_STRAIGHT_2 =>
                o_run_straight_en       <= '1';
                o_run_straight_dist_val <= s_straight_dist_2_3_4;
                o_run_straight_start_speed <= i_kinematics_data.turn_right_90_end_speed;
                o_run_straight_end_speed   <= i_kinematics_data.turn_right_90_start_speed;
                o_run_straight_cells_num   <= i_current_maze_columns - 2;
                o_run_straight_accel_en    <= '0'; -- false
                o_run_straight_lin_speed   <= i_kinematics_data.linear_speed;
                o_run_straight_turn_lin_speed <= i_kinematics_data.turn_right_90_linear_speed;
                if i_movement_done = '1' then
                    next_state <= STATE_TURN_SIDE_2;
                end if;

            when STATE_TURN_SIDE_2 =>
                o_run_side_en           <= '1';
                o_run_side_type         <= C_MOVE_RIGHT_90;
                o_run_side_turn_start_speed <= i_kinematics_data.turn_right_90_start_speed;
                o_run_side_turn_end_speed   <= i_kinematics_data.turn_right_90_end_speed;
                o_run_side_turn_linear_speed<= i_kinematics_data.turn_right_90_linear_speed;
                if i_movement_done = '1' then
                    next_state <= STATE_MOVE_STRAIGHT_3;
                end if;

            when STATE_MOVE_STRAIGHT_3 =>
                o_run_straight_en       <= '1';
                o_run_straight_dist_val <= s_straight_dist_2_3_4; -- Corrected from original C code
                o_run_straight_start_speed <= i_kinematics_data.turn_right_90_end_speed;
                o_run_straight_end_speed   <= i_kinematics_data.turn_right_90_start_speed;
                o_run_straight_cells_num   <= i_current_maze_rows - 2;
                o_run_straight_accel_en    <= '0'; -- false
                o_run_straight_lin_speed   <= i_kinematics_data.linear_speed;
                o_run_straight_turn_lin_speed <= i_kinematics_data.turn_right_90_linear_speed;
                if i_movement_done = '1' then
                    next_state <= STATE_TURN_SIDE_3;
                end if;

            when STATE_TURN_SIDE_3 =>
                o_run_side_en           <= '1';
                o_run_side_type         <= C_MOVE_RIGHT_90;
                o_run_side_turn_start_speed <= i_kinematics_data.turn_right_90_start_speed;
                o_run_side_turn_end_speed   <= i_kinematics_data.turn_right_90_end_speed;
                o_run_side_turn_linear_speed<= i_kinematics_data.turn_right_90_linear_speed;
                if i_movement_done = '1' then
                    next_state <= STATE_MOVE_STRAIGHT_4;
                end if;

            when STATE_MOVE_STRAIGHT_4 =>
                o_run_straight_en       <= '1';
                o_run_straight_dist_val <= s_straight_dist_2_3_4;
                o_run_straight_start_speed <= i_kinematics_data.turn_right_90_end_speed;
                o_run_straight_end_speed   <= i_kinematics_data.turn_right_90_start_speed;
                o_run_straight_cells_num   <= i_current_maze_columns - 2;
                o_run_straight_accel_en    <= '0'; -- false
                o_run_straight_lin_speed   <= i_kinematics_data.linear_speed;
                o_run_straight_turn_lin_speed <= i_kinematics_data.turn_right_90_linear_speed;
                if i_movement_done = '1' then
                    next_state <= STATE_TURN_SIDE_4;
                end if;

            when STATE_TURN_SIDE_4 =>
                o_run_side_en           <= '1';
                o_run_side_type         <= C_MOVE_RIGHT_90;
                o_run_side_turn_start_speed <= i_kinematics_data.turn_right_90_start_speed;
                o_run_side_turn_end_speed   <= i_kinematics_data.turn_right_90_end_speed;
                o_run_side_turn_linear_speed<= i_kinematics_data.turn_right_90_linear_speed;
                if i_movement_done = '1' then
                    next_state <= STATE_FINAL_STOP;
                end if;

            when STATE_FINAL_STOP =>
                o_move_back_stop_en <= '1';
                next_state <= STATE_RESET_RACE_FLAG;

            when STATE_RESET_RACE_FLAG =>
                o_set_race_started_flag <= '0';
                next_state <= STATE_DONE;

            when STATE_DONE =>
                next_state <= STATE_DONE;
                if i_start_timetrial = '1' then
                    next_state <= STATE_START_INIT;
                end if;

            when others =>
                next_state <= STATE_IDLE;
        end case;
    end process;

end architecture Behavioral;
