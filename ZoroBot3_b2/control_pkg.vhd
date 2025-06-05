
-- control_pkg.vhd
-- Package for shared types and constants for the control module

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.float_pkg.all; -- Required for float operations

package control_pkg is

    -- Kinematics structure (from C code)
    type t_linear_accel is record
        accel_soft : signed(15 downto 0); -- int16_t accel_soft
        speed_hard : signed(15 downto 0); -- int16_t speed_hard
        accel_hard : signed(15 downto 0); -- int16_t accel_hard
        break_accel: signed(15 downto 0); -- int16_t break_accel
    end record t_linear_accel;

    type t_kinematics is record
        linear_accel : t_linear_accel;
        -- Add other kinematics fields if they exist in the C get_kinematics() struct
    end record t_kinematics;

    -- Constants (adjust values as per your robot's configuration)
    constant MOTORES_MAX_PWM          : signed(15 downto 0) := to_signed(255, 16); -- Max PWM value
    constant CONTROL_FREQUENCY_HZ     : natural := 1000;                          -- C code: CONTROL_FREQUENCY_HZ (e.g., 1000 Hz for 1ms loop)
    constant SENSOR_FRONT_DETECTION_START: signed(15 downto 0) := to_signed(50, 16); -- Example value, adjust
    constant SENSOR_START_MIN_MS      : unsigned(31 downto 0) := to_unsigned(100, 32); -- Example value, adjust

    -- PID Gains (adjust these values based on your robot's tuning)
    constant KP_LINEAR                : float32 := to_float32(0.01);
    constant KD_LINEAR                : float32 := to_float32(0.001);

    constant KP_ANGULAR               : float32 := to_float32(0.1);
    constant KD_ANGULAR               : float32 := to_float32(0.01);

    constant KP_SIDE_SENSORS          : float32 := to_float32(0.05);
    constant KI_SIDE_SENSORS          : float32 := to_float32(0.0001);
    constant KD_SIDE_SENSORS          : float32 := to_float32(0.005);

    constant KP_FRONT_SENSORS         : float32 := to_float32(0.03);
    constant KI_FRONT_SENSORS         : float32 := to_float32(0.00005);

    constant KP_FRONT_DIAGONAL_SENSORS: float32 := to_float32(0.02);
    constant KI_FRONT_DIAGONAL_SENSORS: float32 := to_float32(0.00003);
    constant KD_FRONT_DIAGONAL_SENSORS: float32 := to_float32(0.002);

    -- Sensor IDs (re-using from debug_pkg or defining if debug_pkg is not shared)
    constant SENSOR_FRONT_LEFT_WALL_ID : natural := 1;
    constant SENSOR_FRONT_RIGHT_WALL_ID: natural := 2;
    constant SENSOR_SIDE_LEFT_WALL_ID  : natural := 0;
    constant SENSOR_SIDE_RIGHT_WALL_ID : natural := 3;

end package control_pkg;
