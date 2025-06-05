
-- robot_types_pkg.vhd
-- Paquete que define tipos de datos y constantes compartidas para el robot.
-- Es crucial que todos los valores flotantes de C se hayan convertido a enteros escalados (punto fijo).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all; -- Solo para cálculos de constantes en tiempo de diseño/síntesis

package robot_types_pkg is

    -- =========================================================================
    -- CONSTANTES GLOBALES Y FACTORES DE ESCALA
    -- Definir estos aquí para una gestión centralizada y consistente.
    -- =========================================================================
    -- Factor de escala para convertir 'float' de C a 'natural'/'signed' en VHDL.
    -- Multiplicar el valor flotante por este factor para obtener el entero.
    -- Ejemplo: 10.1633 * 1000 = 10163 (aproximado).
    constant C_FLOAT_TO_FIXED_SCALE_MM_PER_S : natural := 1000;
    -- Factor de escala para velocidades angulares (rad/s)
    -- Ejemplo: 13.460 rad/s * 100000 = 1346000
    constant C_FLOAT_TO_FIXED_SCALE_RAD_PER_S : natural := 100000;

    -- Constantes para el menú de configuración (from menu_configs.h)
    constant C_MODE_CALIBRATION          : natural := 0;
    constant C_MODE_DEBUG                : natural := 1;
    constant C_NUM_MODES                 : natural := 2;
    constant C_NUM_VALUES_CALIBRATION    : natural := 5;
    constant C_NUM_VALUES_DEBUG          : natural := 10;

    -- Constantes para el menú principal (from menu.h)
    constant C_MENU_RUN                  : natural := 0;
    constant C_MENU_CONFIG               : natural := 1;
    constant C_MENU_TYPES                : natural := 2; -- Number of menu types

    -- =========================================================================
    -- TIPOS ENUMERADOS (Extraídos de "move.h" y otros .h)
    -- =========================================================================
    -- Enum para tipos de movimiento
    type movement is (
        MOVE_NONE,
        MOVE_HOME,
        MOVE_START,
        MOVE_END,
        MOVE_FRONT,
        MOVE_LEFT,
        MOVE_RIGHT,
        MOVE_LEFT_90,
        MOVE_RIGHT_90,
        MOVE_LEFT_180,
        MOVE_RIGHT_180,
        MOVE_DIAGONAL,
        MOVE_LEFT_TO_45,
        MOVE_RIGHT_TO_45,
        MOVE_LEFT_TO_135,
        MOVE_RIGHT_TO_135,
        MOVE_LEFT_45_TO_45,
        MOVE_RIGHT_45_TO_45,
        MOVE_LEFT_FROM_45,
        MOVE_RIGHT_FROM_45,
        MOVE_LEFT_FROM_45_180,
        MOVE_RIGHT_FROM_45_180,
        MOVE_BACK,
        MOVE_BACK_WALL,
        MOVE_BACK_STOP
    );

    -- Enum para estrategias de velocidad
    type speed_strategy is (
        SPEED_EXPLORE,
        SPEED_NORMAL,
        SPEED_MEDIUM,
        SPEED_FAST,
        SPEED_SUPER,
        SPEED_HAKI
    );

    -- Enum para tipos de menú
    type menu_type is (
        MENU_RUN_TYPE,
        MENU_CONFIG_TYPE
    );

    -- Enum para modos de configuración del menú (from menu_configs.h)
    type menu_config_mode_type is (
        MODE_CALIBRATION,
        MODE_DEBUG
    );

    -- Enum para valores de calibración (from menu_configs.h, assumed context)
    type calibrate_value_type is (
        CALIBRATE_NONE,
        CALIBRATE_GYRO_Z,
        CALIBRATE_SIDE_SENSORS_OFFSET,
        CALIBRATE_FRONT_SENSORS,
        CALIBRATE_STORE_EEPROM
    );

    -- =========================================================================
    -- TIPOS RECORD (ESTRUCTURAS)
    -- =========================================================================

    -- Estructura para parámetros de aceleración lineal (equivalente a accel_params en C)
    -- Los valores de 'float' (como break_accel en C) se convierten a 'natural' escalados.
    type accel_params is record
        break_accel: natural; -- Aceleración de frenado (mm/s^2), escalado por C_FLOAT_TO_FIXED_SCALE_MM_PER_S
        accel_hard : natural; -- Aceleración fuerte (mm/s^2), escalado por C_FLOAT_TO_FIXED_SCALE_MM_PER_S
        speed_hard : natural; -- Velocidad a la que se aplica accel_hard (mm/s), no escalado
        accel_soft : natural; -- Aceleración suave (mm/s^2), escalado por C_FLOAT_TO_FIXED_SCALE_MM_PER_S
    end record;

    -- Estructura para parámetros de movimientos "in place" (equivalente a inplace_params en C)
    -- Los tiempos 't_accel' y 't_max' ya están en ms, no se escalan.
    type inplace_params is record
        start              : natural; -- En mm, escalado (no usado en C para estos movimientos específicos)
        end_val            : natural; -- En mm, escalado (no usado en C para estos movimientos específicos)
        linear_speed       : natural; -- Velocidad lineal (mm/s)
        angular_accel      : signed(20 downto 0); -- Aceleración angular (rad/s^2), escalado por C_FLOAT_TO_FIXED_SCALE_RAD_PER_S
        max_angular_speed  : signed(20 downto 0); -- Velocidad angular máxima (rad/s), escalado por C_FLOAT_TO_FIXED_SCALE_RAD_PER_S
        t_accel            : natural; -- Tiempo de aceleración en ms
        t_max              : natural; -- Tiempo a velocidad máxima en ms
        sign               : signed(0 to 0); -- Dirección del giro (-1 o 1)
    end record;

    -- Estructura para parámetros de giros en arco (equivalente a turn_params en C)
    -- 'start', 'end', 'transition', 'arc' son distancias (mm), escalado.
    -- 'max_angular_speed' es (rad/s), escalado.
    type turn_params is record
        start             : signed(15 downto 0); -- Distancia de inicio de giro (mm), escalado por C_FLOAT_TO_FIXED_SCALE_MM_PER_S
        end_val           : signed(15 downto 0); -- Distancia de fin de giro (mm), escalado por C_FLOAT_TO_FIXED_SCALE_MM_PER_S
        linear_speed      : natural; -- Velocidad lineal (mm/s)
        max_angular_speed : signed(20 downto 0); -- Velocidad angular máxima (rad/s), escalado por C_FLOAT_TO_FIXED_SCALE_RAD_PER_S
        transition        : natural; -- Distancia de transición (mm), escalado por C_FLOAT_TO_FIXED_SCALE_MM_PER_S
        arc               : natural; -- Longitud del arco (mm), escalado por C_FLOAT_TO_FIXED_SCALE_MM_PER_S
        sign              : signed(0 to 0); -- Dirección del giro (-1 o 1)
    end record;

    -- Estructura para los datos completos de cinemática (equivalente a struct kinematics en C)
    type kinematics_data is record
        linear_speed : natural; -- Velocidad lineal global (mm/s)
        linear_accel : accel_params;
        fan_speed    : natural; -- Velocidad del ventilador
        turns        : array(movement range MOVE_LEFT to MOVE_RIGHT_FROM_45_180) of turn_params; -- Array de parámetros de giro
    end record;

    -- Estado de detección de paredes (equivalente a struct walls en C)
    type walls_status is record
        front : std_logic;
        left  : std_logic;
        right : std_logic;
    end record;

    -- Tipo para array de distancias de sensores
    type sensor_distance_array_t is array (natural range <>) of natural;
    -- Tipo para array de secuencia de movimientos
    type movement_sequence_array_t is array (natural range <>) of movement;

    -- Tipo para valueConfig (array de int8_t en C)
    type value_config_array_t is array (menu_config_mode_type) of natural range 0 to C_NUM_VALUES_DEBUG; -- Rango suficiente

    -- =========================================================================
    -- CONSTANTES DE ARRAYS DE PARÁMETROS DE MOVIMIENTO
    -- Convertidos de los arrays estáticos de C a constantes VHDL
    -- =========================================================================

    -- Array para movimientos "in-place" (MOVE_BACK, MOVE_BACK_WALL, MOVE_BACK_STOP)
    -- Valores de 'angular_accel' y 'max_angular_speed' escalados por C_FLOAT_TO_FIXED_SCALE_RAD_PER_S
    constant C_TURNS_INPLACE : array (movement range MOVE_BACK to MOVE_BACK_STOP) of inplace_params := (
        MOVE_BACK => (
            start => 0, end_val => 0, linear_speed => 0,
            angular_accel => to_signed(integer(612.5 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21),
            max_angular_speed => to_signed(integer(13.460 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21),
            t_accel => 22, t_max => 209, sign => to_signed(-1, 1)
        ),
        MOVE_BACK_WALL => (
            start => 0, end_val => 0, linear_speed => 0,
            angular_accel => to_signed(integer(612.5 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21),
            max_angular_speed => to_signed(integer(13.460 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21),
            t_accel => 22, t_max => 209, sign => to_signed(-1, 1)
        ),
        MOVE_BACK_STOP => (
            start => 0, end_val => 0, linear_speed => 0,
            angular_accel => to_signed(integer(612.5 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21),
            max_angular_speed => to_signed(integer(13.460 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21),
            t_accel => 22, t_max => 209, sign => to_signed(-1, 1)
        )
    );

    -- Array para giros de exploración (MOVE_LEFT, MOVE_RIGHT)
    -- Valores 'start', 'end', 'transition', 'arc' escalados por C_FLOAT_TO_FIXED_SCALE_MM_PER_S
    -- Valores 'max_angular_speed' escalados por C_FLOAT_TO_FIXED_SCALE_RAD_PER_S
    constant C_TURNS_EXPLORE : array (movement range MOVE_LEFT to MOVE_RIGHT) of turn_params := (
        MOVE_LEFT => (
            start => to_signed(integer(10.1633 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16),
            end_val => to_signed(integer(10.1581 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16),
            linear_speed => 650,
            max_angular_speed => to_signed(integer(11.8182 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21),
            transition => natural(63.3458 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S),
            arc => natural(5.7460 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S),
            sign => to_signed(-1, 1)
        ),
        MOVE_RIGHT => (
            start => to_signed(integer(10.1633 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16),
            end_val => to_signed(integer(10.1581 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16),
            linear_speed => 650,
            max_angular_speed => to_signed(integer(11.8182 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21),
            transition => natural(63.3458 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S),
            arc => natural(5.7460 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S),
            sign => to_signed(1, 1)
        )
    );

    -- Array para giros normales
    constant C_TURNS_NORMAL : array (movement range MOVE_LEFT_90 to MOVE_RIGHT_FROM_45_180) of turn_params := (
        MOVE_LEFT_90 => (start => to_signed(integer(-24.1966 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-24.2030 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1000, max_angular_speed => to_signed(integer(11.1111 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3450 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(60.7300 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_90 => (start => to_signed(integer(-24.1966 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-24.2030 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1000, max_angular_speed => to_signed(integer(11.1111 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3450 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(60.7300 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_180 => (start => to_signed(integer(-44.9900 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-44.9900 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1000, max_angular_speed => to_signed(integer(11.1111 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3450 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(202.1000 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_180 => (start => to_signed(integer(-44.9900 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-44.9900 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1000, max_angular_speed => to_signed(integer(11.1111 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3450 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(202.1000 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_TO_45 => (start => to_signed(integer(-73.0659 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(54.2130 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1000, max_angular_speed => to_signed(integer(8.3333 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.6100 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_TO_45 => (start => to_signed(integer(-73.0659 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(54.2130 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1000, max_angular_speed => to_signed(integer(8.3333 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.6100 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_TO_135 => (start => to_signed(integer(-26.5865 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1000, max_angular_speed => to_signed(integer(13.4120 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_TO_135 => (start => to_signed(integer(-26.5865 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1000, max_angular_speed => to_signed(integer(13.4120 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_45_TO_45 => (start => to_signed(integer(39.0156 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(39.0130 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1000, max_angular_speed => to_signed(integer(15.7134 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3450 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(19.3200 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_45_TO_45 => (start => to_signed(integer(39.0156 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(39.0130 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1000, max_angular_speed => to_signed(integer(15.7134 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3450 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(19.3200 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_FROM_45 => (start => to_signed(integer(54.2133 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-73.0662 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1000, max_angular_speed => to_signed(integer(8.3333 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.6100 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_FROM_45 => (start => to_signed(integer(54.2133 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-73.0662 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1000, max_angular_speed => to_signed(integer(8.3333 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.6100 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_FROM_45_180 => (start => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-26.5867 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1000, max_angular_speed => to_signed(integer(13.4120 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_FROM_45_180 => (start => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-26.5867 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1000, max_angular_speed => to_signed(integer(13.4120 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1))
    );

    -- Array para giros medios
    constant C_TURNS_MEDIUM : array (movement range MOVE_LEFT_90 to MOVE_RIGHT_FROM_45_180) of turn_params := (
        MOVE_LEFT_90 => (start => to_signed(integer(-24.1966 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-24.1970 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1400, max_angular_speed => to_signed(integer(15.5556 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3500 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(60.7180 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_90 => (start => to_signed(integer(-24.1966 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-24.1970 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1400, max_angular_speed => to_signed(integer(15.5556 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3500 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(60.7180 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_180 => (start => to_signed(integer(-44.9860 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-44.9860 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1400, max_angular_speed => to_signed(integer(15.5556 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3500 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(202.0900 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_180 => (start => to_signed(integer(-44.9860 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-44.9860 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1400, max_angular_speed => to_signed(integer(15.5556 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3500 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(202.0900 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_TO_45 => (start => to_signed(integer(-73.0659 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(54.2130 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1400, max_angular_speed => to_signed(integer(11.6667 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3500 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.5940 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_TO_45 => (start => to_signed(integer(-73.0659 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(54.2130 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1400, max_angular_speed => to_signed(integer(11.6667 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3500 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.5940 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_TO_135 => (start => to_signed(integer(-26.5865 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1400, max_angular_speed => to_signed(integer(18.7768 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3500 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0320 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_TO_135 => (start => to_signed(integer(-26.5865 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1400, max_angular_speed => to_signed(integer(18.7768 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3500 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0320 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_45_TO_45 => (start => to_signed(integer(39.0156 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(39.0070 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1400, max_angular_speed => to_signed(integer(21.9987 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3500 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(19.3200 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_45_TO_45 => (start => to_signed(integer(39.0156 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(39.0070 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1400, max_angular_speed => to_signed(integer(21.9987 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3500 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(19.3200 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_FROM_45 => (start => to_signed(integer(54.2133 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-73.0662 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1400, max_angular_speed => to_signed(integer(11.6667 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3500 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.5940 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_FROM_45 => (start => to_signed(integer(54.2133 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-73.0662 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1400, max_angular_speed => to_signed(integer(11.6667 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3500 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.5940 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_FROM_45_180 => (start => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-26.5947 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1400, max_angular_speed => to_signed(integer(18.7768 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3500 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0320 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_FROM_45_180 => (start => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-26.5947 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1400, max_angular_speed => to_signed(integer(18.7768 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3500 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0320 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1))
    );

    -- Array para giros rápidos
    constant C_TURNS_FAST : array (movement range MOVE_LEFT_90 to MOVE_RIGHT_FROM_45_180) of turn_params := (
        MOVE_LEFT_90 => (start => to_signed(integer(-24.1966 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-24.2090 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1800, max_angular_speed => to_signed(integer(20.0000 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3510 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(60.7320 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_90 => (start => to_signed(integer(-24.1966 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-24.2090 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1800, max_angular_speed => to_signed(integer(20.0000 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3510 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(60.7320 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_180 => (start => to_signed(integer(-44.9820 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-44.9820 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1800, max_angular_speed => to_signed(integer(20.0000 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3510 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(202.1040 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_180 => (start => to_signed(integer(-44.9820 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-44.9820 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1800, max_angular_speed => to_signed(integer(20.0000 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3510 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(202.1040 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_TO_45 => (start => to_signed(integer(-73.0659 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(54.2010 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1800, max_angular_speed => to_signed(integer(15.0000 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3510 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.6080 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_TO_45 => (start => to_signed(integer(-73.0659 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(54.2010 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1800, max_angular_speed => to_signed(integer(15.0000 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3510 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.6080 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_TO_135 => (start => to_signed(integer(-26.5865 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1800, max_angular_speed => to_signed(integer(24.1416 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3510 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_TO_135 => (start => to_signed(integer(-26.5865 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1800, max_angular_speed => to_signed(integer(24.1416 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3510 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_45_TO_45 => (start => to_signed(integer(39.0156 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(39.0150 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1800, max_angular_speed => to_signed(integer(28.2841 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3420 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(19.3320 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_45_TO_45 => (start => to_signed(integer(39.0156 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(39.0150 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1800, max_angular_speed => to_signed(integer(28.2841 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3420 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(19.3320 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_FROM_45 => (start => to_signed(integer(54.2133 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-73.0782 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1800, max_angular_speed => to_signed(integer(15.0000 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3510 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.6080 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_FROM_45 => (start => to_signed(integer(54.2133 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-73.0782 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1800, max_angular_speed => to_signed(integer(15.0000 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3510 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.6080 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_FROM_45_180 => (start => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-26.6007 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1800, max_angular_speed => to_signed(integer(24.1416 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3510 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_FROM_45_180 => (start => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-26.6007 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 1800, max_angular_speed => to_signed(integer(24.1416 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3510 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1))
    );

    -- Array para giros super rápidos
    constant C_TURNS_SUPER : array (movement range MOVE_LEFT_90 to MOVE_RIGHT_FROM_45_180) of turn_params := (
        MOVE_LEFT_90 => (start => to_signed(integer(-24.1966 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-24.2110 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2200, max_angular_speed => to_signed(integer(24.4444 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3600 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(60.7200 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_90 => (start => to_signed(integer(-24.1966 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-24.2110 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2200, max_angular_speed => to_signed(integer(24.4444 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3600 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(60.7200 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_180 => (start => to_signed(integer(-44.9780 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-44.9780 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2200, max_angular_speed => to_signed(integer(24.4444 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3600 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(202.0920 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_180 => (start => to_signed(integer(-44.9780 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-44.9780 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2200, max_angular_speed => to_signed(integer(24.4444 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3600 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(202.0920 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_TO_45 => (start => to_signed(integer(-73.0659 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(54.1990 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2200, max_angular_speed => to_signed(integer(18.3333 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3600 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.5960 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_TO_45 => (start => to_signed(integer(-73.0659 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(54.1990 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2200, max_angular_speed => to_signed(integer(18.3333 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3600 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.5960 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_TO_135 => (start => to_signed(integer(-26.5865 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2200, max_angular_speed => to_signed(integer(29.5064 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3600 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0180 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_TO_135 => (start => to_signed(integer(-26.5865 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2200, max_angular_speed => to_signed(integer(29.5064 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3600 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0180 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_45_TO_45 => (start => to_signed(integer(39.0156 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(38.9990 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2200, max_angular_speed => to_signed(integer(34.5695 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3600 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(19.3160 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_45_TO_45 => (start => to_signed(integer(39.0156 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(38.9990 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2200, max_angular_speed => to_signed(integer(34.5695 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3600 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(19.3160 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_FROM_45 => (start => to_signed(integer(54.2133 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-73.0802 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2200, max_angular_speed => to_signed(integer(18.3333 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3600 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.5960 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_FROM_45 => (start => to_signed(integer(54.2133 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-73.0802 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2200, max_angular_speed => to_signed(integer(18.3333 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3600 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.5960 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_FROM_45_180 => (start => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-26.5927 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2200, max_angular_speed => to_signed(integer(29.5064 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3600 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0180 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_FROM_45_180 => (start => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-26.5927 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2200, max_angular_speed => to_signed(integer(29.5064 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3600 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0180 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1))
    );

    -- Array para giros Haki
    constant C_TURNS_HAKI : array (movement range MOVE_LEFT_90 to MOVE_RIGHT_FROM_45_180) of turn_params := (
        MOVE_LEFT_90 => (start => to_signed(integer(-24.1966 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-24.2010 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2600, max_angular_speed => to_signed(integer(28.8889 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3620 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(60.7100 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_90 => (start => to_signed(integer(-24.1966 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-24.2010 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2600, max_angular_speed => to_signed(integer(28.8889 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3620 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(60.7100 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_180 => (start => to_signed(integer(-44.9740 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-44.9740 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2600, max_angular_speed => to_signed(integer(28.8889 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3620 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(202.0980 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_180 => (start => to_signed(integer(-44.9740 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-44.9740 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2600, max_angular_speed => to_signed(integer(28.8889 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3620 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(202.0980 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_TO_45 => (start => to_signed(integer(-73.0659 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(54.1970 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2600, max_angular_speed => to_signed(integer(21.6667 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3620 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.5980 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_TO_45 => (start => to_signed(integer(-73.0659 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(54.1970 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2600, max_angular_speed => to_signed(integer(21.6667 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3620 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.5980 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_TO_135 => (start => to_signed(integer(-26.5865 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2600, max_angular_speed => to_signed(integer(34.8712 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3620 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0300 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_TO_135 => (start => to_signed(integer(-26.5865 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2600, max_angular_speed => to_signed(integer(34.8712 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3620 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0300 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_45_TO_45 => (start => to_signed(integer(38.2276 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(38.2122 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2600, max_angular_speed => to_signed(integer(40.8548 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(62.0360 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(23.1400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_45_TO_45 => (start => to_signed(integer(38.2276 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(38.2122 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2600, max_angular_speed => to_signed(integer(40.8548 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(62.0360 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(23.1400 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_FROM_45 => (start => to_signed(integer(54.2133 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-73.0822 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2600, max_angular_speed => to_signed(integer(21.6667 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3620 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.5980 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_FROM_45 => (start => to_signed(integer(54.2133 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-73.0822 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2600, max_angular_speed => to_signed(integer(21.6667 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3620 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(13.5980 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1)),
        MOVE_LEFT_FROM_45_180 => (start => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-26.6047 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2600, max_angular_speed => to_signed(integer(34.8712 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3620 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0300 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(-1, 1)),
        MOVE_RIGHT_FROM_45_180 => (start => to_signed(integer(47.9719 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), end_val => to_signed(integer(-26.6047 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), 16), linear_speed => 2600, max_angular_speed => to_signed(integer(34.8712 * C_FLOAT_TO_FIXED_SCALE_RAD_PER_S), 21), transition => natural(63.3620 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), arc => natural(95.0300 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S), sign => to_signed(1, 1))
    );

    -- Array principal de configuraciones de cinemática (kinematics_settings en C)
    constant C_KINEMATICS_SETTINGS : array (speed_strategy) of kinematics_data := (
        SPEED_EXPLORE => (
            linear_speed => 650,
            linear_accel => (
                break_accel => natural(5000 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S),
                accel_hard => natural(5000 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S),
                speed_hard => 0,
                accel_soft => 0
            ),
            fan_speed => 30,
            -- Asigna C_TURNS_EXPLORE a los índices correspondientes, y el resto 'others'
            turns => (
                MOVE_LEFT  => C_TURNS_EXPLORE(MOVE_LEFT),
                MOVE_RIGHT => C_TURNS_EXPLORE(MOVE_RIGHT),
                others => (start => (others => '0'), end_val => (others => '0'), linear_speed => 0, max_angular_speed => (others => '0'), transition => 0, arc => 0, sign => (others => '0'))
            )
        ),
        SPEED_NORMAL => (
            linear_speed => 2000,
            linear_accel => (
                break_accel => natural(5000 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S),
                accel_hard => natural(5000 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S),
                speed_hard => 0,
                accel_soft => 0
            ),
            fan_speed => 60,
            turns => C_TURNS_NORMAL
        ),
        SPEED_MEDIUM => (
            linear_speed => 3000,
            linear_accel => (
                break_accel => natural(10000 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S),
                accel_hard => natural(10000 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S),
                speed_hard => 0,
                accel_soft => 0
            ),
            fan_speed => 65,
            turns => C_TURNS_MEDIUM
        ),
        SPEED_FAST => (
            linear_speed => 4000,
            linear_accel => (
                break_accel => natural(15000 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S),
                accel_hard => natural(15000 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S),
                speed_hard => 2500,
                accel_soft => natural(10000 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S)
            ),
            fan_speed => 75,
            turns => C_TURNS_FAST
        ),
        SPEED_SUPER => (
            linear_speed => 5000,
            linear_accel => (
                break_accel => natural(20000 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S),
                accel_hard => natural(20000 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S),
                speed_hard => 3500,
                accel_soft => natural(15000 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S)
            ),
            fan_speed => 85,
            turns => C_TURNS_SUPER
        ),
        SPEED_HAKI => (
            linear_speed => 6000,
            linear_accel => (
                break_accel => natural(25000 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S),
                accel_hard => natural(25000 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S),
                speed_hard => 4000,
                accel_soft => natural(18000 * C_FLOAT_TO_FIXED_SCALE_MM_PER_S)
            ),
            fan_speed => 90,
            turns => C_TURNS_HAKI
        )
    );

end package robot_types_pkg;


package body robot_types_pkg is
    -- Puedes incluir funciones o procedimientos aquí si son necesarios
    -- para inicializar o manipular los tipos y constantes.
    -- Por ejemplo, funciones para la conversión de float a punto fijo en tiempo de ejecución
    -- si no se hace directamente en la declaración de constantes.
end package body robot_types_pkg;
