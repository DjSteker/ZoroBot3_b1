
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

    -- Constantes para Macroarray (from macroarray.h)
    constant C_MACROARRAY_LENGTH         : natural := 100; -- Assuming a length for the circular buffer
    constant C_MACROARRAY_MAX_LABELS     : natural := 20;  -- Max 'size' parameter

    -- Scaling factor for float conversion in macroarray_print (value / 100.0)
    constant C_MACROARRAY_FLOAT_SCALE    : natural := 100;

    -- Constantes para LSM6DSR (from lsm6dsr.h)
    constant C_PI_FIXED_POINT            : natural := 314159265; -- PI * 10^8
    constant C_PI_FIXED_POINT_SCALE      : natural := 100000000; -- Scale for PI
    constant C_MPU_DPS_TO_RADPS_FIXED    : natural := natural(real(C_PI_FIXED_POINT) / real(180) / real(C_PI_FIXED_POINT_SCALE) * real(1000000)); -- Scaled by 10^6
    constant C_MPU_DPS_TO_RADPS_FIXED_SCALE : natural := 1000000; -- Scale for MPU_DPS_TO_RADPS
    constant C_GYRO_SENSITIVITY_2000DPS_FIXED : natural := 70 * 100000; -- Scaled by 10^5
    constant C_GYRO_SENSITIVITY_4000DPS_FIXED : natural := 140 * 100000; -- Scaled by 10^5
    constant C_GYRO_SENSITIVITY_FIXED_SCALE : natural := 100000; -- Scale for sensitivities
    constant C_SYSTICK_FREQUENCY_HZ      : natural := 1000; -- Assuming 1000Hz for system tick

    -- =========================================================================
    -- FIXED-POINT ARITHMETIC CONFIGURATION (for encoder calculations)
    -- =========================================================================
    -- Number of fractional bits for fixed-point representation
    constant C_FIXED_POINT_FRAC_BITS : natural := 16;
    -- Scaling factor for converting float to fixed-point integer (2^16 = 65536)
    constant C_FIXED_POINT_SCALE     : integer := 2**C_FIXED_POINT_FRAC_BITS;

    -- Type for signed fixed-point numbers (e.g., 32 bits total for Q16.16)
    subtype T_FIXED_POINT is signed(31 downto 0);
    -- A simple function to convert an integer to fixed point
    function to_fixed(val : integer) return T_FIXED_POINT;

    -- =========================================================================
    -- ENCODER CONSTANTS (from encoders.h context, assuming typical values)
    -- =========================================================================
    constant C_WHEEL_DIAMETER_MM        : real := 32.0; -- Example wheel diameter in mm
    constant C_ENCODER_TICKS_PER_REV    : natural := 1440; -- Example encoder ticks per revolution (e.g. 360 CPR * 4)

    -- MICROMETERS_PER_TICK = (PI * WHEEL_DIAMETER_MM) / ENCODER_TICKS_PER_REVOLUTION * 1000.0f
    constant C_MICROMETERS_PER_TICK_FP : T_FIXED_POINT := to_fixed(integer(round(math_pi * C_WHEEL_DIAMETER_MM / real(C_ENCODER_TICKS_PER_REV) * 1000.0 * real(C_FIXED_POINT_SCALE))));
    constant C_MICROMETERS_PER_MILLIMETER : natural := 1000; -- 1000 micrometers in 1 millimeter

    -- WHEELS_SEPARATION in millimeters
    constant C_WHEELS_SEPARATION_MM     : real := 80.0; -- Example wheel separation in mm
    constant C_WHEELS_SEPARATION_MM_FP  : T_FIXED_POINT := to_fixed(integer(round(C_WHEELS_SEPARATION_MM * real(C_FIXED_POINT_SCALE))));

    -- MILLIMETERS_PER_METER (used in C angular speed calculation, here effectively 1000)
    constant C_MILLIMETERS_PER_METER    : natural := 1000;


    -- =========================================================================
    -- EEPROM CONSTANTS
    -- =========================================================================
    constant DATA_LENGTH        : natural := 256; -- Total size of eeprom_data array (int16_t)
    constant EEPROM_BASE_ADDRESS: natural := 0;  -- Conceptual base address for internal RAM
    constant EEPROM_SECTOR      : natural := 0;  -- Conceptual sector for internal RAM (not directly used for internal RAM)

    -- Data indices for EEPROM (from C code context, example values)
    constant DATA_INDEX_MAZE    : natural := 50; -- Example index for maze data
    constant DATA_INDEX_LSM6DSR : natural := 0;  -- Example index for LSM6DSR data
    constant DATA_INDEX_SENSORS : natural := 10; -- Example index for sensor data
    constant DATA_INDEX_MENU    : natural := 20; -- Example index for menu data
    constant DATA_INDEX_RC5     : natural := 30; -- Example index for RC5 data


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

    -- Tipo para almacenar los datos en el circular buffer (int16_t en C)
    type macroarray_data_t is array (natural range 0 to C_MACROARRAY_LENGTH-1) of signed(15 downto 0);
    -- Tipo para almacenar los índices de las etiquetas (uint8_t en C)
    type macroarray_label_indices_t is array (natural range 0 to C_MACROARRAY_MAX_LABELS-1) of natural range 0 to C_MACROARRAY_MAX_LABELS-1;

    -- Tipo para datos de EEPROM (int16_t en C)
    type eeprom_data_t is array (natural range 0 to DATA_LENGTH-1) of signed(15 downto 0);

    -- =========================================================================
    -- RECORD TYPE FOR CELL WEIGHTS
    -- =========================================================================
    -- Equivalent to struct cell_weigth in C
    type cell_weigth is record
        speed      : natural range 0 to 65535; -- uint16_t
        time       : T_FIXED_POINT;          -- float (fixed-point)
        total_time : T_FIXED_POINT;          -- float (fixed-point)
        penalty    : T_FIXED_POINT;          -- float (fixed-point)
    end record;

    -- Type for the array of cell_weights
    -- Assuming a maximum number of cells for the floodfill table, e.g., 256
    constant C_MAX_FLOODFILL_CELLS : natural := 256;
    type T_CELL_WEIGHT_ARRAY is array (natural range 0 to C_MAX_FLOODFILL_CELLS-1) of cell_weigth;

    -- =========================================================================
    -- CONFIGURACIÓN DEL LABERINTO (DEBE SER AJUSTADA A TU MAZE REAL)
    -- =========================================================================
    constant MAZE_ROWS      : natural := 6;  -- Ejemplo: 6 filas
    constant MAZE_COLUMNS   : natural := 6;  -- Ejemplo: 6 columnas
    constant MAZE_CELLS     : natural := MAZE_ROWS * MAZE_COLUMNS; -- Número total de celdas
    -- Valor máximo para distancias de floodfill no alcanzadas
    constant MAZE_MAX_DISTANCE_FP : T_FIXED_POINT := to_fixed(9999); -- Un valor grande en punto fijo

    -- =========================================================================
    -- BITS DE PAREDES Y ESTADO DE CELDA
    -- =========================================================================
    constant EAST_BIT       : integer := 1;  -- 0b0001
    constant SOUTH_BIT      : integer := 2;  -- 0b0010
    constant WEST_BIT       : integer := 4;  -- 0b0100
    constant NORTH_BIT      : integer := 8;  -- 0b1000
    constant VISITED_BIT    : integer := 16; -- 0b10000

    -- =========================================================================
    -- ENUMERACIONES DE DIRECCIÓN Y PASO
    -- =========================================================================
    type compass_direction is (
        NORTH, NORTH_EAST, EAST, SOUTH_EAST,
        SOUTH, SOUTH_WEST, WEST, NORTH_WEST,
        TARGET, NONE_DIRECTION
    );

    type step_direction is (
        FRONT, LEFT, RIGHT, BACK,
        -- TARGET se usa para direcciones de floodfill, no de paso directo.
        NONE_STEP
    );

    -- =========================================================================
    -- ESTRUCTURAS DE DATOS DE COLAS Y PILAS
    -- =========================================================================
    -- Tipo para las entradas de la cola del Floodfill
    type queue_cell is record
        cell      : natural range 0 to MAZE_CELLS - 1;
        direction : compass_direction;
        last_step : compass_direction; -- Reemplazando step_direction por compass_direction para consistencia con C
        count     : natural;
    end record;

    -- Tamaño máximo de la cola/pila, ajusta según el tamaño de tu laberinto
    constant C_MAX_QUEUE_SIZE : natural := MAZE_CELLS * 2; -- Aproximación

    type T_CELLS_QUEUE_ARRAY is array (0 to C_MAX_QUEUE_SIZE - 1) of queue_cell;
    type cells_queue_t is record
        head  : natural range 0 to C_MAX_QUEUE_SIZE;
        tail  : natural range 0 to C_MAX_QUEUE_SIZE;
        queue : T_CELLS_QUEUE_ARRAY;
    end record;

    type T_CELLS_STACK_ARRAY is array (0 to MAZE_CELLS - 1) of natural range 0 to MAZE_CELLS - 1;
    type cells_stack_t is record
        size  : natural range 0 to MAZE_CELLS;
        stack : T_CELLS_STACK_ARRAY;
    end record;

    -- Valores de dirección para cálculos (en lugar de int8_t en C)
    type compass_direction_values_t is record
        EAST  : integer;
        SOUTH : integer;
        WEST  : integer;
        NORTH : integer;
    end record;

    -- Tipo para la tabla del laberinto (int16_t en C)
    type T_MAZE_ARRAY is array (0 to MAZE_CELLS - 1) of integer range -32768 to 32767;

    -- Tipo para la tabla de Floodfill (float en C, aquí T_FIXED_POINT)
    type T_FLOODFILL_ARRAY is array (0 to MAZE_CELLS - 1) of T_FIXED_POINT;

    -- Tipo para la secuencia de movimientos de carrera
    type T_RUN_SEQUENCE_MOVEMENTS_ARRAY is array (0 to MAZE_CELLS + 2) of movement;


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

    -- =========================================================================
    -- ROM PARA ETIQUETAS DE MACROARRAY (ASCII)
    -- Cada elemento es un std_logic_vector que representa una cadena ASCII.
    -- La longitud de cada string debe ser consistente o gestionada por una longitud variable.
    -- Aquí, definimos algunas etiquetas de ejemplo para ilustrar.
    -- Las etiquetas deben ser pre-procesadas (ej. con Python/script) a un formato VHDL.
    -- "distance", "cells", "current_cell", "end_offset", "angular_speed", "millimeters"
    -- Las etiquetas se representan como arrays de std_logic_vector(7 downto 0) (ASCII).
    -- La longitud máxima de la etiqueta es un factor a considerar.
    -- El valor en la ROM sería la cadena ASCII codificada.
    -- La ROM sería un array de std_logic_vector o strings.

    -- Battery monitoring constants (Add these if not already present)
    constant AUX_BATTERY_ID                 : natural   := 4; -- Example ID, adjust as per your system
    constant ADC_BITS                       : natural   := 12; -- Example ADC resolution (e.g., 12-bit ADC)
    constant ADC_VREF_MV                    : natural   := 3300; -- Example ADC reference voltage in mV (e.g., 3.3V)
    constant ADC_LSB_FP                     : integer   := 8;  -- Fractional bits for ADC_LSB_VHDL (e.g., 8 means value * 2^-8)
    constant ADC_LSB_VHDL                   : unsigned(ADC_BITS+ADC_LSB_FP-1 downto 0) := to_unsigned(ADC_VREF_MV * 2**ADC_LSB_FP / (2**ADC_BITS), ADC_BITS+ADC_LSB_FP);
                                                               -- Represents ADC_VREF_MV / (2^ADC_BITS) in fixed point (mV per LSB)

    -- Example voltage divider factor. If your C code's VOLT_DIV_FACTOR is a float like 11.0f,
    -- then use its fixed-point equivalent. Assuming 11.0 for now, scaled by 8 fractional bits.
    constant VOLT_DIV_FACTOR_FP             : integer   := 8;
    constant VOLT_DIV_FACTOR_VHDL           : unsigned(15 downto 0) := to_unsigned(11 * 2**VOLT_DIV_FACTOR_FP, 16);

    -- Battery voltage limits in millivolts (integers for fixed-point math)
    -- Multiplied by 1000 to work with mV, then scaled up by fixed-point factor
    -- Assuming a common fractional bit count for all voltage values, e.g., 8
    constant BATTERY_VOLTAGE_FP             : integer   := 8; -- Fractional bits for voltage values

    constant BATTERY_3S_LOW_LIMIT_VOLTAGE_MV_FP : unsigned(23 downto 0) := to_unsigned(9900 * 2**BATTERY_VOLTAGE_FP, 24); -- 9.9V * 1000
    constant BATTERY_3S_HIGH_LIMIT_VOLTAGE_MV_FP: unsigned(23 downto 0) := to_unsigned(12600 * 2**BATTERY_VOLTAGE_FP, 24); -- 12.6V * 1000

    constant BATTERY_2S_LOW_LIMIT_VOLTAGE_MV_FP : unsigned(23 downto 0) := to_unsigned(6600 * 2**BATTERY_VOLTAGE_FP, 24); -- 6.6V * 1000
    constant BATTERY_2S_HIGH_LIMIT_VOLTAGE_MV_FP: unsigned(23 downto 0) := to_unsigned(8400 * 2**BATTERY_VOLTAGE_FP, 24); -- 8.4V * 1000

    -- Low-pass filter alpha value. Represent as an integer, scaled.
    -- E.g., if alpha is 0.1f, and we use 8 fractional bits, 0.1 * 2^8 = 25.6, so use 26.
    constant BATTERY_LPF_ALPHA_FP           : integer   := 8;
    constant BATTERY_LPF_ALPHA_VHDL         : unsigned(BATTERY_LPF_ALPHA_FP-1 downto 0) := to_unsigned(round(0.1 * real(2**BATTERY_LPF_ALPHA_FP)), BATTERY_LPF_ALPHA_FP);
    -- 1 - alpha: (1 - 0.1) * 2^8 = 0.9 * 256 = 230.4, use 230
    constant BATTERY_LPF_ONE_MINUS_ALPHA_VHDL: unsigned(BATTERY_LPF_ALPHA_FP-1 downto 0) := to_unsigned(round(0.9 * real(2**BATTERY_LPF_ALPHA_FP)), BATTERY_LPF_ALPHA_FP);


    -- Explore algorithm types (from menu_run_get_explore_algorithm)
    type explore_algorithm_type is (
        EXPLORE_HANDWALL,
        EXPLORE_FLOODFILL,
        EXPLORE_TIME_TRIAL,
        EXPLORE_NONE -- Default or error state
    );

    -- GPIO Constants (assuming simple bit positions or IDs)
    constant GPIOB_13_ID : natural := 13;
    constant GPIOB_15_ID : natural := 15;

    -- AUX IDs (re-confirming if they are in robot_types_pkg already or need adding)
    constant AUX_CURRENT_LEFT_ID: natural := 5; -- Example ID, adjust
    constant AUX_CURRENT_RIGHT_ID: natural := 6; -- Example ID, adjust
    constant AUX_MENU_BTN_ID    : natural := 3; -- Example ID, adjust

    -- Sensor IDs (re-confirming)
    constant SENSOR_FRONT_LEFT_WALL_ID : natural := 1;
    constant SENSOR_FRONT_RIGHT_WALL_ID: natural := 2;

    -- Print Message IDs (additional for main.c printf)
    constant MSG_ID_AUX_RAW_FMT : natural := 20; -- For "BA: %4d CI: %4d CD: %4d BO: %4d\n"
    constant MSG_ID_MPU_FMT     : natural := 21;
    constant MSG_ID_MPU_ZRAW_FMT: natural := 22;
    constant MSG_ID_SENSORS_RAW_MAIN_FMT : natural := 23;
    constant MSG_ID_ENCODERS_MAIN_FMT : natural := 24;
    constant MSG_ID_BATTERY_VOLTAGE_FMT : natural := 25;
    constant MSG_ID_HINVERNADERO_INIT_MAIN : natural := 26; -- For setup() initial print
    constant MSG_ID_INIT_DONE_MAIN : natural := 27; -- For setup() initial print

end package robot_types_pkg;


package body robot_types_pkg is
    function to_fixed(val : integer) return T_FIXED_POINT is
    begin
        -- Resize to 32 bits and then multiply by the scale factor
        return resize(to_signed(val, 32) * C_FIXED_POINT_SCALE, 32);
    end function;
end package body robot_types_pkg;
