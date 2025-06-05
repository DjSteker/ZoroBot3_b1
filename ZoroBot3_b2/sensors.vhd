
-- Librerías IEEE para tipos estándar y numéricos
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all; -- SOLO para cálculos de constantes en síntesis, NO para lógica de ejecución

-- !!! IMPORTANTE: REQUERIMIENTOS CRÍTICOS PARA LA SÍNTESIS !!!
-- 1. ARITMÉTICA DE PUNTO FIJO: Todos los cálculos con 'float' en C
--    (especialmente en 'update_sensors_magics' y las calibraciones)
--    DEBEN ser convertidos a operaciones de punto fijo (enteros escalados)
--    para ser implementados eficientemente en hardware.
--    Esto implica elegir un factor de escalado (ej. 1000 o 100000) y
--    manejar las divisiones/multiplicaciones de enteros con ese escalado.
-- 2. ADC: Necesitarás un módulo ADC VHDL que controle la conversión
--    y proporcione los datos digitales.
-- 3. GPIO: Necesitarás un módulo GPIO VHDL que reciba comandos de encendido/apagado.
-- 4. EEPROM: Necesitarás un módulo VHDL para la interfaz EEPROM/Flash.
-- 5. Frecuencia de reloj: El 'sm_emitter_adc' necesita un reloj base para su operación.
-- 6. Constantes: Todas las constantes (CELL_DIMENSION, ROBOT_FRONT_LENGTH, etc.)
--    deben ser definidas y sus valores ajustados a la escala de punto fijo.

-- --- Constantes y Definiciones ---
-- Estos deben ser consistentes con tus definiciones en C y tu estrategia de punto fijo
constant C_NUM_AUX_ADC_CHANNELS : natural := 4;
constant C_NUM_SENSORES         : natural := 4;
constant C_ADC_RESOLUTION       : natural := 4096; -- Para un ADC de 12 bits (0 a 4095)
constant C_LOG_LINEARIZATION_TABLE_STEP : natural := 4;
constant C_LOG_LINEARIZATION_TABLE_SIZE : natural := C_ADC_RESOLUTION / C_LOG_LINEARIZATION_TABLE_STEP; -- 4096 / 4 = 1024

-- IDs de sensores (deben coincidir con los enums de C si los hay)
constant SENSOR_FRONT_LEFT_WALL_ID  : natural := 0;
constant SENSOR_FRONT_RIGHT_WALL_ID : natural := 1;
constant SENSOR_SIDE_LEFT_WALL_ID   : natural := 2;
constant SENSOR_SIDE_RIGHT_WALL_ID  : natural := 3;

-- Constantes para detección de paredes (ajusta a tu escala de punto fijo)
constant C_SENSOR_SIDE_DETECTION  : natural := 90;  -- Ejemplo, ajusta
constant C_SENSOR_FRONT_DETECTION : natural := 90;  -- Ejemplo, ajusta
constant C_MIDDLE_MAZE_DISTANCE   : natural := 200; -- Ejemplo, ajusta
constant C_ROBOT_FRONT_LENGTH     : natural := 50;  -- Ejemplo, ajusta
constant C_ROBOT_MIDDLE_WIDTH     : natural := 50;  -- Ejemplo, ajusta
constant C_WALL_WIDTH             : natural := 12;  -- Ejemplo, ajusta
constant C_CELL_DIMENSION         : natural := 180; -- Ejemplo, ajusta

-- Rangos para el punto fijo (ej. 1000 unidades = 1 mm)
-- Si estás usando punto fijo, define la resolución (bits después del punto decimal)
-- Por ejemplo, Q16.16 significa 16 bits enteros, 16 bits fraccionarios.
-- Para simplificar, aquí usaremos 'natural' o 'signed' con valores escalados.
-- Si necesitas un tipo de punto fijo real, usarías una librería como 'fixed_pkg'.

-- Tipo para datos de calibración de distancia (simula la estructura C)
-- Los valores 'a', 'b', 'c' son floats en C. Aquí deben ser punto fijo (ej. enteros escalados).
-- Si 'a' = 2.881 y escala es 1000, entonces 'a_scaled' = 2881.
type calibration_data_t is record
    a_scaled : signed(15 downto 0); -- Ejemplo: bits para a, b, c
    b_scaled : signed(15 downto 0);
    c_scaled : signed(15 downto 0);
end record;
-- Array de calibraciones (ROM)
type calibration_rom_t is array (0 to C_NUM_SENSORES - 1) of calibration_data_t;

-- !!! DEFINE LOS VALORES DE CALIBRACIÓN EN PUNTO FIJO PARA TU ROBOT ESPECÍFICO !!!
-- Ejemplo para ZOROBOT3_A con una escala de 1000 (1 = 1000 unidades)
-- 2.881 -> 2881, 0.333 -> 333, -3.307 -> -3307
constant C_SENSORS_CALIBRATIONS : calibration_rom_t := (
    -- SENSOR_FRONT_LEFT_WALL_ID (0)
    (a_scaled => to_signed(2881, 16), b_scaled => to_signed(333, 16), c_scaled => to_signed(-3307, 16)),
    -- SENSOR_FRONT_RIGHT_WALL_ID (1)
    (a_scaled => to_signed(2866, 16), b_scaled => to_signed(333, 16), c_scaled => to_signed(2251, 16)),
    -- SENSOR_SIDE_LEFT_WALL_ID (2)
    (a_scaled => to_signed(2357, 16), b_scaled => to_signed(297, 16), c_scaled => to_signed(-50430, 16)), -- Ajusta a tu escala
    -- SENSOR_SIDE_RIGHT_WALL_ID (3)
    (a_scaled => to_signed(2399, 16), b_scaled => to_signed(307, 16), c_scaled => to_signed(-4694, 16))
);


-- Tabla de linealización (ROM)
-- Los valores de ln_lookup son floats. Para VHDL, deben ser enteros escalados.
-- Si usas una escala de 1000, 1.3863 se convierte en 1386.
-- La resolución de este array debe ser al menos 12 bits para cubrir hasta 4095
type ln_lookup_rom_t is array (0 to C_LOG_LINEARIZATION_TABLE_SIZE - 1) of signed(12 downto 0); -- Asumiendo 12 bits para los valores logarítmicos escalados

-- !!! DEBES CONVERTIR TUS VALORES 'float' A 'signed' SCALED INTEGERS !!!
constant C_LN_LOOKUP_TABLE : ln_lookup_rom_t := (
    -- Convertir cada valor float a un entero escalado (ej. *1000)
    to_signed(1000, 13), to_signed(1386, 13), to_signed(2079, 13), to_signed(2485, 13),
    -- ... continua con todos los 1024 valores de tu tabla, convertidos
    -- Este es un EJEMPLO, necesitas hacer la conversión completa y precisa.
    others => (others => '0') -- Rellena el resto con ceros por simplicidad aquí
);


--------------------------------------------------------------------------------
-- Entidad: sensors_controller
-- Descripción: Controla los emisores de los sensores, lee los ADCs asociados,
--              y calcula las distancias e información de pared.
--              Contiene la FSM para 'sm_emitter_adc' y la lógica para 'update_sensors_magics'.
--
-- Entradas:
--   i_clk               : Señal de reloj principal del sistema (ej. 168MHz)
--   i_reset             : Señal de reset sincronizado (activo alto)
--   i_emitter_adc_tick  : Pulso que activa la FSM de 'sm_emitter_adc' (ej. cada 16kHz)
--   i_update_magics_tick: Pulso que activa la lógica de 'update_sensors_magics' (ej. cada 1ms)
--   i_sensors_enabled_cmd: Habilitación de sensores desde el exterior (ej. 'set_sensors_enabled')
--   i_adc_data_in       : Datos de lectura del ADC (desde el módulo ADC)
--   i_eeprom_data_in    : Datos de compensación de EEPROM (desde el módulo EEPROM)
--   i_eeprom_read_en    : Pulso para cargar datos de EEPROM
--
-- Salidas:
--   o_gpio_emitter_out  : Salidas a los pines GPIO de los emisores (ej. GPIOA0, GPIOA1, GPIOA2, GPIOA3)
--   o_adc_read_en       : Habilita el módulo ADC para iniciar una conversión
--   o_adc_channel_select: Selecciona el canal del ADC a leer
--   o_sensors_distance  : Distancias calculadas de los sensores (array)
--   o_walls_status      : Estado de detección de paredes (front, left, right)
--   o_side_error        : Error de sensores laterales para control de dirección
--   o_front_angle_error : Error de ángulo de sensores frontales
--   o_led_info_control  : Salida a LEDs de información (para 'update_side_sensors_leds')
--------------------------------------------------------------------------------
entity sensors_controller is
    port (
        i_clk                 : in  std_logic;
        i_reset               : in  std_logic;
        i_emitter_adc_tick    : in  std_logic; -- Pulso para sm_emitter_adc (ej. 16kHz)
        i_update_magics_tick  : in  std_logic; -- Pulso para update_sensors_magics (ej. 1ms)
        i_sensors_enabled_cmd : in  std_logic; -- Entrada para set_sensors_enabled()
        i_adc_data_in         : in  std_logic_vector(11 downto 0); -- Salida de datos del ADC (ej. 12 bits)
        i_adc_data_valid      : in  std_logic; -- Pulso que indica que i_adc_data_in es válido

        -- Interfaz a EEPROM (asume que existe un módulo EEPROM)
        i_eeprom_read_data    : in  std_logic_vector(15 downto 0); -- Dato de int16_t de EEPROM
        i_eeprom_data_valid   : in  std_logic; -- Pulso que indica que el dato de EEPROM es válido
        o_eeprom_read_addr    : out natural; -- Dirección a leer en EEPROM
        o_eeprom_read_request : out std_logic; -- Pulso para solicitar lectura de EEPROM

        -- Salidas a GPIO de emisores (Ej: 4 pines)
        o_gpio_emitter_out    : out std_logic_vector(C_NUM_SENSORES-1 downto 0);

        -- Salidas de control ADC
        o_adc_read_en         : out std_logic; -- Pulso para iniciar conversión ADC
        o_adc_channel_select  : out natural range 0 to 15; -- Canal ADC a convertir

        -- Salidas de datos de sensores
        o_sensors_distance    : out (C_NUM_SENSORES-1 downto 0) (natural'range(0 to 4095)); -- Distancia calculada (natural)
        o_walls_front_det     : out std_logic; -- Detección de pared frontal
        o_walls_left_det      : out std_logic; -- Detección de pared izquierda
        o_walls_right_det     : out std_logic; -- Detección de pared derecha
        o_side_error          : out signed(15 downto 0); -- Error de lado
        o_front_angle_error   : out signed(15 downto 0); -- Error de ángulo frontal

        -- Salidas para LEDs de información (ej. 8 LEDs discretos)
        o_info_leds_control   : out std_logic_vector(7 downto 0)
    );
end entity sensors_controller;

architecture Behavioral of sensors_controller is

    -- --- Señales internas que corresponden a variables estáticas/volátiles de C ---
    signal s_sensors_enabled     : std_logic := '0';
    signal s_emitter_status      : natural range 1 to 4 := 1;
    signal s_sensor_index        : natural range 0 to C_NUM_SENSORES - 1 := SENSOR_FRONT_LEFT_WALL_ID;

    -- Arrays de datos de sensores (registros para persistencia)
    type sensor_raw_array_t is array (0 to C_NUM_SENSORES - 1) of natural range 0 to C_ADC_RESOLUTION - 1;
    signal s_sensors_raw_off : sensor_raw_array_t := (others => 0);
    signal s_sensors_raw_on  : sensor_raw_array_t := (others => 0);
    signal s_sensors_raw_filtered : sensor_raw_array_t := (others => 0); -- sensors_on - sensors_off
    signal s_sensors_distance_reg : sensor_raw_array_t := (others => 0); -- sensors_distance

    -- Compensaciones de distancia (cargadas desde EEPROM)
    type distance_offset_array_t is array (0 to C_NUM_SENSORES - 1) of signed(15 downto 0);
    signal s_sensors_distance_offset : distance_offset_array_t := (others => (others => '0'));

    -- Último error de ángulo frontal (para suavizado, si se implementa)
    signal s_last_front_sensors_angle_error : signed(15 downto 0) := (others => '0');

    -- Señales para el control GPIO de los emisores
    signal s_emitter_gpio_enable : std_logic_vector(C_NUM_SENSORES-1 downto 0) := (others => '0');

    -- Maquina de estados para sm_emitter_adc
    type emitter_adc_state_type is (
        EMITTER_ADC_IDLE,
        EMITTER_ADC_READ_OFF_WAIT,
        EMITTER_ADC_SET_ON,
        EMITTER_ADC_READ_ON_WAIT,
        EMITTER_ADC_SET_OFF_CYCLE
    );
    signal current_emitter_adc_state : emitter_adc_state_type;
    signal next_emitter_adc_state    : emitter_adc_state_type;

    -- Maquina de estados para la carga de EEPROM (si es un proceso de varios ciclos)
    type eeprom_load_state_type is (
        EEPROM_LOAD_IDLE,
        EEPROM_LOAD_REQUEST,
        EEPROM_LOAD_WAIT_DATA,
        EEPROM_LOAD_STORE_DATA,
        EEPROM_LOAD_NEXT_INDEX
    );
    signal current_eeprom_load_state : eeprom_load_state_type;
    signal next_eeprom_load_state    : eeprom_load_state_type;
    signal s_eeprom_load_index       : natural range 0 to C_NUM_SENSORES; -- Index for loading offsets

    -- Señales internas para control de LEDs de info
    signal s_info_leds_control : std_logic_vector(7 downto 0);

begin

    -- --- Lógica de la Máquina de Estados 'sm_emitter_adc' ---
    -- Proceso de registro de estado
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                current_emitter_adc_state <= EMITTER_ADC_IDLE;
                s_emitter_status          <= 1;
                s_sensor_index            <= SENSOR_FRONT_LEFT_WALL_ID;
            else
                current_emitter_adc_state <= next_emitter_adc_state;
            end if;
        end if;
    end process;

    -- Lógica de siguiente estado y salidas para 'sm_emitter_adc'
    process (current_emitter_adc_state, i_emitter_adc_tick, s_sensors_enabled,
             s_sensor_index, s_emitter_status, i_adc_data_in, i_adc_data_valid)
    begin
        -- Default outputs
        o_adc_read_en        <= '0';
        o_adc_channel_select <= 0;
        s_emitter_gpio_enable <= (others => '0'); -- Apaga todos los emisores por defecto
        next_emitter_adc_state <= current_emitter_adc_state;

        if s_sensors_enabled = '0' then -- Si los sensores están deshabilitados, apaga todo
            s_emitter_gpio_enable <= (others => '0');
            next_emitter_adc_state <= EMITTER_ADC_IDLE;
        else
            case current_emitter_adc_state is
                when EMITTER_ADC_IDLE =>
                    if i_emitter_adc_tick = '1' then
                        next_emitter_adc_state <= EMITTER_ADC_READ_OFF_WAIT;
                        -- Trigger primera lectura ADC OFF (para el sensor actual s_sensor_index)
                        o_adc_read_en        <= '1';
                        o_adc_channel_select <= to_integer(sensores(s_sensor_index)); -- Asume 'sensores' es global o accesible
                    end if;

                when EMITTER_ADC_READ_OFF_WAIT =>
                    if i_adc_data_valid = '1' then -- El ADC ha terminado la lectura
                        s_sensors_raw_off(s_sensor_index) <= to_natural(unsigned(i_adc_data_in)); -- Guarda lectura OFF
                        -- Enciende el emisor para la lectura ON
                        s_emitter_gpio_enable(s_sensor_index) <= '1';
                        next_emitter_adc_state <= EMITTER_ADC_SET_ON;
                    end if;

                when EMITTER_ADC_SET_ON =>
                    -- Da un ciclo para que el GPIO se asiente si es necesario
                    if i_emitter_adc_tick = '1' then -- Usar el mismo tick para secuenciar
                        next_emitter_adc_state <= EMITTER_ADC_READ_ON_WAIT;
                        o_adc_read_en        <= '1';
                        o_adc_channel_select <= to_integer(sensores(s_sensor_index));
                    end if;

                when EMITTER_ADC_READ_ON_WAIT =>
                    if i_adc_data_valid = '1' then -- El ADC ha terminado la lectura
                        s_sensors_raw_on(s_sensor_index) <= to_natural(unsigned(i_adc_data_in)); -- Guarda lectura ON
                        -- Apaga el emisor
                        s_emitter_gpio_enable(s_sensor_index) <= '0'; -- Desactiva el pin del emisor
                        next_emitter_adc_state <= EMITTER_ADC_SET_OFF_CYCLE;
                    end if;

                when EMITTER_ADC_SET_OFF_CYCLE =>
                    -- Da un ciclo para que el GPIO se asiente si es necesario
                    if i_emitter_adc_tick = '1' then -- Usar el mismo tick para secuenciar
                        -- Avanza al siguiente sensor
                        if s_sensor_index = (C_NUM_SENSORES - 1) then
                            s_sensor_index <= 0;
                        else
                            s_sensor_index <= s_sensor_index + 1;
                        end if;
                        next_emitter_adc_state <= EMITTER_ADC_IDLE; -- Vuelve al inicio del ciclo
                    end if;
            end case;
        end if;
    end process;
    -- Asignación de salida de GPIO de emisores
    o_gpio_emitter_out <= s_emitter_gpio_enable;

    -- --- Lógica para 'set_sensors_enabled' y 'get_sensors_enabled' ---
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                s_sensors_enabled <= '0';
            else
                if i_sensors_enabled_cmd = '1' and s_sensors_enabled = '0' then
                    -- Reset de estado al habilitar los sensores
                    s_emitter_status <= 1;
                    s_sensor_index   <= SENSOR_FRONT_LEFT_WALL_ID;
                end if;
                s_sensors_enabled <= i_sensors_enabled_cmd;
            end if;
        end if;
    end process;

    -- --- Lógica para 'update_sensors_magics' ---
    -- ESTO ES MUY CRÍTICO POR EL PUNTO FLOTANTE.
    -- Se asume que todas las operaciones se harán con números enteros escalados.
    -- Ejemplo: float_value * SCALE_FACTOR = integer_value
    -- Necesitarás un módulo para la división/multiplicación entera o punto fijo.
    -- La tabla ln_lookup debe ser una ROM con valores enteros escalados.
    process (i_clk)
        -- Variables temporales para cálculos intermedios en punto fijo
        variable v_raw_minus_c : signed(15 downto 0);
        variable v_ln_index    : natural;
        variable v_ln_value    : signed(15 downto 0); -- Valor ln escalado
        variable v_new_distance_scaled : signed(31 downto 0); -- Usar más bits para cálculos
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                s_sensors_raw_filtered <= (others => 0);
                s_sensors_distance_reg <= (others => 0);
            elsif i_update_magics_tick = '1' then -- Se activa con un pulso periódico
                for sensor_idx in 0 to C_NUM_SENSORES - 1 loop
                    -- Calcula sensors_raw[sensor] = sensors_on[sensor] - sensors_off[sensor];
                    if s_sensors_raw_on(sensor_idx) > s_sensors_raw_off(sensor_idx) then
                        s_sensors_raw_filtered(sensor_idx) <= s_sensors_raw_on(sensor_idx) - s_sensors_raw_off(sensor_idx);
                    else
                        s_sensors_raw_filtered(sensor_idx) <= 0;
                    end if;

                    if s_sensors_enabled = '1' and s_sensors_raw_filtered(sensor_idx) /= 0 then
                        -- Calcular ln_index = (sensors_raw[sensor] + sensors_distance_calibrations[sensor].c) / 4;
                        -- Todos los valores son ahora enteros/punto fijo.
                        v_raw_minus_c := to_signed(s_sensors_raw_filtered(sensor_idx), v_raw_minus_c'length) + C_SENSORS_CALIBRATIONS(sensor_idx).c_scaled;
                        v_ln_index := to_integer(v_raw_minus_c) / C_LOG_LINEARIZATION_TABLE_STEP;

                        -- Limitar ln_index para evitar acceso fuera de rango de la tabla
                        if v_ln_index < 0 then
                            v_ln_index := 0;
                        elsif v_ln_index >= C_LOG_LINEARIZATION_TABLE_SIZE then
                            v_ln_index := C_LOG_LINEARIZATION_TABLE_SIZE - 1;
                        end if;

                        -- Obtener valor de ln de la tabla ROM
                        v_ln_value := C_LN_LOOKUP_TABLE(v_ln_index);

                        -- Calcular new_sensor_distance = (a / ln - b) * 1000.0f
                        -- Esto es la parte más compleja en punto fijo.
                        -- Necesitarás un módulo de división de punto fijo o una implementación.
                        -- Ejemplo conceptual: v_new_distance_scaled := (C_SENSORS_CALIBRATIONS(sensor_idx).a_scaled * SCALE_FACTOR / v_ln_value - C_SENSORS_CALIBRATIONS(sensor_idx).b_scaled) * ANOTHER_SCALE;
                        -- Aquí una simplificación que no representa la complejidad de punto fijo:
                        if v_ln_value /= 0 then -- Evitar división por cero
                            v_new_distance_scaled := (resize(C_SENSORS_CALIBRATIONS(sensor_idx).a_scaled, v_new_distance_scaled'length) * 1000) / resize(v_ln_value, v_new_distance_scaled'length) - resize(C_SENSORS_CALIBRATIONS(sensor_idx).b_scaled, v_new_distance_scaled'length) * 1000;
                        else
                            v_new_distance_scaled := (others => '0'); -- O un valor de error
                        end if;

                        -- Limitar a 0 si es negativo
                        if v_new_distance_scaled < 0 then
                            v_new_distance_scaled := (others => '0');
                        end if;

                        -- Añadir offset de robot y calibración
                        case sensor_idx is
                            when SENSOR_FRONT_LEFT_WALL_ID | SENSOR_FRONT_RIGHT_WALL_ID =>
                                v_new_distance_scaled := v_new_distance_scaled + to_signed(C_ROBOT_FRONT_LENGTH * 1000, v_new_distance_scaled'length);
                            when SENSOR_SIDE_LEFT_WALL_ID | SENSOR_SIDE_RIGHT_WALL_ID =>
                                v_new_distance_scaled := v_new_distance_scaled + to_signed(C_ROBOT_MIDDLE_WIDTH * 1000, v_new_distance_scaled'length);
                            when others => null;
                        end case;
                        v_new_distance_scaled := v_new_distance_scaled + resize(s_sensors_distance_offset(sensor_idx) * 1000, v_new_distance_scaled'length); -- Ajusta offset a escala

                        s_sensors_distance_reg(sensor_idx) <= to_natural(v_new_distance_scaled / 1000); -- Vuelve a la escala original (ej. mm)
                    end if;
                end loop;
            end if;
        end if;
    end process;
    o_sensors_distance <= s_sensors_distance_reg; -- Asigna la salida al puerto

    -- --- Lógica para 'sensors_load_eeprom' (integrado con el módulo EEPROM) ---
    -- Esta FSM gestionaría la lectura secuencial de los 4 offsets desde EEPROM.
    -- Suponiendo que el módulo EEPROM tiene un puerto para pedir una dirección y devuelve un dato.
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                current_eeprom_load_state <= EEPROM_LOAD_IDLE;
                s_eeprom_load_index       <= 0;
                o_eeprom_read_request     <= '0';
                o_eeprom_read_addr        <= 0;
            else
                current_eeprom_load_state <= next_eeprom_load_state;
            end if;
        end if;
    end process;

    process (current_eeprom_load_state, i_eeprom_read_en, i_eeprom_data_valid, i_eeprom_read_data, s_eeprom_load_index)
    begin
        next_eeprom_load_state <= current_eeprom_load_state;
        o_eeprom_read_request  <= '0'; -- Default to no request
        o_eeprom_read_addr     <= 0;

        case current_eeprom_load_state is
            when EEPROM_LOAD_IDLE =>
                if i_eeprom_read_en = '1' then -- Pulso externo para iniciar la carga
                    s_eeprom_load_index <= 0;
                    next_eeprom_load_state <= EEPROM_LOAD_REQUEST;
                end if;

            when EEPROM_LOAD_REQUEST =>
                o_eeprom_read_request <= '1'; -- Pide el dato
                o_eeprom_read_addr    <= DATA_INDEX_SENSORS_OFFSETS + s_eeprom_load_index;
                next_eeprom_load_state <= EEPROM_LOAD_WAIT_DATA;

            when EEPROM_LOAD_WAIT_DATA =>
                if i_eeprom_data_valid = '1' then -- Dato recibido
                    next_eeprom_load_state <= EEPROM_LOAD_STORE_DATA;
                end if;

            when EEPROM_LOAD_STORE_DATA =>
                s_sensors_distance_offset(s_eeprom_load_index) <= signed(i_eeprom_read_data); -- Guarda el offset
                next_eeprom_load_state <= EEPROM_LOAD_NEXT_INDEX;

            when EEPROM_LOAD_NEXT_INDEX =>
                if s_eeprom_load_index < C_NUM_SENSORES - 1 then
                    s_eeprom_load_index <= s_eeprom_load_index + 1;
                    next_eeprom_load_state <= EEPROM_LOAD_REQUEST; -- Siguiente lectura
                else
                    next_eeprom_load_state <= EEPROM_LOAD_IDLE; -- Fin de carga
                end if;
        end case;
    end process;

    -- --- Lógica de detección de paredes (combinacional) ---
    -- Las constantes de detección deben estar en la misma escala que s_sensors_distance_reg
    o_walls_left_det  <= '1' when s_sensors_distance_reg(SENSOR_SIDE_LEFT_WALL_ID) < C_SENSOR_SIDE_DETECTION else '0';
    o_walls_right_det <= '1' when s_sensors_distance_reg(SENSOR_SIDE_RIGHT_WALL_ID) < C_SENSOR_SIDE_DETECTION else '0';
    o_walls_front_det <= '1' when (s_sensors_distance_reg(SENSOR_FRONT_LEFT_WALL_ID) < C_SENSOR_FRONT_DETECTION) or
                                   (s_sensors_distance_reg(SENSOR_FRONT_RIGHT_WALL_ID) < C_SENSOR_FRONT_DETECTION) else '0';

    -- --- Lógica de cálculo de errores (combinacional o con registros si se filtra) ---
    -- get_side_sensors_close_error
    process(s_sensors_distance_reg)
        variable v_left_error_side  : signed(15 downto 0);
        variable v_right_error_side : signed(15 downto 0);
    begin
        v_left_error_side  := to_signed(s_sensors_distance_reg(SENSOR_SIDE_LEFT_WALL_ID), 16) - to_signed(C_MIDDLE_MAZE_DISTANCE, 16);
        v_right_error_side := to_signed(s_sensors_distance_reg(SENSOR_SIDE_RIGHT_WALL_ID), 16) - to_signed(C_MIDDLE_MAZE_DISTANCE, 16);

        if v_left_error_side > 0 and v_right_error_side < 0 then
            o_side_error <= v_right_error_side;
        elsif v_right_error_side > 0 and v_left_error_side < 0 then
            o_side_error <= -v_left_error_side;
        else
            o_side_error <= (others => '0');
        end if;
    end process;

    -- get_front_sensors_angle_error (sin filtrado suave, solo la diferencia)
    process(o_walls_front_det, s_sensors_distance_reg)
        variable v_error : signed(15 downto 0);
    begin
        if o_walls_front_det = '0' then
            o_front_angle_error <= (others => '0');
            s_last_front_sensors_angle_error <= (others => '0'); -- Reset en VHDL FSM (si se filtra)
        else
            v_error := to_signed(s_sensors_distance_reg(SENSOR_FRONT_LEFT_WALL_ID), 16) - to_signed(s_sensors_distance_reg(SENSOR_FRONT_RIGHT_WALL_ID), 16);
            -- Si quieres el filtro: o_front_angle_error <= (v_error * 102) / 1024 + (s_last_front_sensors_angle_error * 922) / 1024;
            -- y actualizar s_last_front_sensors_angle_error en un proceso secuencial.
            o_front_angle_error <= v_error;
        end if;
    end process;


    -- --- Lógica para 'update_side_sensors_leds' ---
    -- Esta podría ser una FSM o lógica combinacional grande, dependiendo de la
    -- complejidad y si necesitas un control secuencial de los LEDs.
    -- Aquí, una asignación combinacional para 'o_info_leds_control'.
    process(o_side_error)
    begin
        o_info_leds_control <= (others => '0'); -- Apaga todos por defecto
        -- Asume 'abs' para signed.
        if abs(to_integer(o_side_error)) < 2 then
            o_info_leds_control <= (others => '0'); -- Clear_info_leds()
        elsif to_integer(o_side_error) >= 20 then
            o_info_leds_control(0) <= '1'; -- set_info_led(0, true)
        elsif to_integer(o_side_error) >= 10 then
            o_info_leds_control(1) <= '1'; -- set_info_led(1, true)
        elsif to_integer(o_side_error) >= 5 then
            o_info_leds_control(2) <= '1'; -- set_info_led(2, true)
        elsif to_integer(o_side_error) >= 0 then -- Asume 0 significa un poco a la izquierda
            o_info_leds_control(3) <= '1'; -- set_info_led(3, true)
        elsif to_integer(o_side_error) <= -20 then
            o_info_leds_control(7) <= '1'; -- set_info_led(7, true)
        elsif to_integer(o_side_error) <= -10 then
            o_info_leds_control(6) <= '1'; -- set_info_led(6, true)
        elsif to_integer(o_side_error) <= -5 then
            o_info_leds_control(5) <= '1'; -- set_info_led(5, true)
        elsif to_integer(o_side_error) <= 0 then -- Asume 0 significa un poco a la derecha
            o_info_leds_control(4) <= '1'; -- set_info_led(4, true)
        end if;
    end process;


    -- --- Conexiones de Puertos de Salida ---
    -- Las salidas directas a los puertos (que son esencialmente 'get' de C)
    -- ya están manejadas por la asignación de señales internas a los puertos de salida.
    -- o_sensors_distance <= s_sensors_distance_reg; -- Ya asignado arriba

end architecture Behavioral;
