
-- macroarray.vhd
-- Este módulo VHDL implementa un buffer circular para almacenar datos de depuración.
-- Es una traducción de la funcionalidad C de 'macroarray.h'.
-- Proporciona una interfaz para almacenar datos y para leerlos secuencialmente.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Incluye el paquete de tipos personalizados
use work.robot_types_pkg.all;

entity macroarray_controller is
    port (
        i_clk                       : in  std_logic;                                -- Reloj del sistema
        i_reset                     : in  std_logic;                                -- Reset síncrono (activo alto)
        i_get_clock_ticks           : in  natural;                                  -- Ticks del reloj en ms (para el rate limiter)

        -- Entrada para la función macroarray_store()
        i_store_en                  : in  std_logic;                                -- Pulso para activar el almacenamiento
        i_store_ms_threshold        : in  natural;                                  -- Umbral de tiempo en ms para el rate limiter
        i_store_float_bits          : in  std_logic_vector(19 downto 0);            -- Bitmask para indicar cuáles son floats (up to 20 bits)
        i_store_label_indices       : in  macroarray_label_indices_t;               -- Array de índices de etiquetas (hasta C_MACROARRAY_MAX_LABELS)
        i_store_data_size           : in  natural range 0 to C_MACROARRAY_MAX_LABELS; -- Número de elementos de datos a almacenar
        i_store_data_values         : in  macroarray_data_t(0 to C_MACROARRAY_MAX_LABELS-1); -- Array de datos a almacenar

        -- Salidas para la función macroarray_print() / lectura de datos
        o_read_data_ready           : out std_logic;                                -- Pulso cuando un nuevo conjunto de datos está listo para leer
        o_read_label_idx            : out natural range 0 to C_MACROARRAY_MAX_LABELS-1; -- Índice de la etiqueta actual
        o_read_data_val             : out signed(15 downto 0);                      -- Valor del dato actual
        o_read_is_float             : out std_logic;                                -- Indica si el dato debe imprimirse como float
        o_read_done_all_items       : out std_logic                                 -- Pulso cuando se han leído todos los elementos desde 'macroarray_start' hasta 'macroarray_end'
    );
end entity macroarray_controller;

architecture Behavioral of macroarray_controller is

    -- =========================================================================
    -- SEÑALES INTERNAS (equivalentes a variables 'static' en C)
    -- =========================================================================
    signal s_macroarray_last_update_ms : natural := 0;
    signal s_macroarray_size           : natural range 0 to C_MACROARRAY_MAX_LABELS := 0;
    signal s_macroarray_float_bits     : std_logic_vector(19 downto 0) := (others => '0');
    signal s_macroarray_start          : natural range 0 to C_MACROARRAY_LENGTH-1 := 0;
    signal s_macroarray_end            : natural range 0 to C_MACROARRAY_LENGTH-1 := 0;
    signal s_macroarray_data_buffer    : macroarray_data_t; -- El array de datos
    signal s_macroarray_label_indices_buffer : macroarray_label_indices_t; -- El array de índices de etiquetas

    -- =========================================================================
    -- FSM para la lectura de datos (macroarray_print)
    -- Permite leer el buffer de forma secuencial.
    -- =========================================================================
    type read_fsm_state_type is (
        READ_IDLE,          -- Esperando un comando de lectura
        READ_INIT,          -- Inicializando la lectura
        READ_DATA_ITEM,     -- Leyendo un elemento de datos
        READ_NEXT_ITEM      -- Preparando el siguiente elemento o terminando
    );
    signal current_read_state : read_fsm_state_type := READ_IDLE;
    signal next_read_state    : read_fsm_state_type;

    signal s_read_current_ptr   : natural range 0 to C_MACROARRAY_LENGTH-1 := 0;
    signal s_read_current_col   : natural range 1 to C_MACROARRAY_MAX_LABELS := 1;
    signal s_o_read_data_ready_pulse : std_logic := '0';
    signal s_o_read_done_all_items_pulse : std_logic := '0';

begin

    -- =========================================================================
    -- PROCESO PRINCIPAL DE ALMACENAMIENTO (macroarray_store)
    -- =========================================================================
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                s_macroarray_last_update_ms <= 0;
                s_macroarray_size           <= 0;
                s_macroarray_float_bits     <= (others => '0');
                s_macroarray_start          <= 0;
                s_macroarray_end            <= 0;
                s_macroarray_data_buffer    <= (others => (others => '0'));
                s_macroarray_label_indices_buffer <= (others => 0);
            else
                -- Rate limiter y validación de tamaño
                if i_store_en = '1' and
                   (i_get_clock_ticks - s_macroarray_last_update_ms >= i_store_ms_threshold) and
                   (i_store_data_size <= C_MACROARRAY_MAX_LABELS) then

                    s_macroarray_last_update_ms <= i_get_clock_ticks;

                    -- Lógica: if (macroarray_size != size)
                    if s_macroarray_size /= i_store_data_size then
                        s_macroarray_start <= 0;
                        s_macroarray_end   <= 0;
                        s_macroarray_size  <= i_store_data_size;
                        s_macroarray_float_bits <= i_store_float_bits;
                        -- Copiar etiquetas (solo los índices, asumiendo ROM externa para strings)
                        for i in 0 to i_store_data_size - 1 loop
                            s_macroarray_label_indices_buffer(i) <= i_store_label_indices(i);
                        end loop;
                    end if;

                    -- Lógica: if (macroarray_end + size > MACROARRAY_LENGTH)
                    if s_macroarray_end + i_store_data_size > C_MACROARRAY_LENGTH then
                        s_macroarray_start <= s_macroarray_end - i_store_data_size;
                        s_macroarray_end   <= 0;
                    end if;

                    -- Almacenar los nuevos datos (equivalente a va_arg loop)
                    for i in 0 to i_store_data_size - 1 loop
                        s_macroarray_data_buffer(s_macroarray_end) <= i_store_data_values(i);
                        s_macroarray_end <= (s_macroarray_end + 1) mod C_MACROARRAY_LENGTH; -- Circular buffer
                    end loop;

                    -- Ajustar s_macroarray_start si el buffer se llenó y 'end' se ha adelantado a 'start'
                    -- Esto es necesario si el buffer es lo suficientemente pequeño como para que end envuelva start
                    if s_macroarray_end > s_macroarray_start and s_macroarray_end - s_macroarray_start > C_MACROARRAY_LENGTH - i_store_data_size then
                        -- Envolver el puntero de inicio si la nueva escritura sobrescribió el inicio
                        s_macroarray_start <= (s_macroarray_start + i_store_data_size) mod C_MACROARRAY_LENGTH;
                    end if;

                end if;
            end if;
        end if;
    end process;


    -- =========================================================================
    -- PROCESO DE CONTROL DE LECTURA (macroarray_print - FSM)
    -- Esto permite que un módulo externo (ej. UART) solicite y lea los datos.
    -- =========================================================================
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                current_read_state <= READ_IDLE;
                s_read_current_ptr <= 0;
                s_read_current_col <= 1;
                s_o_read_data_ready_pulse <= '0';
                s_o_read_done_all_items_pulse <= '0';
            else
                current_read_state <= next_read_state;
                -- Borrar pulsos en cada ciclo
                s_o_read_data_ready_pulse <= '0';
                s_o_read_done_all_items_pulse <= '0';
            end if;
        end if;
    end process;

    process (current_read_state, s_macroarray_size, s_macroarray_start, s_macroarray_end,
             s_read_current_ptr, s_read_current_col,
             s_macroarray_data_buffer, s_macroarray_float_bits)
    begin
        next_read_state <= current_read_state;
        -- Default outputs
        o_read_label_idx <= 0;
        o_read_data_val  <= (others => '0');
        o_read_is_float  <= '0';
        o_read_data_ready <= s_o_read_data_ready_pulse;
        o_read_done_all_items <= s_o_read_done_all_items_pulse;


        case current_read_state is
            when READ_IDLE =>
                -- Esperando un comando para iniciar la lectura
                -- Un input i_read_command_en podría ser útil aquí.
                -- Por ahora, si hay datos almacenados, iniciar lectura.
                if s_macroarray_size > 0 and s_macroarray_start /= s_macroarray_end then
                    next_read_state <= READ_INIT;
                end if;

            when READ_INIT =>
                s_read_current_ptr <= s_macroarray_start;
                s_read_current_col <= 1;
                next_read_state <= READ_DATA_ITEM;

            when READ_DATA_ITEM =>
                if s_read_current_ptr = s_macroarray_end then
                    next_read_state <= READ_DONE_ALL_ITEMS; -- Todos los elementos leídos
                else
                    -- Salidas para el elemento actual
                    o_read_label_idx <= s_macroarray_label_indices_buffer(s_read_current_col - 1);
                    o_read_data_val  <= s_macroarray_data_buffer(s_read_current_ptr);
                    o_read_is_float  <= s_macroarray_float_bits(s_macroarray_size - s_read_current_col);
                    s_o_read_data_ready_pulse <= '1'; -- Pulso para indicar que el dato está listo

                    next_read_state <= READ_NEXT_ITEM;
                end if;

            when READ_NEXT_ITEM =>
                -- Mover al siguiente elemento
                if s_read_current_col = s_macroarray_size then
                    s_read_current_col <= 1; -- Reiniciar columna para el siguiente "grupo"
                    s_read_current_ptr <= (s_read_current_ptr + 1) mod C_MACROARRAY_LENGTH; -- Siguiente elemento en el buffer
                else
                    s_read_current_col <= s_read_current_col + 1; -- Siguiente columna en el grupo actual
                    s_read_current_ptr <= (s_read_current_ptr + 1) mod C_MACROARRAY_LENGTH; -- Siguiente elemento en el buffer
                end if;

                next_read_state <= READ_DATA_ITEM; -- Volver a leer el siguiente elemento

            when READ_DONE_ALL_ITEMS =>
                s_o_read_done_all_items_pulse <= '1';
                next_read_state <= READ_IDLE; -- Volver a IDLE

            when others =>
                next_read_state <= READ_IDLE;
        end case;
    end process;


    -- =========================================================================
    -- ASIGNACIONES DE SALIDA (Combinacional, desde el proceso de lectura)
    -- =========================================================================
    o_read_data_ready     <= s_o_read_data_ready_pulse;
    o_read_done_all_items <= s_o_read_done_all_items_pulse;

end architecture Behavioral;
