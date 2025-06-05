
-- IEEE Library includes for standard logic and numeric types
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Definir constantes para las dimensiones del laberinto
-- Estas pueden pasarse como genéricos para permitir la configuración
-- o declararse como constantes si son fijas dentro de esta entidad.
constant C_MAZE_ROWS_HOME       : natural := 6;
constant C_MAZE_COLUMNS_HOME    : natural := 6;
constant C_MAZE_ROWS_COMPETITION  : natural := 16;
constant C_MAZE_COLUMNS_COMPETITION : natural := 16;

-- Definir el tamaño máximo de la pila para objetivos según el máximo de celdas posibles.
-- Para 16x16, el máximo de objetivos es 4. Sin embargo, para una pila general, debería ser mayor.
-- Supongamos un máximo razonable para 'goals.stack'.
constant C_MAX_GOALS_STACK_SIZE : natural := 4; -- Max goals for competition maze (fixed 4 goals)
constant C_MAX_CELL_INDEX     : natural := C_MAZE_ROWS_COMPETITION * C_MAZE_COLUMNS_COMPETITION;

-- Definir enumeración de tipo laberinto
type maze_type_t is (MAZE_HOME, MAZE_COMPETITION);
-- Nota: 'menu_run_get_maze_type()' de C sería una señal de entrada de este tipo.
-- La representaremos como entrada 'i_maze_type'.

--------------------------------------------------------------------------------
-- Entidad: maze_logic_module
-- Descripción: Equivalente VHDL de la dimensión del laberinto y la lógica de generación de objetivos. --
-- Entradas:
-- i_clk: Señal de reloj (si se requiere lógica secuencial para las escrituras en la pila)
-- i_reset: Señal de reinicio
-- i_load_goals: Pulso para activar la carga de objetivos (equivalente a la llamada a maze_get_goals())
-- i_maze_type: Señal de entrada que representa la salida de menu_run_get_maze_type()
--
-- Salidas:
-- o_maze_rows: Número actual de filas del laberinto
-- o_maze_columns: Número actual de columnas del laberinto
-- o_maze_cells: Número total de celdas del laberinto
-- o_goals_stack_size: Tamaño actual de la pila de objetivos
-- o_goals_stack_data: Datos de los objetivos (pueden ser un array o una conexión a memoria)
--------------------------------------------------------------------------------
entity maze_logic_module is
    port (
        i_clk             : in  std_logic;
        i_reset           : in  std_logic;
        i_load_goals      : in  std_logic; -- Pulse to trigger goal loading
        i_maze_type       : in  maze_type_t; -- From menu_run_get_maze_type()

        o_maze_rows       : out natural;
        o_maze_columns    : out natural;
        o_maze_cells      : out natural;
        o_goals_stack_size: out natural;
        o_goals_stack_data: out std_logic_vector(C_MAX_GOALS_STACK_SIZE-1 downto 0)(natural'range(to_integer(ceil(log2(real(C_MAX_CELL_INDEX)))))-1 downto 0) -- Array of cell indices
    );
end entity maze_logic_module;

architecture Behavioral of maze_logic_module is

    -- Internal signals to hold current maze dimensions
    signal s_current_rows    : natural;
    signal s_current_columns : natural;

    -- Signal for the goals stack (representing 'struct cells_stack goals;')
    -- VHDL arrays need defined bounds.
    -- Assuming cell index fits in a reasonable number of bits, e.g., 8 bits for 16x16 = 256 cells (0 to 255)
    -- ceil(log2(real(C_MAX_CELL_INDEX))) gives the number of bits needed.
    -- For C_MAX_CELL_INDEX = 256, it's 8 bits (0 to 255)
    subtype cell_index_t is natural range 0 to C_MAX_CELL_INDEX - 1;
    type goals_stack_array_t is array (0 to C_MAX_GOALS_STACK_SIZE - 1) of cell_index_t;
    signal s_goals_stack     : goals_stack_array_t;
    signal s_goals_stack_ptr : natural range 0 to C_MAX_GOALS_STACK_SIZE; -- Represents goals.size

    -- Define ROMs (Read-Only Memory) for fixed goal coordinates
    -- These are fixed values, so they're suitable for ROM.
    type home_goals_rom_t is array (0 to 3) of cell_index_t; -- 4 goals for home maze
    constant C_HOME_GOALS_ROM : home_goals_rom_t := (
        (6-1) + (6-1) * C_MAZE_COLUMNS_HOME, -- add_goal(6,6)
        (6-1) + (5-1) * C_MAZE_COLUMNS_HOME, -- add_goal(6,5)
        (5-1) + (6-1) * C_MAZE_COLUMNS_HOME, -- add_goal(5,6)
        (5-1) + (5-1) * C_MAZE_COLUMNS_HOME  -- add_goal(5,5)
    );

    type competition_goals_rom_t is array (0 to 3) of cell_index_t; -- 4 goals for competition maze
    constant C_COMPETITION_GOALS_ROM : competition_goals_rom_t := (
        (8-1) + (8-1) * C_MAZE_COLUMNS_COMPETITION, -- add_goal(8,8)
        (8-1) + (9-1) * C_MAZE_COLUMNS_COMPETITION, -- add_goal(8,9)
        (9-1) + (8-1) * C_MAZE_COLUMNS_COMPETITION, -- add_goal(9,8)
        (9-1) + (9-1) * C_MAZE_COLUMNS_COMPETITION  -- add_goal(9,9)
    );

begin

    -- Combinatorial process for maze dimensions (equivalent to maze_get_rows/columns/cells)
    process (i_maze_type)
    begin
        case i_maze_type is
            when MAZE_HOME =>
                s_current_rows    <= C_MAZE_ROWS_HOME;
                s_current_columns <= C_MAZE_COLUMNS_HOME;
            when MAZE_COMPETITION =>
                s_current_rows    <= C_MAZE_ROWS_COMPETITION;
                s_current_columns <= C_MAZE_COLUMNS_COMPETITION;
            when others =>
                -- Default or error handling (e.g., set to 0 or a known safe state)
                s_current_rows    <= 0;
                s_current_columns <= 0;
        end case;
    end process;

    o_maze_rows    <= s_current_rows;
    o_maze_columns <= s_current_columns;
    o_maze_cells   <= s_current_rows * s_current_columns; -- Combinatorial calculation

    -- Sequential process for loading goals (equivalent to maze_get_goals() and add_goal())
    -- This requires a clock and a trigger (i_load_goals) as it involves writing to a "stack"
    process (i_clk)
        variable v_goal_index : natural range 0 to C_MAX_GOALS_STACK_SIZE -1;
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                s_goals_stack_ptr <= 0; -- Reset stack size
            elsif i_load_goals = '1' then -- On a rising edge of 'load_goals'
                s_goals_stack_ptr <= 0; -- Reset stack pointer before loading new goals

                case i_maze_type is
                    when MAZE_HOME =>
                        for v_goal_index in 0 to 3 loop -- Iterate 4 times for 4 goals
                            s_goals_stack(v_goal_index) <= C_HOME_GOALS_ROM(v_goal_index);
                        end loop;
                        s_goals_stack_ptr <= 4; -- Set size to 4
                    when MAZE_COMPETITION =>
                        for v_goal_index in 0 to 3 loop -- Iterate 4 times for 4 goals
                            s_goals_stack(v_goal_index) <= C_COMPETITION_GOALS_ROM(v_goal_index);
                        end loop;
                        s_goals_stack_ptr <= 4; -- Set size to 4
                    when others =>
                        s_goals_stack_ptr <= 0; -- No goals for unknown type
                end case;
            end if;
        end if;
    end process;

    -- Assign outputs
    o_goals_stack_size <= s_goals_stack_ptr;
    -- Map integer array to std_logic_vector array for output.
    -- This mapping needs to correctly handle the bit width of cell_index_t
    G_OUTPUT_GOALS : for i in 0 to C_MAX_GOALS_STACK_SIZE - 1 generate
        o_goals_stack_data(i) <= std_logic_vector(to_unsigned(s_goals_stack(i), o_goals_stack_data(i)'length));
    end generate G_OUTPUT_GOALS;

end architecture Behavioral;
