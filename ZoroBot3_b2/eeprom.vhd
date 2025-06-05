
-- eeprom.vhd
-- Este módulo VHDL implementa la funcionalidad de una EEPROM (memoria no volátil)
-- utilizando RAM interna de la FPGA. Almacena y gestiona datos persistentes
-- para el robot, como la configuración del laberinto, parámetros de sensores, etc.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Incluye el paquete de tipos y constantes personalizados,
-- que contiene la definición de eeprom_data_t y las constantes de EEPROM.
use work.robot_types_pkg.all;

entity eeprom_controller is
    port (
        i_clk                       : in  std_logic;                                -- Reloj del sistema
        i_reset                     : in  std_logic;                                -- Reset síncrono (activo alto)
        i_get_clock_ticks           : in  natural;                                  -- Ticks del reloj en ms (para delays visuales)

        -- Comandos de control (pulsos, activos en alto por un ciclo de reloj)
        i_save_en                   : in  std_logic;                                -- Pulso para activar eeprom_save()
        i_load_en                   : in  std_logic;                                -- Pulso para activar eeprom_load()
        i_clear_en                  : in  std_logic;                                -- Pulso para activar eeprom_clear()
        i_restore_en                : in  std_logic;                                -- Pulso para activar eeprom_restore()
        i_backup_en                 : in  std_logic;                                -- Pulso para activar eeprom_backup() (Debug)

        -- Entradas para eeprom_set_data()
        i_set_data_en               : in  std_logic;                                -- Pulso para activar eeprom_set_data()
        i_set_data_index            : in  natural range 0 to DATA_LENGTH - 1;       -- Índice de inicio para set_data
        i_set_data_value            : in  signed(15 downto 0);                      -- Dato a escribir en set_data
        i_set_data_write_en         : in  std_logic;                                -- Habilitador de escritura para set_data (para arrays)

        -- Entradas/Salidas para acceder a los datos (simula eeprom_get_data())
        i_read_data_index           : in  natural range 0 to DATA_LENGTH - 1;       -- Índice para lectura
        o_read_data_value           : out signed(15 downto 0);                      -- Dato leído en el índice

        -- Salidas para notificar a otros módulos que carguen sus datos
        o_lsm6dsr_load_eeprom_en    : out std_logic;                                -- Pulso para lsm6dsr_load_eeprom()
        o_sensors_load_eeprom_en    : out std_logic;                                -- Pulso para sensors_load_eeprom()
        o_floodfill_load_maze_en    : out std_logic;                                -- Pulso para floodfill_load_maze()
        o_menu_run_load_values_en   : out std_logic;                                -- Pulso para menu_run_load_values()
        o_rc5_load_eeprom_en        : out std_logic;                                -- Pulso para rc5_load_eeprom()

        -- Salidas para LEDs de estado (simula set_status_led, toggle_status_led)
        o_set_status_led_en         : out std_logic;                                -- Pulso para set_status_led
        o_status_led_val            : out std_logic;                                -- Valor para set_status_led
        o_toggle_status_led_en      : out std_logic                                 -- Pulso para toggle_status_led
    );
end entity eeprom_controller;

architecture Behavioral of eeprom_controller is

    -- =========================================================================
    -- SEÑALES INTERNAS (equivalentes a variables 'static' en C)
    -- =========================================================================
    signal s_eeprom_data : eeprom_data_t := (others => (others => '0')); -- Memoria interna, int16_t (signed 15 downto 0)

    -- Para la simulación del delay_us y el toggle_status_led
    signal s_delay_start_ms  : natural := 0;
    signal s_delay_duration_ms : natural := 0;
    signal s_delay_done_pulse  : std_logic := '0';
    signal s_save_loop_cnt   : natural range 0 to 2 := 0; -- Counter for save loop

    -- =========================================================================
    -- ESTADOS DE LA MÁQUINA DE ESTADOS
    -- =========================================================================
    type eeprom_fsm_state is (
        STATE_IDLE,                 -- Esperando un comando
        STATE_SAVE_INIT,            -- Inicia secuencia de guardado
        STATE_SAVE_BLINK_LED,       -- Parpadeo de LED durante el guardado
        STATE_SAVE_WRITE_DATA,      -- Escritura de datos
        STATE_SAVE_END,             -- Finaliza guardado
        STATE_LOAD_INIT,            -- Inicia secuencia de carga
        STATE_LOAD_READ_DATA,       -- Lectura de datos
        STATE_LOAD_NOTIFY_MODULES,  -- Notifica a otros módulos
        STATE_LOAD_END,             -- Finaliza carga
        STATE_CLEAR_INIT,           -- Inicia borrado
        STATE_CLEAR_PROCESS,        -- Proceso de borrado
        STATE_CLEAR_END,            -- Finaliza borrado
        STATE_RESTORE_INIT,         -- Inicia restauración
        STATE_RESTORE_PROCESS,      -- Proceso de restauración (hardcoded data)
        STATE_RESTORE_END,          -- Finaliza restauración
        STATE_SET_DATA              -- Eeprom_set_data()
    );
    signal current_state : eeprom_fsm_state := STATE_IDLE;
    signal next_state    : eeprom_fsm_state;

    -- Variables para el bucle de save/load/clear
    signal s_addr_idx      : natural range 0 to DATA_LENGTH := 0; -- current index for loops


begin

    -- =========================================================================
    -- PROCESO PRINCIPAL DE LA MÁQUINA DE ESTADOS Y LÓGICA DE CONTROL
    -- =========================================================================
    process (i_clk)
        -- Declaración de la copia de seguridad hardcoded (similar a eeprom_backup en C)
        -- Ajusta este array con tus datos reales si lo necesitas
        constant C_EEPROM_BACKUP_DATA : eeprom_data_t := (
            1, 56, 0, 0, 2, 17, 520, 85, 325, 45, 15, 28, 4, 20, 20, 6, 9, 21, 17, 21,
            5, 2, 11, 12, 4, 6, 11, 10, 11, 24, 16, 2, 11, 10, 11, 13, 7, 26, 9, 2,
            25, 19, 25, 21, 17, 19, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 0, 0, 1 -- End of the provided example data
        );

    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                current_state <= STATE_IDLE;
                s_addr_idx <= 0;
                s_save_loop_cnt <= 0;
                s_delay_start_ms <= 0;
                s_delay_duration_ms <= 0;
                s_delay_done_pulse <= '0';
                s_eeprom_data <= (others => (others => '0')); -- Clear memory on reset

                -- Reset all pulse outputs
                o_lsm6dsr_load_eeprom_en <= '0';
                o_sensors_load_eeprom_en <= '0';
                o_floodfill_load_maze_en <= '0';
                o_menu_run_load_values_en <= '0';
                o_rc5_load_eeprom_en <= '0';
                o_set_status_led_en <= '0';
                o_status_led_val <= '0';
                o_toggle_status_led_en <= '0';

            else
                -- Lógica no bloqueante para el delay
                if s_delay_duration_ms /= 0 and (i_get_clock_ticks - s_delay_start_ms) >= s_delay_duration_ms then
                    s_delay_duration_ms <= 0; -- Termina el delay
                    s_delay_done_pulse <= '1'; -- Pulso de "delay done"
                else
                    s_delay_done_pulse <= '0';
                end if;

                -- Reset all pulse outputs at the start of each clock cycle, then set them if needed
                o_lsm6dsr_load_eeprom_en <= '0';
                o_sensors_load_eeprom_en <= '0';
                o_floodfill_load_maze_en <= '0';
                o_menu_run_load_values_en <= '0';
                o_rc5_load_eeprom_en <= '0';
                o_set_status_led_en <= '0';
                o_toggle_status_led_en <= '0';

                -- Update state machine
                current_state <= next_state;

                -- Lógica para eeprom_set_data()
                if i_set_data_en = '1' and i_set_data_write_en = '1' then
                    if i_set_data_index < DATA_LENGTH then
                        s_eeprom_data(i_set_data_index) <= i_set_data_value;
                    end if;
                end if;

                case current_state is
                    when STATE_IDLE =>
                        if i_save_en = '1' then
                            s_save_loop_cnt <= 0;
                            o_set_status_led_en <= '1'; o_status_led_val <= '0'; -- set_status_led(false)
                            next_state <= STATE_SAVE_BLINK_LED;
                        elsif i_load_en = '1' then
                            s_addr_idx <= 0;
                            next_state <= STATE_LOAD_READ_DATA;
                        elsif i_clear_en = '1' then
                            s_addr_idx <= 0;
                            next_state <= STATE_CLEAR_PROCESS;
                        elsif i_restore_en = '1' then
                            s_addr_idx <= 0;
                            next_state <= STATE_RESTORE_PROCESS;
                        when others => null; -- Stay in IDLE
                end case;

                -- =========================================================
                -- eeprom_save()
                -- =========================================================
                when STATE_SAVE_BLINK_LED =>
                    if s_save_loop_cnt < 2 then -- Loop twice for blinking
                        o_toggle_status_led_en <= '1'; -- toggle_status_led()
                        s_delay_start_ms <= i_get_clock_ticks;
                        s_delay_duration_ms <= 50; -- delay_us(50000) is 50ms
                        next_state <= STATE_SAVE_INIT;
                    else
                        o_set_status_led_en <= '1'; o_status_led_val <= '1'; -- set_status_led(true)
                        s_addr_idx <= 0;
                        next_state <= STATE_SAVE_WRITE_DATA;
                    end if;

                when STATE_SAVE_INIT => -- Wait for delay to finish
                    if s_delay_done_pulse = '1' then
                        o_toggle_status_led_en <= '1'; -- toggle_status_led()
                        s_delay_start_ms <= i_get_clock_ticks;
                        s_delay_duration_ms <= 50;
                        s_save_loop_cnt <= s_save_loop_cnt + 1;
                        next_state <= STATE_SAVE_BLINK_LED;
                    end if;

                when STATE_SAVE_WRITE_DATA =>
                    if s_addr_idx < DATA_LENGTH then
                        -- flash_program_word(addr, eeprom_data[i])
                        -- In this model, it's just writing to internal RAM
                        -- s_eeprom_data(s_addr_idx) <= s_eeprom_data(s_addr_idx); -- Already implicitly stored by s_eeprom_data
                        s_addr_idx <= s_addr_idx + 1;
                        next_state <= STATE_SAVE_WRITE_DATA;
                    else
                        -- flash_lock();
                        o_set_status_led_en <= '1'; o_status_led_val <= '0'; -- set_status_led(false)
                        next_state <= STATE_SAVE_END;
                    end if;

                when STATE_SAVE_END =>
                    next_state <= STATE_IDLE;

                -- =========================================================
                -- eeprom_load()
                -- =========================================================
                when STATE_LOAD_READ_DATA =>
                    if s_addr_idx < DATA_LENGTH then
                        -- eeprom_data[i] = MMIO32(addr);
                        -- Data is directly available from s_eeprom_data(s_addr_idx)
                        s_addr_idx <= s_addr_idx + 1;
                        next_state <= STATE_LOAD_READ_DATA;
                    else
                        next_state <= STATE_LOAD_NOTIFY_MODULES;
                    end if;

                when STATE_LOAD_NOTIFY_MODULES =>
                    -- Emit pulses to signal other modules to load their data
                    o_lsm6dsr_load_eeprom_en <= '1';
                    o_sensors_load_eeprom_en <= '1';
                    o_floodfill_load_maze_en <= '1';
                    o_menu_run_load_values_en <= '1';
                    o_rc5_load_eeprom_en <= '1';
                    next_state <= STATE_LOAD_END;

                when STATE_LOAD_END =>
                    next_state <= STATE_IDLE;

                -- =========================================================
                -- eeprom_clear()
                -- =========================================================
                when STATE_CLEAR_PROCESS =>
                    if s_addr_idx < DATA_LENGTH then
                        s_eeprom_data(s_addr_idx) <= (others => '0'); -- Clear data (simulate flash_erase_sector)
                        s_addr_idx <= s_addr_idx + 1;
                        next_state <= STATE_CLEAR_PROCESS;
                    else
                        next_state <= STATE_CLEAR_END;
                    end if;

                when STATE_CLEAR_END =>
                    next_state <= STATE_IDLE;

                -- =========================================================
                -- eeprom_restore()
                -- =========================================================
                when STATE_RESTORE_PROCESS =>
                    if s_addr_idx < DATA_LENGTH then
                        s_eeprom_data(s_addr_idx) <= C_EEPROM_BACKUP_DATA(s_addr_idx); -- Copy hardcoded data
                        s_addr_idx <= s_addr_idx + 1;
                        next_state <= STATE_RESTORE_PROCESS;
                    else
                        next_state <= STATE_RESTORE_END;
                    end if;

                when STATE_RESTORE_END =>
                    next_state <= STATE_IDLE;

                -- =========================================================
                -- eeprom_set_data() (handled combinatorially/concurrently)
                -- =========================================================
                when STATE_SET_DATA =>
                    -- This state is not needed as s_eeprom_data updates concurrently
                    -- when i_set_data_en and i_set_data_write_en are high.
                    -- Transition back to IDLE immediately if this state were entered.
                    next_state <= STATE_IDLE;

                when others =>
                    next_state <= STATE_IDLE;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- LÓGICA CONCURRENTE / ASIGNACIONES DE SALIDA
    -- =========================================================================

    -- eeprom_get_data() simulation
    o_read_data_value <= s_eeprom_data(i_read_data_index);

end architecture Behavioral;
