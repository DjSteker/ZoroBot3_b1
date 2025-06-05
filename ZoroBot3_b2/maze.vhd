
-- menu.vhd
-- Este módulo VHDL gestiona la selección del menú activo (MENU_RUN o MENU_CONFIG)
-- y delega el manejo a los submódulos correspondientes.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Incluye el paquete de tipos personalizados donde se definen enumeraciones y records
use work.robot_types_pkg.all;

entity menu_controller is
    port (
        i_clk                       : in  std_logic;                                -- Reloj del sistema
        i_reset                     : in  std_logic;                                -- Reset síncrono (activo alto)

        -- Entradas desde el submódulo menu_run_controller
        i_menu_run_exit_flag        : in  std_logic;                                -- Pulso si menu_run_handler quiere salir/cambiar

        -- Entradas desde el submódulo menu_configs_controller
        i_menu_config_exit_flag     : in  std_logic;                                -- Pulso si menu_config_handler quiere salir/cambiar

        -- Comandos RC5 (desde rc5_decoder.vhd)
        i_rc5_mode_change_en        : in  std_logic;                                -- Pulso para rc5_mode_change
        i_rc5_up_en                 : in  std_logic;                                -- Pulso para rc5_up
        i_rc5_down_en               : in  std_logic;                                -- Pulso para rc5_down

        -- Comandos de reset (desde un nivel superior o botón de reset físico)
        i_menu_reset_en             : in  std_logic;                                -- Pulso para resetear el menú

        -- Salidas de habilitación/control para los submódulos
        o_menu_run_active           : out std_logic;                                -- Activa el módulo menu_run
        o_menu_config_active        : out std_logic;                                -- Activa el módulo menu_configs

        -- Salidas de comandos para los submódulos
        o_menu_run_rc5_mode_change_en : out std_logic;                              -- Pulso para menu_run_mode_change
        o_menu_run_rc5_up_en        : out std_logic;                                -- Pulso para menu_run_up
        o_menu_run_rc5_down_en      : out std_logic;                                -- Pulso para menu_run_down
        o_menu_run_reset_en         : out std_logic;                                -- Pulso para menu_run_reset

        o_menu_config_reset_mode_en : out std_logic                                 -- Pulso para menu_config_reset_mode
    );
end entity menu_controller;

architecture Behavioral of menu_controller is

    -- =========================================================================
    -- ESTADOS DE LA MÁQUINA DE ESTADOS PRINCIPAL (menu_handler)
    -- =========================================================================
    type menu_fsm_state_type is (
        STATE_HANDLE_MENU,      -- Delegar el control al menú actual
        STATE_CHANGE_MENU       -- Cambiar al siguiente menú
    );
    signal current_menu_state : menu_fsm_state_type := STATE_HANDLE_MENU;
    signal next_menu_state    : menu_fsm_state_type;

    -- =========================================================================
    -- SEÑALES INTERNAS (equivalentes a variables 'static' en C)
    -- =========================================================================
    signal s_current_menu : menu_type := MENU_RUN_TYPE; -- Equivalente a current_menu en C

    -- Señales de pulso para las salidas
    signal s_o_menu_run_rc5_mode_change_en_pulse : std_logic := '0';
    signal s_o_menu_run_rc5_up_en_pulse        : std_logic := '0';
    signal s_o_menu_run_rc5_down_en_pulse      : std_logic := '0';
    signal s_o_menu_run_reset_en_pulse         : std_logic := '0';
    signal s_o_menu_config_reset_mode_en_pulse : std_logic := '0';

begin

    -- =========================================================================
    -- PROCESO PRINCIPAL DE LA MÁQUINA DE ESTADOS
    -- Controla qué menú está activo y maneja el cambio de menú.
    -- =========================================================================
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                current_menu_state <= STATE_HANDLE_MENU;
                s_current_menu     <= MENU_RUN_TYPE;
                s_o_menu_run_reset_en_pulse <= '1'; -- Resetea menu_run al inicio
                s_o_menu_config_reset_mode_en_pulse <= '1'; -- Resetea menu_config al inicio
            else
                current_menu_state <= next_menu_state;
                -- Borrar pulsos en cada ciclo
                s_o_menu_run_rc5_mode_change_en_pulse <= '0';
                s_o_menu_run_rc5_up_en_pulse        <= '0';
                s_o_menu_run_rc5_down_en_pulse      <= '0';
                s_o_menu_run_reset_en_pulse         <= '0';
                s_o_menu_config_reset_mode_en_pulse <= '0';

                -- Lógica para menu_reset()
                if i_menu_reset_en = '1' then
                    s_current_menu <= MENU_RUN_TYPE;
                    s_o_menu_run_reset_en_pulse <= '1'; -- menu_run_reset()
                end if;
            end if;
        end if;
    end process;

    process (current_menu_state, s_current_menu, i_menu_run_exit_flag, i_menu_config_exit_flag)
    begin
        next_menu_state <= current_menu_state;

        -- Control de activación de submódulos
        o_menu_run_active    <= '0';
        o_menu_config_active <= '0';

        case current_menu_state is
            when STATE_HANDLE_MENU =>
                case s_current_menu is
                    when MENU_RUN_TYPE =>
                        o_menu_run_active <= '1'; -- Activa el módulo MENU_RUN
                        if i_menu_run_exit_flag = '1' then -- Si menu_run_handler() devuelve true
                            -- (El código C original no tiene un 'return true' para MENU_RUN,
                            -- solo para MENU_CONFIG. Asumo que i_menu_run_exit_flag podría ser para otros fines.)
                            null; -- Por ahora, no hay cambio de menú desde MENU_RUN por su propio handler
                        end if;

                    when MENU_CONFIG_TYPE =>
                        o_menu_config_active <= '1'; -- Activa el módulo MENU_CONFIG
                        if i_menu_config_exit_flag = '1' then -- Si menu_config_handler() devuelve true (pulsación larga)
                            next_menu_state <= STATE_CHANGE_MENU;
                        end if;
                end case;

            when STATE_CHANGE_MENU =>
                s_o_menu_config_reset_mode_en_pulse <= '1'; -- menu_config_reset_mode()
                -- Cambiar al siguiente menú
                if s_current_menu = MENU_RUN_TYPE then
                    s_current_menu <= MENU_CONFIG_TYPE;
                else
                    s_current_menu <= MENU_RUN_TYPE;
                end if;
                next_menu_state <= STATE_HANDLE_MENU; -- Volver a delegar el control
        end case;
    end process;

    -- =========================================================================
    -- LÓGICA DE MANEJO DE COMANDOS RC5 (Passthrough)
    -- Traduce las llamadas RC5 a los handlers del menú activo.
    -- =========================================================================
    process (i_rc5_mode_change_en, i_rc5_up_en, i_rc5_down_en, s_current_menu)
    begin
        -- Por defecto, no se activa nada
        s_o_menu_run_rc5_mode_change_en_pulse <= '0';
        s_o_menu_run_rc5_up_en_pulse <= '0';
        s_o_menu_run_rc5_down_en_pulse <= '0';

        case s_current_menu is
            when MENU_RUN_TYPE =>
                -- rc5_mode_change(), rc5_up(), rc5_down() solo afectan a MENU_RUN
                if i_rc5_mode_change_en = '1' then
                    s_o_menu_run_rc5_mode_change_en_pulse <= '1';
                end if;
                if i_rc5_up_en = '1' then
                    s_o_menu_run_rc5_up_en_pulse <= '1';
                end if;
                if i_rc5_down_en = '1' then
                    s_o_menu_run_rc5_down_en_pulse <= '1';
                end if;
            when MENU_CONFIG_TYPE =>
                -- El código C no tiene manejo RC5 para MENU_CONFIG, así que no hacemos nada aquí.
                null;
        end case;
    end process;


    -- =========================================================================
    -- ASIGNACIONES DE SALIDA (Combinacional)
    -- =========================================================================
    o_menu_run_rc5_mode_change_en <= s_o_menu_run_rc5_mode_change_en_pulse;
    o_menu_run_rc5_up_en        <= s_o_menu_run_rc5_up_en_pulse;
    o_menu_run_rc5_down_en      <= s_o_menu_run_rc5_down_en_pulse;
    o_menu_run_reset_en         <= s_o_menu_run_reset_en_pulse;
    o_menu_config_reset_mode_en <= s_o_menu_config_reset_mode_en_pulse;

end architecture Behavioral;
