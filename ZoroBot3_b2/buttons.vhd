
-- buttons.vhd
-- Este módulo VHDL gestiona la lógica de los botones del robot,
-- incluyendo el debounceo y la provisión de sus estados.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Incluye el paquete de tipos y constantes personalizados,
-- que contiene las definiciones para AUX_MENU_BTN_ID y otras.
library work;
use work.robot_types_pkg.all;

entity buttons is
    port (
        i_clk             : in  std_logic;                                 -- Reloj del sistema
        i_rst_n           : in  std_logic;                                 -- Reset asíncrono activo-bajo
        i_get_clock_ticks : in  unsigned(31 downto 0);                     -- get_clock_ticks() - Contador de milisegundos del sistema

        -- Entrada del botón analógico (simula get_aux_raw(AUX_MENU_BTN_ID))
        i_btn_analog_raw  : in  unsigned(15 downto 0);                     -- Valor raw del ADC para el botón analógico

        -- Entradas de control para set_debug_btn (desde otros módulos)
        i_set_debug_btn_en: in  std_logic;                                 -- Pulso para habilitar set_debug_btn
        i_set_debug_btn_val: in std_logic;                                 -- Valor para set_debug_btn (true/false)

        -- Salidas de estado de los botones (equivalentes a get_menu_up_btn, etc.)
        o_get_menu_up_btn   : out std_logic;                               -- Estado del botón de menú arriba
        o_get_menu_down_btn : out std_logic;                               -- Estado del botón de menú abajo
        o_get_menu_mode_btn : out std_logic;                               -- Estado del botón de menú modo
        o_get_debug_btn     : out std_logic                                -- Estado del botón de depuración
    );
end entity buttons;

architecture rtl of buttons is

    -- Señales internas (equivalentes a variables 'static' en C)
    signal s_btn_analog     : unsigned(15 downto 0) := (others => '0');

    signal s_btn_menu_up_ms   : unsigned(31 downto 0) := (others => '0');
    signal s_btn_menu_down_ms : unsigned(31 downto 0) := (others => '0');
    signal s_btn_menu_mode_ms : unsigned(31 downto 0) := (others => '0');

    signal s_debug_btn      : boolean := false;

    -- Constante para el umbral de debounce
    constant C_DEBOUNCE_MS : unsigned(31 downto 0) := to_unsigned(50, 32);

begin

    -- Proceso principal para la función check_buttons()
    process (i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            s_btn_analog     <= (others => '0');
            s_btn_menu_up_ms   <= (others => '0');
            s_btn_menu_down_ms <= (others => '0');
            s_btn_menu_mode_ms <= (others => '0');
            s_debug_btn      <= false;
        elsif rising_edge(i_clk) then
            -- Actualiza la lectura analógica del botón
            s_btn_analog <= i_btn_analog_raw;

            -- Lógica para btn_menu_up_ms (1300 <= btn_analog <= 1700)
            if s_btn_analog >= to_unsigned(1300, 16) and s_btn_analog <= to_unsigned(1700, 16) then
                if s_btn_menu_up_ms = (others => '0') then
                    s_btn_menu_up_ms <= i_get_clock_ticks;
                end if;
            else
                s_btn_menu_up_ms <= (others => '0');
            end if;

            -- Lógica para btn_menu_mode_ms (2300 <= btn_analog <= 2700)
            if s_btn_analog >= to_unsigned(2300, 16) and s_btn_analog <= to_unsigned(2700, 16) then
                if s_btn_menu_mode_ms = (others => '0') then
                    s_btn_menu_mode_ms <= i_get_clock_ticks;
                end if;
            else
                s_btn_menu_mode_ms <= (others => '0');
            end if;

            -- Lógica para btn_menu_down_ms (btn_analog >= 3700)
            if s_btn_analog >= to_unsigned(3700, 16) then
                if s_btn_menu_down_ms = (others => '0') then
                    s_btn_menu_down_ms <= i_get_clock_ticks;
                end if;
            else
                s_btn_menu_down_ms <= (others => '0');
            end if;

            -- Lógica para set_debug_btn
            if i_set_debug_btn_en = '1' then
                s_debug_btn <= (i_set_debug_btn_val = '1');
            end if;

        end if;
    end process;

    -- Lógica para get_menu_up_btn()
    -- return btn_menu_up_ms > 0 && get_clock_ticks() - btn_menu_up_ms > 50;
    o_get_menu_up_btn <= '1' when (s_btn_menu_up_ms /= (others => '0') and
                                   (i_get_clock_ticks >= s_btn_menu_up_ms and
                                    (i_get_clock_ticks - s_btn_menu_up_ms) > C_DEBOUNCE_MS) or
                                   (i_get_clock_ticks < s_btn_menu_up_ms and -- Handle overflow
                                    (to_unsigned(2**32-1, 32) - s_btn_menu_up_ms) + i_get_clock_ticks > C_DEBOUNCE_MS))
                           else '0';

    -- Lógica para get_menu_down_btn()
    -- return btn_menu_down_ms > 0 && get_clock_ticks() - btn_menu_down_ms > 50;
    o_get_menu_down_btn <= '1' when (s_btn_menu_down_ms /= (others => '0') and
                                     (i_get_clock_ticks >= s_btn_menu_down_ms and
                                      (i_get_clock_ticks - s_btn_menu_down_ms) > C_DEBOUNCE_MS) or
                                     (i_get_clock_ticks < s_btn_menu_down_ms and -- Handle overflow
                                      (to_unsigned(2**32-1, 32) - s_btn_menu_down_ms) + i_get_clock_ticks > C_DEBOUNCE_MS))
                            else '0';

    -- Lógica para get_menu_mode_btn()
    -- return btn_menu_mode_ms > 0 && get_clock_ticks() - btn_menu_mode_ms > 50;
    o_get_menu_mode_btn <= '1' when (s_btn_menu_mode_ms /= (others => '0') and
                                     (i_get_clock_ticks >= s_btn_menu_mode_ms and
                                      (i_get_clock_ticks - s_btn_menu_mode_ms) > C_DEBOUNCE_MS) or
                                     (i_get_clock_ticks < s_btn_menu_mode_ms and -- Handle overflow
                                      (to_unsigned(2**32-1, 32) - s_btn_menu_mode_ms) + i_get_clock_ticks > C_DEBOUNCE_MS))
                            else '0';

    -- Lógica para get_debug_btn()
    -- return debug_btn;
    o_get_debug_btn <= '1' when s_debug_btn else '0';

end architecture rtl;
