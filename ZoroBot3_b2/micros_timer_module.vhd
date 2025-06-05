
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

--------------------------------------------------------------------------------
-- Entity: micros_timer_module
-- Description: Implements a non-blocking microsecond timer similar to TramaTiempo.
--              It uses a free-running microsecond counter and triggers a 'done' pulse.
--              Handles 32-bit unsigned rollover.
--
-- Generics:
--   G_CLK_FREQ_HZ : The frequency of the input clock in Hz (e.g., 100_000_000 for 100MHz)
--
-- Inputs:
--   i_clk            : Master clock signal
--   i_reset          : Asynchronous reset signal
--   i_set_interval_en: Pulse to load a new interval
--   i_interval_us    : The interval in microseconds (equivalent to TramaTiempo::interval)
--   i_timer_enable   : Actively enables the timer check (equivalent to TramaTiempo::active)
--
-- Outputs:
--   o_timer_done     : Pulse output when the interval has expired (equivalent to execute())
--   o_current_micros : Current free-running microsecond counter (for debugging/monitoring)
--------------------------------------------------------------------------------
entity micros_timer_module is
    generic (
        G_CLK_FREQ_HZ : natural := 100_000_000 -- Default to 100 MHz clock
    );
    port (
        i_clk            : in  std_logic;
        i_reset          : in  std_logic;
        i_set_interval_en: in  std_logic; -- Enable to set new interval
        i_interval_us    : in  natural;   -- Interval in microseconds
        i_timer_enable   : in  std_logic; -- Enable for this specific timer (active flag)

        o_timer_done     : out std_logic; -- Output pulse when timer expires
        o_current_micros : out unsigned(31 downto 0) -- Free-running microsecond counter
    );
end entity micros_timer_module;

architecture Behavioral of micros_timer_module is

    -- Free-running microsecond counter
    signal s_micros_counter : unsigned(31 downto 0) := (others => '0'); -- Equivalent to micros()
    constant C_CLK_CYCLES_PER_US : real := real(G_CLK_FREQ_HZ) / 1_000_000.0;
    -- For integer division, need to be careful if CLK_FREQ_HZ is not a multiple of 1MHz.
    -- Assuming a system where G_CLK_FREQ_HZ is an integer multiple of 1_000_000 (e.g., 100MHz, 50MHz)
    -- If not, then fractional parts must be handled (more complex).

    -- Internal signals for TramaTiempo logic
    signal s_previous_micros : unsigned(31 downto 0) := (others => '0'); -- Equivalent to TramaTiempo::previous
    signal s_interval_us     : unsigned(31 downto 0) := (others => '0'); -- Equivalent to TramaTiempo::interval

    -- Signal to capture the 'done' condition combinatorially
    signal s_done_condition  : std_logic;

begin

    -- Free-running microsecond counter (micros())
    process (i_clk)
        variable v_clk_count : natural := 0;
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                s_micros_counter <= (others => '0');
                v_clk_count := 0;
            else
                v_clk_count := v_clk_count + 1;
                -- Increment microsecond counter when enough clock cycles have passed
                if v_clk_count >= natural(C_CLK_CYCLES_PER_US) then
                    s_micros_counter <= s_micros_counter + 1;
                    v_clk_count := 0;
                end if;
            end if;
        end if;
    end process;
    o_current_micros <= s_micros_counter;

    -- TramaTiempo logic for setting interval and checking
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                s_previous_micros <= (others => '0');
                s_interval_us     <= (others => '0');
            else
                -- Load new interval and reset 'previous' on 'set_interval_en' pulse
                if i_set_interval_en = '1' then
                    s_interval_us     <= to_unsigned(i_interval_us, s_interval_us'length);
                    s_previous_micros <= s_micros_counter; -- Reset 'previous' to current micros()
                end if;

                -- If the timer was just 'done' in the previous cycle, reset s_previous_micros
                -- This ensures a new interval starts immediately after the previous one finishes.
                if s_done_condition = '1' then
                    s_previous_micros <= s_micros_counter;
                end if;
            end if;
        end if;
    end process;

    -- Combinatorial logic for the 'check' function (including rollover handling)
    -- This implements: if (active && (micros() - previous >= interval))
    -- Or: if (active && micros() < previous) { // Rollover case
    --       unsigned long TMr = (4294967295 - previous);
    --       if (TMr < interval) { previous = interval - TMr; } else { execute(); }
    --     }
    -- In VHDL, unsigned arithmetic naturally handles rollover for comparison
    s_done_condition <= '1' when (i_timer_enable = '1') and
                                 ((s_micros_counter - s_previous_micros) >= s_interval_us)
                         else '0';

    -- Output a pulse for one clock cycle when the done condition is met
    -- This creates a single-cycle pulse for 'o_timer_done'
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                o_timer_done <= '0';
            else
                o_timer_done <= s_done_condition;
            end if;
        end if;
    end process;

end architecture Behavioral;
