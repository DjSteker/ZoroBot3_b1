library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Package for common types (you might already have this or need to create it)
-- Example robot_types_pkg.vhd:
-- package robot_types_pkg is
--     -- Define constants for timer limits if needed
--     constant C_MAX_UINT32 : natural := 4294967295; -- 2^32 - 1
-- end package robot_types_pkg;
-- library work;
-- use work.robot_types_pkg.all;

entity task_scheduler is
    generic (
        G_NUM_TIMERS : natural := 5 -- Number of TramaTiempo instances to manage
    );
    port (
        i_clk        : in  std_logic;                                      -- System clock
        i_rst_n      : in  std_logic;                                      -- Asynchronous active-low reset
        i_micros_tick: in  unsigned(31 downto 0);                          -- Current microsecond tick (from an external counter)
        i_set_interval: in  std_logic_vector(G_NUM_TIMERS-1 downto 0);    -- Pulse to set interval for a specific timer
        i_interval   : in  unsigned(31 downto 0);                          -- New interval value
        i_timer_idx  : in  natural range 0 to G_NUM_TIMERS-1;              -- Index of the timer to configure

        i_reset_timer: in  std_logic_vector(G_NUM_TIMERS-1 downto 0);      -- Pulse to reset previous time for a specific timer
        i_enable_timer: in std_logic_vector(G_NUM_TIMERS-1 downto 0);     -- Pulse to enable specific timer
        i_disable_timer: in std_logic_vector(G_NUM_TIMERS-1 downto 0);    -- Pulse to disable specific timer

        o_task_trigger: out std_logic_vector(G_NUM_TIMERS-1 downto 0)     -- Output pulses for each task when due
    );
end entity task_scheduler;

architecture rtl of task_scheduler is

    type t_timer_state_array is array (0 to G_NUM_TIMERS-1) of unsigned(31 downto 0);
    signal s_previous_micros : t_timer_state_array;
    signal s_interval        : t_timer_state_array;

    type t_active_array is array (0 to G_NUM_TIMERS-1) of boolean;
    signal s_active          : t_active_array;

    -- Local signal for maximum 32-bit unsigned value
    constant C_MAX_UINT32_VAL : unsigned(31 downto 0) := to_unsigned(4294967295, 32);

begin

    process (i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            for i in 0 to G_NUM_TIMERS-1 loop
                s_previous_micros(i) <= (others => '0');
                s_interval(i)        <= to_unsigned(1, 32); -- Default interval 1 microsecond (like default constructor)
                s_active(i)          <= true;
                o_task_trigger(i)    <= '0';
            end loop;
        elsif rising_edge(i_clk) then
            -- Default to no triggers
            o_task_trigger <= (others => '0');

            for i in 0 to G_NUM_TIMERS-1 loop
                -- Update previous and interval based on setup calls
                if i_set_interval(i) = '1' and i = i_timer_idx then
                    s_interval(i) <= i_interval;
                end if;

                -- Reset previous time (similar to reset() method)
                if i_reset_timer(i) = '1' then
                    s_previous_micros(i) <= i_micros_tick;
                end if;

                -- Enable/Disable timer
                if i_enable_timer(i) = '1' then
                    s_active(i) <= true;
                elsif i_disable_timer(i) = '1' then
                    s_active(i) <= false;
                end if;

                -- Check method logic (similar to TramaTiempo::check())
                if s_active(i) then
                    -- Normal operation: micros() - previous >= interval
                    if i_micros_tick >= s_previous_micros(i) then
                        if (i_micros_tick - s_previous_micros(i)) >= s_interval(i) then
                            s_previous_micros(i) <= i_micros_tick;
                            o_task_trigger(i)    <= '1'; -- Trigger the task
                        end if;
                    -- Handle overflow: micros() < previous
                    else
                        -- unsigned long TMr = (4294967295 - previous);
                        -- if (TMr < interval) { previous = interval - TMr; }
                        -- else { previous = micros(); execute(); }
                        -- This is equivalent to: if (current_micros + (MAX_UINT32 - previous_micros) + 1) >= interval
                        -- The `+1` is for the wrap-around.
                        -- Or simply, if (current_micros - previous_micros) overflows and still >= interval.
                        -- In VHDL, subtraction of unsigned values automatically handles modulo arithmetic,
                        -- so `i_micros_tick - s_previous_micros(i)` will correctly give the wrapped difference.
                        if (i_micros_tick - s_previous_micros(i)) >= s_interval(i) then
                            s_previous_micros(i) <= i_micros_tick;
                            o_task_trigger(i)    <= '1'; -- Trigger the task
                        end if;
                    end if;
                end if;
            end loop;
        end if;
    end process;

end architecture rtl;
