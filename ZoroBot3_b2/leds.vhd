
-- leds_pkg.vhd (Package for common declarations)
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package leds_pkg is
    -- Constants for LED_MAX_PWM (adjust based on your PWM resolution)
    constant LEDS_MAX_PWM : integer := 255; -- Example for 8-bit PWM

    -- Define GPIO pins (replace with actual pin assignments for your Cyclone II)
    -- These would be output ports in your top-level entity
    subtype T_GPIO_PORT_C is std_logic_vector(15 downto 13);
    subtype T_GPIO_PORT_B is std_logic_vector(14 downto 0); -- Assuming the C code refers to individual bits, adjust size as needed

    -- Type for RGB color
    type T_RGB_COLOR is record
        r, g, b : std_logic_vector(7 downto 0); -- Assuming 8-bit PWM for RGB
    end record;

end package leds_pkg;

-- ---

-- leds_controller.vhd (Main entity)
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.leds_pkg.all;

entity leds_controller is
    port (
        clk             : in  std_logic;
        reset_n         : in  std_logic;
        battery_level_in : in  std_logic_vector(7 downto 0); -- Example: 0-100 scaled to 8 bits
        
        -- Individual LED outputs
        status_led_out  : out std_logic;
        
        -- RGB LED outputs (PWM signals)
        rgb_r_pwm       : out std_logic;
        rgb_g_pwm       : out std_logic;
        rgb_b_pwm       : out std_logic;
        
        -- Info LED outputs (assuming groups for simplicity, adjust as per actual wiring)
        gpio_c_out      : out T_GPIO_PORT_C; -- Example: GPIOC[15:13]
        gpio_b_out      : out T_GPIO_PORT_B  -- Example: GPIOB[14, 9:8, 2:0]
    );
end entity leds_controller;

architecture rtl of leds_controller is
    -- Internal signals for clock ticks (like get_clock_ticks)
    signal clock_ticks_counter : natural range 0 to natural'high := 0;
    
    -- Internal signals for LED states and timers (corresponding to static variables in C)
    signal last_ticks_rainbow    : natural := 0;
    signal rainbow_rgb_s         : T_RGB_COLOR := (r => to_slv(LEDS_MAX_PWM, 8), g => (others => '0'), b => (others => '0'));
    signal rainbow_color_desc    : natural range 0 to 2 := 0;
    signal rainbow_color_asc     : natural range 0 to 2 := 1;
    
    signal rgb_while_ms          : natural := 0;
    
    signal last_ticks_warning    : natural := 0;
    
    signal last_blink_rgb        : natural := 0;
    signal blink_rgb_state       : boolean := false;
    
    signal last_ticks_wave       : natural := 0;
    signal current_step_wave     : integer range -1 to 1 := 1;
    signal current_index_wave    : natural range 0 to 4 := 0;
    
    signal last_tick_side_sensors : natural := 0;
    signal current_step_side_sensors : integer range -1 to 1 := 1;
    signal current_index_side_sensors : natural range 0 to 1 := 0;

    signal last_tick_front_sensors : natural := 0;
    signal current_step_front_sensors : integer range -1 to 1 := 1;
    signal current_index_front_sensors : natural range 0 to 1 := 0;
    
    signal last_ticks_leds_blink : natural := 0;
    
    signal last_ticks_warning_bateria : natural := 0;

    -- Internal signals to hold current RGB values
    signal current_r_pwm_val : std_logic_vector(7 downto 0) := (others => '0');
    signal current_g_pwm_val : std_logic_vector(7 downto 0) := (others => '0');
    signal current_b_pwm_val : std_logic_vector(7 downto 0) := (others => '0');

    -- Internal signals for GPIO control
    signal gpio_c_internal : std_logic_vector(15 downto 13) := (others => '0');
    signal gpio_b_internal : std_logic_vector(14 downto 0) := (others => '0'); -- Placeholder, define individual bits clearly

    -- Component for PWM generation (you'd need to create this)
    component pwm_generator is
        port (
            clk         : in  std_logic;
            reset_n     : in  std_logic;
            duty_cycle  : in  std_logic_vector(7 downto 0);
            pwm_out     : out std_logic
        );
    end component pwm_generator;

begin

    -- Clock Tick Generator Process (Replaces get_clock_ticks())
    process (clk, reset_n)
    begin
        if reset_n = '0' then
            clock_ticks_counter <= 0;
        elsif rising_edge(clk) then
            clock_ticks_counter <= clock_ticks_counter + 1;
        end if;
    end process;

    -- Map internal GPIO signals to output ports (example, adjust for your actual wiring)
    gpio_c_out(15) <= gpio_c_internal(15);
    gpio_c_out(14) <= gpio_c_internal(14);
    gpio_c_out(13) <= gpio_c_internal(13);

    -- For GPIOB, you'd need to explicitly map each bit used in the C code
    -- Example for GPIO14, GPIO9, GPIO8, GPIO2, GPIO1, GPIO0
    gpio_b_out(14) <= gpio_b_internal(14); 
    gpio_b_out(9)  <= gpio_b_internal(9);
    gpio_b_out(8)  <= gpio_b_internal(8);
    gpio_b_out(2)  <= gpio_b_internal(2);
    gpio_b_out(1)  <= gpio_b_internal(1);
    gpio_b_out(0)  <= gpio_b_internal(0);
    -- And so on for other GPIOB pins

    -- Instantiate PWM generators for RGB
    pwm_r_inst : pwm_generator
    port map (
        clk         => clk,
        reset_n     => reset_n,
        duty_cycle  => current_r_pwm_val,
        pwm_out     => rgb_r_pwm
    );

    pwm_g_inst : pwm_generator
    port map (
        clk         => clk,
        reset_n     => reset_n,
        duty_cycle  => current_g_pwm_val,
        pwm_out     => rgb_g_pwm
    );

    pwm_b_inst : pwm_generator
    port map (
        clk         => clk,
        reset_n     => reset_n,
        duty_cycle  => current_b_pwm_val,
        pwm_out     => rgb_b_pwm
    );

    -- Main LED Control Process
    process (clk, reset_n)
        -- Helper function to set individual GPIOs
        function f_set_gpio(current_val : std_logic_vector; bit_index : natural; state : boolean) return std_logic_vector is
            variable temp_val : std_logic_vector(current_val'range) := current_val;
        begin
            if state then
                temp_val(bit_index) := '1';
            else
                temp_val(bit_index) := '0';
            end if;
            return temp_val;
        end function;

        -- Helper function to toggle individual GPIOs
        function f_toggle_gpio(current_val : std_logic_vector; bit_index : natural) return std_logic_vector is
            variable temp_val : std_logic_vector(current_val'range) := current_val;
        begin
            temp_val(bit_index) := not current_val(bit_index);
            return temp_val;
        end function;

    begin
        if reset_n = '0' then
            -- Reset all signals to initial states
            status_led_out <= '0';
            current_r_pwm_val <= (others => '0');
            current_g_pwm_val <= (others => '0');
            current_b_pwm_val <= (others => '0');
            gpio_c_internal <= (others => '0');
            gpio_b_internal <= (others => '0');
            
            last_ticks_rainbow <= 0;
            rainbow_rgb_s      <= (r => to_slv(LEDS_MAX_PWM, 8), g => (others => '0'), b => (others => '0'));
            rainbow_color_desc <= 0;
            rainbow_color_asc  <= 1;
            
            rgb_while_ms       <= 0;
            last_ticks_warning <= 0;
            last_blink_rgb     <= 0;
            blink_rgb_state    <= false;
            last_ticks_wave    <= 0;
            current_step_wave  <= 1;
            current_index_wave <= 0;
            last_tick_side_sensors <= 0;
            current_step_side_sensors <= 1;
            current_index_side_sensors <= 0;
            last_tick_front_sensors <= 0;
            current_step_front_sensors <= 1;
            current_index_front_sensors <= 0;
            last_ticks_leds_blink <= 0;
            last_ticks_warning_bateria <= 0;

        elsif rising_edge(clk) then
            -- Implement the C functions as synchronous logic

            -- set_status_led, toggle_status_led, warning_status_led
            -- This would typically be driven by external inputs or an FSM for specific modes
            -- For demonstration, let's assume a 'mode' signal for different behaviors

            -- Example for warning_status_led (you'd integrate this into a larger FSM)
            -- if current_mode = WARNING_MODE then
            --    if clock_ticks_counter > last_ticks_warning + ms_delay_for_warning then
            --        status_led_out <= not status_led_out;
            --        last_ticks_warning <= clock_ticks_counter;
            --    end if;
            -- end if;

            -- set_RGB_color and set_RGB_color_while
            -- These would be triggered by external signals or an FSM
            -- For simplicity, let's assume inputs `set_rgb_trigger` and `rgb_r_in`, `rgb_g_in`, `rgb_b_in`
            -- if set_rgb_trigger = '1' then
            --     current_r_pwm_val <= rgb_r_in;
            --     current_g_pwm_val <= rgb_g_in;
            --     current_b_pwm_val <= rgb_b_in;
            --     rgb_while_ms <= clock_ticks_counter + to_natural(ms_for_while_effect); -- Convert ms to clock ticks
            -- end if;

            -- check_leds_while
            if rgb_while_ms > 0 and clock_ticks_counter > rgb_while_ms then
                current_r_pwm_val <= (others => '0');
                current_g_pwm_val <= (others => '0');
                current_b_pwm_val <= (others => '0');
                rgb_while_ms <= 0;
            end if;

            -- blink_RGB_color (similar FSM integration)
            -- if current_mode = BLINK_RGB_MODE then
            --     if clock_ticks_counter > last_blink_rgb + ms_blink_rgb_delay then
            --         blink_rgb_state <= not blink_rgb_state;
            --         if blink_rgb_state then
            --             current_r_pwm_val <= (others => '0');
            --             current_g_pwm_val <= (others => '0');
            --             current_b_pwm_val <= (others => '0');
            --         else
            --             current_r_pwm_val <= rgb_r_blink_val;
            --             current_g_pwm_val <= rgb_g_blink_val;
            --             current_b_pwm_val <= rgb_b_blink_val;
            --         end if;
            --         last_blink_rgb <= clock_ticks_counter;
            --     end if;
            -- end if;

            -- set_RGB_rainbow
            -- if current_mode = RAINBOW_MODE then
            --     if clock_ticks_counter > last_ticks_rainbow + 10 then -- 10 clock ticks for example
            --         last_ticks_rainbow <= clock_ticks_counter;
            --
            --         -- Decrement rainbowRGB[rainbowColorDesc]
            --         if unsigned(rainbow_rgb_s(rainbow_color_desc)) > 20 then
            --             rainbow_rgb_s(rainbow_color_desc) <= std_logic_vector(unsigned(rainbow_rgb_s(rainbow_color_desc)) - 20);
            --         else
            --             rainbow_rgb_s(rainbow_color_desc) <= (others => '0');
            --         end if;
            --
            --         -- Increment rainbowRGB[rainbowColorAsc]
            --         if unsigned(rainbow_rgb_s(rainbow_color_asc)) < (LEDS_MAX_PWM - 20) then
            --             rainbow_rgb_s(rainbow_color_asc) <= std_logic_vector(unsigned(rainbow_rgb_s(rainbow_color_asc)) + 20);
            --         else
            --             rainbow_rgb_s(rainbow_color_asc) <= to_slv(LEDS_MAX_PWM, 8);
            --         end if;
            --
            --         current_r_pwm_val <= rainbow_rgb_s.r;
            --         current_g_pwm_val <= rainbow_rgb_s.g;
            --         current_b_pwm_val <= rainbow_rgb_s.b;
            --
            --         if (unsigned(rainbow_rgb_s(rainbow_color_desc)) <= 0) or (unsigned(rainbow_rgb_s(rainbow_color_asc)) >= LEDS_MAX_PWM) then
            --             rainbow_rgb_s(rainbow_color_desc) <= (others => '0');
            --             rainbow_rgb_s(rainbow_color_asc)  <= to_slv(LEDS_MAX_PWM, 8);
            --
            --             rainbow_color_desc <= (rainbow_color_desc + 1) mod 3;
            --             if rainbow_color_desc = 2 then
            --                 rainbow_color_asc <= 0;
            --             else
            --                 rainbow_color_asc <= rainbow_color_desc + 1;
            --             end if;
            --         end if;
            --     end if;
            -- end if;

            -- set_leds_wave, set_leds_side_sensors, set_leds_front_sensors, set_leds_blink
            -- These will involve a similar timing mechanism and 'case' or 'if/else if' statements
            -- to control the `gpio_c_internal` and `gpio_b_internal` signals.
            -- Example for set_leds_wave:
            -- if current_mode = WAVE_MODE then
            --     if clock_ticks_counter > last_ticks_wave + ms_wave_delay then
            --         last_ticks_wave <= clock_ticks_counter;
            --
            --         -- Clear all relevant info LEDs first
            --         gpio_c_internal(4) <= '0'; gpio_c_internal(5) <= '0';
            --         gpio_b_internal(0) <= '0'; gpio_b_internal(1) <= '0'; gpio_b_internal(2) <= '0';
            --         gpio_c_internal(15) <= '0'; gpio_c_internal(14) <= '0'; gpio_c_internal(13) <= '0';
            --         gpio_b_internal(9) <= '0'; gpio_b_internal(8) <= '0';
            --
            --         case current_index_wave is
            --             when 0 =>
            --                 gpio_c_internal(4) <= '1';
            --                 gpio_c_internal(5) <= '1';
            --             when 1 =>
            --                 gpio_b_internal(0) <= '1';
            --                 gpio_b_internal(1) <= '1';
            --             when 2 =>
            --                 gpio_b_internal(2) <= '1';
            --                 gpio_c_internal(15) <= '1';
            --             when 3 =>
            --                 gpio_c_internal(14) <= '1';
            --                 gpio_c_internal(13) <= '1';
            --             when 4 =>
            --                 gpio_b_internal(9) <= '1';
            --                 gpio_b_internal(8) <= '1';
            --             when others => null;
            --         end case;
            --
            --         if current_index_wave >= 4 then
            --             current_step_wave <= -1;
            --         elsif current_index_wave <= 0 then
            --             current_step_wave <= 1;
            --         end if;
            --         current_index_wave <= current_index_wave + current_step_wave;
            --     end if;
            -- end if;

            -- set_leds_battery_level
            -- This would be a combinational or sequential block reacting to battery_level_in
            -- It would set/clear `gpio_c_internal` and `gpio_b_internal` based on the level.
            -- if battery_level_in <= to_slv(10, 8) then
            --    gpio_b_internal(0) <= '0'; ... etc.
            --    if clock_ticks_counter > last_ticks_warning_bateria + 50 then
            --        gpio_b_internal(2) <= not gpio_b_internal(2);
            --        gpio_c_internal(15) <= not gpio_c_internal(15);
            --        last_ticks_warning_bateria <= clock_ticks_counter;
            --    end if;
            -- ... else if ...
            -- end if;

            -- all_leds_clear, set_info_led, set_info_leds, clear_info_leds
            -- These would be specific states or triggered by external inputs in your FSM.
            -- For example, `all_leds_clear` would set all relevant signals to '0'.
            -- `set_info_led` would use the f_set_gpio helper function based on `index` and `state`.
        end if;
    end process;

    -- You would implement an FSM here to control the different LED modes
    -- and call/enable the logic for each function based on the current state.

end architecture rtl;

-- ---

-- pwm_generator.vhd (Example PWM Component - you would need to implement this)
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity pwm_generator is
    port (
        clk         : in  std_logic;
        reset_n     : in  std_logic;
        duty_cycle  : in  std_logic_vector(7 downto 0); -- 0-255
        pwm_out     : out std_logic
    );
end entity pwm_generator;

architecture behavioral of pwm_generator is
    signal counter : unsigned(7 downto 0) := (others => '0'); -- 8-bit counter for PWM
begin
    process (clk, reset_n)
    begin
        if reset_n = '0' then
            counter <= (others => '0');
            pwm_out <= '0';
        elsif rising_edge(clk) then
            counter <= counter + 1;
            if counter < unsigned(duty_cycle) then
                pwm_out <= '1';
            else
                pwm_out <= '0';
            end if;
        end if;
    end process;
end architecture behavioral;
