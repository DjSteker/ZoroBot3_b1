
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- You might define a package for common utility types if you have many such functions
-- For simplicity, let's keep it self-contained for now.

entity utils_constrain is
    generic (
        -- G_DATA_WIDTH: Example for integer/fixed-point types
        -- If you're constraining floats, this would be more complex (e.g., IEEE 754 float representation)
        G_DATA_WIDTH : natural := 32
    );
    port (
        -- Input value to constrain
        i_x_signed   : in signed(G_DATA_WIDTH-1 downto 0);
        i_min_signed : in signed(G_DATA_WIDTH-1 downto 0);
        i_max_signed : in signed(G_DATA_WIDTH-1 downto 0);

        -- Output constrained value
        o_constrained_x_signed : out signed(G_DATA_WIDTH-1 downto 0)
    );
end entity utils_constrain;

architecture Behavioral of utils_constrain is
begin

    -- Combinatorial logic for the constrain function
    -- This directly translates the if-else logic from C
    process (i_x_signed, i_min_signed, i_max_signed)
    begin
        if i_x_signed > i_max_signed then
            o_constrained_x_signed <= i_max_signed;
        elsif i_x_signed < i_min_signed then
            o_constrained_x_signed <= i_min_signed;
        else
            o_constrained_x_signed <= i_x_signed;
        end if;
    end process;

end architecture Behavioral;
