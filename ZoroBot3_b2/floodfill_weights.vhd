
-- floodfill_weights.vhd
-- Este módulo VHDL implementa las funciones de cálculo de pesos para el algoritmo Floodfill.
-- Utiliza aritmética de punto fijo para las operaciones flotantes.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.robot_types_pkg.all; -- Incluye tipos personalizados y configuración de punto fijo

entity floodfill_weights is
    port (
        i_clk                       : in  std_logic;                                -- Reloj del sistema
        i_reset                     : in  std_logic;                                -- Reset síncrono (activo alto)

        -- Inputs for floodfill_weights_table
        i_calc_en                   : in  std_logic;                                -- Pulso para iniciar el cálculo de la tabla
        i_distance                  : in  T_FIXED_POINT;                            -- float distance (fixed-point)
        i_init_speed                : in  natural range 0 to 65535;                 -- uint16_t init_speed
        i_max_speed                 : in  natural range 0 to 65535;                 -- uint16_t max_speed
        i_accel                     : in  natural range 0 to 65535;                 -- uint16_t accel
        i_cells_to_max_speed        : in  natural range 0 to C_MAX_FLOODFILL_CELLS; -- uint16_t cells_to_max_speed

        -- Output for the calculated weights table
        o_weights_out               : out T_CELL_WEIGHT_ARRAY;                      -- struct cell_weigth *weights_out
        o_calc_done                 : out std_logic                                 -- Pulso cuando el cálculo está completo
    );
end entity floodfill_weights;

architecture Behavioral of floodfill_weights is

    -- =========================================================================
    -- FSM STATES FOR CALCULATION
    -- =========================================================================
    type calc_fsm_state_type is (
        STATE_IDLE,                 -- Esperando comando de cálculo
        STATE_FLOODFILL_INIT,       -- Inicializa la primera celda
        STATE_FLOODFILL_LOOP,       -- Bucle principal de cálculo de celdas
        STATE_FLOODFILL_DONE        -- Cálculo completo
    );
    signal current_state : calc_fsm_state_type := STATE_IDLE;
    signal next_state    : calc_fsm_state_type;

    -- =========================================================================
    -- INTERNAL SIGNALS (equivalent to static variables and loop variables)
    -- =========================================================================
    signal s_loop_idx          : natural range 0 to C_MAX_FLOODFILL_CELLS; -- Loop index 'i'
    signal s_current_speed     : natural range 0 to 65535;
    signal s_current_time      : T_FIXED_POINT;
    signal s_current_total_time: T_FIXED_POINT;
    signal s_current_penalty   : T_FIXED_POINT;

    -- Internal storage for the output table
    signal s_weights_out_internal : T_CELL_WEIGHT_ARRAY;

    -- Constants converted to fixed-point for calculations
    constant C_FP_0_5 : T_FIXED_POINT := to_fixed_signed(5) / 10; -- 0.5 as fixed-point
    constant C_FP_2_0 : T_FIXED_POINT := to_fixed_signed(2);   -- 2.0 as fixed-point
    constant C_FP_4_0 : T_FIXED_POINT := to_fixed_signed(4);   -- 4.0 as fixed-point


    -- =========================================================================
    -- HELPER FUNCTIONS (for fixed-point arithmetic)
    -- These are simplified. Full fixed-point libraries are complex.
    -- Assuming T_FIXED_POINT is Q(N.M) where N is integer bits, M is frac bits.
    -- We assume 32-bit signed fixed point, so integer part has 32-C_FIXED_POINT_FRAC_BITS bits.
    -- =========================================================================

    -- Fixed-point division (A / B)
    function fixed_div(A, B : T_FIXED_POINT) return T_FIXED_POINT is
    begin
        -- To avoid overflow, shift A left before dividing, then compensate
        -- This is a very simplified division. Real fixed-point division is more complex.
        -- Consider using a dedicated fixed-point division IP or robust library.
        if B = 0 then return (others => '0'); end if; -- Avoid division by zero
        return resize((A * C_FIXED_POINT_SCALE) / B, A'length);
    end function;

    -- Fixed-point multiplication (A * B)
    function fixed_mul(A, B : T_FIXED_POINT) return T_FIXED_POINT is
    begin
        return resize((A * B) / C_FIXED_POINT_SCALE, A'length);
    end function;

    -- Fixed-point square root (simplified, usually requires dedicated IP or iterative algorithm)
    -- This is a very basic placeholder and will not work for real square roots.
    -- For real applications, use a CORDIC algorithm or an Altera IP core.
    function fixed_sqrt(A : T_FIXED_POINT) return T_FIXED_POINT is
        variable res : T_FIXED_POINT;
    begin
        -- Placeholder: For real designs, implement a sqrt algorithm (e.g., CORDIC)
        -- Or use Intel FPGA's Floating-Point IP Core if needed.
        -- For now, returning 0 or a very rough estimate.
        -- A proper fixed-point SQRT is iterative and would take many cycles.
        -- This function would likely be implemented as a separate process/module.
        if A < 0 then
            return (others => '0'); -- Error or NaN
        else
            -- Very crude approximation for example
            res := to_fixed(integer(unsigned(A) / C_FIXED_POINT_SCALE)); -- Takes integer part sqrt
            -- A proper fixed-point sqrt:
            -- e.g., Newton-Raphson or bit-by-bit method over multiple clock cycles.
            -- This is a major implementation detail for real designs.
            return res;
        end if;
    end function;

    -- Second degree equation solver (using fixed-point arithmetic)
    -- return (float)((-b + sqrt(delta)) / (2 * a));
    function second_degree_equation_fp(a_fp, b_fp, c_fp : T_FIXED_POINT) return T_FIXED_POINT is
        variable delta_val_fp : T_FIXED_POINT;
        variable sqrt_delta_fp : T_FIXED_POINT;
        variable result_fp : T_FIXED_POINT;
    begin
        -- delta = b * b - 4 * a * c;
        delta_val_fp := fixed_mul(b_fp, b_fp) - fixed_mul(fixed_mul(C_FP_4_0, a_fp), c_fp);

        if delta_val_fp < 0 then
            return to_fixed_signed(-1); -- Placeholder for negative delta
        else
            sqrt_delta_fp := fixed_sqrt(delta_val_fp); -- *** Placeholder for actual sqrt implementation ***
            -- return (float)((-b + sqrt(delta)) / (2 * a));
            result_fp := fixed_div((0 - b_fp) + sqrt_delta_fp, fixed_mul(C_FP_2_0, a_fp));
            return result_fp;
        end if;
    end function;

    -- time_penalty(uint16_t speed, uint16_t init_speed, uint16_t accel)
    function time_penalty_fp(speed_val, init_speed_val, accel_val : natural) return T_FIXED_POINT is
        variable diff_speed_fp : T_FIXED_POINT;
    begin
        if init_speed_val < speed_val then
            diff_speed_fp := to_fixed(speed_val - init_speed_val);
            return fixed_div(diff_speed_fp, to_fixed(accel_val));
        else
            return (others => '0'); -- 0.0f
        end if;
    end function;

    -- time_taken_distance(float distance, uint16_t speed, uint16_t max_speed, uint16_t accel)
    function time_taken_distance_fp(distance_fp : T_FIXED_POINT; speed_val, max_speed_val, accel_val : natural) return T_FIXED_POINT is
        variable time_to_max_speed_fp : T_FIXED_POINT;
        variable time_to_distance_fp  : T_FIXED_POINT;
        variable left_distance_fp     : T_FIXED_POINT;
        variable return_time_fp       : T_FIXED_POINT;
        variable speed_val_fp         : T_FIXED_POINT;
        variable max_speed_val_fp     : T_FIXED_POINT;
    begin
        speed_val_fp    := to_fixed(speed_val);
        max_speed_val_fp := to_fixed(max_speed_val);

        if speed_val < max_speed_val then
            -- time_to_max_speed = (max_speed - speed) / (float)accel;
            time_to_max_speed_fp := fixed_div(max_speed_val_fp - speed_val_fp, to_fixed(accel_val));

            -- time_to_distance = second_degree_equation(0.5f * accel, speed, -distance);
            time_to_distance_fp := second_degree_equation_fp(
                fixed_mul(C_FP_0_5, to_fixed(accel_val)),
                speed_val_fp,
                0 - distance_fp
            );

            if time_to_distance_fp < time_to_max_speed_fp then
                return_time_fp := time_to_distance_fp;
            else
                -- left_distance = distance - (speed * time_to_max_speed) + (0.5f * accel * time_to_max_speed * time_to_max_speed);
                left_distance_fp := distance_fp -
                                   fixed_mul(speed_val_fp, time_to_max_speed_fp) +
                                   fixed_mul(fixed_mul(C_FP_0_5, to_fixed(accel_val)), fixed_mul(time_to_max_speed_fp, time_to_max_speed_fp));
                return_time_fp := time_to_max_speed_fp + fixed_div(left_distance_fp, max_speed_val_fp);
            end if;
        else
            -- distance / (float)speed;
            return_time_fp := fixed_div(distance_fp, speed_val_fp);
        end if;
        return return_time_fp;
    end function;

    -- speed_after_time(float time, uint16_t speed, uint16_t max_speed, uint16_t accel)
    function speed_after_time_fp(time_fp : T_FIXED_POINT; speed_val, max_speed_val, accel_val : natural) return natural is
        variable speed_at_time_fp : T_FIXED_POINT;
        variable result_speed     : natural;
        variable speed_val_fp     : T_FIXED_POINT;
        variable max_speed_val_fp : T_FIXED_POINT;
    begin
        speed_val_fp    := to_fixed(speed_val);
        max_speed_val_fp := to_fixed(max_speed_val);

        if speed_val < max_speed_val then
            -- speed_at_time = speed + (accel * time);
            speed_at_time_fp := speed_val_fp + fixed_mul(to_fixed(accel_val), time_fp);

            if speed_at_time_fp > max_speed_val_fp then
                result_speed := max_speed_val;
            else
                -- Convert fixed-point back to natural (integer part)
                result_speed := to_integer(speed_at_time_fp / C_FIXED_POINT_SCALE);
            end if;
        else
            result_speed := max_speed_val;
        end if;
        return result_speed;
    end function;

begin

    -- =========================================================================
    -- FSM PROCESS
    -- =========================================================================
    process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                current_state <= STATE_IDLE;
                s_loop_idx <= 0;
                s_current_speed <= 0;
                s_current_time <= (others => '0');
                s_current_total_time <= (others => '0');
                s_current_penalty <= (others => '0');
                o_calc_done <= '0';
            else
                current_state <= next_state;

                -- Default output pulse reset
                o_calc_done <= '0';

                case current_state is
                    when STATE_IDLE =>
                        if i_calc_en = '1' then
                            s_loop_idx <= 0;
                            s_current_speed <= i_init_speed;
                            s_current_total_time <= (others => '0'); -- Will be calculated in init
                            s_current_penalty <= (others => '0'); -- Will be calculated in init
                            next_state <= STATE_FLOODFILL_INIT;
                        end if;

                    when STATE_FLOODFILL_INIT =>
                        -- Equivalent to weights_out[0] calculation
                        -- time = distance / speed;
                        s_current_time := fixed_div(i_distance, to_fixed(s_current_speed));
                        s_current_total_time := s_current_time;
                        s_current_penalty := (others => '0'); -- penalty is 0 for first cell

                        s_weights_out_internal(0).speed      <= s_current_speed;
                        s_weights_out_internal(0).time       <= s_current_time;
                        s_weights_out_internal(0).total_time <= s_current_total_time;
                        s_weights_out_internal(0).penalty    <= s_current_penalty;

                        s_loop_idx <= 1; -- Prepare for loop starting from i = 1
                        if i_cells_to_max_speed = 0 then -- Handle empty loop case
                             next_state <= STATE_FLOODFILL_DONE;
                        else
                             next_state <= STATE_FLOODFILL_LOOP;
                        end if;

                    when STATE_FLOODFILL_LOOP =>
                        if s_loop_idx < i_cells_to_max_speed then
                            -- time = time_taken_distance(distance, speed, max_speed, accel);
                            s_current_time := time_taken_distance_fp(i_distance, s_current_speed, i_max_speed, i_accel);

                            -- total_time += time;
                            s_current_total_time := s_current_total_time + s_current_time;

                            -- speed = speed_after_time(time, speed, max_speed, accel);
                            s_current_speed := speed_after_time_fp(s_current_time, s_current_speed, i_max_speed, i_accel);

                            -- penalty = time_penalty(speed, init_speed, accel);
                            s_current_penalty := time_penalty_fp(s_current_speed, i_init_speed, i_accel);

                            -- Store results in the array
                            s_weights_out_internal(s_loop_idx).speed      <= s_current_speed;
                            s_weights_out_internal(s_loop_idx).time       <= s_current_time;
                            s_weights_out_internal(s_loop_idx).total_time <= s_current_total_time;
                            s_weights_out_internal(s_loop_idx).penalty    <= s_current_penalty;

                            s_loop_idx <= s_loop_idx + 1; -- Increment loop index
                            next_state <= STATE_FLOODFILL_LOOP; -- Stay in loop
                        else
                            next_state <= STATE_FLOODFILL_DONE; -- Loop finished
                        end if;

                    when STATE_FLOODFILL_DONE =>
                        o_calc_done <= '1'; -- Signal completion
                        next_state <= STATE_IDLE; -- Return to idle

                    when others =>
                        next_state <= STATE_IDLE;
                end case;
            end if;
        end if;
    end process;

    -- Assign internal storage to output port
    o_weights_out <= s_weights_out_internal;

end architecture Behavioral;
