
-- Librerías IEEE para tipos estándar y numéricos
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- !!! IMPORTANTE: La siguiente línea asumirá que defines tipos como 'kinematics_data_t'
-- en un paquete VHDL (ej. 'robot_types_pkg.vhd') para poder usarlos aquí.
-- use work.robot_types_pkg.all; -- Descomentar si usas un paquete de tipos personalizados

--------------------------------------------------------------------------------
-- Entidad: setup_system
-- Descripción: Módulo de nivel superior (top-level) que integra y configura
--              todos los periféricos del robot en hardware.
--              Equivalente al 'setup()' de C, pero en el dominio del hardware.
--
-- Entradas:
--   i_clk_crystal : Entrada del oscilador de cristal (ej. 8 MHz)
--   i_reset_n     : Reset activo bajo (ej. botón de reset)
--   -- Pines GPIO conectados al mundo exterior (ejemplos)
--   i_gpio_a_in   : Entrada para pines GPIO de Puerto A (ej. sensores analógicos)
--   o_gpio_a_out  : Salida para pines GPIO de Puerto A (ej. salidas digitales)
--   -- ... y así para todos los puertos GPIO y sus modos.
--   -- Aquí solo listamos un subconjunto representativo.
--   o_led_rgb_r   : Salida PWM para LED Rojo
--   o_led_rgb_g   : Salida PWM para LED Verde
--   o_led_rgb_b   : Salida PWM para LED Azul
--   o_motor_pwm_l : Salida PWM para Motor Izquierdo
--   o_motor_pwm_r : Salida PWM para Motor Derecho
--   i_encoder_l_a : Entrada Encoder Izq. Fase A
--   i_encoder_l_b : Entrada Encoder Izq. Fase B
--   i_encoder_r_a : Entrada Encoder Der. Fase A
--   i_encoder_r_b : Entrada Encoder Der. Fase B
--   i_rc5_exti_pin: Entrada del pin EXTI para RC5
--   i_usart3_rx   : Entrada RX para USART3
--   o_usart3_tx   : Salida TX para USART3
--   -- ... más puertos para SPI, ADC, etc.
--------------------------------------------------------------------------------
entity setup_system is
    port (
        -- Entradas de reloj y reset principales
        i_clk_crystal   : in  std_logic;
        i_reset_n       : in  std_logic; -- Reset activo bajo

        -- Pines GPIO (ejemplo, se expandiría para todos los pines usados)
        i_gpio_a_in     : in  std_logic_vector(7 downto 0); -- Ejemplo: GPIO A Pines 0-7
        o_gpio_a_out    : out std_logic_vector(7 downto 0); -- Ejemplo: GPIO A Pines 0-7
        o_gpio_a_en_out : out std_logic_vector(7 downto 0); -- Ejemplo: Habilitación de salida (para tristate)

        -- Salidas de PWM (ejemplos)
        o_led_rgb_r     : out std_logic; -- Salida de pin físico, típicamente una señal PWM
        o_led_rgb_g     : out std_logic;
        o_led_rgb_b     : out std_logic;
        o_motor_pwm_l   : out std_logic;
        o_motor_pwm_r   : out std_logic;
        o_fan_pwm       : out std_logic; -- Salida PWM para el ventilador (succión)

        -- Entradas de Encoder (ejemplos)
        i_encoder_l_a   : in  std_logic;
        i_encoder_l_b   : in  std_logic;
        i_encoder_r_a   : in  std_logic;
        i_encoder_r_b   : in  std_logic;

        -- USART3 Pines
        i_usart3_rx     : in  std_logic;
        o_usart3_tx     : out std_logic;

        -- MPU (SPI) Pines
        o_mpu_cs_n      : out std_logic;
        o_mpu_sclk      : out std_logic;
        o_mpu_mosi      : out std_logic;
        i_mpu_miso      : in  std_logic;

        -- ADC (Entradas analógicas simuladas, o interfaz a ADC real)
        i_adc_menu_in   : in std_logic_vector(11 downto 0); -- Ejemplo de entrada ADC de 12 bits
        i_adc_sensors_in: in std_logic_vector(31 downto 0); -- Múltiples sensores analógicos
        i_adc_battery_in: in std_logic_vector(11 downto 0);
        i_adc_motors_current_in: in std_logic_vector(23 downto 0);

        -- IR 38kHz (RC5)
        i_rc5_exti_pin  : in  std_logic -- Pin de entrada para la interrupción externa
    );
end entity setup_system;

architecture Structural of setup_system is

    -- --- Señales de Reloj (generadas por el PLL) ---
    signal s_clk_168mhz : std_logic; -- SYSCLK
    signal s_clk_ahb    : std_logic; -- HCLK (ej. 168MHz)
    signal s_clk_apb1   : std_logic; -- PCLK1 (ej. 42MHz)
    signal s_clk_apb2   : std_logic; -- PCLK2 (ej. 84MHz)
    signal s_reset      : std_logic; -- Reset sincronizado (activo alto)

    -- --- Señales internas para comunicación entre módulos ---
    -- Control Loop Signals
    signal s_main_control_loop_tick_1ms : std_logic; -- Pulso cada 1ms para control_loop()
    signal s_wall_sensor_manager_tick_16khz : std_logic; -- Pulso cada 16kHz para sm_emitter_adc()

    -- Encoder Signals (salidas del decodificador de cuadratura)
    signal s_left_encoder_ticks_diff  : signed(15 downto 0); -- Diferencia de ticks para el left encoder
    signal s_right_encoder_ticks_diff : signed(15 downto 0); -- Diferencia de ticks para el right encoder

    -- USART3 Signals
    signal s_usart3_rx_data_byte : std_logic_vector(7 downto 0);
    signal s_usart3_tx_data_byte : std_logic_vector(7 downto 0);
    signal s_usart3_rx_data_valid : std_logic;
    signal s_usart3_tx_data_request : std_logic;

    -- SPI3 Signals (para MPU)
    signal s_spi3_sclk    : std_logic;
    signal s_spi3_mosi    : std_logic;
    signal s_spi3_miso    : std_logic;
    signal s_spi3_cs_n    : std_logic; -- Chip Select
    signal s_spi3_tx_data : std_logic_vector(7 downto 0);
    signal s_spi3_rx_data : std_logic_vector(7 downto 0);
    signal s_spi3_tx_en   : std_logic;
    signal s_spi3_rx_valid: std_logic;

    -- ADC Signals (ejemplo: si son módulos ADC dedicados)
    signal s_adc1_data_out : std_logic_vector(11 downto 0);
    signal s_adc2_data_out : std_logic_vector(11 downto 0);
    signal s_adc1_eoc_irq  : std_logic; -- Fin de conversión
    signal s_dma_adc1_data_ready : std_logic; -- DMA ha movido datos

    -- PWM Signals (ciclo de trabajo en std_logic_vector o natural)
    signal s_led_r_pwm_duty   : natural range 0 to 255; -- 8-bit PWM
    signal s_led_g_pwm_duty   : natural range 0 to 255;
    signal s_led_b_pwm_duty   : natural range 0 to 255;
    signal s_motor_l_pwm_duty : natural range 0 to 1000; -- Ejemplo, depende de MOTORES_MAX_PWM
    signal s_motor_r_pwm_duty : natural range 0 to 1000;
    signal s_fan_pwm_duty     : natural range 0 to 255;

    -- RC5 Signals
    signal s_rc5_trigger_falling_edge : std_logic;
    signal s_rc5_trigger_rising_edge  : std_logic;
    signal s_rc5_command_out          : std_logic_vector(7 downto 0);
    signal s_rc5_data_valid           : std_logic;

    -- --- Instanciación de Componentes VHDL (declaraciones de componentes) ---
    -- Aquí declararías los 'entity' de tus módulos individuales
    -- Por ejemplo:
    -- component clock_gen_pll is ... end component;
    -- component gpio_interface is ... end component;
    -- component usart_module is ... end component;
    -- component spi_master_module is ... end component;
    -- component adc_interface_module is ... end component;
    -- component dma_controller_module is ... end component;
    -- component pwm_timer_module is ... end component;
    -- component quadrature_decoder is ... end component;
    -- component systick_timer is ... end component;
    -- component lsm6dsr_interface is ... end component; (para MPU)
    -- component rc5_decoder_module is ... end component;
    -- component irq_arbiter is ... end component; -- Para prioridades

begin
    -- --- Conexiones y Asignaciones ---
    -- Mapea el reset asíncrono activo-bajo a un reset sincronizado activo-alto para la lógica interna
    process (i_clk_crystal, i_reset_n)
    begin
        if i_reset_n = '0' then
            s_reset <= '1'; -- Reset activo alto
        elsif rising_edge(i_clk_crystal) then
            s_reset <= '0';
        end if;
    end process;

    -- 1. Generación de Relojes (setup_clock)
    -- Instanciar un IP core de PLL/MMCM del fabricante de tu FPGA
    -- CLK_GEN_PLL_INST : entity work.clock_gen_pll
    --     generic map (
    --         G_INPUT_FREQ_HZ   => 8_000_000,   -- i_clk_crystal
    --         G_OUTPUT_SYSCLK_HZ => 168_000_000 -- s_clk_168mhz
    --     )
    --     port map (
    --         i_clk_in    => i_clk_crystal,
    --         i_reset     => s_reset,
    --         o_sysclk    => s_clk_168mhz,
    --         o_clk_ahb   => s_clk_ahb,   -- HCLK (168MHz)
    --         o_clk_apb1  => s_clk_apb1,  -- PCLK1 (42MHz)
    --         o_clk_apb2  => s_clk_apb2   -- PCLK2 (84MHz)
    --     );

    -- Asignaciones directas para simulación si no se usa PLL real
    s_clk_168mhz <= i_clk_crystal; -- O una señal de reloj dividida manualmente
    s_clk_ahb    <= i_clk_crystal;
    s_clk_apb1   <= i_clk_crystal;
    s_clk_apb2   <= i_clk_crystal;

    -- 2. Configuración SysTick (setup_systick)
    -- SYSTICK_TIMER_INST : entity work.systick_timer
    --     generic map (
    --         G_SYSCLK_FREQ_HZ => 168_000_000, -- s_clk_168mhz
    --         G_TICK_FREQ_HZ   => SYSTICK_FREQUENCY_HZ -- Tu constante de 1ms (1000Hz)
    --     )
    --     port map (
    --         i_clk       => s_clk_168mhz,
    --         i_reset     => s_reset,
    --         o_tick_pulse => s_main_control_loop_tick_1ms -- Pulso cada 1ms
    --     );


    -- 3. Configuración de GPIO (setup_gpio)
    -- Cada grupo de pines con un modo similar podría ser un subsistema GPIO.
    -- Las asignaciones de pines físicos a funcionalidades se hacen aquí.
    -- Por ejemplo, para LEDs y Motores:
    -- LED_RGB_R_PWM_OUT <= s_led_r_pwm_output_from_timer; -- Conectado a la salida del timer PWM
    -- o_led_rgb_r <= s_led_r_pwm_output_from_timer; -- Ejemplo si el pin se conecta directamente
    -- Los demás pines se conectan de manera similar a sus respectivos módulos.

    -- 4. Configuración USART (setup_usart)
    -- USART_MODULE_INST : entity work.usart_module
    --     generic map (
    --         G_CLK_FREQ_HZ => natural(84_000_000), -- s_clk_apb2
    --         G_BAUD_RATE   => 115200
    --     )
    --     port map (
    --         i_clk        => s_clk_apb2,
    --         i_reset      => s_reset,
    --         i_rx_pin     => i_usart3_rx,
    --         o_tx_pin     => o_usart3_tx,
    --         o_rx_data    => s_usart3_rx_data_byte,
    --         o_rx_data_valid => s_usart3_rx_data_valid,
    --         i_tx_data    => s_usart3_tx_data_byte,
    --         i_tx_data_request => s_usart3_tx_data_request
    --     );

    -- 5. Configuración de ADC y DMA (setup_adc1, setup_adc2, setup_dma_adc1)
    -- Esto depende fuertemente de si usas ADC externos o simulas internos.
    -- Si simulas ADC internos de STM32, esto es muy complejo.
    -- ADC_INTERFACE_INST : entity work.adc_interface_module
    --     port map (
    --         i_clk         => s_clk_apb2,
    --         i_reset       => s_reset,
    --         i_analog_in   => i_adc_sensors_in(11 downto 0), -- Ejemplo: un canal de sensor
    --         o_digital_out => s_adc2_data_out,
    --         o_eoc_irq     => s_adc1_eoc_irq
    --     );
    -- DMA_CONTROLLER_INST : entity work.dma_controller_module
    --     port map (
    --         i_clk          => s_clk_ahb,
    --         i_reset        => s_reset,
    --         i_peripheral_data => s_adc1_data_out, -- Datos del ADC
    --         i_peripheral_request => s_adc1_eoc_irq, -- Request de DMA desde ADC
    --         o_memory_address => open, -- Dirección de memoria a escribir
    --         o_memory_data    => open, -- Datos a memoria
    --         i_memory_data    => open, -- Datos de memoria
    --         o_transfer_complete => s_dma_adc1_data_ready
    --     );

    -- 6. Configuración de PWM (setup_leds_pwm, setup_motors_pwm)
    -- PWM_LEDS_INST : entity work.pwm_timer_module
    --     generic map (
    --         G_CLK_FREQ_HZ  => natural(84_000_000), -- s_clk_apb2
    --         G_PWM_FREQ_HZ  => 20_000,
    --         G_MAX_DUTY     => 255 -- LEDS_MAX_PWM
    --     )
    --     port map (
    --         i_clk      => s_clk_apb2,
    --         i_reset    => s_reset,
    --         i_duty_r   => s_led_r_pwm_duty,
    --         i_duty_g   => s_led_g_pwm_duty,
    --         i_duty_b   => s_led_b_pwm_duty,
    --         i_duty_fan => s_fan_pwm_duty, -- Si el ventilador también usa TIM1
    --         o_pwm_r    => o_led_rgb_r,
    --         o_pwm_g    => o_led_rgb_g,
    --         o_pwm_b    => o_led_rgb_b,
    --         o_pwm_fan  => o_fan_pwm
    --     );
    -- PWM_MOTORS_INST : entity work.pwm_timer_module
    --     generic map (
    --         G_CLK_FREQ_HZ  => natural(84_000_000), -- s_clk_apb2
    --         G_PWM_FREQ_HZ  => 20_000,
    --         G_MAX_DUTY     => 1000 -- MOTORES_MAX_PWM
    --     )
    --     port map (
    --         i_clk      => s_clk_apb2,
    --         i_reset    => s_reset,
    --         i_duty_l   => s_motor_l_pwm_duty,
    --         i_duty_r   => s_motor_r_pwm_duty,
    --         o_pwm_l    => o_motor_pwm_l,
    --         o_pwm_r    => o_motor_pwm_r
    --     );


    -- 7. Configuración de Timers para Bucle Principal y Sensores de Pared (setup_main_loop_timer, setup_wall_sensor_manager)
    -- MAIN_LOOP_TIMER_INST : entity work.simple_timer_module
    --     generic map (
    --         G_CLK_FREQ_HZ  => natural(42_000_000), -- s_clk_apb1
    --         G_PERIOD_US    => 1000 -- 1ms
    --     )
    --     port map (
    --         i_clk       => s_clk_apb1,
    --         i_reset     => s_reset,
    --         o_pulse_out => s_main_control_loop_tick_1ms
    --     );
    -- WALL_SENSOR_TIMER_INST : entity work.simple_timer_module
    --     generic map (
    --         G_CLK_FREQ_HZ  => natural(42_000_000), -- s_clk_apb1
    --         G_PERIOD_US    => natural(1000000 / 16000) -- ~62.5 us for 16kHz
    --     )
    --     port map (
    --         i_clk       => s_clk_apb1,
    --         i_reset     => s_reset,
    --         o_pulse_out => s_wall_sensor_manager_tick_16khz
    --     );

    -- 8. Configuración de Encoders de Cuadratura (setup_quadrature_encoders)
    -- QUAD_DECODER_LEFT_INST : entity work.quadrature_decoder
    --     generic map (
    --         G_CLK_FREQ_HZ => natural(168_000_000) -- s_clk_168mhz
    --     )
    --     port map (
    --         i_clk       => s_clk_168mhz,
    --         i_reset     => s_reset,
    --         i_phase_a   => i_encoder_l_a,
    --         i_phase_b   => i_encoder_l_b,
    --         o_count_diff => s_left_encoder_ticks_diff
    --     );
    -- QUAD_DECODER_RIGHT_INST : entity work.quadrature_decoder
    --     generic map (
    --         G_CLK_FREQ_HZ => natural(168_000_000)
    --     )
    --     port map (
    --         i_clk       => s_clk_168mhz,
    --         i_reset     => s_reset,
    --         i_phase_a   => i_encoder_r_a,
    --         i_phase_b   => i_encoder_r_b,
    --         o_count_diff => s_right_encoder_ticks_diff
    --     );

    -- 9. MPU (LSM6DSR) setup_mpu
    -- INTERFACE_MPU_INST : entity work.lsm6dsr_interface
    --     port map (
    --         i_clk       => s_clk_ahb,
    --         i_reset     => s_reset,
    --         i_spi_sclk  => s_spi3_sclk,
    --         i_spi_mosi  => s_spi3_mosi,
    --         o_spi_miso  => s_spi3_miso,
    --         o_spi_cs_n  => s_spi3_cs_n,
    --         i_spi_rx_data => s_spi3_rx_data,
    --         i_spi_rx_valid => s_spi3_rx_valid,
    --         o_spi_tx_data => s_spi3_tx_data,
    --         o_spi_tx_request => s_spi3_tx_en,
    --         -- ... entradas/salidas de datos de acelerómetro/giróscopo
    --         o_gyro_z_radps => open
    --     );

    -- 10. Interrupción Externa (RC5)
    -- EXTI_RC5_INST : entity work.exti_trigger_detector
    --     port map (
    --         i_clk       => s_clk_168mhz,
    --         i_reset     => s_reset,
    --         i_exti_pin  => i_rc5_exti_pin,
    --         o_falling_edge_pulse => s_rc5_trigger_falling_edge,
    --         o_rising_edge_pulse  => s_rc5_trigger_rising_edge
    --     );
    -- RC5_DECODER_INST : entity work.rc5_decoder_module
    --     port map (
    --         i_clk       => s_clk_168mhz,
    --         i_reset     => s_reset,
    --         i_ir_pin    => i_rc5_exti_pin, -- o directamente la señal del detector de flancos
    --         o_command   => s_rc5_command_out,
    --         o_data_valid => s_rc5_data_valid
    --     );

    -- --- Lógica de lazo principal y gestores de ISR (control_loop, sm_emitter_adc) ---
    -- CONTROL_LOOP_MANAGER : entity work.control_loop_manager
    --     port map (
    --         i_clk => s_clk_168mhz,
    --         i_reset => s_reset,
    --         i_1ms_tick => s_main_control_loop_tick_1ms,
    --         i_encoder_l_diff => s_left_encoder_ticks_diff,
    --         i_encoder_r_diff => s_right_encoder_ticks_diff,
    --         -- ... otras entradas de sensores, salidas a motores
    --         o_motor_l_duty => s_motor_l_pwm_duty,
    --         o_motor_r_duty => s_motor_r_pwm_duty,
    --         o_fan_duty     => s_fan_pwm_duty
    --     );

    -- WALL_SENSOR_MANAGER : entity work.wall_sensor_manager
    --     port map (
    --         i_clk => s_clk_168mhz,
    --         i_reset => s_reset,
    --         i_16khz_tick => s_wall_sensor_manager_tick_16khz,
    --         -- ... entradas/salidas para control de emisores y lectura ADC de sensores
    --         o_emitter_control_out => open
    --     );


    -- --- Asignaciones de Salidas a los Pines Físicos ---
    -- Conecta las señales internas de los módulos a los pines de la entidad top-level
    o_led_rgb_r <= '0'; -- Placeholder, conectar a la salida del módulo PWM
    o_led_rgb_g <= '0';
    o_led_rgb_b <= '0';
    o_motor_pwm_l <= '0';
    o_motor_pwm_r <= '0';
    o_fan_pwm <= '0';
    o_usart3_tx <= '0'; -- Conectar a s_usart3_tx_pin del módulo USART
    o_mpu_cs_n <= '1'; -- Conectar a s_spi3_cs_n del módulo SPI
    o_mpu_sclk <= '0';
    o_mpu_mosi <= '0';

    -- Ejemplos de GPIO:
    -- o_gpio_a_out(0) <= some_internal_signal;
    -- o_gpio_a_en_out(0) <= '1'; -- Siempre habilitado como salida

end architecture Structural;
