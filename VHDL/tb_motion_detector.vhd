library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_TEXTIO.ALL;
use STD.TEXTIO.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity tb_motion_detector is
--  Port ( );
end tb_motion_detector;

architecture Behavioral of tb_motion_detector is

    component motion_detector is
        Port ( clk : in STD_LOGIC;
               start : in STD_LOGIC;
               reset : in STD_LOGIC;
               pixelAvailable : in STD_LOGIC;
               pixelValue : in STD_LOGIC_VECTOR (7 downto 0);
               nextPixel : out STD_LOGIC;
               result : out STD_LOGIC;
               ready : out STD_LOGIC);
    end  component;
    
    -- Auxiliary signals
    signal s_clk : STD_LOGIC := '0';
    signal s_reset : STD_LOGIC := '0';
    signal s_start : STD_LOGIC := '0';
    signal s_pixelAvailable : STD_LOGIC := '0';
    signal s_pixelValue : STD_LOGIC_VECTOR(7 downto 0) := (others => '0');
    signal s_nextPixel : STD_LOGIC := '0';
    signal s_result : STD_LOGIC := '0';
    signal s_ready : STD_LOGIC := '0';
    signal s_pixel_counter : INTEGER range 0 to 4608000 := 0; -- 15 frames com 640*480 pixels vindo do videoTeste_Movendo.txt
    signal s_read_file_start : STD_LOGIC := '0';

begin

    s_clk <= not s_clk after 5 ns; -- T = 10 ns; f = 100 MHz
    s_reset <= '0','1' after 15 ns,'0' after 25 ns,'1' after 12683775 ns,'0' after 12683785 ns, '1' after 25367525 ns,'0' after 25367535 ns; --38083
    s_read_file_start <= '0','1' after 35 ns,'0' after 45 ns;
    s_start <= '0','1' after 45 ns,'0' after 55 ns,'1' after 12683795 ns,'0' after 12683805 ns, '1' after 25367545 ns,'0' after 25367555 ns;
    
    
    --s_pixelAvailable <= s_nextPixel;
    
    
    UUT: motion_detector
    port map (clk => s_clk,
              start => s_start,
              reset => s_reset,
              pixelAvailable => s_pixelAvailable,
              pixelValue => s_pixelValue,
              nextPixel => s_nextPixel,
              result => s_result,
              ready => s_ready);
    
    read_file : process(s_clk)
        --file in_file : TEXT open READ_MODE is "Parado.txt";
        file in_file : TEXT open READ_MODE is "Movendo.txt";
        variable in_line : LINE;
        variable data : STD_LOGIC_VECTOR(7 downto 0);
    begin
        if rising_edge(s_clk) then
            if not endfile(in_file) then
                if (s_nextPixel = '1' or s_read_file_start = '1') then
                    s_pixelAvailable <= '0';
                    readline(in_file,in_line);
                    read(in_line,data);
                    s_pixelValue <= data;
                    s_pixel_counter <= s_pixel_counter + 1;
                end if;
                s_pixelAvailable <= '1'; 
            elsif endfile(in_file) then
                report "ALL CONTENTS FROM FILE HAVE BEEN READ." severity note;    
            end if;
        end if;
    end process;


end Behavioral;
