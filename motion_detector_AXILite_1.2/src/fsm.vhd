-- Leonardo Brand�o Borges de Freitas - TCC2 - Detec��o de Movimento em V�deo

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
--use IEEE.STD_LOGIC_SIGNED.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity motion_detector is
    Port ( clk : in STD_LOGIC;
           start : in STD_LOGIC;
           reset : in STD_LOGIC;
           pixelAvailable : in STD_LOGIC_VECTOR (3 downto 0);
           pixelValue : in STD_LOGIC_VECTOR (7 downto 0);
           nextPixel : out STD_LOGIC;
           pixelPosition : out STD_LOGIC_VECTOR (3 downto 0); -- Apenas para testar o IP no overlay em python (recebe b)
           pixelAccumulator : out STD_LOGIC_VECTOR (15 downto 0); -- Apenas para testar o IP no overlay em python (recebe reg[0])
           motionAccumulator : out STD_LOGIC_VECTOR (10 downto 0); -- Apenas para testar o IP no overlay em python (recebe am)
           result : out STD_LOGIC;
           ready : out STD_LOGIC);
end motion_detector;

architecture Behavioral of motion_detector is
    
    -- Tipo: Vetor com posi��es de 16 bits
        -- Utilizado nos registradores de quadrantes (reg[40]).
        -- Cada posi��o deste vetor acumular� o valor dos 
        -- pixels por quadrante que pode atingir um valor 
        -- m�ximo de 65280, precisando de 16 bits, ou 2 bytes, 
        -- para representa-lo.
    --type uint16_v is array (natural range <>) of integer range 0 to 65280;
    type uint16_v is array (natural range <>) of unsigned (15 downto 0);
    
    -- Tipo: Matriz com posi��es de 9 bits representados em complemento de 2.
        -- Utilizado na matriz de m�dias dos quadrantes (MC[30][40]). 
        -- Matriz de 9 bits com sinal, pois cada posi��o desta matriz
        -- armazenar� em primeiro momento a m�dia do acumulado dos 
        -- quadrantes(0 a 255) e depois armazenar� a diferen�a entre 
        -- as m�dias dos quadrantes dos frames comparados (-255 a 255).
    --type int9_m is array (natural range <>, natural range <>) of integer range -255 to 255; 
    type int9_m is array (natural range <>, natural range <>) of signed (8 downto 0);

    -- Declara��o dos estado da FSM
    type states is(init,            -- Inicializador
                   getPixel,        -- L� Pixel
                   accumulator,     -- Acumulador
                   selIterator,     -- Itera Seletora
                   selZero,         -- Zera Seletora
                   shiftRight,      -- Shift Right
                   averageAlloc,    -- Aloca M�dia
                   averageDiff,     -- M�dias Diff
                   twoComplement,   -- Complemento de 2
                   threshold,       -- Limiariza��o
                   countOnes,
                   motionCheck);    -- Verifica Movimento
    
    -- Inicializando os sinais controladores dos estados:
	signal current_state, next_state : states := init;
	
	-- sinal para atualizar a sa�da nextPixel
	signal s_nextPixel : STD_LOGIC;
	
	-- sinal para atualizar a sa�da ready
	signal s_ready : STD_LOGIC;
	
	-- Sinal para atualizar a sa�da result
	signal s_result : STD_LOGIC;
	
	-- Contador de Bytes (0 <= b <= 16) (20 bits sem sinal)
    signal b : integer range 0 to 16 := 0;
    
    -- Seletora dos Registradores (0 <= sel < 40) (6 bits sem sinal)
    signal sel : integer range 0 to 40 := 0;
    
    -- Contador de Linhas de Pixels (0 <= lc <= 16) (4 bits sem sinal)
    signal lp : integer range 0 to 16 := 0;
    
    -- Contador de Linhas de Chunks (0 <= lc <= 30) (5 bits sem sinal)
    signal lc : integer range 0 to 30 := 0;
    
    -- Contador de Linhas da MC (0 <= l <= 30) (5 bits sem sinal)
    signal l : integer range 0 to 30 := 0;
    
    -- Contador de Colunas da MC (0 <= c <= 40) (6 bits sem sinal)
    --signal c : integer range 0 to 40 := 0;  
    
    -- Acumulador de Movimento (0 <= AM < 1200) (11 bits sem sinal)
    signal am : integer range 0 to 1200;
    
    -- Registradores do acumulado de pixels por quadrante (40 posi��es de 16 bits sem sinal)
    signal reg : uint16_v (0 to 39) := (others => (others => '0'));
    
    -- Matriz de M�dia dos Chunks (30x40 posi��es de 9 bits com sinal).
    signal MC : int9_m (0 to 29, 0 to 39) := (others => (others => (others => '0')));
    
    -- Vetor de Movimento (40 posi��es de 1 bit)
    signal VM : STD_LOGIC_VECTOR (0 to 39) := (others => '0');
    
    -- Limiar utilizado no estado threshold para determinar se houve movimento no chunk entre os frames comparados
    constant thresh : integer range 0 to 15 := 15;
    
    -- Fun��o auxiliar para contar a quantidade de 1's dentro do vetor de movimento (VM)
    -- Ou seja, essa fun��o verifica a quantidade de chunks em que houve movimento 
    function count_ones(s : STD_LOGIC_VECTOR) return integer is
        variable temp : natural := 0;
        begin
            for i in s'range loop
                if s(i) = '1' then 
                    temp := temp + 1; 
                end if;
            end loop;
            return temp;
        end function count_ones;
    
begin

    -- Processo s�ncrono respons�vel por:
        -- Atualizar o estado atual
    ATT_CURRENT_STATE : process (clk)
        begin 
        
            if rising_edge(clk) then
                if reset = '1' then
                    current_state <= init;
                else 
                    current_state <= next_state;      
                end if;
            end if;
            
        end process;
        
    -- Processo assincrono para definir qual ser� o proximo estado baseado 
        -- no estado atual e 
        -- nos sinais contadores
    ATT_NEXT_STATE : process (current_state, start, pixelAvailable, b, sel, lp, lc, l)
        begin    
        
            case current_state is
                
                -- Estado Init
                when init =>
                
                    if start = '1' then
                        next_state <= getPixel;
                    end if;

                -- Estado L�Pixel
                when getPixel =>
                
                    -- Se o novo byte estiver dispon�vel em pixelValue
                    if (b = to_integer(unsigned(pixelAvailable))) then
                        next_state <= accumulator;
                    end if;
     
                -- Estado Acumulador
                when accumulator =>
                    
                    -- Se ainda n�o completou a linha de pixels do chunk atual (16 bytes)
                    if (b < 15) then
                        -- Proximo estado ser� o Pega Pixel
                        next_state <= getPixel;
                    -- mas se j� completou a linha do chunk
                    elsif (b = 15) then
                        -- Proximo estado ser� o Itera Seletora
                        next_state <= selIterator;
                    end if;
                    
                -- Estado Itera Seletora
                when selIterator =>
                    
                    -- Enquanto n�o tiver acumulado em todos os chunks da linha de chunks
                    if (sel < 39) then
                        -- Proximo estado ser� o Pega Pixel
                        next_state <= getPixel;
                    -- Mas se a seletora chegar a 40
                    elsif (sel = 39) then
                        -- Proximo estado ser� o Zera Seletora
                        next_state <= selZero;
                    end if;
                    
                -- Estado Zera Seletora   
                when selZero =>
                
                    -- Enquanto n�o tiver preenchido as 16 linhas pixels nos registradores
                    if (lp < 15) then
                        -- Proximo estado ser� o Pega Pixel
                        next_state <= getPixel;
                    -- Mas se j� tiver preenchido as 16 linhas completas para os 40 registradores
                    elsif (lp = 15) then
                        -- Proximo estado ser� o shift right
                        next_state <= shiftRight;
                    end if;
                    
                -- Estado Shift Right
                when shiftRight =>
                
                    -- Enquanto n�o tiver alocado todas as linhas de chunks do primeiro frame
                    if (lc < 30) then
                        -- Proximo estado ser� o Aloca M�dias
                        next_state <= averageAlloc;
                    -- Mas se ja tiver alocado todos os chunks do primeiro frame
                    elsif (lc = 30) then
                        -- Proximo estado ser� o M�dias Diff
                        next_state <= averageDiff;
                    end if;
                    
                -- Estado Aloca M�dia    
                when averageAlloc =>
                
                    -- Proximo estado ser� o Pega Pixel
                    next_state <= getPixel;
                
                -- Estado M�dias Diff
                when averageDiff =>
                     
                    -- Proximo estado ser� o Complemento de 2
                    next_state <= twoComplement;
                
                -- Estado Complemento de 2
                when twoComplement =>
                
                    -- Proximo estado ser� Limiariza��o
                    next_state <= threshold;
                
                -- Estado Limiariza��o
                when threshold =>
                
                    -- Proximo estado ser� L�Pixel
                    next_state <= countOnes;
                    
                when countOnes =>
                
                    -- Enquanto n�o tiver comparado todas as linhas do segundo frame com as do primeiro frame
                    if l < 30 then
                        -- Proximo estado ser� L�Pixel
                        next_state <= getPixel;
                    elsif l = 30 then 
                        next_state <= motionCheck;
                    end if;
                            
                -- Estado Verifica Movimento
                when motionCheck =>
                    
                    -- Proximo estado ser� o init
                    next_state <= init;
                        
            end case; 
        
        
        end process;     

    -- Processo s�ncrono respons�vel por:
        -- Receber as entradas
        -- Atualizar os contadores e registradores
    ATT_SIGNALS : process (clk, current_state, reset, start, pixelAvailable, pixelValue)
     
        begin  
        
            if rising_edge(clk) then
            
                if reset = '1' then
                
                     -- Abaixa a flag ready
                    s_ready <= '0';
                    
                    -- Zera o resultado da detec��o
                    s_result <= '0';
                    
                    -- Abaixa a flag de pr�ximo pixel
                    s_nextPixel <= '0';
                    
                    -- Zera os contadores
                    b <= 0;
                    sel <= 0;
                    lp <= 0;
                    lc <= 0;
                    l <=0;
                    --c <= 0;
                    am <= 0;
                    
                    -- Zera os registradores 
                    reg <= (others => (others => '0'));
                    MC <= (others => (others => (others => '0')));
                    VM <= (others => '0');
                
                else
                
                    -- trata as subrotina de cada um dos estados da FSM
                    case current_state is
                        
                        -- Estado Init
                        when init =>
                        
                            -- Abaixa a flag ready
                            --s_ready <= '0';
                            
                            -- Zera o resultado da detec��o
                            --s_result <= '0';
                            
                            -- Abaixa a flag de pr�ximo pixel
                            s_nextPixel <= '0';
                            
                            -- Zera os contadores
                            b <= 0;
                            sel <= 0;
                            lp <= 0;
                            lc <= 0;
                            l <=0;
                            --c <= 0;
                            am <= 0;
                            
                            -- Zera os registradores 
                            reg <= (others => (others => '0'));
                            MC <= (others => (others => (others => '0')));
                            VM <= (others => '0');
        
                        -- Estado L�Pixel
                        when getPixel =>
                        
                            s_ready <= '0';
                            
                            s_result <= '0';
                            
                            s_nextPixel <= '1';
                                
                        -- Estado Acumulador
                        when accumulator =>
                            
                            -- abaixa a flag de pr�ximo pixel
                            s_nextPixel <= '0';
                        
                            -- Acumula o valor do pixel na posi��o sel do vetor reg
                            reg(sel) <= reg(sel) + unsigned(pixelValue);
                            
                            -- Incrementa o contador de bytes em uma unidade
                            b <= b + 1;
                       
                        -- Estado Itera Seletora
                        when selIterator =>
                        
                            -- Zera o contador de bytes
                            b <= 0;
                        
                            -- Incrementa a seletora para passar para o proximo chunk
                            sel <= sel + 1;
                            
                        -- Estado Zera Seletora   
                        when selZero =>
                            
                            -- Zera a seletora
                            sel <= 0;
                            
                            -- Incrementa o contador de linhas de pixels
                            lp <= lp + 1;
                            
                        -- Estado Shift Right
                        when shiftRight =>
                       
                            -- Zera o contador de linhas de pixels
                            lp <= 0;
                            
                            -- Executa o deslocamento a direita em todos os registradore
                            for i in 0 to (reg'length-1) loop 
                                reg(i) <= shift_right(reg(i), 8); 
                            end loop;
                            
                        -- Estado Aloca M�dia    
                        when averageAlloc =>
                        
                            -- para cada posi��o do registrador
                            for i in 0 to (reg'length-1) loop
                                -- aloca a m�dia na matriz de chunks
                                MC(lc, i) <=  signed(resize(reg(i), MC(lc, i)'length));
                                -- zera o registrador que foi alocado
                                reg(i) <= (others => '0');   
                            end loop;
                            
                            -- Incrementa o contador de linha de chunks
                            lc <= lc + 1;
                        
                        -- Estado M�dias Diff
                        when averageDiff =>
                        
                            -- para cada posi��o do registrador
                            for i in 0 to (reg'length-1) loop
                            
                                -- Subtrai a m�dia do quadrante do primeiro frame 
                                -- com a m�dia do quadrante do segundo frame
                                MC(l, i) <= MC(l, i) - signed(resize(reg(i), MC(l, i)'length));
                            
                                -- A posi��o do registrador � zerada
                                reg(i) <= (others => '0');
                                   
                            end loop;

                        -- Estado Complemento de 2
                        when twoComplement =>
                        
                            -- para cada posi��o do registrador
                            for i in 0 to (reg'length-1) loop
                            
                                -- Se a diferen�a entre as m�dias for negativa
                                if MC(l, i) < 0 then
                                    
                                    -- Positiva o valor 
                                    MC(l, i) <= -MC(l, i);
                                
                                end if;
                            
                            end loop;
                        
                        -- Estado Limiariza��o
                        when threshold =>
                        
                            -- para cada posi��o do registrador
                            for i in 0 to (reg'length-1) loop
                            
                                    -- Se a diferen�a entre os quadrantes for maior que o limiar (15) 
                                    if MC(l, i) > thresh then
                               
                                        -- A posi��o em quest�o do vetor de movimento vai para 1
                                        -- Indicando que naquele chunk h� movimento
                                        VM(i) <= '1';
                                        
                                    -- Se n�o,    
                                    else
                                    
                                        -- A posi��o em quest�o do vetor de movimento vai para 0
                                        -- Indicando que naquele chunk n�o h� movimento
                                        VM(i) <= '0';
                                        
                                    end if;
                                    
                            end loop;
                            
                            -- Itera o contador de linhas do segundo frame
                            l <= l + 1;
                            
                        when countOnes =>
                        
                            am <= am + count_ones(VM);

                        -- Estado Verifica Movimento
                        when motionCheck =>
                        
                            -- Se ao menos 1/4 dos 1200 quadrantes acusar movimento
                            if am >= 300 then
                            
                                -- Levanta a flag de movimento
                                s_result <= '1';
                            
                            -- Se n�o,
                            elsif am < 300 then
                            
                                -- Abaixa a flag de movimento
                                s_result <= '0';
                            
                            end if;
                        
                            -- Levanda a flag de que o resultado foi calculado
                            s_ready <= '1';
                            
                         --
                         when others =>
                         
                            s_ready <= '0';
                            
                            s_result <= '0';
                                
                    end case;   
                end if;
             end if;                                           
        end process;
        
   
    -- Atualiza as sa�das       
    nextPixel <= s_nextPixel;
    pixelPosition <= std_logic_vector(to_unsigned(b, pixelPosition'length));
    pixelAccumulator <= std_logic_vector(reg(0));
    motionAccumulator <= std_logic_vector(to_unsigned(am, motionAccumulator'length));
    result <= s_result;
    ready <= s_ready;
            
end Behavioral;
