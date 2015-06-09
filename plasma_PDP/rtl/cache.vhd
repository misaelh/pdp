---------------------------------------------------------------------
-- TITLE: Cache Controller
-- AUTHOR: Steve Rhoads (rhoadss@yahoo.com)
-- DATE CREATED: 12/22/08
-- FILENAME: cache.vhd
-- PROJECT: Plasma CPU core
-- COPYRIGHT: Software placed into the public domain by the author.
--    Software 'as is' without warranty.  Author liable for nothing.
-- DESCRIPTION:
--    4KB unified cache that uses the lower 4KB of the 8KB cache_ram.  
--    Only lowest 2MB of DDR is cached.
---------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
library UNISIM;
use UNISIM.vcomponents.all;
use work.mlite_pack.all;

entity cache is
   port(clk            		: in  std_logic;
        reset          		: in  std_logic;
        address_next   		: in  std_logic_vector(31 downto 2);
        byte_we_next   		: in  std_logic_vector(3 downto 0);
        cpu_address    		: in  std_logic_vector(31 downto 2);
        mem_busy       		: in  std_logic;
		
        cache_ram_enable  	: in  std_logic;
        cache_ram_byte_we 	: in  std_logic_vector(15 downto 0);
        cache_ram_address 	: in  std_logic_vector(31 downto 2);
        cache_ram_address_out   : out std_logic_vector(20 downto 4);
        cache_ram_data_w  	: in  std_logic_vector(127 downto 0);
        cache_ram_data_r  	: out std_logic_vector(127 downto 0);

        cache_access   		: out std_logic;   --access 4KB cache
        cache_checking 		: out std_logic;   --checking if cache hit
        cache_miss     		: out std_logic;  --cache miss
        cache_wb     		: out std_logic;  --cache wb
        cache_wb_hold  		: out std_logic;  --cache wb
        stall_comp              : out std_logic);
end; --cache

architecture logic of cache is
   subtype state_type is std_logic_vector(2 downto 0);
   constant STATE_IDLE     : state_type := "000";
   constant STATE_CHECKING : state_type := "001";
   constant STATE_MISSED   : state_type := "010";
   constant STATE_WAITING  : state_type := "011";
   constant STATE_WB       : state_type := "100";
   constant STATE_WB_RD    : state_type := "101";
   constant STATE_WB_WAIT  : state_type := "110";
   constant STATE_WB_WR    : state_type := "111";

   signal state_reg        : state_type;
   signal state            : state_type;
   signal state_next       : state_type;

   signal stall_cpu        : std_logic;
   
   signal cache_address    : std_logic_vector(9 downto 0);
   signal cache_tag_in     : std_logic_vector(8 downto 0);
   signal cache_tag_reg    : std_logic_vector(8 downto 0);
   signal cache_tag_out    : std_logic_vector(8 downto 0);
   signal cache_di         : std_logic_vector(15 downto 0);
   signal cache_do         : std_logic_vector(15 downto 0);
   signal cache_we         : std_logic;
   signal byte_we          : std_logic_vector(3 downto 0);
   signal byte_we_hold     : std_logic_vector(3 downto 0);
   signal cache_ram_address_reg   : std_logic_vector(20 downto 4);

   signal c_stall       : std_logic;
   signal c_stall_ff    : std_logic;
   signal c_miss        : std_logic;  --cache miss
   signal c_checking    : std_logic;
   signal c_nothit      : std_logic;  --cache miss
   signal c_enable  	: std_logic;
   signal c_byte_we 	: std_logic_vector(15 downto 0);
   signal c_address 	: std_logic_vector(31 downto 2);
   signal c_address_out : std_logic_vector(20 downto 4);
   signal c_data_w  	: std_logic_vector(127 downto 0);
   signal c_data_r  	: std_logic_vector(127 downto 0);

   signal not_dirty_in         : std_logic;
   signal not_dirty_out        : std_logic;
   --signal cache_ram_data_r128 : std_logic_vector(127 downto 0);
begin

   cache_proc: process(clk, reset, mem_busy, cache_address, 
      state_reg, state, state_next, not_dirty_out, cache_ram_address_reg,  byte_we, byte_we_hold,
      address_next, byte_we_next, cache_tag_in, --Stage1
      cache_tag_reg, cache_tag_out,             --Stage2
      cpu_address)                              --Stage3
   begin
      not_dirty_in <= not_dirty_out;
      cache_wb <= '0';
      cache_wb_hold <= '0';
      c_miss <= '0';
      c_checking <= '0';

      case state_reg is
      when STATE_IDLE =>            --cache idle
         c_checking <= '0';
         c_miss <= '0'; 
         state <= STATE_IDLE;
      when STATE_CHECKING =>        --current read in cached range, check if match
         c_checking <= '1';
         --cache_ram_address_out <=
         if cache_tag_out /= cache_tag_reg or cache_tag_out = ONES(8 downto 0) then
            if not_dirty_out = '0' then
              cache_wb <= '1';
              c_miss <= '1';
              state <= STATE_WB;
            else
              c_miss <= '1';
              state <= STATE_MISSED;
            end if;
         else
            c_miss <= '0';
            state <= STATE_IDLE;
         end if;
      when STATE_MISSED =>          --current read cache miss
         c_checking <= '0';
         c_miss <= '1';
         if mem_busy = '1' then
            state <= STATE_MISSED;
         else
            if byte_we_hold = "0000" then
               state <= STATE_WAITING;
            else
               state <= STATE_WB_WAIT;
            end if;
            --state <= STATE_WAITING;
         end if;
      when STATE_WAITING =>         --waiting for memory access to complete
         c_checking <= '0';
         c_miss <= '0';
         if mem_busy = '1' then
            state <= STATE_WAITING;
         else
            state <= STATE_IDLE;
         end if;
      when STATE_WB =>
         c_miss <= '1';
         cache_wb <= '1';
         if mem_busy = '1' then
            state <= STATE_WB;
         else
            state <= STATE_WB_RD;
         end if;
      when STATE_WB_RD =>
         c_miss <= '1';
         cache_wb_hold <= '1';
         if mem_busy = '1' then
            state <= STATE_WB_RD;
         else
            cache_wb_hold <= '0';
            state <= STATE_WB_WAIT;
         end if;
      when STATE_WB_WAIT =>
         c_miss <= '0';
         cache_wb_hold <= '0';
         if mem_busy = '1' then
            state <= STATE_WB_WAIT;
         else
            if byte_we_hold = "0000" then
               state <= STATE_IDLE;
            else
               state <= STATE_WB_WR;
            end if;
         end if;
      when STATE_WB_WR =>
        state <= STATE_IDLE;
      when others =>
         c_checking <= '0';
         c_miss <= '0';
         state <= STATE_IDLE;
      end case; --state

      if state = STATE_IDLE  then    --check if next access in cached range
         cache_address <= "00" & address_next(11 downto 4);
         if address_next(30 downto 21) = "0010000000" then  --first 2MB of DDR
            cache_access <= '1';
            --if byte_we_next = "0000" then     --read cycle
               cache_we <= '0';
               state_next <= STATE_CHECKING;  --need to check if match
            --else
            --   cache_we <= '1';               --update cache tag
            --   state_next <= STATE_WAITING;
            --end if;
         else
            cache_access <= '0';
            cache_we <= '0';
            state_next <= STATE_IDLE;
         end if;
         --if stall_cpu = '1' then
         --  cache_access <= '0';
         --  cache_we <= '0';
         --  state_next <= STATE_IDLE;
         --end if;
      else
         cache_address <= "00" & cpu_address(11 downto 4);
         cache_access <= '0';
         state_next <= state;
      end if;

      --if state_reg = STATE_CHECKING and  byte_we_next /= "0000" then
      if state = STATE_WAITING or state = STATE_WB_WAIT then
         cache_we <= '1';                  --update cache tag
         if byte_we_hold = "0000" then
            not_dirty_in <= '1';
         else
            not_dirty_in <= '0';
         end if;
      else
         if  (state /= STATE_MISSED and state /= STATE_WB and byte_we /= "0000") then
            cache_we <= '1';
            not_dirty_in <= '0';
         else
            cache_we <= '0';
         end if;
      end if;

      --if byte_we_next = "0000" then  --read or 32-bit write
         cache_tag_in <= address_next(20 downto 12);
      --else
      --   cache_tag_in <= ONES(8 downto 0);  --invalid tag
      --end if;

      if reset = '1' then
         state_reg <= STATE_IDLE;
         cache_tag_reg <= ZERO(8 downto 0);
         byte_we <= ZERO(3 downto 0);
         cache_ram_address_reg <= ZERO(20 downto 4);
         byte_we_hold <= ZERO(3 downto 0);
         c_stall_ff <= '0';
      elsif rising_edge(clk) then
         state_reg <= state_next;
         --cache_ram_address_reg <= cache_ram_address_reg;
         byte_we <= byte_we_next;

         if state_reg = STATE_WB_WR then
            c_stall_ff <= '1';
         else
            c_stall_ff <= '0';
         end if;

         if state_reg = STATE_CHECKING then
           byte_we_hold <= byte_we;
         end if;
         if state = STATE_IDLE and state_reg /= STATE_MISSED then
           cache_tag_reg <= cache_tag_in;
         end if;
         if state = STATE_WB then
           cache_ram_address_reg <= cache_tag_out & address_next(11 downto 4);
         end if;
      end if;

   end process;

      not_dirty_out <= cache_do(15);
      cache_tag_out <= cache_do(8 downto 0);
      cache_ram_address_out <= cache_ram_address_reg;
      cache_di <= not_dirty_in & ZERO(5 downto 0) & cache_tag_in(8 downto 0);


    cache_tag: RAMB16_S18  --Xilinx specific
        generic map (
        INIT => X"FFF", -- Value of output RAM registers at startup
        SRVAL => X"000", -- Ouput value upon SSR assertion
        --WRITE_MODE => "WRITE_FIRST", -- WRITE_FIRST, READ_FIRST or NO_CHANGE
        -- The following INIT_xx declarations specify the initial contents of the RAM
        -- Address 0 to 511
        INIT_00 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_01 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_02 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_03 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_04 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_05 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_06 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_07 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_08 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_09 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_0A => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_0B => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_0C => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_0D => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_0E => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_0F => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        -- Address 512 to 1023
        INIT_10 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_11 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_12 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_13 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_14 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_15 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_16 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_17 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_18 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_19 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_1A => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_1B => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_1C => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_1D => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_1E => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_1F => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        -- Address 1024 to 1535
        INIT_20 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_21 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_22 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_23 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_24 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_25 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_26 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_27 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_28 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_29 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_2A => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_2B => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_2C => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_2D => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_2E => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_2F => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        -- Address 1536 to 2047
        INIT_30 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_31 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_32 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_33 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_34 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_35 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_36 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_37 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_38 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_39 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_3A => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_3B => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",    
        INIT_3C => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_3D => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_3E => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INIT_3F => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        -- The next set of INITP_xx are for the parity bits
        -- Address 0 to 511
        INITP_00 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INITP_01 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        -- Address 512 to 1023
        INITP_02 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INITP_03 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        -- Address 1024 to 1535
        INITP_04 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INITP_05 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        -- Address 1536 to 2047
        INITP_06 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        INITP_07 => X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF")
    port map (
         DO   => cache_do, --not_dirty_out & ZERO(5 downto 0) &  cache_tag_out(8 downto 0),
         DOP  => open,--cache_tag_out(8 downto 8), 
         ADDR => cache_address,             --registered
         CLK  => clk, 
         DI   => cache_di, --not_dirty_in & ZERO(5 downto 0) & cache_tag_in(8 downto 0),  --registered
         DIP  => ZERO(1 downto 0),--cache_tag_in(8 downto 8),
         EN   => '1',
         SSR  => ZERO(0),
         WE   => cache_we);

   cache_miss    <= c_miss;
   cache_checking <= c_checking;
   c_nothit      <= c_miss and c_checking;
   c_enable      <= '1' when state_reg = STATE_WB_WAIT else cache_ram_enable;
   c_byte_we     <= ZERO(11 downto 0) & byte_we_hold when state_reg = STATE_WB_WAIT else cache_ram_byte_we;
   c_address     <= ZERO(31 downto 12) & cpu_address(11 downto 2) when state_reg = STATE_WB_WAIT else cache_ram_address;
--   c_data_w      <= when state = STATE_WB_WAIT else cache_ram_data_w;
   
	cache_data: cache_ram     -- cache data storage
	port map (
         clk               => clk,
         enable            => c_enable,
         c_nothit          => c_nothit,
         write_byte_enable => c_byte_we,
         address           => c_address,
         data_write        => cache_ram_data_w,
         data_read         => cache_ram_data_r,
	 stall_comp        => stall_cpu,
	 byte_we_next	   => byte_we_next);

--   c_stall <= '1' when (state /= STATE_MISSED and state /= STATE_WB and byte_we /= "0000")
--              or state_reg = STATE_WB_WR else '0';--stall_cpu;
   c_stall <= '1' when (state_reg = STATE_CHECKING and cache_tag_out = cache_tag_reg and byte_we /= "0000")
                or state_reg = STATE_WB_WR else '0';--stall_cpu;
   stall_comp <= c_stall or c_stall_ff;
   --cache_ram_data_r <= cache_ram_data_r128(31 downto 0)   when cpu_address(3 downto 2) = "00" else
   --                    cache_ram_data_r128(63 downto 32)  when cpu_address(3 downto 2) = "01" else
   --                    cache_ram_data_r128(95 downto 64)  when cpu_address(3 downto 2) = "10" else
   --                    cache_ram_data_r128(127 downto 96) when cpu_address(3 downto 2) = "11";

end; --logic

