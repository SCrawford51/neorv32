-- #################################################################################################
-- # << NEORV32 - Cache Memory >>                                                                  #
-- # ********************************************************************************************* #
-- # Direct mapped (ASSOCIATIVITY = 1) or 2-way set-associative (ASSOCIATIVITY = 2).           #
-- # Least recently used replacement policy (if ASSOCIATIVITY > 1).                              #
-- # Read-only for host, write-only for control. All output signals have one cycle latency.        #
-- #                                                                                               #
-- # Cache sets are mapped to individual memory components - no multi-dimensional memory arrays    #
-- # are used as some synthesis tools have problems to map these to actual BRAM primitives.        #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_dcache_memory is
  generic (
    DCACHE_NUM_BLOCKS  : natural := 64; -- number of blocks (min 1), has to be a power of 2
    DCACHE_BLOCK_SIZE  : natural := 4;  -- block size in bytes (min 4), has to be a power of 2
    ASSOCIATIVITY      : natural := 1;  -- associativity; 1=direct-mapped, 2=2-way set-associative
    DCACHE_REPLACE_POL : natural := 1   -- cache replacement policy; 1=LRU, 2=Pseudo-LRU, 3=FIFO, 4=Random
  );
  port (
    -- global control --
    clk_i          : in  std_ulogic; -- global clock, rising edge
    invalidate_i   : in  std_ulogic; -- invalidate whole cache
    -- host cache access (read-only) --
    host_addr_i    : in  std_ulogic_vector(31 downto 0); -- access address
    host_re_i      : in  std_ulogic; -- read enable
    host_rdata_o   : out std_ulogic_vector(31 downto 0); -- read data
    -- access status (1 cycle delay to access) --
    hit_o          : out std_ulogic; -- hit access
    -- ctrl cache access (write-only) --
    ctrl_en_i      : in  std_ulogic; -- control interface enable
    ctrl_addr_i    : in  std_ulogic_vector(31 downto 0); -- access address
    ctrl_we_i      : in  std_ulogic; -- write enable (full-word)
    ctrl_wdata_i   : in  std_ulogic_vector(31 downto 0); -- write data
    ctrl_tag_we_i  : in  std_ulogic; -- write tag to selected block
    ctrl_valid_i   : in  std_ulogic; -- make selected block valid
    ctrl_invalid_i : in  std_ulogic  -- make selected block invalid
  );
end neorv32_dcache_memory;

architecture neorv32_dcache_memory_rtl of neorv32_dcache_memory is

  -- cache layout --
  constant cache_offset_size_c : natural := num_bits_f((DCACHE_BLOCK_SIZE/4)-1); -- offset addresses full 32-bit words
  constant cache_index_size_c  : natural := num_bits_f((DCACHE_NUM_BLOCKS/ASSOCIATIVITY)-1);
  constant cache_tag_size_c    : natural := 32 - (cache_offset_size_c + cache_index_size_c + 2); -- 2 additional bits for byte offset
  constant block_precision_c   : natural := num_bits_f(ASSOCIATIVITY-1); 
  constant num_sets_c          : natural := DCACHE_NUM_BLOCKS / ASSOCIATIVITY;

  -- status flag memory -- 
  signal valid_flags : std_ulogic_vector(DCACHE_NUM_BLOCKS-1 downto 0) := (others => '0');
  signal valid       : std_ulogic_vector(ASSOCIATIVITY-1 downto 0)     := (others => '0'); -- valid flag read data

  -- tag memory --
  type tag_mem_t is array (0 to DCACHE_NUM_BLOCKS-1) of std_ulogic_vector(cache_tag_size_c-1 downto 0);  
  signal tag_mem     : tag_mem_t := (others => (others => '0'));

  type tag_rd_t is array (0 to ASSOCIATIVITY-1) of std_ulogic_vector(cache_tag_size_c-1 downto 0);
  signal tag         : tag_rd_t  := (others => (others => '0')); -- tag read data

  -- access status --
  signal hit : std_ulogic_vector(ASSOCIATIVITY-1 downto 0) := (others => '0');

  -- access address decomposition --
  type acc_addr_t is record
    tag    : std_ulogic_vector(29 downto cache_index_size_c+cache_offset_size_c);
    index  : std_ulogic_vector(cache_index_size_c+cache_offset_size_c-1 downto cache_offset_size_c);
    offset : std_ulogic_vector(cache_offset_size_c-1 downto 0);
  end record;
  signal host_acc_addr : acc_addr_t := (
    tag    => (others => '0'),
    index  => (others => '0'),
    offset => (others => '0')
  );
  signal ctrl_acc_addr : acc_addr_t := (
    tag    => (others => '0'),
    index  => (others => '0'),
    offset => (others => '0')
  );
  
  -- cache data memory --
  type cache_mem_t is array (0 to DCACHE_NUM_BLOCKS-1, 0 to cache_offset_size_c) of std_ulogic_vector(31 downto 0);
  signal cache_data_memory : cache_mem_t  := (others => (others => (others => '0')));

  -- cache data memory access --
  type cache_rdata_t is array (0 to ASSOCIATIVITY-1) of std_ulogic_vector(31 downto 0);
  signal cache_rdata  : cache_rdata_t := (others => (others => '0'));
  signal cache_index  : std_ulogic_vector(cache_index_size_c-1 downto 0) := (others => '0');
  signal cache_offset : std_ulogic_vector(cache_offset_size_c-1 downto 0) := (others => '0');
  signal cache_we     : std_ulogic := '0' ; -- write enable (full-word)
  type way_sel_t is array (0 to num_sets_c-1) of std_ulogic_vector(block_precision_c-1 downto 0); -- array of sets with selected block or line in each
  signal way_select   : way_sel_t := (others => (others => '0'));
  
  -- access history --
  type history_t is record
    re_ff          : std_ulogic;
    least_used_set : way_sel_t;
    first_set      : std_ulogic_vector(block_precision_c-1 downto 0);
    to_be_replaced : way_sel_t;
	  plru_set       : std_ulogic_vector(block_precision_c-1 downto 0);
  end record;
  
  signal history : history_t := (
    re_ff          => '0', 
    least_used_set => (others => (others => '0')), 
    first_set      => (others => '0'), 
    to_be_replaced => (others => (others => '0')),
	  plru_set       => (others => '0')
  );

  -- FIFO signals
  signal fifo_cnt : unsigned(block_precision_c-1 downto 0) := (others => '0');

  -- Random signals
  signal rand_dout  : std_logic_vector(block_precision_c-1 downto 0) := (others => '0');
  signal reseed     : std_logic := '0';
  signal newseed    : std_logic_vector(31 downto 0) := x"AAAACCCC";
  signal rand_ready : std_logic := '0';
  signal rand_valid : std_logic := '0';
  signal rand_data  : std_logic_vector(31 downto 0)  := (others => '0'); 

  -- LRU signals
  --type lru_set is array (0 to num_sets_c-1, 0 to ASSOCIATIVITY-1) of unsigned(9 downto 0);
  type lru_blks is array (0 to ASSOCIATIVITY-1) of unsigned(9 downto 0); -- small array
  type lru_sets is array (0 to num_sets_c-1) of lru_blks; -- big array
  signal age : lru_sets := (others => (others => (others => '0')));
  
  -- PLRU signals
  signal plru_path : unsigned(block_precision_c-1 downto 0) := (others => '0');
  signal plru_nat  : natural;

  function maxindex(a : lru_blks) return integer is
    variable index : integer := 0;
    variable foundmax : unsigned(9 downto 0) := (others => '0');
  begin
    for i in 0 to a'high loop
      if a(i) > foundmax then
        index := i;
        foundmax := a(i);
      end if;
    end loop;
    return index;
  end function;

  function minindex(a : lru_blks) return integer is
    variable index : integer := 0;
    variable foundmin : unsigned(9 downto 0) := (others => '1');
  begin
    for i in 0 to a'high loop
      if a(i) < foundmin then
        index := i;
        foundmin := a(i);
      end if;
    end loop;
    return index;
  end function;
  
begin

	-- Access Address Decomposition -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  host_acc_addr.tag    <= host_addr_i(31 downto 31-(cache_tag_size_c-1));
  host_acc_addr.index  <= host_addr_i(31-cache_tag_size_c downto 2+cache_offset_size_c);
  host_acc_addr.offset <= host_addr_i(2+(cache_offset_size_c-1) downto 2); -- discard byte offset

  ctrl_acc_addr.tag    <= ctrl_addr_i(31 downto 31-(cache_tag_size_c-1));
  ctrl_acc_addr.index  <= ctrl_addr_i(31-cache_tag_size_c downto 2+cache_offset_size_c);
  ctrl_acc_addr.offset <= ctrl_addr_i(2+(cache_offset_size_c-1) downto 2); -- discard byte offset

	-- Status flag memory ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  status_memory : process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- write access --
      if (invalidate_i = '1') then -- invalidate whole cache
        valid_flags <= (others => '0');
      elsif (ctrl_en_i = '1') then
        if (ctrl_invalid_i = '1') then -- make current block invalid
          for ii in 0 to ASSOCIATIVITY - 1 loop
            if unsigned(way_select(to_integer(unsigned(cache_index)))) = ii then
              valid_flags(to_integer(unsigned(cache_index) & unsigned(way_select(to_integer(unsigned(cache_index)))))) <= '0';
            end if;
          end loop;
        elsif (ctrl_valid_i = '1') then -- make current block valid
          for ii in 0 to ASSOCIATIVITY - 1 loop
            if unsigned(way_select(to_integer(unsigned(cache_index)))) = ii then
              valid_flags(to_integer(unsigned(cache_index) & unsigned(way_select(to_integer(unsigned(cache_index)))))) <= '1';
            end if;
          end loop;
        end if;
      end if;
      -- read access (sync) --
      for ii in 0 to ASSOCIATIVITY - 1 loop
        valid(ii) <= valid_flags(to_integer(unsigned(cache_index) & to_unsigned(ii,way_select(0)'length)-1));
      end loop;
    end if;
  end process status_memory;


	-- Tag memory -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tag_memory : process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl_en_i = '1') and (ctrl_tag_we_i = '1') then -- write access
        for ii in 0 to ASSOCIATIVITY - 1 loop
          if unsigned(way_select(to_integer(unsigned(cache_index)))) = ii then
            tag_mem(to_integer(unsigned(cache_index) & unsigned(way_select(to_integer(unsigned(cache_index)))))) <= ctrl_acc_addr.tag;
          end if;
        end loop;
      end if;
      for ii in 0 to ASSOCIATIVITY - 1 loop
        tag(ii) <= tag_mem(to_integer(unsigned(cache_index) & to_unsigned(ii,way_select(0)'length)));
      end loop;
    end if;
  end process tag_memory;

  -- comparator --
  comparator : process(host_acc_addr, tag, valid)
  begin
    hit <= (others => '0');
    for i in 0 to ASSOCIATIVITY-1 loop
      if (host_acc_addr.tag = tag(i)) and (valid(i) = '1') then -- Hit
        hit(i) <= '1';
      end if;
    end loop; -- i
  end process comparator;

  -- global hit --
  hit_o <= or_reduce_f(hit);


	-- Cache Data Memory ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cache_mem_access : process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (cache_we = '1') then -- write access from control (full-word)
        age(to_integer(unsigned(cache_index)))(to_integer(unsigned(way_select(to_integer(unsigned(cache_index)))))) <= age(to_integer(unsigned(cache_index)))(to_integer(unsigned(way_select(to_integer(unsigned(cache_index)))))) + 1;
        cache_data_memory(to_integer(unsigned(cache_index) & unsigned(way_select(to_integer(unsigned(cache_index))))),to_integer(unsigned(cache_offset))) <= ctrl_wdata_i;
      elsif (history.re_ff = '1') and (or_reduce_f(hit) = '1') and (ctrl_en_i = '0') then
        age(to_integer(unsigned(cache_index)))(to_integer(unsigned(way_select(to_integer(unsigned(cache_index)))))) <= age(to_integer(unsigned(cache_index)))(to_integer(unsigned(way_select(to_integer(unsigned(cache_index)))))) + 1;
      end if;
      -- read access from host (full-word) --
      for ii in 0 to ASSOCIATIVITY - 1 loop
        cache_rdata(ii) <= cache_data_memory(to_integer(unsigned(cache_index) & to_unsigned(ii,way_select(0)'length)),to_integer(unsigned(cache_offset)));
      end loop;
    end if;
  end process cache_mem_access;

  -- data output --
  output_mux : process(clk_i)
  begin
    if rising_edge(clk_i) then
      host_rdata_o <= cache_rdata(0);
      for ii in 0 to ASSOCIATIVITY - 1 loop
        if hit(ii) = '1' then
          host_rdata_o <= cache_rdata(ii);
        end if;
      end loop;
    end if;
  end process output_mux;

  -- cache access select --
  cache_index  <= host_acc_addr.index  when (ctrl_en_i = '0') else ctrl_acc_addr.index;
  cache_offset <= host_acc_addr.offset when (ctrl_en_i = '0') else ctrl_acc_addr.offset;
  cache_we     <= '0'                  when (ctrl_en_i = '0') else ctrl_we_i;

  -- LRU Cache Access History -------------------------------------------------------------------
  -- --------------------------------------------------------------------------------------------
  DCACHE_LRU_INST : if (DCACHE_REPLACE_POL = 1) generate
    access_history : process(clk_i)
    begin
      if rising_edge(clk_i) then
        history.re_ff <= host_re_i;
        if (invalidate_i = '1') then -- invalidate whole cache
          history.least_used_set <= (others => (others => '0'));
        elsif (history.re_ff = '1') and (or_reduce_f(hit) = '1') and (ctrl_en_i = '0') then -- store last accessed set that caused a hit
          history.least_used_set(to_integer(unsigned(cache_index))) <= std_ulogic_vector(to_unsigned(minindex(age(to_integer(unsigned(cache_index)))(0 to ASSOCIATIVITY-1)), history.least_used_set(to_integer(unsigned(cache_index)))'length));
        end if;
        if(and_reduce_f(cache_offset) = '1') then
          history.to_be_replaced(to_integer(unsigned(cache_index))) <= history.least_used_set(to_integer(unsigned(cache_index)));
        end if;
      end if;
    end process access_history;

    -- which set is going to be replaced? -> opposite of last used set = least recently used set --
    way_select(to_integer(unsigned(cache_index))) <= (others => '0') when (ASSOCIATIVITY = 1) else history.to_be_replaced(to_integer(unsigned(cache_index)));
  end generate;
    
end neorv32_dcache_memory_rtl;