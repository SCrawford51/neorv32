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
  constant cache_offset_size_c  : natural := DCACHE_BLOCK_SIZE / 4; -- offset addresses full 32-bit words
  constant cache_offset_bits_c  : natural := num_bits_f(cache_offset_size_c - 1); 
  constant cache_offset_index_c : natural := index_size_f(cache_offset_size_c - 1); 
  constant cache_index_size_c   : natural := DCACHE_NUM_BLOCKS / ASSOCIATIVITY;
  constant cache_index_bits_c   : natural := num_bits_f(cache_index_size_c - 1);
  constant cache_tag_bits_c     : natural := 32 - (cache_offset_bits_c + cache_index_bits_c + 2); -- 2 additional bits for byte offset
  constant block_precision_c    : natural := num_bits_f(ASSOCIATIVITY - 1); 

  -- status flag memory -- 
  signal valid_flags : std_ulogic_vector(DCACHE_NUM_BLOCKS-1 downto 0) := (others => '0');
  signal valid       : std_ulogic_vector(ASSOCIATIVITY-1 downto 0)     := (others => '0'); -- valid flag read data

  -- tag memory --
  type tag_mem_t is array (0 to DCACHE_NUM_BLOCKS-1) of std_ulogic_vector(cache_tag_bits_c-1 downto 0);  
  signal tag_mem     : tag_mem_t := (others => (others => '1'));

  type tag_rd_t is array (0 to ASSOCIATIVITY-1) of std_ulogic_vector(cache_tag_bits_c-1 downto 0);
  signal tag         : tag_rd_t  := (others => (others => '0')); -- tag read data

  -- access status --
  signal hit : std_ulogic_vector(ASSOCIATIVITY-1 downto 0) := (others => '0');

  -- access address decomposition --
  type acc_addr_t is record
    tag    : std_ulogic_vector(29 downto cache_index_bits_c+cache_offset_bits_c);
    index  : std_ulogic_vector(cache_index_bits_c+cache_offset_bits_c-1 downto cache_offset_bits_c);
    offset : std_ulogic_vector(cache_offset_bits_c-1 downto 0);
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
  type cache_mem_t is array (0 to DCACHE_NUM_BLOCKS-1, 0 to cache_offset_bits_c) of std_ulogic_vector(31 downto 0);
  signal cache_data_memory : cache_mem_t  := (others => (others => (others => '0')));

  -- cache data memory access --
  type cache_rdata_t is array (0 to ASSOCIATIVITY-1) of std_ulogic_vector(31 downto 0);
  signal cache_rdata  : cache_rdata_t := (others => (others => '0'));

  signal cache_index  : unsigned(cache_index_bits_c-1 downto 0) := (others => '0');
  signal cache_offset : unsigned(cache_offset_bits_c-1 downto 0) := (others => '0');
  signal cache_we     : std_ulogic := '0' ; -- write enable (full-word)

  type way_sel_t is array (0 to cache_index_size_c-1) of unsigned(block_precision_c-1 downto 0);
  signal way_select   : way_sel_t := (others => (others => '0'));
  
  -- access history --
  type history_t is record
    re_ff          : std_ulogic;
    least_used_way : way_sel_t;
    first_way      : way_sel_t;
    to_be_replaced : way_sel_t;
	  plru_way       : way_sel_t;
  end record;
  
  signal history : history_t := (
    re_ff          => '0',
    least_used_way => (others => (others => '0')), 
    first_way      => (others => (others => '0')), 
    to_be_replaced => (others => (others => '0')),
	  plru_way       => (others => (others => '0'))
  );

  -- FIFO signals
  type fifo_cnt_t is array (0 to cache_index_size_c-1) of unsigned(block_precision_c-1 downto 0);
  signal fifo_cnt :  fifo_cnt_t := (others => (others => '0'));

  -- Random signals
  signal rand_dout  : std_logic_vector(block_precision_c-1 downto 0) := (others => '0');
  signal reseed     : std_logic := '0';
  signal newseed    : std_logic_vector(31 downto 0) := x"AAAACCCC";
  signal rand_ready : std_logic := '0';
  signal rand_valid : std_logic := '0';
  signal rand_data  : std_logic_vector(31 downto 0)  := (others => '0'); 

  -- LRU signals
  type lru_blks is array (0 to ASSOCIATIVITY-1) of unsigned(9 downto 0); -- small array
  type lru_sets is array (0 to cache_index_size_c-1) of lru_blks; -- big array
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
  host_acc_addr.tag    <= host_addr_i(31 downto 31-(cache_tag_bits_c-1));
  host_acc_addr.index  <= host_addr_i(31-cache_tag_bits_c downto 2+cache_offset_bits_c);
  host_acc_addr.offset <= host_addr_i(2+(cache_offset_bits_c-1) downto 2); -- discard byte offset

  ctrl_acc_addr.tag    <= ctrl_addr_i(31 downto 31-(cache_tag_bits_c-1));
  ctrl_acc_addr.index  <= ctrl_addr_i(31-cache_tag_bits_c downto 2+cache_offset_bits_c);
  ctrl_acc_addr.offset <= ctrl_addr_i(2+(cache_offset_bits_c-1) downto 2); -- discard byte offset

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
          if ASSOCIATIVITY = 1 then
            valid_flags(to_integer(cache_index)) <= '0';
          else
            for ii in 0 to ASSOCIATIVITY - 1 loop
              if way_select(to_integer(cache_index)) = ii then
                valid_flags(to_integer(cache_index & way_select(to_integer(cache_index)))) <= '0';
              end if;
            end loop;
          end if;
        elsif (ctrl_valid_i = '1') then -- make current block valid
          if ASSOCIATIVITY = 1 then
            valid_flags(to_integer(cache_index)) <= '1';
          else
            for ii in 0 to ASSOCIATIVITY - 1 loop
              if way_select(to_integer(cache_index)) = ii then
                valid_flags(to_integer(cache_index & way_select(to_integer(cache_index)))) <= '1';
              end if;
            end loop;
          end if;
        end if;
      end if;
    end if;
  end process status_memory;


	-- Tag memory -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tag_memory : process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl_en_i = '1') and (ctrl_tag_we_i = '1') then -- write access
        if ASSOCIATIVITY = 1 then
          tag_mem(to_integer(cache_index)) <= ctrl_acc_addr.tag;
        else
          for ii in 0 to ASSOCIATIVITY - 1 loop
            if way_select(to_integer(cache_index)) = ii then
              tag_mem(to_integer(cache_index & way_select(to_integer(cache_index)))) <= ctrl_acc_addr.tag;
            end if;
          end loop;
        end if;
      end if;
    end if;
  end process tag_memory;

  -- comparator --
  comparator : process(host_acc_addr, history)
  begin
    hit <= (others => '0');
    if ctrl_en_i = '1' then
    elsif history.re_ff = '1' and ASSOCIATIVITY = 1 then
      if (host_acc_addr.tag = tag_mem(to_integer(cache_index)) and valid_flags(to_integer(cache_index)) = '1') then -- Hit
        hit(0) <= '1';
      end if;
    elsif history.re_ff = '1' then
      for i in 0 to ASSOCIATIVITY-1 loop
        if (host_acc_addr.tag = tag_mem(to_integer(cache_index & to_unsigned(i,way_select(0)'length))) and valid_flags(to_integer(cache_index & to_unsigned(i,way_select(0)'length))) = '1') then -- Hit
          hit(i) <= '1';
        end if;
      end loop; -- i
    end if;
  end process comparator;

  -- global hit --
  hit_o <= or_reduce_f(hit);


	-- Cache Data Memory ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cache_mem_access : process(clk_i)
  begin
    if ASSOCIATIVITY = 1 then
      way_select(to_integer(cache_index)) <= (others => '0');
    else
      way_select(to_integer(cache_index)) <= history.to_be_replaced(to_integer(cache_index));
    end if;
    if rising_edge(clk_i) then
      history.re_ff <= host_re_i;
      if (cache_we = '1') then -- write access from control (full-word)
        if ASSOCIATIVITY = 1 then
          if cache_index_size_c = 1 then
            cache_data_memory(to_integer(cache_index),0) <= ctrl_wdata_i;
          else
            cache_data_memory(to_integer(cache_index),to_integer(cache_offset)) <= ctrl_wdata_i;
          end if;
        else
          age(to_integer(cache_index))(to_integer(way_select(to_integer(cache_index)))) <= age(to_integer(cache_index))(to_integer(way_select(to_integer(cache_index)))) + 1;
          if cache_index_size_c = 1 then
            cache_data_memory(to_integer(cache_index & way_select(to_integer(cache_index))),0) <= ctrl_wdata_i;
          else
            cache_data_memory(to_integer(cache_index & way_select(to_integer(cache_index))),to_integer(cache_offset)) <= ctrl_wdata_i;
          end if;
        end if;
      elsif (history.re_ff = '1') and (or_reduce_f(hit) = '1') and (ctrl_en_i = '0') then
        if ASSOCIATIVITY > 1 then
          age(to_integer(cache_index))(to_integer(way_select(to_integer(cache_index)))) <= age(to_integer(cache_index))(to_integer(way_select(to_integer(cache_index)))) + 1;
        end if;
      end if;
      -- read access from host (full-word) --
      if ASSOCIATIVITY = 1 then
        if cache_index_size_c = 1 then
          cache_rdata(0) <= cache_data_memory(to_integer(cache_index),0);
        else
          cache_rdata(0) <= cache_data_memory(to_integer(cache_index),to_integer(cache_offset));
        end if;
      else
        for ii in 0 to ASSOCIATIVITY - 1 loop
          if cache_index_size_c = 1 then
            cache_rdata(ii) <= cache_data_memory(to_integer(cache_index & to_unsigned(ii,way_select(0)'length)),0);
          else
            cache_rdata(ii) <= cache_data_memory(to_integer(cache_index & to_unsigned(ii,way_select(0)'length)),to_integer(cache_offset));
          end if;
        end loop;
      end if;
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
  cache_index  <= unsigned(host_acc_addr.index)  when (ctrl_en_i = '0') else unsigned(ctrl_acc_addr.index);
  cache_offset <= unsigned(host_acc_addr.offset) when (ctrl_en_i = '0') else unsigned(ctrl_acc_addr.offset);
  cache_we     <= '0'                            when (ctrl_en_i = '0') else ctrl_we_i;

  -- LRU Cache Access History -------------------------------------------------------------------
  -- --------------------------------------------------------------------------------------------
  DCACHE_LRU_INST : if (DCACHE_REPLACE_POL = 1) generate
    access_history : process(clk_i)
    begin
      if rising_edge(clk_i) then
        if (invalidate_i = '1') then -- invalidate whole cache
          history.least_used_way <= (others => (others => '0'));
        elsif (history.re_ff = '1') and (or_reduce_f(hit) = '1') and (ctrl_en_i = '0') then -- store last accessed set that caused a hit
          history.least_used_way(to_integer(cache_index)) <= to_unsigned(minindex(age(to_integer(cache_index))(0 to ASSOCIATIVITY-1)), history.least_used_way(0)'length);
        end if;
        if(and_reduce_f(std_ulogic_vector(cache_offset)) = '1') then
          history.to_be_replaced(to_integer(cache_index)) <= history.least_used_way(to_integer(cache_index));
        end if;
      end if;
    end process access_history;
  end generate;
  
  
  -- PLRU Cache Access History -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
--  DCACHE_PLRU_INST : if (DCACHE_REPLACE_POL = 2) generate
--    plru_access_history: process(clk_i) is
--    
--    -- Tree-PLRU algorithm to determine block to select
--    -- low_idx : index of lowest 'half' of plru bits
--    -- high_idx: index of highest 'half' of plru bits
--    -- mid_idx: index of the current root being observed
--    -- level: which level of the 'tree' the algorithm is currently on
--    impure function plru_replacement(low_idx: natural;
--                                    high_idx: natural;
--                                    mid_idx: natural; 
--                                    level: natural) return natural is
--      -- see NOTE below: may not want to instantiate this and instead set it on the first iteration of the function
--      variable prev_acc_loc: integer := to_integer(plru_path); -- PLRU comparison value
--
--      -- using the constant block_precision_c instead (they share the same value)
--      -- variable max_level: natural := index_to_f(ASSOCIATIVITY);   
--    begin
--      -- base case where the max iteration has been passed => stop algorithm
--      if level > block_precision_c then  
--        return 1;
--      end if;
--
--      -- NOTE: Not sure if prev_acc_loc will re-instantiate on each call of the function
--      -- This may be a problem since the function is recursive
--
--      -- if level = 1 then
--      --   prev_acc_loc = to_integer(plru_path);
--      -- end if;
--
--      if mid_idx < prev_acc_loc then -- call algorithm on the lower half, update current bit to '0'
--        history.plru_way(mid_idx) <= '0'; 
--        plru_path(level - 1)      <= '1';   -- used for next replacement
--
--        return plru_replacement(low_idx   => low_idx, 
--                                high_idx  => mid_idx, 
--                                mid_idx   => high_idx / 2 + low_idx,
--                                level     => level + 1);
--      elsif mid_idx > prev_acc_loc then -- call algorithm on the upper half, update current bit to '1'
--        history.plru_way(mid_idx) <= '1';
--        plru_path(level - 1)      <= '0';   -- used for next replacement
--
--        return plru_replacement(low_idx   => mid_idx, 
--                                high_idx  => high_idx, 
--                                mid_idx   => high_idx / 2 + low_idx,
--                                level     => level + 1);
--
--      end if;
--
--      return 1;   -- stop algorithm if no conditions are met
--      end function plru_replacement;
--
--    begin
--      if rising_edge(clk_i) then
--        history.re_ff <= host_re_i;
--        if (invalidate_i = '1') then -- invalidate whole cache
--          history.plru_way <= (others => '1');
--        elsif (history.re_ff = '1') and (or_reduce_f(hit) = '1') and (ctrl_en_i = '0') then -- do plru on hit
--          -- NOTE: this function updates the history.plru_way signal
--          plru_nat <= plru_replacement(low_idx => 0, 
--                                       high_idx => ASSOCIATIVITY-1, 
--                                       mid_idx  => (ASSOCIATIVITY-1) / 2, 
--                                       level    => 1
--          ); 
--        end if;
--        history.to_be_replaced <= history.plru_way;
--      end if;
--    end process plru_access_history;
--  end generate;

	-- FIFO Cache Access History -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  DCACHE_FIFO_INST : if (DCACHE_REPLACE_POL = 3) generate
    fifo_access_history : process(clk_i)
    begin
      if rising_edge(clk_i) then
        if (invalidate_i = '1') then -- invalidate whole cache
          history.first_way <= (others => (others => '0'));
        elsif (history.re_ff = '1' and (or_reduce_f(hit) = '0') and (ctrl_en_i = '0')) then -- update counter on a cache miss
          if fifo_cnt(to_integer(cache_index)) <= block_precision_c then
            fifo_cnt(to_integer(cache_index)) <= fifo_cnt(to_integer(cache_index)) + 1;
          else
            fifo_cnt(to_integer(cache_index)) <= (others => '0');
          end if;
        end if;
        history.first_way(to_integer(cache_index)) <= fifo_cnt(to_integer(cache_index)); -- have first block to set to counter value
        if (and_reduce_f(std_ulogic_vector(cache_offset)) = '1') then
          history.to_be_replaced(to_integer(cache_index)) <= history.first_way(to_integer(cache_index)); -- assign first block to be replaced
        end if;
      end if;
    end process fifo_access_history;
  end generate;

  -- Random Cache Access History -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  DCACHE_RANDOM_INST : if (DCACHE_REPLACE_POL = 4) generate
    random_gen_inst : rng_mt19937 -- 32-bit random number generator
    generic map (
        init_seed       => x"ACACACAC",
        force_const_mul => false
    ) 
    port map (
        clk       => clk_i,
        rst       => '0',
        reseed    => '0',
        newseed   => x"AAAACCCC",
        out_ready => rand_ready,
        out_valid => rand_valid,
        out_data  => rand_data
    );

    rand_access_history : process(clk_i)
    begin
      if rising_edge(clk_i) then
        rand_ready <= '1';
        if (invalidate_i = '1') then -- invalidate whole cache
          history.first_way <= (others => (others => '0'));
        elsif (rand_valid = '1') then
          rand_dout <= rand_data(block_precision_c-1 downto 0);
        end if;
        if (and_reduce_f(std_ulogic_vector(cache_offset)) = '1') then
          history.to_be_replaced(to_integer(cache_index)) <= unsigned(rand_dout);
        end if;
      end if;
    end process rand_access_history;
  end generate;
    
end neorv32_dcache_memory_rtl;