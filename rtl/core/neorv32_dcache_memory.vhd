-- #################################################################################################
-- # << NEORV32 - Cache Memory >>                                                                  #
-- # ********************************************************************************************* #
-- # Direct mapped (DCACHE_NUM_SETS = 1) or 2-way set-associative (DCACHE_NUM_SETS = 2).           #
-- # Least recently used replacement policy (if DCACHE_NUM_SETS > 1).                              #
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
    DCACHE_NUM_BLOCKS : natural := 4;  -- number of blocks (min 1), has to be a power of 2
    DCACHE_BLOCK_SIZE : natural := 16; -- block size in bytes (min 4), has to be a power of 2
    DCACHE_NUM_SETS   : natural := 1   -- associativity; 1=direct-mapped, 2=2-way set-associative
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
  constant cache_offset_size_c : natural := index_size_f(DCACHE_BLOCK_SIZE/4); -- offset addresses full 32-bit words
  constant cache_index_size_c  : natural := index_size_f(DCACHE_NUM_BLOCKS);
  constant cache_tag_size_c    : natural := 32 - (cache_offset_size_c + cache_index_size_c + 2); -- 2 additional bits for byte offset
  constant cache_entries_c     : natural := DCACHE_NUM_BLOCKS * (DCACHE_BLOCK_SIZE/4); -- number of 32-bit entries (per set)
  
  -- status flag memory --
  signal valid_flag_s0 : std_ulogic_vector(DCACHE_NUM_BLOCKS-1 downto 0) := (others => '0');
  signal valid_flag_s1 : std_ulogic_vector(DCACHE_NUM_BLOCKS-1 downto 0) := (others => '0');
  signal valid         : std_ulogic_vector(1 downto 0) := (others => '0'); -- valid flag read data
  
  -- tag memory --
  type tag_mem_t is array (0 to DCACHE_NUM_BLOCKS-1) of std_ulogic_vector(cache_tag_size_c-1 downto 0);
  signal tag_mem_s0 : tag_mem_t := (others => (others => '0'));
  signal tag_mem_s1 : tag_mem_t := (others => (others => '0'));
  type tag_rd_t is array (0 to 1) of std_ulogic_vector(cache_tag_size_c-1 downto 0);
  signal tag : tag_rd_t := (others => (others => '0')); -- tag read data
  
  -- access status --
  signal hit : std_ulogic_vector(1 downto 0) := (others => '0');
  
  -- access address decomposition --
  type acc_addr_t is record
    tag    : std_ulogic_vector(cache_tag_size_c-1 downto 0);
    index  : std_ulogic_vector(cache_index_size_c-1 downto 0);
    offset : std_ulogic_vector(cache_offset_size_c-1 downto 0);
  end record;
  signal host_acc_addr, ctrl_acc_addr : acc_addr_t;
  
  -- cache data memory --
  type cache_mem_t is array (0 to cache_entries_c-1) of std_ulogic_vector(31 downto 0);
  signal cache_data_memory_s0 : cache_mem_t := (others => (others => '0')); -- set 0
  signal cache_data_memory_s1 : cache_mem_t := (others => (others => '0')); -- set 1
  
  -- cache data memory access --
  type cache_rdata_t is array (0 to 1) of std_ulogic_vector(31 downto 0);
  signal cache_rdata  : cache_rdata_t := (others => (others => '0'));
  signal cache_index  : std_ulogic_vector(cache_index_size_c-1 downto 0) := (others => '0');
  signal cache_offset : std_ulogic_vector(cache_offset_size_c-1 downto 0) := (others => '0');
  signal cache_addr   : std_ulogic_vector((cache_index_size_c+cache_offset_size_c)-1 downto 0) := (others => '0'); -- index & offset
  signal cache_we     : std_ulogic := '0' ; -- write enable (full-word)
  signal set_select   : std_ulogic := '0';
  
  -- access history --
  type history_t is record
    re_ff          : std_ulogic;
    last_used_set  : std_ulogic_vector(DCACHE_NUM_BLOCKS-1 downto 0);
    to_be_replaced : std_ulogic;
  end record;
  signal history : history_t;
  
begin

	-- Access Address Decomposition -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  host_acc_addr.tag    <= host_addr_i(31 downto 31-(cache_tag_size_c-1));
  host_acc_addr.index  <= host_addr_i(31-cache_tag_size_c downto 2+cache_offset_size_c);
  host_acc_addr.offset <= host_addr_i(2+(cache_offset_size_c-1) downto 2); -- discard byte offset

  ctrl_acc_addr.tag    <= ctrl_addr_i(31 downto 31-(cache_tag_size_c-1));
  ctrl_acc_addr.index  <= ctrl_addr_i(31-cache_tag_size_c downto 2+cache_offset_size_c);
  ctrl_acc_addr.offset <= ctrl_addr_i(2+(cache_offset_size_c-1) downto 2); -- discard byte offset


	-- Cache Access History -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  access_history: process(clk_i)
  begin
    if rising_edge(clk_i) then
      history.re_ff <= host_re_i;
      if (invalidate_i = '1') then -- invalidate whole cache
        history.last_used_set <= (others => '1');
      elsif (history.re_ff = '1') and (or_reduce_f(hit) = '1') and (ctrl_en_i = '0') then -- store last accessed set that caused a hit
        history.last_used_set(to_integer(unsigned(cache_index))) <= not hit(0);
      end if;
      history.to_be_replaced <= history.last_used_set(to_integer(unsigned(cache_index)));
    end if;
  end process access_history;

  -- which set is going to be replaced? -> opposite of last used set = least recently used set --
  set_select <= '0' when (DCACHE_NUM_SETS = 1) else (not history.to_be_replaced);


	-- Status flag memory ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  status_memory: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- write access --
      if (invalidate_i = '1') then -- invalidate whole cache
        valid_flag_s0 <= (others => '0');
        valid_flag_s1 <= (others => '0');
      elsif (ctrl_en_i = '1') then
        if (ctrl_invalid_i = '1') then -- make current block invalid
          if (set_select = '0') then
            valid_flag_s0(to_integer(unsigned(cache_index))) <= '0';
          else
            valid_flag_s1(to_integer(unsigned(cache_index))) <= '0';
          end if;
        elsif (ctrl_valid_i = '1') then -- make current block valid
          if (set_select = '0') then
            valid_flag_s0(to_integer(unsigned(cache_index))) <= '1';
          else
            valid_flag_s1(to_integer(unsigned(cache_index))) <= '1';
          end if;
        end if;
      end if;
      -- read access (sync) --
      valid(0) <= valid_flag_s0(to_integer(unsigned(cache_index)));
      valid(1) <= valid_flag_s1(to_integer(unsigned(cache_index)));
    end if;
  end process status_memory;


	-- Tag memory -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tag_memory: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl_en_i = '1') and (ctrl_tag_we_i = '1') then -- write access
        if (set_select = '0') then
          tag_mem_s0(to_integer(unsigned(cache_index))) <= ctrl_acc_addr.tag;
        else
          tag_mem_s1(to_integer(unsigned(cache_index))) <= ctrl_acc_addr.tag;
        end if;
      end if;
      tag(0) <= tag_mem_s0(to_integer(unsigned(cache_index)));
      tag(1) <= tag_mem_s1(to_integer(unsigned(cache_index)));
    end if;
  end process tag_memory;

  -- comparator --
  comparator: process(host_acc_addr, tag, valid)
  begin
    hit <= (others => '0');
    for i in 0 to DCACHE_NUM_SETS-1 loop
      if (host_acc_addr.tag = tag(i)) and (valid(i) = '1') then
        hit(i) <= '1';
      end if;
    end loop; -- i
  end process comparator;

  -- global hit --
  hit_o <= or_reduce_f(hit);


	-- Cache Data Memory ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cache_mem_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (cache_we = '1') then -- write access from control (full-word)
        if (set_select = '0') or (DCACHE_NUM_SETS = 1) then
          cache_data_memory_s0(to_integer(unsigned(cache_addr))) <= ctrl_wdata_i;
        else
          cache_data_memory_s1(to_integer(unsigned(cache_addr))) <= ctrl_wdata_i;
        end if;
      end if;
      -- read access from host (full-word) --
      cache_rdata(0) <= cache_data_memory_s0(to_integer(unsigned(cache_addr)));
      cache_rdata(1) <= cache_data_memory_s1(to_integer(unsigned(cache_addr)));
    end if;
  end process cache_mem_access;

  -- data output --
  host_rdata_o <= cache_rdata(0) when (hit(0) = '1') or (DCACHE_NUM_SETS = 1) else cache_rdata(1);

  -- cache block ram access address --
  cache_addr <= cache_index & cache_offset;

  -- cache access select --
  cache_index  <= host_acc_addr.index  when (ctrl_en_i = '0') else ctrl_acc_addr.index;
  cache_offset <= host_acc_addr.offset when (ctrl_en_i = '0') else ctrl_acc_addr.offset;
  cache_we     <= '0'                  when (ctrl_en_i = '0') else ctrl_we_i;
    
end neorv32_dcache_memory_rtl;