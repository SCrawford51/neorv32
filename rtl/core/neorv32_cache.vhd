-- #################################################################################################
-- # << NEORV32 - Processor-Internal Cache >>                                                      #
-- # ********************************************************************************************* #
-- # Direct mapped (CACHE_NUM_SETS = 1) or 2-way set-associative (CACHE_NUM_SETS = 2).             #
-- # Least recently used replacement policy (if CACHE_NUM_SETS > 1).                               #
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

entity neorv32_cache is
  generic (
    CACHE_NUM_BLOCKS  :  natural; -- number of blocks (min 1), has to be a power of 2
    CACHE_BLOCK_SIZE  :  natural; -- block size in bytes (min 4), has to be a power of 2
    ASSOCIATIVITY     :  natural; -- associativity / number of sets (1=direct_mapped), has to be a power of 2
    CACHE_REPLACE_POL :  natural  -- cache replacement policy; 1=LRU, 2=Pseudo-LRU, 3=FIFO, 4=Random
  );
  port (
    -- global control --
    clk_i        : in  std_ulogic; -- global clock, rising edge
    rstn_i       : in  std_ulogic; -- global reset, low-active, async
    clear_i      : in  std_ulogic; -- cache clear
    miss_o       : out std_ulogic; -- cache miss
    -- host controller interface --
    host_addr_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
    host_rdata_o : out std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
    host_wdata_i : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
    host_ben_i   : in  std_ulogic_vector(03 downto 0); -- byte enable
    host_we_i    : in  std_ulogic; -- write enable
    host_re_i    : in  std_ulogic; -- read enable
    host_ack_o   : out std_ulogic; -- bus transfer acknowledge
    host_err_o   : out std_ulogic; -- bus transfer error
    -- peripheral bus interface --
    bus_addr_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
    bus_rdata_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
    bus_wdata_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
    bus_ben_o    : out std_ulogic_vector(03 downto 0); -- byte enable
    bus_we_o     : out std_ulogic; -- write enable
    bus_re_o     : out std_ulogic; -- read enable
    bus_ack_i    : in  std_ulogic; -- bus transfer acknowledge
    bus_err_i    : in  std_ulogic  -- bus transfer error
  );
end neorv32_cache;

architecture neorv32_cache_rtl of neorv32_cache is

  -- cache layout --
  constant cache_offset_size_c : natural := CACHE_BLOCK_SIZE/4; -- offset addresses full 32-bit words
  constant cache_offset_bits_c  : natural := num_bits_f(cache_offset_size_c - 1); 

  -- cache memory --
  component neorv32_cache_memory
  generic (
    CACHE_NUM_BLOCKS  : natural := 64; -- number of blocks (min 1), has to be a power of 2
    CACHE_BLOCK_SIZE  : natural := 4;  -- block size in bytes (min 4), has to be a power of 2
    ASSOCIATIVITY     : natural := 1;  -- associativity; 1=direct-mapped, 2=2-way set-associative
    CACHE_REPLACE_POL : natural := 1   -- cache replacement policy; 1=LRU, 2=Pseudo-LRU, 3=FIFO, 4=Random
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
  end component;

  -- cache interface --
  type cache_if_t is record
    clear           : std_ulogic; -- cache clear
    host_addr       : std_ulogic_vector(31 downto 0); -- cpu access address
    host_rdata      : std_ulogic_vector(31 downto 0); -- cpu read data
    hit             : std_ulogic; -- hit access
    ctrl_en         : std_ulogic; -- control access enable
    ctrl_addr       : std_ulogic_vector(31 downto 0); -- control access address
    ctrl_we         : std_ulogic; -- control write enable
    ctrl_wdata      : std_ulogic_vector(31 downto 0); -- control write data
    ctrl_tag_we     : std_ulogic; -- control tag write enabled
    ctrl_valid_we   : std_ulogic; -- control valid flag set
    ctrl_invalid_we : std_ulogic; -- control valid flag clear
  end record;
  signal cache : cache_if_t;

  -- control engine --
  type ctrl_engine_state_t is (S_IDLE, S_CACHE_CLEAR, S_CACHE_CHECK, S_CACHE_MISS, S_BUS_DOWNLOAD_REQ, S_BUS_DOWNLOAD_GET,
                               S_CACHE_RESYNC_0, S_CACHE_RESYNC_1, S_BUS_ERROR);
  type ctrl_t is record
    state         : ctrl_engine_state_t; -- current state
    state_nxt     : ctrl_engine_state_t; -- next state
    addr_reg      : std_ulogic_vector(31 downto 0); -- address register for block download
    addr_reg_nxt  : std_ulogic_vector(31 downto 0);
    re_buf        : std_ulogic; -- read request buffer
    re_buf_nxt    : std_ulogic;
    we_buf        : std_ulogic; -- write request buffer
    we_buf_nxt    : std_ulogic;
    clear_buf     : std_ulogic; -- clear request buffer
    clear_buf_nxt : std_ulogic;
  end record;
  signal ctrl : ctrl_t;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- configuration --
  assert not (is_power_of_two_f(CACHE_NUM_BLOCKS) = false) report "NEORV32 PROCESSOR CONFIG ERROR! cache number of blocks <CACHE_NUM_BLOCKS> has to be a power of 2." severity error;
  assert not (is_power_of_two_f(CACHE_BLOCK_SIZE) = false) report "NEORV32 PROCESSOR CONFIG ERROR! cache block size <CACHE_BLOCK_SIZE> has to be a power of 2." severity error;
  assert not ((is_power_of_two_f(ASSOCIATIVITY) = false)) report "NEORV32 PROCESSOR CONFIG ERROR! cache associativity <ASSOCIATIVITY> has to be a power of 2." severity error;
  assert not (CACHE_NUM_BLOCKS < 1) report "NEORV32 PROCESSOR CONFIG ERROR! cache number of blocks <CACHE_NUM_BLOCKS> has to be >= 1." severity error;
  assert not (CACHE_BLOCK_SIZE < 4) report "NEORV32 PROCESSOR CONFIG ERROR! cache block size <CACHE_BLOCK_SIZE> has to be >= 4." severity error;

  -- Control Engine FSM Sync ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_fsm_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.state     <= S_CACHE_CLEAR;
      ctrl.re_buf    <= '0';
      ctrl.we_buf    <= '0';
      ctrl.clear_buf <= '0';
      ctrl.addr_reg  <= (others => '-');
    elsif rising_edge(clk_i) then
      ctrl.state     <= ctrl.state_nxt;
      ctrl.re_buf    <= ctrl.re_buf_nxt;
      ctrl.we_buf    <= ctrl.we_buf_nxt;
      ctrl.clear_buf <= ctrl.clear_buf_nxt;
      ctrl.addr_reg  <= ctrl.addr_reg_nxt;
    end if;
  end process ctrl_engine_fsm_sync;

  -- Control Engine FSM Comb ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_fsm_comb: process(ctrl, cache, clear_i, host_addr_i, host_re_i, host_we_i, bus_rdata_i, bus_ack_i, bus_err_i)
  begin
    -- control defaults --
    ctrl.state_nxt        <= ctrl.state;
    ctrl.addr_reg_nxt     <= ctrl.addr_reg;
    ctrl.re_buf_nxt       <= ctrl.re_buf or host_re_i;
    ctrl.we_buf_nxt       <= ctrl.we_buf or host_we_i;
    ctrl.clear_buf_nxt    <= ctrl.clear_buf or clear_i; -- buffer clear request from CPU

    -- cache defaults --
    cache.clear           <= '0';
    cache.host_addr       <= host_addr_i;
    cache.ctrl_en         <= '0';
    cache.ctrl_addr       <= ctrl.addr_reg;
    cache.ctrl_we         <= '0';
    cache.ctrl_wdata      <= bus_rdata_i;
    cache.ctrl_tag_we     <= '0';
    cache.ctrl_valid_we   <= '0';
    cache.ctrl_invalid_we <= '0';

    -- host interface defaults --
    host_ack_o            <= '0';
    host_err_o            <= '0';
    host_rdata_o          <= cache.host_rdata;

    -- peripheral bus interface defaults --
    bus_addr_o            <= ctrl.addr_reg;
    bus_wdata_o           <= (others => '0'); -- cache is read-only
    bus_ben_o             <= (others => '0'); -- cache is read-only
    bus_we_o              <= '0'; -- cache is read-only
    bus_re_o              <= '0';

    -- fsm --
    case ctrl.state is

      when S_IDLE => -- wait for host access request or cache control operation
      -- ------------------------------------------------------------
        if (ctrl.clear_buf = '1') then -- cache control operation?
          ctrl.state_nxt <= S_CACHE_CLEAR;
        elsif (host_re_i = '1') or (ctrl.re_buf = '1') then -- cache access
          ctrl.re_buf_nxt <= '0';
          ctrl.state_nxt  <= S_CACHE_CHECK;
        elsif (host_we_i = '1') or (ctrl.we_buf = '1') then -- write cache access
          ctrl.we_buf_nxt <= '0';
          ctrl.state_nxt  <= S_CACHE_MISS;
        end if;

      when S_CACHE_CLEAR => -- invalidate all cache entries
      -- ------------------------------------------------------------
        ctrl.clear_buf_nxt <= '0';
        cache.clear        <= '1';
        ctrl.state_nxt     <= S_IDLE;

      when S_CACHE_CHECK => -- finalize host access if cache hit
      -- ------------------------------------------------------------
        if (cache.hit = '1') then -- cache HIT
          host_ack_o     <= '1';
          ctrl.state_nxt <= S_IDLE;
        else -- cache MISS
          ctrl.state_nxt <= S_CACHE_MISS;
        end if;

      when S_CACHE_MISS => -- 
      -- ------------------------------------------------------------
        -- compute block base address --
        ctrl.addr_reg_nxt <= host_addr_i;
        ctrl.addr_reg_nxt((2+cache_offset_bits_c)-1 downto 2) <= (others => '0'); -- block-aligned
        ctrl.addr_reg_nxt(1 downto 0) <= "00"; -- word-aligned
        --
        ctrl.state_nxt <= S_BUS_DOWNLOAD_REQ;

      when S_BUS_DOWNLOAD_REQ => -- download new cache block: request new word
      -- ------------------------------------------------------------
        cache.ctrl_en  <= '1'; -- we are in cache control mode
        bus_re_o       <= '1'; -- request new read transfer
        ctrl.state_nxt <= S_BUS_DOWNLOAD_GET;

      when S_BUS_DOWNLOAD_GET => -- download new cache block: wait for bus response
      -- ------------------------------------------------------------
        cache.ctrl_en <= '1'; -- we are in cache control mode
        --
        if (bus_err_i = '1') then -- bus error
          ctrl.state_nxt <= S_BUS_ERROR;
        elsif (bus_ack_i = '1') then -- ACK = write to cache and get next word
          cache.ctrl_we <= '1'; -- write to cache
          if (and_reduce_f(ctrl.addr_reg((2+cache_offset_bits_c)-1 downto 2)) = '1') then -- block complete?
            cache.ctrl_tag_we   <= '1'; -- write tag of current address
            cache.ctrl_valid_we <= '1'; -- current block is valid now
            ctrl.state_nxt      <= S_CACHE_RESYNC_0;
          else -- get next word
            ctrl.addr_reg_nxt <= std_ulogic_vector(unsigned(ctrl.addr_reg) + 4);
            ctrl.state_nxt    <= S_BUS_DOWNLOAD_REQ;
          end if;
        end if;

      when S_CACHE_RESYNC_0 => -- re-sync host/cache access: cache read-latency
      -- ------------------------------------------------------------
        ctrl.state_nxt <= S_CACHE_RESYNC_1;

      when S_CACHE_RESYNC_1 => -- re-sync host/cache access: finalize CPU request
      -- ------------------------------------------------------------
        host_ack_o     <= '1';
        ctrl.state_nxt <= S_IDLE;

      when S_BUS_ERROR => -- bus error during download
      -- ------------------------------------------------------------
        host_err_o     <= '1';
        ctrl.state_nxt <= S_IDLE;

      when others => -- undefined
      -- ------------------------------------------------------------
        ctrl.state_nxt <= S_IDLE;

    end case;
  end process ctrl_engine_fsm_comb;

  -- signal cache miss to CPU --
  miss_o <= '1' when (ctrl.state = S_CACHE_MISS) else '0';

  -- Cache Memory ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cache_memory_inst: neorv32_cache_memory
  generic map (
    CACHE_NUM_BLOCKS  => CACHE_NUM_BLOCKS, -- number of blocks (min 1), has to be a power of 2
    CACHE_BLOCK_SIZE  => CACHE_BLOCK_SIZE, -- block size in bytes (min 4), has to be a power of 2
    ASSOCIATIVITY     => ASSOCIATIVITY,    -- associativity; 0=direct-mapped, 1=2-way set-associative
    CACHE_REPLACE_POL => CACHE_REPLACE_POL -- cache replacement policy; 1=LRU, 2=Pseudo-LRU, 3=FIFO, 4=Random
  )
  port map (
    -- global control --
    clk_i          => clk_i,                -- global clock, rising edge
    invalidate_i   => cache.clear,          -- invalidate whole cache
    -- host cache access (read-only) --
    host_addr_i    => cache.host_addr,      -- access address
    host_re_i      => host_re_i,            -- read enable
    host_rdata_o   => cache.host_rdata,     -- read data
    -- access status (1 cycle delay to access) --
    hit_o          => cache.hit,            -- hit access
    -- ctrl cache access (write-only) --
    ctrl_en_i      => cache.ctrl_en,        -- control interface enable
    ctrl_addr_i    => cache.ctrl_addr,      -- access address
    ctrl_we_i      => cache.ctrl_we,        -- write enable (full-word)
    ctrl_wdata_i   => cache.ctrl_wdata,     -- write data
    ctrl_tag_we_i  => cache.ctrl_tag_we,    -- write tag to selected block
    ctrl_valid_i   => cache.ctrl_valid_we,  -- make selected block valid
    ctrl_invalid_i => cache.ctrl_invalid_we -- make selected block invalid
  );

end neorv32_cache_rtl;