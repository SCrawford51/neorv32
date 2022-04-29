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
use ieee.math_real.all;

library neorv32;
use neorv32.neorv32_package.all;
use neorv32.neorv32_dcache_memory_tb_pkg.all;

entity tb_neorv32_dcache_memory is
  generic (
    CLOCK_FREQUENCY    : natural := 50000000; -- clock frequency of clk_i in Hz
    DCACHE_NUM_BLOCKS  : natural := 64;       -- number of blocks (min 1), has to be a power of 2
    DCACHE_BLOCK_SIZE  : natural := 8;        -- block size in bytes (min 4), has to be a power of 2
    ASSOCIATIVITY      : natural := 3;        -- associativity; 2**(n-1)-way 1=direct-mapped, 2=2-way set-associative, 3=4-way set associative
    DCACHE_REPLACE_POL : natural := 1;        -- cache replacement policy; 1=LRU, 2=Pseudo-LRU, 3=FIFO, 4=Random
    SELF_TERM          : boolean := true      -- When true the testbench will stop running if pass/fail has been determined
  );
end tb_neorv32_dcache_memory;

architecture tb_neorv32_dcache_memory_rtl of tb_neorv32_dcache_memory is

  -- cache layout --
  constant cache_offset_size_c : natural                                     := DCACHE_BLOCK_SIZE/4;
  constant cache_offset_bits_c : natural                                     := num_bits_f(cache_offset_size_c-1);
  constant cache_num_sets_c    : natural                                     := DCACHE_NUM_BLOCKS/ASSOCIATIVITY;
  constant cache_index_bits_c  : natural                                     := num_bits_f(cache_num_sets_c-1);

  -- internals - hands off! --
  constant t_clock_c           : time                                        := (1 sec) / CLOCK_FREQUENCY;
  constant self_term_c         : time                                        := 4 ms;
  constant max_cycles_c        : natural                                     := self_term_c / t_clock_c;
  constant rand_setup_cycles_c : natural                                     := 4*624;

  -- read count
  constant max_num_reads       : natural                                     := 100_000;
  
  -- counters
  type counter is array (ASSOCIATIVITY-1 downto 0) of natural;
  signal hit_count             : counter                                     := (others => 0);
  signal read_count            : counter                                     := (others => 0);

  -- generators --
  signal run_tb                : std_ulogic                                  := '1';
  signal clk_gen               : std_ulogic                                  := '0';
  signal rst_gen               : std_ulogic                                  := '0';

  -- testbench signals
  signal prev_addr             : unsigned(31 downto 0)                       := x"00000000";
  signal next_addr             : unsigned(31 downto 0)                       := x"00000000";
  signal curr_addr             : unsigned(31 downto 0)                       := x"00000000";
  signal init_mem              : std_ulogic                                  := '1';
  signal ctrl_en               : std_ulogic                                  := '0';
  signal ctrl_we               : std_ulogic                                  := '0';
  signal ctrl_tag_we           : std_ulogic                                  := '0';
  signal ctrl_valid_we         : std_ulogic                                  := '0';
  signal ctrl_invalid_we       : std_ulogic                                  := '0';
  signal ctrl_addr             : std_ulogic_vector(31 downto 0)              := x"00000000";
  signal ctrl_wdata            : std_ulogic_vector(31 downto 0)              := x"00000000";

  type host_rdata_t is array (0 to ASSOCIATIVITY-1) of std_ulogic_vector(31 downto 0);
  
  signal host_re               : std_ulogic                                  := '0';
  signal host_addr             : std_ulogic_vector(31 downto 0)              := x"00000000";
  signal host_rdata            : host_rdata_t;

  signal hit                   : std_ulogic_vector(ASSOCIATIVITY-1 downto 0) := (others => '0');

  signal timeout_err           : std_ulogic                                  := '0';
  signal invalid_err           : std_ulogic                                  := '0';
  signal invalid_block_err     : std_ulogic                                  := '0';
  signal bad_data_read_err     : std_ulogic                                  := '0';
  signal data_read_err         : std_ulogic                                  := '0';
  signal tb_error              : std_ulogic                                  := '0';
  signal tb_finished           : std_ulogic                                  := '0';

begin
  -- Clock/Reset Generator ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  clk_gen <= not clk_gen after (t_clock_c/2) when run_tb = '1' else '0';

  -- Testbench error/completion messages -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (timeout_err = '1')       report "Testbench has stalled somewhere."                                 severity error;
  assert not (invalid_err = '1')       report "Memory reported a cache hit while the selected block is invalid." severity error;
  assert not (invalid_block_err = '1') report "Memory reported a cache miss while the selected block is valid."  severity error;
  assert not (bad_data_read_err = '1') report "Cache reported a hit for data that should not be present."        severity error;
  assert not (data_read_err = '1')     report "Cache reported a miss for data that should be present."           severity error;
  assert not (tb_finished = '1')       report "Testbench successful."                                            severity note;

  -- Self terminate the testbench if desired ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  self_term_gen: if SELF_TERM generate 
    self_term_proc : process (clk_gen)
      variable sim_time : natural := 0;
    begin
      if rising_edge(clk_gen) then
        sim_time := sim_time + 1;
        tb_error <= timeout_err or invalid_err or invalid_block_err or bad_data_read_err or data_read_err;
        if sim_time > max_cycles_c then
          run_tb      <= '0';
          timeout_err <= '1';
        elsif tb_error = '1' then
          run_tb <= '0';
        elsif tb_finished = '1' then
          run_tb <= '0';
        end if;
      end if;
    end process self_term_proc;
  end generate self_term_gen;

  -- Main control process ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  run_test : process
    variable init_rand   : boolean := true;
    variable write_num   : natural := 0;
    variable offset_addr : natural := 0;
    variable init_read   : boolean := true;
    variable rand_addr   : integer := 0;

    -- seeds for random number generator
    variable seed1       : integer := 999;
    variable seed2       : integer := 999;

    impure function rand_int(min_val, max_val : integer) return integer is
      variable r : real;
    begin
      uniform(seed1, seed2, r);
      return integer(
        round(r * real(max_val - min_val + 1) + real(min_val) - 0.5));
    end function;

  begin
    if DCACHE_REPLACE_POL = 4 and init_rand then
      wait for rand_setup_cycles_c*t_clock_c;
      wait until rising_edge(clk_gen);
      init_rand := false;
    elsif init_read = true then
      wait until rising_edge(clk_gen);
      init_read     := false;
      host_re       <= '1';
      host_addr     <= std_ulogic_vector(curr_addr);

      wait until rising_edge(clk_gen);
      host_re       <= '0';
    elsif init_mem = '1' then -- Fill cache from memory at beginning of testbench
      if write_num < DCACHE_NUM_BLOCKS * cache_offset_size_c then

        if (write_num /= 0) and (write_num mod cache_offset_size_c = 0) then
          
          wait until rising_edge(clk_gen);
          host_re       <= '1';
          host_addr     <= std_ulogic_vector(prev_addr);

          wait until rising_edge(clk_gen);
          host_re       <= '0';

          wait for 5*t_clock_c;
          wait until rising_edge(clk_gen);
          host_re       <= '1';
          host_addr     <= std_ulogic_vector(next_addr);

          wait until rising_edge(clk_gen);
          host_re       <= '0';

          wait for 5*t_clock_c;
        end if;

        wait until rising_edge(clk_gen);
        ctrl_en       <= '1';
        ctrl_we       <= '1';
        ctrl_tag_we   <= '1';
        ctrl_valid_we <= '1';
        ctrl_addr     <= std_ulogic_vector(curr_addr);
        ctrl_wdata    <= cache_ext_mem(to_integer(curr_addr)); -- From neorv32_dcache_memory_tb_pkg.vhd (run dmem_gen.py to generate)

        write_num  := write_num + 1;
        prev_addr  <= curr_addr;
        next_addr  <= curr_addr + 4; 

        wait until rising_edge(clk_gen); -- One clock cycle to change hit to true
        if (write_num /= 0) and (write_num mod cache_offset_size_c = 0) then
          ctrl_en       <= '0';
          ctrl_we       <= '0';
          ctrl_tag_we   <= '0';
          ctrl_valid_we <= '0';
        end if;
        curr_addr  <= next_addr;

      else
        wait until rising_edge(clk_gen);
        init_mem <= '0';
      end if;
    else
      -- Tests while sets are invalid
      -- Invalidate block, attempt a read and report error if successful
      wait until rising_edge(clk_gen);
      ctrl_invalid_we   <= '1';
      ctrl_en           <= '1';
      ctrl_addr         <= x"00000000";

      wait until rising_edge(clk_gen);
      ctrl_invalid_we   <= '0';
      ctrl_en           <= '0';

      wait until rising_edge(clk_gen);
      host_addr         <= x"00000000";
      host_re           <= '1';

      wait until rising_edge(clk_gen);
      host_re           <= '0';
      invalid_err       <= or_reduce_f(hit);

      -- Revalidate block, attempt a read and report error if unsuccessful
      wait until rising_edge(clk_gen);
      ctrl_valid_we     <= '1';
      ctrl_tag_we       <= '1';
      ctrl_en           <= '1';

      wait until rising_edge(clk_gen);
      ctrl_en           <= '0';
      ctrl_valid_we     <= '0';
      ctrl_tag_we       <= '0';

      wait until rising_edge(clk_gen);
      host_re           <= '1';

      wait until rising_edge(clk_gen);
      host_re           <= '0';

      wait until rising_edge(clk_gen);
      invalid_block_err <= not(and_reduce_f(hit));

      -- Tests while sets are valid
      -- Read data that is not in cache, report error if successful
      wait until rising_edge(clk_gen);
      host_re           <= '1';
      host_addr         <= std_ulogic_vector(to_unsigned((DCACHE_NUM_BLOCKS * cache_offset_size_c) + 1, 32));

      wait until rising_edge(clk_gen);
      host_re           <= '0';
      bad_data_read_err <= or_reduce_f(hit);

      -- Write then read, report error if unsuccessful
      wait until rising_edge(clk_gen);
      ctrl_en           <= '1';
      ctrl_we           <= '1';
      ctrl_tag_we       <= '1';
      ctrl_valid_we     <= '1';
      ctrl_addr         <= std_ulogic_vector(to_unsigned((DCACHE_NUM_BLOCKS * cache_offset_size_c) + 1, 32));
      ctrl_wdata        <= cache_ext_mem(((DCACHE_NUM_BLOCKS * cache_offset_size_c) + 1)*4); -- From neorv32_dcache_memory_tb_pkg.vhd (run dmem_gen.py to generate)

      wait until rising_edge(clk_gen);
      ctrl_en           <= '0';
      ctrl_we           <= '0';
      ctrl_tag_we       <= '0';
      ctrl_valid_we     <= '0';

      wait until rising_edge(clk_gen);
      host_re           <= '1';

      wait until rising_edge(clk_gen);
      host_re           <= '0';

      wait until rising_edge(clk_gen);
      data_read_err     <= not(or_reduce_f(hit));

      -- reinitialize signals
      wait until rising_edge(clk_gen);
      ctrl_en           <= '0';
      ctrl_we           <= '0';
      ctrl_tag_we       <= '0';
      ctrl_valid_we     <= '0';
      ctrl_invalid_we   <= '0';
      ctrl_addr         <= x"00000000";
      ctrl_wdata        <= x"00000000";
      host_re           <= '0';
      host_addr         <= x"00000000";

      -- TODO: Perform reads on random address using rand_int() and max_num_reads
      for i in 0 to max_num_reads loop
        rand_addr := rand_int(min_val => 0, max_val => DCACHE_NUM_BLOCKS * cache_offset_size_c + 1);
        -- Read data that is not in cache, report error if successful
        wait until rising_edge(clk_gen);
        host_re           <= '1';
        host_addr         <= std_ulogic_vector(to_unsigned(rand_addr, 32));

        wait until rising_edge(clk_gen);
        host_re           <= '0';
        bad_data_read_err <= or_reduce_f(hit);

        rand_addr := rand_int(min_val => 0, max_val => DCACHE_NUM_BLOCKS * cache_offset_size_c + 1);
        -- Write then read with random address, report error if unsuccessful
        wait until rising_edge(clk_gen);
        ctrl_en           <= '1';
        ctrl_we           <= '1';
        ctrl_tag_we       <= '1';
        ctrl_valid_we     <= '1';
        ctrl_addr         <= std_ulogic_vector(to_unsigned(rand_addr, 32));
        ctrl_wdata        <= cache_ext_mem(rand_addr*4); -- From neorv32_dcache_memory_tb_pkg.vhd (run dmem_gen.py to generate)

        wait until rising_edge(clk_gen);
        ctrl_en           <= '0';
        ctrl_we           <= '0';
        ctrl_tag_we       <= '0';
        ctrl_valid_we     <= '0';

        wait until rising_edge(clk_gen);
        host_re           <= '1';

        wait until rising_edge(clk_gen);
        host_re           <= '0';

        wait until rising_edge(clk_gen);
        data_read_err     <= not(or_reduce_f(hit));
      end loop;
      
      wait for 2*t_clock_c;
      wait until rising_edge(clk_gen);
      tb_finished       <= '1';

    end if;  --init_mem = '1'
  end process; --run_test


  -- Design Under Test--------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- Generate loop allows testing all associativity options at once
  n_way_assoc_mem : for n in 0 to ASSOCIATIVITY-1 generate --actually (2^n)-way associative memory
  neorv32_dcache_memory_inst_x: neorv32_dcache_memory
  generic map (
    DCACHE_NUM_BLOCKS  => DCACHE_NUM_BLOCKS,      -- number of blocks (min 1), has to be a power of 2
    DCACHE_BLOCK_SIZE  => DCACHE_BLOCK_SIZE,      -- block size in bytes (min 4), has to be a power of 2
    ASSOCIATIVITY      => 2**n,                   -- associativity; 0=direct-mapped, 1=2-way set-associative
      DCACHE_REPLACE_POL => DCACHE_REPLACE_POL      -- cache replacement policy; 1=LRU, 2=Pseudo-LRU, 3=FIFO, 4=Random
      )
      port map (
        -- global control --
        clk_i          => clk_gen,                    -- global clock, rising edge
        invalidate_i   => rst_gen,                    -- invalidate whole cache
        -- host cache access (read-only) --
      host_addr_i    => host_addr,                  -- access address
      host_re_i      => host_re,                    -- read enable
      host_rdata_o   => host_rdata(n),              -- read data
      -- access status (1 cycle delay to access) --
      hit_o          => hit(n),                     -- hit access
      -- ctrl cache access (write-only) --
      ctrl_en_i      => ctrl_en,                    -- control interface enable
      ctrl_addr_i    => ctrl_addr,                  -- access address
      ctrl_we_i      => ctrl_we,                    -- write enable (full-word)
      ctrl_wdata_i   => ctrl_wdata,                 -- write data
      ctrl_tag_we_i  => ctrl_tag_we,                -- write tag to selected block
      ctrl_valid_i   => ctrl_valid_we,              -- make selected block valid
      ctrl_invalid_i => ctrl_invalid_we             -- make selected block invalid
      );
  
  -- count the hits and reads
  counter_process : process(clk_gen)
  begin
    if (rst_gen = '0') then
      if (hit(n) = '1') then
        hit_count(n) <= hit_count(n) + 1;
      end if;
      if (host_re = '1') then
        read_count(n) <= read_count(n) + 1;
      end if;
    else  -- rst_gen = '1'
      hit_count(n)  <= 0;
      read_count(n) <= 0;
    end if;
  end process counter_process;
  
  -- report number of hits and number of reads for each associativity
  report_rates : process (tb_finished, timeout_err)
    variable hit_rate : real;
    variable miss_rate : real;
  begin
    if (tb_finished = '1' or timeout_err = '1') then
      hit_rate := real(hit_count(n)) / real(read_count(n)) * real(100);
      miss_rate := real((read_count(n) - hit_count(n))) / real(read_count(n)) * real(100);
      if n = 0 then
        report lf & "Direct-Mapped HITS: " & integer'image(hit_count(n)) & lf &
                    "Direct-Mapped READS: " & integer'image(read_count(n)) & lf &
                    "Hit Rate (%): " & real'image(hit_rate) & lf &
                    "Miss Rate (%): " & real'image(miss_rate) & lf;
      else
        report lf & integer'image(2**n) & "-way HITS: " & integer'image(hit_count(n)) & lf &
                    integer'image(2**n) & "-way READS: " & integer'image(read_count(n)) & lf &
                    "Hit Rate (%): " & real'image(hit_rate) & lf &
                    "Miss Rate (%): " & real'image(miss_rate) & lf;
      end if;
    end if;  
  end process report_rates;

  end generate;
end tb_neorv32_dcache_memory_rtl;
      