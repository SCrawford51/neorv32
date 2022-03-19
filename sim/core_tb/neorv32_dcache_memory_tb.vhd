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

entity tb_neorv32_dcache_memory is
  generic (
    CLOCK_FREQUENCY   : natural := 50000000; -- clock frequency of clk_i in Hz
    DCACHE_NUM_BLOCKS : natural := 4;        -- number of blocks (min 1), has to be a power of 2
    DCACHE_BLOCK_SIZE : natural := 16;       -- block size in bytes (min 4), has to be a power of 2
    DCACHE_NUM_SETS   : natural := 1;        -- associativity; 1=direct-mapped, 2=2-way set-associative
    SELF_TERM         : boolean := true      -- When true the testbench will stop running if pass/fail has been determined
  );
end tb_neorv32_dcache_memory;

architecture tb_neorv32_dcache_memory_rtl of tb_neorv32_dcache_memory is

  -- cache layout --
  constant cache_entries_c : natural    := DCACHE_NUM_BLOCKS * (DCACHE_BLOCK_SIZE/4); -- number of 32-bit entries (per set)

  -- internals - hands off! --
  constant t_clock_c       : time       := (1 sec) / CLOCK_FREQUENCY;

  -- generators --
  signal run_tb            : boolean    := true;
  signal clk_gen           : std_ulogic := '0';
  signal rst_gen           : std_ulogic := '0';

  -- testbench signals
  signal init_mem          : std_ulogic := '1';
  signal invalid_test      : std_ulogic := '1';
  signal ctrl_en           : std_ulogic := '0';
  signal ctrl_we           : std_ulogic := '0';
  signal ctrl_tag_we       : std_ulogic := '0';
  signal ctrl_valid_we     : std_ulogic := '0';
  signal ctrl_invalid_we   : std_ulogic := '0';
  signal ctrl_addr         : std_ulogic_vector(31 downto 0);
  signal ctrl_wdata        : std_ulogic_vector(31 downto 0);

  type host_rdata_t is array (0 to DCACHE_NUM_SETS-1) of std_ulogic_vector(31 downto 0);
  
  signal host_re           : std_ulogic := '0';
  signal host_addr         : std_ulogic_vector(31 downto 0);
  signal host_rdata        : host_output_t;

  signal hit               : std_ulogic_vector(DCACHE_NUM_SETS-1 downto 0);

  signal invalid_err       : std_ulogic := '0';
  signal invalid_block_err : std_ulogic := '0';
  
begin

  -- Clock/Reset Generator ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  clk_gen <= not clk_gen after (t_clock_c/2) while run_tb;

  -- Testbench error/completion messages -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (invalid_err)       report "Memory reported a cache hit while entire cache is invalid."       severity error;
  assert not (invalid_block_err) report "Memory reported a cache hit while the selected block is invalid." severity error;
      
  assert not (tb_finished)       report "Testbench successful."                                            severity note;

  -- Self terminate the testbench if desired ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  self_term: if SELF_TERM generate 
    report_err : process (invalid_err, invalid_block_err, tb_finished)
    begin
      run_tb <= false;
    end process;
  end generate self_term;

  -- Main control process ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  run_test : process(clk_gen) 
  begin
    if rising_edge(clk_gen) then
      if init_mem = '1' then -- Fill cache from memory at beginning of testbench
        for ii in 0 to cache_entries_c loop
          ctrl_en       <= '1';
          ctrl_we       <= '1';
          ctrl_tag_we   <= '1';
          ctrl_valid_we <= '1';
          ctrl_addr     <= std_ulogic_vector(ii);
          ctrl_wdata    <= ext_mem(ii); -- From XXX.vhd (run XXX.py to generate)
          
          wait t_clock_c;
          ctrl_en       <= '0';
          ctrl_we       <= '0';
          ctrl_tag_we   <= '0';
          ctrl_valid_we <= '0';
          host_re       <= '1';
          host_addr     <= std_ulogic_vector(ii);

          wait until host_rdata(0) = ctrl_wdata;
          host_re <= '0';
        end loop;
        init_mem <= '0';

      elsif invalid_test = '1' then -- Tests while sets are invalid
        rst_gen           <= '1';
        host_re           <= '1';
        host_addr         <= std_ulogic_vector(0);

        wait 5*t_clock_c;
        invalid_err       <= or_reduce_f(hit);

        wait 5*t_clock_c;
        rst_gen           <= '0';
        ctrl_valid_we     <= '1';

        wait 5*t_clock_c;
        ctrl_valid_we     <= '0';

        wait 5*t_clock_c;
        invalid_block_err <= or_reduce_f(hit);

        wait 5*t_clock_c;
        ctrl_valid_we     <= '1';

        wait 5*t_clock_c;
        ctrl_valid_we     <= '0';
        invalid_test      <= '0';
        host_re           <= '0';

      else -- Tests while sets are valid
        host_re           <= '1';
        

      end if;
    end if;
  end process; --fill_mem


  -- Design Under Test--- ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- Generate loop allows testing all associativy options at once
  n_way_assoc_mem : for n in 0 to DCACHE_NUM_SETS-1 generate --actually (2^n)-way associative memory
    neorv32_dcache_memory_inst_x: neorv32_dcache_memory
    generic map (
      DCACHE_NUM_BLOCKS => DCACHE_NUM_BLOCKS, -- number of blocks (min 1), has to be a power of 2
      DCACHE_BLOCK_SIZE => DCACHE_BLOCK_SIZE, -- block size in bytes (min 4), has to be a power of 2
      DCACHE_NUM_SETS   => n                  -- associativity; 0=direct-mapped, 1=2-way set-associative
    )
    port map (
      -- global control --
      clk_i          => clk_gen,              -- global clock, rising edge
      invalidate_i   => rst_gen,              -- invalidate whole cache
      -- host cache access (read-only) --
      host_addr_i    => host_addr,            -- access address
      host_re_i      => host_re,              -- read enable
      host_rdata_o   => host_rdata,           -- read data
      -- access status (1 cycle delay to access) --
      hit_o          => hit(n),               -- hit access
      -- ctrl cache access (write-only) --
      ctrl_en_i      => ctrl_en,              -- control interface enable
      ctrl_addr_i    => ctrl_addr,            -- access address
      ctrl_we_i      => ctrl_we,              -- write enable (full-word)
      ctrl_wdata_i   => ctrl_wdata,           -- write data
      ctrl_tag_we_i  => ctrl_tag_we,          -- write tag to selected block
      ctrl_valid_i   => ctrl_valid_we,        -- make selected block valid
      ctrl_invalid_i => ctrl_invalid_we       -- make selected block invalid
    );
  end generate;

end tb_neorv32_dcache_memory_rtl;