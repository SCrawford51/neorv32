-- #################################################################################################
-- Random Block Selector
-- Author - Angel Silva
-- Date - 4/2/2022
-- Description - Uses random number generator from Joris van Rantwijk to generate a 32-bit number
-- and use 32-bit number to generate a random number between 0 and 1 by counting number of ones in 
-- 32-bit number. An odd number of ones will select block 0 and an even number of ones will select
-- block 1. 
-- #################################################################################################
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity random_selector is
    port (
         -- Clock, rising edge active.
         clk_i :        in  std_logic;
         set_select_o : out std_logic;
    );
  end random_selector;

  architecture random_selector_rtl of random_selector is

    signal rand_ready : std_logic;
    signal rand_valid : std_logic;
    signal rand_data  : std_logic_vector(31 downto 0); 
    signal rand_rst   : std_logic;
    signal rand_cnt   : std_logic_vector(7 downto 0) := x"00";
    signal ones_cnt   : std_logic_vector(7 downto 0) := x"00";

    -- Random Reset Generator --------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    rand_rst_gen: process(clk_i)
    begin
        if rising_edge(clk_i) then
            if(rst_cnt < x"0F") then
                rand_rst <= '1';
                rst_cnt <= rst_cnt + '1';
            else
                rand_rst <= '0';
            end if;
        end if;
    end process rand_rst_gen;

    -- Random Way Generation ---------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    rand_one_cnt: process(rand_data) -- count number of ones in 32-bit random data
    begin
        ones_cnt <= x"00";
        for i in 0 to 31 loop
        if(rand_data(i) = '1') then
            ones_cnt <= ones_cnt + '1';
        end if;
        end loop;
    end process rand_one_cnt;

    rand_way_gen: process(clk_i) -- Determine whether number of ones is odd or even to determine set
    begin
        if rising_edge(clk_i) then
            if(ones_cnt(0) = '1') then -- Odd number
                set_select_o <= '0';
            else -- Even number
                set_select_o <= '1';
            end if;
        end if;
    end process rand_way_gen;

    random_gen_inst : rng_mt19937 -- 32-bit random number generator
    generic map (
        init_seed       => x"DCDCABAB",
        force_const_mul => false
    ) 
    port map (
        clk       => clk_i,
        rst       => rand_rst,
        reseed    => '0',
        newseed   => x"AAAACCCC",
        out_ready => rand_ready,
        out_valid => rand_valid,
        out_data  => rand_data
    );
    
  end random_selector_rtl;