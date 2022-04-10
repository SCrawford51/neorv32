-- #################################################################################################
-- Random Block Selector
-- Author - Angel Silva
-- Date - 4/2/2022
-- Description - Uses random number generator from Joris van Rantwijk to generate a 32-bit random
-- number and use the last n-bits of the number to generate a random number to select the block in 
-- the cache.
-- #################################################################################################
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_arith.all;

library neorv32;
use neorv32.neorv32_package.all;

entity random_selector is
    generic (
        init_seed       : std_logic_vector(31 downto 0);
        force_const_mul : boolean 
    );
    port (
         -- Clock, rising edge active.
         clk_i        : in  std_logic;
         reseed       : in  std_logic;
         newseed      : in  std_logic_vector(31 downto 0);
         rand_ready   : out std_logic;
         rand_valid   : out std_logic;
         rand_data    : out std_logic_vector(31 downto 0)
    );
  end random_selector;

architecture random_selector_rtl of random_selector is

    signal rand_rst   : std_logic;
    signal rst_cnt    : std_logic_vector(7 downto 0);



begin

    -- Random Reset Generator --------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    rand_rst_gen : process(clk_i)
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

    random_gen_inst : rng_mt19937 -- 32-bit random number generator
    generic map (
        init_seed       => init_seed,
        force_const_mul => force_const_mul
    ) 
    port map (
        clk       => clk_i,
        rst       => rand_rst,
        reseed    => reseed,
        newseed   => newseed,
        out_ready => rand_ready,
        out_valid => rand_valid,
        out_data  => rand_data
    );
    
  end random_selector_rtl;