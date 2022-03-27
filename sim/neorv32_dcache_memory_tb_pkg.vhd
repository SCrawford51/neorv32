library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package neorv32_dcache_memory_tb_pkg is

	constant cache_size : integer := 8;

	type ext_mem_type is array (0 to 15) of std_ulogic_vector(31 downto 0);

	constant cache_ext_mem : ext_mem_type := (
		00000000 => x"6b3d9b3",
		00000001 => x"22433ed9",
		00000002 => x"b28eed7",
		00000003 => x"626d29be",
		00000004 => x"34207f83",
		00000005 => x"221e6f79",
		00000006 => x"dc91f05",
		00000007 => x"6b54154a",
		00000008 => x"735a7c19",
		00000009 => x"6ffa4a47",
		00000010 => x"4e2382a",
		00000011 => x"3087256a",
		00000012 => x"44a244be",
		00000013 => x"47fa0f46",
		00000014 => x"2a857444",
		00000015 => x"2b9f46bb",
	);

end package neorv32_dcache_memory_tb_pkg;
