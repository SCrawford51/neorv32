#!/usr/bin/env python3

"""Generates VHDL file to be used for memory storage in test benches

  Usage (while in /sim directory): python dmem_gen.py cache_size file_name

  cache_size = integer arg. representing size of the cache. DEFAULTS to 256.
  file_name = string arg. representing the name of the vhdl file (.vhd extension not needed)
        DEFAULTS to neorv32_dcache_memory_tb_pkg.vhd
"""

# imports
import random
import argparse
from pathlib import Path


random.seed(123)

MAX_INT = (2**31) - 1       # max possible 32-bit value

FILE_EXT = '.vhd'           # file extension of output file

LIBS = [
    'library ieee;\n',
    'use ieee.std_logic_1164.all;\n',
    'use ieee.numeric_std.all;\n'
]


def parse_args() -> argparse.Namespace:
    """Parses command line arguments
    Possible args are cache_size and file_name. See docstring at top of file
    for descriptions.

    Returns:
        argparse.Namespace: object containing all arguments
    """
    parser = argparse.ArgumentParser()

    # Add positional/optional arguments
    parser.add_argument(
        'cache_size',
        help='Size of the cache (in bytes)',
        type=int,
        default=256)
    parser.add_argument(
        '--file_name',
        help='Name of destination file (extension excluded)',
        default=f'neorv32_dcache_memory_tb_pkg{FILE_EXT}')

    args = parser.parse_args()

    return args


def format_int_to_hex(num: int) -> str:
    """Converts an integer to a hexadecimal number formatted for VHDL (.vhd) files

    Args:
        num (int): decimal to convert to hex

    Returns:
        str: string representation of converted hex number
    """

    hex_num = hex(num)
    hex_num = f'{hex_num[1]}"{hex_num[2:]}"'
    return hex_num


def create_vhd_pkg(file_name: Path, cache_size: int) -> None:
    with open(file_name, 'w') as f_out:
        # add libraries
        f_out.writelines(LIBS)
        f_out.write('\n')

        # define package
        f_out.write(f'package {file_name.with_suffix("")} is\n\n')

        # create constant for cache_size
        f_out.write(f'\tconstant cache_size : integer := {cache_size};\n\n')

        # create array type
        f_out.write(
            f'\ttype ext_mem_type is array (0 to {2 * cache_size - 1}) of std_ulogic_vector(31 downto 0);\n\n')

        # create memory
        f_out.write(f'\tconstant cache_ext_mem : ext_mem_type := (\n')
        for i in range(2 * cache_size):
            rand_hex = format_int_to_hex(random.randrange(MAX_INT))
            f_out.write(f'\t\t{i:0>8} => {rand_hex},\n')
        f_out.write('\t);\n\n')

        # end package
        f_out.write(f'end package {file_name.with_suffix("")};')


def main() -> None:
    # Parse arguments
    args = parse_args()
    cache_size = args.cache_size
    file_name = Path(args.file_name).with_suffix(FILE_EXT)

    # create output file
    create_vhd_pkg(file_name, cache_size)


if __name__ == '__main__':
    main()
