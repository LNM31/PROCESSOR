#!/bin/bash
set -e

iverilog -o lic ../ALU_tb2.v -c ../../../files_debian.txt
vvp lic -fst
gtkwave wave.gtkw