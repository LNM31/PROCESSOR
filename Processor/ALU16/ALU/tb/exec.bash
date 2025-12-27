#!/bin/bash
set -e

iverilog -o lic ../ALU_tb2.v -c ../../../../files.txt
vvp lic -fst
gtkwave wave.gtkw