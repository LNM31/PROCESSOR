#!/bin/bash
set -e

iverilog -o lic.vvp ALU_tb.v 
vvp lic.vvp
gtkwave wave.gtkw 