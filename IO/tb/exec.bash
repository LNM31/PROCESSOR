#!/bin/bash
set -e

iverilog -o tb/lic io_tb.v 
tb/lic
gtkwave tb/wave.gtkw