#!/bin/bash

# Script pentru compilare si rulare testbench-uri PC si SP
# Folosire: ./exec.bash [pc|sp|all]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REG_DIR="$(dirname "$SCRIPT_DIR")"
ALU_DIR="$REG_DIR/../ALU16"

# Include paths
INCLUDES="-I $ALU_DIR/Combinational/muxes -I $ALU_DIR/Combinational/RCA -I $ALU_DIR/Registers" 

# Culori pentru output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

run_pc_test() {
    echo -e "${YELLOW}=== Compilare PC ===${NC}"
    cd "$REG_DIR"
    iverilog $INCLUDES -o tb/pc_tb PC.v PC_tb.v \
        "$ALU_DIR/Registers/ffd.v" \
        "$ALU_DIR/Combinational/muxes/mux_2s.v" \
        "$ALU_DIR/Combinational/RCA/RCA.v"
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Compilare PC reusita!${NC}"
        echo -e "${YELLOW}=== Rulare PC Testbench ===${NC}"
        vvp tb/pc_tb
    else
        echo -e "${RED}Eroare la compilare PC!${NC}"
        return 1
    fi
}

run_sp_test() {
    echo -e "${YELLOW}=== Compilare SP ===${NC}"
    cd "$REG_DIR"
    iverilog $INCLUDES -o tb/sp_tb SP.v SP_tb.v \
        "$ALU_DIR/Registers/ffd.v" \
        "$ALU_DIR/Combinational/muxes/mux_2s.v" \
        "$ALU_DIR/Combinational/RCA/RCA.v"
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Compilare SP reusita!${NC}"
        echo -e "${YELLOW}=== Rulare SP Testbench ===${NC}"
        vvp tb/sp_tb
    else
        echo -e "${RED}Eroare la compilare SP!${NC}"
        return 1
    fi
}

# Creare folder tb daca nu exista
mkdir -p "$REG_DIR/tb"

case "${1:-all}" in
    pc)
        run_pc_test
        ;;
    sp)
        run_sp_test
        ;;
    all)
        run_pc_test
        echo ""
        run_sp_test
        ;;
    *)
        echo "Folosire: $0 [pc|sp|all]"
        echo "  pc  - Ruleaza doar testbench-ul PC"
        echo "  sp  - Ruleaza doar testbench-ul SP"
        echo "  all - Ruleaza ambele testbench-uri (implicit)"
        exit 1
        ;;
esac

echo -e "\n${GREEN}=== Fisiere VCD generate in tb/ ===${NC}"
ls -la "$REG_DIR/tb/"*.vcd 2>/dev/null || echo "Niciun fisier VCD generat inca."
