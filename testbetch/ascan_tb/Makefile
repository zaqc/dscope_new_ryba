VERILATOR=verilator
IVERILOG=iverilog
VVP=vvp

# Target executable
TARGET = ascan_tb.vvp
VCD_FILE = ascan_tb.vcd
WAVE_VCD = ascan_tb.vcd
WAVE_CFG = ascan_tb.gtkw

# Source files
SOURCES = ../../src/ascan.v ../../src/rst_sync.v ../../src/dp_ram.v ascan_tb.v

# Compiler flags
FLAGS = -Wall -Winfloop -g2005-sv -DTESTMODE=1 -DTARGET_NAME="\""$(TARGET)"\""

all: run

build:
	$(IVERILOG) $(FLAGS) -o $(TARGET) $(SOURCES)

run: build
	$(VVP) $(TARGET)

wave: run
	gtkwave --rcvar 'fontname_signals Monospace 11' $(WAVE_VCD) $(WAVE_CFG)

clean:
	rm -f $(TARGET) *.vcd
