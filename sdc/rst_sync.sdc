# =========================================================================
# File: src/rst_sync.sdc
# Description: Synopsys Design Constraints for 16-Channel UT Defectoscope
#              Handles PLL-derived clocks, synchronous resets, and CDC.
# Target: Intel Quartus Prime / TimeQuest Timing Analyzer
# =========================================================================

# -------------------------------------------------------------------------
# 1. PLL Clocks Derivation (Quartus Specific as per ai_context.md)
# -------------------------------------------------------------------------
# Automatically creates clocks on all PLL output pins with correct periods:
#  - sys_clk : 80 MHz (12.5 ns)
#  - adc_clk : 65 MHz (15.385 ns)
#  - log_clk : 25 MHz (40.0 ns)
#  - dac_clk : 50 MHz (20.0 ns)
#  - hi_clk  : 250 MHz (4.0 ns)
derive_pll_clocks -create_base_clocks

# Automatically calculate setup/hold clock uncertainty (jitter) for FPGA
derive_clock_uncertainty

# fallback/Standalone definitions (only active if PLL is not found in the netlist)
if { [get_collection_size [get_clocks -nowarn {*sys_clk*}]] == 0 } {
    post_message -type info "PLL clocks not found. Creating virtual base clocks for standalone run."
    create_clock -name sys_clk -period 12.500 [get_ports sys_clk]
    create_clock -name adc_clk -period 15.385 [get_ports adc_clk]
    create_clock -name log_clk -period 40.000 [get_ports log_clk]
    create_clock -name dac_clk -period 20.000 [get_ports dac_clk]
    create_clock -name hi_clk  -period  4.000 [get_ports hi_clk]
}

# -------------------------------------------------------------------------
# 2. Clock Groups (Asynchronous Domains)
# -------------------------------------------------------------------------
# Since we cross domains using CDC (cdc_pulse_sync / toggle synchronizers),
# we define all these clock domains as mutually asynchronous.
# Using wildcards (*) ensures it matches both derived PLL clocks and manual clocks.
set_clock_groups -asynchronous \
    -group [get_clocks -nowarn {*sys_clk*}] \
    -group [get_clocks -nowarn {*adc_clk*}] \
    -group [get_clocks -nowarn {*log_clk*}] \
    -group [get_clocks -nowarn {*dac_clk*}] \
    -group [get_clocks -nowarn {*hi_clk*}]

# -------------------------------------------------------------------------
# 3. False Paths & Timing Exceptions
# -------------------------------------------------------------------------
# Asynchronous master reset input (rst_n) is synchronized inside 'rst_gen'.
# We cut the path from the physical input port to the first synchronization stage.
set_false_path -from [get_ports {rst_n}]

# For the CDC pulse synchronizers (cdc_pulse_sync), we cut the timing path 
# from the 'src_toggle' register to the first stage of the synchronizer.
# This prevents timing violations on the metastability-prone node.
set_false_path -to [get_registers {*cdc_pulse_sync:*|dst_sync_reg[0]}]

# Apply max delay constraint to ensure the compiler places synchronizer 
# registers physically close to each other, minimizing MTBF.
set_max_delay -to [get_registers {*cdc_pulse_sync:*|dst_sync_reg[0]}] 4.000

# -------------------------------------------------------------------------
# 4. Input Constraints
# -------------------------------------------------------------------------
# i_sys_sync arrives in the sys_clk domain. 
# We constrain it relative to sys_clk (assuming a standard 3ns setup window).
set_input_delay -clock [get_clocks {*sys_clk*}] -max 3.000 [get_ports {i_sys_sync}]
set_input_delay -clock [get_clocks {*sys_clk*}] -min 0.500 [get_ports {i_sys_sync}]
