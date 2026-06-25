# =========================================================================
# Synopsys Design Constraints (SDC) for A-Scan Processor Project
# Target Analyzer: Quartus Prime TimeQuest Timing Analyzer
# Note: Base and derived PLL clocks are defined in the auto-generated PLL SDC.
# =========================================================================

# -------------------------------------------------------------------------
# 1. Clock Uncertainty (Jitter & Guard Bands)
# -------------------------------------------------------------------------
# Calculate clock uncertainty dynamically based on device characteristics
derive_clock_uncertainty

# -------------------------------------------------------------------------
# 2. Asynchronous Clock Groups (Cut Paths Between Domains)
# -------------------------------------------------------------------------
# All PLL output clock domains are mutually asynchronous according to the spec.
# We cut timing paths between them. Pattern matching is used to robustly identify
# clocks regardless of PLL hierarchy names.
set_clock_groups -asynchronous \
    -group [get_clocks -nocase {*sys_clk*}] \
    -group [get_clocks -nocase {*adc_clk*}] \
    -group [get_clocks -nocase {*log_clk*}] \
    -group [get_clocks -nocase {*dac_clk*}] \
    -group [get_clocks -nocase {*hi_clk*}]

# -------------------------------------------------------------------------
# 3. Explicit Timing Cuts for CDC (Clock Domain Crossing) Elements
# -------------------------------------------------------------------------

# Cut the path to the input of 2-stage synchronizers (ascan_sync)
# Matches both vector and scalar registers safely (e.g. sync_reg_0 or sync_reg_0[0])
set_false_path -to [get_registers -nowarn {*ascan_sync*|sync_reg_0*}]

# Apply False Paths for the global external asynchronous reset line rst_n
# (Targets the physical input port defined in ai_context.md)
if { [length_collection [get_ports -nowarn {rst_n}]] > 0 } {
    set_false_path -from [get_ports {rst_n}] -to [get_registers *]
}

# Limit max delay on CDC paths to prevent excessively long routing detours.
# Note: Since set_clock_groups cuts paths globally, this max_delay serves as a 
# routing constraint for physical placement of synchronizers.
set_max_delay -from [get_registers *] -to [get_registers -nowarn {*ascan_sync*|sync_reg_0*}] 4.000

# -------------------------------------------------------------------------
# 4. Input & Output Delays (External Interfaces Only)
# -------------------------------------------------------------------------

# ADC Input Pins Timing (referenced to the derived adc_clk clock)
# Note: i_adc_sync is internal CDC signal from rst_sync and is excluded from get_ports.
set adc_clk_ref [get_clocks -nocase {*adc_clk*}]
if { [length_collection $adc_clk_ref] > 0 } {
    set adc_ports [get_ports -nowarn {i_adc_data[*]}]
    if { [length_collection $adc_ports] > 0 } {
        set_input_delay -clock $adc_clk_ref -max 3.500 $adc_ports
        set_input_delay -clock $adc_clk_ref -min 1.500 $adc_ports
    }
}

# System Stream Output Pins Timing (referenced to the derived sys_clk clock)
# Dynamic retrieval of sys_clk to accommodate any PLL prefix
set sys_clk_ref [get_clocks -nocase {*sys_clk*}]
if { [length_collection $sys_clk_ref] > 0 } {
    set sys_out_ports [get_ports -nowarn {o_out_data[*] o_out_vld o_out_size[*] o_data_ready}]
    if { [length_collection $sys_out_ports] > 0 } {
        set_output_delay -clock $sys_clk_ref -max 4.000 $sys_out_ports
        set_output_delay -clock $sys_clk_ref -min -1.000 $sys_out_ports
    }
    
    set sys_in_ports [get_ports -nowarn {i_out_rdy}]
    if { [length_collection $sys_in_ports] > 0 } {
        set_input_delay  -clock $sys_clk_ref -max 3.000 $sys_in_ports
        set_input_delay  -clock $sys_clk_ref -min 1.000 $sys_in_ports
    }
}