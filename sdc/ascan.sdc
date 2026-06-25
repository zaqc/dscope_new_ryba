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
# All PLL output clock domains are mutually asynchronous. We cut timing 
# paths between them. Pattern matching is used to robustly identify clocks 
# regardless of PLL hierarchy names.
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
# This prevents TimeQuest from analyzing setup/hold on the metastability-prone node.
set_false_path -to [get_registers {*ascan_sync*|sync_reg_0[*]}]

# Apply False Paths for reset lines crossing clock domains asynchronously
set_false_path -from [get_ports {adc_rst}] -to [get_registers *]
set_false_path -from [get_ports {sys_rst}] -to [get_registers *]

# Limit max delay on CDC paths to prevent long routing detours
# (Ensures that the physical layout places synchronizer registers close to each other)
set_max_delay -from [get_registers *] -to [get_registers {*ascan_sync*|sync_reg_0[*]}] 4.000

# -------------------------------------------------------------------------
# 4. Input & Output Delays (External Interfaces)
# -------------------------------------------------------------------------

# ADC Input Pins Timing (referenced to the derived adc_clk clock)
# Dynamic retrieval of adc_clk to accommodate any PLL prefix
set adc_clk_ref [get_clocks -nocase {*adc_clk*}]
if { [length_collection $adc_clk_ref] > 0 } {
    set_input_delay -clock $adc_clk_ref -max 3.500 [get_ports {i_adc_data[*] i_adc_sync}]
    set_input_delay -clock $adc_clk_ref -min 1.500 [get_ports {i_adc_data[*] i_adc_sync}]
}

# System Stream Output Pins Timing (referenced to the derived sys_clk clock)
# Dynamic retrieval of sys_clk to accommodate any PLL prefix
set sys_clk_ref [get_clocks -nocase {*sys_clk*}]
if { [length_collection $sys_clk_ref] > 0 } {
    set_output_delay -clock $sys_clk_ref -max 4.000 [get_ports {o_out_data[*] o_out_vld o_out_size[*] o_data_ready}]
    set_output_delay -clock $sys_clk_ref -min -1.000 [get_ports {o_out_data[*] o_out_vld o_out_size[*] o_data_ready}]
    set_input_delay  -clock $sys_clk_ref -max 3.000 [get_ports {i_out_rdy}]
    set_input_delay  -clock $sys_clk_ref -min 1.000 [get_ports {i_out_rdy}]
}