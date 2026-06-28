# =========================================================================
# Synopsys Design Constraints (SDC) for ascan_buffer Clock Domain Crossing
# Target Analyzer: Intel Quartus Prime TimeQuest Timing Analyzer
# =========================================================================

# -------------------------------------------------------------------------
# 1. Declare Asynchronous Relations Between ADC and System Domains
# -------------------------------------------------------------------------
# Cuts all timing paths between the 65 MHz ADC domain and the 80 MHz System domain.
# This successfully covers the asynchronous DP-RAM boundary inside the buffer.
set_clock_groups -asynchronous \
    -group [get_clocks -nocase {*adc_clk*}] \
    -group [get_clocks -nocase {*sys_clk*}]

# -------------------------------------------------------------------------
# 2. Timing Cuts on Control Synchronizers (ascan_sync)
# -------------------------------------------------------------------------
# Ignore setup and hold timing violations on the input of the first stage 
# of the synchronizers (u_sync_clear_0, u_sync_clear_1, u_sync_avail_0, u_sync_avail_1).
# Metastability here is expected and resolved by the second stage.
set_false_path -to [get_registers {*ascan_buffer*|ascan_sync:*|sync_reg_0[*]}]

# -------------------------------------------------------------------------
# 3. Max Delay Enforcement on Cross-Domain Paths (Placement Constraint)
# -------------------------------------------------------------------------
# Enforces the Fitter to place the cross-domain registers physically close 
# to each other on the FPGA silicon die. This limits the routing delay 
# to a maximum of 5.0 ns, ensuring reliable MTBF and low transport latency.
set_max_delay -to [get_registers {*ascan_buffer*|ascan_sync:*|sync_reg_0[*]}] 5.000