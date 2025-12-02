## This file is a general .xdc for the ARTY Z7-20 Rev.B adapted for RISC-V Processor

## ----------------------------------------------------------------------------
## 1. Clock Signals
## ----------------------------------------------------------------------------
# Mapping clock_proc to the 125 MHz System Clock (H16)
# This allows Vivado to perform Timing Analysis for your report.
set_property -dict { PACKAGE_PIN H16    IOSTANDARD LVCMOS33 } [get_ports { clock_proc }]; #IO_L13P_T2_MRCC_35 Sch=SYSCLK
create_clock -add -name sys_clk_pin -period 8.00 -waveform {0 4} [get_ports { clock_proc }];

# Mapping clock_mem to Button 1 (D20) for manual control or placeholder.
# We allow non-dedicated routing because a push button is driving a clock net.
set_property -dict { PACKAGE_PIN D20    IOSTANDARD LVCMOS33 } [get_ports { clock_mem }]; #IO_L4N_T0_35 Sch=BTN1
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets clock_mem_IBUF]

## ----------------------------------------------------------------------------
## 2. Reset
## ----------------------------------------------------------------------------
# Mapping rst to Button 0 (D19)
set_property -dict { PACKAGE_PIN D19    IOSTANDARD LVCMOS33 } [get_ports { rst }]; #IO_L4P_T0_35 Sch=BTN0

## ----------------------------------------------------------------------------
## 3. Outputs
## ----------------------------------------------------------------------------
# Mapping halt to LED 0 (R14)
set_property -dict { PACKAGE_PIN R14    IOSTANDARD LVCMOS33 } [get_ports { halt }]; #IO_L6N_T0_VREF_34 Sch=LED0

## ----------------------------------------------------------------------------
## Configuration
## ----------------------------------------------------------------------------
# Standard configuration for Arty Z7
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]