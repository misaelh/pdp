
vlib work
vlib grlib

vcom -quiet  -93  -work grlib  ../simlib/grlib/stdlib/version.vhd
vcom -quiet  -93  -work grlib  ../simlib/grlib/stdlib/stdlib.vhd
vcom -quiet  -93  -work grlib  ../simlib/grlib/stdlib/stdio.vhd
vcom -quiet  -93  -work work   ../simlib/micron/ddr_sdram/mti_pkg.vhd
vcom -quiet  -93  -work work   ../simlib/micron/ddr_sdram/mt46v16m16.vhd
vcom -quiet  -93  -work work   ../rtl/mlite_pack.vhd
vcom -quiet  -93  -work work   ../rtl/alu.vhd
vcom -quiet  -93  -work work   ../rtl/bus_mux.vhd
vcom -quiet  -93  -work work   ../rtl/control.vhd
vcom -quiet  -93  -work work   ../rtl/mem_ctrl.vhd
vcom -quiet  -93  -work work   ../rtl/mult.vhd
vcom -quiet  -93  -work work   ../rtl/pc_next.vhd
vcom -quiet  -93  -work work   ../rtl/reg_bank.vhd
vcom -quiet  -93  -work work   ../rtl/pipeline.vhd
vcom -quiet  -93  -work work   ../rtl/shifter.vhd
vcom -quiet  -93  -work work   ../rtl/uart.vhd
vcom -quiet  -93  -work work   ../rtl/cache_ram.vhd
vcom -quiet  -93  -work work   ../rtl/cache.vhd
vcom -quiet  -93  -work work   boot_ram_sim.vhd
vcom -quiet  -93  -work work   ../rtl/mlite_cpu.vhd
vcom -quiet  -93  -work work   ../rtl/plasma.vhd
vcom -quiet  -93  -work work   ../rtl/ddr_ctrl.vhd
vcom -quiet  -93  -work work   ../rtl/ddr_init.vhd
vcom -quiet  -93  -work work   ../rtl/ddr_ctrl_top.vhd
vcom -quiet  -93  -work work   ../rtl/clk_gen.vhd
vcom -quiet  -93  -work work   ../rtl/plasma_top.vhd
vcom -quiet  -93  -work work   sim_tb_top.vhd


vsim -t ps -novopt -L unisim work.sim_tb_top

onerror {resume}
#Log all the objects in design. These will appear in .wlf file#
log -r /*
#View sim_tb_top signals in waveform#
#add wave -group sim_top sim:/sim_tb_top/*
add wave -group plasma_top sim:/sim_tb_top/u1_plasma_top/*
add wave -group ddr sim:/sim_tb_top/u1_plasma_top/u2_ddr/*

quietly virtual signal -install sim:/sim_tb_top/u1_plasma_top/u1_plasma/u1_cpu/u1_pc_next { sim:/sim_tb_top/u1_plasma_top/u1_plasma/u1_cpu/u1_pc_next/pc_current(19 downto 8)} PC
quietly virtual signal -install sim:/sim_tb_top/u1_plasma_top/u1_plasma/u1_cpu/u1_pc_next { sim:/sim_tb_top/u1_plasma_top/u1_plasma/u1_cpu/u1_pc_next/pc_current(19 downto 4)} PC_
quietly virtual signal -install sim:/sim_tb_top/u1_plasma_top { sim:/sim_tb_top/u1_plasma_top/SD_A(12 downto 1)} DDR_ADDR
quietly virtual signal -install sim:/sim_tb_top/u1_plasma_top/u1_plasma/opt_cache2/u_cache { sim:/sim_tb_top/u1_plasma_top/u1_plasma/opt_cache2/u_cache/cache_ram_address(13 downto 2)} CACHE_ADDR
quietly virtual signal -install sim:/sim_tb_top/u1_plasma_top/u1_plasma/opt_cache2/u_cache { sim:/sim_tb_top/u1_plasma_top/u1_plasma/opt_cache2/u_cache/cache_ram_address(14 downto 2)} CACHE_ADDR001
quietly virtual signal -install sim:/sim_tb_top/u1_plasma_top/u1_plasma/opt_cache2/u_cache { sim:/sim_tb_top/u1_plasma_top/u1_plasma/opt_cache2/u_cache/cache_ram_address(10 downto 2)} CACHE_ADDR002
add wave  sim:/sim_tb_top/u1_plasma_top/u1_plasma/u1_cpu/u1_pc_next/PC_
add wave  sim:/sim_tb_top/u1_plasma_top/u1_plasma/u1_cpu/u2_mem_ctrl/opcode_out
add wave  sim:/sim_tb_top/u1_plasma_top/u1_plasma/u1_cpu/u1_pc_next/pc_current
add wave  sim:/sim_tb_top/u1_plasma_top/DDR_ADDR
add wave  sim:/sim_tb_top/u1_plasma_top/SD_A
add wave  sim:/sim_tb_top/u1_plasma_top/SD_RAS
add wave  sim:/sim_tb_top/u1_plasma_top/SD_CAS
add wave  sim:/sim_tb_top/u1_plasma_top/SD_WE
add wave  sim:/sim_tb_top/u1_plasma_top/SD_DQ
add wave  sim:/sim_tb_top/u1_plasma_top/SD_UDM
add wave  sim:/sim_tb_top/u1_plasma_top/SD_UDQS
add wave  sim:/sim_tb_top/u1_plasma_top/SD_LDM
add wave  sim:/sim_tb_top/u1_plasma_top/SD_LDQS
add wave  sim:/sim_tb_top/u1_plasma_top/u1_plasma/opt_cache2/u_cache/cache_ram_enable
add wave  sim:/sim_tb_top/u1_plasma_top/u1_plasma/opt_cache2/u_cache/cache_ram_byte_we
add wave  sim:/sim_tb_top/u1_plasma_top/u1_plasma/opt_cache2/u_cache/CACHE_ADDR002
add wave  sim:/sim_tb_top/u1_plasma_top/u1_plasma/opt_cache2/u_cache/cache_ram_address
add wave  sim:/sim_tb_top/u1_plasma_top/u1_plasma/opt_cache2/u_cache/cache_ram_data_w
add wave  sim:/sim_tb_top/u1_plasma_top/u1_plasma/opt_cache2/u_cache/cache_ram_data_r

#Change radix to Hexadecimal#
radix hex
#Supress Numeric Std package and Arith package warnings.#
#For VHDL designs we get some warnings due to unknown values on some signals at startup#
# ** Warning: NUMERIC_STD.TO_INTEGER: metavalue detected, returning 0#
#We may also get some Arithmetic packeage warnings because of unknown values on#
#some of the signals that are used in an Arithmetic operation.#
#In order to suppress these warnings, we use following two commands#
set NumericStdNoWarnings 1
set StdArithNoWarnings 1

run 1ms
stop
