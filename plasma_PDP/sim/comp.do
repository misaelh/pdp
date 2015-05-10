
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
