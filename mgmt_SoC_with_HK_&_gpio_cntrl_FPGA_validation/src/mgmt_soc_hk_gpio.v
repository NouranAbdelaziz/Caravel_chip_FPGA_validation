`default_nettype none

`ifndef __USER_DEFINES_H
// User GPIO initial configuration parameters
`define __USER_DEFINES_H

// Useful GPIO mode values.  These match the names used in defs.h.
`define GPIO_MODE_MGMT_STD_INPUT_NOPULL    13'h0403
`define GPIO_MODE_MGMT_STD_INPUT_PULLDOWN  13'h0803
`define GPIO_MODE_MGMT_STD_INPUT_PULLUP    13'h0c03
`define GPIO_MODE_MGMT_STD_OUTPUT          13'h1809
`define GPIO_MODE_MGMT_STD_BIDIRECTIONAL   13'h1801
`define GPIO_MODE_MGMT_STD_ANALOG          13'h000b

`define GPIO_MODE_USER_STD_INPUT_NOPULL    13'h0402
`define GPIO_MODE_USER_STD_INPUT_PULLDOWN  13'h0802
`define GPIO_MODE_USER_STD_INPUT_PULLUP    13'h0c02
`define GPIO_MODE_USER_STD_OUTPUT          13'h1808
`define GPIO_MODE_USER_STD_BIDIRECTIONAL   13'h1800
`define GPIO_MODE_USER_STD_OUT_MONITORED   13'h1802
`define GPIO_MODE_USER_STD_ANALOG          13'h000a

// The power-on configuration for GPIO 0 to 4 is fixed and cannot be
// modified (allowing the SPI and debug to always be accessible unless
// overridden by a flash program).

// The values below can be any of the standard types defined above,
// or they can be any 13-bit value if the user wants a non-standard
// startup state for the GPIO.  By default, every GPIO from 5 to 37
// is set to power up as an input controlled by the management SoC.
// Users may want to redefine these so that the user project powers
// up in a state that can be used immediately without depending on
// the management SoC to run a startup program to configure the GPIOs.

`define USER_CONFIG_GPIO_5_INIT  `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_6_INIT  `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_7_INIT  `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_8_INIT  `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_9_INIT  `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_10_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_11_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_12_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_13_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_14_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL

// Configurations of GPIO 15 to 25 are used on caravel but not caravan.
`define USER_CONFIG_GPIO_15_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_16_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_17_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_18_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_19_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_20_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_21_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_22_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_23_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_24_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_25_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL

`define USER_CONFIG_GPIO_26_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_27_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_28_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_29_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_30_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_31_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_32_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_33_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_34_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_35_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_36_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL
`define USER_CONFIG_GPIO_37_INIT `GPIO_MODE_MGMT_STD_INPUT_NOPULL

`endif // __USER_DEFINES_H




`ifndef __GLOBAL_DEFINE_H
// Global parameters
`define __GLOBAL_DEFINE_H

`define MPRJ_IO_PADS_1 19	/* number of user GPIO pads on user1 side */
`define MPRJ_IO_PADS_2 19	/* number of user GPIO pads on user2 side */
`define MPRJ_IO_PADS (`MPRJ_IO_PADS_1 + `MPRJ_IO_PADS_2)

`define MPRJ_PWR_PADS_1 2	/* vdda1, vccd1 enable/disable control */
`define MPRJ_PWR_PADS_2 2	/* vdda2, vccd2 enable/disable control */
`define MPRJ_PWR_PADS (`MPRJ_PWR_PADS_1 + `MPRJ_PWR_PADS_2)

// Analog pads are only used by the "caravan" module and associated
// modules such as user_analog_project_wrapper and chip_io_alt.

`define ANALOG_PADS_1 5
`define ANALOG_PADS_2 6

`define ANALOG_PADS (`ANALOG_PADS_1 + `ANALOG_PADS_2)

// Size of soc_mem_synth

// Type and size of soc_mem
// `define USE_OPENRAM
`define USE_CUSTOM_DFFRAM
// don't change the following without double checking addr widths
`define MEM_WORDS 256

// Number of columns in the custom memory; takes one of three values:
// 1 column : 1 KB, 2 column: 2 KB, 4 column: 4KB
`define DFFRAM_WSIZE 4
`define DFFRAM_USE_LATCH 0

// not really parameterized but just to easily keep track of the number
// of ram_block across different modules
`define RAM_BLOCKS 1

// Clock divisor default value
`define CLK_DIV 3'b010

// GPIO control default mode and enable for most I/Os
// Most I/Os set to be user input pins on startup.
// NOTE:  To be modified, with GPIOs 5 to 35 being set from a build-time-
// programmable block.
`define MGMT_INIT 1'b0
`define OENB_INIT 1'b0
`define DM_INIT 3'b001

`endif // __GLOBAL_DEFINE_H

/*
 *  SPDX-FileCopyrightText: 2015 Clifford Wolf
 *  PicoSoC - A simple example SoC using PicoRV32
 *
 *  Copyright (C) 2017  Clifford Wolf <clifford@clifford.at>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *  Revision 1,  July 2019:  Added signals to drive flash_clk and flash_csb
 *  output enable (inverted), tied to reset so that the flash is completely
 *  isolated from the processor when the processor is in reset.
 *
 *  Also: Made ram_wenb a 4-bit bus so that the memory access can be made
 *  byte-wide for byte-wide instructions.
 *
 *  SPDX-License-Identifier: ISC
 */


`default_nettype none
//`define USE_POWER_PINS

/* Wrapper module around management SoC core for pin compatibility	*/
/* with the Caravel harness chip. */	

module mgmt_soc_hk_gpio (
`ifdef USE_POWER_PINS
    inout wire VPWR,	    /* 1.8V domain */
    inout wire VGND,
`endif
    // Clock and reset
    input wire clk,
    input wire RST,

    // GPIO (one pin)
    inout wire gpio,	// Connect to gpio pad

	//inout wire [`MPRJ_IO_PADS-1:0] mprj_io,

	inout wire [34:0] mprj_io,

    // Flash memory control (SPI master)
    output wire flash_csb,
    output wire flash_clk,
    inout wire flash_io0,
    inout wire flash_io1,

	output wire csb,
    output wire sck,
    output wire sdi
  
);
	// housekeeping spi loopback 

    assign sck = mprj_io [4];
    assign csb = mprj_io [3];
    assign sdi = mprj_io [2];

    wire gpio_out_pad;	// Connect to out on gpio pad
    wire  gpio_in_pad;		// Connect to in on gpio pad
    wire gpio_mode0_pad;	// Connect to dm[0] on gpio pad
    wire gpio_mode1_pad;	// Connect to dm[2] on gpio pad
    wire gpio_outenb_pad;	// Connect to oe_n on gpio pad
    wire gpio_inenb_pad;	// Connect to inp_dis on gpio pad

    wire flash_io0_do;
    wire flash_io1_do;
    wire flash_io2_do;
    wire flash_io3_do;

    wire flash_io0_di;
    wire flash_io1_di;
    wire flash_io2_di;
    wire flash_io3_di;

    wire flash_io0_oeb;
    wire flash_io1_oeb;
    wire flash_io2_oeb;
    wire flash_io3_oeb;

    // Exported Wishbone Bus (processor facing)
    wire mprj_iena_wb;
    wire mprj_cyc_o_core;
    wire mprj_stb_o_core;
    wire mprj_we_o_core;
    wire [3:0] mprj_sel_o_core;
    wire [31:0] mprj_adr_o_core;
    wire [31:0] mprj_dat_o_core;
    wire mprj_ack_i_core;
    wire [31:0] mprj_dat_i_core;

    // Flash SPI communication (management SoC to housekeeping)
    wire flash_clk_core,     flash_csb_core;
    wire flash_clk_oeb_core, flash_csb_oeb_core;
    wire flash_clk_ieb_core, flash_csb_ieb_core;
    wire flash_io0_oeb_core, flash_io1_oeb_core;
    wire flash_io2_oeb_core, flash_io3_oeb_core;
    wire flash_io0_ieb_core, flash_io1_ieb_core;
    wire flash_io2_ieb_core, flash_io3_ieb_core;
    wire flash_io0_do_core,  flash_io1_do_core;
    wire flash_io2_do_core,  flash_io3_do_core;
    wire flash_io0_di_core,  flash_io1_di_core;
    wire flash_io2_di_core,  flash_io3_di_core;

	// Flash SPI communication (
    wire flash_clk_frame;
    wire flash_csb_frame;
    wire flash_clk_oeb, flash_csb_oeb;
    wire flash_clk_ieb, flash_csb_ieb;
    wire flash_io0_oeb, flash_io1_oeb;
    wire flash_io0_ieb, flash_io1_ieb;
    wire flash_io0_do,  flash_io1_do;
    wire flash_io0_di,  flash_io1_di;

    // Module status
    wire qspi_enabled;
    wire uart_enabled;
    wire spi_enabled;
    wire debug_mode;

    // Module I/O
    wire ser_tx;
    wire ser_rx;
    wire spi_csb;
    wire spi_sck;
    wire spi_sdo;
    wire spi_sdoenb;
    wire spi_sdi;
    

    // Trap state from CPU
    wire trap;

    wire debug_in;
    wire debug_out;
    wire debug_oeb;

    wire [31:0] hk_dat_i;
    wire hk_ack_i;
    wire hk_stb_o;
    wire hk_cyc_o;

    // SRAM read-only access from houskeeping
    wire 	hkspi_sram_clk;
    wire 	hkspi_sram_csb;
    wire [7:0]	hkspi_sram_addr;
    wire [31:0]	hkspi_sram_data;

    // One-bit GPIO dedicated to management SoC (outside of user control)
    wire gpio_out_core;
    wire gpio_in_core;
    wire gpio_mode0_core;
    wire gpio_mode1_core;
    wire gpio_outenb_core;
    wire gpio_inenb_core;

    // User Project Control (pad-facing)
    wire [`MPRJ_IO_PADS-1:0] mprj_io_inp_dis;
    wire [`MPRJ_IO_PADS-1:0] mprj_io_oeb;
    wire [`MPRJ_IO_PADS-1:0] mprj_io_ib_mode_sel;
    wire [`MPRJ_IO_PADS-1:0] mprj_io_vtrip_sel;
    wire [`MPRJ_IO_PADS-1:0] mprj_io_slow_sel;
    wire [`MPRJ_IO_PADS-1:0] mprj_io_holdover;
    wire [`MPRJ_IO_PADS-1:0] mprj_io_analog_en;
    wire [`MPRJ_IO_PADS-1:0] mprj_io_analog_sel;
    wire [`MPRJ_IO_PADS-1:0] mprj_io_analog_pol;
    wire [`MPRJ_IO_PADS*3-1:0] mprj_io_dm;
    wire [`MPRJ_IO_PADS-1:0] mprj_io_in;
    wire [`MPRJ_IO_PADS-1:0] mprj_io_out;

    // User Project Control (user-facing)
    wire [`MPRJ_IO_PADS-1:0] user_io_oeb;
    wire [`MPRJ_IO_PADS-1:0] user_io_in;
    wire [`MPRJ_IO_PADS-1:0] user_io_out;
    wire [`MPRJ_IO_PADS-10:0] user_analog_io;

    /* Padframe control signals */
    wire [`MPRJ_IO_PADS_1-1:0] gpio_serial_link_1;
    wire [`MPRJ_IO_PADS_2-1:0] gpio_serial_link_2;
    wire mprj_io_loader_resetn;
    wire mprj_io_loader_clock;
    wire mprj_io_loader_strobe;
    wire mprj_io_loader_data_1;		/* user1 side serial loader */
    wire mprj_io_loader_data_2;		/* user2 side serial loader */

    // User Project Control management I/O
    // There are two types of GPIO connections:
    // (1) Full Bidirectional: Management connects to in, out, and oeb
    //     Uses:  JTAG and SDO
    // (2) Selectable bidirectional:  Management connects to in and out,
    //	   which are tied together.  oeb is grounded (oeb from the
    //	   configuration is used)

    // SDI 	 = mprj_io[2]		(input)
    // CSB 	 = mprj_io[3]		(input)
    // SCK	 = mprj_io[4]		(input)
    // ser_rx    = mprj_io[5]		(input)
    // ser_tx    = mprj_io[6]		(output)
    // irq 	 = mprj_io[7]		(input)

    wire [`MPRJ_IO_PADS-1:0] mgmt_io_in;	/* one- and three-pin data */
    wire [`MPRJ_IO_PADS-5:0] mgmt_io_nc;	/* no-connects */
    wire [4:0] mgmt_io_out;			/* three-pin interface out */
    wire [4:0] mgmt_io_oeb;			/* three-pin output enable */
    wire [`MPRJ_PWR_PADS-1:0] pwr_ctrl_nc;	/* no-connects */

	assign user_io_oeb = mgmt_io_oeb;
	

    wire rstn;
    reset_sync rst_sync (.clk(clk), .rst_n(~RST), .rst_sync_n(rstn));

mgmt_core_wrapper soc (
	`ifdef USE_POWER_PINS	
		.VPWR		  (vccd_core),
		.VGND		  (vssd_core),
	`endif
		.core_clk(clk),
    	.core_rstn(rstn),

    	.gpio_out_pad(gpio_out_pad),		
    	.gpio_in_pad(gpio_in_pad),		
    	.gpio_mode0_pad(gpio_mode0_pad),	
    	.gpio_mode1_pad(gpio_mode1_pad),	
    	.gpio_outenb_pad(gpio_outenb_pad),	
    	.gpio_inenb_pad(gpio_inenb_pad),

		.la_input(),			
        .la_output(),			
        .la_oenb(),			
        .la_iena(),

		// Primary SPI flash controller
        .flash_csb(flash_csb_core),
        .flash_clk(flash_clk_core),
        .flash_io0_oeb(flash_io0_oeb_core),
        .flash_io0_di(flash_io0_di_core),
        .flash_io0_do(flash_io0_do_core),
        .flash_io1_oeb(flash_io1_oeb_core),
        .flash_io1_di(flash_io1_di_core),
        .flash_io1_do(flash_io1_do_core),
        .flash_io2_oeb(flash_io2_oeb_core),
        .flash_io2_di(flash_io2_di_core),
        .flash_io2_do(flash_io2_do_core),
        .flash_io3_oeb(flash_io3_oeb_core),
        .flash_io3_di(flash_io3_di_core),
        .flash_io3_do(flash_io3_do_core),

        // Exported Wishbone Bus
        .mprj_wb_iena(mprj_iena_wb),
        .mprj_cyc_o(mprj_cyc_o_core),
        .mprj_stb_o(mprj_stb_o_core),
        .mprj_we_o(mprj_we_o_core),
        .mprj_sel_o(mprj_sel_o_core),
        .mprj_adr_o(mprj_adr_o_core),
        .mprj_dat_o(mprj_dat_o_core),
        .mprj_ack_i(mprj_ack_i_core),
        .mprj_dat_i(mprj_dat_i_core),

        .hk_stb_o(hk_stb_o),
        .hk_cyc_o(hk_cyc_o),
        .hk_dat_i(hk_dat_i),
        .hk_ack_i(hk_ack_i),

        .irq(6'b0),
        .user_irq_ena(),

        .qspi_enabled(qspi_enabled),
        .uart_enabled(uart_enabled),
        .spi_enabled(spi_enabled),
        .debug_mode(debug_mode),

		.ser_tx(ser_tx),
    	.ser_rx(ser_rx),
    	.spi_csb(spi_csb),
    	.spi_sck(spi_sck),
    	.spi_sdo(spi_sdo),
    	.spi_sdoenb(spi_sdoenb),
    	.spi_sdi(spi_sdi),
        .debug_in(debug_in),
    	.debug_out(debug_out),
    	.debug_oeb(debug_oeb),

        // SRAM Read-only access from housekeeping
        .sram_ro_clk(hkspi_sram_clk),
        .sram_ro_csb(hkspi_sram_csb),
        .sram_ro_addr(hkspi_sram_addr),
        .sram_ro_data(hkspi_sram_data),

        .trap(trap)        
	);

    housekeeping housekeeping (
    `ifdef USE_POWER_PINS
		.VPWR(vccd_core),
		.VGND(vssd_core),
    `endif

        .wb_clk_i(clk),
        .wb_rstn_i(rstn),

        .wb_adr_i(mprj_adr_o_core),
        .wb_dat_i(mprj_dat_o_core),
        .wb_sel_i(mprj_sel_o_core),
        .wb_we_i(mprj_we_o_core),
        .wb_cyc_i(hk_cyc_o),
        .wb_stb_i(hk_stb_o),
        .wb_ack_o(hk_ack_i),
        .wb_dat_o(hk_dat_i),

        .porb(rstn),

        .pll_ena(),
        .pll_dco_ena(),
        .pll_div(),
        .pll_sel(),
        .pll90_sel(),
        .pll_trim(),
        .pll_bypass(),

	.qspi_enabled(qspi_enabled),
	.uart_enabled(uart_enabled),
	.spi_enabled(spi_enabled),
	.debug_mode(debug_mode),

	.ser_tx(ser_tx),
	.ser_rx(ser_rx),

	.spi_sdi(spi_sdi),
	.spi_csb(spi_csb),
	.spi_sck(spi_sck),
	.spi_sdo(spi_sdo),
	.spi_sdoenb(spi_sdoenb),

	.debug_in(debug_in),
	.debug_out(debug_out),
	.debug_oeb(debug_oeb),

        .irq(),
        .reset(),
    
        .serial_clock(mprj_io_loader_clock),
        .serial_load(mprj_io_loader_strobe),
        .serial_resetn(mprj_io_loader_resetn),
        .serial_data_1(mprj_io_loader_data_1),
        .serial_data_2(mprj_io_loader_data_2),
	.mgmt_gpio_in(mgmt_io_in),
	.mgmt_gpio_out({mgmt_io_out[4:2], mgmt_io_in[`MPRJ_IO_PADS-4:2],
			mgmt_io_out[1:0]}),
	.mgmt_gpio_oeb({mgmt_io_oeb[4:2], mgmt_io_nc[`MPRJ_IO_PADS-6:0],
			mgmt_io_oeb[1:0]}), 

	.pwr_ctrl_out(),	/* Not used in this version*/ 

        .trap(trap),

	.user_clock(),

        .mask_rev_in(),

	.spimemio_flash_csb(flash_csb_core),
	.spimemio_flash_clk(flash_clk_core),
	.spimemio_flash_io0_oeb(flash_io0_oeb_core),
	.spimemio_flash_io1_oeb(flash_io1_oeb_core),
	.spimemio_flash_io2_oeb(flash_io2_oeb_core),
	.spimemio_flash_io3_oeb(flash_io3_oeb_core),
	.spimemio_flash_io0_do(flash_io0_do_core),
	.spimemio_flash_io1_do(flash_io1_do_core),
	.spimemio_flash_io2_do(flash_io2_do_core),
	.spimemio_flash_io3_do(flash_io3_do_core),
	.spimemio_flash_io0_di(flash_io0_di_core),
	.spimemio_flash_io1_di(flash_io1_di_core),
	.spimemio_flash_io2_di(flash_io2_di_core),
	.spimemio_flash_io3_di(flash_io3_di_core),

	.pad_flash_csb(flash_csb_frame),
	.pad_flash_csb_oeb(flash_csb_oeb),
	.pad_flash_clk(flash_clk_frame),
	.pad_flash_clk_oeb(flash_clk_oeb),
	.pad_flash_io0_oeb(flash_io0_oeb),
	.pad_flash_io1_oeb(flash_io1_oeb),
	.pad_flash_io0_ieb(flash_io0_ieb),
	.pad_flash_io1_ieb(flash_io1_ieb),
	.pad_flash_io0_do(flash_io0_do),
	.pad_flash_io1_do(flash_io1_do),
	.pad_flash_io0_di(flash_io0_di),
	.pad_flash_io1_di(flash_io1_di),

	.sram_ro_clk(hkspi_sram_clk),
	.sram_ro_csb(hkspi_sram_csb),
	.sram_ro_addr(hkspi_sram_addr),
	.sram_ro_data(hkspi_sram_data),

	.usr1_vcc_pwrgood(),
	.usr2_vcc_pwrgood(),
	.usr1_vdd_pwrgood(),
	.usr2_vdd_pwrgood()
    );

    wire [`MPRJ_IO_PADS_1-1:0] gpio_serial_link_1_shifted;
    wire [`MPRJ_IO_PADS_2-1:0] gpio_serial_link_2_shifted;

    assign gpio_serial_link_1_shifted = {gpio_serial_link_1[`MPRJ_IO_PADS_1-2:0],
					 mprj_io_loader_data_1};
    // Note that serial_link_2 is backwards compared to serial_link_1, so it
    // shifts in the other direction.
    assign gpio_serial_link_2_shifted = {mprj_io_loader_data_2,
					 gpio_serial_link_2[`MPRJ_IO_PADS_2-1:1]};

    // Propagating clock and reset to mitigate timing and fanout issues
    wire [`MPRJ_IO_PADS_1-1:0] gpio_clock_1;
    wire [`MPRJ_IO_PADS_2-1:0] gpio_clock_2;
    wire [`MPRJ_IO_PADS_1-1:0] gpio_resetn_1;
    wire [`MPRJ_IO_PADS_2-1:0] gpio_resetn_2;
    wire [`MPRJ_IO_PADS_1-1:0] gpio_load_1;
    wire [`MPRJ_IO_PADS_2-1:0] gpio_load_2;
    wire [`MPRJ_IO_PADS_1-1:0] gpio_clock_1_shifted;
    wire [`MPRJ_IO_PADS_2-1:0] gpio_clock_2_shifted;
    wire [`MPRJ_IO_PADS_1-1:0] gpio_resetn_1_shifted;
    wire [`MPRJ_IO_PADS_2-1:0] gpio_resetn_2_shifted;
    wire [`MPRJ_IO_PADS_1-1:0] gpio_load_1_shifted;
    wire [`MPRJ_IO_PADS_2-1:0] gpio_load_2_shifted;

    assign gpio_clock_1_shifted = {gpio_clock_1[`MPRJ_IO_PADS_1-2:0],
					 mprj_io_loader_clock};
    assign gpio_clock_2_shifted = {mprj_io_loader_clock,
					gpio_clock_2[`MPRJ_IO_PADS_2-1:1]};
    assign gpio_resetn_1_shifted = {gpio_resetn_1[`MPRJ_IO_PADS_1-2:0],
					 mprj_io_loader_resetn};
    assign gpio_resetn_2_shifted = {mprj_io_loader_resetn,
					gpio_resetn_2[`MPRJ_IO_PADS_2-1:1]};
    assign gpio_load_1_shifted = {gpio_load_1[`MPRJ_IO_PADS_1-2:0],
					 mprj_io_loader_strobe};
    assign gpio_load_2_shifted = {mprj_io_loader_strobe,
					gpio_load_2[`MPRJ_IO_PADS_2-1:1]};


    wire vddio;	// Common 3.3V padframe/ESD power
    wire vddio_2;	// Common 3.3V padframe/ESD power
    wire vssio;	// Common padframe/ESD ground
    wire vssio_2;	// Common padframe/ESD ground
    wire vdda;		// Management 3.3V power
    wire vssa;		// Common analog ground
    wire vccd;		// Management/Common 1.8V power
    wire vssd;		// Common digital ground
    wire vdda1;	// User area 1 3.3V power
    wire vdda1_2;	// User area 1 3.3V power
    wire vdda2;	// User area 2 3.3V power
    wire vssa1;	// User area 1 analog ground
    wire vssa1_2;	// User area 1 analog ground
    wire vssa2;	// User area 2 analog ground
    wire vccd1;	// User area 1 1.8V power
    wire vccd2;	// User area 2 1.8V power
    wire vssd1;	// User area 1 digital ground
    wire vssd2;	// User area 2 digital ground
	wire gpio;		// Used for external LDO control
    wire clock;    	// CMOS core clock input, not a crystal
    wire resetb;	// Reset input (sense inverted)

	wire vddio_core;
	wire vssio_core;
	wire vdda_core;
	wire vssa_core;
	wire vccd_core;
	wire vssd_core;
	wire vdda1_core;
	wire vdda2_core;
	wire vssa1_core;
	wire vssa2_core;
	wire vccd1_core;
	wire vccd2_core;
	wire vssd1_core;
	wire vssd2_core;

	wire flash_csb;
    wire flash_clk;
    wire flash_io0;
    wire flash_io1;

	// Power-on-reset signal.  The reset pad generates the sense-inverted
    // reset at 3.3V.  The 1.8V signal and the inverted 1.8V signal are
    // derived.

    wire porb_h;
    wire porb_l;
    wire por_l;

	wire clock_core;

	wire rstb_h;
    wire rstb_l;
	
	chip_io_FPGA padframe(
	`ifndef TOP_ROUTING
		// Package Pins
		.vddio_pad	(vddio),		// Common padframe/ESD supply
		.vddio_pad2	(vddio_2),
		.vssio_pad	(vssio),		// Common padframe/ESD ground
		.vssio_pad2	(vssio_2),
		.vccd_pad	(vccd),			// Common 1.8V supply
		.vssd_pad	(vssd),			// Common digital ground
		.vdda_pad	(vdda),			// Management analog 3.3V supply
		.vssa_pad	(vssa),			// Management analog ground
		.vdda1_pad	(vdda1),		// User area 1 3.3V supply
		.vdda1_pad2	(vdda1_2),		
		.vdda2_pad	(vdda2),		// User area 2 3.3V supply
		.vssa1_pad	(vssa1),		// User area 1 analog ground
		.vssa1_pad2	(vssa1_2),
		.vssa2_pad	(vssa2),		// User area 2 analog ground
		.vccd1_pad	(vccd1),		// User area 1 1.8V supply
		.vccd2_pad	(vccd2),		// User area 2 1.8V supply
		.vssd1_pad	(vssd1),		// User area 1 digital ground
		.vssd2_pad	(vssd2),		// User area 2 digital ground
	`endif
	// Core Side Pins
	.vddio	(vddio_core),
	.vssio	(vssio_core),
	.vdda	(vdda_core),
	.vssa	(vssa_core),
	.vccd	(vccd_core),
	.vssd	(vssd_core),
	.vdda1	(vdda1_core),
	.vdda2	(vdda2_core),
	.vssa1	(vssa1_core),
	.vssa2	(vssa2_core),
	.vccd1	(vccd1_core),
	.vccd2	(vccd2_core),
	.vssd1	(vssd1_core),
	.vssd2	(vssd2_core),

	.gpio(gpio),
	.mprj_io(mprj_io),
	.clock(clk),
	.resetb(RST),
	.flash_csb(flash_csb),
	.flash_clk(flash_clk),
	.flash_io0(flash_io0),
	.flash_io1(flash_io1),
	// SoC Core Interface
	.porb_h(porb_h),
	.por(por_l),
	.resetb_core_h(rstb_h),
	.clock_core(clock_core),
	.gpio_out_core(gpio_out_core),
	.gpio_in_core(gpio_in_core),
	.gpio_mode0_core(gpio_mode0_core),
	.gpio_mode1_core(gpio_mode1_core),
	.gpio_outenb_core(gpio_outenb_core),
	.gpio_inenb_core(gpio_inenb_core),
	.flash_csb_core(flash_csb_frame),
	.flash_clk_core(flash_clk_frame),
	.flash_csb_oeb_core(flash_csb_oeb),
	.flash_clk_oeb_core(flash_clk_oeb),
	.flash_io0_oeb_core(flash_io0_oeb),
	.flash_io1_oeb_core(flash_io1_oeb),
	.flash_csb_ieb_core(flash_csb_ieb),
	.flash_clk_ieb_core(flash_clk_ieb),
	.flash_io0_ieb_core(flash_io0_ieb),
	.flash_io1_ieb_core(flash_io1_ieb),
	.flash_io0_do_core(flash_io0_do),
	.flash_io1_do_core(flash_io1_do),
	.flash_io0_di_core(flash_io0_di),
	.flash_io1_di_core(flash_io1_di),
	.mprj_io_in(mprj_io_in),
	.mprj_io_out(mprj_io_out),
	.mprj_io_oeb(mprj_io_oeb),
	.mprj_io_inp_dis(mprj_io_inp_dis),
	.mprj_io_ib_mode_sel(mprj_io_ib_mode_sel),
	.mprj_io_vtrip_sel(mprj_io_vtrip_sel),
	.mprj_io_slow_sel(mprj_io_slow_sel),
	.mprj_io_holdover(mprj_io_holdover),
	.mprj_io_analog_en(mprj_io_analog_en),
	.mprj_io_analog_sel(mprj_io_analog_sel),
	.mprj_io_analog_pol(mprj_io_analog_pol),
	.mprj_io_dm(mprj_io_dm),
	.mprj_analog_io(user_analog_io)
    );

    /* GPIO defaults (via programmed) */
    wire [`MPRJ_IO_PADS*13-1:0] gpio_defaults;

    /* Fixed defaults for the first 5 GPIO pins */

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(13'h1803)
    ) gpio_defaults_block_0 [1:0] (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[25:0])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(13'h0403)
    ) gpio_defaults_block_2 [2:0] (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[64:26])
    );

    /* Via-programmable defaults for the rest of the GPIO pins */

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_5_INIT)
    ) gpio_defaults_block_5 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[77:65])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_6_INIT)
    ) gpio_defaults_block_6 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[90:78])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_7_INIT)
    ) gpio_defaults_block_7 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[103:91])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_8_INIT)
    ) gpio_defaults_block_8 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[116:104])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_9_INIT)
    ) gpio_defaults_block_9 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[129:117])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_10_INIT)
    ) gpio_defaults_block_10 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[142:130])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_11_INIT)
    ) gpio_defaults_block_11 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[155:143])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_12_INIT)
    ) gpio_defaults_block_12 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[168:156])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_13_INIT)
    ) gpio_defaults_block_13 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[181:169])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_14_INIT)
    ) gpio_defaults_block_14 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[194:182])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_15_INIT)
    ) gpio_defaults_block_15 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[207:195])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_16_INIT)
    ) gpio_defaults_block_16 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[220:208])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_17_INIT)
    ) gpio_defaults_block_17 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[233:221])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_18_INIT)
    ) gpio_defaults_block_18 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[246:234])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_19_INIT)
    ) gpio_defaults_block_19 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[259:247])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_20_INIT)
    ) gpio_defaults_block_20 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[272:260])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_21_INIT)
    ) gpio_defaults_block_21 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[285:273])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_22_INIT)
    ) gpio_defaults_block_22 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[298:286])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_23_INIT)
    ) gpio_defaults_block_23 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[311:299])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_24_INIT)
    ) gpio_defaults_block_24 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[324:312])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_25_INIT)
    ) gpio_defaults_block_25 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[337:325])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_26_INIT)
    ) gpio_defaults_block_26 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[350:338])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_27_INIT)
    ) gpio_defaults_block_27 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[363:351])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_28_INIT)
    ) gpio_defaults_block_28 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[376:364])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_29_INIT)
    ) gpio_defaults_block_29 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[389:377])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_30_INIT)
    ) gpio_defaults_block_30 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[402:390])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_31_INIT)
    ) gpio_defaults_block_31 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[415:403])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_32_INIT)
    ) gpio_defaults_block_32 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[428:416])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_33_INIT)
    ) gpio_defaults_block_33 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[441:429])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_34_INIT)
    ) gpio_defaults_block_34 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[454:442])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_35_INIT)
    ) gpio_defaults_block_35 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[467:455])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_36_INIT)
    ) gpio_defaults_block_36 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[480:468])
    );

    gpio_defaults_block #(
	.GPIO_CONFIG_INIT(`USER_CONFIG_GPIO_37_INIT)
    ) gpio_defaults_block_37 (
    	`ifdef USE_POWER_PINS
	    .VPWR(vccd_core),
	    .VGND(vssd_core),
        `endif
	.gpio_defaults(gpio_defaults[493:481])
    );

    // Each control block sits next to an I/O pad in the user area.
    // It gets input through a serial chain from the previous control
    // block and passes it to the next control block.  Due to the nature
    // of the shift register, bits are presented in reverse, as the first
    // bit in ends up as the last bit of the last I/O pad control block.

    // There are two types of block;  the first two and the last two
    // are configured to be full bidirectional under control of the
    // management Soc (JTAG and SDO for the first two;  flash_io2 and
    // flash_io3 for the last two).  The rest are configured to be default
    // (input).  Note that the first two and last two are the ones closest
    // to the management SoC on either side, which minimizes the wire length
    // of the extra signals those pads need.


    /* First two GPIOs (JTAG and SDO) */

    gpio_control_block gpio_control_bidir_1 [1:0] (
    	`ifdef USE_POWER_PINS
	    .vccd(vccd_core),
	    .vssd(vssd_core),
	    .vccd1(vccd1_core),
	    .vssd1(vssd1_core),
        `endif

	.gpio_defaults(gpio_defaults[25:0]),

    	// Management Soc-facing signals

    	.resetn(gpio_resetn_1_shifted[1:0]),
    	.serial_clock(gpio_clock_1_shifted[1:0]),
    	.serial_load(gpio_load_1_shifted[1:0]),

    	.resetn_out(gpio_resetn_1[1:0]),
    	.serial_clock_out(gpio_clock_1[1:0]),
    	.serial_load_out(gpio_load_1[1:0]),

    	.mgmt_gpio_in(mgmt_io_in[1:0]),
	.mgmt_gpio_out(mgmt_io_out[1:0]),
	.mgmt_gpio_oeb(mgmt_io_oeb[1:0]),

        .one(),
        .zero(),

    	// Serial data chain for pad configuration
    	.serial_data_in(gpio_serial_link_1_shifted[1:0]),
    	.serial_data_out(gpio_serial_link_1[1:0]),

    	// User-facing signals
    	.user_gpio_out(user_io_out[1:0]),
    	.user_gpio_oeb(user_io_oeb[1:0]),
    	.user_gpio_in(user_io_in[1:0]),

    	// Pad-facing signals (Pad GPIOv2)
    	.pad_gpio_inenb(mprj_io_inp_dis[1:0]),
    	.pad_gpio_ib_mode_sel(mprj_io_ib_mode_sel[1:0]),
    	.pad_gpio_vtrip_sel(mprj_io_vtrip_sel[1:0]),
    	.pad_gpio_slow_sel(mprj_io_slow_sel[1:0]),
    	.pad_gpio_holdover(mprj_io_holdover[1:0]),
    	.pad_gpio_ana_en(mprj_io_analog_en[1:0]),
    	.pad_gpio_ana_sel(mprj_io_analog_sel[1:0]),
    	.pad_gpio_ana_pol(mprj_io_analog_pol[1:0]),
    	.pad_gpio_dm(mprj_io_dm[5:0]),
    	.pad_gpio_outenb(mprj_io_oeb[1:0]),
    	.pad_gpio_out(mprj_io_out[1:0]),
    	.pad_gpio_in(mprj_io_in[1:0])
    );

    /* Section 1 GPIOs (GPIO 0 to 18) */
    wire [`MPRJ_IO_PADS_1-1:2] one_loop1;

    /* Section 1 GPIOs (GPIO 2 to 7) that start up under management control */

    gpio_control_block gpio_control_in_1a [5:0] (
        `ifdef USE_POWER_PINS
            .vccd(vccd_core),
	    .vssd(vssd_core),
	    .vccd1(vccd1_core),
	    .vssd1(vssd1_core),
        `endif

	.gpio_defaults(gpio_defaults[103:26]),

    	// Management Soc-facing signals

    	.resetn(gpio_resetn_1_shifted[7:2]),
    	.serial_clock(gpio_clock_1_shifted[7:2]),
    	.serial_load(gpio_load_1_shifted[7:2]),

    	.resetn_out(gpio_resetn_1[7:2]),
    	.serial_clock_out(gpio_clock_1[7:2]),
    	.serial_load_out(gpio_load_1[7:2]),

	.mgmt_gpio_in(mgmt_io_in[7:2]),
	.mgmt_gpio_out(mgmt_io_in[7:2]),
	.mgmt_gpio_oeb(one_loop1[7:2]),

        .one(one_loop1[7:2]),
        .zero(),

    	// Serial data chain for pad configuration
    	.serial_data_in(gpio_serial_link_1_shifted[7:2]),
    	.serial_data_out(gpio_serial_link_1[7:2]),

    	// User-facing signals
    	.user_gpio_out(user_io_out[7:2]),
    	.user_gpio_oeb(user_io_oeb[7:2]),
    	.user_gpio_in(user_io_in[7:2]),

    	// Pad-facing signals (Pad GPIOv2)
    	.pad_gpio_inenb(mprj_io_inp_dis[7:2]),
    	.pad_gpio_ib_mode_sel(mprj_io_ib_mode_sel[7:2]),
    	.pad_gpio_vtrip_sel(mprj_io_vtrip_sel[7:2]),
    	.pad_gpio_slow_sel(mprj_io_slow_sel[7:2]),
    	.pad_gpio_holdover(mprj_io_holdover[7:2]),
    	.pad_gpio_ana_en(mprj_io_analog_en[7:2]),
    	.pad_gpio_ana_sel(mprj_io_analog_sel[7:2]),
    	.pad_gpio_ana_pol(mprj_io_analog_pol[7:2]),
    	.pad_gpio_dm(mprj_io_dm[23:6]),
    	.pad_gpio_outenb(mprj_io_oeb[7:2]),
    	.pad_gpio_out(mprj_io_out[7:2]),
    	.pad_gpio_in(mprj_io_in[7:2])
    );

    /* Section 1 GPIOs (GPIO 8 to 18) */

    gpio_control_block gpio_control_in_1 [`MPRJ_IO_PADS_1-9:0] (
        `ifdef USE_POWER_PINS
            .vccd(vccd_core),
	    .vssd(vssd_core),
	    .vccd1(vccd1_core),
	    .vssd1(vssd1_core),
        `endif

	.gpio_defaults(gpio_defaults[(`MPRJ_IO_PADS_1*13-1):104]),

    	// Management Soc-facing signals

    	.resetn(gpio_resetn_1_shifted[(`MPRJ_IO_PADS_1-1):8]),
    	.serial_clock(gpio_clock_1_shifted[(`MPRJ_IO_PADS_1-1):8]),
    	.serial_load(gpio_load_1_shifted[(`MPRJ_IO_PADS_1-1):8]),

    	.resetn_out(gpio_resetn_1[(`MPRJ_IO_PADS_1-1):8]),
    	.serial_clock_out(gpio_clock_1[(`MPRJ_IO_PADS_1-1):8]),
    	.serial_load_out(gpio_load_1[(`MPRJ_IO_PADS_1-1):8]),

	.mgmt_gpio_in(mgmt_io_in[(`MPRJ_IO_PADS_1-1):8]),
	.mgmt_gpio_out(mgmt_io_in[(`MPRJ_IO_PADS_1-1):8]),
	.mgmt_gpio_oeb(one_loop1[(`MPRJ_IO_PADS_1-1):8]),

        .one(one_loop1[(`MPRJ_IO_PADS_1-1):8]),
        .zero(),

    	// Serial data chain for pad configuration
    	.serial_data_in(gpio_serial_link_1_shifted[(`MPRJ_IO_PADS_1-1):8]),
    	.serial_data_out(gpio_serial_link_1[(`MPRJ_IO_PADS_1-1):8]),

    	// User-facing signals
    	.user_gpio_out(user_io_out[(`MPRJ_IO_PADS_1-1):8]),
    	.user_gpio_oeb(user_io_oeb[(`MPRJ_IO_PADS_1-1):8]),
    	.user_gpio_in(user_io_in[(`MPRJ_IO_PADS_1-1):8]),

    	// Pad-facing signals (Pad GPIOv2)
    	.pad_gpio_inenb(mprj_io_inp_dis[(`MPRJ_IO_PADS_1-1):8]),
    	.pad_gpio_ib_mode_sel(mprj_io_ib_mode_sel[(`MPRJ_IO_PADS_1-1):8]),
    	.pad_gpio_vtrip_sel(mprj_io_vtrip_sel[(`MPRJ_IO_PADS_1-1):8]),
    	.pad_gpio_slow_sel(mprj_io_slow_sel[(`MPRJ_IO_PADS_1-1):8]),
    	.pad_gpio_holdover(mprj_io_holdover[(`MPRJ_IO_PADS_1-1):8]),
    	.pad_gpio_ana_en(mprj_io_analog_en[(`MPRJ_IO_PADS_1-1):8]),
    	.pad_gpio_ana_sel(mprj_io_analog_sel[(`MPRJ_IO_PADS_1-1):8]),
    	.pad_gpio_ana_pol(mprj_io_analog_pol[(`MPRJ_IO_PADS_1-1):8]),
    	.pad_gpio_dm(mprj_io_dm[(`MPRJ_IO_PADS_1*3-1):24]),
    	.pad_gpio_outenb(mprj_io_oeb[(`MPRJ_IO_PADS_1-1):8]),
    	.pad_gpio_out(mprj_io_out[(`MPRJ_IO_PADS_1-1):8]),
    	.pad_gpio_in(mprj_io_in[(`MPRJ_IO_PADS_1-1):8])
    );

    /* Last three GPIOs (spi_sdo, flash_io2, and flash_io3) */

    gpio_control_block gpio_control_bidir_2 [2:0] (
    	`ifdef USE_POWER_PINS
	    .vccd(vccd_core),
	    .vssd(vssd_core),
	    .vccd1(vccd1_core),
	    .vssd1(vssd1_core),
        `endif

	.gpio_defaults(gpio_defaults[(`MPRJ_IO_PADS*13-1):(`MPRJ_IO_PADS*13-39)]),

    	// Management Soc-facing signals

    	.resetn(gpio_resetn_2_shifted[(`MPRJ_IO_PADS_2-1):(`MPRJ_IO_PADS_2-3)]),
    	.serial_clock(gpio_clock_2_shifted[(`MPRJ_IO_PADS_2-1):(`MPRJ_IO_PADS_2-3)]),
    	.serial_load(gpio_load_2_shifted[(`MPRJ_IO_PADS_2-1):(`MPRJ_IO_PADS_2-3)]),

    	.resetn_out(gpio_resetn_2[(`MPRJ_IO_PADS_2-1):(`MPRJ_IO_PADS_2-3)]),
    	.serial_clock_out(gpio_clock_2[(`MPRJ_IO_PADS_2-1):(`MPRJ_IO_PADS_2-3)]),
    	.serial_load_out(gpio_load_2[(`MPRJ_IO_PADS_2-1):(`MPRJ_IO_PADS_2-3)]),

    	.mgmt_gpio_in(mgmt_io_in[(`MPRJ_IO_PADS-1):(`MPRJ_IO_PADS-3)]),
	.mgmt_gpio_out(mgmt_io_out[4:2]),
	.mgmt_gpio_oeb(mgmt_io_oeb[4:2]),

        .one(),
        .zero(),

    	// Serial data chain for pad configuration
    	.serial_data_in(gpio_serial_link_2_shifted[(`MPRJ_IO_PADS_2-1):(`MPRJ_IO_PADS_2-3)]),
    	.serial_data_out(gpio_serial_link_2[(`MPRJ_IO_PADS_2-1):(`MPRJ_IO_PADS_2-3)]),

    	// User-facing signals
    	.user_gpio_out(user_io_out[(`MPRJ_IO_PADS-1):(`MPRJ_IO_PADS-3)]),
    	.user_gpio_oeb(user_io_oeb[(`MPRJ_IO_PADS-1):(`MPRJ_IO_PADS-3)]),
    	.user_gpio_in(user_io_in[(`MPRJ_IO_PADS-1):(`MPRJ_IO_PADS-3)]),

    	// Pad-facing signals (Pad GPIOv2)
    	.pad_gpio_inenb(mprj_io_inp_dis[(`MPRJ_IO_PADS-1):(`MPRJ_IO_PADS-3)]),
    	.pad_gpio_ib_mode_sel(mprj_io_ib_mode_sel[(`MPRJ_IO_PADS-1):(`MPRJ_IO_PADS-3)]),
    	.pad_gpio_vtrip_sel(mprj_io_vtrip_sel[(`MPRJ_IO_PADS-1):(`MPRJ_IO_PADS-3)]),
    	.pad_gpio_slow_sel(mprj_io_slow_sel[(`MPRJ_IO_PADS-1):(`MPRJ_IO_PADS-3)]),
    	.pad_gpio_holdover(mprj_io_holdover[(`MPRJ_IO_PADS-1):(`MPRJ_IO_PADS-3)]),
    	.pad_gpio_ana_en(mprj_io_analog_en[(`MPRJ_IO_PADS-1):(`MPRJ_IO_PADS-3)]),
    	.pad_gpio_ana_sel(mprj_io_analog_sel[(`MPRJ_IO_PADS-1):(`MPRJ_IO_PADS-3)]),
    	.pad_gpio_ana_pol(mprj_io_analog_pol[(`MPRJ_IO_PADS-1):(`MPRJ_IO_PADS-3)]),
    	.pad_gpio_dm(mprj_io_dm[(`MPRJ_IO_PADS*3-1):(`MPRJ_IO_PADS*3-9)]),
    	.pad_gpio_outenb(mprj_io_oeb[(`MPRJ_IO_PADS-1):(`MPRJ_IO_PADS-3)]),
    	.pad_gpio_out(mprj_io_out[(`MPRJ_IO_PADS-1):(`MPRJ_IO_PADS-3)]),
    	.pad_gpio_in(mprj_io_in[(`MPRJ_IO_PADS-1):(`MPRJ_IO_PADS-3)])
    );

    /* Section 2 GPIOs (GPIO 19 to 34) */
    wire [`MPRJ_IO_PADS_2-4:0] one_loop2;

    gpio_control_block gpio_control_in_2 [`MPRJ_IO_PADS_2-4:0] (
    	`ifdef USE_POWER_PINS
            .vccd(vccd_core),
	    .vssd(vssd_core),
	    .vccd1(vccd1_core),
	    .vssd1(vssd1_core),
        `endif

	.gpio_defaults(gpio_defaults[(`MPRJ_IO_PADS*13-40):(`MPRJ_IO_PADS_1*13)]),

    	// Management Soc-facing signals

    	.resetn(gpio_resetn_2_shifted[(`MPRJ_IO_PADS_2-4):0]),
    	.serial_clock(gpio_clock_2_shifted[(`MPRJ_IO_PADS_2-4):0]),
    	.serial_load(gpio_load_2_shifted[(`MPRJ_IO_PADS_2-4):0]),

    	.resetn_out(gpio_resetn_2[(`MPRJ_IO_PADS_2-4):0]),
    	.serial_clock_out(gpio_clock_2[(`MPRJ_IO_PADS_2-4):0]),
    	.serial_load_out(gpio_load_2[(`MPRJ_IO_PADS_2-4):0]),

	.mgmt_gpio_in(mgmt_io_in[(`MPRJ_IO_PADS-4):(`MPRJ_IO_PADS_1)]),
	.mgmt_gpio_out(mgmt_io_in[(`MPRJ_IO_PADS-4):(`MPRJ_IO_PADS_1)]),
	.mgmt_gpio_oeb(one_loop2),

        .one(one_loop2),
        .zero(),

    	// Serial data chain for pad configuration
    	.serial_data_in(gpio_serial_link_2_shifted[(`MPRJ_IO_PADS_2-4):0]),
    	.serial_data_out(gpio_serial_link_2[(`MPRJ_IO_PADS_2-4):0]),

    	// User-facing signals
    	.user_gpio_out(user_io_out[(`MPRJ_IO_PADS-4):(`MPRJ_IO_PADS_1)]),
    	.user_gpio_oeb(user_io_oeb[(`MPRJ_IO_PADS-4):(`MPRJ_IO_PADS_1)]),
    	.user_gpio_in(user_io_in[(`MPRJ_IO_PADS-4):(`MPRJ_IO_PADS_1)]),

    	// Pad-facing signals (Pad GPIOv2)
    	.pad_gpio_inenb(mprj_io_inp_dis[(`MPRJ_IO_PADS-4):(`MPRJ_IO_PADS_1)]),
    	.pad_gpio_ib_mode_sel(mprj_io_ib_mode_sel[(`MPRJ_IO_PADS-4):(`MPRJ_IO_PADS_1)]),
    	.pad_gpio_vtrip_sel(mprj_io_vtrip_sel[(`MPRJ_IO_PADS-4):(`MPRJ_IO_PADS_1)]),
    	.pad_gpio_slow_sel(mprj_io_slow_sel[(`MPRJ_IO_PADS-4):(`MPRJ_IO_PADS_1)]),
    	.pad_gpio_holdover(mprj_io_holdover[(`MPRJ_IO_PADS-4):(`MPRJ_IO_PADS_1)]),
    	.pad_gpio_ana_en(mprj_io_analog_en[(`MPRJ_IO_PADS-4):(`MPRJ_IO_PADS_1)]),
    	.pad_gpio_ana_sel(mprj_io_analog_sel[(`MPRJ_IO_PADS-4):(`MPRJ_IO_PADS_1)]),
    	.pad_gpio_ana_pol(mprj_io_analog_pol[(`MPRJ_IO_PADS-4):(`MPRJ_IO_PADS_1)]),
    	.pad_gpio_dm(mprj_io_dm[(`MPRJ_IO_PADS*3-10):(`MPRJ_IO_PADS_1*3)]),
    	.pad_gpio_outenb(mprj_io_oeb[(`MPRJ_IO_PADS-4):(`MPRJ_IO_PADS_1)]),
    	.pad_gpio_out(mprj_io_out[(`MPRJ_IO_PADS-4):(`MPRJ_IO_PADS_1)]),
    	.pad_gpio_in(mprj_io_in[(`MPRJ_IO_PADS-4):(`MPRJ_IO_PADS_1)])
    );


endmodule

