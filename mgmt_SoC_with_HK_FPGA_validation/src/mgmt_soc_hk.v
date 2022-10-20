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

/* Wrapper module around management SoC core for pin compatibility	*/
/* with the Caravel harness chip. */	

module mgmt_soc_hk (
`ifdef USE_POWER_PINS
    inout VPWR,	    /* 1.8V domain */
    inout VGND,
`endif
    // Clock and reset
    input wire clk,
    input wire RST,

    // GPIO (one pin)
    inout wire gpio_inout_pad,	// Connect to gpio pad

    input wire [4:0] mgmt_io_in,
    output wire [1:0] mgmt_io_out,

    output wire csb,
    output wire sck,
    output wire sdi,

    // Flash memory control (SPI master)
    output wire flash_csb,
    output wire flash_clk,

    inout wire flash_io0_dio,
    inout wire flash_io1_dio,
    inout wire flash_io2_dio,
    inout wire flash_io3_dio
  
);
    // housekeeping spi loopback 

    assign sck = mgmt_io_in [4];
    assign csb = mgmt_io_in [3];
    assign sdi = mgmt_io_in [2];


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

    wire rstn;
    reset_sync rst_sync (.clk(clk), .rst_n(~RST), .rst_sync_n(rstn));
    io_buf #(
        .WIDTH(1)
    ) gpio_bidir (
        .dio_buf(gpio_inout_pad),
        .din_i(gpio_out_pad),
        .dout_o(gpio_in_pad),
        .in_not_out_i(gpio_inenb_pad)
    );

    io_buf #(
        .WIDTH(1)
    ) flash_bidir_0 (
        .dio_buf(flash_io0_dio),
        .din_i(flash_io0_do),
        .dout_o(flash_io0_di),
        .in_not_out_i(flash_io0_oeb)
    );

    io_buf #(
        .WIDTH(1)
    ) flash_bidir_1 (
        .dio_buf(flash_io1_dio),
        .din_i(flash_io1_do),
        .dout_o(flash_io1_di),
        .in_not_out_i(flash_io1_oeb)
    );

    io_buf #(
        .WIDTH(1)
    ) flash_bidir_2 (
        .dio_buf(flash_io2_dio),
        .din_i(flash_io2_do),
        .dout_o(flash_io2_di),
        .in_not_out_i(flash_io2_oeb)
    );

    io_buf #(
        .WIDTH(1)
    ) flash_bidir_3 (
        .dio_buf(flash_io3_dio),
        .din_i(flash_io3_do),
        .dout_o(flash_io3_di),
        .in_not_out_i(flash_io3_oeb)
    );

mgmt_core_wrapper soc (
	`ifdef USE_POWER_PINS
		.VPWR		  (VDD1V8),
		.VGND		  (VSS),
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
    .mgmt_gpio_in(mgmt_io_in),
	.mgmt_gpio_out(mgmt_io_out[1:0]),
    /*
        .serial_clock(mprj_io_loader_clock),
        .serial_load(mprj_io_loader_strobe),
        .serial_resetn(mprj_io_loader_resetn),
        .serial_data_1(mprj_io_loader_data_1),
        .serial_data_2(mprj_io_loader_data_2),

	.mgmt_gpio_in(mgmt_io_in),
	.mgmt_gpio_out({mgmt_io_out[4:2], mgmt_io_in[`MPRJ_IO_PADS-4:2],
			mgmt_io_out[1:0]}),
	.mgmt_gpio_oeb({mgmt_io_oeb[4:2], mgmt_io_nc[`MPRJ_IO_PADS-6:0],
			mgmt_io_oeb[1:0]}), */

	.pwr_ctrl_out(),	/* Not used in this version */

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

	.pad_flash_csb(flash_csb),
	.pad_flash_csb_oeb(),
	.pad_flash_clk(flash_clk),
	.pad_flash_clk_oeb(),
	.pad_flash_io0_oeb(flash_io0_oeb),
	.pad_flash_io1_oeb(flash_io1_oeb),
	.pad_flash_io0_ieb(),
	.pad_flash_io1_ieb(),
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



endmodule
`default_nettype wire

module io_buf #(parameter WIDTH=1) (
    inout  wire [WIDTH-1:0] dio_buf,
    input  wire [WIDTH-1:0] din_i,
    output wire [WIDTH-1:0] dout_o,
    input  wire [WIDTH-1:0] in_not_out_i         // high: input, low: output
);

//`ifdef USE_IO_BUF_BEH
    assign dout_o  = dio_buf;
    generate
        genvar i;
        for (i=0; i<WIDTH; i=i+1)
            assign dio_buf[i] = in_not_out_i[i] ? 1'bz : din_i[i];
    endgenerate
/*`else
    IOBUF io_data [WIDTH-1:0] (
        .O (dout_o),        // data from the pad to the fabric
        .IO(dio_buf),       // connects to the pad
        .I (din_i),         // data from the fabric to tha pad
        .T (in_not_out_i)   // when 1 puts the IO pin in tri-state
    );
`endif*/
endmodule