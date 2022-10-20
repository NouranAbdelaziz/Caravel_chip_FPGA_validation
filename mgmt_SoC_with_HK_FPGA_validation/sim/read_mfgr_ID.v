// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none

`timescale 1 ns / 1 ps


module spi_master_tb;
	reg clock;
	reg RSTB;
	reg power1, power2;

	reg SDI, CSB, SCK; //
	wire [4:0] mgmt_io_in;

	wire gpio;
	wire [15:0] checkbits;
	wire [7:0] spivalue;
	wire [37:0] mprj_io;
	wire flash_csb;
	wire flash_clk;
	wire flash_io0;
	wire flash_io1;
	

	wire [1:0] mgmt_io_out;

	wire SDO; //

//	wire spi_clk;
//	wire spi_cs_n;
//	wire spi_mosi;
//	wire spi_miso;
//	wire spi_sdoenb;

	//assign checkbits = mprj_io[31:16];
	//assign spivalue  = mprj_io[15:8];

	// External clock is used by default.  Make this artificially fast for the
	// simulation.  Normally this would be a slow clock and the digital PLL
	// would be the fast clock.

	always #10 clock <= (clock === 1'b0);

	initial begin
		clock = 0;
	end

	initial begin
		$dumpfile("spi_master.vcd");
		$dumpvars(0, spi_master_tb);
		repeat (5) begin
			repeat (5000) @(posedge clock);
			$display("+5000 cycles");
		end
		$display("%c[1;31m",27);
		`ifdef GL
			$display ("Monitor: Timeout, Test SPI Master (GL) Failed");
		`else
			$display ("Monitor: Timeout, Test SPI Master (RTL) Failed");
		`endif
		 $display("%c[0m",27);
		$finish;
	end

	integer i;

	task start_csb;
	    begin
		SCK <= 1'b0;
		SDI <= 1'b0;
		CSB <= 1'b0;
		#50;
	    end
	endtask

	task end_csb;
	    begin
		SCK <= 1'b0;
		SDI <= 1'b0;
		CSB <= 1'b1;
		#50;
	    end
	endtask

	task write_byte;
	    input [7:0] odata;
	    begin
		SCK <= 1'b0;
		for (i=7; i >= 0; i=i-1) begin
		    #50;
		    SDI <= odata[i];
                    #50;
		    SCK <= 1'b1;
                    #100;
		    SCK <= 1'b0;
		end
	    end
	endtask

	task read_byte;
	    output [7:0] idata;
	    begin
		SCK <= 1'b0;
		SDI <= 1'b0;
		for (i=7; i >= 0; i=i-1) begin
		    #50;
                    idata[i] = SDO;
                    #50;
		    SCK <= 1'b1;
                    #100;
		    SCK <= 1'b0;
		end
	    end
	endtask

	reg [7:0] manufacturer_id_first_byte;
	reg [7:0] manufacturer_id_second_byte;

	initial begin
		#1000
		start_csb();
		write_byte(8'h40);    // command to read  until CSB raised
		write_byte(8'h01);	  // Address of Manufacturer ID bit 0 to 7 
		read_byte(manufacturer_id_first_byte);
		$display("   manufacturer_id_first_byte = 0x%x (should be 0x04)", manufacturer_id_first_byte);
		read_byte(manufacturer_id_second_byte);
		$display("   manufacturer_id_second_byte = 0x%x (should be 0x56)", manufacturer_id_second_byte);
		end_csb();


	end

	initial begin
		RSTB <= 1'b1;
		#1000;
		RSTB <= 1'b0;	    // Release reset
		#2000;
	end

	initial begin		// Power-up sequence
		power1 <= 1'b0;
		power2 <= 1'b0;
		#200;
		power1 <= 1'b1;
		#200;
		power2 <= 1'b1;
	end

	always @(checkbits) begin
		#1 $display("GPIO state = %b ", checkbits);
	end

	wire VDD3V3;
	wire VDD1V8;
	wire VSS;
	
	assign VDD3V3 = power1;
	assign VDD1V8 = power2;
	assign VSS = 1'b0;

	//assign mprj_io[3] = 1'b1;       // Force CSB high.

	assign mgmt_io_in[4] = SCK;
	assign mgmt_io_in[3] = CSB;
	assign mgmt_io_in[2] = SDI;
	assign SDO = mgmt_io_out[1];

	
    mgmt_soc_hk uut (
	`ifdef USE_POWER_PINS
		.VPWR		  (VDD1V8),
		.VGND		  (VSS),
	`endif
		.clk	  (clock),
		.RST	  (RSTB),
		.gpio_inout_pad(gpio),
		.mgmt_io_in(mgmt_io_in),
		.mgmt_io_out(mgmt_io_out),
		.flash_csb(flash_csb),
		.flash_clk(flash_clk),
		.flash_io0_dio(flash_io0),
		.flash_io1_dio(flash_io1),
		.flash_io2_dio(),
        .flash_io3_dio()
	);

	spiflash #(
		.FILENAME()
	) spiflash (
		.csb(flash_csb),
		.clk(flash_clk),
		.io0(flash_io0),
		.io1(flash_io1),
		.io2(),			// not used
		.io3()			// not used
	);

	

endmodule
`default_nettype wire
