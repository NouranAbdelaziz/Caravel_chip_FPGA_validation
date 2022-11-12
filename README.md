# Caravel_chip_FPGA_validation 

The goal of this repository is to validate the caravel chip using FPGA. Caravel is a template chip used in google free shuttles. You can read more about Caravel [here](https://github.com/efabless/caravel). 

# Mangement SoC FPGA validation:

Before programming the FPGA with the whole caravel chip, we started by implementing the management SoC alone. The management SoC contains VexRiscv core and several peripherals like gpio, uart, spi, and timer. It is used to run firmware to read and write in different registers and configure gpio pins of the caravel chip.

In order for the management SoC to read the program from the flash, the flash needs first to be programmed. This can be done by having a flash writer slave which talks to the flash and program it. For the flash writer to do this, it needs a master. The master used here is a uart master which receives commands from PC and program the flash writer which will eventually  program the flash. The uart master and flash writer modules rtl code was based on [this](https://github.com/shalan/SoCBUS ) repository.  The python script used to program the uart master is based on [this](https://github.com/nabadawy/Chameleon_SoC_with-SST26VF080A-flash-/tree/main) repository. You can find an explanation of the commands used to program the flash. 

## Tools used in FPGA validation for the management SoC:
1. Cmod Artix 7-35T
2. QSPI SST26VF080A Flash module
3. USB-TTL module
4. Analog Discovery kit (optional for debugging)
5. Jumper wires for connecting

![Untitled Diagram-Page-1 drawio](https://user-images.githubusercontent.com/79912650/201472822-8a278966-c80b-4a12-a84a-ba6f0bad51df.png)

## Steps of FPGA validation for the management SoC (gpio toggling test):

1. Program the FPGA with the uart master and flash writer design (you can use the bit file [here](https://github.com/NouranAbdelaziz/Caravel_chip_FPGA_validation/blob/main/Flash_programming/bit_file/uart_flash_writer.bit ) or use the source code inorder to synthesize, implement, and generate the bitstream)
2. Get the hex file of the c-code you want to run on the management SoC. (using RISCV GNU toolchain)
3. Use a usb-ttl module to connect the PC with the usrt master then run the python script [here](https://github.com/NouranAbdelaziz/Caravel_chip_FPGA_validation/blob/main/Flash_programming/script.py ). Now, the flash is programmed.
4. Program the FPGA with the management SoC (you can use the bit file [here](https://github.com/NouranAbdelaziz/Caravel_chip_FPGA_validation/blob/main/mgmt_SoC_FPGA_validation/bit_file/mgmt_soc.bit ) or use the source code inorder to synthesize, implement, and generate the bitstream.
5. Connect the flash CSB, SCLK, SDI, and SDO pins of the management SoC in FPGA to those of the flash and connect the vdd and gnd of the flash to those of the pmod connector in the FPGA. The code should be running now. For example, if you used the gpio test provided, you will see a led continuously toggling. 

# Mangement SoC with the housekeeping FPGA validation:

After validating the management SoC alone, we integrated the housekeeping with the management SoC in an RTL design. The housekeeping contains the housekeeping SPI with enables the Caravel chip to communicate with management SoC through an SPI interface. It can write and read several registers and eventually configure gpio pins. 
The housekeeping contains a "front door" SPI interface connected to the padframe through GPIO pins 1 to 4, and a "back door" wishbone interface connected to the management SoC. The test that is provided in this repo is to read the manufacturer ID register using the housekeeping SPI interface. Manufacturer ID is a fixed ID number hardcoded in the Caravel chip and has a value of 0x456. In order to read or write to housekeeping SPI slave, we need a master which will talk to the slave. The master used here is the SPI peripheral of a  Raspberry Pi Pico microcontroller. 

## Software tools used in FPGA validation for the management SoC with the housekeeping:

1. Xilinx Vivado for synthesizing, implementing, and generating the bit stream of the RTL design
2. Digilent Adept for programming the FPGA with the bit file
3. Thonny Python IDE which ease programming the Raspberry Pi Pico with the micropython code
4. Digilent Waveforms for using the analog discovery kit logic analyzer

![Untitled Diagram-Page-3 drawio](https://user-images.githubusercontent.com/79912650/201472941-6656a9e5-51a8-4a4f-bdf5-b34eaa2d0907.png)


## Hardware tools used in FPGA validation for the management SoC with the housekeeping:

1. Cmod Artix 7-35T
2. Raspberry Pi Pico
3. Analog Discovery kit (optional for debugging)
4. Jumper wires for connecting

![Untitled Diagram-Page-2 drawio](https://user-images.githubusercontent.com/79912650/201472853-d9bd0abd-edc6-47fb-a49e-728dd564f366.png)


## Steps of FPGA validation for the management SoC with the housekeeping (reading the manufacturer ID test):

1. Program the FPGA with the design of management SoC and housekeeping (you can use the bit file [here](https://github.com/NouranAbdelaziz/Caravel_chip_FPGA_validation/blob/main/mgmt_SoC_with_HK_FPGA_validation/bit_file/mgmt_soc_hk_38.bit) or use the source code provided in order to synthesize, implement, and generate the bitstream. 
2. Program the Raspberry Pi Pico microcontroller with the micropython script provided [here](https://github.com/NouranAbdelaziz/Caravel_chip_FPGA_validation/blob/main/mgmt_SoC_with_HK_FPGA_validation/Micropython%20script%20for%20pico/spi_pico.py). This script simply sends first 0x40 which is the command used for the housekeeping SPI to do continuous read until the csb signal is raised high. You can read about the different commands for the housekeeping SPI [here](https://caravel-harness.readthedocs.io/en/latest/housekeeping-spi.html). Then it sends 0x01 which is the address of the register we want to read (manufacturer ID register). You can program the pico microcontroller using Thonny IDE as follows:
3. After that, run the python program using this button:
4. You can connect the sclk, csb, and sdi signals to the analog discovery kit and check it in the waveform viewer to make sure that the SPI master is sending the right commands to the housekeeping SPI. This should be the output of the waveform:
5. Check the reply of the housekeeping SPI in the sdo signal, it should reply with 0x456 which is the value of the manufacturer ID register. It should appear in the waveform and will be read in the Thonny IDE. 
 
