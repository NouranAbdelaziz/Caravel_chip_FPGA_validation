# Caravel_chip_FPGA_validatio 

The goal of this repository is to validate the caravel chip using FPGA. Caravel is a template chip used in google free shuttles. You can read more about it [here](https://github.com/efabless/caravel). 

The FPGA used is Cmod Artix 7-35T 
![cmod_a7](https://user-images.githubusercontent.com/79912650/193458455-6ed313b3-190f-4531-bcfa-35cff91bb603.jpg)

Before programming the FPGA with the whole caravel chip, we started by implementing the management SoC alone. The management SoC contains VexRiscv core and several peripherals like gpio, uart, spi, and timer. It is used to run firmware to read and write in different registers and configure gpio pins of the caravel chip.

In order for the management SoC to read the program from the flash, the flash needs first to be programmed. This can be done by having a flash writer slave which talks to the flash and program it. For the flash writer to do this, it needs a master. The master used here is a uart master which receives commands from PC and program the flash writer which will eventually  program the flash. The uart master and flash writer modules rtl code was based on [this](https://github.com/shalan/SoCBUS ) repository.  The python script used to program the uart master is based on [this](https://github.com/nabadawy/Chameleon_SoC_with-SST26VF080A-flash-/tree/main) repository. You can find an explanation of the commands used to program the flash. 


## Steps of FPGA validation in the management SoC:

1. Program the FPGA with the uart master and flash writer design (you can use the bit file [here](https://github.com/NouranAbdelaziz/Caravel_chip_FPGA_validation/blob/main/Flash_programming/bit_file/uart_flash_writer.bit ) or use the source code inorder to synthesize, implement, and generate the bitstream)
2. Get the hex file of the c-code you want to run on the management SoC. (using RISCV GNU toolchain)
3. Use a usb-ttl module to connect the PC with the usrt master then run the python script [here](https://github.com/NouranAbdelaziz/Caravel_chip_FPGA_validation/blob/main/Flash_programming/script.py ). Now, the flash is programmed.
4. Program the FPGA with the management SoC (you can use the bit file [here](https://github.com/NouranAbdelaziz/Caravel_chip_FPGA_validation/blob/main/mgmt_SoC_FPGA_validation/bit_file/mgmt_soc.bit ) or use the source code inorder to synthesize, implement, and generate the bitstream)
5. Connect the flash CSB, SCLK, SDI, and SDO pins of the management SoC in FPGA to those of the flash and connect the vdd and gnd of the flash to those of the pmod connector in the FPGA. The code should be running now. For example, if you used the gpio test provided, you will see a led continuously toggling. 

