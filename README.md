# Caravel_chip_FPGA_validation 

The goal of this repository is to validate the caravel chip using FPGA. Caravel is a template chip used in google free shuttles. You can read more about Caravel [here](https://github.com/efabless/caravel). 

# Mangement SoC FPGA validation:

Before programming the FPGA with the whole caravel chip, we started by implementing the management SoC alone. The management SoC contains VexRiscv core and several peripherals like gpio, uart, spi, and timer. It is used to run firmware to read and write in different registers and configure gpio pins of the caravel chip.

In order for the management SoC to read the program from the flash, the flash needs first to be programmed. This can be done by having a flash writer slave which talks to the flash and program it. For the flash writer to do this, it needs a master. The master used here is a uart master which receives commands from PC and program the flash writer which will eventually  program the flash. The uart master and flash writer modules rtl code was based on [this](https://github.com/shalan/SoCBUS ) repository.  The python script used to program the uart master is based on [this](https://github.com/nabadawy/Chameleon_SoC_with-SST26VF080A-flash-/tree/main) repository. You can find an explanation of the commands used to program the flash. 

## Tools used in FPGA validation for the management SoC:

1. ### Cmod Artix 7-35T:
![cmod_a7](https://user-images.githubusercontent.com/79912650/193458455-6ed313b3-190f-4531-bcfa-35cff91bb603.jpg)

2. ### QSPI SST26VF080A Flash module:


3. ### USB-TTL module:
![usb-ttl](https://user-images.githubusercontent.com/79912650/201466506-2fa9b794-09ef-4e94-9dda-6646e44e70cb.jpg)

4. ### Analog Discovery kit (optional for debugging):
![analog-discovery-0](https://user-images.githubusercontent.com/79912650/201466480-52c9c796-93fd-4cdf-9c29-f1ebe37e7560.png)

5. ### Jumper wires for connecting:
![wires](https://user-images.githubusercontent.com/79912650/201466532-ef57a7f3-a770-47af-bb6a-6dba0795a568.jpg)

## Steps of FPGA validation for the management SoC (gpio toggling test):

1. Program the FPGA with the uart master and flash writer design (you can use the bit file [here](https://github.com/NouranAbdelaziz/Caravel_chip_FPGA_validation/blob/main/Flash_programming/bit_file/uart_flash_writer.bit ) or use the source code inorder to synthesize, implement, and generate the bitstream)
2. Get the hex file of the c-code you want to run on the management SoC. (using RISCV GNU toolchain)
3. Use a usb-ttl module to connect the PC with the usrt master then run the python script [here](https://github.com/NouranAbdelaziz/Caravel_chip_FPGA_validation/blob/main/Flash_programming/script.py ). Now, the flash is programmed.
4. Program the FPGA with the management SoC (you can use the bit file [here](https://github.com/NouranAbdelaziz/Caravel_chip_FPGA_validation/blob/main/mgmt_SoC_FPGA_validation/bit_file/mgmt_soc.bit ) or use the source code inorder to synthesize, implement, and generate the bitstream.
5. Connect the flash CSB, SCLK, SDI, and SDO pins of the management SoC in FPGA to those of the flash and connect the vdd and gnd of the flash to those of the pmod connector in the FPGA. The code should be running now. For example, if you used the gpio test provided, you will see a led continuously toggling. 

# Mangement SoC with the housekeeping FPGA validation:

After validating the management SoC alone, we integrated the housekeeping with the management SoC in an RTL design. The housekeeping contains the housekeeping SPI with enables the Caravel chip to communicate with management SoC through an SPI interface. It can write and read several registers and eventually configure gpio pins. 
The housekeeping contains a "front door" SPI interface connected to the padframe through GPIO pins 1 to 4, and a "back door" wishbone interface connected to the management SoC. The test that is provided in this repo is to read the manufacturer ID register using the housekeeping SPI interface. Manufacturer ID is a fixed ID number hardcoded in the Caravel chip and has a value of 0x456.

## Software tools used in FPGA validation for the management SoC with the housekeeping:
1. ### Xilinx Vivado ML Edition - 2022.2 for synthesizing, implementing, and generating the bit stream of the RTL design:
![vivado](https://user-images.githubusercontent.com/79912650/201467757-2e178d5e-8624-44ed-8a54-75b6125f6610.jpg)

2. ### Digilent Adept for programming the FPGA with the bit file:
![image (1)](https://user-images.githubusercontent.com/79912650/201468028-df1686cf-06f7-45bb-8d2d-2987a8f261fd.png)

3. ### Thonny Python IDE which ease programming the Raspberry Pi Pico with the micropython code:
![Thonny_logo](https://user-images.githubusercontent.com/79912650/201468053-660b531c-7143-44c5-ac6a-1804a08632aa.png)

4. ### Digilent Waveforms for using the analog discovery kit logic analyzer:
![image](https://user-images.githubusercontent.com/79912650/201467981-e6647c4f-5c78-41ef-a074-34cb006c1880.png)


## Hardware tools used in FPGA validation for the management SoC with the housekeeping:
1. ### Cmod Artix 7-35T:
![cmod_a7](https://user-images.githubusercontent.com/79912650/193458455-6ed313b3-190f-4531-bcfa-35cff91bb603.jpg)

2. ### Raspberry Pi Pico: 
![pico](https://user-images.githubusercontent.com/79912650/201466715-b76a8ad2-03c6-4401-b936-da8029c4830c.jpg)

3. ### Analog Discovery kit (optional for debugging):
![analog-discovery-0](https://user-images.githubusercontent.com/79912650/201466480-52c9c796-93fd-4cdf-9c29-f1ebe37e7560.png)

4. ### Jumper wires for connecting:
![wires](https://user-images.githubusercontent.com/79912650/201466532-ef57a7f3-a770-47af-bb6a-6dba0795a568.jpg)

## Steps of FPGA validation for the management SoC with the housekeeping (reading the manufacturer ID test):

1. 
