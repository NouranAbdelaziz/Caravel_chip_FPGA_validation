from machine import Pin, SPI
import sys

spi = SPI(0, baudrate=1_000_000, polarity=0, phase=0, bits=8, sck=Pin(6), mosi=Pin(7), miso=Pin(4))
#spi = SPI(1, 10_000_000, sck=Pin(14), mosi=Pin(15), miso=Pin(12))
cs = Pin(9, mode=Pin.OUT, value=1)
temp= int('40',16)
CONT_READ = temp.to_bytes(1, 'big')

temp= int('01',16)
MFGR_ID_REG = temp.to_bytes(1, 'big')

cs(0)                               
spi.write(CONT_READ)          

spi.write(MFGR_ID_REG)          

MFGR_ID = spi.read(2)
cs(1)

print(MFGR_ID)
