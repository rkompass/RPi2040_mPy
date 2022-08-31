# Test 4 of RP2_Dma micropython class
#
# In this test we use the UART twice.
#
# You have to connect Pin 13 with Pin 8.
# 
# We also setup a second dma and chain the two.
# Also dma.pause() and dma.resume() are demonstrated.

from machine import UART, Pin
from time import sleep_ms
from rp2_dma import RP2_Dma

# receiving uart , we connected Pin 13 (RX(0))  with  Pin 
uart0 = UART(0, baudrate=115200, tx=Pin(12), rx=Pin(13)) # , bits=8, parity=None, stop=1) # works without
barr0 = bytearray((0,)*70)
dma0 = RP2_Dma(uart0,  barr0)

uart1 = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9)) # , bits=8, parity=None, stop=1) # works without
barr1 = bytearray('Hello, this is our message to be transmitted via UART...')
dma1 = RP2_Dma(barr1,  uart1)

print('barr0:', barr0)
print()
print('start dma0 (receiving) and dma1 (transmitting) ..')
dma0.start()
dma1.start()
# print('barr0:', barr0)   # !!!!!! This actually interferes with the UART reception  !!!!!
sleep_ms(100)  # now it's o.k.
print()
print('barr0:', barr0)
print('dma0.transfer_count():', dma0.transfer_count())
print('dma1.transfer_count():', dma1.transfer_count())
print()
print('dma1.restart()')
dma1.restart()
sleep_ms(100)
print('barr0:', barr0)
print('dma0.transfer_count():', dma0.transfer_count())
print('dma1.transfer_count():', dma1.transfer_count())

