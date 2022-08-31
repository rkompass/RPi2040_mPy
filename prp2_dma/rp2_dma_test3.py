# Test 3 of RP2_Dma micropython class
#
# In this test we use the PIO to blink a LED on a Pin,
#   then we DMA timedly different values to the PIO clock frequency divider
#   in order to let the blinking accelerate.
#
# We also setup a second dma and chain the two.
# Also dma.pause() and dma.resume() are demonstrated.

from rp2 import PIO
from rp2_dma import RP2_Dma, TREQ_Timer0
from machine import Pin, Timer
from array import array
from time import ticks_ms, sleep_ms

# Define the blink program.  It has one GPIO to bind to on the set instruction, which is an output pin.
# Use lots of delays to make the blinking visible by eye.
# This is from https://github.com/raspberrypi/pico-micropython-examples/blob/master/pio/pio_1hz.py
@rp2.asm_pio(set_init=PIO.OUT_LOW)
def blink_1hz():
    # Cycles: 1 + 1 + 6 + 32 * (30 + 1) = 1000
    irq(rel(0))
    set(pins, 1)
    set(x, 31)                  [5]
    label("delay_high")
    nop()                       [29]
    jmp(x_dec, "delay_high")

    # Cycles: 1 + 7 + 32 * (30 + 1) = 1000
    set(pins, 0)
    set(x, 31)                  [6]
    label("delay_low")
    nop()                       [29]
    jmp(x_dec, "delay_low")

ltck = 0
tck = 0

def pr_ticks(p):      # The pio calls this routine every 2000 clock cycles
    global ltck, tck
    tck = ticks_ms()
    print(tck - ltck)
    ltck = tck

# Create the StateMachine with the blink_1hz program, outputting on Pin(25) = LED.
sm = rp2.StateMachine(0, blink_1hz, freq=2000, set_base=Pin(25))
sm.irq(pr_ticks)  # Set the IRQ handler to print the milliseconds passed.

sm0_clkdiv = 0x502000c8     # address of StateMachine 0 clock divider
# Create an array of clock dividers for PIO.
# We start with clock divider 62500 as this would lead to 2000 Hz Pio clock.
clk_h = array('i', [
    62500<<16,          # 62500    ->  2000 Hz  ->  1 Hz blink
    62500<<15,          # 31250    ->  4000 Hz  ->  2 Hz
    62500<<14,          # 15625    ->  8000 Hz  ->  3 Hz
    62500<<13,          #  7812.5  -> 16000 Hz  ->  8 Hz
    62500<<12])         #  3906.25 -> 32000 Hz  -> 16 Hz blink

# We need a much larger array if we want the clock divider to change after a second.
# The slowest internaly timed dma is 2000 Hz, so
#     fill clk_arr with above values, each repeated 2000 times.
clk_arr = array('i', (clk_h[i//2000] for i in range(10000)))

# Set up dma of clk_arr to StateMachine 0 clock divider address. Explicitly set write increment to False.
#    Set dma paced by dma timer 0; dma does not start yet.
dma = RP2_Dma(clk_arr,  sm0_clkdiv, rw_incr=(None, False), dreq=TREQ_Timer0)
dma.timer(0, freq=2000)                # Set dma timer 0 to 2000 Hz.
dma2 = RP2_Dma(clk_arr,  sm0_clkdiv, rw_incr=(None, False), dreq=TREQ_Timer0)

ltck = ticks_ms()
sm.active(1)
dma.start()
dma.chain_to(dma2)
# sleep_ms(2_000)

while dma.busy() > 0:
    # print(dma.transfer_count())
    sleep_ms(100)

sleep_ms(2_000)
dma2.pause()
print('dma2 paused')
sleep_ms(2_000)
dma2.resume()
print('dma2 resumed')

while dma2.busy() > 0:
    # print(dma2.transfer_count())
    sleep_ms(100)

sm.active(0) # stop blinking after dma transfer is complete (this could be done by another dma automatically ...)

