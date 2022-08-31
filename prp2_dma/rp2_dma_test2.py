# Test 2 of RP2_Dma micropython class
#
# In this test we use the PIO state machine to generate a sequence of numbers.
#
#


from array import array
from time import sleep_ms
from rp2 import PIO, StateMachine, asm_pio

from rp2_dma import RP2_Dma

@asm_pio(push_thresh=32, in_shiftdir=PIO.SHIFT_LEFT, out_shiftdir=PIO.SHIFT_RIGHT)
def sm_gen():
    set(x, 0)
    label("incr")    # increment x by inverting, decrementing and inverting
    mov(x, invert(x))      [31]
    jmp(x_dec, "here")     [31]
    label("here")
    mov(x, invert(x))      [31]   # 32 x 25 = 800 instructions per loop
    mov(isr, x)            [31]   # could run with 2000 Hz: 
    nop()                  [31]
    nop()                  [31]
    nop()                  [31]
    nop()                  [31]
    nop()                  [31]
    nop()                  [31]
    nop()                  [31]
    nop()                  [31]
    nop()                  [31]
    nop()                  [31]
    nop()                  [31]
    nop()                  [31]
    nop()                  [31]
    nop()                  [31]
    nop()                  [31]
    nop()                  [31]
    nop()                  [31]
    nop()                  [31]
    nop()                  [31]
    push(noblock)          [31]   #    0.4 s per push()  at 200 Hz
    jmp("incr")            [31]


sm = StateMachine(0, sm_gen, freq=2000)
arr = array('l', (0,)*30)
dma = RP2_Dma(sm,  arr) # dma uses push_thresh=32 in @asm_pio to set dsize=2, but you may try with other dsize

dma.start()
sm.active(1)

while dma.transfer_count() > 0:
    print(dma.transfer_count())
    sleep_ms(400)

sm.active(0)
print('arr:', arr)
