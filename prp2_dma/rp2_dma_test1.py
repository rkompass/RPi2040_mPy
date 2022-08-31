# Test 1 of RP2_Dma micropython class
#
# In this test we investigate ordinary memory array transfer
#   and measure memory consumption and timing of the dma
#

from array import array
from time import sleep_ms, ticks_us, ticks_diff
import gc

# import machine
# machine.freq(250_000_000)   # of course works twice as fast

gc.collect(); mem0 = gc.mem_free()
print('before "import RP2_Dma":', mem0)
from rp2_dma import RP2_Dma

gc.collect(); mem1 = gc.mem_free()
print('after "import RP2_Dma":', mem1, 'consumed:', mem0-mem1, 'bytes')

# barr = bytearray((0,)*400)
sarr = array('i', (i for i in range(-10, 9_990)))
print("sarr = array('i', (i for i in range(-10, 9_990)))")
darr = array('L', (0 for _ in range(11_000)))
print("darr = array('L', (0 for _ in range(12_000)))")

gc.collect(); mem2 = gc.mem_free()
print('after seeting up arrays:', mem2, 'consumed:', mem1-mem2, 'bytes')

# ----------------------------------------------
dma = RP2_Dma(sarr,  darr)
# ----------------------------------------------
# dsize is determined automatically from array type as 2 (=32 bit)
# alternatively you may specify dsize explicitly and try out:
# dma = RP2_Dma(sarr,  darr, dsize=0)   # byte transfer, or
# dma = RP2_Dma(sarr,  darr, dsize=1)   # 16-bit transfer
# if dsize is too small we might only transfer half or quarter of the data

print()

gc.collect(); mem3 = gc.mem_free()
print('dma = RP2_Dma(sarr,  darr):', mem3, 'consumed:', mem2-mem3, 'bytes')

t_sta = ticks_us()
dma.start()
while dma.busy():
    pass
t_fin = ticks_us()
t_dif = ticks_diff(t_fin, t_sta)
print('dma transfer took', t_dif, 'us for 10000 elements:', t_dif/10, ' ns per single transfer')
print()

print('darr[5:15]:', darr[5:15])
print('darr[9_990: 10_010]:', darr[9_990: 10_010])