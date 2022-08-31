I wrote my own dma class for RPI2040, inspired by https://github.com/robert-hh/RP2040-Examples/tree/master/rp2_util.

The idea was to minimize configuration arguments by writing helper functions that check the type of source and destination and deliver these.
For example:
```
def par_bytearray(obj, src):
    return (addressof(obj), 1, len(obj), 0, None) if isinstance(obj, bytearray) else None
```
checks, if an argument (source, if src==1, else destination) is of type bytearray.
If so, it returns a tuple containing
* addr:    starting address of transfer       -> addressof(obj)
* incr:    address increment (0 for no, 1 for increment)    -> 1 
* count:   number of data transfers (None if not specified) -> len(obj)
* dsize:   data transfer size: 0 for 8 bits, 1 for 16, 2 for 32 bits (None if not specified) -> 0
* dreq:    data request number, if the object synchronizes its transfers by that, else None  -> None


Usage:
------

```
from rp2_dma import RP2_Dma

src_arr = array('i', (i for i in range(-10, 9_990)))
dst_arr = array('L', (0 for _ in range(11_000)))   # may be longer than the source; dma takes the minimum of both counts

dma = RP2_Dma(sarr,  darr)
dma.start()
```
Dma becomes as simple as putting in the two objects.
Of course the configuration parameters may be specified explicitly, thus overriding the extracted ones.
```
init(self, source, dest, rw_incr=None, cycle=None, count=None, dsize=None, dreq=None, chan=None)
```
* rw_incr = (1, 0)  if only the source is an array to be incremented, destination is a fixed memory address
* cycle =  DMA_Cycle_Src if the source array is to be cyclically read, source may now be a list or tuple of 1, 2, 4, or 8 similar objects.
cycle = DMA_Cycle_Dst works analogoulsy.
* count = 100  to transfer just 100 words (8, 16 or 32 bit).
* dsize = 0, 1 or 2 for 8, 16 or 32 bit transfer.
* dreq = 0x3b for example to synchronize transfers with timer 0. We have a constant for that: dreq = TREQ_Timer0
* chan = 5 to specify dma channel 5 explicitly. Usually the instantiation of the dma class takes the first free channel
and marks that internally als used, until dma.deinit() is executed.

At present there are helper functions for objects of type:
* integer  (defaults for fixed memory addresses, i.e. increment = 0, which may be overridden)
* bytearray
* array
* PIO StateMachine
* UART
* PWM
* ADC  (not tested yet)
* tuples/lists of similar objects in combination with the cycle argument

I plan to add I2C and SPI helpers too.

Once a dma object is instantiated you can use the following member functions:
* init() to change parameters
* deinit() to abort the transfer and free the dma channel (you still have to delete the dma object)
* chain_to() another dma object
* start() to trigger the dma transfer
* restart() to trigger the same transfer again
* busy() to check wheter dma is busy presently. 
* transfer_count() to get number of still to be done transfers
* read_addr() to get the current read register value
* write_addr() to get the current write register value
* pause() to pause the transfer
* resume() to resume it
* abort() to abort a transfer

There are 5 examples:
+ rp2_dma_test1.py does ordinary array transfer and measures memory and time consumption.
+ rp2_dma_test2.py reads from a PIO state machine into an array.
+ rp2_dma_test3.py writes to a fixed location: a PIO clock divider, thus speeding up a PIO state machine that blinks the LED.
+ rp2_dma_test4.py does 2 dma transfers in parallel: from an array to UART1 and from UART0 to another array. Needs a wire connecting both UARTs.
+ rp2_dma_test5.py writes a waveform (sinus) to PWM that is configured with 44.1 kHz, 11 bits.
This tests the cycle option which does reconfiguration of the dma (slave) by another (master) dma.

