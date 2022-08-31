#  Older version of RP2_Dma, that has no extra init() and no cycle/reconf option
#
#  Todo:
# ------
#  test adc conversion at different rates
#  in adc dma empty fifo before starting trigger
#
#  Ping pong with 2, possibly more concatenated dma channels
#  dma single class and dma multi version for ping-pong and repeated fill modes
#  same objects as lists possible, but need flag for circulation or not
#
#  config
#  deinit
#
#  include interrupts, how?

#  Open for discussion:
# --------------------
#  With integer address as source or destination assume incr = 0 or rather incr = 1 ? (now: incr = 0).
#  With timer-paced dma assign timers 0..3 automatically, making timer() a member function?
#  Do we need an option to start dma automatically with setup?

#  Explore:
# -------------
#  SPI (with display), I2C
#  Setting up a PIO channel to generate slower timed transfer requests



import machine
from array import array
from ctypes import addressof
from rp2 import StateMachine

from time import sleep_ms, sleep_us

# source, dest may be of type array, bytearray, integer (referring to memory location) or struct

def array_dsize(arr):
    sizes = {'b': 0, 'B': 0, 'h':1, 'H':1, 'i':2, 'I':2, 'l':2, 'L':2, 'q':3, 'Q':3, 'f':2, 'd':3}
    baddr=bytes(array('O', [arr]))
    addr=int.from_bytes(baddr, 'little')
    ccode = chr(machine.mem8[addr+4])
    if ccode in sizes:
        return sizes[ccode]
    else:
        raise ValueError('Unknown array format code')

# ---- a number of functions that extract data from an object that might be source (src=1) or destination (src=0) in the DMA transfer -----
#
#  arguments:
#     obj:     object
#     src:     1 if object is source, or 0 if destination
#  returns:
#     addr:    starting address of transfer                                                             *  always returned
#     incr:    address increment (0 for no, 1 for increment)                                            *  always returned
#     count:   number of data transfers (None if not specified)                                         -  may be None
#     dsize:   data transfer size: 0 for 8 bits, 1 for 16, 2 for 32 bits (None if not specified)        -  may be None
#                  note that dsize may be 3 (64 bits: that has to be corrected later by doubling the count)
#     dreq:    data request number, if the object synchronizes its transfers by that, else None         -  may be None
#

def par_int(obj, src):      # we have a memory address, assume incr = 0, but don't know anything else: this has to be specified in dma arguments
    return (obj, 0, None, None, None) if isinstance(obj, int) else None

def par_bytearray(obj, src):
    return (addressof(obj), 1, len(obj), 0, None) if isinstance(obj, bytearray) else None

def par_array(obj, src):
    return (addressof(obj), 1, len(obj), array_dsize(obj), None) if isinstance(obj, array) else None

def par_StateMachine(obj, src):
    if not isinstance(obj, StateMachine):
        return None
    #  print('recognized StateMachine:', obj)    # debugging
    sm = int(str(obj).split(')')[0].split('(')[1])
    if sm < 4:                   # PIO 0
        pio = 0x50200000         # PIO0_BASE
        dreq = sm + 4*src        # 0..3 if dest, 4..7 if source
    else:                        # PIO1
        sm %= 4
        pio = 0x50300000         # PIO1_BASE
        dreq = sm + 8 + 4*src    # 8..11 if dest, 12..15 if source
    addr = pio + 0x10 + 0x10*src + 4*sm  # PIO_TXF0 / PIO_RXF0 depending on src
    bits = (machine.mem32[pio+0xd0+24*sm] >> (20 if src else 25)) & 0x1f  # extract PUSH_THRESH if source else PULL_THRESH: 
    if bits > 16 or bits == 0:    # number of bits shifted to ISR/from OSR before autopush/autopull or cond. push/pull; 0 for value 32
        dsize = 2
    elif bits > 8:
        dsize = 1
    else:
        dsize = 0
    return addr, 0, None, dsize, dreq

def par_UART(obj, src):
    if not isinstance(obj, machine.UART):
        return None
    uart = int(str(obj).split(',')[0].split('(')[1])  # 0 or 1, depending on uart
    addr = 0x40034000 + 0x4000*uart
    dreq = 20 + 2*uart + src        # 20: uart0_tx, 21: uart0_rx, 22: uart1_tx, 23: uart1_rx
    return addr, 0, None, 0, dreq   # dsize is 0, as we transmit/receive bytes
    # Note that UART DMA rx und tx enable bits are set already with standard UART setup (nothing to do here).

def par_PWM(obj, src):
    if not isinstance(obj, machine.PWM):
        return None
    slic = int(str(obj).split(' ')[1].split('=')[1])    # 0 .. 7
    chan = int(str(obj).split(' ')[2].split('=')[1][0]) # 0 or 1 for A or B
    if src == 0:  # ordinary PWM mode
        addr = 0x4005000c + 0x14*slic + 0x02*chan  # counter compare register; upper 16 bits if chan == 1
        dreq = 24 + slic        # 24: pwm_wrap0, 25: pwm_wrap1, 26: pwm_wrap2, ...
        # Note that this dreq signals a dma request at every single pwm pulse start: Do you want that?
        #   Perhaps you are rather interested in a timer dreq, if you want the DMA changing
        #   analog output more slowly, thus integrating over pwm pulses to achieve some sort of DAC output.
        #   In this case you have to specify dreq explicitly at DMA configuration.
    else:      # we read counter or timer
        if not chan:
            raise ValueError('A PWM channel A cannot be DMA source')
        addr = 0x40050008 + 0x14*slic # counter reg low 16b, to dma these; which may reflect timing or counting of pin B
        dreq = None
    return addr, 0, None, 1, dreq   # dsize is 1, as we always transmit/receive 16 bits

def par_ADC(obj, src):   
    if not isinstance(obj, machine.ADC):
        return None
    if not src:
        raise ValueError('ADC can only be source for DMA')
    return 0x4004c00c, 0, None, 1, 36   # ADC conversion result FIFO, no incr, no count, 16 bit, DREQ_ADC


_par_funcs = (par_int, par_bytearray, par_array, par_StateMachine, par_UART, par_PWM, par_ADC)

# code from https://github.com/alidasdan/best-rational-approximation
def contfr_rat(t):   # best rational approx of t with max denominator 65535 (may be subst. by other N)
    a, b = 1, 0                # initialize the matrix
    c, d = 0, 1
    x, xi = t, int(t)
    while (c*xi + d) <= 65535: # loop finding terms until denom gets too big
        b, a = a, a * xi + b   # multiply these two matrices:
        d, c = c, c * xi + d   #    (a c) and (xi 1)
        if (x == float(xi)):   #    (c d)     (1  0)
            break
        x = 1.0/(x-xi)         # now remaining x is between 0 and 1/xi. Approx as either 0 or 1/m
        xi = int(x)            # where m is max that will fit in 65535.  
    xi = int((65535-d)/c)      # try the other possibility
    p, q = a*xi+b, c*xi+d
    if abs(t-a/c) <= abs(t-p/q):
        return a, c            # first possibility: try zero
    else:
        return p, q

# transfer (=data) request values for internal DMA transfer timers
TREQ_Timer0 = const(0x3b)    # one of these constants may be used as dreq value
TREQ_Timer1 = const(0x3c)    # then the transfer is paced by the designated
TREQ_Timer2 = const(0x3d)    # internal timer
TREQ_Timer3 = const(0x3e)

class RP2_Dma:

    chans = array('b', (0,0,0,0,0,0,0,0,0,0,0,0))    # stores 0 for free, or 1 for used DMA channel

    # sets up a dma object but does not start the transfer
    #
    def __init__(self, source, dest, rw_incr=None, count=None, dsize=None, dreq=None, chan=None):
        
        if chan is None:               # if not explicitly specified, get a free DMA channel
            try:
                chan = next((i for i, v in enumerate(RP2_Dma.chans) if v == 0))
            except StopIteration:
                raise ValueError('No more free dma channels')
        elif chan < 0 or chan > 11:
            raise ValueError('Dma channel outside range 0..11')
        RP2_Dma.chans[chan] = 1    # mark dma channel as used
        self.chan = chan
        
        # ----------- extract source data from known source types --------------------
        for fun in _par_funcs:
            src_pars = fun(source, 1)
            if src_pars is not None:
                break
        if src_pars is None:
            raise ValueError('Cannot handle source type')
 #       print('src_pars:', src_pars)     # debugging

        # ----------- extract destination data from known destination types -----------
        for fun in _par_funcs:
            dst_pars = fun(dest, 0)
            if dst_pars is not None:
                break
        if dst_pars is None:
            raise ValueError('Cannot handle destination type')
 #       print('dst_pars:', dst_pars)     # debugging

        # ----------- combine known data: (addr, incr, count, dsize, dreq) ------------
        # ----- address data: nothing to do -----
        src_addr = src_pars[0]                                   # not necessary, may use directly
        dst_addr = dst_pars[0]                                   # not necessary, may use directly
        # ----- increment data: explicit has priority -----
        t = rw_incr[0] if isinstance(rw_incr, (tuple, list)) else rw_incr
        src_incr = src_pars[1] if t is None else t               # source increment: explicit argument overwrites value by object parameter function
        t = rw_incr[1] if isinstance(rw_incr, (tuple, list)) else rw_incr
        dst_incr = dst_pars[1] if t is None else t               # destination increment: explicit argument overwrites value by object parameter function
        # ----- count data: explicit has priority, minimum of given source/dest counts -----
        t = src_pars[2] if dst_pars[2] is None else dst_pars[2] if src_pars[2] is None else min(src_pars[2], dst_pars[2])  # may define  emin(x, y) for that
        tr_cnt = t if count is None else count                # transfer count: explicit argument overwrites minimum
        if tr_cnt is None:
            raise ValueError('Unknown transfer count: specify count explicitly')
        # ----- data size data: explicit has priority;  !!! think about resolving confliciting data sizes  !!!! -----
        if dsize is None and src_pars[3] is not None and dst_pars[3] is not None and src_pars[3] != dst_pars[3]:
            raise ValueError('Conflicting source/destination data sizes: specify dsize explicitly')  # !!!! an option for source or dest. preference should be available as special dsize argument
        t = src_pars[3] if dst_pars[3] is None else dst_pars[3]
        tr_dsz = t if dsize is None else dsize                # data size: explicit argument overwrites available values
        if tr_dsz not in (0, 1, 2):  #  <- this includes None
            raise ValueError('Unknown data size: specify dsize explicitly as 0 (8 bits), 1 (16 bits) or 2 (32 bits)')
        # ----- dreq data: explicit has priority;  !!! think about resolving confliciting data sizes  !!!! -----
        if dreq is None and src_pars[4] is not None and dst_pars[4] is not None and src_pars[4] != dst_pars[4]:
            raise ValueError('Conflicting source/destination data requests: specify dreq explicitly')  # !!!! an option for source or dest. preference should be available as special dsize argument
        t = src_pars[4] if dst_pars[4] is None else dst_pars[4]
        tr_drq = t if dreq is None else dreq
        if tr_drq is None:
            tr_drq = 0x3f  # if no other explicit request: permanent request for unpaced transfers

        # ----------- setup dma from determined values and start -----------

        dma_addr = 0x50000000 + self.chan * 0x40      # DMA_BASE + chan * channel_offset
        
        #                           setting to own channel disables chaining
        #         do not generate int   do not chain                              no wrapping
        #            IRQ_QUIET           CHAIN_TO            RING_SEL              RING_SIZE       HIGH_PRIORITY       EN
        dma_ctrl  =  (1 << 21)    |   (chan << 11)     |     (0 << 10)      |      (0 << 6)      |    (1 << 1)   |     1
        
#        print('tr_drq:', hex(tr_drq), 'dst_incr:', dst_incr, 'src_incr:', src_incr, 'tr_dsz:', tr_dsz)  # debug
        dma_ctrl |=  (tr_drq << 15) | (dst_incr << 5) | (src_incr << 4) | (tr_dsz << 2) 
                    
        machine.mem32[dma_addr]    = src_addr      # READ_ADDR
        machine.mem32[dma_addr+4]  = dst_addr      # WRITE_ADDR
        machine.mem32[dma_addr+8]  = tr_cnt        # TRANS_COUNT
#        machine.mem32[dma_addr+12] = dma_ctrl      # CTRL_TRIG, this would start the transfer
        machine.mem32[dma_addr+16] = dma_ctrl      # CTRL in Alias 1, this does not start the transfer
                                      #    you now have to use RP2_Dma.start() for that

    def deinit(self):
        self.abort()
        machine.mem32[0x5000000c + self.chan * 0x40] = 0  # empty control register, does not trigger
        RP2_Dma.chans[self.chan] = 0                      # mark dma channel as not used


    @staticmethod
    def timer(n, freq=None):
        if n < 0 or n > 3:
            raise ValueError('Wrong dma transfer timer channel')
        f_cpu = machine.freq()
        if freq is not None:
            x, y = contfr_rat(freq / f_cpu)
            machine.mem32[0x50000420+n*4] = x<<16 | y   # Set dma pacing fractional timer0..3 register
        xy = machine.mem32[0x50000420+n*4]
        return (xy>>16) * f_cpu / (xy&0xffff)

    def chain_to(self, dma2=None):
        dma_addr = 0x50000000 + self.chan * 0x40      # DMA_BASE + chan * channel_offset
        if isinstance(dma2, RP2_Dma):
            dma2 = dma2.chan
        elif isinstance(dma2, bool) and dma2 == False:  # setting to own channel disables chaining
            dma2 = self.chan
        if isinstance(dma2, int):                       # we do the chain setting
            if dma2 < 0 or dma2 > 11:
                raise ValueError('dma2 outside range 0..11')
            dma_ctrl = machine.mem32[dma_addr+16] & 0xffff87ff
            machine.mem32[dma_addr+16] = dma_ctrl | (dma2 << 11)
        ch_to = (machine.mem32[dma_addr+16] >> 11) & 0x0f
        return ch_to if ch_to != self.chan else False

    @micropython.viper
    def start(self):
        chan = int(self.chan)
        p = ptr32(0x50000430)
        # !!!! # Check if we have ADC as source and then empty FIFO
        p[0] = 1<<chan    # Set the bit in multi-channel trigger register
        
    @micropython.viper
    def is_busy(self) -> bool:
        chan = int(self.chan)
        p = ptr32(0x50000010)
        res = (p[chan<<4] >> 24) & 0x81
        return res == 0x01  # no AHB_ERROR bit but BUSY bit in alias ctrl register

    @micropython.viper
    def transfer_count(self) -> int:
        chan = int(self.chan)
        p = ptr32(0x50000008)
        return p[chan<<4]

    @micropython.viper                # Get the current read register value
    def read_addr(self) -> int:
        chan = int(self.chan)
        p = ptr32(0x50000000)
        return p[chan<<4]

    @micropython.viper                # Get the current write register value
    def write_addr(self) -> int:
        chan = int(self.chan)
        p = ptr32(0x50000004)
        return p[chan<<4]

    @micropython.viper                # Abort the transfer
    def abort(self):
        chan = int(self.chan)
        p = ptr32(0x50000444)
        p[0] = 1 << chan
        while p[0]:
            sleep_us(2)

    @micropython.viper
    def pause(self):
        chan = int(self.chan)
        p = ptr32(0x50000010)        # CTRL reg alias
        p[chan<<4] &= int(0xfffffffe) # clear enable bit

    @micropython.viper
    def resume(self):
        chan = int(self.chan)
        p = ptr32(0x50000010)        # CTRL reg alias
        p[chan<<4] |= 0x01           # set enable bit
