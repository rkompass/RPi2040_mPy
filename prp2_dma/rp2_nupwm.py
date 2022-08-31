# Improved micropython PWM class for RPi2040
#
# (c) 2022 by rkompass
#
# MIT License
#
# Inspired by and derived from https://github.com/danjperron/PicoAudioPWM/blob/main/myPWM.py
#

# Adds divtop() and duty() methods and div and top arguments to init() and initializer.
# Corrects a bug in deinit().
#
# Note:
# ----
#
# top+1 is the maximum effective duty value. It yields 100% constant output.
# The top argument therefore is limited to 0xfffe = 65534.
# The div argument allows 1/16 fractional values, supplied as floats: e.g. div=1.375.
# The original freq() and duty_u16() methods still work and provide sensible values.
#
# Setting div and/or top parameters changes/determines the PWM frequency.
#
#  freq = machine.freq() / (div * (top+1))
#
# Example:
# --------
#
# from machine import Pin
# from rp2_nupwm import nuPWM
# pwm = nuPWM(Pin(9), div=1.375, top=2060)
# 
# print(pwm.divtop())     # --> (1.375, 2060)
#
# print(pwm.freq())       # --> 44109   , we have constructed a frequency of ~ 44.1 kHz
#                         #               with a dynamic range of ~ 11 bits  (~2048 values)
#
# pwm.duty(1000)          # sets raw duty to 1000
#
# print(pwm.duty_u16())   # --> 31797  , 1000/2061 = 0.4852014, we have a duty of 48.52 %
#                         #     31797 = int(0.4852014 * 65535) is the duty scaled to 16 bits
#
# pwm.freq(44109)         # we set the PWM frequency to ~ 44.1 kHz by the original freq() method
#
# print(pwm.divtop())     # --> (1.625, 1727)   , the original method does not give us a suitable range.
#
# # note that duty_ns() method also works correctly

# Todo:
# -----
#
# Add phase corrected mode.
#
# Add channel-B counter and channel-B-gated timer modes.
#

from machine import PWM, mem16, mem32
from array import array

class nuPWM(PWM):
    def __init__(self, pin, freq=None, div=None, top=None):
        baddr=bytes(array('O', [pin]))
        self.pin = mem32[int.from_bytes(baddr, 'little')+4]
        self.addr = 0x40050000 + 0x14*(self.pin>>1)  # PWM_DIV=addr+4; PWM_TOP=addr+16; PWM_CC=addr+12
        super().__init__(pin)             # This alone does not work. Inheritance of builtin (C-coded) classes
        super().freq(10_000)              #   in uPy is only partially supported.
        super().duty_u16(0)               # With these instructions now inheritance of PWM seems to be o.k..
        if freq is not None or div is not None or top is not None:
            self.init(freq=freq, div=div, top=top)

    def init(self, freq=None, div=None, top=None):   # sets duty to 0 !!!!
        if freq is not None:                         # the freq has prioritiy over divider and top
            super().freq(freq)
        elif div is not None and top is not None:    # only divider and top together lead to alternative PWM setting
            self.divtop(div, top)
        else:
            raise ValueError('supply freq or both div and top arguments to init()')
        self.duty(0)

    def divtop(self, div=None, top=None):
        if div is not None:
            mem32[self.addr+4] =  round(div*16) & 0xfff  # div register holds only div value
        if top is not None:
            if top > 0xfffe:
                raise ValueError('max top value: 65534')
            mem32[self.addr+16] = top & 0xffff           # top register holds only top value
        return mem32[self.addr+4]/16, mem32[self.addr+16]

    def duty(self, value=None):
        if value is not None:
            top = mem32[self.addr+16] & 0xffff
            if value > top+1:
                value = top+1                            # duty == top + 1 -> PWM output 100% on
            if value > 0xffff:
                raise ValueError('max duty value: 65535')
            mem16[self.addr+12+((self.pin&1)<<1)] = value  # duty register holds duties for channels A and B
        return mem16[self.addr+12+((self.pin&1)<<1)]       #  -> we use mem16 to access either one

    def busy(self):
        return (mem32[0x40014004 + 0x08*self.pin] & 0b11111) == 4  # 4 for PWM, 5 for SIO, which is default for pins after reset

    def deinit(self):
        super().deinit()
        ctrl = mem32[0x40014004 + 0x08*self.pin]     # GPIOpin_CTRL    # set back to SIO control, 
        mem32[0x40014004 + 0x08*self.pin] = ctrl & 0xffffff20 | 0x05   #  this was missing in super().deinit()


    def __repr__(self):
        d, t = self.divtop()
        return 'nuPWM(machine.Pin({:d}), div={:.4f}, top={:d}) # freq={:d}'.format(self.pin, d, t, self.freq())

