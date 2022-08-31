# Test 5 of RP2_Dma micropython class
#
# In this test we use a slightly modified PWM class, nuPWM to output PWM
#   with divider=1.375 and top=2060.
#
# This leads to a PWM output rate of 44109 Hz at a resolution of 11 bits.
# 

#
# In this test we use the PIO to output sound via PWM,
#

from machine import Pin, Timer
from rp2_nupwm import nuPWM
from rp2_dma import RP2_Dma, TREQ_Timer0, DMA_Cycle_Src
from array import array
from time import sleep
from math import sin, pi

# Setup Pin 14 for PWM. Connect Pin 14 via a resistor (~470 Ohms) to an amplifier.
# Put a capacitor (0.1-0.2 uF) between GND and amplifier input, to reduce the PWM 44kHz carrier. 
pwm = nuPWM(Pin(14), div=1.375, top=2060) #  We construct a frequency of ~ 44.1 kHz, with dynamic range ~ 11 bits.

#  Set up the array for the waveform with 44.109 kHz sampling rate for 0.449 seconds: 19800 (=99*200) 16-bit samples

frequency = 220   # the frequency we want
n = round(pwm.freq() / frequency)   # this is a crude approximation.
                                    # for higher frequencies we should set up the snd_arr array to contain multiple periods of the sine
print('Sine wave of {:.2f} Hz with {:d} sampling points at {:.2f} Hz sample rate'.format(pwm.freq()/n, n, pwm.freq()))
snd_arr = array('H', (round((sin(2*pi*i/n)+1.0)*1029) for i in range(n)))   # sine wave, single period; see above comment
                                       
# Set up dma of snd_arr to pwm counter compare address; dma is paced by dma timer 0
# A master dma to reconfigure the dma (rewrite the address of the next source array cyclically and trigger) 
#   is enforced by the argument cycle=DMA_Cycle_Src.

# dma = RP2_Dma((snd_arr, snd_arr), pwm, cycle=DMA_Cycle_Src, dreq=TREQ_Timer0)   # we may use a tuple/list of arrays as source
dma = RP2_Dma(snd_arr, pwm, cycle=DMA_Cycle_Src, dreq=TREQ_Timer0)     # but can also use a single array as source; synchonize with dma timer 0
dma.timer(0, freq=44109)                                               # Set dma timer 0 to 40109 Hz.
dma.start()

Timer(period=8000, mode=Timer.ONE_SHOT, callback=lambda t:dma.deinit())  # stop the dma automatically after 9 seconds

for _ in range(12):
    print('dma.busy():', dma.busy(), '    pwm.busy():', pwm.busy())
    sleep(1)
pwm.deinit()
print('dma.busy():', dma.busy(), '    pwm.busy():', pwm.busy())