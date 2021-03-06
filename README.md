ScratchMonkey
=============
This fork of ScratchMonkey adds support for using the A7 pin available on SMD packages of the ATmega328(P) and the Arduino Pro Mini (or chinese clones) as a input pin, saving the cost of a shift register. There is exactly enough pins to do HVPP programming, no more, no less.

When in this configuration, all the pins are used except for the pins used for the resonator/crystal oscillator, which are not usable in general (for Arduinos, at least). So there is NO DEBUG SERIAL. If it works, it works. Otherwise, debugging is nigh-impossible.

Unless you need the special case of using an Arduino Mini's A6/A7 pins, such as when one might be too stingy or short on board space, i strongly recommend you use Microtherion's code at [his GitHub](https://github.com/microtherion/ScratchMonkey).

For details, please consult the [Original User Manual](http://microtherion.github.com/ScratchMonkey/) Note, this is the original user manual and the additions in this fork are largely undocumented. Proceed at your own risk.

In addition, I have rewritten some parts of the code to be more platform-agnostic so that it can be easily ported to different devices. All hardware calls are handled in SMoHWIF_* files instead of being directly called.

This release is forked from the original ScratchMonkey.
The majority of the code is licensed exactly the same as the original.
It is an alpha release, and i cannot guarantee that it works as intended, or at all.

Proceed with caution.

No warranty is implied...

The following data is the wiring plan for using a Arduino Pro Mini. You will also need to uncomment a line near the bottom of SMoHWIF.h to compile.

HVPP|.
----|----
Data0:7 | D2:D9 |
HV_RST | A3
SVCC (Target Vcc) | A2
XTAL | A1
Ctrl0 | A0
Ctrl2:3 |A4:A5
Ctrl4:7 | D10:D13
Ctrl1 (AKA RDY) | A7

HVPP Notes:

Wire HV_RST to the gate of the mosfet used for pulling down the +12V. (or base of transistor through a 10k resistor)

FYI, for wiring, Ctrl0:7={BS2,RDY,OE,WR,BS1,XA0,XA1,PAGEL}, as shown on the ATmega8 programming diagram.

-------------------------------------------------------

HVSP|.
----|----
SDI | D8
SII | D9
SDO | 12
SCI | 13
SVCC | A2
HV_RST | A3
HVSP Notes:

Wire HV_RST to the gate of the mosfet used for pulling down the +12V.

-------------------------------------------------------

ISP|.
----|----
MOSI | 11
MISO | 12
SCK | 13
RST | 10

ISP Notes:

Connect the Vcc of host and target together. 

A aux 1MHz clock is provided on pin 9 for chips that might have fuse bits wrongly set to use external clock or oscillator where none is present.
