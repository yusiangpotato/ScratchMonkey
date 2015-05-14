ScratchMonkey
=============
This fork of ScratchMonkey adds support for using the A6/A7 pin available on SMD packages of the ATmega328(P) and the Arduino Mini (or chinese clones) as a input pin, saving the cost of a shift register. There is exactly enough pins to do programming, no more, no less.

When in this configuration, all the pins are used except for the XTAL pins, which are not usable in general (for Arduino Mini, at least). So there is NO DEBUG SERIAL.

THIS FORK OF SCRATCHMONKEY IS UNMAINTAINED! Unless you need the special case of using an Arduino Mini's A6/A7 pins, i strongly recommend you use Microtherion's code at [his GitHub](https://github.com/microtherion/ScratchMonkey).

For details, please consult the [User Manual](http://microtherion.github.com/ScratchMonkey/) Note, this is the original user manual and the additions in this fork are largely undocumented. Proceed at your own risk.


This release is forked from the original ScratchMonkey.
The majority of the code is licensed exactly the same as the original.
It is an alpha release, and i cannot guarantee that it works as intended, or at all.

Proceed with caution.

No warranty is implied...
