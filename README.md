# README

The NUCLEO-F411RE board can produce decent audio signal through PWM with a
maximum resolution of 11-bits.

To save space on the flash memory, conversion from 16-bit signed PCM to 11-bit
unsigned PCM audio is done while building, outside the microcontroller.
