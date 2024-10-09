# quadrature_encoder_d2_d3_interrupts_arduino_uno_r4
Decode a quadrature signal at high speed, several thousand pulses per rotation at several hundred rpm.
Board-specific code for Arduino Uno R4 Wifi, using interrupts on D2 and D3 and port register reading and writing.

rationale: managed to get to about 600 rpm @ 4096 interrupts/rotation using an old Uno (Atmel 168). Above that it would start missing pulses.
Based the first versions on Paul Stoffregen's Encoder.h but with added modulus (pulses/rotation) and a signed char LUT to replace some assembly.

The extremely stripped-down AVR specific port code would not compile for the newer Uno R4, so first made it work again using the high-level
digitalRead and digitalWrite. This seemed to be on a par with the old Uno low level port code.

Next step was to make it go faster. Finding practical Arduino Uno R4 bitbanging info was a lot harder than the AVR stuff and hopefully this is the correct way to do it.
Important note when choosing the physical pins for your circuit: the Uno R4 does not have the same hardware pins grouped into the same bits in the same contiguous registers.
E.g. what used to be a one-byte write in PORTD for D0-D7 is now split into different registers. Check the pinout pictures for the R4 and compare that to the register and bitmask you get when you query them in code.
You must use the api to get them, simply moving the bitmask 5 bits up from D2 to get D7 for example will not work.
