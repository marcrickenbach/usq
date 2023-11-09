USQ Console

PC-1

The first program for USQ is an 'unclocked' two-channel sequencer.

There are two sequencers, a high and low, of 8 steps each. Each step has an LED
button and two potentiometers: the LED button gives visual feedback on the
active state of the step, while the two pots determine voltage and time delay
from the end of the previous gate.

External triggers start, pause and reset each sequencer channel. When a start
command is detected, a hardware timer is set with a period determined by the
Time potentiometer. When that timer is up, we send the sequencer channel gate
high (GPIO), send the current voltage information to the DAC peripheral for a
control voltage output, and write to the LED driver over SPI.

Previous iteration demo:
https://www.instagram.com/reel/CuARqAMAVtb/?utm_source=ig_web_copy_link&igshid=MzRlODBiNWFlZA==
