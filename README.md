# ADS1115_InstESRE_Pyranometer
Documentation and test for an Arduino/ADS1115 version of the InstESRE pyranometer kit

This repository contains documentation and a test program for a modified version 
of the pyranometer kit offered by Dr. David Brooks of the Institute for Earth Science
Research and Education (InstESRE). An Arduino and ADS1115 analog-to-digital-converter
are used to measure the photodiode output and translate that to an irradiance value.
There is also support for a TMP36 temperature sensor co-located with the photodiode,
allowing the software to compensate for the effect of temperature on the photodiode
output.

The IV Swinger 2 IV curve tracer (csatt/IV_Swinger) supports this design as an optional
sensor, and that was the motivation for its design. However, since other users of the
InstESRE pyranometer may find it useful, this repository describes the design 
independently from the IV Swinger 2 project. The included Arduino sketch can be
used to test (and calibrate) the hardware. It can also be used as a starting point
for other projects (data loggers, etc.)
