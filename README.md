# Distreon INA232 Arduino Library

An Arduino library for interfacing with the INA232 Power Monitor IC over I2C.

The INA232 is an ADC chip made by TI is specifically designed to provide precision (16-bit) high-side current shunt measurements along with measurement of the main supply voltage. It is similar to the INA219 used on popular breakout boards, but can handle a higher bus voltage, and has a higher precision ADC.

2 examples are provided, both showing how to set the library up and one to test the speed of different conversion modes and timings.

Not all features of the INA232 are supported yet - in particular setting any of the Alert pin thresholds or settings.
