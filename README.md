# Kapta2000
Arduino control of the Kapta2000 water chlorine and temperature sensor via an SPI interface.
The KAPTA™ 2000-AC2 sensor solution has been specifically developed to measure active chlorine (HOCl) and temperature directly within a pressurised pipe. It works in both 0-5V mode and 4-20mA.
## Temperature measurement
A MAX31865 RTD-To-Digital converter is used to interface the sensor to the arduino. It is optimised for platinum RTD's.
The 
## Chlorine measurement
Chlorine measurement is done via the analog pins. The brown and white wires from the sensors are connected in series to a resistor and to a battery. The voltage is read across the resistor. The value of the resistor can be calculated as follows:
`
