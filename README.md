# Kapta2000
Arduino control of the Kapta2000 water chlorine and temperature sensor via an SPI interface.
The KAPTA™ 2000-AC2 sensor solution has been specifically developed to measure active chlorine (HOCl) and temperature directly within a pressurised pipe. It works in both 0-5V mode and 4-20mA.
## Temperature measurement
A MAX31865 RTD-To-Digital converter is used to interface the sensor to the arduino. It is optimised for platinum RTD's.
The 
## Chlorine measurement
Chlorine measurement is done via the analog pins. The brown and white wires from the sensors are connected in series to a resistor and to a battery. The voltage is read across the resistor. The value of the resistor can be calculated as follows:

`20mA × R = 5V -> R = 5V / 20mA -> R = 250Ω`. A `250Ω` resistor is not readily available, so a `220Ω` resistor will be used. This means that at the 'zero' i.e. `4mA`, a voltage of `0.88V` will be read while at 'maximum',a voltage of `4.4V` will be read. These values are within the allowable range of an Arduino Mega.
The input from the sensor will be connected to one of the analog pins of the Mega. The value read from the pin will be the result of analog to digital conversion.

The ADC step voltage is calculated as follows:

`Resolution of the ADC / System Voltage = ADC Reading / Analog Voltage Measured`

Making `Analog Voltage Measured` the subject of the formula:

`Analog Voltage = ADC Reading x System Voltage / Resolution of the ADC`
where
* ADC Reading = analog pin reading. This is the voltage due to chlorine
* System voltage = 4.4V - 0.88V
* Resolution of the ADC = 1023

From the Kapta2000 AC2 datasheet, the amount of chlorine in the water is calculated as follows:

[HOCL] = ((V<sub>HOCl</sub> – V<sub>zero</sub> without chlorine) x 1000) / S<sub>sensor</sub>
where
* [HOCl] : HOCl concentration in ppm or mg/L
* V<sub>HOCl</sub> : HOCl output voltage (V)
* V<sub>zero</sub> without chlorine : HOCl output voltage without chlorine (V)
* S<sub>sensor</sub> : Sensitivity of the chlorine sensor (mV/mgL-1)
