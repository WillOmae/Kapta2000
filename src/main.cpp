/*
 * Kapta2000
 * main.cpp
 *
 * Created: 8/20/2019 3:52:54 PM
 * Author: WILLOMAE
 */
#include <Arduino.h>
#include <SPI.h>
// values required for temperature measurement
#define RTD_A       3.90830e-3              // Callendar-van-dusen constant A
#define RTD_B       -5.7750e-7		        // Callendar-van-dusen constant B
#define RREF        4280.0			        // Reference resistance 4xnominal
#define RNOMINAL    1000.0			        // RTD resistance at 0 degrees
#define BOARD_RES   428				        // Onboard resistance R7
#define R1          220				        // Resistance connected in parallel
#define CS_PIN      53				        // Slave select pin
#define ADC_MAX     32768			        // ADC is 15-bit, so max is 2^15
// values required for chlorine measurement
#define CL_APIN     A0                      // Analog pin to which chlorine sensor is attached
#define CL_ADC_MIN	0                       // 10-bit ADC minimum value
#define CL_ADC_MAX	1023                    // 10-bit ADC maximum value
#define RES420MA    250                     // Resistor used for 4-20mA measurement
#define VOLT_MAX    5.0                     // maximum measurable 4-20mA output voltage with RES420MA
#define VOLT_MIN    0.00                    // minimum measurable 4-20mA output voltage with RES420MA
#define V_ZERO      0.518                   // HOCL output voltage without chlorine in volts
#define SENSITIVITY 563                     // Cl sensor sensitivity in mV/mgL-1
// user defined functions
uint8_t readByteReg (uint8_t);
uint16_t readWordReg (uint8_t);
void writeConfig (void);
double calcTemp (void);
double callendarVanDussen (double);
double adafruit (float);
void dispFaults (uint8_t);
double readChlorine (void);
double readAnalogInput (void);
double calcVoltage (double);
double calcCurrent (double);
double calcChlorine (double);

SPISettings spiSettings (1000000, MSBFIRST, SPI_MODE1);
float loopCount = 0.0;

void setup()
{
    Serial.begin (115200);
    pinMode (CS_PIN, OUTPUT);
    digitalWrite (CS_PIN, HIGH);
    SPI.begin ();
    writeConfig ();
    delay (2000);
}
void loop()
{
    Serial.println (loopCount);
    double tmp = calcTemp ();
    double concCl = readChlorine();
    Serial.println ();
    loopCount += 0.01;
    delay (5000);
}
uint8_t readByteReg (uint8_t reg)
{
    uint8_t _rtd_res;
    SPI.beginTransaction (spiSettings);
    digitalWrite (CS_PIN, LOW);
    SPI.transfer (reg);
    _rtd_res = SPI.transfer (0x00);
    digitalWrite (CS_PIN, HIGH);
    SPI.endTransaction ();
    return _rtd_res;
}
uint16_t readWordReg (uint8_t reg)
{
    uint8_t _rtd_res_h, _rtd_res_l;
    SPI.beginTransaction (spiSettings);
    digitalWrite (CS_PIN, LOW);
    SPI.transfer (reg);
    _rtd_res_h = SPI.transfer (0x00);
    _rtd_res_l = SPI.transfer (0x00);
    digitalWrite (CS_PIN, HIGH);
    SPI.endTransaction ();
    return ((_rtd_res_h << 8) | _rtd_res_l) >> 1;
}
void writeConfig()
{
    SPI.beginTransaction (spiSettings);
    digitalWrite (CS_PIN, LOW);
    SPI.transfer (0x80);
    SPI.transfer (0b11000010);
    digitalWrite (CS_PIN, HIGH);
    SPI.endTransaction ();
}
double calcTemp ()
{
    uint16_t rtdReg = readWordReg (0x01);
    uint16_t hftReg = readWordReg (0x03);
    uint16_t lftReg = readWordReg (0x05);
    uint8_t statReg = readByteReg (0x07);

    if (0 == statReg)
    {
        double adc1 = rtdReg;
        Serial.print ("A1:");
        Serial.print (adc1, 3);
        double rt = adc1 * BOARD_RES / ADC_MAX;
        Serial.print ("\tRt:");
        Serial.print (rt, 3);
        Serial.print ("Ω");
        double r2 = (R1 * rt) / (R1 - rt);
        Serial.print ("\tR2:");
        Serial.print (r2, 3);
        Serial.print ("Ω");
        double adc2 = (r2 * ADC_MAX) / RREF;
        Serial.print ("\tA2:");
        Serial.print (adc2, 3);
        double tmp = callendarVanDussen (r2);
        Serial.print ("\tT:");
        Serial.print (tmp, 3);
        Serial.println ("°C");
        return tmp;
    }
    else
    {
        dispFaults (statReg);
        return -1000000.0;
    }
}
double callendarVanDussen (double rtd)
{
    double tmp = (-RTD_A + sqrt (RTD_A * RTD_A - 4 * RTD_B * (1 - (rtd / RNOMINAL)))) / (2 * RTD_B);
    return tmp >= 0 ? tmp : -1000000.0;
}
void dispFaults (uint8_t status)
{
    Serial.print ("RTD Fault, register: ");
    Serial.print (status);
    Serial.print (" ");

    if (0x80 & status) Serial.println ("RTD High Threshold Met");
    else if (0x40 & status) Serial.println ("RTD Low Threshold Met");
    else if (0x20 & status) Serial.println ("REFin- > 0.85 x Vbias");
    else if (0x10 & status) Serial.println ("FORCE- open");
    else if (0x08 & status) Serial.println ("FORCE- open");
    else if (0x04 & status) Serial.println ("Over/Under voltage fault");
    else Serial.println ("Unknown fault, check connection");
}
double readChlorine()
{
    double adc = readAnalogInput();
    double volt = calcVoltage (adc);
    double amp = calcCurrent (volt);
    double conc = calcChlorine (volt);
    Serial.print ("A:");
    Serial.print (adc, 3);
    Serial.print ("\t\tV:");
    Serial.print (volt, 3);
    Serial.print ("V");
    Serial.print ("\tZ:");
    Serial.print (V_ZERO, 3);
    Serial.print ("V");
    Serial.print ("\tA:");
    Serial.print (amp, 3);
    Serial.print ("A");
    Serial.print ("\tS:");
    Serial.print (SENSITIVITY);
    Serial.print ("mV/mgL-1");
    Serial.print ("\tC:");
    Serial.print (conc, 3);
    Serial.print ("ppm");
    return conc; // return HOCL concentration in ppm or mg/L
}
double readAnalogInput()
{
    int buf[20], temp;

    for (int i = 0; i < 20; i++)
    {
        buf[i] = analogRead (CL_APIN);
        //delay (10);
    }

    // bubble sort algorithm
    for (int i = 0; i < 19; i++)
    {
        for (int j = i + 1; j < 20; j++)
        {
            if (buf[i] > buf[j])
            {
                temp = buf[i];
                buf[i] = buf[j];
                buf[j] = temp;
            }
        }
    }

    double avgValue = 0;

    for (int i = 4; i < 16; i++)
    {
        avgValue += buf[i];
    }

    avgValue /= 12;
    return avgValue; // return the average of 10 analog readings without outliers
}
double calcVoltage (double adc)
{
    double volt = (double)adc * (VOLT_MAX - VOLT_MIN) / (CL_ADC_MAX - CL_ADC_MIN);
    int temp = volt * 100;
    volt = (double)temp / 100;
    return volt; // return HOCL output voltage in V
}
double calcCurrent (double volt)
{
    return (volt / RES420MA) * 1000; // return current in mA
}
double calcChlorine (double volt)
{
    return ((volt - V_ZERO) * 1000) / SENSITIVITY; // return chlorine concentration in ppm or mg/L
}