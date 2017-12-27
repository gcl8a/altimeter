/* 
 MPL3115A2 Barometric Pressure Sensor Library
 By: Nathan Seidle
 SparkFun Electronics
 Date: September 24th, 2013
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 
 Get pressure, altitude and temperature from the MPL3115A2 sensor.
 
 */

//2017-11-23 (GCL): removed Fahrenheit and feet

#include "Arduino.h"

#include <altimeter.h>

//namespace MPL{
    
#define MPL3115A2_ADDRESS 0x60 // Unshifted 7-bit I2C address for sensor

class MPL3115A2 : public Altimeter
{
protected:
    int interrupt1Pin = -1;
    int interrupt2Pin = -1;

    //Private Functions
    byte IIC_Read(byte regAddr);
    void IIC_Write(byte regAddr, byte value);
    
public:
    MPL3115A2(uint8_t int1Pin = -1, uint8_t int2Pin = -1)
        : interrupt1Pin(int1Pin), interrupt2Pin(int2Pin) {}

    void Init(); // Gets sensor on the I2C bus.
    
    void EnableInt1DataRdy();
    bool CheckInt1() {return digitalRead(interrupt1Pin);}
    
    bool CheckForNewDatum(AltimeterDatum& datum);
    bool CheckForNewDatum(void);

    int ReadDataAlt(); // Returns float with meters above sealevel. Ex: 1638.94

    float CalcOffset(float);
    int8_t SetOffset(int);

    float readPressure(); // Returns float with barometric pressure in Pa. Ex: 83351.25
    float readTemp(); // Returns float with current temperature in Celsius. Ex: 23.37
  
    void setModeBarometer(); // Puts the sensor into Pascal measurement mode.
    void setModeAltimeter(); // Puts the sensor into altimetery mode.
    void setModeStandby(); // Puts the sensor into Standby mode. Required when changing CTRL1 register.
    void setModeActive(); // Start taking measurements!
    void setOversampleRate(byte); // Sets the # of samples from 1 to 128. See datasheet.

    void enableEventFlags(); // Sets the fundamental event flags. Required during setup.
    void toggleOneShot();
};
//}
