/* 
 MPL3115A2 Barometric Pressure Sensor Library
 By: Nathan Seidle
 SparkFun Electronics
 Date: September 24th, 2013
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 
 Get pressure, altitude and temperature from the MPL3115A2 sensor.
 
 */

//2017-11-23 (GCL): removed Fahrenheit and feet

#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>
#include <altimeter.h>

#define MPL3115A2_ADDRESS 0x60 // Unshifted 7-bit I2C address for sensor

#define STATUS     0x00
#define OUT_P_MSB  0x01
#define OUT_P_CSB  0x02
#define OUT_P_LSB  0x03
#define OUT_T_MSB  0x04
#define OUT_T_LSB  0x05
#define DR_STATUS  0x06
#define OUT_P_DELTA_MSB  0x07
#define OUT_P_DELTA_CSB  0x08
#define OUT_P_DELTA_LSB  0x09
#define OUT_T_DELTA_MSB  0x0A
#define OUT_T_DELTA_LSB  0x0B
#define WHO_AM_I   0x0C
#define F_STATUS   0x0D
#define F_DATA     0x0E
#define F_SETUP    0x0F
#define TIME_DLY   0x10
#define SYSMOD     0x11
#define INT_SOURCE 0x12
#define PT_DATA_CFG 0x13
#define BAR_IN_MSB 0x14
#define BAR_IN_LSB 0x15
#define P_TGT_MSB  0x16
#define P_TGT_LSB  0x17
#define T_TGT      0x18
#define P_WND_MSB  0x19
#define P_WND_LSB  0x1A
#define T_WND      0x1B
#define P_MIN_MSB  0x1C
#define P_MIN_CSB  0x1D
#define P_MIN_LSB  0x1E
#define T_MIN_MSB  0x1F
#define T_MIN_LSB  0x20
#define P_MAX_MSB  0x21
#define P_MAX_CSB  0x22
#define P_MAX_LSB  0x23
#define T_MAX_MSB  0x24
#define T_MAX_LSB  0x25
#define CTRL_REG1  0x26
#define CTRL_REG2  0x27
#define CTRL_REG3  0x28
#define CTRL_REG4  0x29
#define CTRL_REG5  0x2A
#define OFF_P      0x2B
#define OFF_T      0x2C
#define OFF_H      0x2D

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

  //Public Variables

};
