/*
 MPL3115A2 Barometric Pressure Sensor Library
 By: Nathan Seidle
 SparkFun Electronics
 Date: September 22nd, 2013
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

 This library allows an Arduino to read from the MPL3115A2 low-cost high-precision pressure sensor.
 
 If you have feature suggestions or need support please use the github support page: https://github.com/sparkfun/MPL3115A2_Breakout

 Hardware Setup: The MPL3115A2 lives on the I2C bus. Attach the SDA pin to A4, SCL to A5. Use inline 10k resistors
 if you have a 5V board. If you are using the SparkFun breakout board you *do not* need 4.7k pull-up resistors 
 on the bus (they are built-in).
 
 Link to the breakout board product:
 
 Software:
 .begin() Gets sensor on the I2C bus.
 .readAltitude() Returns float with meters above sealevel. Ex: 1638.94
 .readAltitudeFt() Returns float with feet above sealevel. Ex: 5376.68
 .readPressure() Returns float with barometric pressure in Pa. Ex: 83351.25
 .readTemp() Returns float with current temperature in Celsius. Ex: 23.37
 .readTempF() Returns float with current temperature in Fahrenheit. Ex: 73.96
 .setModeBarometer() Puts the sensor into Pascal measurement mode.
 .setModeAltimeter() Puts the sensor into altimetery mode.
 .setModeStandy() Puts the sensor into Standby mode. Required when changing CTRL1 register.
 .setModeActive() Start taking measurements!
 .setOversampleRate(byte) Sets the # of samples from 1 to 128. See datasheet.
 .enableEventFlags() Sets the fundamental event flags. Required during setup.
 
 */

#include <Wire.h>

#include "MPL3115A2.h"

//Begin
/*******************************************************************************************/
//Start I2C communication
void MPL3115A2::Init(void)
{
  Wire.begin();
    
    //Configure the sensor
    setModeAltimeter(); // Measure altitude above sea level in meters
    //myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
    
    setOversampleRate(7); // Set Oversample to the recommended 128
    enableEventFlags(); // Enable all three pressure and temp event flags
    EnableInt1DataRdy(); //enable interrupts
}

//Returns the number of meters above sea level
//Returns -1 if no new data is available
int MPL3115A2::ReadDataAlt(void)
{
    // Read pressure registers
	Wire.beginTransmission(MPL3115A2_ADDRESS);
	Wire.write(OUT_P_MSB);  // Address of data to get
	Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start
	if (Wire.requestFrom(MPL3115A2_ADDRESS, 5) != 5)
    {
		return 0;
	}

	byte msb, csb, lsb;
	msb = Wire.read();
	csb = Wire.read();
	lsb = Wire.read();

	// The least significant bytes l_altitude and l_temp are 4-bit,
	// fractional values, so you must cast the calulation in (float),
	// shift the value over 4 spots to the right and divide by 16 (since 
	// there are 16 values in 4-bits). 
	float tempcsb = (lsb>>4)/16.0;
	workingDatum.altitude = (float)( (msb << 8) | csb) + tempcsb;
    
    msb = Wire.read();
    lsb = Wire.read();

    //Negative temperature fix by D.D.G.
    word foo = 0;
    bool negSign = false;
    
    //Check for 2s compliment
    if(msb > 0x7F)
    {
        foo = ~((msb << 8) + lsb) + 1;  //2’s complement
        msb = foo >> 8;
        lsb = foo & 0x00F0;
        negSign = true;
    }
    
    // The least significant bytes l_altitude and l_temp are 4-bit,
    // fractional values, so you must cast the calulation in (float),
    // shift the value over 4 spots to the right and divide by 16 (since
    // there are 16 values in 4-bits).
    float templsb = (lsb>>4)/16.0; //temp, fraction of a degree
    
    float temperature = (float)(msb + templsb);
    if (negSign) temperature = 0 - temperature;
    
    workingDatum.temperature = temperature;
    
    workingDatum.timestamp = millis();
    
    return 1;
}

int8_t MPL3115A2::SetOffset(int offset)
{
    setModeStandby();
    
    //-128 <= offset <= 127
    if(offset < -128) offset = -128;
    if(offset >  127) offset =  127;
    
    IIC_Write(OFF_H, (int8_t)offset);
    
    return IIC_Read(OFF_H);
}

float MPL3115A2::CalcOffset(float currAlt)
{
    //We need to clear the data ready register, but there might be a pending request
    //so we'll go through the whole cycle once
    toggleOneShot();
    while(!CheckInt1()) {} //wait for new data
    ReadDataAlt(); //clears interrupt

    setModeStandby();
    IIC_Write(OFF_H, 0); //clear offset

    //Now take a fresh reading
    toggleOneShot();
    while(!CheckInt1()) {} //wait for new data
    ReadDataAlt();

    //calculate the offset and write it
    float fOffset = currAlt - workingDatum.altitude;

    return fOffset;
}

//
//Reads the current pressure in Pa
//Unit must be set in barometric pressure mode
//Returns -1 if no new data is available
float MPL3115A2::readPressure()
{
	//Check PDR bit, if it's not set then toggle OST
	if((IIC_Read(STATUS) & (1<<2)) == 0) toggleOneShot(); //Toggle the OST bit causing the sensor to immediately take another reading

	//Wait for PDR bit, indicates we have new pressure data
	int counter = 0;
	while((IIC_Read(STATUS) & (1<<2)) == 0)
	{
		if(++counter > 600) return(-999); //Error out after max of 512ms for a read
		delay(1);
	}

	// Read pressure registers
	Wire.beginTransmission(MPL3115A2_ADDRESS);
	Wire.write(OUT_P_MSB);  // Address of data to get
	Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start. THIS IS NECESSARY and not supported before Arduino V1.0.1!
	if (Wire.requestFrom(MPL3115A2_ADDRESS, 3) != 3) { // Request three bytes
		return -999;
	}

	byte msb, csb, lsb;
	msb = Wire.read();
	csb = Wire.read();
	lsb = Wire.read();
	
	//toggleOneShot(); //Toggle the OST bit causing the sensor to immediately take another reading

	// Pressure comes back as a left shifted 20 bit number
	long pressure_whole = (long)msb<<16 | (long)csb<<8 | (long)lsb;
	pressure_whole >>= 6; //Pressure is an 18 bit number with 2 bits of decimal. Get rid of decimal portion.

	lsb &= B00110000; //Bits 5/4 represent the fractional component
	lsb >>= 4; //Get it right aligned
	float pressure_decimal = (float)lsb/4.0; //Turn it into fraction

	float pressure = (float)pressure_whole + pressure_decimal;

	return(pressure);
}

//Sets the mode to Barometer
//CTRL_REG1, ALT bit
void MPL3115A2::setModeBarometer()
{
  byte tempSetting = IIC_Read(CTRL_REG1); //Read current settings
  tempSetting &= ~(1<<7); //Clear ALT bit
  IIC_Write(CTRL_REG1, tempSetting);
}

//Sets the mode to Altimeter
//CTRL_REG1, ALT bit
void MPL3115A2::setModeAltimeter()
{
  byte tempSetting = IIC_Read(CTRL_REG1); //Read current settings
  tempSetting |= (1<<7); //Set ALT bit
  IIC_Write(CTRL_REG1, tempSetting);
}

//Puts the sensor in standby mode
//This is needed so that we can modify the major control registers
void MPL3115A2::setModeStandby()
{
  byte tempSetting = IIC_Read(CTRL_REG1); //Read current settings
  tempSetting &= ~(1<<0); //Clear SBYB bit for Standby mode
  IIC_Write(CTRL_REG1, tempSetting);
}

//Puts the sensor in active mode
//This is needed so that we can modify the major control registers
void MPL3115A2::setModeActive()
{
  byte tempSetting = IIC_Read(CTRL_REG1); //Read current settings
  tempSetting |= (1<<0); //Set SBYB bit for Active mode
  IIC_Write(CTRL_REG1, tempSetting);
}

//Call with a rate from 0 to 7. See page 33 for table of ratios.
//Sets the over sample rate. Datasheet calls for 128 but you can set it 
//from 1 to 128 samples. The higher the oversample rate the greater
//the time between data samples.
void MPL3115A2::setOversampleRate(byte sampleRate)
{
  if(sampleRate > 7) sampleRate = 7; //OS cannot be larger than 0b.0111
  sampleRate <<= 3; //Align it for the CTRL_REG1 register
  
  byte tempSetting = IIC_Read(CTRL_REG1); //Read current settings
  tempSetting &= B11000111; //Clear out old OS bits
  tempSetting |= sampleRate; //Mask in new OS bits
  IIC_Write(CTRL_REG1, tempSetting);
}

//Enables the pressure and temp measurement event flags so that we can
//test against them. This is recommended in datasheet during setup.
void MPL3115A2::enableEventFlags()
{
  IIC_Write(PT_DATA_CFG, 0x07); // Enable all three pressure and temp event flags 
}

void MPL3115A2::EnableInt1DataRdy()
{
    IIC_Write(CTRL_REG3, 0x20); // interrupt active HIGH (default is active LOW)
    IIC_Write(CTRL_REG4, 0x80); // enable data ready interrupt
    IIC_Write(CTRL_REG5, 0x80); // route to INT1 (default is INT2)
}

//Clears then sets the OST bit which causes the sensor to immediately take another reading
//Needed to sample faster than 1Hz
void MPL3115A2::toggleOneShot(void)
{
    byte tempSetting = IIC_Read(CTRL_REG1); //Read current settings
    tempSetting &= ~(1<<1); //Clear OST bit
    IIC_Write(CTRL_REG1, tempSetting);

    tempSetting = IIC_Read(CTRL_REG1); //Read current settings to be safe
    tempSetting |= (1<<1); //Set OST bit
    IIC_Write(CTRL_REG1, tempSetting);
}

// These are the two I2C functions in this sketch.
byte MPL3115A2::IIC_Read(byte regAddr)
{
  // This function reads one byte over IIC
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(regAddr);  // Address of CTRL_REG1
  Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start. THIS IS NECESSARY and not supported before Arduino V1.0.1!
  Wire.requestFrom(MPL3115A2_ADDRESS, 1); // Request the data...
  return Wire.read();
}

void MPL3115A2::IIC_Write(byte regAddr, byte value)
{
  // This function writes one byto over IIC
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(regAddr);
  Wire.write(value);
  Wire.endTransmission(true);
}

