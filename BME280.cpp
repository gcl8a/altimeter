/******************************************************************************
SparkFunBME280.cpp
BME280 Arduino and Teensy Driver
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/BME280_Breakout

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/
//See SparkFunBME280.h for additional topology notes.

#include "BME280.h"
#include "stdint.h"
#include <math.h>

#include "Wire.h"
#include "SPI.h"

//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "mySensor.settings.commInterface = SPI_MODE;" to 
//  configure before calling .begin();
//
//****************************************************************************//
uint8_t BME280::Init(const BME280Settings& s)
{
    settings = s;
    
	//Check the settings structure values to determine how to setup the device
	uint8_t dataToWrite = 0;  //Temporary variable

    Wire.begin();
	
	//Reading all compensation data, range 0x88:A1, 0xE1:E7
	
	calibration.dig_T1 = ((uint16_t)((readRegister(BME280_DIG_T1_MSB_REG) << 8) + readRegister(BME280_DIG_T1_LSB_REG)));
	calibration.dig_T2 = ((int16_t)((readRegister(BME280_DIG_T2_MSB_REG) << 8) + readRegister(BME280_DIG_T2_LSB_REG)));
	calibration.dig_T3 = ((int16_t)((readRegister(BME280_DIG_T3_MSB_REG) << 8) + readRegister(BME280_DIG_T3_LSB_REG)));

	calibration.dig_P1 = ((uint16_t)((readRegister(BME280_DIG_P1_MSB_REG) << 8) + readRegister(BME280_DIG_P1_LSB_REG)));
	calibration.dig_P2 = ((int16_t)((readRegister(BME280_DIG_P2_MSB_REG) << 8) + readRegister(BME280_DIG_P2_LSB_REG)));
	calibration.dig_P3 = ((int16_t)((readRegister(BME280_DIG_P3_MSB_REG) << 8) + readRegister(BME280_DIG_P3_LSB_REG)));
	calibration.dig_P4 = ((int16_t)((readRegister(BME280_DIG_P4_MSB_REG) << 8) + readRegister(BME280_DIG_P4_LSB_REG)));
	calibration.dig_P5 = ((int16_t)((readRegister(BME280_DIG_P5_MSB_REG) << 8) + readRegister(BME280_DIG_P5_LSB_REG)));
	calibration.dig_P6 = ((int16_t)((readRegister(BME280_DIG_P6_MSB_REG) << 8) + readRegister(BME280_DIG_P6_LSB_REG)));
	calibration.dig_P7 = ((int16_t)((readRegister(BME280_DIG_P7_MSB_REG) << 8) + readRegister(BME280_DIG_P7_LSB_REG)));
	calibration.dig_P8 = ((int16_t)((readRegister(BME280_DIG_P8_MSB_REG) << 8) + readRegister(BME280_DIG_P8_LSB_REG)));
	calibration.dig_P9 = ((int16_t)((readRegister(BME280_DIG_P9_MSB_REG) << 8) + readRegister(BME280_DIG_P9_LSB_REG)));

	calibration.dig_H1 = ((uint8_t)(readRegister(BME280_DIG_H1_REG)));
	calibration.dig_H2 = ((int16_t)((readRegister(BME280_DIG_H2_MSB_REG) << 8) + readRegister(BME280_DIG_H2_LSB_REG)));
	calibration.dig_H3 = ((uint8_t)(readRegister(BME280_DIG_H3_REG)));
	calibration.dig_H4 = ((int16_t)((readRegister(BME280_DIG_H4_MSB_REG) << 4) + (readRegister(BME280_DIG_H4_LSB_REG) & 0x0F)));
	calibration.dig_H5 = ((int16_t)((readRegister(BME280_DIG_H5_MSB_REG) << 4) + ((readRegister(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
	calibration.dig_H6 = ((uint8_t)readRegister(BME280_DIG_H6_REG));

	//Set the oversampling control words.
	//config will only be writeable in sleep mode, so first insure that.
	writeRegister(BME280_CTRL_MEAS_REG, 0x00);
	
	//Set the config word
	dataToWrite = (settings.tStandby << 0x5) & 0xE0;
	dataToWrite |= (settings.filter << 0x02) & 0x1C;
	writeRegister(BME280_CONFIG_REG, dataToWrite);
	
	//Set ctrl_hum first, then ctrl_meas to activate ctrl_hum
	dataToWrite = settings.humidOverSample & 0x07; //all other bits can be ignored
	writeRegister(BME280_CTRL_HUMIDITY_REG, dataToWrite);
	
	//set ctrl_meas
	//First, set temp oversampling
	dataToWrite = (settings.tempOverSample << 0x5) & 0xE0;
	//Next, pressure oversampling
	dataToWrite |= (settings.pressOverSample << 0x02) & 0x1C;
	//Last, set mode
	dataToWrite |= (settings.runMode) & 0x03;
	//Load the byte
	writeRegister(BME280_CTRL_MEAS_REG, dataToWrite);
    
    ForceReading();
	
	return readRegister(0xD0);
}

void BME280::ForceReading(void)
{
    //set ctrl_meas
    //First, set temp oversampling
    uint8_t dataToWrite = (settings.tempOverSample << 0x5) & 0xE0;
    //Next, pressure oversampling
    dataToWrite |= (settings.pressOverSample << 0x02) & 0x1C;
    //Last, set mode
    dataToWrite |= 0x01; //(settings.runMode) & 0x03;
    //Load the byte
    writeRegister(BME280_CTRL_MEAS_REG, dataToWrite);
    state = MEASURING;
}

//Strictly resets.  Run .begin() afterwards
void BME280::reset( void )
{
	writeRegister(BME280_RST_REG, 0xB6);
}

uint8_t BME280::ReadStatus(void)
{
    return readRegister(BME280_STAT_REG);
}

uint8_t BME280::IsMeasuring(void)
{
    return ReadStatus() & 0x08;
}

bool BME280::CheckForNewDatum(void)
{
    bool retVal = false;
    if(state == MEASURING && !IsMeasuring()) //current measurement is complete
    {
        workingDatum.timestamp = millis();
        state = STANDBY;
        retVal = true;
    }
    
    return retVal;
}

/*
 * Function to read all the data registers at once. Not only is this faster than the poorly
 * thought out method in the original code, but it's also recommended by the data sheet,
 * since reading a byte at a time leads to inconsistent readings.
 */

bool BME280::ReadDatum(void)
{
    Wire.beginTransmission(settings.I2CAddress);
    Wire.write(0xf7);
    Wire.endTransmission();
    
    // request bytes from sensor
    uint8_t data[8];
    Wire.requestFrom(settings.I2CAddress, 8);
    uint8_t i = 0;
    while (Wire.available() && (i < 8))  // slave may send more than requested
    {
        data[i++] = Wire.read(); // receive a byte as character
    }

    //do temperature first to update t_fine, a carryover adjustment parameter
    int32_t t_fine = 0;

    int32_t adc_T = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((data[5] >> 4) & 0x0F);
    workingDatum.temperature = CalcTemperature(adc_T, t_fine);
    
    int32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((data[2] >> 4) & 0x0F);
    workingDatum.pressure = CalcPressure(adc_P, t_fine);
    workingDatum.altitude = (-45846.2)*(pow((workingDatum.pressure/101325.0), 0.190263) - 1.0);

    int32_t adc_H = ((uint32_t)data[6] << 8) | ((uint32_t)data[7]);
    workingDatum.humidity = CalcHumidity(adc_H, t_fine);
    
    return 1;
}

float BME280::CalcPressure( int32_t adc_P, int32_t t_fine )
{
	// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
	
	int64_t var1, var2, p_acc;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calibration.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calibration.dig_P5)<<17);
	var2 = var2 + (((int64_t)calibration.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)calibration.dig_P3)>>8) + ((var1 * (int64_t)calibration.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calibration.dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p_acc = (int32_t)1048576 - adc_P;
	p_acc = (((p_acc<<31) - var2)*3125)/var1;
	var1 = (((int64_t)calibration.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
	var2 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;
	p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7)<<4);
	
	return p_acc / 256.0;
}

float BME280::CalcAltitude( float pressure )
{
	return (-45846.2)*(pow((pressure/101325.0), 0.190263) - 1.0);
}

float BME280::CalcHumidity( int32_t adc_H, int32_t t_fine )
{
	// Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of “47445” represents 47445/1024 = 46. 333 %RH
    
	int32_t var1;
	var1 = (t_fine - ((int32_t)76800));
	var1 = (((((adc_H << 14) - (((int32_t)calibration.dig_H4) << 20) - (((int32_t)calibration.dig_H5) * var1)) +
	((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)calibration.dig_H6)) >> 10) * (((var1 * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)calibration.dig_H2) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	return (float)(var1>>12) / 1024.0;
}

float BME280::CalcTemperature( int32_t adc_T, int32_t& t_fine ) //C
{
	// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
	// t_fine is passed by reference since it is needed in calculations for pressure and humidity

	int64_t var1, var2;

	var1 = ((((adc_T>>3) - ((int32_t)calibration.dig_T1<<1))) * ((int32_t)calibration.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)calibration.dig_T1)) * ((adc_T>>4) - ((int32_t)calibration.dig_T1))) >> 12) *
	((int32_t)calibration.dig_T3)) >> 14;
	t_fine = var1 + var2;
	float output = (t_fine * 5 + 128) >> 8;

	output = output / 100.0;
	
	return output;
}

//****************************************************************************//
//
//  Utility
//
//****************************************************************************//

uint8_t BME280::readRegister(const uint8_t& offset)
{
    uint8_t result = 0;
    
    Wire.beginTransmission(settings.I2CAddress);
    Wire.write(offset);
    Wire.endTransmission();

    Wire.requestFrom(settings.I2CAddress, 1);
    while ( Wire.available() ) // slave may send less than requested
    {
        result = Wire.read(); // receive a byte as a proper uint8_t
    }

	return result;
}

void BME280::writeRegister(const uint8_t& offset, const uint8_t& dataToWrite)
{
    //Write the byte
    Wire.beginTransmission(settings.I2CAddress);
    Wire.write(offset);
    Wire.write(dataToWrite);
    Wire.endTransmission();
}
