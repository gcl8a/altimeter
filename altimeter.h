//
//  altimeter.h
//  
//
//  Created by Gregory C Lewin on 11/24/17.
//

#ifndef altimeter_h
#define altimeter_h

#include <Arduino.h>

struct AltimeterDump
{
    //uint32_t timestamp = 0;
    uint32_t mPascal = 0; //milliPascal
    int16_t temperature; // hundreths of C
    uint16_t humidity; //hundreths of %
};

struct AltimeterDatum
{
    uint32_t timestamp = 0;
    
    float pressure = -99;
    float altitude = -99;
    float temperature = -99;
    float humidity = -99;

public:
    String MakeDataString(void)
    {
        char dataStr[100];
        
        sprintf(dataStr, "%lu,%2.1f,%2.1f,%2.1f,%2.1f",
                timestamp%1000,
                pressure,
                altitude,
                temperature,
                humidity);

        return String(dataStr);
    }
    
    String MakeShortDataString(void)
    {
        char dataStr[100];
        sprintf(dataStr, "%2.2f,%2.2f",
                altitude,
                temperature);
        
        return String(dataStr);
    }
    
    AltimeterDump MakeDataDump(void)
    {
        AltimeterDump dump;
        
        dump.mPascal = pressure * 1000;
        dump.temperature = temperature * 100;
        dump.humidity = humidity * 100;
        
        return dump;
    }

    String ParseDataDump(const AltimeterDump& dump)
    {
        pressure = dump.mPascal / 1000.0;
        temperature = dump.temperature / 100.0;
        humidity = dump.humidity / 100.0;
        
        return MakeDataString();
    }
};

class Altimeter
{
protected:
    AltimeterDatum workingDatum; //someday make a FIFO?
        
public:
    String MakeDataString(void)
    {
        return workingDatum.MakeDataString();
    }
    
    String MakeShortDataString(void)
    {
        return workingDatum.MakeShortDataString();
    }
    
    AltimeterDump MakeDataDump(void) { return workingDatum.MakeDataDump();}
};

#endif /* altimeter_h */
