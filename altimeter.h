//
//  altimeter.h
//  
//
//  Created by Gregory C Lewin on 11/24/17.
//

#ifndef altimeter_h
#define altimeter_h

#include <Arduino.h>

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
        
        sprintf(dataStr, "%lu,%2.2f,%2.2f,%2.2f,%2.2f",
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
};

#endif /* altimeter_h */
