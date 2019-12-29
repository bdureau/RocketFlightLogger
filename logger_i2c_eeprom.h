#ifndef _LOGGER_I2C_EEPROM_H
#define _LOGGER_I2C_EEPROM_H

#include <Wire.h>
#include "config.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "Wstring.h"
#include "Wiring.h"
#endif
// TWI buffer needs max 2 bytes for eeprom address
// 1 byte for eeprom register address is available in txbuffer
#define I2C_TWIBUFFERSIZE  30
/*struct FlightConfigStruct {
  long flight1_start;    
  long flight1_stop;       
  long flight2_start;     
  long flight2_stop;  
  long flight3_start;     
  long flight3_stop; 
  long flight4_start;     
  long flight4_stop;  
  long flight5_start;     
  long flight5_stop;   
  long flight6_start;     
  long flight6_stop; 
  long flight7_start;     
  long flight7_stop;   
  long flight8_start;     
  long flight8_stop; 
  long flight9_start;     
  long flight9_stop; 
  long flight10_start;     
  long flight10_stop; 
  long flight11_start;     
  long flight11_stop; 
  long flight12_start;     
  long flight12_stop; 
  long flight13_start;     
  long flight13_stop; 
  long flight14_start;     
  long flight14_stop; 
  long flight15_start;     
  long flight15_stop; 
  
};*/

struct FlightDataStruct {
  long diffTime;
  long altitude;
  long temperature;
  
};
struct FlightConfigStruct {
  long flight_start;    
  long flight_stop; 
};

#define LOGGER_I2C_EEPROM_VERSION "1.0.0"

// The DEFAULT page size. This is overriden if you use the second constructor.
// I2C_EEPROM_PAGESIZE must be multiple of 2 e.g. 16, 32 or 64
// 24LC256 -> 64 bytes
#define LOGGER_I2C_EEPROM_PAGESIZE 64
#define FLIGHT_LIST_START 0
#define FLIGHT_DATA_START 200
class logger_I2C_eeprom
{
public:
    /**
     * Initializes the EEPROM with a default pagesize of I2C_EEPROM_PAGESIZE.
     */
    logger_I2C_eeprom(uint8_t deviceAddress);
    logger_I2C_eeprom(uint8_t deviceAddress, const unsigned int deviceSize);
    uint8_t _deviceAddress;
    void begin();
    void clearFlightList();
    void write_byte( unsigned int eeaddress, uint8_t data );
    uint8_t read_byte(  unsigned int eeaddress );
    int readFlight(int eeaddress);
    int writeFlight(int eeaddress);
    int readFlightList();
    int writeFlightList();
    int getLastFlightNbr();
    int printFlightList();
    void setFlightStartAddress(int flightNbr, long startAddress);
    void setFlightEndAddress(int flightNbr, long endAddress);
    void setFlightTimeData( long difftime);
    long getFlightTimeData();
    void setFlightAltitudeData( long altitude);
    void setFlightTemperatureData (long temperature);
    long getFlightAltitudeData();
    long getFlightStart(int flightNbr);
    long getFlightStop(int flightNbr);
    void PrintFlight(int flightNbr);
    void printFlightData(int flightNbr);
    boolean CanRecord();
    int writeFastFlight(uint16_t eeaddress);
    
private:
    
    FlightConfigStruct _FlightConfig[25];
    FlightDataStruct _FlightData;
    uint8_t _pageSize;
};

#endif
// END OF FILE
