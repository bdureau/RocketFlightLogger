#include "logger_i2c_eeprom.h"
#include "IC2extEEPROM.h"
extEEPROM eep(kbits_512, 1, 64);
logger_I2C_eeprom::logger_I2C_eeprom(uint8_t deviceAddress)
{
  //_deviceAddress = deviceAddress;
  //logger_I2C_eeprom(deviceAddress, LOGGER_I2C_EEPROM_PAGESIZE);
}

void logger_I2C_eeprom::begin()
{
  Wire.begin();
  //initialize Flight structure

}
/*
   clearFlightList()
   Clear the flight list. Rather than clearing the entire eeprom
   let's just reset addresses 0 to 200 which contains the flights addresses

*/
void logger_I2C_eeprom::clearFlightList()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    _FlightConfig[i].flight_start = 0;
    _FlightConfig[i].flight_stop = 0;
  }
}


/*
   readFlightList()

*/
int logger_I2C_eeprom::readFlightList()
{
  eep.read(0, ((byte*)&_FlightConfig), sizeof(_FlightConfig));
  return FLIGHT_LIST_START + sizeof(_FlightConfig) ;
}
/*
   readFlight(int eeaddress)

*/
unsigned long logger_I2C_eeprom::readFlight(unsigned long eeaddress)
{
  eep.read(eeaddress, ((byte*)&_FlightData), sizeof(_FlightData));
  return eeaddress + sizeof(_FlightData);
}

/*
   writeFlightList()

*/
int logger_I2C_eeprom::writeFlightList()
{
  eep.write(FLIGHT_LIST_START, ((byte*)&_FlightConfig), sizeof(_FlightConfig));
  return FLIGHT_LIST_START + sizeof(_FlightConfig);
}

/*
   writeFastFlight(int eeaddress)

*/
unsigned long logger_I2C_eeprom::writeFastFlight(unsigned long eeaddress)
{
  eep.write(eeaddress, ((byte*)&_FlightData), sizeof(_FlightData));
  return eeaddress + sizeof(_FlightData);
}

/*

   getLastFlightNbr()
   Parse the flight index end check if the flight_start address is > 0
   return -1 if no flight have been recorded else return the flight number

*/
int logger_I2C_eeprom::getLastFlightNbr()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_FlightConfig[i].flight_start == 0)
    {
      break;
    }
  }
  i--;
  return i;
}
/*

  eraseLastFlight()

*/
bool logger_I2C_eeprom::eraseLastFlight() {
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_FlightConfig[i].flight_start == 0)
    {
      if (i > 0) {
        _FlightConfig[i - 1].flight_start = 0;
        _FlightConfig[i - 1].flight_stop = 0;
        writeFlightList();
        return true;
      }
    }
  }
  return false;
}
/*

   getLastFlightEndAddress()
   Parse the flight index end check if the flight_start address is > 0
   return -1 if no flight have been recorded else return the flight number

*/
long logger_I2C_eeprom::getLastFlightEndAddress()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_FlightConfig[i].flight_start == 0)
    {
      break;
    }
  }
  i--;
  return _FlightConfig[i].flight_stop;
}

/*

   printFlightList()


*/
int logger_I2C_eeprom::printFlightList()
{
  //retrieve from the eeprom
  int v_ret =  readFlightList();

  //Read the stucture
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_FlightConfig[i].flight_start == 0)
      break;
    SerialCom.print("Flight Nbr: ");
    SerialCom.println(i);
    SerialCom.print("Start: ");
    SerialCom.println(_FlightConfig[i].flight_start);
    SerialCom.print("End: ");
    SerialCom.println(_FlightConfig[i].flight_stop);
  }
  return i;
}

void logger_I2C_eeprom::setFlightStartAddress(int flightNbr, long startAddress)
{
  _FlightConfig[flightNbr].flight_start = startAddress;
}

void logger_I2C_eeprom::setFlightEndAddress(int flightNbr, long endAddress)
{
  _FlightConfig[flightNbr].flight_stop = endAddress;
}

void logger_I2C_eeprom::setFlightTimeData( long difftime)
{
  _FlightData.diffTime = difftime;
}
void logger_I2C_eeprom::setFlightAltitudeData( long altitude)
{
  _FlightData.altitude = altitude;
}
void logger_I2C_eeprom::setFlightTemperatureData( long temperature)
{
  _FlightData.temperature = temperature;
}
void logger_I2C_eeprom::setFlightPressureData( long pressure) {
  _FlightData.pressure = pressure;
}
#ifdef LOG_VOLTAGE
void logger_I2C_eeprom::setFlightVoltageData( long voltage) {
  _FlightData.voltage = voltage;
}
#endif
#if defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
void logger_I2C_eeprom::setADXL375accelX(long accelX)
{
  _FlightData.ADXL375accelX = accelX;
}
void logger_I2C_eeprom::setADXL375accelY(long accelY)
{
  _FlightData.ADXL375accelY = accelY; 
}
void logger_I2C_eeprom::setADXL375accelZ(long accelZ)
{
  _FlightData.ADXL375accelZ = accelZ; 
}
void logger_I2C_eeprom::setADXL345accelX(long accelX)
{
  _FlightData.ADXL345accelX = accelX; 
}
void logger_I2C_eeprom::setADXL345accelY(long accelY)
{
  _FlightData.ADXL345accelY = accelY;
}
void logger_I2C_eeprom::setADXL345accelZ(long accelZ)
{
  _FlightData.ADXL345accelZ = accelZ;
}
#endif    
long logger_I2C_eeprom::getFlightStart(int flightNbr)
{
  return _FlightConfig[flightNbr].flight_start;
}
long logger_I2C_eeprom::getFlightStop(int flightNbr)
{
  return _FlightConfig[flightNbr].flight_stop;
}
long logger_I2C_eeprom::getFlightTimeData()
{
  return _FlightData.diffTime;
}
long logger_I2C_eeprom::getFlightAltitudeData()
{
  return _FlightData.altitude;
}

long logger_I2C_eeprom::getSizeOfFlightData()
{
  return sizeof(_FlightData);
}

void logger_I2C_eeprom::printFlightData(int flightNbr)
{
  unsigned long startaddress;
  unsigned long endaddress;

  startaddress = getFlightStart(flightNbr);
  endaddress = getFlightStop(flightNbr);

  if (startaddress > 200)
  {
    unsigned long i = startaddress;
    unsigned long currentTime = 0;

    while (i < (endaddress + 1))
    {
      i = readFlight(i) + 1;
      char flightData[120] = "";
      char temp[20] = "";
      currentTime = currentTime + getFlightTimeData();
      strcat(flightData, "data,");
      sprintf(temp, "%i,", flightNbr );
      strcat(flightData, temp);
      sprintf(temp, "%i,", currentTime );
      strcat(flightData, temp);
      sprintf(temp, "%i,", getFlightAltitudeData() );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.temperature );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.pressure );
      strcat(flightData, temp);
#ifdef LOG_VOLTAGE
      sprintf(temp, "%i,", _FlightData.voltage );
      strcat(flightData, temp);
#endif
#if defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
      sprintf(temp, "%i,", _FlightData.ADXL375accelX );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.ADXL375accelY );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.ADXL375accelZ );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.ADXL345accelX );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.ADXL345accelY );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.ADXL345accelZ );
      strcat(flightData, temp);
#endif
      unsigned int chk = msgChk(flightData, sizeof(flightData));
      sprintf(temp, "%i", chk);
      strcat(flightData, temp);
      strcat(flightData, ";\n");
      //#ifdef ALTIMULTIESP32
      #if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
        Serial.print("$");
        Serial.print(flightData);
      #endif
        SerialCom.print("$");
        SerialCom.print(flightData);

      //This will slow down the data
      // this is for telemetry modules without enought buffer
      if (config.telemetryType == 0) 
        delay(0);
      else if (config.telemetryType == 1)  
        delay(20); 
      else if (config.telemetryType == 2)
        delay(50);
      else if (config.telemetryType == 3)
        delay(100);
    }
  }
}
/*
   CanRecord()
   First count the number of flights. It cannot be greter than 25
   if last flight end address is greater than the max possible
   address then the EEprom is full
*/
boolean logger_I2C_eeprom::CanRecord()
{
  long lastFlight;
  lastFlight = getLastFlightNbr();
  if (lastFlight == -1)
    return true;

  if (lastFlight == 24)
  {
    //#ifdef SERIAL_DEBUG
    //Serial.println("25 flights");
    //#endif
    return false;
  }
  // Check if eeprom is full
  if (getFlightStop(lastFlight) > 65500 )
  {
    //#ifdef SERIAL_DEBUG
    //Serial.println("memory is full");
    //#endif
    return false;
  }
  return true;
}
