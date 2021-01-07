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
int logger_I2C_eeprom::readFlight(int eeaddress)
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
int logger_I2C_eeprom::writeFastFlight(int eeaddress)
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

   getLastFlightNbr()
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
long logger_I2C_eeprom::getFlightStart(int flightNbr)
{
  return  _FlightConfig[flightNbr].flight_start;
}
long logger_I2C_eeprom::getFlightStop(int flightNbr)
{
  return  _FlightConfig[flightNbr].flight_stop;
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
  int startaddress;
  int endaddress;

  startaddress = getFlightStart(flightNbr);
  endaddress = getFlightStop(flightNbr);

  if (startaddress > 200)
  {
    int i = startaddress;
    unsigned long currentTime = 0;

    while (i < (endaddress + 1))
    {
      i = readFlight(i) + 1;
      char flightData[120] = "";
      char temp[9] = "";
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
      unsigned int chk = msgChk(flightData, sizeof(flightData));
      sprintf(temp, "%i", chk);
      strcat(flightData, temp);
      strcat(flightData, ";\n");
      SerialCom.print("$");
      SerialCom.print(flightData);
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
