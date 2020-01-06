#include "logger_i2c_eeprom.h"
#include "IC2extEEPROM.h"
extEEPROM eep(kbits_512, 1, 64); 
logger_I2C_eeprom::logger_I2C_eeprom(uint8_t deviceAddress)
{
  //_deviceAddress = deviceAddress;
  //logger_I2C_eeprom(deviceAddress, LOGGER_I2C_EEPROM_PAGESIZE);
}

/*logger_I2C_eeprom::logger_I2C_eeprom(uint8_t deviceAddress, const unsigned int deviceSize)
{
  _deviceAddress = deviceAddress;
 _pageSize = 64;
  // Chips 16Kbit (2048 Bytes) or smaller only have one-word addresses.
  // Also try to guess page size from device size (going by Microchip 24LCXX datasheets here).
  
}*/

void logger_I2C_eeprom::begin()
{
  Wire.begin();
  //initialize Flight structure

}

void logger_I2C_eeprom::clearFlightList()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    _FlightConfig[i].flight_start = 0;
    _FlightConfig[i].flight_stop = 0;
  }

}

/*void logger_I2C_eeprom::write_byte( unsigned int eeaddress, uint8_t data ) {
  int rdata = data;
  int writeDelay = 10;
  Wire.beginTransmission(_deviceAddress);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(rdata);
  Wire.endTransmission();
  delay(writeDelay);
}

uint8_t logger_I2C_eeprom::read_byte(  unsigned int eeaddress ) {
  uint8_t rdata = 0xFF;
  int readDelay = 5;
  Wire.beginTransmission(_deviceAddress);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)_deviceAddress, (uint8_t)1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}*/

int logger_I2C_eeprom::readFlightList() {

/*  int i;
  
  for ( i = 0; i < sizeof(_FlightConfig); i++ ) {
    // Serial.println(read_byte( FLIGHT_LIST_START+i ));
    *((char*)&_FlightConfig + i) = read_byte( FLIGHT_LIST_START + i );
  }
  return FLIGHT_LIST_START + i ;*/
   eep.read(0, ((byte*)&_FlightConfig), sizeof(_FlightConfig));
  return FLIGHT_LIST_START + sizeof(_FlightConfig) ;
}

int logger_I2C_eeprom::readFlight(int eeaddress) {

  
  eep.read(eeaddress, ((byte*)&_FlightData), sizeof(_FlightData));
  return eeaddress + sizeof(_FlightData);
}

int logger_I2C_eeprom::writeFlightList()
{
  
  eep.write(FLIGHT_LIST_START, ((byte*)&_FlightConfig), sizeof(_FlightConfig));
  return FLIGHT_LIST_START + sizeof(_FlightConfig);
}
int logger_I2C_eeprom::writeFastFlight(int eeaddress){
  eep.write(eeaddress, ((byte*)&_FlightData), sizeof(_FlightData));
  return eeaddress + sizeof(_FlightData);
}

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
    Serial1.print("Flight Nbr: ");
    Serial1.println(i);
    Serial1.print("Start: ");
    Serial1.println(_FlightConfig[i].flight_start);
    Serial1.print("End: ");
    Serial1.println(_FlightConfig[i].flight_stop);
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

void logger_I2C_eeprom::PrintFlight(int flightNbr)
{

  long startaddress;
  long endaddress;
  startaddress = getFlightStart(flightNbr);
  endaddress = getFlightStop(flightNbr);

  if (startaddress > 200)
  {
    int i = startaddress;
    unsigned long currentTime = 0;
    SerialCom.println("StartFlight;" );
    while (i < (endaddress + 1))
    {
      i = readFlight(i) + 1;

      currentTime = currentTime + getFlightTimeData();

      SerialCom.println(String(currentTime) + "," + getFlightAltitudeData()+"," +_FlightData.temperature);

    }
    SerialCom.println("EndFlight;" );
  }
  else
    SerialCom.println(F("No such flight\n"));
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

      currentTime = currentTime + getFlightTimeData();

      SerialCom.println("$" + String("data,") + String(flightNbr) + "," + String(currentTime) + 
      "," + String(getFlightAltitudeData()) + "," + String(_FlightData.temperature)+ ";");

    }

  }
}

boolean logger_I2C_eeprom::CanRecord()
{
  long lastFlight;
  lastFlight = getLastFlightNbr();
  if (lastFlight == -1)
    return true;
  // Serial.println(lastFlight);
  if (lastFlight == 24)
  {
    //#ifdef SERIAL_DEBUG
    //Serial.println("25 flights");
    //#endif
    return false;
  }
  if (getFlightStop(lastFlight) > 65500 )
  {
    //#ifdef SERIAL_DEBUG
    //Serial.println("memory is full");
    //#endif
    return false;
  }
  return true;
}
