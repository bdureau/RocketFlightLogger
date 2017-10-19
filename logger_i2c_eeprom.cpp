#include "logger_i2c_eeprom.h"

logger_I2C_eeprom::logger_I2C_eeprom(int deviceAddress)
{
  _deviceAddress=deviceAddress;
    logger_I2C_eeprom(deviceAddress, LOGGER_I2C_EEPROM_PAGESIZE);
}

logger_I2C_eeprom::logger_I2C_eeprom(int deviceAddress, const unsigned int deviceSize)
{
    _deviceAddress =deviceAddress;
    
    // Chips 16Kbit (2048 Bytes) or smaller only have one-word addresses.
    // Also try to guess page size from device size (going by Microchip 24LCXX datasheets here).
    /*if (deviceSize <= 256)
    {
        this->_isAddressSizeTwoWords = false;
        this->_pageSize = 8;
    }
    else if (deviceSize <= 256 * 8)
    {
        this->_isAddressSizeTwoWords = false;
        this->_pageSize = 16;
    }
    else
    {
        this->_isAddressSizeTwoWords = true;
        this->_pageSize = 32;
    }*/
}

void logger_I2C_eeprom::begin()
{
    Wire.begin();
    //initialize Flight structure

}

void logger_I2C_eeprom::clearFlightList()
{
  int i;
  for(i=0;i<25;i++)
  {
    _FlightConfig[i].flight_start=0;    
    _FlightConfig[i].flight_stop=0;
  }
 
}

void logger_I2C_eeprom::write_byte( unsigned int eeaddress, byte data ) {
  int rdata = data;
  int writeDelay=10;
  Wire.beginTransmission(_deviceAddress);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(rdata);
  Wire.endTransmission();
  delay(writeDelay);
}

byte logger_I2C_eeprom::read_byte(  unsigned int eeaddress ) {
  byte rdata = 0xFF;
  int readDelay=5;
  Wire.beginTransmission(_deviceAddress);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(_deviceAddress,1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

int logger_I2C_eeprom::readFlightList() {

  int i;
  /*Serial.print("Size of struct:");
  Serial.println(sizeof(_FlightConfig));
  Serial.print("device adress:");
  Serial.println(_deviceAddress);*/
  for( i=0; i< sizeof(_FlightConfig); i++ ) {
   // Serial.println(read_byte( FLIGHT_LIST_START+i ));
    *((char*)&_FlightConfig + i) = read_byte( FLIGHT_LIST_START+i );
  }
  return FLIGHT_LIST_START+i ;
}

int logger_I2C_eeprom::readFlight(int eeaddress) {

  int i;
  for( i=0; i< sizeof(_FlightData); i++ ) {
    *((char*)&_FlightData + i) = read_byte( eeaddress+i );
  }
  return eeaddress+i;
}

int logger_I2C_eeprom::writeFlightList()
{
    int i;
    for( i=0; i<sizeof(_FlightConfig); i++ ) {
      write_byte( FLIGHT_LIST_START+i, *((char*)&_FlightConfig + i) );
    }
    return FLIGHT_LIST_START+i;
}

int logger_I2C_eeprom::writeFlight(int eeaddress)
{
    int i;
    for( i=0; i<sizeof(_FlightData); i++ ) {
      write_byte( eeaddress+i, *((char*)&_FlightData + i) );
    }
    return eeaddress+i;
}
int logger_I2C_eeprom::getLastFlightNbr()
{
  int i;
  for(i=0;i<25;i++)
  {
   // Serial.print("i:");
  //Serial.println(i);
 // Serial.println(_FlightConfig[i].flight_start);
    if(_FlightConfig[i].flight_start==0)
    {
      break;
    }
  }
  i--;
    //Serial.print("ret i:");
  //Serial.println(i);
 return i;
}

int logger_I2C_eeprom::printFlightList()
{
  //retrieve from the eeprom
  int v_ret=  readFlightList();
  //Serial.println(v_ret);
  //Read the stucture
  int i;
  for(i=0;i<25;i++)
  {
    if(_FlightConfig[i].flight_start==0)
      break;
    Serial.print("Flight Nbr: ");
    Serial.println(i);
    Serial.print("Start: ");
    Serial.println(_FlightConfig[i].flight_start);
    Serial.print("End: ");
    Serial.println(_FlightConfig[i].flight_stop);
    
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
 
  if (startaddress>200)
  {
    int i = startaddress;
    unsigned long currentTime=0;
    Serial.println("StartFlight;" );
    while(i < (endaddress +1))
    {
      i = readFlight(i)+1;
 
      currentTime= currentTime + getFlightTimeData();
     
      Serial.println(String(currentTime) + ","+ getFlightAltitudeData());
      
    }
    Serial.println("EndFlight;" );
  }
  else
    Serial.println(F("No such flight\n"));
}

void logger_I2C_eeprom::printFlightData(int flightNbr)
{

  int startaddress;
  int endaddress;

  startaddress = getFlightStart(flightNbr);
  endaddress = getFlightStop(flightNbr);
  
  if (startaddress>200)
  {
    int i = startaddress;
    unsigned long currentTime=0;
  
    while(i < (endaddress +1))
    {
      i = readFlight(i)+1;
 
      currentTime= currentTime + getFlightTimeData();
      
      Serial.println("$" + String("data,") +String(flightNbr) +","+String(currentTime) + ","+ String(getFlightAltitudeData()) +";");
    
    }
   
  }
}

boolean logger_I2C_eeprom::CanRecord()
{
  long lastFlight;
  lastFlight = getLastFlightNbr();
  if (lastFlight==-1)
  return true;
 // Serial.println(lastFlight);
  if (lastFlight == 24)
  {
   // #ifdef SERIAL_DEBUG
  //Serial.println("25 flights");
 // #endif
    return false;
  }
 if(getFlightStop(lastFlight)> 65500 )
 {
   // #ifdef SERIAL_DEBUG
  //Serial.println("memory is full");
  //#endif
   return false;   
 }
 return true;   
} 
