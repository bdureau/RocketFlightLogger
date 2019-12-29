#include "logger_i2c_eeprom.h"

logger_I2C_eeprom::logger_I2C_eeprom(uint8_t deviceAddress)
{
  _deviceAddress = deviceAddress;
  logger_I2C_eeprom(deviceAddress, LOGGER_I2C_EEPROM_PAGESIZE);
}

logger_I2C_eeprom::logger_I2C_eeprom(uint8_t deviceAddress, const unsigned int deviceSize)
{
  _deviceAddress = deviceAddress;
 _pageSize = 64;
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
  for (i = 0; i < 25; i++)
  {
    _FlightConfig[i].flight_start = 0;
    _FlightConfig[i].flight_stop = 0;
  }

}

void logger_I2C_eeprom::write_byte( unsigned int eeaddress, uint8_t data ) {
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
}

int logger_I2C_eeprom::readFlightList() {

  int i;
  /*Serial.print("Size of struct:");
    Serial.println(sizeof(_FlightConfig));
    Serial.print("device adress:");
    Serial.println(_deviceAddress);*/
  for ( i = 0; i < sizeof(_FlightConfig); i++ ) {
    // Serial.println(read_byte( FLIGHT_LIST_START+i ));
    *((char*)&_FlightConfig + i) = read_byte( FLIGHT_LIST_START + i );
  }
  return FLIGHT_LIST_START + i ;
}

int logger_I2C_eeprom::readFlight(int eeaddress) {

  int i;
  for ( i = 0; i < sizeof(_FlightData); i++ ) {
    *((char*)&_FlightData + i) = read_byte( eeaddress + i );
  }
  return eeaddress + i;
}

int logger_I2C_eeprom::writeFlightList()
{
  int i;
  for ( i = 0; i < sizeof(_FlightConfig); i++ ) {
    write_byte( FLIGHT_LIST_START + i, *((char*)&_FlightConfig + i) );
  }
  return FLIGHT_LIST_START + i;
}

int logger_I2C_eeprom::writeFlight(int eeaddress)
{
  int i;
  for ( i = 0; i < sizeof(_FlightData); i++ ) {
    write_byte( eeaddress + i, *((char*)&_FlightData + i) );
  }
  return eeaddress + i;
}

int  logger_I2C_eeprom::writeFastFlight(uint16_t eeaddress) {
  // Have to handle write page wrapping,
  // 24lc512 has 128 byte
  // 24lc64 has 32 byte

  //const uint16_t len;
  const uint8_t pageSize = _pageSize;
  uint16_t bk = sizeof(_FlightData);
  bool abort = false;
  uint8_t i;
  uint16_t j = 0;
  uint32_t timeout;
  uint16_t mask = pageSize - 1;
  while ((bk > 0) && !abort) {
    i = I2C_TWIBUFFERSIZE; // maximum data bytes that Wire.h can send in one transaction
    if (i > bk) i = bk; // data block is bigger than Wire.h can handle in one transaction
    if (((eeaddress) & ~mask) != ((((eeaddress) + i) - 1) & ~mask)) { // over page! block would wrap around page
      i = (((eeaddress) | mask) - (eeaddress)) + 1; // shrink the block until it stops at the end of the current page

    }
    //wait for the EEPROM device to complete a prior write, or 10ms
    timeout = millis();
    bool ready = false;
    while (!ready && (millis() - timeout < 10)) {
      Wire.beginTransmission(_deviceAddress);
      ready = (Wire.endTransmission(true) == 0); // wait for device to become ready!
    }
    if (!ready) { // chip either does not exist, is hung, or died
      abort = true;

      break;
    }

    // start sending this current block
    Wire.beginTransmission(_deviceAddress);
    Wire.write((uint8_t)highByte(eeaddress));
    Wire.write((uint8_t)lowByte(eeaddress));

    bk = bk - i;
    eeaddress = (eeaddress) + i;

    while (i > 0) {
      Wire.write(*((char*)&_FlightData + (j++)));
      i--;
    }

    uint8_t err = Wire.endTransmission();
    //delay(10);
    if(err!=0){
 SerialCom.print(F("write Failure="));
 SerialCom.println(err,DEC);
 //abort = true;

 }
  }

  return eeaddress;
}

int logger_I2C_eeprom::getLastFlightNbr()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    // Serial.print("i:");
    //Serial.println(i);
    // Serial.println(_FlightConfig[i].flight_start);
    if (_FlightConfig[i].flight_start == 0)
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
  int v_ret =  readFlightList();
  //Serial.println(v_ret);
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
