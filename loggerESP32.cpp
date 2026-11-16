/***********************************************************

   Description:
   Flight logger saving the flights in the ESP32 flash (LittleFS)
   Only used by the ALTIMULTIESP32V2 board, see loggerESP32.h

 **********************************************************/
#include "loggerESP32.h"

#ifdef ALTIMULTIESP32V2

// json document used to read one record, works with ArduinoJson 6 and 7
#if ARDUINOJSON_VERSION_MAJOR >= 7
#define RECORD_JSON_DOC(name) JsonDocument name
#else
#define RECORD_JSON_DOC(name) DynamicJsonDocument name(256)
#endif

loggerESP32::loggerESP32() {
  _fsMounted = false;
  _lastFlightNbr = -1;
  _fsTotalBytes = 0;
  _fsUsedBytes = 0;
  _recording = false;
  _writeError = false;
  _recordNbr = 0;
  _bytesWritten = 0;
  _writeLen = 0;
  _flightSize = 0;
  memset(&_currentRecord, 0, sizeof(_currentRecord));
  memset(&_FlightMinAndMax, 0, sizeof(_FlightMinAndMax));
}

/*
   initFileSystem()
   Mount the file system, format it if it cannot be mounted (first use)
*/
bool loggerESP32::initFileSystem() {
  if (!LittleFS.begin(false)) {
#ifdef SERIAL_DEBUG
    Serial.println(F("LITTLEFS Mount failed, formatting"));
#endif
    // format if begin fails
    if (!LittleFS.begin(true)) {
#ifdef SERIAL_DEBUG
      Serial.println(F("LITTLEFS mount failed, formatting not possible"));
#endif
      _fsMounted = false;
      return false;
    }
  }
  _fsMounted = true;
  return true;
}

/*
   getFlightFileName()
   flight 0 is saved in /flight1.json, flight 1 in /flight2.json ...
*/
void loggerESP32::getFlightFileName(int flightNbr, char *fileName, size_t size) {
  snprintf(fileName, size, "/flight%d.json", flightNbr + 1);
}

/*
   isFlightFile()
   check if the file name is flight<number>.json and return the number
   depending on the ESP32 core version the name can start with a /
*/
bool loggerESP32::isFlightFile(const char *name, int *fileNbr) {
  if (name == NULL)
    return false;
  if (name[0] == '/')
    name++;
  if (strncmp(name, "flight", 6) != 0)
    return false;
  const char *p = name + 6;
  if (*p < '0' || *p > '9')
    return false;
  int nbr = 0;
  while (*p >= '0' && *p <= '9') {
    nbr = nbr * 10 + (*p - '0');
    p++;
  }
  if (strcmp(p, ".json") != 0 || nbr < 1)
    return false;
  *fileNbr = nbr;
  return true;
}

/*
   readFlightList()
   Look for the flight files and remember the last flight number.
   Also remember how much of the flash is used so that we do not have
   to ask the file system during the flight (this is slow)
*/
int loggerESP32::readFlightList() {
  // the list is up to date while recording
  if (_recording)
    return _lastFlightNbr + 1;

  _lastFlightNbr = -1;
  if (!_fsMounted)
    return 0;

  int maxFileNbr = 0;
  File root = LittleFS.open("/");
  if (root && root.isDirectory()) {
    File file = root.openNextFile();
    while (file) {
      int fileNbr;
      if (!file.isDirectory() && isFlightFile(file.name(), &fileNbr)) {
        if (fileNbr > maxFileNbr)
          maxFileNbr = fileNbr;
      }
      file.close();
      file = root.openNextFile();
    }
    root.close();
  }
  _lastFlightNbr = maxFileNbr - 1;
  _fsTotalBytes = LittleFS.totalBytes();
  _fsUsedBytes = LittleFS.usedBytes();
  return _lastFlightNbr + 1;
}

/*
   writeFlightList()
   Nothing to do, the flight file is closed by setFlightEndAddress()
*/
int loggerESP32::writeFlightList() {
  return _lastFlightNbr + 1;
}

/*
   clearFlightList()
   Delete all the flight files.
   The file names are not removed while the directory is being read
   because this can skip some files.
*/
void loggerESP32::clearFlightList() {
  if (!_fsMounted)
    return;
  if (_recording)
    setFlightEndAddress(0, 0);

  // delete one file at a time, at most 100 files
  for (int i = 0; i < 100; i++) {
    char fileName[32] = "";
    File root = LittleFS.open("/");
    if (!root || !root.isDirectory())
      break;
    File file = root.openNextFile();
    while (file) {
      int fileNbr;
      if (!file.isDirectory() && isFlightFile(file.name(), &fileNbr)) {
        getFlightFileName(fileNbr - 1, fileName, sizeof(fileName));
        file.close();
        break;
      }
      file.close();
      file = root.openNextFile();
    }
    root.close();

    if (fileName[0] == '\0')
      break; // no more flight files
    if (!LittleFS.remove(fileName))
      break;
#ifdef SERIAL_DEBUG
    Serial.print(F("Deleted: "));
    Serial.println(fileName);
#endif
  }
  readFlightList();
}

/*
   eraseLastFlight()
   return false if there is no flight
*/
bool loggerESP32::eraseLastFlight() {
  if (!_fsMounted || _recording || _lastFlightNbr < 0)
    return false;
  char fileName[32];
  getFlightFileName(_lastFlightNbr, fileName, sizeof(fileName));
  bool success = LittleFS.remove(fileName);
  readFlightList();
  return success;
}

/*
   getLastFlightNbr()
   return -1 if no flight have been recorded else return the flight number
*/
int loggerESP32::getLastFlightNbr() {
  return _lastFlightNbr;
}

/*
   printFlightList()
   Start is always 0 and End is the size of the file
*/
int loggerESP32::printFlightList() {
  readFlightList();
  int i;
  for (i = 0; i <= _lastFlightNbr; i++) {
    char fileName[32];
    long fileSize = 0;
    getFlightFileName(i, fileName, sizeof(fileName));
    File file = LittleFS.open(fileName, "r");
    if (file) {
      fileSize = file.size();
      file.close();
    }
    SerialCom.print(F("Flight Nbr: "));
    SerialCom.println(i);
    SerialCom.print(F("Start: "));
    SerialCom.println(0);
    SerialCom.print(F("End: "));
    SerialCom.println(fileSize > 0 ? fileSize - 1 : 0);
  }
  return i;
}

// there are no addresses in the flash, these are kept so that the main program is the same
long loggerESP32::getFlightStart(int flightNbr) {
  return 0;
}
long loggerESP32::getFlightStop(int flightNbr) {
  return 0;
}
long loggerESP32::getLastFlightEndAddress() {
  return _fsUsedBytes;
}

/*
   CanRecord()
   We cannot record if there are already 25 flights or if the flash is full
*/
boolean loggerESP32::CanRecord() {
  if (!_fsMounted)
    return false;
  if (_lastFlightNbr + 1 >= ESP32_MAX_FLIGHTS)
    return false;
  if (_fsTotalBytes < _fsUsedBytes + ESP32_FS_MIN_FREE + ESP32_WRITE_BUFFER_SIZE)
    return false;
  return true;
}

/*
   hasRoomForRecord()
   Check during the flight that the flash is not full and that we can still write
*/
bool loggerESP32::hasRoomForRecord() {
  if (!_recording || _writeError)
    return false;
  unsigned long used = _fsUsedBytes + _bytesWritten + _writeLen + ESP32_MAX_RECORD_SIZE;
  return (_fsTotalBytes > used + ESP32_FS_MIN_FREE);
}

/*
   getUsedPercent()
   percentage of the flash used, sent in the telemetry
*/
int loggerESP32::getUsedPercent() {
  if (_fsTotalBytes == 0)
    return 0;
  unsigned long used = _fsUsedBytes;
  if (_recording)
    used += _bytesWritten + _writeLen;
  return (int)((100ULL * used) / _fsTotalBytes);
}

/*
   setFlightStartAddress()
   Called at lift off: create the flight file
*/
void loggerESP32::setFlightStartAddress(int flightNbr, long startAddress) {
  if (!_fsMounted)
    return;
  if (_recording)
    setFlightEndAddress(0, 0);

  char fileName[32];
  getFlightFileName(flightNbr, fileName, sizeof(fileName));
  _flightFile = LittleFS.open(fileName, "w");
  _recordNbr = 0;
  _bytesWritten = 0;
  _writeLen = 0;
  if (!_flightFile) {
    _writeError = true;
    _recording = false;
    return;
  }
  _writeError = false;
  _recording = true;
  if (flightNbr > _lastFlightNbr)
    _lastFlightNbr = flightNbr;

  // same format as before: {"/flight1.json":{"0":{...},"1":{...}}}
  _writeLen = snprintf(_writeBuffer, sizeof(_writeBuffer), "{\"%s\":{", fileName);
  // write the beginning of the file now so that the flight exists even if we lose the power
  flushWriteBuffer();
}

/*
   flushWriteBuffer()
   write the buffer in the flash. flush() makes sure that it is really written
*/
void loggerESP32::flushWriteBuffer() {
  if (_writeLen == 0 || !_flightFile)
    return;
  size_t written = _flightFile.write((const uint8_t *)_writeBuffer, _writeLen);
  if (written != _writeLen)
    _writeError = true;  // flash is full
  _flightFile.flush();
  _bytesWritten += written;
  _writeLen = 0;
}

/*
   writeFastFlight()
   Add the current record to the flight.
   The record goes in a buffer that is written in the flash when it is full
*/
unsigned long loggerESP32::writeFastFlight(unsigned long eeaddress) {
  if (!_recording)
    return eeaddress;

  char record[ESP32_MAX_RECORD_SIZE];
  int len = snprintf(record, sizeof(record),
                     "%s\"%ld\":{\"diffTime\":%ld,\"altitude\":%ld,\"pressure\":%ld,\"temperature\":%ld,\"voltage\":%ld}",
                     (_recordNbr == 0) ? "" : ",",
                     _recordNbr,
                     _currentRecord.diffTime,
                     _currentRecord.altitude,
                     _currentRecord.pressure,
                     _currentRecord.temperature,
                     _currentRecord.voltage);
  if (len <= 0 || len >= (int)sizeof(record))
    return eeaddress;

  if (_writeLen + len >= sizeof(_writeBuffer))
    flushWriteBuffer();
  memcpy(_writeBuffer + _writeLen, record, len);
  _writeLen += len;
  _recordNbr++;
  return eeaddress + len;
}

/*
   setFlightEndAddress()
   Called when the recording stops: finish and close the flight file
*/
void loggerESP32::setFlightEndAddress(int flightNbr, long endAddress) {
  if (!_recording)
    return;
  if (_writeLen + 3 >= sizeof(_writeBuffer))
    flushWriteBuffer();
  memcpy(_writeBuffer + _writeLen, "}}", 2);
  _writeLen += 2;
  flushWriteBuffer();
  _flightFile.close();
  _recording = false;
  _fsUsedBytes = LittleFS.usedBytes();
}

long loggerESP32::getSizeOfFlightData() {
  return sizeof(ESP32FlightRecord);
}

void loggerESP32::setFlightTimeData(long diffTime) {
  _currentRecord.diffTime = diffTime;
}
void loggerESP32::setFlightAltitudeData(long altitude) {
  _currentRecord.altitude = altitude;
}
void loggerESP32::setFlightTemperatureData(long temperature) {
  _currentRecord.temperature = temperature;
}
void loggerESP32::setFlightPressureData(long pressure) {
  _currentRecord.pressure = pressure;
}
void loggerESP32::setFlightVoltageData(long voltage) {
  _currentRecord.voltage = voltage;
}
void loggerESP32::setAccelX(float accelX) {
  _currentRecord.accelX = accelX;
}
void loggerESP32::setAccelY(float accelY) {
  _currentRecord.accelY = accelY;
}
void loggerESP32::setAccelZ(float accelZ) {
  _currentRecord.accelZ = accelZ;
}
long loggerESP32::getFlightTimeData() {
  return _currentRecord.diffTime;
}
long loggerESP32::getFlightAltitudeData() {
  return _currentRecord.altitude;
}

/*
   openFlightForReading()
   open the file and go to the first record
*/
bool loggerESP32::openFlightForReading(int flightNbr, File &file) {
  if (!_fsMounted || flightNbr < 0)
    return false;
  char fileName[32];
  getFlightFileName(flightNbr, fileName, sizeof(fileName));
  file = LittleFS.open(fileName, "r");
  if (!file)
    return false;
  // do not wait when we reach the end of the file
  file.setTimeout(0);
  // skip {"/flight1.json":{
  int c;
  while ((c = file.read()) != -1 && c != '{');
  while (c != -1 && (c = file.read()) != -1 && c != ':');
  while (c != -1 && (c = file.read()) != -1 && c != '{');
  if (c == -1) {
    file.close();
    return false;
  }
  return true;
}

/*
   readNextRecord()
   read one record "n":{...} and return false at the end of the flight.
   A file that was not closed (power lost during the flight) is read up to the last
   complete record.
*/
bool loggerESP32::readNextRecord(File &file, JsonDocument &doc, ESP32FlightRecord &record) {
  int c;
  // look for the next key, a } means that this is the end of the flight
  while ((c = file.read()) != -1 && c != '"' && c != '}');
  if (c != '"')
    return false;
  // skip the key and the :
  while ((c = file.read()) != -1 && c != '"');
  while (c != -1 && (c = file.read()) != -1 && c != ':');
  if (c == -1)
    return false;

  DeserializationError err = deserializeJson(doc, file);
  if (err)
    return false;

  record.diffTime = doc["diffTime"].as<long>();
  record.altitude = doc["altitude"].as<long>();
  record.temperature = doc["temperature"].as<long>();
  record.pressure = doc["pressure"].as<long>();
  record.voltage = doc["voltage"].as<long>();
  record.accelX = doc["accelX"].as<float>();
  record.accelY = doc["accelY"].as<float>();
  record.accelZ = doc["accelZ"].as<float>();
  return true;
}

/*
   printFlightData()
   Send the flight to the altimeter console.
   The data frame is the same as the one sent by the eeprom logger (logger_I2C_eeprom)
*/
void loggerESP32::printFlightData(int flightNbr) {
  File file;
  if (!openFlightForReading(flightNbr, file))
    return;

  RECORD_JSON_DOC(doc);
  ESP32FlightRecord record;
  unsigned long currentTime = 0;

  while (readNextRecord(file, doc, record)) {
    char flightData[120] = "";
    char temp[20] = "";
    currentTime = currentTime + record.diffTime;
    STRCAT_P(flightData, "data,");
    sprintf(temp, "%i,", flightNbr );
    strcat(flightData, temp);
    sprintf(temp, "%lu,", currentTime );
    strcat(flightData, temp);
    sprintf(temp, "%ld,", record.altitude );
    strcat(flightData, temp);
    sprintf(temp, "%ld,", record.temperature );
    strcat(flightData, temp);
    sprintf(temp, "%ld,", record.pressure );
    strcat(flightData, temp);
#ifdef LOG_VOLTAGE
    sprintf(temp, "%ld,", record.voltage );
    strcat(flightData, temp);
#endif
    unsigned int chk = msgChk(flightData, sizeof(flightData));
    sprintf(temp, "%i", chk);
    strcat(flightData, temp);
    strcat(flightData, ";\n");

#ifdef TELEMETRY_ESP32
    Serial.print('$');
    Serial.print(flightData);
#endif
    SerialCom.print('$');
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
  file.close();
}

/*
   recoverFlightData()
   With the eeprom, samples recorded after the last flight can be recovered if the flight
   list was not saved (power lost during the flight).
   In the flash the flight file is created at lift off and written during the flight,
   so an interrupted flight is a normal flight (the last one) and there is nothing to recover.
*/
void loggerESP32::recoverFlightData(int nbrOfSamples) {
}

/*
   getFlightMinAndMax()
   Read the flight and find the min and max values.
   return false if the flight cannot be read
*/
bool loggerESP32::getFlightMinAndMax(int flightNbr) {
  memset(&_FlightMinAndMax, 0, sizeof(_FlightMinAndMax));
  _flightSize = 0;

  File file;
  if (!openFlightForReading(flightNbr, file))
    return false;

  RECORD_JSON_DOC(doc);
  ESP32FlightRecord r;
  while (readNextRecord(file, doc, r)) {
    _FlightMinAndMax.flightDuration += r.diffTime;
    if (_flightSize == 0) {
      // the first record gives the initial min and max
      _FlightMinAndMax.minAltitude = _FlightMinAndMax.maxAltitude = r.altitude;
      _FlightMinAndMax.minTemperature = _FlightMinAndMax.maxTemperature = r.temperature;
      _FlightMinAndMax.minPressure = _FlightMinAndMax.maxPressure = r.pressure;
      _FlightMinAndMax.minAccelX = _FlightMinAndMax.maxAccelX = r.accelX;
      _FlightMinAndMax.minAccelY = _FlightMinAndMax.maxAccelY = r.accelY;
      _FlightMinAndMax.minAccelZ = _FlightMinAndMax.maxAccelZ = r.accelZ;
    } else {
      if (r.altitude < _FlightMinAndMax.minAltitude) _FlightMinAndMax.minAltitude = r.altitude;
      if (r.altitude > _FlightMinAndMax.maxAltitude) _FlightMinAndMax.maxAltitude = r.altitude;
      if (r.temperature < _FlightMinAndMax.minTemperature) _FlightMinAndMax.minTemperature = r.temperature;
      if (r.temperature > _FlightMinAndMax.maxTemperature) _FlightMinAndMax.maxTemperature = r.temperature;
      if (r.pressure < _FlightMinAndMax.minPressure) _FlightMinAndMax.minPressure = r.pressure;
      if (r.pressure > _FlightMinAndMax.maxPressure) _FlightMinAndMax.maxPressure = r.pressure;
      if (r.accelX < _FlightMinAndMax.minAccelX) _FlightMinAndMax.minAccelX = r.accelX;
      if (r.accelX > _FlightMinAndMax.maxAccelX) _FlightMinAndMax.maxAccelX = r.accelX;
      if (r.accelY < _FlightMinAndMax.minAccelY) _FlightMinAndMax.minAccelY = r.accelY;
      if (r.accelY > _FlightMinAndMax.maxAccelY) _FlightMinAndMax.maxAccelY = r.accelY;
      if (r.accelZ < _FlightMinAndMax.minAccelZ) _FlightMinAndMax.minAccelZ = r.accelZ;
      if (r.accelZ > _FlightMinAndMax.maxAccelZ) _FlightMinAndMax.maxAccelZ = r.accelZ;
    }
    _flightSize++;
  }
  file.close();
  return true;
}

long loggerESP32::getFlightDuration() {
  return _FlightMinAndMax.flightDuration;
}
long loggerESP32::getMinAltitude() {
  return _FlightMinAndMax.minAltitude;
}
long loggerESP32::getMaxAltitude() {
  return _FlightMinAndMax.maxAltitude;
}
long loggerESP32::getMinTemperature() {
  return _FlightMinAndMax.minTemperature;
}
long loggerESP32::getMaxTemperature() {
  return _FlightMinAndMax.maxTemperature;
}
long loggerESP32::getMinPressure() {
  return _FlightMinAndMax.minPressure;
}
long loggerESP32::getMaxPressure() {
  return _FlightMinAndMax.maxPressure;
}
float loggerESP32::getMinAccelX() {
  return _FlightMinAndMax.minAccelX;
}
float loggerESP32::getMaxAccelX() {
  return _FlightMinAndMax.maxAccelX;
}
float loggerESP32::getMinAccelY() {
  return _FlightMinAndMax.minAccelY;
}
float loggerESP32::getMaxAccelY() {
  return _FlightMinAndMax.maxAccelY;
}
float loggerESP32::getMinAccelZ() {
  return _FlightMinAndMax.minAccelZ;
}
float loggerESP32::getMaxAccelZ() {
  return _FlightMinAndMax.maxAccelZ;
}
// number of records read by getFlightMinAndMax()
long loggerESP32::getFlightSize() {
  return _flightSize;
}

#endif // ALTIMULTIESP32V2
