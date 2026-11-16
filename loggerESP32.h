/***********************************************************

   Description:
   This is a wrapper to log the flight to the ESP32 flash (LittleFS)
   It is only used by the ALTIMULTIESP32V2 board.

   It has the same functions as the logger_I2C_eeprom so that the main program
   does not need to know where the flights are saved.

   Each flight is saved in its own file /flight<n>.json where n starts at 1
   (flight 0 for the altimeter console is the file /flight1.json)
   The file format is:
   {"/flight1.json":{"0":{"diffTime":10,"altitude":1,...},"1":{...},...}}

   The file is written during the flight (every few samples) so that if we lose
   the power during the flight the samples that are already written can still be read.
   The file is also read one record at a time so that big flights can be read
   without using a lot of memory.
 **********************************************************/
#ifndef _LOGGER_ESP32_H
#define _LOGGER_ESP32_H

// config.h must be included first so that the board compilation directive is known
#include "config.h"

#ifdef ALTIMULTIESP32V2
#include <LittleFS.h>
#include <ArduinoJson.h>

// maximum number of flights, same as the eeprom version
#define ESP32_MAX_FLIGHTS 25
// do not fill the flash completely, LittleFS needs some free blocks to work
#define ESP32_FS_MIN_FREE 16384
// samples are kept in this buffer and written to the flash when it is full
#define ESP32_WRITE_BUFFER_SIZE 2048
// maximum size of one record once written in json
#define ESP32_MAX_RECORD_SIZE 200

struct ESP32FlightRecord {
  long diffTime;
  long altitude;
  long temperature;
  long pressure;
  long voltage;
  float accelX;
  float accelY;
  float accelZ;
};

struct FlightMinAndMaxStruct {
  long minAltitude;
  long maxAltitude;
  long minTemperature;
  long maxTemperature;
  long minPressure;
  long maxPressure;
  float minAccelX;
  float maxAccelX;
  float minAccelY;
  float maxAccelY;
  float minAccelZ;
  float maxAccelZ;
  long flightDuration;
};

class loggerESP32 {
  public:
    loggerESP32();
    bool initFileSystem();

    // same functions as the logger_I2C_eeprom
    int readFlightList();
    int writeFlightList();
    void clearFlightList();
    bool eraseLastFlight();
    int getLastFlightNbr();
    int printFlightList();
    long getFlightStart(int flightNbr);
    long getFlightStop(int flightNbr);
    long getLastFlightEndAddress();
    boolean CanRecord();
    void setFlightStartAddress(int flightNbr, long startAddress);
    void setFlightEndAddress(int flightNbr, long endAddress);
    unsigned long writeFastFlight(unsigned long eeaddress);
    long getSizeOfFlightData();
    void setFlightTimeData(long diffTime);
    void setFlightAltitudeData(long altitude);
    void setFlightTemperatureData(long temperature);
    void setFlightPressureData(long pressure);
    void setFlightVoltageData(long voltage);
    long getFlightTimeData();
    long getFlightAltitudeData();
    void printFlightData(int flightNbr);
    void recoverFlightData(int nbrOfSamples);

    // ESP32 flash only
    bool hasRoomForRecord();
    int getUsedPercent();
    void setAccelX(float accelX);
    void setAccelY(float accelY);
    void setAccelZ(float accelZ);
    bool getFlightMinAndMax(int flightNbr);
    long getMinAltitude();
    long getMaxAltitude();
    long getMinTemperature();
    long getMaxTemperature();
    long getMinPressure();
    long getMaxPressure();
    float getMinAccelX();
    float getMaxAccelX();
    float getMinAccelY();
    float getMaxAccelY();
    float getMinAccelZ();
    float getMaxAccelZ();
    long getFlightDuration();
    long getFlightSize();

  private:
    void getFlightFileName(int flightNbr, char *fileName, size_t size);
    bool isFlightFile(const char *name, int *fileNbr);
    bool openFlightForReading(int flightNbr, File &file);
    bool readNextRecord(File &file, JsonDocument &doc, ESP32FlightRecord &record);
    void flushWriteBuffer();

    bool _fsMounted;
    int _lastFlightNbr;          // -1 if no flight
    unsigned long _fsTotalBytes;
    unsigned long _fsUsedBytes;  // used bytes when the flight list was read

    // flight being recorded
    File _flightFile;
    bool _recording;
    bool _writeError;
    long _recordNbr;
    unsigned long _bytesWritten;
    char _writeBuffer[ESP32_WRITE_BUFFER_SIZE];
    size_t _writeLen;

    ESP32FlightRecord _currentRecord;
    long _flightSize;
    FlightMinAndMaxStruct _FlightMinAndMax;
};

#endif // ALTIMULTIESP32V2
#endif // _LOGGER_ESP32_H
