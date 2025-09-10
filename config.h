#ifndef _CONFIG_H
#define _CONFIG_H
/*
 * 
 * This is where you should do all the configuration
 * 
 * 
 * First you need to make sure that you have all the require Arduino libraries to compile it
 * 
 * required libraries:
 * Adafruit_BMP085 or BMP280 or BMP085_stm32
 * 
 * 
 */


/////////////// config changes start here ///////////
// here choose one of the board that you want to use
// note that you will need to compile using the Arduino Uno or SMT32 board
// if you have the original ALTIMULTI board using an ATMega328 define ALTIMULTI
//#define ALTIMULTI

// if you have a modified ALTIMULTI board using an ATMega328 using different Arduino pins for the
// pyro output so that they do not fire following a reset of the board then define ALTIMULTIV2
// select Arduino Uno board to compile it
//#define ALTIMULTIV2

// if you have the STM32 shield then define ALTIMULTISTM32
//  Compiling the firmware:
//  You will need to download the following core
//  https://github.com/rogerclarkmelbourne/Arduino_STM32/wiki/Installation
//  and follow all instructions
//  version 1.6.12 of the core M3 need to be install
//  Choose "Generic STM32F103C board" to compile
#define ALTIMULTISTM32

// if you have the ESP32 AltiMulti board then define ALTIMULTIESP32
// choose the Wroom board to compile it
//#define ALTIMULTIESP32

// if you have the ESP32 AltiMulti board then define ALTIMULTIESP32
// choose the Wroom board to compile it
//#define ALTIMULTIESP32_ACCELERO

// if you have the ESP32 AltiMulti board then define ALTIMULTIESP32
// choose the Wroom board to compile it
//#define ALTIMULTIESP32_ACCELERO_375

// if you have the ESP32 AltiMulti board then define ALTIMULTIESP32
// choose the Wroom board to compile it
//#define ALTIMULTIESP32_ACCELERO_345

// if you have the ESP32-C3F AltiDuo board then define ALTIDUOESP32
//#define ALTIDUOESP32

// choose the pressure sensor that you are using
// for most board the pressure sensor is either BMP085 or BMP180 
// note that BMP085 and 180 are compatible no need to use the new BMP180 library
#define BMP085_180

//#define BMP_180
// if you have a custom ATMega 328 board using a BMP280 pressure sensor 
//#define BMP280_sensor

// If you want to have additionnal debugging uncomment it
//#define SERIAL_DEBUG
#undef SERIAL_DEBUG

#define BAT_MIN_VOLTAGE 7.0
//Voltage divider
#define R1 4.7
#define R2 10

#define VOLT_DIVIDER 10*(R1/(R1+R2))

////////////// config changes end here /////////////
//////////// do not change anything after unless you know what you are doing /////////////////////

#define MAJOR_VERSION 2
#define MINOR_VERSION 2
#define BUILD 1
#define CONFIG_START 32

#ifdef ALTIMULTISTM32
#include <itoa.h>
#endif



#ifdef ALTIMULTI 
#define BOARD_FIRMWARE "AltiMulti"
#define NBR_PYRO_OUT3
#define SerialCom Serial
#endif

#ifdef ALTIMULTIV2
#define BOARD_FIRMWARE "AltiMultiV2"
#define NBR_PYRO_OUT3
#define SerialCom Serial
#endif

#ifdef ALTIMULTISTM32
#define BOARD_FIRMWARE "AltiMultiSTM32"
#define NBR_PYRO_OUT4
#define SerialCom Serial1
#define LOG_VOLTAGE
#endif

#ifdef ALTIMULTIESP32
#define BOARD_FIRMWARE "AltiMultiESP32"
#define TELEMETRY_ESP32
#define NBR_PYRO_OUT3
#include "BluetoothSerial.h"
extern BluetoothSerial SerialBT;
#define SerialCom SerialBT
#define BUFFER_LENGTH I2C_BUFFER_LENGTH
#define LOG_VOLTAGE
#endif

#if defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
#define TELEMETRY_ESP32
#define NBR_PYRO_OUT3
#include "BluetoothSerial.h"
extern BluetoothSerial SerialBT;
#define SerialCom SerialBT
#define BUFFER_LENGTH I2C_BUFFER_LENGTH
#define LOG_VOLTAGE
#endif

#if defined ALTIMULTIESP32_ACCELERO
#define BOARD_FIRMWARE "AltiMultiESP32_accel"
#endif

#if defined ALTIMULTIESP32_ACCELERO_375
#define BOARD_FIRMWARE "AltiMultiESP32_accel_375"
#endif

#if defined ALTIMULTIESP32_ACCELERO_345
#define BOARD_FIRMWARE "AltiMultiESP32_accel_345"
#endif

#ifdef ALTIDUOESP32
#define BOARD_FIRMWARE "AltiDuoESP32"
#define NBR_PYRO_OUT2
#include "BluetoothSerial.h"
extern BluetoothSerial SerialBT;
#define SerialCom SerialBT
#define BUFFER_LENGTH I2C_BUFFER_LENGTH
#define LOG_VOLTAGE
#endif


#include "Arduino.h"
//used for writing in the microcontroler internal eeprom
#include <EEPROM.h>


//pyro out 1
extern const int pyroOut1;
//pyro out 2
extern const int pyroOut2;
#ifdef NBR_PYRO_OUT3
//pyro out 3
extern const int pyroOut3;
#endif
#ifdef NBR_PYRO_OUT4
//pyro out 3
extern const int pyroOut3;
//pyro out 4
extern const int pyroOut4;
#endif

extern int continuityPins[4];


struct ConfigStruct {
  int unit;             //0 = meter 1 = feet
  int beepingMode;      // decide which way you want to report the altitude
  int outPut1;          // assign a function to each pyro 0 = main 1 = drogue 2 = timer 4 = landing 5 = liftoff 3 = disable
  int outPut2;
  int outPut3;
  int mainAltitude;     //deployment altitude for the main chute
  int superSonicYesNo;  // if set to yes do not do any altitude measurement when altimeter starts
  int outPut1Delay;      // delay output by x ms
  int outPut2Delay;
  int outPut3Delay;
  int beepingFrequency;  // this beeping frequency can be changed
  int nbrOfMeasuresForApogee; //how many measure to decide that apogee has been reached
  int endRecordAltitude;  // stop recording when landing define under which altitude we are not recording
  int telemetryType;  //telemetry module type 0 is fast, 1 is average, 2 is slow
  int superSonicDelay;   //nbr of ms during when we ignore any altitude measurements
  long connectionSpeed;   //altimeter connection baudrate
  int altimeterResolution; // BMP sensor resolution
  int eepromSize;
  int noContinuity;
  int outPut4;
  int outPut4Delay;
  int liftOffAltitude; //Lift off Altitude in meters
  int batteryType; // 0= Unknown, 1= "2S (7.4 Volts)", 2 = "9 Volts",3 = "3S (11.1 Volts)
  int recordingTimeout; // in Seconds
  int altiID;
  int useTelemetryPort;
  int cksum;  
};
extern ConfigStruct config;

extern void defaultConfig();
extern bool readAltiConfig();
extern int getOutPin(int );
//extern bool writeAltiConfig( char * );
extern bool writeAltiConfigV2( char * );
extern void printAltiConfig( char *);
extern void writeConfigStruc();
extern bool CheckValideBaudRate(long);
extern unsigned int CheckSumConf( ConfigStruct );
extern unsigned int msgChk( char * buffer, long length );
#endif
