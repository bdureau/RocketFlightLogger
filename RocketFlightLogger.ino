/*
  Rocket Flight Logger ver 2.0
  Copyright Boris du Reau 2012-2024

  The following is a datalogger for logging rocket flight.
  So far it can log the rocket altitude during the flight.
  The code works with different boards, you need to adjust the config.h file to reflect your board.


  This is using a BMP085 or BMP180 presure sensor and an Atmega 328 or an STM32
  The compatible boards are Altimulti, AltimultiV2 and AltiMultiSTM32. To complile the
  firmware to the various board go to the config.h file and select one of the
  following compilation directive
  #define ALTIMULTI
  #define ALTIMULTIV2
  #define ALTIMULTISTM32
  #define ALTIMULTIESP32
  #define ALTIMULTIESP32_ACCELERO
  #define ALTIMULTIESP32_ACCELERO_375
  #define ALTIMULTIESP32_ACCELERO_345

  Then on your Arduino environment select the appropriate boards:
  Arduino Uno => AltiMulti and AltiMultiV2
  Generic STM32F103C series => AltimultiSTM32

  The following record the flight (altitude, temperature, pressure) in an EEPROM.
  Note that internally the altimeter is working is metric, for conveniance the beeping
  altitude can be changed to feet.
  You can query the board using AT commands or you can use the Android front end.
  Note that if you do not want to compile the firmware and/or the Android front end you
  can get everything on the Android play store, just search for BearConsole


  For the BMP085 or BMP180 pressure sensor
  Connect VCC of the BMP085 sensor to 5.0V! make sure that you are using the 5V sensor (GY-65 model for the BMP085 )
  Connect GND to Ground
  Connect SCL to i2c clock - on 328 Arduino Uno/Duemilanove/etc thats Analog 5
  Connect SDA to i2c data - on 328 Arduino Uno/Duemilanove/etc thats Analog 4
  EOC is not used, it signifies an end of conversion

  For the EEPROM 24LC512-I/P - SERIAL EEPROM 512K,24LC512, DIP8

  Number Name ConnectTo     Note
  1       a0     GND
  2       a1     GND
  3       a2     GND
  4       GND    GND
  5       SDA    SDA         2kOhm PullUp
  6       SCL    SCL
  7       WP     GND
  8       VCC    (+3v3 ... +5V0)

  The time to write in the EEPROM is 10ms

  For the BMP280


  Major changes on version 1.5
  Altimeter configuration has been softcoded

  Major changes on version 1.6
  The code is now splitted in different files for better readability
  added control so that only 25 flights are allowed
  Check if the EEPROM is full

  Major changes on version 1.7
  bug fixes
  Major changes on version 1.8
  bug fixes
  Major changes on version 1.9
  Eeprom storage manipulation
  Major changes on version 1.10
  rethink alti config
  Major changes on version 1.11
  Added function to default the alti config
  Major changes on version 1.12
  Fixes
  Major changes on version 1.13
  Add compatibility with AltimultiV2
  Change resolution of the sensor
  Major changes on version 1.14
  Added telemetry
  Major changes on version 1.15
  Clean up
  Major changes on version 1.16
  added assynchronous delay
  added STM32 port
  Major changes on version 1.17
  fixes
  Major changes on version 1.18
  added altimeter status
  low battery alarm (only for STM32 boards)
  Major changes on version 1.19
  bug fixes
  added the ability to turn off telemetry
  added config checksum calculation
  added software pull up so that it works with all bluetooth modules
  Major changes on version 1.20
  Changed the EEPROM logging so that it does it a lot faster
  Lot's of tidy up
  Major changes on version 1.21
  added checksum
  Major changes on version 1.22
  added landing event
  added liftoff event
  added multiple main and drogue
  Major changes on version 1.23
  Fixes to the delays when multiple event of the same type
   Major changes on version 1.24
  Optimise the code so that it uses less global variables
  Added altitude event
  Major changes on version 1.25
  Added recording timeout
  Retreive flights one by one so that it can be cancelled
  Adding telemetry module connection test
  Major changes on version 1.26
  Adding ESP32
  starting using custom libraries
  Major changes on version 1.27
  Logging voltage
  Major changes on version 1.28
  Adding ESP32 C3

  Major changes on version 2.0
  Adding accelerometers

  Major changes on version 2.1
  Allow renaming of the bluetooth for the ESP32
*/

//altimeter configuration lib
#include "config.h"
#include <Wire.h> //I2C library

#ifdef BMP085_180
#include "Bear_BMP085.h"
#endif

/*#ifdef BMP_180
  #include <BMP180.h>
  #endif*/
#ifdef BMP280
#include <BMP280.h>
#define P0 1013.25
#endif

#include "kalman.h"
#include "beepfunc.h"

#include "logger_i2c_eeprom.h"

#if defined ALTIMULTIESP32_ACCELERO_375
#include <Adafruit_ADXL375.h>
Adafruit_ADXL375 accel375 = Adafruit_ADXL375(0x53);
//Adafruit_ADXL375 accel375 = Adafruit_ADXL375(0x1D);

#endif
#if defined ALTIMULTIESP32_ACCELERO
#include <Adafruit_ADXL375.h>
Adafruit_ADXL375 accel375 = Adafruit_ADXL375(0x1D); //def
//Adafruit_ADXL375 accel375 = Adafruit_ADXL375(0x53);
#endif
#if defined ALTIMULTIESP32_ACCELERO_345 || defined ALTIMULTIESP32_ACCELERO
#include <Adafruit_ADXL345_U.h>
Adafruit_ADXL345_Unified accel345 = Adafruit_ADXL345_Unified();
#endif

#if defined ALTIDUOESP32 || defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
#include <Preferences.h>
Preferences preferences;
#endif

//////////////////////////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////////////////////////

#ifdef BMP085_180
//Adafruit_BMP085 bmp;
BMP085 bmp;
#endif
/*#ifdef BMP_180
  BMP180 bmp;
  #endif*/
#ifdef BMP280
BMP280 bmp;
#endif


//EEProm address
logger_I2C_eeprom logger(0x50) ;
// End address of the 512 eeprom
long endAddress = 65536;
// current file number that you are recording
int currentFileNbr = 0;
// EEPROM start address for the flights. Anything before that is the flight index
long currentMemaddress = 200;
//stop recording a maximum of 20 seconds after main has fired
long recordingTimeOut = 20000;
boolean canRecord = true;
boolean exitRecording = false;

//ground level altitude
long initialAltitude;
long liftoffAltitude;
long lastAltitude;
//current altitude
long currAltitude;
//Apogee altitude
long apogeeAltitude;
long mainAltitude;
long drogueFiredAltitude;

boolean liftOff = false;
unsigned long initialTime;
boolean FastReading = false;

//nbr of measures to do so that we are sure that apogee has been reached
unsigned long measures = 5;
unsigned long mainDeployAltitude;

#if defined ALTIDUOESP32 || defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
BluetoothSerial SerialBT;
#endif


// pin used by the jumpers
#ifdef ALTIMULTISTM32
const int pinAltitude1 = PB3;
const int pinAltitude2 = PB4;
#endif
#ifdef ALTIMULTI
const int pinAltitude1 = 8;
const int pinAltitude2 = 7;
#endif
#ifdef ALTIMULTIV2
const int pinAltitude1 = 8;
const int pinAltitude2 = 7;
#endif

#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
const int pinAltitude1 = 4;
const int pinAltitude2 = 0;
#endif
#ifdef ALTIDUOESP32
const int pinAltitude1 = -1;
const int pinAltitude2 = -1;
#endif
//note that the STM32 board has 4 pyro output
#ifdef ALTIMULTISTM32
//by default apogee pin
const int pinChannel1Continuity = PA2;
// by default continuity for the main
const int pinChannel2Continuity = PA4;
// third output
const int pinChannel3Continuity = PA6;
#ifdef NBR_PYRO_OUT4
// fourth output
const int pinChannel4Continuity = PB0;
#endif
#endif

#ifdef ALTIMULTI
//by default apogee pin
const int pinChannel1Continuity = 10;
// by default continuity for the main
const int pinChannel2Continuity = 11;
// third output
const int pinChannel3Continuity = 16;
#endif
#ifdef ALTIMULTIV2
//by default apogee pin
const int pinChannel1Continuity = 11;//10;
// by default continuity for the main
const int pinChannel2Continuity = 10;//11;
// third output
const int pinChannel3Continuity = 16;
#endif

#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
//by default apogee pin
const int pinChannel1Continuity = 5;
// by default continuity for the main
const int pinChannel2Continuity = 17;
// third output
const int pinChannel3Continuity = 23;
#endif
#ifdef ALTIDUOESP32
//by default apogee pin
const int pinChannel1Continuity = -1;
// by default continuity for the main
const int pinChannel2Continuity = -1;
#endif
float FEET_IN_METER = 1;


boolean Output1Fired = false;
boolean Output2Fired = false;
#ifdef NBR_PYRO_OUT3
boolean Output3Fired = false;
#endif
#ifdef NBR_PYRO_OUT4
boolean Output3Fired = false;
boolean Output4Fired = false;
#endif
boolean allApogeeFiredComplete = false;
boolean allMainFiredComplete = false;
boolean allTimerFiredComplete = false;
boolean allLiftOffFiredComplete = false;
boolean allLandingFiredComplete = false;
boolean allAltitudeFiredComplete = false;


//telemetry
boolean telemetryEnable = false;
long lastTelemetry = 0;
long lastBattWarning = 0;




void MainMenu();


#ifdef BMP085_180
/*
   ReadAltitude()
   Read Altitude function for a BMP85 or 180 Bosch sensor

*/
float ReadAltitude()
{
  return KalmanCalc(bmp.readAltitude());
  //return bmp.readAltitude();
}
#endif



#ifdef BMP280
/*

   Read Altitude function for a BMP280 Bosch sensor


*/
double ReadAltitude()
{
  double T, P, A;
  char result = bmp.startMeasurment();
  if (result != 0) {
    delay(result);
    result = bmp.getTemperatureAndPressure(T, P);
    A = KalmanCalc(bmp.altitude(P, P0));
  }
  return A;
}
#endif
/*

  ResetGlobalVar()

*/
void ResetGlobalVar() {
  recordingTimeOut = config.recordingTimeout * 1000;

  exitRecording = false;

  allApogeeFiredComplete = false;
  allMainFiredComplete = false;
  allTimerFiredComplete = false;
  allLiftOffFiredComplete = false;
  allLandingFiredComplete = false;
  allAltitudeFiredComplete = false;

  liftOff = false;
  apogeeAltitude = 0;
  mainAltitude = 0;

  Output1Fired = false;
  Output2Fired = false;
#ifdef NBR_PYRO_OUT3
  Output3Fired = false;
#endif
#ifdef NBR_PYRO_OUT4
  Output3Fired = false;
  Output4Fired = false;
#endif

  lastAltitude = 0;//initialAltitude;

}

/*

   initAlti()

*/
void initAlti() {

  ResetGlobalVar();

  // set main altitude (if in feet convert to metrics)
  if (config.unit == 0)
    FEET_IN_METER = 1;
  else
    FEET_IN_METER = 3.28084 ;

  mainDeployAltitude = int(config.mainAltitude / FEET_IN_METER);
  // beepFrequency
  beepingFrequency = config.beepingFrequency;


  //number of measures to do to detect Apogee
  measures = config.nbrOfMeasuresForApogee;

  //check which pyro are enabled
  pos = -1;

  if (config.outPut1 != 3) {
    pos++;
    continuityPins[pos] = pinChannel1Continuity;
  }
  if (config.outPut2 != 3) {
    pos++;
    continuityPins[pos] = pinChannel2Continuity;
  }
#ifdef NBR_PYRO_OUT3
  if (config.outPut3 != 3) {
    pos++;
    continuityPins[pos] = pinChannel3Continuity;
  }
#endif
#ifdef NBR_PYRO_OUT4
  if (config.outPut3 != 3) {
    pos++;
    continuityPins[pos] = pinChannel3Continuity;
  }
  if (config.outPut4 != 3)  {
    pos++;
    continuityPins[pos] = pinChannel4Continuity;
  }
#endif
}

/*
   setup()
   Initialise altimeter

*/
void setup()
{
  int val = 0;     // variable to store the read value
  int val1 = 0;     // variable to store the read value
  //soft configuration
  boolean softConfigValid = false;
  // Read altimeter softcoded configuration
  softConfigValid = readAltiConfig();

  // check if configuration is valid
  if (!softConfigValid)
  {
    //default values
    defaultConfig();
    writeConfigStruc();
  }

  // if the baud rate is invalid let's default it
  if (!CheckValideBaudRate(config.connectionSpeed))
  {
    config.connectionSpeed = 38400;
    config.cksum = CheckSumConf(config);
    writeConfigStruc();
  }

  initAlti();
  // init Kalman filter
  KalmanInit();
  // initialise the connection
  Wire.begin();

#if defined ALTIMULTIESP32
  //You can change the baud rate here
  //and change it to 57600, 115200 etc..
  Serial.begin(38400);
  char bluetoothName [21];
  preferences.begin("namespace", false); 
  //sprintf(altiName, "ESP32Rocket%i", (int)config.altiID );
  sprintf(bluetoothName, "%s", preferences.getString("Name", "ESP32Rocket"));
  preferences.end();
  SerialCom.begin(bluetoothName);

#elif defined(ALTIMULTIESP32_ACCELERO)
  Serial.begin(38400);
  char bluetoothName [21];
  //sprintf(altiName, "ESP32Accel%i", (int)config.altiID );
  preferences.begin("namespace", false); 
  sprintf(bluetoothName, "%s", preferences.getString("Name", "ESP32Accel"));
  preferences.end();
  SerialCom.begin(bluetoothName);

#elif defined(ALTIMULTIESP32_ACCELERO_375)
  Serial.begin(38400);
  char bluetoothName [21];
  //sprintf(altiName, "ESP32A375_%i", (int)config.altiID );
  preferences.begin("namespace", false); 
  sprintf(bluetoothName, "%s", preferences.getString("Name", "ESP32A375"));
  preferences.end();
  SerialCom.begin(bluetoothName);

#elif defined(ALTIMULTIESP32_ACCELERO_345)
  Serial.begin(38400);
  char bluetoothName [21];
  //sprintf(altiName, "ESP32A345_%i", (int)config.altiID );
  preferences.begin("namespace", false); 
  sprintf(bluetoothName, "%s", preferences.getString("Name", "ESP32A345"));
  preferences.end();
  SerialCom.begin(bluetoothName);

#elif defined(ALTIDUOESP32)
  Serial.begin(38400);
  char altiName [21];
  sprintf(altiName, "ESP32Rocket%i", (int)config.altiID );
  SerialCom.begin(altiName);
#else
  SerialCom.begin(config.connectionSpeed);
#endif
  SerialCom.println("Start");

#if defined ALTIMULTIESP32_ACCELERO_375
  /* Initialise the sensor */
  if (!accel375.begin())
  {
    //There was a problem detecting the ADXL375 ... check your connections
    Serial.println("Ooops, no ADXL375 detected ... Check your wiring!");
  }
#endif

#if defined ALTIMULTIESP32_ACCELERO_345
  if (!accel345.begin(0x53))
  {
    //There was a problem detecting the ADXL375 ... check your connections
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
  } else {
    accel345.setRange(ADXL345_RANGE_16_G);
  }
#endif
#if defined ALTIMULTIESP32_ACCELERO
  if (!accel345.begin(0x53))
    //if (!accel345.begin(0x1D))
  {

    //There was a problem detecting the ADXL375 ... check your connections
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
  } else {
    accel345.setRange(ADXL345_RANGE_16_G);
    // init offsets to zero
    /*
      int16_t x, y, z;
      x = accel.getX();
      y = accel.getY();
      z = accel.getZ();
      accel345.setTrimOffsets(0, 0, 0);
      // the trim offsets are in 'multiples' of 8, we want to round, so we add 4
      accel.setTrimOffsets(-(x+4)/8,
                       -(y+4)/8,
                       -(z-250+4)/8);  // Z should be '250' at 1g (4mg per bit)*/

    delay (100);
  }

  /* Initialise the sensor */
  if (!accel375.begin(0x1D))
  {
    //There was a problem detecting the ADXL375 ... check your connections
    Serial.println("Ooops, no ADXL375 detected ... Check your wiring!");
    // while (1);

  } else {
    delay (100);
  }
#endif

  //  pinMode(A0, INPUT);
#ifdef ALTIMULTI
  //software pull up so that all bluetooth modules work!!! took me a good day to figure it out
  pinMode(PD0, INPUT_PULLUP);
#endif

  //software pull up so that all bluetooth modules work!!!
#ifdef ALTIMULTIV2
  pinMode(PD0, INPUT_PULLUP);
#endif
  //software pull up so that all bluetooth modules work!!!
#ifdef ALTIMULTISTM32
  pinMode(PB11, INPUT_PULLUP);
  pinMode(PA8, INPUT_PULLUP);
  //pinMode(PA9, INPUT_PULLUP);
#endif

  //Presure Sensor Initialisation
#ifdef BMP085_180
  // Note that BMP180 is compatible with the BMP085 library
  // Low res should work better at high speed
  SerialCom.print(F("sensor\n"));

  if (!bmp.begin( config.altimeterResolution)) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {
      shortBeep();
      delay(500);
      shortBeep();
      delay(500);
      longBeep();
      delay(500);
    }
  }
#endif

#ifdef BMP280
  bmp.begin();
  bmp.setOversampling(config.altimeterResolution)
#endif


  SerialCom.print(F("Start program\n"));

  //Initialise the output pin
  pinMode(pyroOut1, OUTPUT);
  pinMode(pyroOut2, OUTPUT);
#ifdef NBR_PYRO_OUT3
  pinMode(pyroOut3, OUTPUT);
#endif
  //some Altimeter such as the STM32 have 4 pyro outputs
#ifdef NBR_PYRO_OUT4
  pinMode(pyroOut3, OUTPUT);
  pinMode(pyroOut4, OUTPUT);
#endif
  pinMode(pinSpeaker, OUTPUT);

  pinMode(pinAltitude1, INPUT);
  pinMode(pinAltitude2, INPUT);

  pinMode(pinChannel1Continuity , INPUT);
  pinMode(pinChannel2Continuity , INPUT);
#ifdef NBR_PYRO_OUT3
  pinMode(pinChannel3Continuity , INPUT);
#endif
#ifdef NBR_PYRO_OUT4
  pinMode(pinChannel3Continuity , INPUT);
  pinMode(pinChannel4Continuity , INPUT);
#endif

  //Make sure that the output are turned off
  digitalWrite(pyroOut1, LOW);
  digitalWrite(pyroOut2, LOW);
#ifdef NBR_PYRO_OUT3
  digitalWrite(pyroOut3, LOW);
#endif
#ifdef NBR_PYRO_OUT4
  digitalWrite(pyroOut3, LOW);
  digitalWrite(pyroOut4, LOW);
#endif
  digitalWrite(pinSpeaker, LOW);

  //enable or disable continuity check
  if (config.noContinuity == 1)
    noContinuity = true;
  else
    noContinuity = false;

  SerialCom.print(F("before beep\n"));
  //initialisation give the version of the altimeter
  //One long beep per major number and One short beep per minor revision
  //For example version 1.2 would be one long beep and 2 short beep
  beepAltiVersion(MAJOR_VERSION, MINOR_VERSION);
  SerialCom.print(F("after beep\n"));
  if (!softConfigValid)
  {
    //initialise the deployement altitude for the main
    mainDeployAltitude = 100;

    // On the Alti duo when you close the jumper you set it to 1
    // val is the left jumper and val1 is the right jumper
    // as of version 1.4 only use the jumper if no valid softconfiguration

    val = digitalRead(pinAltitude1);
    val1 = digitalRead(pinAltitude2);
    if (val == 0 && val1 == 0)
    {
      mainDeployAltitude = 50;
    }
    if (val == 0 && val1 == 1)
    {
      mainDeployAltitude = 100;
    }
    if (val == 1 && val1 == 0)
    {
      mainDeployAltitude = 150;
    }
    if (val == 1 && val1 == 1)
    {
      mainDeployAltitude = 200;
    }
  }
  SerialCom.print(F("before reading altitude1\n"));
  // let's do some dummy altitude reading
  // to initialise the Kalman filter
  for (int i = 0; i < 50; i++) {
    ReadAltitude();
  }

  SerialCom.print(F("before reading altitude2\n"));
  //let's read the launch site altitude
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += ReadAltitude();
    delay(50);
  }
  SerialCom.print(F("after reading altitude\n"));
  initialAltitude = (sum / 10.0);
  lastAltitude = 0;//initialAltitude;
  liftoffAltitude = config.liftOffAltitude;//20;

  SerialCom.print(F("before reading eeprom\n"));
  int v_ret;
  v_ret = logger.readFlightList();
#ifdef SERIAL_DEBUG
  SerialCom.print(F("V_ret:"));
  SerialCom.println(v_ret);
#endif
  long lastFlightNbr = logger.getLastFlightNbr();
#ifdef SERIAL_DEBUG
  SerialCom.print(F("Last flight:"));
  SerialCom.println(lastFlightNbr);
#endif
  if (lastFlightNbr < 0)
  {
    currentFileNbr = 0;
    currentMemaddress = 201;
  }
  else
  {
    currentMemaddress = logger.getFlightStop(lastFlightNbr) + 1;
    currentFileNbr = lastFlightNbr + 1;
  }
  SerialCom.print(F("after  reading eeprom\n"));
  // check if eeprom is full
  canRecord = logger.CanRecord();
  if (!canRecord)
    SerialCom.println("Cannot record");
}


/*
   setEventState(int pyroOut, boolean state)
    Set the state of the output
*/
void setEventState(int pyroOut, boolean state)
{
  if (pyroOut == pyroOut1)
  {
    Output1Fired = state;
#ifdef SERIAL_DEBUG
    SerialCom.println(F("Output1Fired"));
#endif
  }

  if (pyroOut == pyroOut2)
  {
    Output2Fired = state;
#ifdef SERIAL_DEBUG
    SerialCom.println(F("Output2Fired"));
#endif
  }
#ifdef NBR_PYRO_OUT3
  if (pyroOut == pyroOut3)
  {
    Output3Fired = state;
#ifdef SERIAL_DEBUG
    SerialCom.println(F("Output3Fired"));
#endif
  }
#endif

#ifdef NBR_PYRO_OUT4
  if (pyroOut == pyroOut3)
  {
    Output3Fired = state;
#ifdef SERIAL_DEBUG
    SerialCom.println(F("Output3Fired"));
#endif
  }
  if (pyroOut == pyroOut4)
  {
    Output4Fired = state;
#ifdef SERIAL_DEBUG
    SerialCom.println(F("Output4Fired"));
#endif
  }
#endif
}
/*
   SendTelemetry(long sampleTime, int freq)
   Send telemety so that we can plot the flight

*/
void SendTelemetry(long sampleTime, int freq) {
  char altiTelem[150] = "";
  char temp[10] = "";

  if (telemetryEnable && (millis() - lastTelemetry) > freq) {
    lastTelemetry = millis();
    int val = 0;
    //check liftoff
    int li = 0;
    if (liftOff)
      li = 1;

    //check apogee
    int ap = 0;
    if (allApogeeFiredComplete)
      ap = 1;

    //check main
    int ma = 0;
    if (allMainFiredComplete)
      ma = 1;
    int landed = 0;
    if ( allMainFiredComplete && currAltitude < 10)
      landed = 1;

    strcat(altiTelem, "telemetry," );
    sprintf(temp, "%i,", currAltitude);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", li);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", ap);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", apogeeAltitude);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", ma);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", mainAltitude);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", landed);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", sampleTime);
    strcat(altiTelem, temp);
    if (config.outPut1 != 3) {
      //check continuity
      val = digitalRead(pinChannel1Continuity);
      if (val == 0)
        strcat(altiTelem, "0,");
      else
        strcat(altiTelem, "1,");
    }
    else {
      strcat(altiTelem, "-1,");
    }
    if (config.outPut2 != 3) {
      //check continuity
      val = digitalRead(pinChannel2Continuity);
      //delay(20);
      if (val == 0)
        strcat(altiTelem, "0,");
      else
        strcat(altiTelem, "1,");
    }
    else {
      strcat(altiTelem, "-1,");
    }
#ifndef ALTIDUOESP32
    if (config.outPut3 != 3) {
      //check continuity
      val = digitalRead(pinChannel3Continuity);
      if (val == 0)
        strcat(altiTelem, "0,");
      else
        strcat(altiTelem, "1,");
    }
    else {
      strcat(altiTelem, "-1,");
    }
#endif

#ifdef ALTIMULTISTM32
    if (config.outPut4 != 3) {
      //check continuity
      val = digitalRead(pinChannel4Continuity);
      if (val == 0)
        strcat(altiTelem, "0,");
      else
        strcat(altiTelem, "1,");
    }
    else {
      strcat(altiTelem, "-1,");
    }
#else
    strcat(altiTelem, "-1,");
#endif
#ifdef ALTIMULTISTM32
    pinMode(PB1, INPUT_ANALOG);
    int batVoltage = analogRead(PB1);
    float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
    //sprintf(temp, "%f,", bat);
    dtostrf(bat, 4, 2, temp);
    strcat(altiTelem, temp);
    strcat(altiTelem, ",");
#else
#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    int batVoltage = analogReadAdjusted(4);
    float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
    dtostrf(bat, 4, 2, temp);
    strcat(altiTelem, temp);
    strcat(altiTelem, ",");
#else
    strcat(altiTelem, "-1,");
#endif
#endif
    // temperature
    float temperature;
#ifdef BMP085_180
    temperature = bmp.readTemperature();
#endif
#ifdef BMP_180
    temperature = bmp.GetTemperature();
#endif
    sprintf(temp, "%i,", (int)temperature );
    strcat(altiTelem, temp);

    sprintf(temp, "%i,", (int)(100 * ((float)logger.getLastFlightEndAddress() / endAddress)) );
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", logger.getLastFlightNbr() + 1 );
    strcat(altiTelem, temp);

    //drogueFiredAltitude
    sprintf(temp, "%i,", drogueFiredAltitude);
    strcat(altiTelem, temp);

#if defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO
    //Get a new sensor event
    sensors_event_t event375;
    accel375.getEvent(&event375);
    sprintf(temp, "%i,", (int)(1000 * ((float)event375.acceleration.x)) );
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", (int)(1000 * ((float)event375.acceleration.y)) );
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", (int)(1000 * ((float)event375.acceleration.z)) );
    strcat(altiTelem, temp);
    //delay(10);
#endif
#ifdef ALTIMULTIESP32_ACCELERO_375
    sprintf(temp, "%i,", 0 );
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", 0 );
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", 0 );
    strcat(altiTelem, temp);
#endif

#ifdef ALTIMULTIESP32_ACCELERO_345
    sprintf(temp, "%i,", 0 );
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", 0 );
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", 0 );
    strcat(altiTelem, temp);
#endif

#if defined ALTIMULTIESP32_ACCELERO_345 || defined ALTIMULTIESP32_ACCELERO
    sensors_event_t event345;
    accel345.getEvent(&event345);
    sprintf(temp, "%i,", (int)(1000 * ((float)event345.acceleration.x)) );
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", (int)(1000 * ((float)event345.acceleration.y)) );
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", (int)(1000 * ((float)event345.acceleration.z)) );
    strcat(altiTelem, temp);
    //delay(10);
#endif


    unsigned int chk;
    chk = msgChk(altiTelem, sizeof(altiTelem));
    sprintf(temp, "%i", chk);
    strcat(altiTelem, temp);
    strcat(altiTelem, ";\n");

    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
#ifdef TELEMETRY_ESP32
    Serial.print("$");
    Serial.print(altiTelem);
#endif
    SerialCom.print("$");
    SerialCom.print(altiTelem);
  }
}

//================================================================
// Main loop which call the menu
//================================================================
void loop()
{
  MainMenu();
}
/*
   Calculate the current velocity
*/
int currentVelocity(long prevTime, long curTime, int prevAltitude, int curAltitude)
{
  int curSpeed = int (float(curAltitude - prevAltitude) / (float( curTime - prevTime) / 1000));
  return curSpeed;
}

//================================================================
// Function:  recordAltitude()
// called for normal recording
//================================================================
void recordAltitude()
{
  ResetGlobalVar();

  boolean OutputFiredComplete[4] = {false, false, false, false};
  int OutputDelay[4] = {0, 0, 0, 0};
  OutputDelay[0] = config.outPut1Delay;
  OutputDelay[1] = config.outPut2Delay;
  OutputDelay[2] = config.outPut3Delay;
#ifdef ALTIMULTISTM32
  OutputDelay[3] = config.outPut4Delay;
#endif
  // 0 = main 1 = drogue 2 = timer 4 = landing 5 = liftoff 3 = disable 6 = altitude
  int OutputType[4] = {3, 3, 3, 3};
  OutputType[0] = config.outPut1;
  OutputType[1] = config.outPut2;
#ifndef NBR_PYRO_OUT2
  OutputType[2] = config.outPut3;
#endif
#ifdef ALTIMULTISTM32
  OutputType[3] = config.outPut4;
#endif
  int OutputPins[4] = { -1, -1, -1, -1};
  if (config.outPut1 != 3)
    OutputPins[0] = pyroOut1;
  if (config.outPut2 != 3)
    OutputPins[1] = pyroOut2;
#ifndef NBR_PYRO_OUT2
  if (config.outPut3 != 3)
    OutputPins[2] = pyroOut3;
#endif
#ifdef ALTIMULTISTM32
  if (config.outPut4 != 3)
    OutputPins[3] = pyroOut4;
#endif



  boolean apogeeReadyToFire = false;
  boolean mainReadyToFire = false;
  boolean landingReadyToFire = false;
  boolean liftOffReadyToFire = false;
  unsigned long apogeeStartTime = 0;
  unsigned long mainStartTime = 0;
  unsigned long landingStartTime = 0;
  unsigned long liftOffStartTime = 0;
  boolean ignoreAltiMeasure = false;
  unsigned long altitudeStartTime[] = {0, 0, 0, 0};

  boolean liftOffHasFired = false;
  //hold the state of all our outputs
  boolean outputHasFired[4] = {false, false, false, false};

  if (config.outPut1 == 3) Output1Fired = true;
  if (config.outPut2 == 3) Output2Fired = true;
#ifdef NBR_PYRO_OUT3
  if (config.outPut3 == 3) Output3Fired = true;
#endif
#ifdef NBR_PYRO_OUT4
  if (config.outPut3 == 3) Output3Fired = true;
  if (config.outPut4 == 3) Output4Fired = true;
#endif

#ifdef SERIAL_DEBUG
  SerialCom.println(F("Config delay:"));
  SerialCom.println(config.outPut1Delay);
  SerialCom.println(config.outPut2Delay);
#ifdef NBR_PYRO_OUT3
  SerialCom.println(config.outPut3Delay);
#endif
#ifdef NBR_PYRO_OUT4
  SerialCom.println(config.outPut3Delay);
  SerialCom.println(config.outPut4Delay);
#endif

#endif

  while (!exitRecording)
  {
    //read current altitude
    currAltitude = (ReadAltitude() - initialAltitude);

    if (( currAltitude > liftoffAltitude) && !liftOff && !allMainFiredComplete)
    {
      liftOff = true;
      SendTelemetry(0, 200);
      // save the time
      initialTime = millis();
      if (config.superSonicYesNo == 1)
        ignoreAltiMeasure = true;

#ifdef SERIAL_DEBUG
      SerialCom.println(F("we have lift off\n"));
#endif
      if (canRecord)
      {
        //Save start address
        logger.setFlightStartAddress (currentFileNbr, currentMemaddress);
#ifdef SERIAL_DEBUG
        SerialCom.println(F("Save start address\n"));
#endif
      }

    }
    unsigned long prevTime = 0;
    long prevAltitude = 0;
    // loop until we have reach an altitude of 3 meter
    while (liftOff)
    {
      unsigned long currentTime;
      unsigned long diffTime;

      currAltitude = (ReadAltitude() - initialAltitude);

      currentTime = millis() - initialTime;
      if (allMainFiredComplete && !allLandingFiredComplete && !landingReadyToFire && currAltitude < 10) {

        if (abs(currentVelocity(prevTime, currentTime, prevAltitude, currAltitude)) < 1  ) {
          //we have landed
          landingReadyToFire = true;
          landingStartTime = millis();
        }
      }
      prevAltitude = currAltitude;
      SendTelemetry(currentTime, 200);
      diffTime = currentTime - prevTime;
      prevTime = currentTime;

      if (!liftOffHasFired && !liftOffReadyToFire) {
        liftOffReadyToFire = true;
        liftOffStartTime = millis();
      }

      if (!allLiftOffFiredComplete) {
        //fire all liftoff that are ready
        for (int li = 0; li < 4; li++ ) {
          if (!outputHasFired[li] && ((millis() - liftOffStartTime) >= OutputDelay[li] ) && OutputType[li] == 5) {
            digitalWrite(OutputPins[li], HIGH);
            outputHasFired[li] = true;
          }
        }
        for (int li = 0; li < 4; li++ ) {
          if ((millis() - liftOffStartTime ) >= (1000 + OutputDelay[li])  && !OutputFiredComplete[li] && OutputType[li] == 5)
          {
            digitalWrite(OutputPins[li], LOW);
            setEventState(OutputPins[li], true);
            OutputFiredComplete[li] = true;
          }
        }

        allLiftOffFiredComplete = true;

        for (int li = 0; li < 4; li++ ) {
          if (!OutputFiredComplete[li] && OutputType[li] == 5)
          {
            allLiftOffFiredComplete = false;
          }
        }
        SendTelemetry(millis() - initialTime, 200);
      }

      //altitude events
      if (!allAltitudeFiredComplete) {
        //fire all altitude that are ready
        for (int al = 0; al < 4; al++ ) {
          if (!outputHasFired[al] && ((currAltitude >= OutputDelay[al]) ) && OutputType[al] == 6) {
            digitalWrite(OutputPins[al], HIGH);
            outputHasFired[al] = true;
            altitudeStartTime[al] = millis();
          }
        }
        for (int al = 0; al < 4; al++ ) {
          if (( millis()  >= (1000 + altitudeStartTime[al]))  && !OutputFiredComplete[al] && OutputType[al] == 6 && outputHasFired[al])
          {
            digitalWrite(OutputPins[al], LOW);
            setEventState(OutputPins[al], true);
            OutputFiredComplete[al] = true;
          }
        }

        allAltitudeFiredComplete = true;

        for (int al = 0; al < 4; al++ ) {
          if (!OutputFiredComplete[al] && OutputType[al] == 6)
          {
            allAltitudeFiredComplete = false;
          }
        }
        SendTelemetry(millis() - initialTime, 200);
      }
      // timer events
      if (!allTimerFiredComplete) {
        //fire all timers that are ready
        for (int ti = 0; ti < 4; ti++ ) {
          if (!outputHasFired[ti] && ((currentTime >= OutputDelay[ti]) ) && OutputType[ti] == 2) {
            digitalWrite(OutputPins[ti], HIGH);
            outputHasFired[ti] = true;
          }
        }
        for (int ti = 0; ti < 4; ti++ ) {
          if ((currentTime  >= (1000 + OutputDelay[ti]))  && !OutputFiredComplete[ti] && OutputType[ti] == 2)
          {
            digitalWrite(OutputPins[ti], LOW);
            setEventState(OutputPins[ti], true);
            OutputFiredComplete[ti] = true;
          }
        }

        allTimerFiredComplete = true;

        for (int ti = 0; ti < 4; ti++ ) {
          if (!OutputFiredComplete[ti] && OutputType[ti] == 2)
          {
            allTimerFiredComplete = false;
          }
        }
        SendTelemetry(millis() - initialTime, 200);
      }
      if (canRecord)
      {
        logger.setFlightTimeData( diffTime);
        logger.setFlightAltitudeData(currAltitude);
        logger.setFlightTemperatureData((long) bmp.readTemperature());
        logger.setFlightPressureData((long) bmp.readPressure());
#ifdef LOG_VOLTAGE

#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
        float bat = VOLT_DIVIDER * ((float)(analogReadAdjusted(4) * 3300) / (float)4096000);
        logger.setFlightVoltageData((long) 100 * bat);
#endif
#ifdef ALTIMULTISTM32
        float bat = VOLT_DIVIDER * ((float)(analogRead(PB1) * 3300) / (float)4096000);
        logger.setFlightVoltageData((long) 100 * bat);
#endif
#endif

#if defined ALTIMULTIESP32_ACCELERO_375 ||defined ALTIMULTIESP32_ACCELERO
        // log accelerometers
        sensors_event_t event375;
        accel375.getEvent(&event375);
        logger.setADXL375accelX((long) 1000 * event375.acceleration.x);
        logger.setADXL375accelY((long) 1000 * event375.acceleration.y);
        logger.setADXL375accelZ((long) 1000 * event375.acceleration.z);
#endif
#ifdef ALTIMULTIESP32_ACCELERO_375
        logger.setADXL345accelX(0);
        logger.setADXL345accelY(0);
        logger.setADXL345accelZ(0);
#endif
#ifdef ALTIMULTIESP32_ACCELERO_345
        logger.setADXL375accelX(0);
        logger.setADXL375accelY(0);
        logger.setADXL375accelZ(0);
#endif
#if defined ALTIMULTIESP32_ACCELERO_345 ||defined ALTIMULTIESP32_ACCELERO
        sensors_event_t event345;
        accel345.getEvent(&event345);
        logger.setADXL345accelX((long) 1000 * event345.acceleration.x);
        logger.setADXL345accelY((long) 1000 * event345.acceleration.y);
        logger.setADXL345accelZ((long) 1000 * event345.acceleration.z);
#endif

        if ( (currentMemaddress + logger.getSizeOfFlightData())  > endAddress) {
          //flight is full let's save it
          //save end address
          logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
          canRecord = false;
        } else {
          currentMemaddress = logger.writeFastFlight(currentMemaddress);
          currentMemaddress++;
        }

#if defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
        //delay(50); ///?????????????
#endif

      }
      if (config.superSonicYesNo == 1)
      {
        //are we still in superSonic mode?
        if (currentTime > 3000)
          ignoreAltiMeasure = false;
      }
      if ((currAltitude < lastAltitude) && !apogeeReadyToFire  && !ignoreAltiMeasure )
      {
        measures = measures - 1;
        if (measures == 0)
        {
          //fire drogue
          apogeeReadyToFire = true;
          apogeeStartTime = millis();
          drogueFiredAltitude = currAltitude;
          apogeeAltitude = lastAltitude;
        }
      }
      else
      {
        lastAltitude = currAltitude;
        measures = config.nbrOfMeasuresForApogee;
      }
      if (apogeeReadyToFire && !allApogeeFiredComplete)
      {
        //fire all drogues if delay ok
        for (int ap = 0; ap < 4; ap++ ) {
          if (!outputHasFired[ap] && ((millis() - apogeeStartTime) >= OutputDelay[ap]) && OutputType[ap] == 1) {
            digitalWrite(OutputPins[ap], HIGH);
            outputHasFired[ap] = true;
          }
        }

        for (int ap = 0; ap < 4; ap++ ) {
          if ((millis() - apogeeStartTime ) >= (1000 + OutputDelay[ap]) && !OutputFiredComplete[ap] && OutputType[ap] == 1)
          {
            digitalWrite(OutputPins[ap], LOW);
            setEventState(OutputPins[ap], true);
            OutputFiredComplete[ap] = true;
          }
        }

        allApogeeFiredComplete = true;

        for (int ap = 0; ap < 4; ap++ ) {
          if (!OutputFiredComplete[ap] && OutputType[ap] == 1)
          {
            allApogeeFiredComplete = false;
          }
        }
        SendTelemetry(millis() - initialTime, 200);
      }
      if ((currAltitude  < mainDeployAltitude) && allApogeeFiredComplete && !mainReadyToFire && !allMainFiredComplete)
      {
        // Deploy main chute  X meters or feet  before landing...
        mainReadyToFire = true;
#ifdef SERIAL_DEBUG
        SerialCom.println(F("preparing main"));
#endif
        mainStartTime = millis();

        mainAltitude = currAltitude;
#ifdef SERIAL_DEBUG
        SerialCom.println(F("main altitude"));
        SerialCom.println(mainAltitude);
#endif
      }
      if (mainReadyToFire && !allMainFiredComplete)
      {
        //fire main
#ifdef SERIAL_DEBUG
        SerialCom.println(F("firing main"));
#endif
        for (int ma = 0; ma < 4; ma++ ) {
          if (!outputHasFired[ma] && ((millis() - mainStartTime) >= OutputDelay[ma]) && OutputType[ma] == 0) {
            digitalWrite(OutputPins[ma], HIGH);
            outputHasFired[ma] = true;
          }
        }


        for (int ma = 0; ma < 4; ma++ ) {
          if ((millis() - mainStartTime ) >= (1000 + OutputDelay[ma]) && !OutputFiredComplete[ma] && OutputType[ma] == 0)
          {
            digitalWrite(OutputPins[ma], LOW);
            setEventState(OutputPins[ma], true);
            OutputFiredComplete[ma] = true;
          }
        }
        allMainFiredComplete = true;

        for (int ma = 0; ma < 4; ma++ ) {
          if (!OutputFiredComplete[ma] && OutputType[ma] == 0)
          {
            allMainFiredComplete = false;
          }
        }
        SendTelemetry(millis() - initialTime, 200);
      }



      if (landingReadyToFire && !allLandingFiredComplete) {
        //fire all landing that are ready
        for (int la = 0; la < 4; la++ ) {
          if (!outputHasFired[la] && ((millis() - landingStartTime) >= OutputDelay[la] ) && OutputType[la] == 4) {
            digitalWrite(OutputPins[la], HIGH);
            outputHasFired[la] = true;
          }
        }
        for (int la = 0; la < 4; la++ ) {
          if ((millis() - landingStartTime ) >= (1000 + OutputDelay[la])  && !OutputFiredComplete[la] && OutputType[la] == 4)
          {
            digitalWrite(OutputPins[la], LOW);
            setEventState(OutputPins[la], true);
            OutputFiredComplete[la] = true;
          }
        }

        allLandingFiredComplete = true;

        for (int la = 0; la < 4; la++ ) {
          if (!OutputFiredComplete[la] && OutputType[la] == 4)
          {
            allLandingFiredComplete = false;
          }
        }
        SendTelemetry(millis() - initialTime, 200);
      }
      //if ((canRecord && MainFiredComplete && (currAltitude < 10)) || (canRecord && MainFiredComplete && (millis() - mainStartTime) > recordingTimeOut))
      if ((canRecord && allMainFiredComplete && (currAltitude < 10) && allLandingFiredComplete) || (canRecord && allMainFiredComplete && (millis() - mainStartTime) > recordingTimeOut))
      {
        //liftOff =false;
        //end loging
        //store start and end address

        //save end address
        logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
#ifdef SERIAL_DEBUG
        SerialCom.println(F("stop recording\n "));
        SerialCom.println(currentMemaddress);
#endif
        logger.writeFlightList();
      }

      if ((allMainFiredComplete && (currAltitude < 10) && allLandingFiredComplete) || (allMainFiredComplete && (millis() - mainStartTime) > recordingTimeOut))
      {
#ifdef SERIAL_DEBUG
        SerialCom.println(F("main fired complete"));
#endif
        liftOff = false;
        SendTelemetry(millis() - initialTime, 200);
        //exitRecording = true;
        // we have landed telemetry is not required anymore
        telemetryEnable = false;
      }
      /*#ifdef NBR_PYRO_OUT4
            if (Output1Fired == true && Output2Fired == true && Output3Fired == true && Output4Fired == true && LandingFiredComplete)
        #else
            if (Output1Fired == true && Output2Fired == true && Output3Fired == true && LandingFiredComplete)
        #endif*/
      if (allLandingFiredComplete)
      {
#ifdef SERIAL_DEBUG
        SerialCom.println(F("all event have fired"));
#endif
        exitRecording = true;
        SendTelemetry(millis() - initialTime, 200);
      }
    } // end while (liftoff)
  } //end while(recording)
}




//================================================================
// Main menu to interpret all the commands sent by the altimeter console
//================================================================
void MainMenu()
{
  char readVal = ' ';
  int i = 0;

  char commandbuffer[100];

  /* SerialCom.println(F("Rocket flight data logger. A maximum of 25 flight can be logged \n"));
    SerialCom.println(F("Commands are: \n"));
    SerialCom.println(F("w = record flight \n"));
    SerialCom.println(F("r (followed by the flight number) = read flight data\n"));
    SerialCom.println(F("l  = print flight list \n"));
    SerialCom.println(F("e  = erase all flight data \n"));
    SerialCom.println(F("c  = toggle continuity on/off \n"));
    SerialCom.println(F("b  = print alti config \n"));
    SerialCom.println(F("Enter Command and terminate it by a ; >>\n"));*/
  i = 0;
  readVal = ' ';
  while ( readVal != ';')
  {
    if (!FastReading)
    {
      currAltitude = (ReadAltitude() - initialAltitude);
      if (liftOff)
        SendTelemetry(millis() - initialTime, 200);
      if (!( currAltitude > liftoffAltitude) )
      {
        //continuityCheckNew();
        continuityCheckAsync();
        SendTelemetry(0, 500);
        checkBatVoltage(BAT_MIN_VOLTAGE);
      }
      else
      {
        recordAltitude();
      }
      long savedTime = millis();
      while (allApogeeFiredComplete  && allMainFiredComplete )
      {
        // check if we have anything on the serial port
#ifdef TELEMETRY_ESP32
        //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
        if (Serial.available())
        {
          readVal = Serial.read();
          if (readVal != ';' )
          {
            if (readVal != '\n')
              commandbuffer[i++] = readVal;
          }
          else
          {
            commandbuffer[i++] = '\0';
            resetFlight();
            interpretCommandBuffer(commandbuffer);
          }
        }
#endif
        if (SerialCom.available())
        {
          readVal = SerialCom.read();
          if (readVal != ';' )
          {
            if (readVal != '\n')
              commandbuffer[i++] = readVal;
          }
          else
          {
            commandbuffer[i++] = '\0';
            resetFlight();
            interpretCommandBuffer(commandbuffer);
          }
        }


        //beep last altitude every 10 second
        while ((millis() - savedTime) > 10000) {

          beginBeepSeq();

          if (config.beepingMode == 0)
            beepAltitude(apogeeAltitude * FEET_IN_METER);
          else
            beepAltitudeNew(apogeeAltitude * FEET_IN_METER);
          beginBeepSeq();

          if (config.beepingMode == 0)
            beepAltitude(mainAltitude * FEET_IN_METER);
          else
            beepAltitudeNew(mainAltitude * FEET_IN_METER);

          savedTime = millis();
        }
      }
    }

#ifdef TELEMETRY_ESP32
    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    while (Serial.available())
    {
      readVal = Serial.read();
      if (readVal != ';' )
      {
        if (readVal != '\n')
          commandbuffer[i++] = readVal;
      }
      else
      {
        commandbuffer[i++] = '\0';
        break;
      }
    }
#endif
    while (SerialCom.available())
    {
      readVal = SerialCom.read();
      if (readVal != ';' )
      {
        if (readVal != '\n')
          commandbuffer[i++] = readVal;
      }
      else
      {
        commandbuffer[i++] = '\0';
        break;
      }
    }

  }
  interpretCommandBuffer(commandbuffer);
}


/*

   This interprets menu commands. This can be used in the commend line or
   this is used by the Android console

   Commands are as folow:
   a  get all flight data
   b  get altimeter config
   c  toggle continuity on and off
   d  reset alti config
   e  erase all saved flights
   f  FastReading on
   g  FastReading off
   h  hello. Does not do much
   i  unused
   k  folowed by a number turn on or off the selected output
   l  list all flights
   m  followed by a number turn main loop on/off. if number is 1 then
      main loop in on else turn it off
   n  Return the number of recorded flights in the EEprom
   o  requesting test trame
   r  followed by a number which is the flight number.
      This will retrieve all data for the specified flight
   s  write altimeter config
   t  reset alti config (why?)
   w  Start or stop recording
   x  delete last curve
   y  followed by a number turn telemetry on/off. if number is 1 then
      telemetry in on else turn it off
   z  send gps raw data

*/
void interpretCommandBuffer(char *commandbuffer) {
  //get all flight data
  if (commandbuffer[0] == 'a')
  {
#ifdef TELEMETRY_ESP32
    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    Serial.print(F("$start;\n"));
#endif
    SerialCom.print(F("$start;\n"));
    int i;
    ///todo
    for (i = 0; i < logger.getLastFlightNbr() + 1; i++)
    {
      logger.printFlightData(i);
    }
#ifdef TELEMETRY_ESP32
    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    Serial.print(F("$end;\n"));
#endif
    SerialCom.print(F("$end;\n"));
  }
  //get altimeter config
  else if (commandbuffer[0] == 'b')
  {
#ifdef TELEMETRY_ESP32
    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    Serial.print(F("$start;\n"));
#endif
    SerialCom.print(F("$start;\n"));
#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    char altiName [15];
    preferences.begin("namespace", false);
    sprintf(altiName, "%s", preferences.getString("Name", "ESP32Rocket"));
    preferences.end();
    printAltiConfig(altiName);
#else
    printAltiConfig("");
#endif
#ifdef TELEMETRY_ESP32
    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    Serial.print(F("$end;\n"));
#endif
    SerialCom.print(F("$end;\n"));
  }
  //toggle continuity on and off
  else if (commandbuffer[0] == 'c')
  {
    if (noContinuity == false)
    {
      noContinuity = true;
      SerialCom.println(F("Continuity off \n"));
    }
    else
    {
      noContinuity = false;
      SerialCom.println(F("Continuity on \n"));
    }
  }
  //reset alti config this is equal to t why do I have 2 !!!!
  else if (commandbuffer[0] == 'd')
  {
    defaultConfig();
    writeConfigStruc();
    initAlti();
  }
  //this will erase all flight
  else if (commandbuffer[0] == 'e')
  {
    SerialCom.println(F("Erase\n"));
    logger.clearFlightList();
    logger.writeFlightList();
    currentFileNbr = 0;
    currentMemaddress = 201;
  }
  //FastReading
  else if (commandbuffer[0] == 'f')
  {
    FastReading = true;
#ifdef TELEMETRY_ESP32
    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    Serial.print(F("$OK;\n"));
#endif
    SerialCom.print(F("$OK;\n"));

  }
  //FastReading off
  else if (commandbuffer[0] == 'g')
  {
    FastReading = false;
#ifdef TELEMETRY_ESP32
    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    Serial.print(F("$OK;\n"));
#endif
    SerialCom.print(F("$OK;\n"));
  }
  //hello
  else if (commandbuffer[0] == 'h')
  {
#ifdef TELEMETRY_ESP32
    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    Serial.print(F("$OK;\n"));
#endif
    SerialCom.print(F("$OK;\n"));
  }
  // unused
  else if (commandbuffer[0] == 'i')
  {
    //exit continuity mode
  }
  //turn on or off the selected output
  else if (commandbuffer[0] == 'k')
  {
    char temp[2];
    boolean fire = true;

    temp[0] = commandbuffer[1];
    temp[1] = '\0';
    if (commandbuffer[2] == 'F')
      fire = false;

    if (atol(temp) > -1)
    {
      switch (atoi(temp))
      {
        case 1:
          fireOutput(pyroOut1, fire);
          break;
        case 2:
          fireOutput(pyroOut2, fire);
          break;
#ifndef NBR_PYRO_OUT2
        case 3:
          fireOutput(pyroOut3, fire);
          break;
#endif
#ifdef NBR_PYRO_OUT4
        case 4:
          fireOutput(pyroOut4, fire);
          break;
#endif
      }
    }
  }
  //list all flights
  else if (commandbuffer[0] == 'l')
  {
    SerialCom.println(F("Flight List: \n"));
    logger.printFlightList();
  }

  //mainloop on/off
  else if (commandbuffer[0] == 'm')
  {
    if (commandbuffer[1] == '1') {
#ifdef SERIAL_DEBUG
      SerialCom.print(F("main Loop enabled\n"));
#endif
      //mainLoopEnable = true;
    }
    else {
#ifdef SERIAL_DEBUG
      SerialCom.print(F("main loop disabled\n"));
#endif
      //mainLoopEnable = false;
    }
#ifdef TELEMETRY_ESP32
    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    Serial.print(F("$OK;\n"));
#endif
    SerialCom.print(F("$OK;\n"));
  }
  //Number of flight
  else if (commandbuffer[0] == 'n')
  {
    char flightData[30] = "";
    char temp[9] = "";

#ifdef TELEMETRY_ESP32
    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    Serial.print(F("$start;\n"));
#endif
    SerialCom.print(F("$start;\n"));

    strcat(flightData, "nbrOfFlight,");
    sprintf(temp, "%i,", logger.getLastFlightNbr() + 1 );
    strcat(flightData, temp);
    unsigned int chk = msgChk(flightData, sizeof(flightData));
    sprintf(temp, "%i", chk);
    strcat(flightData, temp);
    strcat(flightData, ";\n");

#ifdef TELEMETRY_ESP32
    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    Serial.print("$");
    Serial.print(flightData);
    Serial.print(F("$end;\n"));
#endif
    SerialCom.print("$");
    SerialCom.print(flightData);
    SerialCom.print(F("$end;\n"));


  }
  // send test tram
  else if (commandbuffer[0] == 'o')
  {

#ifdef TELEMETRY_ESP32
    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    Serial.print(F("$start;\n"));
#endif
    SerialCom.print(F("$start;\n"));

    sendTestTram();
#ifdef TELEMETRY_ESP32
    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    Serial.print(F("$end;\n"));
#endif
    SerialCom.print(F("$end;\n"));
  }
  //altimeter config param
  //write  config
  else if (commandbuffer[0] == 'p')
  {
    if (writeAltiConfigV2(commandbuffer)) {
#ifdef TELEMETRY_ESP32
      //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
      Serial.print(F("$OK;\n"));
#endif
      SerialCom.print(F("$OK;\n"));
    }
    else {
#ifdef TELEMETRY_ESP32
      //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
      Serial.print(F("$KO;\n"));
#endif
      SerialCom.print(F("$KO;\n"));
    }
  }
  else if (commandbuffer[0] == 'q')
  {
    writeConfigStruc();
    readAltiConfig();
    initAlti();
#ifdef TELEMETRY_ESP32
    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    Serial.print(F("$OK;\n"));
#endif
    SerialCom.print(F("$OK;\n"));
  }
  //this will read one flight
  else if (commandbuffer[0] == 'r')
  {
    char temp[3];

    temp[0] = commandbuffer[1];
    if (commandbuffer[2] != '\0')
    {
      temp[1] = commandbuffer[2];
      temp[2] = '\0';
    }
    else
      temp[1] = '\0';

    if (atol(temp) > -1)
    {
#ifdef TELEMETRY_ESP32
      //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
      Serial.print(F("$start;\n"));
#endif
      SerialCom.print(F("$start;\n"));

      logger.printFlightData(atoi(temp));

#ifdef TELEMETRY_ESP32
      //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
      Serial.print(F("$end;\n"));
#endif
      SerialCom.print(F("$end;\n"));
    }
    else
      SerialCom.println(F("not a valid flight"));
  }
  //write altimeter config
  else if (commandbuffer[0] == 's')
  {
    /*if (writeAltiConfig(commandbuffer)) {

      SerialCom.print(F("$OK;\n"));
      readAltiConfig();
      initAlti();
      }
      else {
      SerialCom.print(F("$KO;\n"));
      //readAltiConfig();
      //initAlti();
      }*/
  }
  //reset config and set it to default
  else if (commandbuffer[0] == 't')
  {
    //reset config
    defaultConfig();
    writeConfigStruc();
    initAlti();
    SerialCom.print(F("config reseted\n"));
  }
  // Recording
  else if (commandbuffer[0] == 'w')
  {
    SerialCom.println(F("Recording \n"));
    recordAltitude();
  }
  //delete last curve
  else if (commandbuffer[0] == 'x')
  {
    logger.eraseLastFlight();
    logger.readFlightList();
    long lastFlightNbr = logger.getLastFlightNbr();
    if (lastFlightNbr < 0)
    {
      currentFileNbr = 0;
      currentMemaddress = 201;
    }
    else
    {
      currentMemaddress = logger.getFlightStop(lastFlightNbr) + 1;
      currentFileNbr = lastFlightNbr + 1;
    }
    canRecord = logger.CanRecord();
  }

  //telemetry on/off
  else if (commandbuffer[0] == 'y')
  {
    if (commandbuffer[1] == '1') {
      SerialCom.print(F("Telemetry enabled\n"));
      telemetryEnable = true;
    }
    else {
      SerialCom.print(F("Telemetry disabled\n"));
      telemetryEnable = false;
    }
#ifdef TELEMETRY_ESP32
    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    Serial.print(F("$OK;\n"));
#endif
    SerialCom.print(F("$OK;\n"));
  }
  //alti Name for ESP32
  else if (commandbuffer[0] == 'z')
  {
    updateAltiName(commandbuffer);
#ifdef TELEMETRY_ESP32
    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    Serial.print(F("$OK;\n"));
#endif
    SerialCom.print(F("$OK;\n"));
  }
  // empty command
  else if (commandbuffer[0] == ' ')
  {
#ifdef TELEMETRY_ESP32
    //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
    Serial.print(F("$K0;\n"));
#endif
    SerialCom.print(F("$K0;\n"));
  }
  else
  {
    SerialCom.print(F("$UNKNOWN;"));
    SerialCom.println(commandbuffer[0]);
  }
}
/*

   re-nitialise all flight related global variables

*/
void resetFlight() {
  recordingTimeOut = config.recordingTimeout * 1000;
  allApogeeFiredComplete  = false;
  allMainFiredComplete = false;
  allTimerFiredComplete = false;
  allLiftOffFiredComplete = false;
  allLandingFiredComplete = false;
  liftOff = false;
  Output1Fired = false;
  Output2Fired = false;
#ifdef NBR_PYRO_OUT4
  Output3Fired = false;
#endif
#ifdef NBR_PYRO_OUT4
  Output3Fired = false;
  Output4Fired = false;
#endif

  apogeeAltitude = 0;
  logger.readFlightList();
  long lastFlightNbr = logger.getLastFlightNbr();
  if (lastFlightNbr < 0)
  {
    currentFileNbr = 0;
    currentMemaddress = 201;
  }
  else
  {
    currentMemaddress = logger.getFlightStop(lastFlightNbr) + 1;
    currentFileNbr = lastFlightNbr + 1;
  }
  canRecord = logger.CanRecord();
}

/*

   Check if the battery voltage is OK.
   If not warn the user so that the battery does not get
   damaged by over discharging
*/
void checkBatVoltage(float minVolt) {
#ifdef ALTIMULTISTM32
  if ((millis() - lastBattWarning) > 10000) {
    lastBattWarning = millis();
    pinMode(PB1, INPUT_ANALOG);
    int batVoltage = analogRead(PB1);

    float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);

    if (bat < minVolt) {
      for (int i = 0; i < 10; i++)
      {
        tone(pinSpeaker, 1600, 1000);
        delay(50);
        noTone(pinSpeaker);
      }
      delay(1000);
    }
  }
#endif

#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
  if ((millis() - lastBattWarning) > 10000) {
    lastBattWarning = millis();

    double batVoltage = analogReadAdjusted(4);

    float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);


    if (bat < minVolt) {
      for (int i = 0; i < 10; i++)
      {
        tone(pinSpeaker, 1600, 1000);
        noTone(pinSpeaker);
        delay(50);
      }
      delay(1000);
    }
  }
#endif

}

#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
double analogReadAdjusted(byte pinNumber) {

  // Specify the adjustment factors.
  const double f1 = 1.7111361460487501e+001;
  const double f2 = 4.2319467860421662e+000;
  const double f3 = -1.9077375643188468e-002;
  const double f4 = 5.4338055402459246e-005;
  const double f5 = -8.7712931081088873e-008;
  const double f6 = 8.7526709101221588e-011;
  const double f7 = -5.6536248553232152e-014;
  const double f8 = 2.4073049082147032e-017;
  const double f9 = -6.7106284580950781e-021;
  const double f10 = 1.1781963823253708e-024;
  const double f11 = -1.1818752813719799e-028;
  const double f12 = 5.1642864552256602e-033;

  // Specify the number of loops for one measurement.
  const int loops = 40;

  // Specify the delay between the loops.
  const int loopDelay = 1;

  // Initialize the used variables.
  int counter = 1;
  int inputValue = 0;
  double totalInputValue = 0;
  double averageInputValue = 0;

  // Loop to get the average of different analog values.
  for (counter = 1; counter <= loops; counter++) {

    // Read the analog value.
    inputValue = analogRead(pinNumber);

    // Add the analog value to the total.
    totalInputValue += inputValue;

    // Wait some time after each loop.
    delay(loopDelay);
  }

  // Calculate the average input value.
  averageInputValue = totalInputValue / loops;

  // Calculate and return the adjusted input value.
  return f1 + f2 * pow(averageInputValue, 1) + f3 * pow(averageInputValue, 2) + f4 * pow(averageInputValue, 3) + f5 * pow(averageInputValue, 4) + f6 * pow(averageInputValue, 5) + f7 * pow(averageInputValue, 6) + f8 * pow(averageInputValue, 7) + f9 * pow(averageInputValue, 8) + f10 * pow(averageInputValue, 9) + f11 * pow(averageInputValue, 10) + f12 * pow(averageInputValue, 11);
}
#endif
/*
   Turn on or off one altimeter output
   This is used to test them
*/
void fireOutput(int pin, boolean fire) {
  if (fire)
    digitalWrite(pin, HIGH);
  else
    digitalWrite(pin, LOW);
}

/*
    Test tram
*/
void sendTestTram() {

  char altiTest[100] = "";
  char temp[10] = "";

  strcat(altiTest, "testTrame," );
  strcat(altiTest, "Bear altimeters are the best!!!!,");
  unsigned int chk;
  chk = msgChk(altiTest, sizeof(altiTest));
  sprintf(temp, "%i", chk);
  strcat(altiTest, temp);
  strcat(altiTest, ";\n");

#ifdef TELEMETRY_ESP32
  //#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
  Serial.print("$");
  Serial.print(altiTest);
#endif
  SerialCom.print("$");
  SerialCom.print(altiTest);
}

#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
void updateAltiName(char *commandbuffer) {
  int i;
  char temp[25];
  char altiName[21];
  strcpy(temp, commandbuffer);

  for (i = 2; i < strlen(temp); i++)
  {
    if (temp[i] == ',')
      break;
    altiName[i - 2] = temp[i];
  }
  preferences.begin("namespace", false);
  preferences.putString("Name", altiName);
  preferences.end();
}
#endif
