/*
  Rocket Flight Logger ver 1.22
  Copyright Boris du Reau 2012-2021

  The following is a datalogger for logging rocket flight.
  So far it can log the rocket altitude during the flight.
  The code works with different boards, you need to adjust the config.h file to reflect your board.


  This is using a BMP085 or BMP180 presure sensor and an Atmega 328 or an STM32
  The following record the flight in an EEPROM


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
*/

//altimeter configuration lib
#include "config.h"
#include <Wire.h> //I2C library


#ifdef BMP085_180
#include <Adafruit_BMP085.h>
#endif

#ifdef BMP280
#include <BMP280.h>
#define P0 1013.25
#endif

#ifdef BMP085_180_STM32
#include <BMP085_stm32.h>
#endif


#include "kalman.h"
#include "beepfunc.h"

#include "logger_i2c_eeprom.h"


//////////////////////////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////////////////////////
int mode = 0; //0 = read; 1 = write;

#ifdef BMP085_180
Adafruit_BMP085 bmp;
#endif
#ifdef BMP280
BMP280 bmp;
#endif
#ifdef BMP085_180_STM32
BMP085 bmp;
#endif

logger_I2C_eeprom logger(0x50) ;
long endAddress = 65536;

//ground level altitude
long initialAltitude;
long liftoffAltitude;
long lastAltitude;
//current altitude
long currAltitude;
//Apogee altitude
long apogeeAltitude;
long mainAltitude;
boolean liftOff = false;
unsigned long initialTime;

boolean FastReading = false;
boolean mainHasFired = false;
boolean landingHasFired = false;
boolean liftOffHasFired = false;
//Our drogue has been ejected i.e: apogee has been detected
boolean apogeeHasFired = false;

//nbr of measures to do so that we are sure that apogee has been reached
unsigned long measures = 5;
unsigned long mainDeployAltitude;

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

//soft configuration
boolean softConfigValid = false;

// main loop
boolean mainLoopEnable = true;

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


//#define NBR_MEASURE_APOGEE 5
float FEET_IN_METER = 1;
boolean canRecord = true;

// to store all event
boolean timerEvent1_enable = false;
boolean timerEvent2_enable = false;
boolean timerEvent3_enable = false;
#ifdef NBR_PYRO_OUT4
boolean timerEvent4_enable = false;
#endif
boolean apogeeEvent_Enable = false;
boolean mainEvent_Enable = false;
boolean landingEvent_Enable = false;
boolean liftOffEvent_Enable = false;
// enable/disable output
boolean out1Enable = true;
boolean out2Enable = true;
boolean out3Enable = true;
#ifdef NBR_PYRO_OUT4
boolean out4Enable = true;
#endif

int apogeeDelay = 0;
int mainDelay = 0;
int landingDelay = 0;
int liftOffDelay = 0;
int out1Delay = 0;
int out2Delay = 0;
int out3Delay = 0;
#ifdef NBR_PYRO_OUT4
int out4Delay = 0;
#endif
boolean Output1Fired = false;
boolean Output2Fired = false;
boolean Output3Fired = false;
#ifdef NBR_PYRO_OUT4
boolean Output4Fired = false;
#endif

// current file number that you are recording
int currentFileNbr = 0;

// EEPROM start address for the flights. Anything before that is the flight index
long currentMemaddress = 200;

boolean telemetryEnable = false;

//stop recording a maximum of 20 seconds after main has fired
long recordingTimeOut = 20000;
//long mainTime =0;
long lastTelemetry = 0;

boolean exitRecording = false;
boolean apogeeReadyToFire = false;
boolean mainReadyToFire = false;
boolean landingReadyToFire = false;
boolean liftOffReadyToFire = false;
unsigned long apogeeStartTime = 0;
unsigned long mainStartTime = 0;
unsigned long landingStartTime = 0;
unsigned long liftOffStartTime = 0;
boolean ignoreAltiMeasure = false;

boolean Event1Fired = false;
boolean Event2Fired = false;
boolean Event3Fired = false;
#ifdef NBR_PYRO_OUT4
boolean Event4Fired = false;
#endif
boolean ApogeeFiredComplete = false;
boolean MainFiredComplete = false;
boolean LandingFiredComplete = false;
boolean LiftOffFiredComplete = false;

void assignPyroOutputs();
void MainMenu();

#ifdef BMP085_180
/*
   ReadAltitude()
   Read Altitude function for a BMP85 or 180 Bosch sensor

*/
double ReadAltitude()
{
  return KalmanCalc(bmp.readAltitude());
}
#endif
#ifdef BMP085_180_STM32
double ReadAltitude()
{
  return KalmanCalc(bmp.readAltitude());
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

   initAlti()

*/
void initAlti() {
  exitRecording = false;
  apogeeReadyToFire = false;
  mainReadyToFire = false;
  landingReadyToFire = false;
  liftOffReadyToFire = false;
  apogeeStartTime = 0;
  mainStartTime = 0;
  landingStartTime = 0;
  liftOffStartTime = 0;
  ignoreAltiMeasure = false;

  Event1Fired = false;
  Event2Fired = false;
  Event3Fired = false;
#ifdef NBR_PYRO_OUT4
  Event4Fired = false;
#endif
  ApogeeFiredComplete = false;
  MainFiredComplete = false;
  LandingFiredComplete = false;
  LiftOffFiredComplete = false;
  landingHasFired = false;
  liftOffHasFired = false;
  apogeeHasFired = false;
  mainHasFired = false;

  liftOff = false;
  apogeeAltitude = 0;
  mainAltitude = 0;


  Output1Fired = false;
  Output2Fired = false;
  Output3Fired = false;
#ifdef NBR_PYRO_OUT4
  Output4Fired = false;
#endif
  //landingHasFired = false;
  //liftOffHasFired = false;
  lastAltitude = 0;//initialAltitude;
  out1Enable = true;
  out2Enable = true;
  out3Enable = true;
#ifdef NBR_PYRO_OUT4
  out4Enable = true;
#endif
  // set main altitude (if in feet convert to metrics)
  if (config.unit == 0)
    FEET_IN_METER = 1;
  else
    FEET_IN_METER = 3.28084 ;

  mainDeployAltitude = int(config.mainAltitude / FEET_IN_METER);
  // beepFrequency
  beepingFrequency = config.beepingFrequency;

  assignPyroOutputs();

  //number of measures to do to detect Apogee
  measures = config.nbrOfMeasuresForApogee;

  //check which pyro are enabled
  pos = -1;

  if (out1Enable) {
    pos++;
    continuityPins[pos] = pinChannel1Continuity;
  }
  if (out2Enable) {
    pos++;
    continuityPins[pos] = pinChannel2Continuity;
  }
  if (out3Enable) {
    pos++;
    continuityPins[pos] = pinChannel3Continuity;
  }
#ifdef NBR_PYRO_OUT4
  if (out4Enable)  {
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
    writeConfigStruc();
  }

  initAlti();

  // init Kalman filter
  KalmanInit();

  // initialise the connection
  Wire.begin();

  //You can change the baud rate here
  //and change it to 57600, 115200 etc..
  //Serial.begin(BAUD_RATE);
  SerialCom.begin(config.connectionSpeed);
  //SerialCom.begin(38400);

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
#endif

  //Presure Sensor Initialisation
#ifdef BMP085_180
  // Note that BMP180 is compatible with the BMP085 library
  // Low res should work better at high speed
  bmp.begin( config.altimeterResolution);
#endif
#ifdef BMP085_180_STM32
  // Note that BMP180 is compatible with the BMP085 library however some modifications have been done for the stm32
  // Low res should work better at high speed
  bmp.begin( config.altimeterResolution);
#endif
#ifdef BMP280
  bmp.begin();
  bmp.setOversampling(config.altimeterResolution)
#endif
  //our drogue has not been fired
  apogeeHasFired = false;
  mainHasFired = false;

  SerialCom.print(F("Start program\n"));

  //Initialise the output pin
  pinMode(pyroOut1, OUTPUT);
  pinMode(pyroOut2, OUTPUT);
  pinMode(pyroOut3, OUTPUT);
  //some Altimeter such as the STM32 have 4 pyro outputs
#ifdef NBR_PYRO_OUT4
  pinMode(pyroOut4, OUTPUT);
#endif
  pinMode(pinSpeaker, OUTPUT);

  pinMode(pinAltitude1, INPUT);
  pinMode(pinAltitude2, INPUT);

  pinMode(pinChannel1Continuity , INPUT);
  pinMode(pinChannel2Continuity , INPUT);
  pinMode(pinChannel3Continuity , INPUT);
#ifdef NBR_PYRO_OUT4
  pinMode(pinChannel4Continuity , INPUT);
#endif

  //Make sure that the output are turned off
  digitalWrite(pyroOut1, LOW);
  digitalWrite(pyroOut2, LOW);
  digitalWrite(pyroOut3, LOW);
#ifdef NBR_PYRO_OUT4
  digitalWrite(pyroOut4, LOW);
#endif
  digitalWrite(pinSpeaker, LOW);

  //enable or disable continuity check
  if (config.noContinuity == 1)
    noContinuity = true;
  else
    noContinuity = false;

  //initialisation give the version of the altimeter
  //One long beep per major number and One short beep per minor revision
  //For example version 1.2 would be one long beep and 2 short beep
  beepAltiVersion(MAJOR_VERSION, MINOR_VERSION);

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

  //number of measures to do to detect Apogee
  //measures = config.nbrOfMeasuresForApogee;

  // let's do some dummy altitude reading
  // to initialise the Kalman filter
  for (int i = 0; i < 50; i++) {
    ReadAltitude();
  }

  //let's read the launch site altitude
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += ReadAltitude();
    delay(50);
  }
  initialAltitude = (sum / 10.0);
  lastAltitude = 0;//initialAltitude;
  liftoffAltitude = config.liftOffAltitude;//20;

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

  // check if eeprom is full
  canRecord = logger.CanRecord();
  if (!canRecord)
    SerialCom.println("Cannot record");
}

/*
   assignPyroOutputs()
   Assign the pyro outputs different fonctionalities such as
   apogge, main, timer, landing or liftoff events

*/
void assignPyroOutputs()
{
  timerEvent1_enable = false;
  timerEvent2_enable = false;
  timerEvent3_enable = false;
  out1Delay = 0;
  out2Delay = 0;
  out3Delay = 0;

  pinOut1 = -1;
  pinOut2 = -1;
  pinOut3 = -1;
#ifdef NBR_PYRO_OUT4
  pinOut4 = -1;
  timerEvent4_enable = false;
  out4Delay = 0;
#endif
  landingDelay = 0;
  liftOffDelay = 0;
  apogeeDelay = 0;
  mainDelay = 0;
  mainEvent_Enable = false;
  apogeeEvent_Enable = false;
  landingEvent_Enable = false;
  liftOffEvent_Enable = false;
  for (int a = 0 ; a < 4 ; a++ ) {
    pinLanding[a] = -1;
    pinMain[a] = -1;
    pinApogee[a] = -1;
    pinLiftOff[a] = -1;
  }

  switch (config.outPut1)
  {
    case 0:
      mainEvent_Enable = true;
      mainDelay = config.outPut1Delay;
      pinMain[0] = pyroOut1;
      break;
    case 1:
      apogeeEvent_Enable = true;
      apogeeDelay = config.outPut1Delay;
      pinApogee[0] = pyroOut1;
      break;
    case 2:
      timerEvent1_enable = true;
      out1Delay = config.outPut1Delay;
      pinOut1 = pyroOut1;
      break;
    case 4:
      landingEvent_Enable = true;
      landingDelay = config.outPut1Delay;
      pinLanding[0] = pyroOut1;
      break;
    case 5:
      liftOffEvent_Enable = true;
      liftOffDelay = config.outPut1Delay;
      pinLiftOff[0] = pyroOut1;
      break;
    default:
      out1Enable = false;
      break;
  }

  switch (config.outPut2)
  {
    case 0:
      mainEvent_Enable = true;
      pinMain[1] = pyroOut2;
      mainDelay = config.outPut2Delay;
      break;
    case 1:
      apogeeEvent_Enable = true;
      pinApogee[1] = pyroOut2;
      apogeeDelay = config.outPut2Delay;
      break;
    case 2:
      timerEvent2_enable = true;
      out2Delay = config.outPut2Delay;
      pinOut2 = pyroOut2;
      break;
    case 4:
      landingEvent_Enable = true;
      landingDelay = config.outPut2Delay;
      pinLanding[1] = pyroOut2;
      break;
    case 5:
      liftOffEvent_Enable = true;
      liftOffDelay = config.outPut2Delay;
      pinLiftOff[1] = pyroOut2;
      break;
    default:
      out2Enable = false;
      break;
  }
  switch (config.outPut3)
  {
    case 0:
      mainEvent_Enable = true;
      mainDelay = config.outPut3Delay;
      pinMain[2] = pyroOut3;
      break;
    case 1:
      apogeeEvent_Enable = true;
      apogeeDelay = config.outPut3Delay;
      pinApogee[2] = pyroOut3;
      break;
    case 2:
      timerEvent3_enable = true;
      out3Delay = config.outPut3Delay;
      pinOut3 = pyroOut3;
      break;
    case 4:
      landingEvent_Enable = true;
      landingDelay = config.outPut3Delay;
      pinLanding[2] = pyroOut3;
      break;
    case 5:
      liftOffEvent_Enable = true;
      liftOffDelay = config.outPut3Delay;
      pinLiftOff[2] = pyroOut3;
      break;
    default:
      out3Enable = false;
      break;
  }
#ifdef NBR_PYRO_OUT4
  //output 4
  switch (config.outPut4)
  {
    case 0:
      mainEvent_Enable = true;
      mainDelay = config.outPut4Delay;
      pinMain[3] = pyroOut4;
      break;
    case 1:
      apogeeEvent_Enable = true;
      apogeeDelay = config.outPut4Delay;
      pinApogee[3] = pyroOut4;
      break;
    case 2:
      timerEvent4_enable = true;
      out4Delay = config.outPut4Delay;
      pinOut4 = pyroOut4;
      break;
    case 4:
      landingEvent_Enable = true;
      landingDelay = config.outPut4Delay;
      pinLanding[3] = pyroOut4;
      break;
    case 5:
      liftOffEvent_Enable = true;
      liftOffDelay = config.outPut4Delay;
      pinLiftOff[3] = pyroOut4;
      break;
    default:
      out4Enable = false;
      break;
  }
#endif
}

/*
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

  if (pyroOut == pyroOut3)
  {
    Output3Fired = state;
#ifdef SERIAL_DEBUG
    SerialCom.println(F("Output3Fired"));
#endif
  }
#ifdef NBR_PYRO_OUT4
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
    if (apogeeHasFired)
      ap = 1;

    //check main
    int ma = 0;
    if (mainHasFired)
      ma = 1;
    int landed = 0;
    if ( mainHasFired && currAltitude < 10)
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
    if (out1Enable) {
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
    if (out2Enable) {
      //check continuity
      val = digitalRead(pinChannel2Continuity);
      delay(20);
      if (val == 0)
        strcat(altiTelem, "0,");
      else
        strcat(altiTelem, "1,");
    }
    else {
      strcat(altiTelem, "-1,");
    }
    if (out3Enable) {
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
#ifdef NBR_PYRO_OUT4
    if (out4Enable) {
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
    sprintf(temp, "%f,", bat);
    strcat(altiTelem, temp);
#else
    strcat(altiTelem, "-1,");
#endif
    // temperature
    float temperature;
    temperature = bmp.readTemperature();
    sprintf(temp, "%i,", (int)temperature );
    strcat(altiTelem, temp);

    sprintf(temp, "%i,", (int)(100 * ((float)logger.getLastFlightEndAddress() / endAddress)) );
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", logger.getLastFlightNbr() + 1 );
    strcat(altiTelem, temp);

    unsigned int chk;
    chk = msgChk(altiTelem, sizeof(altiTelem));
    sprintf(temp, "%i", chk);
    strcat(altiTelem, temp);
    strcat(altiTelem, ";\n");
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

int currentVelocity(int prevTime, int curTime, int prevAltitude, int curAltitude)
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
  exitRecording = false;
  apogeeReadyToFire = false;
  mainReadyToFire = false;
  landingReadyToFire = false;
  liftOffReadyToFire = false;
  apogeeStartTime = 0;
  mainStartTime = 0;
  landingStartTime = 0;
  liftOffStartTime = 0;
  ignoreAltiMeasure = false;

  Event1Fired = false;
  Event2Fired = false;
  Event3Fired = false;
#ifdef NBR_PYRO_OUT4
  Event4Fired = false;
#endif
  ApogeeFiredComplete = false;
  MainFiredComplete = false;
  LandingFiredComplete = false;
  LiftOffFiredComplete = false;
  landingHasFired = false;
  liftOffHasFired = false;
  apogeeHasFired = false;
  mainHasFired = false;

  if (!out1Enable) Output1Fired = true;
  if (!out2Enable) Output2Fired = true;
  if (!out3Enable) Output3Fired = true;
#ifdef NBR_PYRO_OUT4
  if (!out4Enable) Output4Fired = true;
#endif

#ifdef SERIAL_DEBUG
  SerialCom.println(F("Config delay:"));
  SerialCom.println(config.outPut1Delay);
  SerialCom.println(config.outPut2Delay);
  SerialCom.println(config.outPut3Delay);
#ifdef NBR_PYRO_OUT4
  SerialCom.println(config.outPut4Delay);
#endif

  SerialCom.println(apogeeDelay);
  SerialCom.println(mainDelay);
#endif

  while (!exitRecording)
  {

    //read current altitude
    currAltitude = (ReadAltitude() - initialAltitude);
    //if (liftOff)
    //  SendTelemetry(millis() - initialTime, 200);
    if (( currAltitude > liftoffAltitude) && !liftOff && !mainHasFired)
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
      if (mainHasFired && !landingHasFired && !landingReadyToFire) {

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


      if (liftOffReadyToFire)
      {
        if ((millis() - liftOffStartTime) >= liftOffDelay)
        {
          //fire liftOff
          for (int lo = 0; lo < 4; lo++ ) {
            fireOutput(pinLiftOff[lo], true);
          }

          liftOffReadyToFire = false;
          liftOffHasFired = true;
          SendTelemetry(millis() - initialTime, 200);
        }
      }

      if (liftOffHasFired)
      {
        if ((millis() - (liftOffStartTime + liftOffDelay)) >= 1000 && !LiftOffFiredComplete)
        {
          for (int lo = 0; lo < 4; lo++ ) {
            fireOutput(pinLiftOff[lo], false);
            setEventState(pinLiftOff[lo], true);
          }
          LiftOffFiredComplete = true;
        }
      }

      if (timerEvent1_enable && !Event1Fired)
      {
        if (currentTime >= config.outPut1Delay)
        {
          //fire output pyroOut1
          digitalWrite(pyroOut1, HIGH);
          Event1Fired = true;
#ifdef SERIAL_DEBUG
          SerialCom.println(F("Fired 1st out"));
#endif
        }
      }
      if (timerEvent1_enable && Event1Fired)
      {
        if ((currentTime - config.outPut1Delay) >= 1000 && !Output1Fired)
        {
          //switch off output pyroOut1
          digitalWrite(pyroOut1, LOW);
          Output1Fired = true;
#ifdef SERIAL_DEBUG
          SerialCom.println(F("Finished Firing 1st out"));
#endif
        }
      }
      if (timerEvent2_enable && !Event2Fired )
      {
        if (currentTime >= config.outPut2Delay)
        {
          //fire output pyroOut2
          digitalWrite(pyroOut2, HIGH);
          Event2Fired = true;
#ifdef SERIAL_DEBUG
          SerialCom.println(F("Fired 2nd out"));
#endif
        }
      }
      if (timerEvent2_enable && Event2Fired  )
      {
        if ((currentTime - config.outPut2Delay) >= 1000 && !Output2Fired )
        {
          //switch off output pyroOut2
          digitalWrite(pyroOut2, LOW);
          Output2Fired = true;
#ifdef SERIAL_DEBUG
          SerialCom.println(F("Finished Firing 2nd out"));
#endif
        }
      }
      if (timerEvent3_enable && !Event3Fired )
      {
        if (currentTime >= config.outPut3Delay)
        {
          //fire output pyroOut3
          digitalWrite(pyroOut3, HIGH);
          Event3Fired = true;
#ifdef SERIAL_DEBUG
          SerialCom.println(F("Fired 3rd out"));
#endif
        }
      }
      if (timerEvent3_enable && Event3Fired )
      {
        if ((currentTime - config.outPut3Delay) >= 1000 && !Output3Fired)
        {
          //switch off output pyroOut3
          digitalWrite(pyroOut3, LOW);
          Output3Fired = true;
#ifdef SERIAL_DEBUG
          SerialCom.println(F("Finished Firing 3rd out"));
#endif
        }
      }
#ifdef NBR_PYRO_OUT4
      if (timerEvent4_enable && !Event4Fired)
      {
        if (currentTime >= config.outPut4Delay)
        {
          //fire output pyroOut4
          digitalWrite(pyroOut4, HIGH);
          Event4Fired = true;
#ifdef SERIAL_DEBUG
          SerialCom.println(F("Fired 4th out"));
#endif
        }
      }
      if (timerEvent4_enable && Event4Fired )
      {
        if ((currentTime - config.outPut4Delay) >= 1000 && !Output4Fired )
        {
          //switch off output pyroOut4
          digitalWrite(pyroOut4, LOW);
          Output4Fired = true;
#ifdef SERIAL_DEBUG
          SerialCom.println(F("Finished Firing 4th out"));
#endif
        }
      }
#endif

      if (canRecord)
      {
        logger.setFlightTimeData( diffTime);
        logger.setFlightAltitudeData(currAltitude);
        logger.setFlightTemperatureData((long) bmp.readTemperature());

        if ( (currentMemaddress + logger.getSizeOfFlightData())  > endAddress) {
          //flight is full let save it
          //save end address
          logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
          canRecord = false;
        } else {
          currentMemaddress = logger.writeFastFlight(currentMemaddress);
          currentMemaddress++;
        }
        delay(50);

      }
      if (config.superSonicYesNo == 1)
      {
        //are we still in superSonic mode?
        if (currentTime > 3000)
          ignoreAltiMeasure = false;
      }
      if ((currAltitude < lastAltitude) && !apogeeHasFired  && !ignoreAltiMeasure )
      {
        measures = measures - 1;
        if (measures == 0)
        {
          //fire drogue
          apogeeReadyToFire = true;
          apogeeStartTime = millis();
          apogeeAltitude = currAltitude;
        }
      }
      else
      {
        lastAltitude = currAltitude;
        measures = config.nbrOfMeasuresForApogee;
      }
      if (apogeeReadyToFire)
      {
        if ((millis() - apogeeStartTime) >= apogeeDelay)
        {
          //fire drogue
          for (int ap = 0; ap < 4; ap++ ) {
            digitalWrite(pinApogee[ap], HIGH);
            //setEventState(pinApogee[ap], true);
          }

#ifdef SERIAL_DEBUG
          SerialCom.println(F("Apogee has fired"));
#endif
          apogeeReadyToFire = false;
          apogeeHasFired = true;
          SendTelemetry(millis() - initialTime, 200);
        }
      }

      if (apogeeHasFired)
      {
        if ((millis() - (apogeeStartTime + apogeeDelay)) >= 1000 && !ApogeeFiredComplete )
        {
          for (int ap = 0; ap < 4; ap++ ) {
            digitalWrite(pinApogee[ap], LOW);
            setEventState(pinApogee[ap], true);
          }

          ApogeeFiredComplete = true;
        }
      }
      if ((currAltitude  < mainDeployAltitude) && apogeeHasFired && !mainHasFired)
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
      if (mainReadyToFire)
      {
        if ((millis() - mainStartTime) >= mainDelay)
        {
          //fire main
#ifdef SERIAL_DEBUG
          SerialCom.println(F("firing main"));
#endif
          for (int ma = 0; ma < 4; ma++ ) {
            digitalWrite(pinMain[ma], HIGH);
          }
          mainReadyToFire = false;
          mainHasFired = true;
          SendTelemetry(millis() - initialTime, 200);
        }
      }

      if (mainHasFired)
      {
        if ((millis() - (mainStartTime + mainDelay)) >= 1000 && !MainFiredComplete)
        {
          for (int ma = 0; ma < 4; ma++ ) {
            digitalWrite(pinMain[ma], LOW);
            setEventState(pinMain[ma], true);
          }
#ifdef SERIAL_DEBUG
          SerialCom.println("Main fired");
#endif
          MainFiredComplete = true;
        }
      }

      if (landingReadyToFire)
      {
        if ((millis() - landingStartTime) >= landingDelay)
        {
          //fire landing
          for (int la = 0; la < 4; la++ ) {
            fireOutput(pinLanding[la], true);
          }

          landingReadyToFire = false;
          landingHasFired = true;
          SendTelemetry(millis() - initialTime, 200);
        }
      }

      if (landingHasFired)
      {
        if ((millis() - (landingStartTime + landingDelay)) >= 1000 && !LandingFiredComplete)
        {
          for (int la = 0; la < 4; la++ ) {
            fireOutput(pinLanding[la], false);
            setEventState(pinLanding[la], true);
          }
          LandingFiredComplete = true;
        }
      }
      //if ((canRecord && MainFiredComplete && (currAltitude < 10)) || (canRecord && MainFiredComplete && (millis() - mainStartTime) > recordingTimeOut))
      if ((canRecord && MainFiredComplete && (currAltitude < 10) && LandingFiredComplete) || (canRecord && MainFiredComplete && (millis() - mainStartTime) > recordingTimeOut))
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
      //if ((MainFiredComplete && (currAltitude < 10)) || (MainFiredComplete && (millis() - mainStartTime) > recordingTimeOut))
      if ((MainFiredComplete && (currAltitude < 10) && LandingFiredComplete) || (MainFiredComplete && (millis() - mainStartTime) > recordingTimeOut))
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
      if (LandingFiredComplete)
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

  char commandbuffer[200];

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
      while (apogeeHasFired  && mainHasFired )
      {
        // check if we have anything on the serial port
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
   e  erase all saved flights
   r  followed by a number which is the flight number.
      This will retrieve all data for the specified flight
   w  Start or stop recording
   n  Return the number of recorded flights in the EEprom
   l  list all flights
   c  toggle continuity on and off
   a  get all flight data
   b  get altimeter config
   s  write altimeter config
   d  reset alti config
   t  reset alti config (why?)
   f  FastReading on
   g  FastReading off
   h  hello. Does not do much
   i  unused
   k  folowed by a number turn on or off the selected output
   y  followed by a number turn telemetry on/off. if number is 1 then
      telemetry in on else turn it off
   m  followed by a number turn main loop on/off. if number is 1 then
      main loop in on else turn it off
*/
void interpretCommandBuffer(char *commandbuffer) {
  SerialCom.println((char*)commandbuffer);
  //this will erase all flight
  if (commandbuffer[0] == 'e')
  {
    SerialCom.println(F("Erase\n"));
    //i2c_eeprom_erase_fileList();
    logger.clearFlightList();
    logger.writeFlightList();
    currentFileNbr = 0;
    currentMemaddress = 201;
  }
  //this will read one flight
  else if (commandbuffer[0] == 'r')
  {
    char temp[3];
    SerialCom.println(F("Read flight: "));
    SerialCom.println( commandbuffer[1]);
    SerialCom.println( "\n");
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
      //printFlight(atol(temp));
      SerialCom.println("StartFlight;" );
      //logger.PrintFlight(atoi(temp));
      logger.printFlightData(atoi(temp));
      SerialCom.println("EndFlight;" );
    }
    else
      SerialCom.println(F("not a valid flight"));
  }
  // Recording
  else if (commandbuffer[0] == 'w')
  {
    SerialCom.println(F("Recording \n"));
    recordAltitude();
  }
  //Number of flight
  else if (commandbuffer[0] == 'n')
  {
    SerialCom.println(F("Number of flight \n"));
    SerialCom.print(F("n;"));
    //Serial.println(getFlightList());
    logger.printFlightList();
  }
  //list all flights
  else if (commandbuffer[0] == 'l')
  {
    SerialCom.println(F("Flight List: \n"));
    logger.printFlightList();
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
  //get all flight data
  else if (commandbuffer[0] == 'a')
  {
    SerialCom.print(F("$start;\n"));
    //getFlightList()
    int i;
    ///todo
    for (i = 0; i < logger.getLastFlightNbr() + 1; i++)
    {
      logger.printFlightData(i);
    }

    SerialCom.print(F("$end;\n"));
  }
  //get altimeter config
  else if (commandbuffer[0] == 'b')
  {
    SerialCom.print(F("$start;\n"));

    printAltiConfig();

    SerialCom.print(F("$end;\n"));
  }
  //write altimeter config
  else if (commandbuffer[0] == 's')
  {
    if (writeAltiConfig(commandbuffer)) {

      SerialCom.print(F("$OK;\n"));
      readAltiConfig();
      initAlti();
    }
    else {
      SerialCom.print(F("$KO;\n"));
      //readAltiConfig();
      //initAlti();
    }
  }
  //reset alti config this is equal to t why do I have 2 !!!!
  else if (commandbuffer[0] == 'd')
  {
    defaultConfig();
    writeConfigStruc();
    initAlti();
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
  //FastReading
  else if (commandbuffer[0] == 'f')
  {
    FastReading = true;
    SerialCom.print(F("$OK;\n"));

  }
  //FastReading off
  else if (commandbuffer[0] == 'g')
  {
    FastReading = false;
    SerialCom.print(F("$OK;\n"));
  }
  //hello
  else if (commandbuffer[0] == 'h')
  {
    //FastReading = false;
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
        case 3:
          fireOutput(pyroOut3, fire);
          break;
#ifdef NBR_PYRO_OUT4
        case 4:
          fireOutput(pyroOut4, fire);
          break;
#endif
      }
    }
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
    SerialCom.print(F("$OK;\n"));
  }
  //mainloop on/off
  else if (commandbuffer[0] == 'm')
  {
    if (commandbuffer[1] == '1') {
#ifdef SERIAL_DEBUG
      SerialCom.print(F("main Loop enabled\n"));
#endif
      mainLoopEnable = true;
    }
    else {
#ifdef SERIAL_DEBUG
      SerialCom.print(F("main loop disabled\n"));
#endif
      mainLoopEnable = false;
    }
    SerialCom.print(F("$OK;\n"));
  }
  // empty command
  else if (commandbuffer[0] == ' ')
  {
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

  apogeeHasFired = false;
  mainHasFired = false;
  liftOff = false;
  Output1Fired = false;
  Output2Fired = false;
  Output3Fired = false;
#ifdef NBR_PYRO_OUT4
  Output4Fired = false;
#endif

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
  pinMode(PB1, INPUT_ANALOG);
  int batVoltage = analogRead(PB1);
  //float bat = 3.05 * ((float)(batVoltage * 3300) / (float)4096000);
  float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
  //float bat =10*((float)(batVoltage*3300)/(float)4096000);
  //SerialCom.println(bat);
  if (bat < minVolt) {
    for (int i = 0; i < 10; i++)
    {
      tone(pinSpeaker, 1600, 1000);
      delay(50);
      noTone(pinSpeaker);
    }
    delay(1000);
  }
#endif

}
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
