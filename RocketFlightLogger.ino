/*
  Rocket Flight Logger ver 1.24
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
  Major changes on version 1.23
  Fixes to the delays when multiple event of the same type
   Major changes on version 1.24
  Optimise the code so that it uses less global variables
  Added altitude event
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

#ifdef BMP085_180
Adafruit_BMP085 bmp;
#endif
#ifdef BMP280
BMP280 bmp;
#endif
#ifdef BMP085_180_STM32
BMP085 bmp;
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

boolean liftOff = false;
unsigned long initialTime;
boolean FastReading = false;

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


boolean Output1Fired = false;
boolean Output2Fired = false;
boolean Output3Fired = false;
#ifdef NBR_PYRO_OUT4
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

  ResetGlobalVar()

*/
void ResetGlobalVar() {

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
  Output3Fired = false;
#ifdef NBR_PYRO_OUT4
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
  if (config.outPut3 != 3) {
    pos++;
    continuityPins[pos] = pinChannel3Continuity;
  }
#ifdef NBR_PYRO_OUT4
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
   SendTelemetry(long sampleTime, int freq)
   Send telemety so that we can plot the flight

*/
void SendTelemetry(long sampleTime, int freq) {
  char altiTelem[150] = "";
  //char *altiTelem;
  //altiTelem = (char *) malloc(150);
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
      delay(20);
      if (val == 0)
        strcat(altiTelem, "0,");
      else
        strcat(altiTelem, "1,");
    }
    else {
      strcat(altiTelem, "-1,");
    }
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
#ifdef NBR_PYRO_OUT4
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
  //free (altiTelem);
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
  OutputType[2] = config.outPut3;
#ifdef ALTIMULTISTM32
  OutputType[3] = config.outPut4;
#endif
  int OutputPins[4] = { -1, -1, -1, -1};
  if (config.outPut1 != 3)
    OutputPins[0] = pyroOut1;
  if (config.outPut2 != 3)
    OutputPins[1] = pyroOut2;
  if (config.outPut3 != 3)
    OutputPins[2] = pyroOut3;
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
  if (config.outPut3 == 3) Output3Fired = true;
#ifdef NBR_PYRO_OUT4
  if (config.outPut4 == 3) Output4Fired = true;
#endif

#ifdef SERIAL_DEBUG
  SerialCom.println(F("Config delay:"));
  SerialCom.println(config.outPut1Delay);
  SerialCom.println(config.outPut2Delay);
  SerialCom.println(config.outPut3Delay);
#ifdef NBR_PYRO_OUT4
  SerialCom.println(config.outPut4Delay);
#endif

#endif

  while (!exitRecording)
  {
    //read current altitude
    currAltitude = (ReadAltitude() - initialAltitude);
    //if (liftOff)
    //  SendTelemetry(millis() - initialTime, 200);
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
      if (allMainFiredComplete && !allLandingFiredComplete && !landingReadyToFire) {

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

        if ( (currentMemaddress + logger.getSizeOfFlightData())  > endAddress) {
          //flight is full let's save it
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
      if ((currAltitude < lastAltitude) && !apogeeReadyToFire  && !ignoreAltiMeasure )
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
      //if ((MainFiredComplete && (currAltitude < 10)) || (MainFiredComplete && (millis() - mainStartTime) > recordingTimeOut))
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
  //char *commandbuffer;

  /* Initial memory allocation */
  //commandbuffer = (char *) malloc(150);

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
  //free (commandbuffer);
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
      //mainLoopEnable = true;
    }
    else {
#ifdef SERIAL_DEBUG
      SerialCom.print(F("main loop disabled\n"));
#endif
      //mainLoopEnable = false;
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


  allApogeeFiredComplete  = false;
  allMainFiredComplete = false;
  allTimerFiredComplete = false;
  allLiftOffFiredComplete = false;
  allLandingFiredComplete = false;
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
