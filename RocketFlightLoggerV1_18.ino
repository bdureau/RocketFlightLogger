/*
  Rocket Flight Logger ver 1.18
  Copyright Boris du Reau 2012-2019

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
//long BAUD_RATE=57600;
//long BAUD_RATE=38400;

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

//not that the STM32 board has 4 pyro output
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
const int pinChannel1Continuity = 10;
// by default continuity for the main
const int pinChannel2Continuity = 11;
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
// enable/disable output
boolean out1Enable = true;
boolean out2Enable = true;
boolean out3Enable = true;
#ifdef NBR_PYRO_OUT4
boolean out4Enable = true;
#endif

int apogeeDelay = 0;
int mainDelay = 0;
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

// EEPROM start adress for the flights. Anything before that is the flight index
long currentMemaddress = 200;

void assignPyroOutputs();
void MainMenu();

#ifdef BMP085_180
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
// Only used by BMP280
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
//================================================================
// Start program
//================================================================
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
    config.connectionSpeed = 57600;
    writeConfigStruc();
  }

  // set main altitude (if in feet convert to metrics)
  if (config.unit == 0)
    FEET_IN_METER = 1;
  else
    FEET_IN_METER = 3.28084 ;

  mainDeployAltitude = int(config.mainAltitude / FEET_IN_METER);
  // beepFrequency
  beepingFrequency = config.beepingFrequency;

  // init Kalman filter
  KalmanInit();

  // initialise the connection
  Wire.begin();

  //You can change the baud rate here
  //and change it to 57600, 115200 etc..
  //Serial.begin(BAUD_RATE);
  SerialCom.begin(config.connectionSpeed);

  //  pinMode(A0, INPUT);


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
  assignPyroOutputs();

  //SerialCom.print(F("Set outputs\n"));
  //Initialise the output pin
  pinMode(pyroOut1, OUTPUT);
  pinMode(pyroOut2, OUTPUT);
  pinMode(pyroOut3, OUTPUT);
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

  //number of measures to do to detect Apogee
  measures = 5; //NBR_MEASURE_APOGEE;

  if (!softConfigValid)
  {
    //initialise the deployement altitude for the main
    mainDeployAltitude = 100;

    // On the Alti duo when you close the jumper you set it to 1
    // val is the left jumper and val1 is the right jumper
    //as of version 1.4 only use the jumper if no valid softconfiguration

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
  liftoffAltitude = 20;

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
    /*Serial.print("Mem address:");
      Serial.println(currentMemaddress);
      Serial.print("Current flight:");
      Serial.println(currentFileNbr);*/
  }

  // check if eeprom is full
  /*if(isEepromFull())
    canRecord == false;*/
  canRecord = logger.CanRecord();
  if (!canRecord)
    SerialCom.println("Cannot record");

  //SerialCom.println("Init complete");

  //check which pyro are enabled

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
void assignPyroOutputs()
{
  pinMain = -1;
  pinApogee = -1;
  pinOut1 = -1;
  pinOut2 = -1;
  pinOut3 = -1;
#ifdef NBR_PYRO_OUT4
  pinOut4 = -1;
#endif


  switch (config.outPut1)
  {
    case 0:
      mainEvent_Enable = true;
      mainDelay = config.outPut1Delay;
      pinMain = pyroOut1;
      break;
    case 1:
      apogeeEvent_Enable = true;
      apogeeDelay = config.outPut1Delay;
      pinApogee = pyroOut1;
      break;
    case 2:
      timerEvent1_enable = true;
      out1Delay = config.outPut1Delay;
      pinOut1 = pyroOut1;
      break;
    default:
      out1Enable = false;
      break;
  }

  switch (config.outPut2)
  {
    case 0:
      mainEvent_Enable = true;
      pinMain = pyroOut2;
      mainDelay = config.outPut2Delay;
      break;
    case 1:
      apogeeEvent_Enable = true;
      pinApogee = pyroOut2;
      apogeeDelay = config.outPut2Delay;
      break;
    case 2:
      timerEvent2_enable = true;
      out2Delay = config.outPut2Delay;
      pinOut2 = pyroOut2;
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
      pinMain = pyroOut3;
      break;
    case 1:
      apogeeEvent_Enable = true;
      apogeeDelay = config.outPut3Delay;
      pinApogee = pyroOut3;
      break;
    case 2:
      timerEvent3_enable = true;
      out3Delay = config.outPut3Delay;
      pinOut3 = pyroOut3;
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
      pinMain = pyroOut4;
      break;
    case 1:
      apogeeEvent_Enable = true;
      apogeeDelay = config.outPut4Delay;
      pinApogee = pyroOut4;
      break;
    case 2:
      timerEvent4_enable = true;
      out4Delay = config.outPut4Delay;
      pinOut4 = pyroOut4;
      break;
    default:
      out4Enable = false;
      break;
  }
#endif
}

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

void SendTelemetry(long sampleTime) {
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
  SerialCom.print(F("$telemetry,"));
  SerialCom.print(currAltitude);
  SerialCom.print(F(","));
  SerialCom.print(li);
  SerialCom.print(F(","));
  SerialCom.print(ap);
  SerialCom.print(F(","));
  SerialCom.print(apogeeAltitude);
  SerialCom.print(F(","));
  SerialCom.print(ma);
  SerialCom.print(F(","));
  SerialCom.print(mainAltitude);
  SerialCom.print(F(","));
  SerialCom.print(landed);
  SerialCom.print(F(","));
  SerialCom.print(sampleTime);
  SerialCom.print(F(","));
  if(out1Enable) {
    //check continuity
     val = digitalRead(pinChannel1Continuity);
     if (val == 0)
      SerialCom.print(0);  
     else   
      SerialCom.print(1);  
  }
  else {
    SerialCom.print(-1);
  }
  SerialCom.print(F(","));
  if(out2Enable) {
     //check continuity
     val = digitalRead(pinChannel2Continuity);
     delay(20);
     if (val == 0)
      SerialCom.print(0);  
     else   
      SerialCom.print(1);
  }
  else {
    SerialCom.print(-1);
  }
  SerialCom.print(F(","));
if(out3Enable) {
     //check continuity
     val = digitalRead(pinChannel3Continuity);
     if (val == 0)
      SerialCom.print(0);  
     else   
      SerialCom.print(1);
  }
  else {
    SerialCom.print(-1);
  }
  #ifdef NBR_PYRO_OUT4
  SerialCom.print(F(","));
  if(out4Enable) {
     //check continuity
     val = digitalRead(pinChannel4Continuity);
     //delay(20);
     if (val == 0)
      SerialCom.print(0);  
     else   
      SerialCom.print(1);
  }
  else {
    SerialCom.print(-1);
  }
  #endif
  #ifdef ALTIMULTISTM32
  SerialCom.print(F(","));
  pinMode(PB1, INPUT_ANALOG);
  int batVoltage = analogRead(PB1); 
  //float bat =((batVoltage*3300)/4096)/100;
  SerialCom.print(batVoltage);
  #endif
  SerialCom.println(F(";"));
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
  int curSpeed = int ((curAltitude - prevAltitude) / ( curTime - prevTime));
  return curSpeed;
}

//================================================================
// Function:  recordAltitude()
// called for normal recording
//================================================================
void recordAltitude()
{
  boolean exit = false;
  boolean apogeeReadyToFire = false;
  boolean mainReadyToFire = false;
  unsigned long apogeeStartTime = 0;
  unsigned long mainStartTime = 0;
  //unsigned long liftoffStartTime=0;
  boolean ignoreAltiMeasure = false;
  // boolean finishedEvent = false;
  boolean Event1Fired = false;
  boolean Event2Fired = false;
  boolean Event3Fired = false;
#ifdef NBR_PYRO_OUT4
  boolean Event4Fired = false;
#endif
  boolean MainFiredComplete = false;

  if (out1Enable == false) Output1Fired = true;
  if (out2Enable == false) Output2Fired = true;
  if (out3Enable == false) Output3Fired = true;
#ifdef NBR_PYRO_OUT4
  if (out4Enable == false) Output4Fired = true;
#endif

#ifdef SERIAL_DEBUG
  if (pinMain == -1) SerialCom.println(F("Main disable\n"));
  if (pinApogee == -1) SerialCom.println(F("Apogee disable\n"));
  if (pinOut1 == -1) SerialCom.println(F("pinOut1 disable\n"));
  if (pinOut2 == -1) SerialCom.println(F("pinOut2 disable\n"));
  if (pinOut3 == -1) SerialCom.println(F("pinOut3 disable\n"));
#ifdef NBR_PYRO_OUT4
  if (pinOut4 == -1) SerialCom.println(F("pinOut4 disable\n"));
#endif
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

  while (exit == false)
  {

    //read current altitude
    currAltitude = (ReadAltitude() - initialAltitude);
    if (liftOff)
      SendTelemetry(millis() - initialTime);
    if (( currAltitude > liftoffAltitude) == true && liftOff == false && mainHasFired == false)
    {
      liftOff = true;
      SendTelemetry(0);
      // save the time
      initialTime = millis();
      if (config.superSonicYesNo == 1)
        ignoreAltiMeasure = true;
    }
    if (liftOff)
    {
#ifdef SERIAL_DEBUG
      SerialCom.println(F("we have lift off\n"));
#endif
      if (canRecord)
      {
        //Save start address
        logger.setFlightStartAddress (currentFileNbr, currentMemaddress);
      }
      unsigned long prevTime = 0;
      // loop until we have reach an altitude of 3 meter
      //while(currAltitude > 3 && MainFiredComplete==false && liftOff ==true;)
      while (liftOff == true)
      {
        unsigned long currentTime;
        unsigned long diffTime;
        unsigned long timerEvent1_startime;

        currAltitude = (ReadAltitude() - initialAltitude);

        currentTime = millis() - initialTime;
        SendTelemetry(currentTime);
        diffTime = currentTime - prevTime;
        prevTime = currentTime;
        if (timerEvent1_enable && Event1Fired == false)
        {
          if (currentTime >= config.outPut1Delay)
          {
            //fire output pyroOut1
            digitalWrite(pyroOut1, HIGH);
            timerEvent1_startime = currentTime;
            Event1Fired = true;
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Fired 1st out"));
#endif
          }
        }
        if (timerEvent1_enable && Event1Fired == true)
        {
          if ((currentTime - config.outPut1Delay) >= 1000 && Output1Fired == false)
          {
            //switch off output pyroOut1
            digitalWrite(pyroOut1, LOW);
            Output1Fired = true;
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Finished Firing 1st out"));
#endif
          }
        }
        if (timerEvent2_enable && Event2Fired == false)
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
        if (timerEvent2_enable && Event2Fired == true )
        {
          if ((currentTime - config.outPut2Delay) >= 1000 && Output2Fired == false)
          {
            //switch off output pyroOut2
            digitalWrite(pyroOut2, LOW);
            Output2Fired = true;
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Finished Firing 2nd out"));
#endif
          }
        }
        if (timerEvent3_enable && Event3Fired == false)
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
        if (timerEvent3_enable && Event3Fired == true)
        {
          if ((currentTime - config.outPut3Delay) >= 1000 && Output3Fired == false)
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
        if (timerEvent4_enable && Event4Fired == false)
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
        if (timerEvent4_enable && Event4Fired == true)
        {
          if ((currentTime - config.outPut4Delay) >= 1000 && Output4Fired == false)
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
          currentMemaddress = logger.writeFlight(currentMemaddress);
          currentMemaddress++;
        }
        if (config.superSonicYesNo == 1)
        {
          //are we still in superSonic mode?
          if (currentTime > 3000)
            ignoreAltiMeasure = false;
        }
        if (currAltitude < lastAltitude && apogeeHasFired == false && ignoreAltiMeasure == false)
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
          measures = 5;
        }
        if (apogeeReadyToFire)
        {
          if ((millis() - apogeeStartTime) >= apogeeDelay)
          {
            //fire drogue
            digitalWrite(pinApogee, HIGH);
            setEventState(pinApogee, true);
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Apogee has fired"));
#endif
            apogeeReadyToFire = false;
            apogeeHasFired = true;
            SendTelemetry(millis() - initialTime);
          }
        }

        if ((currAltitude  < mainDeployAltitude) && apogeeHasFired == true && mainHasFired == false)
        {
          // Deploy main chute  X meters or feet  before landing...
          digitalWrite(pinApogee, LOW);
#ifdef SERIAL_DEBUG
          SerialCom.println(F("Apogee firing complete"));
#endif
          mainReadyToFire = true;
#ifdef SERIAL_DEBUG
          SerialCom.println(F("preparing main"));
#endif
          mainStartTime = millis();
          //digitalWrite(pinMain, HIGH);
          //mainHasFired=true;
          mainAltitude = currAltitude;
#ifdef SERIAL_DEBUG
          SerialCom.println(F("main altitude"));

          SerialCom.println(mainAltitude);
#endif
        }
        if (mainReadyToFire)
        {
          //Serial.println("conf delay main" + config.outPut1Delay );
          //Serial.println("conf delay" + config.outPut2Delay );
          SerialCom.println(mainStartTime);

          if ((millis() - mainStartTime) >= mainDelay)
          {
            //fire main
#ifdef SERIAL_DEBUG
            SerialCom.println(F("firing main"));
#endif
            digitalWrite(pinMain, HIGH);
            mainReadyToFire = false;
            //setEventState(pinMain, true);
            mainHasFired = true;
            SendTelemetry(millis() - initialTime);
          }
        }

        if (mainHasFired)
        {

          if ((millis() - (mainStartTime + mainDelay)) >= 1000 && MainFiredComplete == false)
          {
            digitalWrite(pinMain, LOW);
            setEventState(pinMain, true);
            //liftOff =false;
#ifdef SERIAL_DEBUG
            SerialCom.println("Main fired");
#endif
            MainFiredComplete = true;
          }
        }
        if (canRecord && MainFiredComplete && currAltitude < 10)
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
          //fileA[currentFileNbr].endaddress = currentMemaddress -1;
          //saveFlightA(currentFileNbr, startMemaddress, currentMemaddress -1);
          logger.writeFlightList();
        }
        if (MainFiredComplete && currAltitude < 10)
        {
          liftOff = false;
          SendTelemetry(millis() - initialTime);
        }
#ifdef NBR_PYRO_OUT4
        if (Output1Fired == true && Output2Fired == true && Output3Fired == true && Output4Fired == true)
#else
        if (Output1Fired == true && Output2Fired == true && Output3Fired == true )
#endif
        {
#ifdef SERIAL_DEBUG
          SerialCom.println(F("all event have fired"));
#endif
          exit = true;
          SendTelemetry(millis() - initialTime);
        }

      }
    }
  }
}




//================================================================
// Main menu to interpret all the commands sent by the altimeter console
//================================================================
void MainMenu()
{
  char readVal = ' ';
  int i = 0;

  char commandbuffer[200];

  SerialCom.println(F("Rocket flight data logger. A maximum of 25 flight can be logged \n"));
  SerialCom.println(F("Commands are: \n"));
  SerialCom.println(F("w = record flight \n"));
  SerialCom.println(F("r (followed by the flight number) = read flight data\n"));
  SerialCom.println(F("l  = print flight list \n"));
  SerialCom.println(F("e  = erase all flight data \n"));
  SerialCom.println(F("c  = toggle continuity on/off \n"));
  SerialCom.println(F("b  = print alti config \n"));
  SerialCom.println(F("Enter Command and terminate it by a ; >>\n"));
  i = 0;
  readVal = ' ';
  while ( readVal != ';')
  {
    if (FastReading == false)
    {
      currAltitude = (ReadAltitude() - initialAltitude);
      if (liftOff)
        SendTelemetry(millis() - initialTime);
      if (( currAltitude > liftoffAltitude) != true)
      {
        continuityCheckNew();
        SendTelemetry(0);
        checkBatVoltage(7.0);
      }
      else
      {
        recordAltitude();
      }
      long savedTime = millis();
      while (apogeeHasFired == true && mainHasFired == true)
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


void interpretCommandBuffer(char *commandbuffer) {
  SerialCom.println((char*)commandbuffer);
  //this will erase all flight
  if (commandbuffer[0] == 'e')
  {
    SerialCom.println(F("Erase\n"));
    //i2c_eeprom_erase_fileList();
    logger.clearFlightList();
    logger.writeFlightList();
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
      logger.PrintFlight(atoi(temp));
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
    //recordAltitude();
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
    writeAltiConfig(commandbuffer);
  }
  //reset alti config
  else if (commandbuffer[0] == 'd')
  {
    defaultConfig();
    writeConfigStruc();
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
  else if (commandbuffer[0] == 't')
  {
    //reset config
    defaultConfig();
    writeConfigStruc();
    SerialCom.print(F("config reseted\n"));
  }
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
    if(commandbuffer[2] == 'F')
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
  else if (commandbuffer[0] == ' ')
  {
    SerialCom.print(F("$K0;\n"));
  }

  else
  {
    // Serial.println(F("Unknown command" ));
    SerialCom.print(F("$UNKNOWN;"));
    SerialCom.println(commandbuffer[0]);

  }
}

void resetFlight() {
  // re-nitialise all flight related global variables
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

void checkBatVoltage(float minVolt) {
  #ifdef ALTIMULTISTM32
  pinMode(PB1, INPUT_ANALOG);
  int batVoltage = analogRead(PB1); 
  float bat =3.05*((float)(batVoltage*3300)/(float)4096000);
  //float bat =10*((float)(batVoltage*3300)/(float)4096000);
  SerialCom.println(bat);
  if(bat < minVolt) {
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
void fireOutput(int pin, boolean fire) {
  if(fire)
    digitalWrite(pin, LOW);
  else
    digitalWrite(pin, HIGH);  
}
