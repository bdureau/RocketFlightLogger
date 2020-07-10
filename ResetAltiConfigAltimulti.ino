

#include "Arduino.h"
//used for writing in the microcontroler internal eeprom
#include <EEPROM.h>

#define SerialCom  Serial

struct ConfigStruct {
  int unit;             //0 = meter 1 = feet
  int beepingMode;      // decide which way you want to report the altitude
  int outPut1;          // assign a function to each pyro
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
  int recordTemperature;  //decide if we want to record temperature
  int superSonicDelay;   //nbr of ms during when we ignore any altitude measurements
  long connectionSpeed;   //altimeter connection baudrate
  int altimeterResolution; // BMP sensor resolution
  int eepromSize;
  int noContinuity;
  int outPut4;
  int outPut4Delay;
  int liftOffAltitude; //Lift off Altitude in meters
  int batteryType; // 0= Unknown, 1= "2S (7.4 Volts)", 2 = "9 Volts",3 = "3S (11.1 Volts)
  int cksum;
};
#define CONFIG_START 32
ConfigStruct config;
void writeConfigStruc()
{
  int i;
  for ( i = 0; i < sizeof(config); i++ ) {
    EEPROM.write(CONFIG_START + i, *((char*)&config + i));
  }
}

//================================================================
// read and write in the microcontroler eeprom
//================================================================
void defaultConfig()
{
  config.unit = 0;
  config.beepingMode = 0;
  config.outPut1 = 0;
  config.outPut2 = 1;
  config.outPut3 = 3;
  config.outPut1Delay = 0;
  config.outPut2Delay = 0;
  config.outPut3Delay = 0;
  config.mainAltitude = 50;
  config.superSonicYesNo = 0;
  config.beepingFrequency = 440;
  //config.separationVelocity = 10;
  config.nbrOfMeasuresForApogee = 5;
  config.endRecordAltitude = 3; // stop recording when landing define under which altitude we are not recording
  config.recordTemperature = 0; //decide if we want to record temperature
  config.superSonicDelay = 0;
  config.connectionSpeed = 38400;
  config.altimeterResolution = 0; //0 to 4 ie: from low resolution to high
  config.eepromSize = 512;
  config.noContinuity = 1;
  config.outPut4=3;
  config.outPut4Delay=0;
  config.liftOffAltitude=10;
  config.batteryType=0;
  config.cksum = CheckSumConf(config);
}

void setup() {
  SerialCom.begin(38400);

  defaultConfig();
  // put your setup code here, to run once:
  writeConfigStruc();
  printAltiConfig();
  SerialCom.println("chk:");
  SerialCom.print(CheckSumConf(config));
  SerialCom.println("chk2:");
  SerialCom.print(config.cksum);
//pin 12 and 13 are for the speaker on the altimulti and altimultiV2
//Just advice the user that the configuration is complete
  tone(12, 440, 1000);
  tone(13, 440, 1000);
}

unsigned int CheckSumConf( ConfigStruct cnf)
{
  int i;
  unsigned int chk = 0;

  for (i = 0; i < (sizeof(cnf) - sizeof(int)); i++)
    chk += *((char*)&cnf + i);

  return chk;
}

boolean readAltiConfig() {
  //set the config to default values so that if any have not been configured we can use the default ones
  defaultConfig();
  int i;
  for ( i = 0; i < sizeof(config); i++ ) {
    *((char*)&config + i) = EEPROM.read(CONFIG_START + i);
  }

  if ( config.cksum != CheckSumConf(config) ) {
    return false;
  }

  return true;

}

void printAltiConfig()
{

  bool ret = readAltiConfig();
  if (!ret)
    SerialCom.print(F("invalid conf"));
  SerialCom.print(F("$alticonfig"));
  SerialCom.print(F(","));
  //Unit
  SerialCom.print(config.unit);
  SerialCom.print(F(","));
  //beepingMode
  SerialCom.print(config.beepingMode);
  SerialCom.print(F(","));
  //output1
  SerialCom.print(config.outPut1);
  SerialCom.print(F(","));
  //output2
  SerialCom.print(config.outPut2);
  SerialCom.print(F(","));
  //output3
  SerialCom.print(config.outPut3);
  SerialCom.print(F(","));
  //supersonicYesNo
  SerialCom.print(config.superSonicYesNo);
  SerialCom.print(F(","));
  //mainAltitude
  SerialCom.print(config.mainAltitude);
  SerialCom.print(F(","));
  //AltimeterName
  //  SerialCom.print(F(BOARD_FIRMWARE));
  SerialCom.print(F(","));
  //alti major version
  //SerialCom.print(MAJOR_VERSION);
  //alti minor version
  SerialCom.print(F(","));
  // SerialCom.print(MINOR_VERSION);
  SerialCom.print(F(","));
  //output1 delay
  SerialCom.print(config.outPut1Delay);
  SerialCom.print(F(","));
  //output2 delay
  SerialCom.print(config.outPut2Delay);
  SerialCom.print(F(","));
  //output3 delay
  SerialCom.print(config.outPut3Delay);
  SerialCom.print(F(","));
  //Beeping frequency
  SerialCom.print(config.beepingFrequency);
  SerialCom.print(F(","));
  SerialCom.print(config.nbrOfMeasuresForApogee);
  SerialCom.print(F(","));
  SerialCom.print(config.endRecordAltitude);
  SerialCom.print(F(","));
  SerialCom.print(config.recordTemperature);
  SerialCom.print(F(","));
  SerialCom.print(config.superSonicDelay);
  SerialCom.print(F(","));
  SerialCom.print(config.connectionSpeed);
  SerialCom.print(F(","));
  SerialCom.print(config.altimeterResolution);
  SerialCom.print(F(","));
  SerialCom.print(config.eepromSize);
  SerialCom.print(F(","));
  SerialCom.print(config.noContinuity);
  SerialCom.print(F(";\n"));


}
void loop() {
  // put your main code here, to run repeatedly:

}
