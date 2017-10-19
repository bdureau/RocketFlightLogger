#ifndef _CONFIG_H
#define _CONFIG_H

#define MAJOR_VERSION 1
#define MINOR_VERSION 13
#define CONFIG_START 32
//#define BOARD_FIRMWARE "AltiMultiV2"
#define BOARD_FIRMWARE "AltiMulti"
//#define ALTIMULTIV2
#define ALTIMULTI
#include "Arduino.h"
#include <EEPROM.h>

//pyro out 1
extern const int pyroOut1;
extern int pinApogee;
//pyro out 2
extern const int pyroOut2;
extern int pinMain;
//pyro out 3
extern const int pyroOut3;
extern int pinOut3;

extern int pinOut2;
extern int pinOut1;

struct ConfigStruct {
  int unit;             //0 = meter 1 = feet
  int beepingMode;      // decide whiche way you want to report the altitude
  int outPut1;          // assign a function to each pyro
  int outPut2;
  int outPut3;
  int mainAltitude;     //deployment altitude for the main chute
  int superSonicYesNo;  // if set to yes do not do any altitude measurement when altimeter starts
  int outPut1Delay;      // delay output by x ms
  int outPut2Delay;
  int outPut3Delay;
  int beepingFrequency;  // this beeping frequency can be changed
  //int separationVelocity; // decide at which speed to separate
  int nbrOfMeasuresForApogee; //how many measure to decide that apogge has been reached
  int endRecordAltitude;  // stop recording when landing define under which altitude we are not recording
  int recordTemperature;  //decide if we want to record temparature
  int superSonicDelay;   //nbr of ms during when we ignore any altitude measurements
  int cksum;  
};
extern ConfigStruct config;

extern void defaultConfig();
extern boolean readAltiConfig();
extern int getOutPin(int );
extern int writeAltiConfig( char * );
extern void printAltiConfig();
extern void writeConfigStruc();
#endif
