#ifndef _CONFIG_H
#define _CONFIG_H

#define MAJOR_VERSION 1
#define MINOR_VERSION 0
#define CONFIG_START 32

#include "Arduino.h"
//used for writing in the microcontroler internal eeprom
#include <EEPROM.h>

struct ConfigStruct {
  int ax_offset;
  int ay_offset;
  int az_offset;
  int gx_offset;
  int gy_offset;
  int gz_offset;
  int KpX;
  int KiX;
  int KdX;
  int KpY;
  int KiY;
  int KdY;
  int ServoXMin;
  int ServoXMax;
  int ServoYMin;
  int ServoYMax;
  int connectionSpeed;
  int altimeterResolution;
  int eepromSize;
  int unit;
  int endRecordAltitude;
  int beepingFrequency;
  int liftOffDetect;
  int cksum;  
};
extern ConfigStruct config;
extern void defaultConfig();
extern boolean readAltiConfig();
extern void writeConfigStruc();
extern unsigned int CheckSumConf( ConfigStruct );
extern bool writeAltiConfig( char *p );
#endif
