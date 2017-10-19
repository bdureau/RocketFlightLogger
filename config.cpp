#include "config.h"

//pyro out 1
const int pyroOut1 = 9;
int pinApogee = 9;
//pyro out 2
const int pyroOut2= 13;
int pinMain = 13;
//pyro out 3
const int pyroOut3=17;
int pinOut3 =17;
int pinOut2 =-1;
int pinOut1 =-1;
ConfigStruct config;
//================================================================
// read and write in the microcontroler eeprom
//================================================================
void defaultConfig()
{
  
  config.unit = 0;
  config.beepingMode=0;
  config.outPut1=0;
  config.outPut2=1;
  config.outPut3=3;
  config.outPut1Delay=0;
  config.outPut2Delay=0;
  config.outPut3Delay=0;
  config.mainAltitude=50;
  config.superSonicYesNo=0;
  config.beepingFrequency = 440;
  //config.separationVelocity = 10; 
  config.nbrOfMeasuresForApogee = 5;
  config.endRecordAltitude=3;  // stop recording when landing define under which altitude we are not recording
  config.recordTemperature =0;  //decide if we want to record temparature
  config.superSonicDelay =0;
  config.cksum=0xBA;  
}
boolean readAltiConfig() {

  int i;
  for( i=0; i< sizeof(config); i++ ) {
    *((char*)&config + i) = EEPROM.read(CONFIG_START + i);
  }

  if ( config.cksum != 0xBA ) {
    return false;
  }

  return true;

}

/*int getOutPin(int value)
{

  switch (value)
  {
  case 0:
    pinMain = pyroOut1;
    break;
  case 1:
    pinApogee = pyroOut2;
    break;
  case 2:
    pinOut3 = pyroOut3;
    break; 
  default:
    break;
  }
  return 1;
}*/
/*
* write the config received by the console
*
*/
int writeAltiConfig( char *p ) {

  char *str;
  int i=0;
  while ((str = strtok_r(p, ",", &p)) != NULL) // delimiter is the comma
  {
    Serial.println(str);
    switch (i)
    {
    case 1:
      config.unit =atol(str);
      break;
    case 2:
      config.beepingMode=atol(str);
      break;
    case 3:
      config.outPut1=atol(str);
      break;   
    case 4:
      config.outPut2=atol(str);
      break;
    case 5:
      config.outPut3=atol(str);
      break;
    case 6:
      config.mainAltitude=atol(str);
      break;
    case 7:
      config.superSonicYesNo=atol(str);
      break;
    case 8:
      config.outPut1Delay=atol(str);
      break;
    case 9:
      config.outPut2Delay=atol(str);
      break;
    case 10:
      config.outPut3Delay=atol(str);
      break;
    case 11:
      config.beepingFrequency =atol(str);
      break;
    case 12:
      break;
    }
    i++;

  }

  config.cksum = 0xBA;

  /*for( i=0; i<sizeof(config); i++ ) {
    EEPROM.write(CONFIG_START+i, *((char*)&config + i));
  }*/
  writeConfigStruc();
}
//////////////////////////////////////////////////////////////////////////////////////
void writeConfigStruc()
{
    int i;
    for( i=0; i<sizeof(config); i++ ) {
      EEPROM.write(CONFIG_START+i, *((char*)&config + i));
    }
}

void printAltiConfig()
{
  readAltiConfig();
  Serial.print(F("$alticonfig"));
  Serial.print(F(","));
  //Unit
  Serial.print(config.unit);
  Serial.print(F(","));
  //beepingMode
  Serial.print(config.beepingMode);
  Serial.print(F(","));
  //output1
  Serial.print(config.outPut1);
  Serial.print(F(","));
  //output2
  Serial.print(config.outPut2);
  Serial.print(F(","));
  //output3
  Serial.print(config.outPut3);
  Serial.print(F(","));
  //supersonicYesNo
  Serial.print(config.superSonicYesNo);
  Serial.print(F(","));
  //mainAltitude
  Serial.print(config.mainAltitude);
  Serial.print(F(","));
  //AltimeterName
  Serial.print(F("AltiMulti"));
  Serial.print(F(","));
  //alti major version
  Serial.print(MAJOR_VERSION);
  //alti minor version
  Serial.print(F(","));
  Serial.print(MINOR_VERSION);
  Serial.print(F(","));
  //output1 delay
  Serial.print(config.outPut1Delay);
  Serial.print(F(","));
  //output2 delay
  Serial.print(config.outPut2Delay);
  Serial.print(F(","));
  //output3 delay
  Serial.print(config.outPut3Delay);
  Serial.print(F(","));
  //Beeping frequency
  Serial.print(config.beepingFrequency);
  Serial.print(F(";\n"));

}
