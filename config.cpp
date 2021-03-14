#include "config.h"

//pyro out 1
#ifdef ALTIMULTISTM32
const int pyroOut1 = PA1;
int pinApogee = PA1;
#endif
#ifdef ALTIMULTIV2
const int pyroOut1 = 12; 
//int pinApogee = 12; 
int pinApogee[]= {-1, -1, -1, -1};
#endif
#ifdef ALTIMULTI
const int pyroOut1 = 9;
//int pinApogee = 9;
int pinApogee[]= {-1, -1, -1, -1};
#endif
//pyro out 2
#ifdef ALTIMULTIV2
const int pyroOut2 = 9; 
//int pinMain = 9;
int pinMain[]= {-1, -1, -1, -1};
#endif
#ifdef ALTIMULTI
const int pyroOut2 = 13;
//int pinMain = 13;
int pinMain[]= {-1, -1, -1, -1};
#endif
#ifdef ALTIMULTISTM32
const int pyroOut2 = PA3;
//int pinMain = PA3;
int pinMain[]= {-1, -1, -1, -1};
#endif
//pyro out 3
#ifdef ALTIMULTISTM32
const int pyroOut3 = PA5; //17;
int pinOut3 = PA5;//17;
#else
const int pyroOut3 = 17;
int pinOut3 = 17;
#endif
//pyro out 4
#ifdef ALTIMULTISTM32
const int pyroOut4 = PA7;
int pinOut4 = PA7;
#endif
int pinOut2 = -1;
int pinOut1 = -1;
int continuityPins[4];
//pinLanding
int pinLanding[]= {-1, -1, -1, -1};
//PinLiftOff
int pinLiftOff[]= {-1, -1, -1, -1};
ConfigStruct config;
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
  config.noContinuity = 0;
  config.outPut4 = 3;
  config.outPut4Delay = 0;
  config.liftOffAltitude = 10;
  config.batteryType = 0;
  config.cksum = CheckSumConf(config);
}
bool readAltiConfig() {
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
  write the config received by the console

*/
bool writeAltiConfig( char *p ) {

  char *str;
  int i = 0;
  int strChk=0;
  char msg[100]="";
  
  while ((str = strtok_r(p, ",", &p)) != NULL) // delimiter is the comma
  {
    //SerialCom.println(str);
    switch (i)
    {
      case 1:
        config.unit = atoi(str);
        strcat(msg, str);
        break;
      case 2:
        config.beepingMode = atoi(str);
        strcat(msg, str);
        break;
      case 3:
        config.outPut1 = atoi(str);
        strcat(msg, str);
        break;
      case 4:
        config.outPut2 = atoi(str);
        strcat(msg, str);
        break;
      case 5:
        config.outPut3 = atoi(str);
        strcat(msg, str);
        break;
      case 6:
        config.mainAltitude = atoi(str);
        strcat(msg, str);
        break;
      case 7:
        config.superSonicYesNo = atoi(str);
        strcat(msg, str);
        break;
      case 8:
        config.outPut1Delay = atol(str);
        strcat(msg, str);
        break;
      case 9:
        config.outPut2Delay = atol(str);
        strcat(msg, str);
        break;
      case 10:
        config.outPut3Delay = atol(str);
        strcat(msg, str);
        break;
      case 11:
        config.beepingFrequency = atoi(str);
        strcat(msg, str);
        break;
      case 12:
        config.nbrOfMeasuresForApogee = atoi(str);
        strcat(msg, str);
        break;
      case 13:
        config.endRecordAltitude = atol(str);
        strcat(msg, str);
        break;
      case 14:
        config.recordTemperature = atoi(str);
        strcat(msg, str);
        break;
      case 15:
        config.superSonicDelay = atoi(str);
        strcat(msg, str);
        break;
      case 16:
        config.connectionSpeed = atol(str);
        strcat(msg, str);
        break;
      case 17:
        config.altimeterResolution = atoi(str);
        strcat(msg, str);
        break;
      case 18:
        config.eepromSize = atoi(str);
        strcat(msg, str);
        break;
      case 19:
        config.noContinuity = atoi(str);
        strcat(msg, str);
        break;
      case 20:
        config.outPut4 = atoi(str);
        strcat(msg, str);
        break;
      case 21:
        config.outPut4Delay = atol(str);
        strcat(msg, str);
        break;
      case 22:
        config.liftOffAltitude = atoi(str);
        strcat(msg, str);
        break;
      case 23:
        config.batteryType = atoi(str);
        strcat(msg, str);
        //strcat(msg, "\0");
        break;
      case 24:
        //our checksum
        strChk= atoi(str);
        break;
    }
    i++;

  }
  //we have a partial config
  if (i<23)
    return false;

  if(msgChk(msg, sizeof(msg)) != strChk)
    return false;  
  // add checksum
  config.cksum = CheckSumConf(config);

  writeConfigStruc();
  return true;
}
/*

   Write config structure to the EEPROM

*/
void writeConfigStruc()
{
  int i;
  for ( i = 0; i < sizeof(config); i++ ) {
    EEPROM.write(CONFIG_START + i, *((char*)&config + i));
  }
}
/*

   Print altimeter config to the Serial line

*/
void printAltiConfig()
{
  char altiConfig[120] = "";
  char temp[10] = "";
  bool ret = readAltiConfig();
  if (!ret)
    SerialCom.print(F("invalid conf"));

  strcat(altiConfig, "alticonfig,");
  
  //Unit
  sprintf(temp, "%i,", config.unit);
  strcat(altiConfig, temp);
  //beepingMode
  sprintf(temp, "%i,", config.beepingMode);
  strcat(altiConfig, temp);
  //output1
  sprintf(temp, "%i,", config.outPut1);
  strcat(altiConfig, temp);
  //output2
  sprintf(temp, "%i,", config.outPut2);
  strcat(altiConfig, temp);
  //output3
  sprintf(temp, "%i,", config.outPut3);
  strcat(altiConfig, temp);
  //supersonicYesNo
  sprintf(temp, "%i,", config.superSonicYesNo);
  strcat(altiConfig, temp);
  //mainAltitude
  sprintf(temp, "%i,", config.mainAltitude);
  strcat(altiConfig, temp);
  //AltimeterName
  strcat(altiConfig, BOARD_FIRMWARE);
   strcat(altiConfig,",");
  //alti major version
  sprintf(temp, "%i,", MAJOR_VERSION);
  strcat(altiConfig, temp);
  //alti minor version
  sprintf(temp, "%i,", MINOR_VERSION);
  strcat(altiConfig, temp);
  //output1 delay
  sprintf(temp, "%i,", config.outPut1Delay);
  strcat(altiConfig, temp);
  //output2 delay
  sprintf(temp, "%i,", config.outPut2Delay);
  strcat(altiConfig, temp);
  //output3 delay
  sprintf(temp, "%i,", config.outPut3Delay);
  strcat(altiConfig, temp);
  //Beeping frequency
  sprintf(temp, "%i,", config.beepingFrequency);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.nbrOfMeasuresForApogee);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.endRecordAltitude);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.recordTemperature);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.superSonicDelay);
  strcat(altiConfig, temp);
  sprintf(temp, "%lu,", config.connectionSpeed);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.altimeterResolution);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.eepromSize);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.noContinuity);
  strcat(altiConfig, temp);
  //output4
  sprintf(temp, "%i,", config.outPut4);
  strcat(altiConfig, temp);
  //output4 delay
  sprintf(temp, "%i,", config.outPut4Delay);
  strcat(altiConfig, temp);
  //Lift off altitude
  sprintf(temp, "%i,", config.liftOffAltitude);
  strcat(altiConfig, temp);
  //Battery type
  sprintf(temp, "%i,", config.batteryType);
  strcat(altiConfig, temp);
  unsigned int chk = 0;
  chk = msgChk( altiConfig, sizeof(altiConfig) );
  sprintf(temp, "%i;\n", chk);
  strcat(altiConfig, temp);

  SerialCom.print("$");
  SerialCom.print(altiConfig);


}
bool CheckValideBaudRate(long baudRate)
{
  bool valid = false;
  if (baudRate == 300 ||
      baudRate == 1200 ||
      baudRate == 2400 ||
      baudRate == 4800 ||
      baudRate == 9600 ||
      baudRate == 14400 ||
      baudRate == 19200 ||
      baudRate == 28800 ||
      baudRate == 38400 ||
      baudRate == 57600 ||
      baudRate == 115200 ||
      baudRate == 230400)
    valid = true;
  return valid;
}
long checkEEPromEndAdress(int eepromSize)
{
  /*long endAdress=0;
    switch(eepromSize)
    {
    case 64:
  	endAdress =16384;
  	break;
    case 128:
  	endAdress =16384;
  	break;
    case 256:
  	endAdress =32768;
  	break;
    case 512:
  	endAdress =65536;
  	break;
    case 1024:
  	endAdress =131072;
  	break;
    }*/
  return eepromSize * 128;
}
/*
   Calculate Checksum for the config
*/
unsigned int CheckSumConf( ConfigStruct cnf)
{
  int i;
  unsigned int chk = 0;

  for (i = 0; i < (sizeof(cnf) - sizeof(int)); i++)
    chk += *((char*)&cnf + i);

  return chk;
}
unsigned int msgChk( char * buffer, long length ) {

  long index;
  unsigned int checksum;

  for ( index = 0L, checksum = 0; index < length; checksum += (unsigned int) buffer[index++] );
  return (unsigned int) ( checksum % 256 );

}
