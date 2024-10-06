#include "config.h"

//pyro out 1
#ifdef ALTIMULTISTM32
const int pyroOut1 = PA1;
#endif
#ifdef ALTIMULTIV2
const int pyroOut1 = 9;  
#endif
#ifdef ALTIMULTI
const int pyroOut1 = 9;
#endif
/*#ifdef ALTIMULTIESP32
const int pyroOut1 = 2;
#endif*/
#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
//const int pyroOut1 = 2;
const int pyroOut1 = 18;
#endif
#ifdef ALTIDUOESP32
const int pyroOut1 = -1;
#endif
//pyro out 2
#ifdef ALTIMULTIV2
const int pyroOut2 = 12;  
#endif
#ifdef ALTIMULTI
const int pyroOut2 = 13;
#endif
#ifdef ALTIMULTISTM32
const int pyroOut2 = PA3;
#endif
/*#ifdef ALTIMULTIESP32
const int pyroOut2 = 18;
#endif*/
#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
//const int pyroOut2 = 18;
const int pyroOut2 = 2;
#endif
#ifdef ALTIDUOESP32
const int pyroOut2 = -1;
#endif
//pyro out 3
#ifdef ALTIMULTISTM32
const int pyroOut3 = PA5; 
#endif
#ifdef ALTIMULTI
const int pyroOut3 = 17;
#endif
#ifdef ALTIMULTIV2
const int pyroOut3 = 17;
#endif
/*#ifdef ALTIMULTIESP32
const int pyroOut3 = 19;
#endif*/
#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
const int pyroOut3 = 19;

#endif
//pyro out 4
#ifdef ALTIMULTISTM32
const int pyroOut4 = PA7;
#endif

int continuityPins[4];

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
  config.nbrOfMeasuresForApogee = 5;
  config.endRecordAltitude = 3; // stop recording when landing define under which altitude we are not recording
  config.telemetryType = 0; //what type of telemetry module
  config.superSonicDelay = 0;
  config.connectionSpeed = 38400;
  config.altimeterResolution = 0; //0 to 4 ie: from low resolution to high
  config.eepromSize = 512;
  config.noContinuity = 0;
  config.outPut4 = 3;
  config.outPut4Delay = 0;
  config.liftOffAltitude = 10;
  config.batteryType = 0;
  config.recordingTimeout = 120;
  config.altiID = 0;
  config.useTelemetryPort =0;
  config.cksum = CheckSumConf(config);
}

bool readAltiConfig() {
  //set the config to default values so that if any have not been configured we can use the default ones
  defaultConfig();
  int i;
  #ifdef ALTIMULTIESP32
  EEPROM.begin(512);
  #endif
  #if defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
  EEPROM.begin(512);
  #endif
  for ( i = 0; i < sizeof(config); i++ ) {
    *((char*)&config + i) = EEPROM.read(CONFIG_START + i);
  }
  #ifdef ALTIMULTIESP32
  EEPROM.end();
  #endif
  #if defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
  EEPROM.end();
  #endif
  if ( config.cksum != CheckSumConf(config) ) {
    return false;
  }

  return true;

}


/*
  write the config received by the console

*/

bool writeAltiConfigV2( char *p ) {

  char *str;
  int i = 0;
  int command =0;
  long commandVal =0;
  int strChk = 0;
  char msg[100] = "";

  while ((str = strtok_r(p, ",", &p)) != NULL) // delimiter is the comma
  {
    //SerialCom.println(str);
    if (i == 1) {
      command = atoi(str);
      strcat(msg, str);
    }
    if (i == 2) {
      commandVal =  atol(str);
      strcat(msg, str);
    }
    if (i == 3) {
      strChk  =  atoi(str);  
    }
    i++;

  }
    //we have a partial config
  if (i < 4)
    return false;
  //checksum is ivalid ? 
  if (msgChk(msg, sizeof(msg)) != strChk)
    return false;  
    
  switch (command)
    {
      case 1:
        config.unit = (int) commandVal;
        break;
      case 2:
        config.beepingMode = (int) commandVal;
        break;
      case 3:
        config.outPut1 = (int) commandVal;
        break;
      case 4:
        config.outPut2 = (int) commandVal;
        break;
      case 5:
        config.outPut3 = (int) commandVal;
        break;
      case 6:
        config.mainAltitude = (int) commandVal;
        break;
      case 7:
        config.superSonicYesNo = (int)commandVal;
        break;
      case 8:
        config.outPut1Delay = (int)commandVal;
        break;
      case 9:
        config.outPut2Delay = (int)commandVal;
        break;
      case 10:
        config.outPut3Delay = (int)commandVal;
        break;
      case 11:
        config.beepingFrequency = (int)commandVal;
        break;
      case 12:
        config.nbrOfMeasuresForApogee =(int) commandVal;
        break;
      case 13:
        config.endRecordAltitude = (int)commandVal;
        break;
      case 14:
        config.telemetryType = (int)commandVal;
        break;
      case 15:
        config.superSonicDelay =(int)commandVal;
        break;
      case 16:
        config.connectionSpeed = commandVal;
        break;
      case 17:
        config.altimeterResolution = (int)commandVal;
        break;
      case 18:
        config.eepromSize = (int)commandVal;
        break;
      case 19:
        config.noContinuity = (int)commandVal;
        break;
      case 20:
        config.outPut4 = (int)commandVal;
        break;
      case 21:
        config.outPut4Delay = (int)commandVal;
        break;
      case 22:
        config.liftOffAltitude = (int)commandVal;
        break;
      case 23:
        config.batteryType = (int)commandVal;
        break;
      case 24:
        config.recordingTimeout = (int)commandVal;
        break;
      case 25:
        config.altiID = (int)commandVal;
        break;
      case 26:
        config.useTelemetryPort = (int)commandVal;  
        break;  
    }

  // add checksum
  config.cksum = CheckSumConf(config);

  return true;
}
/*

   Write config structure to the EEPROM

*/
void writeConfigStruc()
{
  int i;
  #ifdef ALTIMULTIESP32
  EEPROM.begin(512);
  #endif
  #if defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
  EEPROM.begin(512);
  #endif
  for ( i = 0; i < sizeof(config); i++ ) {
    EEPROM.write(CONFIG_START + i, *((char*)&config + i));
  }
  #ifdef ALTIMULTIESP32
  EEPROM.commit();
  EEPROM.end();
  #endif
  #if defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
  EEPROM.commit();
  EEPROM.end();
  #endif
}
/*

   Print altimeter config to the Serial line

*/
void printAltiConfig(char *altiName)
{
  // char *altiConfig;
 // altiConfig = (char *) malloc(120);
  char altiConfig[120] = "";
  char temp[25] = "";
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
  sprintf(temp, "%i,", config.telemetryType);
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
  // recording timeout
  sprintf(temp, "%i,", config.recordingTimeout);
  strcat(altiConfig, temp);
  //altiID
  sprintf(temp, "%i,", config.altiID);
  strcat(altiConfig, temp);
  //useTelemetryPort
  sprintf(temp, "%i,", config.useTelemetryPort);
  strcat(altiConfig, temp);
  #if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
  //strcpy(temp, altiName);
  sprintf(temp, "%s,",altiName);
  strcat(altiConfig,temp);
  #endif
  unsigned int chk = 0;
  chk = msgChk( altiConfig, sizeof(altiConfig) );
  sprintf(temp, "%i;\n", chk);
  strcat(altiConfig, temp);

  /*#ifdef ALTIMULTIESP32
  Serial.print("$");
  Serial.print(altiConfig);
  #endif*/
  #ifdef TELEMETRY_ESP32
  //#if defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
  Serial.print("$");
  Serial.print(altiConfig);
  #endif
  SerialCom.print("$");
  //SerialCom.print(altiConfig);
  // the following will slow down the connection speed so that it works better with telemetry module
  // or bluetooth module with no buffer
  if (config.useTelemetryPort == 1){
  for(int j=0; j< sizeof(altiConfig);j++) {
    SerialCom.print(altiConfig[j]);
    delay(2);
  }
  } 
  else
    SerialCom.print(altiConfig);
  
  
  
  

//free(altiConfig);
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
