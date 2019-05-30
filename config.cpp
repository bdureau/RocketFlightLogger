#include "config.h"

//pyro out 1
#ifdef ALTIMULTISTM32
const int pyroOut1 = PA1;//9;
int pinApogee = PA1;//9;
#else
const int pyroOut1 = 9;
int pinApogee = 9;
#endif
//pyro out 2
#ifdef ALTIMULTIV2
const int pyroOut2= 12;
int pinMain = 12;
#endif
#ifdef ALTIMULTI
const int pyroOut2= 13;
int pinMain = 13;
#endif
#ifdef ALTIMULTISTM32
const int pyroOut2= PA3;
int pinMain = PA3;
#endif
//pyro out 3
#ifdef ALTIMULTISTM32
const int pyroOut3=PA5; //17;
int pinOut3 = PA5;//17;
#else
const int pyroOut3= 17;
int pinOut3 = 17;
#endif
//pyro out 4
#ifdef ALTIMULTISTM32
const int pyroOut4=PA7; 
int pinOut4 = PA7;
#endif
int pinOut2 =-1;
int pinOut1 =-1;
int continuityPins[4];
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
  config.recordTemperature =0;  //decide if we want to record temperature
  config.superSonicDelay =0;
  config.connectionSpeed =38400;
  config.altimeterResolution = 0; //0 to 4 ie: from low resolution to high
  config.eepromSize=512;
  config.noContinuity = 0;
  #ifdef NBR_PYRO_OUT4
  config.outPut4=3;
  config.outPut4Delay=0;
  #endif
  config.cksum=CheckSumConf(config);   
}
boolean readAltiConfig() {
	//set the config to default values so that if any have not been configured we can use the default ones
	defaultConfig();
  int i;
  for( i=0; i< sizeof(config); i++ ) {
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
* write the config received by the console
*
*/
void writeAltiConfig( char *p ) {

  char *str;
  int i=0;
  while ((str = strtok_r(p, ",", &p)) != NULL) // delimiter is the comma
  {
    SerialCom.println(str);
    switch (i)
    {
    case 1:
      config.unit =atoi(str);
      break;
    case 2:
      config.beepingMode=atoi(str);
      break;
    case 3:
      config.outPut1=atoi(str);
      break;   
    case 4:
      config.outPut2=atoi(str);
      break;
    case 5:
      config.outPut3=atoi(str);
      break;
    case 6:
      config.mainAltitude=atoi(str);
      break;
    case 7:
      config.superSonicYesNo=atoi(str);
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
      config.beepingFrequency =atoi(str);
      break;
    case 12:
      config.nbrOfMeasuresForApogee=atoi(str);
      break;
    case 13:
      config.endRecordAltitude=atol(str);
      break;
    case 14:
      config.recordTemperature=atoi(str);
      break;
    case 15:
      config.superSonicDelay=atoi(str);
      break;
    case 16:
      config.connectionSpeed=atol(str);
      break;
    case 17:
      config.altimeterResolution=atoi(str);
      break;
    case 18:
      config.eepromSize =atoi(str);
      break;
    case 19:
      config.noContinuity=atoi(str);
      break;
    #ifdef NBR_PYRO_OUT4
    case 20:
      config.outPut4=atoi(str);
      break;  
    case 21:
      config.outPut4Delay=atol(str);
    	//SerialCom.print(F("WTF "));
      break;
    #endif
    }
    i++;

  }

  config.cksum = CheckSumConf(config);

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
    SerialCom.print(F("End address: "));
    SerialCom.print(CONFIG_START+i);
    SerialCom.print(F("EEPROM length: "));
    //Serial.print(EEPROM.length());
}

void printAltiConfig()
{

  bool ret= readAltiConfig();
  if(!ret)
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
  SerialCom.print(F(BOARD_FIRMWARE));
  SerialCom.print(F(","));
  //alti major version
  SerialCom.print(MAJOR_VERSION);
  //alti minor version
  SerialCom.print(F(","));
  SerialCom.print(MINOR_VERSION);
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
  
  #ifdef NBR_PYRO_OUT4
  SerialCom.print(F(","));
  //output4
  SerialCom.print(config.outPut4);
  SerialCom.print(F(","));
   //output4 delay
  SerialCom.print(config.outPut4Delay);
  //SerialCom.print(F(","));
  #endif
  SerialCom.print(F(";\n"));

}
bool CheckValideBaudRate(long baudRate)
{
	bool valid = false;
	if(baudRate == 300 ||
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
	return eepromSize*128;
}

unsigned int CheckSumConf( ConfigStruct cnf)
 {
     int i;
     unsigned int chk=0;
    
     for (i=0; i < (sizeof(cnf)-2); i++) 
     chk += *((char*)&cnf + i);
    
     return chk;
 }
