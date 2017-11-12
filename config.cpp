#include "config.h"

//pyro out 1
const int pyroOut1 = 9;
int pinApogee = 9;
//pyro out 2
#ifdef ALTIMULTIV2
const int pyroOut2= 12;
int pinMain = 12;
#endif
#ifdef ALTIMULTI
const int pyroOut2= 13;
int pinMain = 13;
#endif
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
  config.recordTemperature =0;  //decide if we want to record temperature
  config.superSonicDelay =0;
  config.connectionSpeed =57600;
  config.altimeterResolution = 0; //0 to 4 ie: from low resolution to high
  config.eepromSize=512;
  config.noContinuity = 0;
  config.cksum=0xBA;  
}
boolean readAltiConfig() {
	//set the config to default values so that if any have not been configured we can use the default ones
	defaultConfig();
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
void writeAltiConfig( char *p ) {

  char *str;
  int i=0;
  while ((str = strtok_r(p, ",", &p)) != NULL) // delimiter is the comma
  {
    Serial.println(str);
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
    case 20:
    	Serial.print(F("WTF "));
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
    Serial.print(F("End address: "));
    Serial.print(CONFIG_START+i);
    Serial.print(F("EEPROM length: "));
    Serial.print(EEPROM.length());
}

void printAltiConfig()
{

  bool ret= readAltiConfig();
  if(!ret)
	  Serial.print(F("invalid conf"));
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
  Serial.print(F(BOARD_FIRMWARE));
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
  Serial.print(F(","));
  Serial.print(config.nbrOfMeasuresForApogee);
  Serial.print(F(","));
  Serial.print(config.endRecordAltitude);
  Serial.print(F(","));
  Serial.print(config.recordTemperature);
  Serial.print(F(","));
  Serial.print(config.superSonicDelay);
  Serial.print(F(","));
  Serial.print(config.connectionSpeed);
  Serial.print(F(","));
  Serial.print(config.altimeterResolution);
  Serial.print(F(","));
  Serial.print(config.eepromSize);
  Serial.print(F(","));
  Serial.print(config.noContinuity);
  Serial.print(F(";\n"));

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
