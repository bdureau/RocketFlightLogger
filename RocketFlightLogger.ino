/*
Rocket Flight Logger ver 1.0
Copyright Boris du Reau 2012-2013

The following is a datalogger for logging rocket flight.
So far it can log the rocket altitude during the flight.


This is using a BMP085 presure sensor and an Atmega 328
The following record the flight in an EEPROM


 For the BMP085 pressure sensor
 Connect VCC of the BMP085 sensor to 5.0V! make sure that you are using the 5V sensor (GY-65 model)
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

*/


#include <Wire.h> //I2C library
#include <Adafruit_BMP085.h>

void i2c_eeprom_write_byte( int deviceaddress, unsigned int eeaddress, byte data ) {
  int rdata = data;
  int writeDelay=10;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(rdata);
  Wire.endTransmission();
  delay(writeDelay);
}

byte i2c_eeprom_read_byte( int deviceaddress, unsigned int eeaddress ) {
  byte rdata = 0xFF;
  int readDelay=5;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress,1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

void i2c_eeprom_erase_fileList()
{
  Serial.println(F("Start erasing files"));
  for (long i = 0; i < 101; i++)
  {
    i2c_eeprom_write_byte( 0x50, i, 0x00 );
  }
  Serial.println(F("Done erasing file"));
}

//////////////////////////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////////////////////////


int mode = 0; //0 = read; 1 = write;
Adafruit_BMP085 bmp;
long endAddress = 65536;

//ground level altitude
long initialAltitude;
long liftoffAltitude;
long lastAltitude;
//current altitude
long currAltitude;
boolean liftOff= false;
long initialTime;

long currentMemaddress;
long startMemaddress;
long currentFileNbr = 1;

typedef struct fileaddresses
{
  int startaddress;
  int endaddress;
};

fileaddresses fileA[8];

int getFlightList()
{
  //get address where we are allowed to record
  //read the table to see what has been recorded so far

  int i = 0;
  byte startaddressMSB = B00000000;
  byte startaddressLSB = B00000000;
  byte endaddressMSB = B00000000;
  byte endaddressLSB = B00000000;
  boolean finish = false;

  while (i < 101 && finish == false)
  {
    int startaddress;
    int endaddress;
    startaddressMSB = i2c_eeprom_read_byte(0x50, i);
    i++;
    startaddressLSB = i2c_eeprom_read_byte(0x50, i);
    i++;
    startaddress = (startaddressMSB << 8) | startaddressLSB;

    endaddressMSB = i2c_eeprom_read_byte(0x50, i);
    i++;
    endaddressLSB = i2c_eeprom_read_byte(0x50, i);
    i++;
    endaddress = (endaddressMSB << 8) | endaddressLSB;

    fileA[currentFileNbr].startaddress = startaddress;
    fileA[currentFileNbr].endaddress = endaddress;
    
    currentFileNbr++;
    if (startaddress == 0) 
    {
      currentFileNbr = currentFileNbr -1;
      currentMemaddress =fileA[currentFileNbr-1].endaddress +1;
      finish = true;
    } 
  } 
  if (currentMemaddress == 1)
    currentMemaddress=101;

  startMemaddress = currentMemaddress; 
  //return the number of flight
  return (currentFileNbr -1);
}

boolean saveFlightA(int flightNbr, long startAdress, long endAdress)
{

  byte startaddressMSB = B00000000;
  byte startaddressLSB = B00000000;
  byte endaddressMSB = B00000000;
  byte endaddressLSB = B00000000;

  if (flightNbr < 26)
  {
    //save start address
    startaddressMSB = startMemaddress >> 8;
    i2c_eeprom_write_byte(0x50, (currentFileNbr*4)-4, startaddressMSB);

    startaddressLSB = startMemaddress;
    i2c_eeprom_write_byte(0x50, (currentFileNbr*4)-3, startaddressLSB); 
    endaddressMSB = (currentMemaddress -1) >> 8;
    i2c_eeprom_write_byte(0x50, (currentFileNbr*4)-2, endaddressMSB);
    endaddressLSB = (currentMemaddress -1);
    i2c_eeprom_write_byte(0x50, (currentFileNbr*4)-1, endaddressLSB);
    return true;
  }
  else
    return false;

}


void setup()
{
  Wire.begin(); // initialise the connection
  Serial.begin(9600);

  pinMode(A0, INPUT);
  //Presure Sensor Initialisation
  bmp.begin();
  Serial.print(F("Start program\n"));

  //let's read the lauch site altitude
  long sum = 0;
  for (int i=0; i<10; i++){
    sum += bmp.readAltitude();
    delay(50); 
  }
  initialAltitude = (sum / 10.0);
  lastAltitude = initialAltitude; 
  liftoffAltitude = initialAltitude + 20;

  getFlightList();

}

void printFlightList()
{
  int startaddress;
  int endaddress;
  int flightNbr= 1;
  boolean exit=false;
  while(exit == false)
  {
    startaddress = fileA[flightNbr].startaddress;
    endaddress = fileA[flightNbr].endaddress;
    if (startaddress == 0) 
      exit = true;
    else
    {
      Serial.println("Flight Nbr: " + String(flightNbr) + " Start address: " + String(startaddress) + " End address: " + String(endaddress) + "\n");  
      flightNbr ++;
    }
  }
}

void loop()
{
  choice2();
}  

void printFlight(int flightNbr)
{

  byte startaddressMSB = B00000000;
  byte startaddressLSB = B00000000;
  byte endaddressMSB = B00000000;
  byte endaddressLSB = B00000000;
  int startaddress;
  int endaddress;

  startaddressMSB = i2c_eeprom_read_byte(0x50, (flightNbr * 4)-4);

  startaddressLSB = i2c_eeprom_read_byte(0x50, (flightNbr * 4)-3);

  startaddress = (startaddressMSB << 8) | startaddressLSB;

  endaddressMSB = i2c_eeprom_read_byte(0x50, (flightNbr * 4)-2);

  endaddressLSB = i2c_eeprom_read_byte(0x50, (flightNbr * 4)-1);

  endaddress = (endaddressMSB << 8) | endaddressLSB;
  if (startaddress>100)
  {
    int i = startaddress;
    while(i < (endaddress +1))
    {
      long currentTime;
      byte currentTimeMSB = B00000000;
      byte currentTimeLSB = B00000000;
      byte currAltitudeMSB = B00000000;
      byte currAltitudeLSB = B00000000;

      currentTimeMSB = i2c_eeprom_read_byte(0x50, i);
      i++;
      currentTimeLSB = i2c_eeprom_read_byte(0x50, i);
      i++;

      currAltitudeMSB = i2c_eeprom_read_byte(0x50, i);
      i++;
      currAltitudeLSB = i2c_eeprom_read_byte(0x50, i);
      i++;
      Serial.println(String((currentTimeMSB << 8) | currentTimeLSB) + ","+ String((currAltitudeMSB << 8) | currAltitudeLSB) + "\n");
    }
  }
  else
  Serial.println(F("No such flight\n"));
}

void recordAltitude()
{
  boolean exit=false;

  while (exit == false) 
  {
    byte startaddressMSB = B00000000;
    byte startaddressLSB = B00000000;
    byte endaddressMSB = B00000000;
    byte endaddressLSB = B00000000;

    //read current altitude
    currAltitude = bmp.readAltitude()- initialAltitude;
    if (( currAltitude > liftoffAltitude) == true && liftOff == false)
    {
      liftOff = true;
      // save the time
      initialTime =millis();
    }  
    if(liftOff)
    {
      Serial.println(F("we have lift off\n"));
      //Save start address
      fileA[currentFileNbr].startaddress = startMemaddress;
 
      while(currAltitude > liftoffAltitude)
      {
        long currentTime;
        byte currentTimeMSB = B00000000;
        byte currentTimeLSB = B00000000;

        byte currAltitudeMSB = B00000000;
        byte currAltitudeLSB = B00000000;

        currAltitude = bmp.readAltitude()- initialAltitude;

        currentTime = millis()- initialTime;
        currentTimeLSB = currentTime;
        currentTimeMSB = currentTime >> 8;
        i2c_eeprom_write_byte(0x50, currentMemaddress, currentTimeMSB);
        currentMemaddress++;
        i2c_eeprom_write_byte(0x50, currentMemaddress, currentTimeLSB);
        currentMemaddress++;

        currAltitudeLSB = currAltitude;
        currAltitudeMSB = currAltitude >> 8;
        i2c_eeprom_write_byte(0x50, currentMemaddress, currAltitudeMSB);
        currentMemaddress++;
        i2c_eeprom_write_byte(0x50, currentMemaddress, currAltitudeLSB);
        currentMemaddress++;
      }
      //end loging
      //store start and end address
      Serial.println(F("stop recording\n "));
      //save end address
      fileA[currentFileNbr].endaddress = currentMemaddress -1;
      saveFlightA(currentFileNbr, startMemaddress, currentMemaddress -1);
      exit =true;
    }
  }
}


void choice2()
{
  char readVal=' ';
  int i=0;
  //boolean exit = false;
  char commandbuffer[100];
 // String commandBuf;
  Serial.println(F("Rocket flight data logger. A maximum of 25 flight can be logged \n"));
  Serial.println(F("Commands are: \n"));
  Serial.println(F("w = record flight \n"));
  Serial.println(F("r (followed by the flight number) = read flight data\n"));
  Serial.println(F("l  = print flight list \n"));
  Serial.println(F("e  = erase all flight data \n"));
  Serial.println(F("Enter Command and terminate it by a ; >>\n"));
  i=0;
  readVal=' ';
  while( readVal != ';') {
    if(Serial.available()){
      readVal = Serial.read();
      if (readVal != ';' )
      {
        if (readVal != '\n')
        commandbuffer[i++] = readVal;
      }
      else
        commandbuffer[i++]='\0';
    }
  }
  Serial.println((char*)commandbuffer);
  if (commandbuffer[0] == 'e')
  {
    Serial.println(F("Erase\n"));
    i2c_eeprom_erase_fileList();
  }  
  else if (commandbuffer[0] == 'r')  
  {
    char temp[3];
    Serial.println(F("Read flight: "));
    Serial.println( commandbuffer[1]);
    Serial.println( "\n");
    temp[0] = commandbuffer[1];
    if (commandbuffer[2] != '\0')
    {
       temp[1] = commandbuffer[2];
       temp[2] = '\0';
    }
    else
    temp[1] ='\0';
    
    if(atol(temp)>0)
    printFlight(atol(temp));
    else
     Serial.println(F("not a valid flight"));
  }
  else if (commandbuffer[0] == 'w') 
  {
    Serial.println(F("Recording \n"));
    recordAltitude();
  }
  else if (commandbuffer[0] == 'l')
  {
    Serial.println(F("Flight List: \n"));
    printFlightList();
  }
  else
  {
    Serial.println(F("Unknown command\n"));
  }
}




