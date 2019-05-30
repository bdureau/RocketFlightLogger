//================================================================
// beeping functions
//================================================================
#include "beepfunc.h"

boolean noContinuity = false;
//Our drogue has been ejected i.e: apogee has been detected
boolean apogeeHasFired = false;

boolean NoBeep = false;
#ifdef ALTIMULTIV2
const int pinSpeaker = 13;
#endif
#ifdef ALTIMULTI
const int pinSpeaker = 12;
#endif

#ifdef ALTIMULTISTM32
const int pinSpeaker = PA0;
#endif

int beepingFrequency;

VirtualDelay delay1, delay2, delay3;
int lastPin = -1;
int currentPinPos = 0;
int currentPin = -1;
int pos = -1;

/*void continuityCheck(int pin)
{
  int val = 0;     // variable to store the read value
  // read the input pin to check the continuity if apogee has not fired
  if (noContinuity == false)
    if (apogeeHasFired == false )
    {
      val = digitalRead(pin);
      if (val == 0)
      {
        //no continuity long beep
        longBeep();
      }
      else
      {
        //continuity short beep
        shortBeep();
      }
    }
}*/



void beepAltitude(long altitude)
{
  int i;
  int nbrLongBeep = 0;
  int nbrShortBeep = 0;
  // this is the last thing that I need to write, some code to beep the altitude
  //altitude is in meters
  //find how many digits
  if (altitude > 99)
  {
    // 1 long beep per hundred meter
    nbrLongBeep = int(altitude / 100);
    //then calculate the number of short beep
    nbrShortBeep = (altitude - (nbrLongBeep * 100)) / 10;
  }
  else
  {
    nbrLongBeep = 0;
    nbrShortBeep = (altitude / 10);
  }
  if (nbrLongBeep > 0)
    for (i = 1; i <  nbrLongBeep + 1 ; i++)
    {
      longBeep();
      delay(50);
    }

  if (nbrShortBeep > 0)
    for (i = 1; i <  nbrShortBeep + 1 ; i++)
    {
      shortBeep();
      delay(50);
    }

  delay(5000);

}

void beginBeepSeq()
{
  int i = 0;
  if (NoBeep == false)
  {
    for (i = 0; i < 10; i++)
    {
      tone(pinSpeaker, 1600, 1000);
      delay(50);
      noTone(pinSpeaker);
    }
    delay(1000);
  }
}
void longBeep()
{
  if (NoBeep == false)
  {
    tone(pinSpeaker, beepingFrequency, 1000);
    delay(1500);
    noTone(pinSpeaker);
  }
}
void shortBeep()
{
  if (NoBeep == false)
  {
    tone(pinSpeaker, beepingFrequency, 25);
    delay(300);
    noTone(pinSpeaker);
  }
}
void beepAltiVersion (int majorNbr, int minorNbr)
{
  int i;
  for (i = 0; i < majorNbr; i++)
  {
    longBeep();
  }
  for (i = 0; i < minorNbr; i++)
  {
    shortBeep();
  }
}
//================================================================
// The following is some Code written by Leo Nutz and modified so that it works
// Output the maximum achieved altitude (apogee) via flashing LED / Buzzer
//================================================================

void longBeepRepeat( int digit ) {
#ifdef DEBUG
  SerialCom.println("longBeepRepeat: " );
  SerialCom.println( digit);
#endif
  int i;
  for ( i = 0; i < digit; i++ ) {
    if ( i > 0 ) {
      delay(250);
    }
    longBeep();
  }
}

void shortBeepRepeat( int digit ) {
#ifdef DEBUG
  SerialCom.println("shortBeepRepeat: " );
  SerialCom.println( digit);
#endif
  int i;
  for ( i = 0; i < digit; i++ ) {
    if ( i > 0 ) {
      delay(250);
    }
    shortBeep();
  }
}

void beepAltitudeNew( long value)
{
  char Apogee_String[5];                        // Create an array with a buffer of 5 digits

  ultoa(value, Apogee_String, 10);              // Convert unsigned long to string array, radix 10 for decimal

  //sprintf(Apogee_String,"%l", value);
  uint8_t length = strlen(Apogee_String);       // Get string length

  delay(3000);                                  // Pause for 3 seconds

  for (uint8_t i = 0; i < length; i++ )
  {
    delay(1000);                                // Pause 1 second for every digit output

    uint8_t digit = (Apogee_String[i] - '0');   // Convert ASCI to actual numerical digit
    if ( digit == 0 ) {
      digit = 10;
    }
    if ( digit == 0 ) {
      longBeepRepeat(1);
    }
    else {
      shortBeepRepeat(digit);
    }
  }
}

void continuityCheckNew()
{
  int val = 0;     // variable to store the read value
  // read the input pin to check the continuity if apogee has not fired
  if (noContinuity == false)
    if (apogeeHasFired == false )
    {
      if (lastPin == -1)
      {
#ifdef SERIAL_DEBUG
        SerialCom.println("Last pin =-1");
#endif
        DO_ONCE(delay1.start(0));
        currentPin = continuityPins[currentPinPos];
#ifdef SERIAL_DEBUG
        SerialCom.print("currentPin:");
        SerialCom.println(currentPin);
#endif
        lastPin = currentPin;
      }
      else
      {
        if (delay1.elapsed())
        {
#ifdef SERIAL_DEBUG
          SerialCom.println("delay1.elapsed()");
#endif
          noTone(pinSpeaker);
          lastPin = currentPin;
          currentPinPos++;

          if (currentPinPos > pos) currentPinPos = 0;
#ifdef SERIAL_DEBUG
          SerialCom.print("currentPin:");
          SerialCom.println(currentPin);
          SerialCom.print("currentPinPos:");
          SerialCom.println(currentPinPos);
#endif
          if (currentPinPos == 0)
          {
            delay3.start(2500);
#ifdef SERIAL_DEBUG
            SerialCom.println("delay3 car currentPinPos == 0");
#endif
          }
          else {
            delay2.start(250);
#ifdef SERIAL_DEBUG
            SerialCom.println("delay2 car pas pin 0");
#endif
          }
        }
        if (delay3.elapsed())
        {
          delay2.start(250);
#ifdef SERIAL_DEBUG
          SerialCom.println("delay2 car 3 elapsed");
#endif
        }
        if (delay2.elapsed())
        {
#ifdef SERIAL_DEBUG
          SerialCom.println("delay2.elapsed()");
#endif
          currentPin = continuityPins[currentPinPos];

          val = digitalRead(currentPin);
          if (val == 0)
          {
            //no continuity long beep
#ifdef SERIAL_DEBUG
            SerialCom.println("longBeep");
#endif
            tone(pinSpeaker, beepingFrequency, 1000);
            delay1.start(1500);
            //delay2.start(1750);
          }
          else
          {
            //continuity short beep

#ifdef SERIAL_DEBUG
            SerialCom.println("shortBeep");
#endif
            tone(pinSpeaker, beepingFrequency, 1000);
            delay1.start(300);

          }
        }

      }
    }
}
