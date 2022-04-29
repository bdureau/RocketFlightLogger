//================================================================
// beeping functions
//================================================================
#include "beepfunc.h"

boolean noContinuity = false;


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

#ifdef ALTIMULTIESP32
//#include <ESP32Tone.h>
const int pinSpeaker = 16;
#endif
int beepingFrequency;


int lastPin = -1;
int currentPinPos = 0;
int currentPin = -1;
int pos = -1;

long tempo = 0;
long startTempo = 0;
long tempo2 = 0;
long startTempo2 = 0 ;
long bigdelay = 0;
long savedDelay =0;

/*
 * 
 * void beepAltitude(long altitude)
 * 
 * 
 */
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
      #ifndef ALTIMULTIESP32
      tone(pinSpeaker, 1600, 1000);
      delay(50);
      noTone(pinSpeaker);
      #endif
      #ifdef ALTIMULTIESP32
      tone(pinSpeaker, 1600, 50);
      noTone(pinSpeaker);
      delay(50);
      #endif
    }
    delay(1000);
  }
}
void longBeep()
{
  if (NoBeep == false)
  {
    #ifndef ALTIMULTIESP32
    tone(pinSpeaker, beepingFrequency, 1000);
    delay(1500);
    noTone(pinSpeaker);
    #endif

    #ifdef ALTIMULTIESP32
    tone(pinSpeaker, beepingFrequency, 1000);
    delay(1000);
    noTone(pinSpeaker);
    #endif
  }
}
void shortBeep()
{
  if (NoBeep == false)
  {
   #ifndef ALTIMULTIESP32
    tone(pinSpeaker, beepingFrequency, 25);
    delay(300);
    noTone(pinSpeaker);
   #endif
   #ifdef ALTIMULTIESP32
    tone(pinSpeaker, beepingFrequency, 25);
    noTone(pinSpeaker);
    delay(300);
   #endif
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

/*
 * 
 * continuityCheckAsync()
 * This will check continuity on all altimeter outputs then wait for 10s and do it again
 * long beep = no continuity
 * short beep = continuity
 */

void continuityCheckAsync()
{
#ifndef ALTIMULTIESP322222
  int val = 0;     // variable to store the read value
  if (!noContinuity && !allApogeeFiredComplete )
  {
    if ((millis() - savedDelay ) > bigdelay ) {
      if (lastPin == -1)
      {
        currentPin = continuityPins[currentPinPos];
        currentPinPos++;
        if (currentPinPos > pos)
        {
          currentPinPos = 0;
        }


        lastPin = currentPin;

        if (currentPin != -1) {
          val = digitalRead(currentPin);
          if (val == 0)
          {
            // no continuity long beep
            tempo = 1500;
          }
          else {
            //short beep
            tempo = 300;
          }

          tone(pinSpeaker, beepingFrequency);
          startTempo = millis();
        }
        else {
          noTone(pinSpeaker);
        }

      }
      else {
        if ((millis() - startTempo   ) > tempo && tempo > 0) {
          //Serial1.print("End tempo:");
          //Serial1.println((millis() - startTempo ));

          noTone(pinSpeaker);
          tempo = 0;
          tempo2 = 500;
          startTempo2 = millis();
        }
        else {
          if ((millis() - startTempo2  ) > tempo2 && tempo2 > 0) {
            lastPin = -1;
             if (currentPinPos == 0) {
                // Then 10s delay
                bigdelay = 10000;
                savedDelay = millis();
             }
            tempo2 = 0;
          }
        }
      }
    }
  }
  else
    noTone(pinSpeaker);
#endif
}
