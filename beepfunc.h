#ifndef _BEEPFUNC_H
#define _BEEPFUNC_H
#include "config.h"
#include "Arduino.h"
#if defined ALTIMULTIESP32 || defined ALTIMULTIESP32_ACCELERO || defined ALTIMULTIESP32_ACCELERO_375 || defined ALTIMULTIESP32_ACCELERO_345
#include <ESP32Tone.h>
#endif
#ifdef ALTIDUOESP32
#include <ESP32Tone.h>
#endif
extern boolean noContinuity;
//Our drogue has been ejected i.e: apogee has been detected
extern boolean allApogeeFiredComplete ;
extern boolean NoBeep;
extern const int pinSpeaker;
extern int beepingFrequency;
extern int pos;

extern void beepAltitude(long altitude);
extern void beginBeepSeq();
extern void longBeep();
extern void shortBeep();
extern void beepAltiVersion (int majorNbr, int minorNbr);
extern void longBeepRepeat( int digit );
extern void shortBeepRepeat( int digit );
extern void beepAltitudeNew( long value);
extern void continuityCheckAsync();
#endif
