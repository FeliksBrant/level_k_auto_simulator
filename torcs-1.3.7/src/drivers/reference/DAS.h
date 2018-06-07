/***************************************************************************
*
*    file        : DAS.h
*    author      : Guankun Su
*    summary     : driver assistance system
*
****************************************************************************/



#ifndef _DAS_H_
#define _DAS_H_

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <plib/js.h>

#include <tgfclient.h>
#include <portability.h>

#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include <playerpref.h>
#include "pref.h"
#include "human.h"
#include "record.h"

using namespace std;

static const int alarm_period = 40; // 0.8s beep period
static const int alarm_sound_freq = 1000; // ms between each beep
static const int warning_period = 40; // 0.8s beep period
static const int warning_sound_freq = 500; // ms between each beep
static const int beepPeriod = 30; // beep period in millisecond

struct DASmisc{
    int soundCounter;
};

// REQUIRES:
// MODIFIES: 
// EFFECTS: DAS main function
int DAS_main(tCarElt* car, tHumanContext *HCtx, DASmisc &DAS_misc);

// REQUIRES:
// MODIFIES: 
// EFFECTS: try to keep min speed<=speed<=max speed
//          * [ ] speed < min speed
//              * [ ] PID(min speed)
//              * [ ] alarm sound
//          * [ ] speed > max speed
//              * [ ] PID(max speed)
//              * [ ] alarm sound
//          * [ ] min speed <= speed < min speed + 5km/h
//              * [ ] warning sound
//          * [ ] max speed - 5km/h <= speed < max speed
//              * [ ] warning sound
void limit_speed(tCarElt* car, tHumanContext *HCtx, DASmisc &DAS_misc);

// REQUIRES:
// MODIFIES: 
// EFFECTS: PID controller to setspeed
void SpeedPID(float setSpeed, tCarElt* car, tHumanContext *HCtx);

void alarm(DASmisc &DAS_misc);
void warning(DASmisc &DAS_misc);


#endif /* _DAS_H_ */