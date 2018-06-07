/***************************************************************************
*
*    file        : DAS.h
*    author      : Guankun Su
*    summary     : driver assistance system
*
****************************************************************************/

#include "DAS.h"
#include <windows.h>

// REQUIRES:
// MODIFIES: 
// EFFECTS: DAS main function
int DAS_main(tCarElt* car, tHumanContext *HCtx, DASmisc &DAS_misc){
    limit_speed(car, HCtx, DAS_misc);
    return 0;
}

// REQUIRES:
// MODIFIES: 
// EFFECTS: try to keep min speed<=speed<=max speed
//          * speed < min speed - 5km/h
//              * PID(min speed)
//              * alarm sound
//          * min speed - 5km/h <= speed < min speed
//              * alarm sound
//          * min speed <= speed < min speed + 5km/h
//              * warning sound
//          * max speed - 5km/h < speed <= max speed
//              * warning sound
//          * max speed < speed <= max speed + 5 km/h
//              * alarm sound
//          * max speed + 5 km/h < speed
//              * PID(max speed)
//              * alarm sound
void limit_speed(tCarElt* car, tHumanContext *HCtx, DASmisc &DAS_misc){
    double velLong, velLat;
    getVel(car, velLong, velLat);

    if(velLong < (min_speed-speed_limit_margin)){
        SpeedPID(min_speed, car, HCtx);
        alarm(DAS_misc);
    }else if((min_speed-speed_limit_margin) <= velLong && velLong < min_speed){
        alarm(DAS_misc);
    }else if(min_speed <= velLong && velLong < (min_speed+speed_limit_margin)){
        warning(DAS_misc);
    }else if((max_speed-speed_limit_margin) < velLong && velLong <= max_speed){
        warning(DAS_misc);
    }else if(max_speed < velLong && velLong <= (max_speed+speed_limit_margin)){
        alarm(DAS_misc);
    }else if((max_speed+speed_limit_margin) < velLong){
        SpeedPID(max_speed, car, HCtx);
        alarm(DAS_misc);
    }else{
        DAS_misc.soundCounter = 0;
    }
}

//MODIFIES: DAS_misc.soundCounter
//EFFECTS: produce alarm sound
void alarm(DASmisc &DAS_misc){
    if(DAS_misc.soundCounter > alarm_period){
        Beep(alarm_sound_freq,beepPeriod);
        DAS_misc.soundCounter = 0;
    }else{
        ++DAS_misc.soundCounter;
    }
}

//MODIFIES: DAS_misc.soundCounter
//EFFECTS: produce warning sound
void warning(DASmisc &DAS_misc){
    if(DAS_misc.soundCounter > warning_period){
        Beep(warning_sound_freq,beepPeriod);
        DAS_misc.soundCounter = 0;
    }else{
        ++DAS_misc.soundCounter;
    }
}

// REQUIRES:
// MODIFIES: 
// EFFECTS: PID controller to setspeed
void SpeedPID(float setSpeed, tCarElt* car, tHumanContext *HCtx)
{
    float realAccel = sqrt(car->pub.DynGCg.acc.x*car->pub.DynGCg.acc.x+car->pub.DynGCg.acc.y*car->pub.DynGCg.acc.y);
    if((car->pub.DynGCg.acc.x*car->_speed_x+car->pub.DynGCg.acc.y*car->_speed_y)<0) realAccel = -realAccel;

    double velLong, velLat;
    getVel(car, velLong, velLat);

    // P error
    float speedDelta = (setSpeed - velLong)/setSpeed; // normalized by setSpeed
    // D error
    float accel0; // normalized by speed error over 0.002s
    if(abs(speedDelta)<0.1) accel0 = realAccel/(0.1/0.002); // prevent being divided by a extremem small value
    else accel0 = realAccel/(speedDelta/0.002);
    // I error
    if(abs(velLong-setSpeed) < speedPID_I_margin) HCtx->speedPID_int += speedDelta;
    // total PID value 
    float PIDvalue = speedPID_accel_P * speedDelta - speedPID_accel_D * accel0 + speedPID_accel_I * HCtx->speedPID_int;

    // actuator
    if(PIDvalue > 0)
    {
        car->_brakeCmd = 0;
        car->_accelCmd = PIDvalue;
    }
    else
    {
        car->_brakeCmd = -PIDvalue;
        car->_accelCmd = 0;
    }
}