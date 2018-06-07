/***************************************************************************
*
*    file        : DAS.h
*    author      : Guankun Su
*    summary     : driver assistance system
*
****************************************************************************/

#include "DAS.h"

#ifdef _WIN32
#include <windows.h>
#endif


#include "LevelKDriver.h"
#include "record.h"


// REQUIRES:
// MODIFIES: 
// EFFECTS: DAS main function
int DAS_main(tSituation *s, tCarElt* car, tHumanContext *HCtx, DASmisc &DAS_misc, observeData &obsData){

    if (isStuck_(car, DAS_misc)) {
        cout<<"stucked "<<s->currentTime<<endl;
        car->_steerCmd = getYaw(car) / car->_steerLock;
        DAS_misc.goReverse = true;     // Reverse gear.
        car->_accelCmd = 1.0f;  // 100% accelerator pedal.
        car->_brakeCmd = 0.0f;  // No brakes.
        car->_clutchCmd = 0.0f; // Full clutch (gearbox connected with engine).
    }else{
        limit_speed(car, HCtx, DAS_misc, obsData);
        laneChangeAssist(s, car, DAS_misc);
        DAS_misc.goReverse = false;     // not Reverse gear.
    }

    return 0;
}

bool safeMod_NearCar(float dist, float multiplier) {
    return dist < (LevelKDriver::carlength * multiplier) && dist > 0;
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
void limit_speed(tCarElt* car, tHumanContext *HCtx, DASmisc &DAS_misc, observeData &obsData){
    double velLong, velLat;
    getVel(velLong, velLat, car);
    float min_speed = LevelKDriver::min_speed;
    float max_speed = LevelKDriver::max_speed;

    if(velLong < (min_speed-speed_limit_margin)){
        if(!safeMod_NearCar(obsData.fc_d, 3.0)){
            SpeedPID(min_speed, car, HCtx);
        }else{
            if(velLong < 30.0/3.6){
                SpeedPID(30.0/3.6, car, HCtx);                
            }
        }
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
        #ifdef _WIN32
        Beep(alarm_sound_freq,beepPeriod);
        #endif
        DAS_misc.soundCounter = 0;
    }else{
        ++DAS_misc.soundCounter;
    }
}

//MODIFIES: DAS_misc.soundCounter
//EFFECTS: produce warning sound
void warning(DASmisc &DAS_misc){
    if(DAS_misc.soundCounter > warning_period){
        #ifdef _WIN32
        Beep(warning_sound_freq,beepPeriod);
        #endif
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
    float speedPID_I_margin = LevelKDriver::speedPID_I_margin; 
    float speedPID_accel_P = LevelKDriver::speedPID_accel_P;
    float speedPID_accel_D = LevelKDriver::speedPID_accel_D;
    float speedPID_accel_I = LevelKDriver::speedPID_accel_I;

    float realAccel = sqrt(car->pub.DynGCg.acc.x*car->pub.DynGCg.acc.x+car->pub.DynGCg.acc.y*car->pub.DynGCg.acc.y);
    if((car->pub.DynGCg.acc.x*car->_speed_x+car->pub.DynGCg.acc.y*car->_speed_y)<0) realAccel = -realAccel;

    double velLong, velLat;
    getVel(velLong, velLat, car);

    // P error
    float speedDelta = (setSpeed - velLong)/setSpeed; // normalized by setSpeed
    // D error
    float accel0; // normalized by speed error over 0.002s
    if(abs(speedDelta)<0.1) accel0 = realAccel/(0.1/0.002); // prevent being divided by a extremem small value
    else accel0 = realAccel/(speedDelta/0.002);
    // I error
    if(abs(velLong-setSpeed) < speedPID_I_margin) HCtx->speedPID_int += speedDelta;
    // total PID value 
    float PIDvalue = speedPID_accel_P * speedDelta + speedPID_accel_D * accel0 + speedPID_accel_I * HCtx->speedPID_int;

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

void laneChangeAssist(tSituation *s, tCarElt* car, DASmisc &DAS_misc){

    float oneChangeThreshold = 0.05;
    float twoChangeThreshold = 0.15;

    if(car->_steerCmd<-twoChangeThreshold) DAS_misc.laneChangeState = -2;
    else if(car->_steerCmd>twoChangeThreshold) DAS_misc.laneChangeState = 2;
    else if(car->_steerCmd<-oneChangeThreshold) DAS_misc.laneChangeState = -1;
    else if(car->_steerCmd>oneChangeThreshold) DAS_misc.laneChangeState = 1;
    else DAS_misc.laneChangeState = 0;

    float laneChangeHappen=false;

    if(DAS_misc.laneChangeState_old==-1 && DAS_misc.laneChangeState == 0){
        --DAS_misc.lanePos;
        laneChangeHappen=true;
    } 
    if(DAS_misc.laneChangeState_old==1 && DAS_misc.laneChangeState == 0){
        ++DAS_misc.lanePos;
        laneChangeHappen=true;
    }
    if(DAS_misc.laneChangeState_old==-2 && DAS_misc.laneChangeState == 0){
        --DAS_misc.lanePos;
        // --DAS_misc.lanePos;
        laneChangeHappen=true;
    } 
    if(DAS_misc.laneChangeState_old==2 && DAS_misc.laneChangeState == 0){
        ++DAS_misc.lanePos;
        // ++DAS_misc.lanePos;
        laneChangeHappen=true;
    }

    DAS_misc.laneChangeState_old = DAS_misc.laneChangeState;

    if(DAS_misc.lanePos<-1){
        DAS_misc.lanePos=-1;
        laneChangeHappen=false;
    }
    else if(DAS_misc.lanePos>1){
        DAS_misc.lanePos=1;
        laneChangeHappen=false;
    }

    if(laneChangeHappen){
        double distLon, distLat;
        getPos(distLon, distLat, car);
        DAS_misc.t0 = s->currentTime;
        DAS_misc.y0 = distLat;
        laneChangeHappen=false;
        // cout<<"change lane "<< s->currentTime <<endl;
    }

    car->_steerCmd = GetSteer(s,car,DAS_misc.lanePos,DAS_misc);
}




// Compute steer value.
float GetSteer(tSituation *s, tCarElt* car, int targetLane, DASmisc &DAS_misc)
{
    float steer_P = LevelKDriver::steer_P;
    float steer_D = LevelKDriver::steer_D;
    float targetAngle;
    vec2f target = GetTargetPoint(car, targetLane, DAS_misc, s);

    targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
    DAS_misc.angleError = targetAngle - car->_yaw;

    //map to -M_PI to M_PI
    while(DAS_misc.angleError<-M_PI) DAS_misc.angleError+=2*M_PI;
    while(DAS_misc.angleError>M_PI) DAS_misc.angleError-=2*M_PI;    

    double dTime = s->currentTime - DAS_misc.oldsimtime;
    dTime = (dTime <= 0) ? 0.02 : dTime;
    double Ppart = DAS_misc.angleError * steer_P;
    double Dpart = (DAS_misc.angleError - DAS_misc.angleError_old) / dTime * steer_D;

    targetAngle = Ppart + Dpart;

    // cout<<" human steer PD "<< car->_yaw << " " << DAS_misc.angleError << " " <<Ppart<<" "<<Dpart<<" "<<dTime<<endl;

    NORM_PI_PI(targetAngle);
    DAS_misc.angleError_old = DAS_misc.angleError;
    DAS_misc.oldsimtime = s->currentTime;
    return targetAngle / car->_steerLock;
}

// Compute target point for steering.
vec2f GetTargetPoint(tCarElt* car, int targetLane, DASmisc &DAS_misc, tSituation *ss)
{
    tTrackSeg *seg = car->_trkPos.seg;
    float lookahead;
    float length = GetDistToSegEnd(car);
    float currentSpeed = sqrt(car->_speed_X * car->_speed_X + car->_speed_Y * car->_speed_Y);
    float Tstr = LevelKDriver::steer_time;
    if (OnLane(car, targetLane, DAS_misc)){ 
        lookahead = LevelKDriver::carlength + Tstr * currentSpeed / 100.0;
        // cout<<"on lane "<< ss->currentTime <<endl;
    }
    else lookahead = LevelKDriver::carlength * 1.2 + Tstr * currentSpeed / 100.0 * 10.0;

    // Search for the segment containing the target point.
    while (length < lookahead) {
        seg = seg->next;
        length += seg->length;
    }

    length = lookahead - length + seg->length;

    vec2f s;
    s.x = (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x) / 2.0f;
    s.y = (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y) / 2.0f;

    /* set constant invariant y_target. */ 
    float width = seg->startWidth + (seg->endWidth - seg->startWidth) / seg->length * length;
    width = targetLane * width / 3;

    /* set smooth varying y_target. */ 
    float t0 = DAS_misc.t0; // time when lane change happen, set when lane change happens
    float y0 = DAS_misc.y0; // my car y, set when lane change happens
    float yt = width; // target lane y
    float deltaT = ss->currentTime-t0; // time since lane change happened
    deltaT = (deltaT>Tstr) ? Tstr : deltaT;
    // float smooth_width = y0+0.5*(yt-y0)*(1.0+sin(M_PI/Tstr*(deltaT-Tstr/2.0)));
    float smooth_width = y0+(yt-y0)*deltaT/Tstr;

    float y_target = smooth_width;

    if ( seg->type == TR_STR) {
        vec2f d, n;
        n.x = (seg->vertex[TR_EL].x - seg->vertex[TR_ER].x) / seg->length;
        n.y = (seg->vertex[TR_EL].y - seg->vertex[TR_ER].y) / seg->length;
        n.normalize();
        d.x = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x) / seg->length;
        d.y = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y) / seg->length;
        return s + d * length +  y_target * n;
    } else {
        vec2f c, n;
        c.x = seg->center.x;
        c.y = seg->center.y;
        float arc = length / seg->radius;
        float arcsign = (seg->type == TR_RGT) ? -1.0f : 1.0f;
        arc = arc * arcsign;
        s = s.rotate(c, arc);

        n = c - s;
        n.normalize();
        return s + y_target * arcsign * n;
    }
}

// Compute the length to the end of the segment.
float GetDistToSegEnd(tCarElt* car)
{
    if (car->_trkPos.seg->type == TR_STR) {
        return car->_trkPos.seg->length - car->_trkPos.toStart;
    } else {
        return (car->_trkPos.seg->arc - car->_trkPos.toStart) * car->_trkPos.seg->radius;
    }
}

// Compute the length to the start of the segment.
float GetDistToSegStart(tCarElt* car)
{
    if (car->_trkPos.seg->type == TR_STR) {
        return car->_trkPos.toStart;
    } else {
        return car->_trkPos.toStart * car->_trkPos.seg->radius;
    }
}

// check if close to targetlane with in 1/5 lanewith (1/15 track width)
bool Near(tCarElt* car, int targetlane) {
    tTrackSeg *seg = car->_trkPos.seg;
    float length = GetDistToSegStart(car);
    float width = seg->startWidth + (seg->endWidth - seg->startWidth) / seg->length * length;
    return ( ((targetlane * width / 3.0 - width / 15.0) < car->_trkPos.toMiddle) && (car->_trkPos.toMiddle < targetlane * width / 3.0 + width / 15.0) );
}

bool OnLane(tCarElt* car, int targetLane, DASmisc &DAS_misc){
    if (Near(car,targetLane)) ++DAS_misc.onLaneCount;
    else DAS_misc.onLaneCount = 0;

    if (DAS_misc.onLaneCount > 15) return true;
    else return false;
}

bool isStuck_(tCarElt* car, DASmisc &DAS_misc) {
    float angle=-getYaw(car);
    if (fabs(angle) > LevelKDriver::MAX_UNSTUCK_ANGLE && 
        car->_speed_x < LevelKDriver::MAX_UNSTUCK_SPEED &&
        fabs(car->_trkPos.toMiddle) > LevelKDriver::MIN_UNSTUCK_DIST) {
        int MAX_UNSTUCK_COUNT = int(LevelKDriver::UNSTUCK_TIME_LIMIT / RCM_MAX_DT_ROBOTS);
        if (DAS_misc.stuck > MAX_UNSTUCK_COUNT && car->_trkPos.toMiddle * angle < 0.0) {
            return true;
        } else {
            DAS_misc.stuck++;
            return false;
        }
    } else {
        DAS_misc.stuck = 0;
        return false;
    }
}

