/***************************************************************************

    file                 : LevelKDriver.cpp
    created              : 2018-01-13 02:59:26 UTC
    author               : Guankun Su
    email                : guanksu@umich.edu
    modified time        : 2018-05-04 00:17:15
    modified by          : guanksu
    summary              :

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/


#include "LevelKDriver.h"
#include <iomanip>

#include <stdlib.h>
#include <stdio.h>



// #include <xmlrpc-c/base.h>
// #include <xmlrpc-c/client.h>

// #include "config.h" 

using namespace std;

const float LevelKDriver::MAX_UNSTUCK_ANGLE = 15.0f / 180.0f * PI; // [radians] If the angle of the car on the track is smaller, we assume we are not stuck.
const float LevelKDriver::UNSTUCK_TIME_LIMIT = 2.0f;              // [s] We try to get unstuck after this time.
const float LevelKDriver::MAX_UNSTUCK_SPEED = 5.0f;               // [m/s] Below this speed we consider being stuck.
const float LevelKDriver::MIN_UNSTUCK_DIST = 3.0f;                // [m] If we are closer to the middle we assume to be not stuck.
const float LevelKDriver::SHIFT = 0.9f;
const float LevelKDriver::SHIFT_MARGIN = 4.0f;

// car type parameters: car1-trb1 for all level drivers
const float LevelKDriver::carlength = 4.7f; // [m] body length

// level-k const variable
const int   LevelKDriver::ndriverMax = 10;
const int   LevelKDriver::inLaneCountNum = 45;
const float LevelKDriver::steer_time = 1.0f;
const float LevelKDriver::steer_P = 0.7f;
const float LevelKDriver::steer_D = 0.01f;
const float LevelKDriver::lateral_speed = 4.0 / 2.0; // m/s
const float LevelKDriver::constraint_dist = carlength * 1.5f; //m
const float LevelKDriver::safe_dist = carlength; //m
const float LevelKDriver::out_safe_dist = safe_dist * 2.0f; //m
const float LevelKDriver::safe_decel = 2.0;// m/s
const float LevelKDriver::nominal_speed = 80.0 / 3.6; // m/s
const float LevelKDriver::speedPID_accel_P = 10.0f;
const float LevelKDriver::speedPID_accel_D = 0.01f;
const float LevelKDriver::speedPID_accel_I = 0.1f;
const float LevelKDriver::speedPID_brake_P = 10.0f;
const float LevelKDriver::speedPID_brake_D = 1.0f;
const float LevelKDriver::speedPID_brake_I = 0.1f;
const float LevelKDriver::speedPID_I_margin = 5.0 / 3.6;
const float LevelKDriver::min_speed = 80.0 / 3.6 - 2.5 * 2.0; // m/s
const float LevelKDriver::max_speed = 80.0 / 3.6 + 2.5 * 2.0; // m/s

// #define steerPlimit 10
// #define steerDlimit 10

// Static variables.
Cardata *LevelKDriver::cardata = NULL;
double   LevelKDriver::currentsimtime;
double   LevelKDriver::oldsimtime;
bool     LevelKDriver::allInit = false;
int      LevelKDriver::allInitCount = 0;
int      LevelKDriver::TotalCarInRace = 0;

pair<bool, double> LevelKDriver::initInfo(false, 0.0);

LevelKDriver::LevelKDriver(int index, int level): LEVEL(level) {
    INDEX = index;
    isInit = false;
    isLaneInit = false;
    speedPID_int = 0;
    angleError = 0;
    angleError_old = 0;
    inDanger = false;
    onLane = false;
    onLaneCount = 0;

    srand(time(0));
    if (LEVEL == 0) stragety = new L0_Policy();
    else if (LEVEL == 1) stragety = new L1_Policy();
    else if (LEVEL == 2) stragety = new L2_Policy();
}

LevelKDriver::~LevelKDriver() {
    delete opponents;
    if (cardata != NULL) {
        delete cardata;
        cardata = NULL;
    }
    delete stragety;
}

// Called for every track change or new race.
void LevelKDriver::initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s) {
    track = t;
    *carParmHandle = NULL;

    s->LKinfo.allLKInitCount=0;
    s->LKinfo.allLKInit=false;
    s->LKinfo.LKInitInfo.first=false;
    s->LKinfo.LKInitInfo.second=0;
    s->LKinfo.TotalLKCarInRace=0;
    s->LKinfo.afterLKinitCount=0;

    s->LKinfo.wantPic=false;
    s->LKinfo.wantRecord=false;
    s->LKinfo.askPic=false;
    s->LKinfo.askRecord=false;
    s->LKinfo.isMkdir=false;
    s->LKinfo.hostLevel=-1;
    s->LKinfo.hostIndex=-1;
}

// Start a new race.
void LevelKDriver::newRace(tCarElt* car, tSituation *s) {
    MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT / RCM_MAX_DT_ROBOTS);
    stuck = 0;
    this->car = car;

    wantRecord(s);
    wantPic(s);

    if(s->LKinfo.wantRecord){
        recordMisc_init(&record_Misc,"Experiment data/", "SimTime\tAction\tObservation\tdist_lon\tdist_lat\tvel_lon\tvel_lat\taccel_lon\taccel_lat\tyaw\tyaw_dot\tfl_d\tfl_v\tfc_d\tfc_v\tfr_d\tfr_v\trl_d\trl_v\trr_d\trr_v\tlane\tcollision\n", "0", 0, -1, 0, 0, 0, 0, 0);
        cout << "\n ============== \n Experiment starts now \n ============== \n" << endl;
        record_init(record_Misc, myfile, track, s, LEVEL, INDEX);       
    }

    // Create just one instance of cardata shared by all drivers.
    if (cardata == NULL) {
        cardata = new Cardata(s);
    }
    mycardata = cardata->findCar(car);
    oldsimtime = 0;
    currentsimtime = s->currentTime;

    // initialize the list of opponents.
    opponents = new Opponents(s, this, cardata);
    opponent = opponents->getOpponentPtr();

    ++s->LKinfo.TotalLKCarInRace;
}

void LevelKDriver::drive(tSituation *s) {
    update(car, s);
    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    //init
    // if (!s->LKinfo.allLKInit) {
    if(!s->LKinfo.allLKInit){
        if (!isLaneInit) {
            double distLon, distLat;
            getPos(distLon, distLat, car);
            lanePos = classify_lanepos(car);
            steer_t0 = s->currentTime;
            steer_y0 = distLat;
            isLaneInit = true;
        }

        SpeedPID(nominal_speed); // control accel and brake
        car->_steerCmd = getSteer(lanePos, s);
        car->_gearCmd = getGear();
        float speedError = (currentSpeed - nominal_speed) / nominal_speed;
        if (speedError > -0.01 && speedError < 0.01)
        {
            inLaneCount = inLaneCountNum; // prevent inlane counts
            isInit = true;
            referenceSpeed = nominal_speed;
            isChangedLane = 0;
            actionChangeCount = 0;
            actionType = Maintain;
            prevaction = actionType;
            prevreferenceSpeed = referenceSpeed;
        }      


        // if(allInit){
        //     prevActTime = currentsimtime;
        //     cout << "All Level" << LEVEL << " TotalCarInRace: " << TotalCarInRace << " init ready" << endl;
        // }

        if (isInit) {
            if (!(s->LKinfo.LKInitInfo.first)) {
                s->LKinfo.LKInitInfo.first = true;
                s->LKinfo.LKInitInfo.second = currentsimtime;
                s->LKinfo.allLKInitCount = 1;
            } else {
                if (s->LKinfo.LKInitInfo.second == currentsimtime) ++s->LKinfo.allLKInitCount;
                else {
                    s->LKinfo.LKInitInfo.second = currentsimtime;
                    s->LKinfo.allLKInitCount = 1;
                }
            }
            cout << "Level" << LEVEL << " " << INDEX << " init ready " << currentsimtime << " " << s->currentTime << " allInitCount " << s->LKinfo.allLKInitCount << endl;
        } else {
            s->LKinfo.LKInitInfo.first = false;
            s->LKinfo.allLKInitCount = 0;
        }

        if (s->LKinfo.allLKInitCount == s->LKinfo.TotalLKCarInRace) {
            s->LKinfo.allLKInit = true;
            // prevActTime = currentsimtime;
            // cout << "Level" << LEVEL << " " << INDEX << " init ready " << currentsimtime << " " << s->currentTime << endl;
            cout << ">> All Level" << LEVEL << " TotalLKCarInRace: " << s->LKinfo.TotalLKCarInRace << " init ready" << endl;
        }

        // initHandler(isInit, initInfo, allInitCount, currentsimtime, TotalCarInRace, allInit);
        // initHandler(allInit, s->LKinfo.LKInitInfo, s->LKinfo.allLKInitCount, currentsimtime, s->LKinfo.totalLK, s->LKinfo.allLKInit);

        // if(allInit){
        //     cout << ">> Level" << LEVEL << " init ready " << " " << s->currentTime << " allLKInitCount " << s->LKinfo.allLKInitCount << "<< s->LKinfo.totalLK" << s->LKinfo.totalLK << endl;
        // }
        
        // if(s->LKinfo.allLKInit){
        //     cout << ">> Total LK InRace: " << s->LKinfo.totalLK << " init ready at " << currentsimtime << endl;
        //     cout << "\n===================\n" << endl;
        // }
    }else if (isStuck()) {
        car->_steerCmd = -mycardata->getCarAngle() / car->_steerLock;
        car->_gearCmd = -1;     // Reverse gear.
        car->_accelCmd = 1.0f;  // 100% accelerator pedal.
        car->_brakeCmd = 0.0f;  // No brakes.
        car->_clutchCmd = 0.0f; // Full clutch (gearbox connected with engine).
    }
    //after initiation
    else {
        if (actionChangeCount == policyPeriod) {
            actionChangeCount = 0;

            //ObserveTtoLK
            ObserveTtoLK(s);

            // //LKController
            //     // choose one of the following strageties
            LKController(s);
            //     // actionTypeIterate();
            //     // Cruise();

            //     safetyController();

            //     // set command for actuator
            //     policyTransition();

            //     // record for next action
            //     prevaction = actionType;
            //     if (actionType == ChangeToLeft || actionType == ChangeToRight) prevreferenceSpeed = speedBeforeChange;
            //     else prevreferenceSpeed = referenceSpeed;

            // cout<<"Level" << LEVEL <<" "<<INDEX<<" lanePos "<<lanePos<<" "<<currentsimtime<<" "<<s->currentTime<<endl;
        } else {
            actionChangeCount ++;
        }

        //Action_LKtoT;
        Action_LKtoT(s);

        // steerTest();
        // observeTest();
    }
    if(s->LKinfo.wantRecord){
        record_main(car, s, track, myfile, record_Misc, &obsData, LEVEL, INDEX);
    }
            // cout << "Level" << LEVEL << " " << INDEX << " meet constraint 1: left car in parallel position. actionType: " << actionType << endl;
}

void LevelKDriver::steerTest(tSituation *ss) {
    SpeedPID(nominal_speed);
    car->_steerCmd = getSteer(lanePos, ss);

    if (INDEX == 0) {
        int temp = currentsimtime/3;
        temp = -(temp % 3 - 1) * 5;
        SpeedPID(nominal_speed + temp);
    }
    if (INDEX == 0) {
        lanePos = currentsimtime/3;
        lanePos = lanePos % 3 - 1;
        car->_steerCmd = getSteer(lanePos, ss);
    }

    car->_gearCmd = getGear();
}

void LevelKDriver::observeTest(tSituation *ss) {
    SpeedPID(nominal_speed);
    lanePos = INDEX % 3 - 1;
    car->_steerCmd = getSteer(lanePos, ss);

    if (INDEX == 0) {
        int temp = currentsimtime;
        temp = -(temp % 3 - 1) * 5;
        SpeedPID(nominal_speed + temp);
    }
    if (INDEX == 0) {
        lanePos = currentsimtime;
        lanePos = lanePos % 3 - 1;
        car->_steerCmd = getSteer(lanePos, ss);
    }

    car->_gearCmd = getGear();
}

int LevelKDriver::pitCommand(tSituation *s) {
    return ROB_PIT_IM; // return immediately.
}

void LevelKDriver::endRace(tSituation *s) {
}

bool LevelKDriver::isStuck() {
    if (fabs(angle) > MAX_UNSTUCK_ANGLE && 
        car->_speed_x < MAX_UNSTUCK_SPEED &&
        fabs(car->_trkPos.toMiddle) > MIN_UNSTUCK_DIST) {
        if (stuck > MAX_UNSTUCK_COUNT && car->_trkPos.toMiddle * angle < 0.0) {
            return true;
        } else {
            stuck++;
            return false;
        }
    } else {
        stuck = 0;
        return false;
    }
}

void LevelKDriver::update(tCarElt *car, tSituation *s) {
    // Update global car data (shared by all instances) just once per timestep.
    if (currentsimtime != s->currentTime) {
        oldsimtime = currentsimtime;
        currentsimtime = s->currentTime;
        cardata->update();
    }
    opponents->update(s, this);

    currentSpeed = sqrt(car->_speed_X * car->_speed_X + car->_speed_Y * car->_speed_Y);
    trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    angle = trackangle - car->_yaw;
    NORM_PI_PI(angle);

    // check if close to lanePos. this effect lookahead distance
    if (near_(lanePos)) ++onLaneCount;
    else onLaneCount = 0;
    if (onLaneCount > 15) onLane = true;
    else onLane = false;
}

// Compute gear.
int LevelKDriver::getGear()
{
    if (car->_gear <= 0) {
        return 1;
    }
    float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
    float omega = car->_enginerpmRedLine / gr_up;
    float wr = car->_wheelRadius(2);

    if (omega * wr * SHIFT < car->_speed_x) {
        return car->_gear + 1;
    } else {
        float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
        omega = car->_enginerpmRedLine / gr_down;
        if (car->_gear > 1 && omega * wr * SHIFT > car->_speed_x + SHIFT_MARGIN) {
            return car->_gear - 1;
        }
    }
    return car->_gear;
}

// Compute steer value.
float LevelKDriver::getSteer(int targetLane, tSituation *ss)
{
    float targetAngle;
    vec2f target = getTargetPoint(targetLane, ss);

    targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
    angleError = targetAngle - car->_yaw;

    //map to -M_PI to M_PI
    while(angleError<-M_PI) angleError+=2*M_PI;
    while(angleError>M_PI) angleError-=2*M_PI;    

    double dTime = currentsimtime - oldsimtime;
    dTime = (dTime <= 0) ? 0.02 : dTime;
    double Ppart = angleError * steer_P;
    double Dpart = (angleError - angleError_old) / dTime * steer_D;

    // if(Ppart>steerPlimit) Ppart=steerPlimit;
    // else if(Ppart<-steerPlimit) Ppart=-steerPlimit;
    // if(Dpart>steerDlimit) Ppart=steerPlimit;
    // else if(Dpart<-steerDlimit) Dpart=-steerDlimit;

    // cout<<" Level" << LEVEL << " " << INDEX << " " << angleError << " " <<Ppart<<" "<<Dpart<<" "<<dTime<<endl;

    targetAngle = Ppart + Dpart;

    NORM_PI_PI(targetAngle);
    angleError_old = angleError;
    return targetAngle / car->_steerLock;
}

// Compute target point for steering.
vec2f LevelKDriver::getTargetPoint(int targetLane, tSituation *ss)
{
    tTrackSeg *seg = car->_trkPos.seg;
    float lookahead;
    float length = getDistToSegEnd();
    float Tstr = steer_time;

    if (onLane) lookahead = carlength + steer_time * currentSpeed / 100.0;
    else lookahead = carlength * 1.2 + steer_time * currentSpeed / 100.0 * 10.0;
    // if(INDEX==2) cout<<"lk 2 "<<lookahead<<" ";

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
    float t0 = steer_t0; // time when lane change happen, set when lane change happens
    float y0 = steer_y0; // my car y, set when lane change happens
    float yt = width; // target lane y
    float deltaT = ss->currentTime-t0; // time since lane change happened
    deltaT = (deltaT>Tstr) ? Tstr : deltaT;
    // float smooth_width = y0+0.5*(yt-y0)*(1.0+sin(M_PI/Tstr*(deltaT-Tstr/2.0)));
    float smooth_width = y0+(yt-y0)*deltaT/Tstr;

    // cout<<" Level" << LEVEL << " " << INDEX << " y0 " << y0 << " yt " << yt << " deltaT " << deltaT << " smooth_width " << smooth_width << endl;

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

// check if close to targetlane with in 1/5 lanewith (1/15 track width)
bool LevelKDriver::near_(int targetlane) {
    tTrackSeg *seg = car->_trkPos.seg;
    float length = getDistToSegStart();
    float width = seg->startWidth + (seg->endWidth - seg->startWidth) / seg->length * length;
    return ( ((targetlane * width / 3.0 - width / 15.0) < car->_trkPos.toMiddle) && (car->_trkPos.toMiddle < targetlane * width / 3.0 + width / 15.0) );
}

// Compute the length to the end of the segment.
float LevelKDriver::getDistToSegEnd()
{
    if (car->_trkPos.seg->type == TR_STR) {
        return car->_trkPos.seg->length - car->_trkPos.toStart;
    } else {
        return (car->_trkPos.seg->arc - car->_trkPos.toStart) * car->_trkPos.seg->radius;
    }
}

// Compute the length to the start of the segment.
float LevelKDriver::getDistToSegStart()
{
    if (car->_trkPos.seg->type == TR_STR) {
        return car->_trkPos.toStart;
    } else {
        return car->_trkPos.toStart * car->_trkPos.seg->radius;
    }
}

// A PID controller to reach target speed
void LevelKDriver::SpeedPID(float setSpeed)
{
    float realAccel = sqrt(car->pub.DynGCg.acc.x * car->pub.DynGCg.acc.x + car->pub.DynGCg.acc.y * car->pub.DynGCg.acc.y);
    if ((car->pub.DynGCg.acc.x * car->_speed_x + car->pub.DynGCg.acc.y * car->_speed_y) < 0) realAccel = -realAccel;

    // P error
    float speedDelta = (setSpeed - currentSpeed) / setSpeed; // normalized by setSpeed
    // D error
    float accel0 = (speedDelta - speedDeltaOld)/0.002;

    // float accel0; // normalized by speed error over 0.002s
    // if (abs(speedDelta) < 0.1) accel0 = realAccel / (0.1 / 0.002); // prevent being divided by a extremem small value
    // else accel0 = realAccel / (speedDelta / 0.002);

    // I error
    if (abs(currentSpeed - setSpeed) < speedPID_I_margin) speedPID_int += speedDelta;
    // total PID value
    float PIDvalue = speedPID_accel_P * speedDelta + speedPID_accel_D * accel0 + speedPID_accel_I * speedPID_int;

    speedDeltaOld = speedDelta;

    // actuator
    if (PIDvalue > 0)    {
        car->_brakeCmd = 0;
        car->_accelCmd = PIDvalue;
    } else {
        car->_brakeCmd = -PIDvalue;
        car->_accelCmd = 0;
    }
}

//update observation message
void LevelKDriver::ObserveTtoLK(tSituation *s)
{
    ::ObserveTtoLK(msg, car, s, track, &obsData);

    // if (opponents->getNOpponents() > 0) {
    //     cout << fixed << setw(3) << setprecision(1) << " Level" << LEVEL << " " << INDEX << " sees " << "lane: " << msg[10] << " fc_d: " << obsData.fc_d << " fc_v: " << obsData.fc_v << " fl_d: " << obsData.fl_d << " fl_v: " << obsData.fl_v << " fr_d: " << obsData.fr_d << " fr_v: " << obsData.fr_v << " rl_d: " << obsData.rl_d << " rl_v: " << obsData.rl_v << " rr_d: " << obsData.rr_d << " rr_v: " << obsData.rr_v << endl;
    //     cout << "Level" << LEVEL << " " << INDEX << " sees " << "lane: " << msg[10] << " fc_d: " << msg[2] << " fc_v: " << msg[3] << " fl_d: " << msg[0] << " fl_v: " << msg[1] << " fr_d: " << msg[4] << " fr_v: " << msg[5] << " rl_d: " << msg[6] << " rl_v: " << msg[7] << " rr_d: " << msg[8] << " rr_v: " << msg[9] << endl;
    // }
}

int LevelKDriver::classify_dist(float distance)
{
    return ::classify_dist(distance);
}

int LevelKDriver::classify_vel(float d, float v)
{
    return ::classify_vel(d,v);
}

// determine actionType according to observation message
// default level 0
void LevelKDriver::LKController(tSituation *ss)
{

    int index = 0; // observation index
    for (int i = 0; i < 11; ++i) index = index + pow(3.0, 10.0 - i) * msg[i];

    if (LEVEL == 0) {
        actionType = stragety->getL0Pol(index);

        longitudinal_constraints();
    } else {
        randnum = rand() / double(RAND_MAX);
        float total = 0;
        stragety->getLKPol(index, &pol);
        // if (LEVEL == 1) {
        //     cout << "Level" << LEVEL << " " << INDEX << " sees: " << index << " pol " << pol[0] << " " << pol[1] << " " << pol[2] << " " << pol[3] << " " << pol[4] << " " << pol[5] << " " << pol[6] << " " << pol << endl;
        // }
        for (int i = 0; i < 7; i++)
        {
            total += pol[i];
            if (randnum <= total)
            {
                actionType = i + 1;
                break;
            }
        }

        lateral_constraints();
        longitudinal_constraints();
    }

    policyTransition(ss);

    cout << "L" << LEVEL << " " << INDEX << " takes actionType: " << actionType << " at " << currentsimtime;            
    cout << " obs: " << INDEX << " sees " << "lane: " << msg[10] << " fc_d: " << msg[2] << " fc_v: " << msg[3] << " fl_d: " << msg[0] << " fl_v: " << msg[1] << " fr_d: " << msg[4] << " fr_v: " << msg[5] << " rl_d: " << msg[6] << " rl_v: " << msg[7] << " rr_d: " << msg[8] << " rr_v: " << msg[9] << endl;

    // cout << "Level" << LEVEL << " " << INDEX << " takes actionType: " << actionType << " at " << currentsimtime << endl;
}

//carry out actions
void LevelKDriver::Action_LKtoT(tSituation *ss)
{
    // if(referenceSpeed<min_speed) referenceSpeed = min_speed;
    car->_steerCmd = getSteer(lanePos, ss);
    SpeedPID(referenceSpeed);
    car->_gearCmd = getGear();
}

void LevelKDriver::policyTransition(tSituation *ss) {
    float laneChangeHappen=false;

    if (actionType == 1) {

    } else if (actionType == 2) {
        referenceSpeed += accel_rate;
    }
    else if (actionType == 3) {
        referenceSpeed += decel_rate;
    }
    else if (actionType == 4) {
        referenceSpeed += hard_accel_rate;
    }
    else if (actionType == 5) {
        referenceSpeed += hard_decel_rate;
    }
    else if (actionType == 6) {
        ++lanePos;
        laneChangeHappen=true;
    }
    else if (actionType == 7) {
        --lanePos;
        laneChangeHappen=true;
    }

    if (lanePos < -1){
        lanePos = -1;
        laneChangeHappen=false;
    }
    else if (lanePos > 1){
        lanePos = 1;
        laneChangeHappen=false;
    }

    if(laneChangeHappen){
        double distLon, distLat;
        getPos(distLon, distLat, car);
        steer_t0 = ss->currentTime;
        steer_y0 = distLat;
        laneChangeHappen=false;
        // cout<<"change lane "<< s->currentTime <<endl;
    }

    if (!inDanger) {
        if (referenceSpeed < min_speed){
            referenceSpeed = min_speed;
        } 
        else if (referenceSpeed > max_speed) referenceSpeed = max_speed;
    } else {
        if (referenceSpeed <= 0) referenceSpeed = 2.5;
        cout << "Level" << LEVEL << " " << INDEX << "take safety action: " << actionType << " referenceSpeed < min_speed : "<< referenceSpeed <<" "<< min_speed << endl;
    }
}

// Level K controller constraints
void LevelKDriver::lateral_constraints()
{
    float l_dist = (abs(obsData.fl_d) > abs(obsData.rl_d)) ? abs(obsData.rl_d) : obsData.fl_d;
    float r_dist = (abs(obsData.fr_d) > abs(obsData.rr_d)) ? abs(obsData.rr_d) : obsData.fr_d;

    if (-constraint_dist < l_dist && l_dist < constraint_dist && actionType == ChangeToLeft)    {
        policyWithout(ChangeToLeft);
        cout << "Level" << LEVEL << " " << INDEX << " meet constraint 1: left car in parallel position. take action: " << actionType << endl;
    }
    if (-constraint_dist < r_dist && r_dist < constraint_dist && actionType == ChangeToRight)    {
        policyWithout(ChangeToRight);
        cout << "Level" << LEVEL << " " << INDEX << " meet constraint 2: right car in parallel position, take action " << actionType << endl;
    }


    if ( ((msg[0] == 0 && msg[1] == 1) || (msg[6] == 0 && msg[7] == 1)) && actionType == ChangeToLeft)    {
        policyWithout(ChangeToLeft);
        cout << "Level" << LEVEL << " " << INDEX << " meet constraint 3: left car close and approaching, take action " << actionType << endl;
        // if(LEVEL==2 && INDEX ==0) cout << "Level" << LEVEL << " " << INDEX << " cannot turn left" << endl;
    }
    if ( ((msg[4] == 0 && msg[5] == 1) || (msg[8] == 0 && msg[9] == 1)) && actionType == ChangeToRight)    {
        policyWithout(ChangeToRight);
        cout << "Level" << LEVEL << " " << INDEX << " meet constraint 4: left car close and approaching, take action " << actionType << endl;
        // if(LEVEL==2 && INDEX ==0) cout << "Level" << LEVEL << " " << INDEX << " cannot turn right" << endl;
    }

    if (lanePos==-1 && actionType == ChangeToRight)    {
        policyWithout(ChangeToRight);
        cout << "Level" << LEVEL << " " << INDEX << " cannot turn right, take action " << actionType << endl;
        // if(LEVEL==2 && INDEX ==0){
        // cout << "Level" << LEVEL << " " << INDEX << " cannot turn right" << endl;
        //     char temp;
            // cout << "L" << LEVEL << " " << INDEX << " cannot turn right, take " << actionType << " at " << currentsimtime;            
        //     cout << " obs: " << INDEX << " sees " << "lane: " << msg[10] << " fc_d: " << msg[2] << " fc_v: " << msg[3] << " fl_d: " << msg[0] << " fl_v: " << msg[1] << " fr_d: " << msg[4] << " fr_v: " << msg[5] << " rl_d: " << msg[6] << " rl_v: " << msg[7] << " rr_d: " << msg[8] << " rr_v: " << msg[9] << endl;
        //     cin >> temp;
        // }    
    }
    if (lanePos==1 && actionType == ChangeToLeft)    {
        policyWithout(ChangeToLeft);
        cout << "Level" << LEVEL << " " << INDEX << " cannot turn left, take action " << actionType << endl;
        // if(LEVEL==2 && INDEX ==0){
        // cout << "Level" << LEVEL << " " << INDEX << " cannot turn left" << endl;
        //     char temp;
            // cout << "L" << LEVEL << " " << INDEX << " cannot turn left, take  " << actionType << " at " << currentsimtime;            
        //     cout << " obs: " << INDEX << " sees " << "lane: " << msg[10] << " fc_d: " << msg[2] << " fc_v: " << msg[3] << " fl_d: " << msg[0] << " fl_v: " << msg[1] << " fr_d: " << msg[4] << " fr_v: " << msg[5] << " rl_d: " << msg[6] << " rl_v: " << msg[7] << " rr_d: " << msg[8] << " rr_v: " << msg[9] << endl;
        //     cin >> temp;
        // }    
    }

}

void LevelKDriver::longitudinal_constraints() {
    inDanger = false;
    dangerousDegree = 0;

    inSafetyZone(obsData.fc_d, obsData.fc_v);

    if (inDanger)
    {
        if (dangerousDegree == 1)  actionType = Decelerate;
        else if (dangerousDegree == 2)  actionType = HardDecelerate;
        // if(LEVEL==2 && INDEX ==0){
        //     cout << "Level" << LEVEL << " " << INDEX << " meet long constraints: " << dangerousDegree << endl;
        // }
        cout << "Level" << LEVEL << " " << INDEX << " meet long constraints " << dangerousDegree << " fcd: " << obsData.fc_d << " fcv: " << obsData.fc_v << ", take action " << actionType << endl;
    }
}

void LevelKDriver::policyWithout(const int noActionType) {
    float randcopy = randnum * (1.0 - pol[noActionType - 1]);
    float total = 0.0;
    for (int i = 0; i < 7; i++)
    {
        if (i != (noActionType - 1)) total += pol[i];
        if (randcopy <= total)
        {
            actionType = i + 1;
            break;
        }
    }
    if (noActionType == actionType){
        actionType=1; // take maintain as default
    }
    // cout << "policyWithout " << noActionType << "; pol:" << pol[0] << ", " << pol[1] << ", " << pol[2] << ", " << pol[3] << ", " << pol[4] << ", " << pol[5] << ", " << pol[6] << "; randcopy: " << randcopy << endl;
    // cout << "take action " << actionType << endl;
    // if (noActionType == actionType) cout << "policy normalization failed. total: " << total << endl;
}

void LevelKDriver::inSafetyZone(float dist, float vel) {
    if (safeMod_RelativeVel(dist, vel, 2.0) || safeMod_AbsoluteDist(dist, vel, 1.0) || safeMod_NearCar(dist, 2.4)) {
        inDanger = true;
        dangerousDegree = 1;
    }
    if (safeMod_RelativeVel(dist, vel, 1.0) || safeMod_AbsoluteDist(dist, vel, 0.5) || safeMod_NearCar(dist, 1.2)) {
        inDanger = true;
        dangerousDegree = 2;
    }
    // cout << "Level" << LEVEL << " " << INDEX << "dangerous degree: " << dangerousDegree << endl;
}

bool LevelKDriver::safeMod_RelativeVel(float dist, float vel, float thres) {
    return dist > 0 && vel < 0 && (-dist / vel < thres);
    // return false;
}

// consider the absolute distance to collide
// Require: 50% < percent < 100%
// percent 100%: worst case, my car decel at -5,while front car also decel at -5.
// percent 50% : my car decel at -5,while front car maintain speed.
bool LevelKDriver::safeMod_AbsoluteDist(float dist, float vel, float percent) {
    // return dist < (vel*vel/abs(hard_decel_rate)*percent) && dist > 0 && vel < 0;
    return false;
}

bool LevelKDriver::safeMod_NearCar(float dist, float multiplier) {
    return dist < (carlength * multiplier) && dist > 0;
    // return false;
}

float LevelKDriver::min_dist_long(float dist) const{
    float temp = track->length/2;
    while(abs(dist)>temp){
        if(dist>0) dist-=track->length;
        else dist+=track->length;
    }
    return dist;
}
