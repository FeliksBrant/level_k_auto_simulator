/***************************************************************************

    file                 : LevelKDriver.h
    created              : Wed Jan 13 02:40:19 UTC 2018
    author               : Guankun Su
    email                : guanksu@umich.edu
    version              : $Id: LevelKDriver.h,v 1.0 2018/01/13 02:40:19 guanksu $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _LEVELKDRIVER_H_
#define _LEVELKDRIVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <math.h>
#include <iostream>
#include <string>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <cmath>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include "opponent.h"
#include "cardata.h"
#include "policy.h"
// #include "parameters.h"
#include "record.h"

#define TO_LEFT_LANE 1
#define TO_MIDDLE_LANE 0
#define TO_RIGHT_LANE -1
#define Maintain 1
#define Accelerate 2
#define Decelerate 3
#define HardAccelerate 4
#define HardDecelerate 5
#define ChangeToLeft 6
#define ChangeToRight 7

#define policyPeriod 50 //1s to change action

#define max_sight_distance 63//400;//200; //m -- max distance for detection of other vehicles in front and behind you
#define far_distance 42 //m
#define close_distance 21 //m

#define car_close 0 
#define car_nominal 1 
#define car_far 2 
#define car_stable 0 
#define car_approaching 1 
#define car_retreating 2 

#define accel_rate 2.5       // m/s^2
#define decel_rate -2.5      // m/s^2
#define hard_accel_rate 5.0  // m/s^2
#define hard_decel_rate -5.0 // m/s^2

#define speed_margin (2.0/3.6f) // m/s

class Opponents;
class Opponent;

class LevelKDriver{
	public:
        LevelKDriver(int index, int level);
        ~LevelKDriver();

        // Callback functions called from TORCS.
        void initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s);
        void newRace(tCarElt* car, tSituation *s);
        void drive(tSituation *s);
        int pitCommand(tSituation *s);
        void endRace(tSituation *s);

        tCarElt *getCarPtr() { return car; }
        tTrack *getTrackPtr() { return track; }
        float getSpeed() { return mycardata->getSpeedInTrackDirection(); /*speed;*/ }

    protected:
        // Utility functions.
        bool  isStuck();
        void  update(tCarElt *car, tSituation *s);
        int   getGear();
        float getSteer(int targetLane, tSituation *ss);
        vec2f getTargetPoint(int targetLane, tSituation *ss);
        float getDistToSegEnd();
        float getDistToSegStart();

        //Level K controller utility
        // int getmyID();
        void  LKController(tSituation *ss);
        void  SpeedPID(float setSpeed);
        void  Action_LKtoT(tSituation *ss);
        // void actionTypeIterate();
        // void Cruise();
        void  ObserveTtoLK(tSituation *s);
        int   classify_dist(float distance);
        int   classify_vel(float d, float v);
        void  lateral_constraints();
        void  longitudinal_constraints();
        void  policyWithout(const int noActionType); // normalize policy without actiontype 
        void  inSafetyZone(float dist, float vel);
        void  policyTransition(tSituation *ss);
        bool  near_(int targetlane);
        bool  safeMod_RelativeVel(float dist, float vel, float thres);
        bool  safeMod_AbsoluteDist(float dist, float vel, float percent);
        bool  safeMod_NearCar(float dist, float multiplier);
        float min_dist_long(float dist) const;

        // tests
        void  steerTest(tSituation *ss);
        void  observeTest(tSituation *ss);

        // Level-k Controller
        int INDEX;
        int msg[11];
        tCarElt *car;           // Pointer to tCarElt struct.
        bool isInit;
        bool isLaneInit;
        int isChangedLane;
        float currentSpeed;
        float speedPID_int;
        int actionChangeCount;
        int inLaneCount;
        int actionType;
        float referenceSpeed;
        float prevreferenceSpeed;
        float speedBeforeChange;
        float speedDeltaOld;
        float angleError;
        float angleError_old;
        float distanceOld;
        int prevaction;
        float *pol;
        double randnum;
        bool inDanger;
        int dangerousDegree;
        int lanePos;
        double prevActTime;
        bool onLane;
        int onLaneCount;
        Policy* stragety;
        observeData obsData;
        ofstream myfile;
        recordMisc record_Misc;
        float steer_t0;
        float steer_y0;

        static int TotalCarInRace;
        static bool allInit;
        static pair<bool,double> initInfo; // init info necessary to ensure all car to be inited at the same time
        static int allInitCount;

        // Per robot global data.
        int stuck;
        float trackangle;
        float angle;

        Opponents *opponents;   // The container for opponents.
        Opponent *opponent;     // The array of opponents.

        static Cardata *cardata;        // Data about all cars shared by all instances.
        SingleCardata *mycardata;       // Pointer to "global" data about my car.
        static double currentsimtime;   // Store time to avoid useless updates.
        static double oldsimtime;
        // Track variables.
        tTrack* track;

        // Data that should stay constant after first initialization.
        int   LEVEL;
        int MAX_UNSTUCK_COUNT;

    public:
        // Class constants.
        static const float MAX_UNSTUCK_ANGLE;
        static const float UNSTUCK_TIME_LIMIT;
        static const float MAX_UNSTUCK_SPEED;               // [m/s] Below this speed we consider being stuck.
        static const float MIN_UNSTUCK_DIST;                // [m] If we are closer to the middle we assume to be not stuck.
        static const float SHIFT;
        static const float SHIFT_MARGIN;

        static const int   ndriverMax;
        static const int   inLaneCountNum; 
        static const float steer_time;
        static const float steer_P;
        static const float steer_D;
        static const float lateral_speed; // set later speed for lane change m/s
        static const float constraint_dist;
        static const float safe_dist;
        static const float out_safe_dist;
        static const float safe_decel;
        static const float nominal_speed; // m/s
        static const float speedPID_accel_P;
        static const float speedPID_accel_D;
        static const float speedPID_accel_I;
        static const float speedPID_brake_P;
        static const float speedPID_brake_D;
        static const float speedPID_brake_I;
        static const float speedPID_I_margin;
        static const float min_speed; // m/s
        static const float max_speed; // m/s
        static const float carlength; //m
};

#endif // _LEVELKDRIVER_H_

