/***************************************************************************
*
*    file        : record.h
*    author      : Guankun Su
*    summary     : record data utilities and main function
*
****************************************************************************/

#ifndef _RECORD_H_
#define _RECORD_H_

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

using namespace std;

// misc. variables for record
struct recordMisc {
    // init variables
    string path;
    string dataFormat;
    string filename;

    // player record global variables
    int actChangeCount;
    int dataIndex;
    int actRecord;
    double simTime;
    float prevSpeed;
    int prevlane;
    double oldSimTime;
};

// REQUIRES:
// MODIFIES: misc
// EFFECT: Start recording at startRecord and prompt user that we have started recording
//         record oen row of data into myfile each second
//         quit recording after we have recorded maxDataAmount rows of data
void record_main(tCarElt* car, tSituation *s, tTrack *curTrack, ofstream &myfile, recordMisc &misc);

// REQUIRES:
// MODIFIES: misc
// EFFECT: prompt user to type in their recording number
//         open the file to record data
//         prompt some description about the recording data
void record_init(recordMisc &misc, ofstream &myfile, tTrack *curTrack);

// REQUIRES: msg[] should have exact 11 elements
// MODIFIES:
// EFFECT: update observation message
void ObserveTtoLK(int* msg, tCarElt* car, tSituation *s, tTrack *curTrack, recordMisc &misc);

// REQUIRES:
// MODIFIES:
// EFFECT: classify the distance to be car_close, car_nominal or car_far
int classify_dist(float distance);

// REQUIRES:
// MODIFIES:
// EFFECT: classify the relative velocity to be car_stable, car_approaching or car_retreating
int classify_vel(float d, float v);

// REQUIRES:
// MODIFIES: car->lanePos
// EFFECT: classify the lane position of the car to be left lane (1), middle lane (0) or right lane (-1)
void classify_lanepos(tCarElt* car);

// REQUIRES:
// MODIFIES:
// EFFECT: classify car action to be 1~7 (according to action space)
//         and return the action
int getAction(tCarElt* car, recordMisc &misc);

// REQUIRES:
// MODIFIES: distLon, distLat
// EFFECT: get the current distLon and distLat from startline
void getPos(double &distLon, double &distLat, tCarElt* car);

// REQUIRES:
// MODIFIES: velLong, velLat
// EFFECT: get the longitudinal and lateral velocity of car along track
void getVel(tCarElt* car, double &velLong, double &velLat);

// REQUIRES:
// MODIFIES: accelLong, accelLat
// EFFECT: get the longitudinal and lateral acceleartion of car
void getAccel(tCarElt* car, double  &accelLong, double &accelLat);

// REQUIRES:
// MODIFIES:
// EFFECT: get the Yaw angle of car
double getYaw(tCarElt* car);

// REQUIRES:
// MODIFIES:
// EFFECT: get the Yaw rate of car
double getYawRate(tCarElt* car);

#endif /* _RECORD_H_ */

