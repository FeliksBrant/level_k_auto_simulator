/***************************************************************************

file                 : driver.cpp
created              : Thu Dec 20 01:21:49 CET 2002
copyright            : (C) 2002-2004 Bernhard Wymann
email                : berniw@bluewin.ch
version              : $Id: driver.cpp,v 1.16.2.2 2008/12/31 03:53:53 berniw Exp $

***************************************************************************/

/***************************************************************************
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
***************************************************************************/

#include "driver.h"
#include <iostream>
#include <string>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <math.h>
using namespace std;

#include "..\..\windows\windowsspec.h"

const float Driver::MAX_UNSTUCK_ANGLE = 15.0f/180.0f*PI;	// [radians] If the angle of the car on the track is smaller, we assume we are not stuck.
const float Driver::UNSTUCK_TIME_LIMIT = 2.0f;				// [s] We try to get unstuck after this time.
const float Driver::MAX_UNSTUCK_SPEED = 5.0f;				// [m/s] Below this speed we consider being stuck.
const float Driver::MIN_UNSTUCK_DIST = 3.0f;				// [m] If we are closer to the middle we assume to be not stuck.
const float Driver::G = 9.81f;								// [m/(s*s)] Welcome on Earth.
const float Driver::FULL_ACCEL_MARGIN = 1.0f;				// [m/s] Margin reduce oscillation of brake/acceleration.
const float Driver::SHIFT = 0.9f;							// [-] (% of rpmredline) When do we like to shift gears.
const float Driver::SHIFT_MARGIN = 4.0f;					// [m/s] Avoid oscillating gear changes.
const float Driver::ABS_SLIP = 2.0f;						// [m/s] range [0..10]
const float Driver::ABS_RANGE = 5.0f;						// [m/s] range [0..10]
const float Driver::ABS_MINSPEED = 3.0f;					// [m/s] Below this speed the ABS is disabled (numeric, division by small numbers).
const float Driver::TCL_SLIP = 2.0f;						// [m/s] range [0..10]
const float Driver::TCL_RANGE = 10.0f;						// [m/s] range [0..10]
const float Driver::LOOKAHEAD_CONST = 17.0f;				// [m]
const float Driver::LOOKAHEAD_FACTOR = 0.33f;				// [-]
const float Driver::WIDTHDIV = 3.0f;						// [-] Defines the percentage of the track to use (2/WIDTHDIV).
const float Driver::SIDECOLL_MARGIN = 3.0f;					// [m] Distance between car centers to avoid side collisions.
const float Driver::BORDER_OVERTAKE_MARGIN = 0.5f;			// [m]
const float Driver::OVERTAKE_OFFSET_SPEED = 5.0f;			// [m/s] Offset change speed.
const float Driver::PIT_LOOKAHEAD = 6.0f;					// [m] Lookahead to stop in the pit.
const float Driver::PIT_BRAKE_AHEAD = 200.0f;				// [m] Workaround for "broken" pitentries.
const float Driver::PIT_MU = 0.4f;							// [-] Friction of pit concrete.
const float Driver::MAX_SPEED = 84.0f;						// [m/s] Speed to compute the percentage of brake to apply.
const float Driver::MAX_FUEL_PER_METER = 0.0008f;			// [liter/m] fuel consumtion.
const float Driver::CLUTCH_SPEED = 5.0f;					// [m/s]
const float Driver::CENTERDIV = 0.1f;						// [-] (factor) [0.01..0.6].
const float Driver::DISTCUTOFF = 200.0f;					// [m] How far to look, terminate while loops.
const float Driver::MAX_INC_FACTOR = 5.0f;					// [m] Increment faster if speed is slow [1.0..10.0].
const float Driver::CATCH_FACTOR = 10.0f;					// [-] select MIN(catchdist, dist*CATCH_FACTOR) to overtake.
const float Driver::CLUTCH_FULL_MAX_TIME = 2.0f;			// [s] Time to apply full clutch.
const float Driver::USE_LEARNED_OFFSET_RANGE = 0.2f;		// [m] if offset < this use the learned stuff

const float Driver::TEAM_REAR_DIST = 50.0f;					//
const int Driver::TEAM_DAMAGE_CHANGE_LEAD = 700;			// When to change position in the team?

// Static variables.
Cardata *Driver::cardata = NULL;
double Driver::currentsimtime;

// level-k const variable
const int Driver::ndriverMax = 10; 
const int Driver::inLaneCountNum = 45;
const float Driver::steer_P = 1.0f;
const float Driver::steer_D = 1.5f;
const double Driver::lateral_speed = 4.0/2.0; // m/s
const float Driver::constraint_dist = carlength*1.5f; //m
const float Driver::safe_dist = carlength; //m
const float Driver::out_safe_dist = safe_dist*2.0f; //m
const float Driver::safe_decel = 2.0;// m/s

Driver::Driver(int index)
{
	INDEX = index;
    static int dummy_counter=0;
    dummy_counter++; 
    counter = dummy_counter;

	//creat for level-k 
	isInit = 0;
	speedPID_int = 0;
	angleError = 0;
	angleError_old = 0;
	inSafety = false;
	srand(time(0));
}


Driver::~Driver()
{
	delete opponents;
	delete pit;
	delete [] radius;
	delete learn;
	delete strategy;
	if (cardata != NULL) {
		delete cardata;
		cardata = NULL;
	}
}


// Called for every track change or new race.
void Driver::initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s)
{
	track = t;

	const int BUFSIZE = 256;
	char buffer[BUFSIZE];
	// Load a custom setup if one is available.
	// Get a pointer to the first char of the track filename.
	char* trackname = strrchr(track->filename, '/') + 1;

	switch (s->_raceType) {
		case RM_TYPE_PRACTICE:
			snprintf(buffer, BUFSIZE, "drivers/bt/%d/practice/%s", INDEX, trackname);
			break;
		case RM_TYPE_QUALIF:
			snprintf(buffer, BUFSIZE, "drivers/bt/%d/qualifying/%s", INDEX, trackname);
			break;
		case RM_TYPE_RACE:
			snprintf(buffer, BUFSIZE, "drivers/bt/%d/race/%s", INDEX, trackname);
			break;
		default:
			break;
	}

	*carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
	if (*carParmHandle == NULL) {
		snprintf(buffer, BUFSIZE, "drivers/bt/%d/default.xml", INDEX);
		*carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
	}

	// Create a pit stop strategy object.
	strategy = new SimpleStrategy2();

	// Init fuel.
	strategy->setFuelAtRaceStart(t, carParmHandle, s, INDEX);

	// Load and set parameters.
	MU_FACTOR = GfParmGetNum(*carParmHandle, BT_SECT_PRIV, BT_ATT_MUFACTOR, (char*)NULL, 0.69f);
}


// Start a new race.
void Driver::newRace(tCarElt* car, tSituation *s)
{
	float deltaTime = (float) RCM_MAX_DT_ROBOTS;
	MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT/deltaTime);
	OVERTAKE_OFFSET_INC = OVERTAKE_OFFSET_SPEED*deltaTime;
	stuck = 0;
	alone = 1;
	clutchtime = 0.0f;
	oldlookahead = 0.0f;
	this->car = car;
	CARMASS = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0f);
	myoffset = 0.0f;
	initCa();
	initCw();
	initTireMu();
	initTCLfilter();

	// Create just one instance of cardata shared by all drivers.
	if (cardata == NULL) {
		cardata = new Cardata(s);
	}
	mycardata = cardata->findCar(car);
	currentsimtime = s->currentTime;

	// initialize the list of opponents.
	opponents = new Opponents(s, this, cardata);
	opponent = opponents->getOpponentPtr();

	// Set team mate.
	const char *teammate = GfParmGetStr(car->_carHandle, BT_SECT_PRIV, BT_ATT_TEAMMATE, NULL);
	if (teammate != NULL) {
		opponents->setTeamMate(teammate);
	}

	// Initialize radius of segments.
	radius = new float[track->nseg];
	computeRadius(radius);

	learn = new SegLearn(track, s, INDEX);

	// create the pit object.
	pit = new Pit(s, this);
}


// Drive during race.
void Driver::drive(tSituation *s)
{
	memset(&car->ctrl, 0, sizeof(tCarCtrl));

	update(s);

	currentSpeed = sqrt(car->_speed_X*car->_speed_X+car->_speed_Y*car->_speed_Y);

	//init
	if(isInit==0)
	{
		car->lanePos = - (INDEX % 3) + 1; //assign later
		car->_steerCmd = getSteer(car->lanePos);
		car->_gearCmd = getGear();
		SpeedPID(nominal_speed);
		float speedError = (currentSpeed-nominal_speed)/nominal_speed;
		if(speedError>-0.01&&speedError<0.01)
		{
			inLaneCount = inLaneCountNum; // prevent inlane counts
			isInit = 1;
			referenceSpeed = nominal_speed;
			isChangedLane = 0;
			actionChangeCount = 0;
			actionType = Maintain;
			prevaction = actionType;
			prevreferenceSpeed = referenceSpeed;
		}
	}
	//after initiation
	else
	{
		if (actionChangeCount>actionChangeCountMax)
		{
			actionChangeCount = 0;

		//ObserveTtoLK
			ObserveTtoLK();
			
		//LKController
			// choose one of the following strageties
			LKController();
			// actionTypeIterate();
			// Cruise();

			safetyController();

			// set command for actuator
			policyTransition();
			
			// record for next action
			prevaction = actionType;
			if (actionType == ChangeToLeft || actionType == ChangeToRight) prevreferenceSpeed = speedBeforeChange;
			else prevreferenceSpeed = referenceSpeed;
		}
		else
		{
			actionChangeCount ++;
		}

	//Action_LKtoT;
		Action_LKtoT();
	}
}

/***************************************************************************
*
* Level-k driver utility functions (created by Guankun Su)
*
***************************************************************************/
void Driver::policyTransition(){
	if(actionType==1){
		referenceSpeed = prevreferenceSpeed;
		// speedPID_int = 0;
	}else if(actionType==2){
		referenceSpeed += new_accel_rate*actionChangeCountMax/50.0f;
		// speedPID_int = 0;
	}
	else if(actionType==3){
		referenceSpeed += new_decel_rate*actionChangeCountMax/50.0f;
		// speedPID_int = 0;
	}
	else if(actionType==4){
		referenceSpeed += new_hard_accel_rate*actionChangeCountMax/50.0f;
		speedPID_int = 0;
	}
	else if(actionType==5){
		referenceSpeed += new_hard_decel_rate*actionChangeCountMax/50.0f;
		// speedPID_int = 0;
	}
	else if(actionType==6){
		isChangedLane = 0;
		inLaneCount = 0;
		speedBeforeChange = referenceSpeed;
		referenceSpeed = sqrt(lateral_speed*lateral_speed+referenceSpeed*referenceSpeed);
		// speedPID_int = 0;
	}
	else if(actionType==7){
		isChangedLane = 0;
		inLaneCount = 0;
		speedBeforeChange = referenceSpeed;
		referenceSpeed = sqrt(lateral_speed*lateral_speed+referenceSpeed*referenceSpeed);
		// speedPID_int = 0;
	}
	
	if(!inSafety){
		if(referenceSpeed < min_speed) referenceSpeed = min_speed;
		else if(referenceSpeed > max_speed) referenceSpeed = max_speed;
	}else{
		cout<<"take safety action: "<<actionType<<endl;
	}
}


// Level K controller constraints
void Driver::LKconstraints()
{
	double myDistLon, myDistLat;
	getPos(myDistLon, myDistLat, car);
	int mylane = car->lanePos;

	for(int i=0; i<opponents->getNOpponents(); i++)
	{
		tCarElt* oppCar = opponent[i].getCarPtr();
		double oppDistLon, oppDistLat;
		getPos(oppDistLon, oppDistLat, oppCar);
		float dist = oppDistLon - myDistLon;
		int opplane = classify_lanepos(oppCar);

		if(-constraint_dist < dist && dist < constraint_dist && opplane == (car->lanePos+1) && actionType == ChangeToLeft)
		{
			cout<<"level1 car "<<INDEX+1<<" meet constraint 1: left car in parallel position. actionType: "<<actionType<<endl;
			policyWithout(ChangeToLeft);
		}
		if(-constraint_dist < dist && dist < constraint_dist && opplane == (car->lanePos-1) && actionType == ChangeToRight)
		{
			cout<<"level1 car "<<INDEX+1<<" meet constraint 2: right car in parallel position"<<endl;
			policyWithout(ChangeToRight);
		}
	}
	if( ((msg[0]==0 && msg[1]==1) || (msg[6]==0 && msg[7]==1)) && actionType == ChangeToLeft)
	{
		cout<<"level1 car "<<INDEX+1<<" meet constraint 3: left car close and approaching"<<endl;
		policyWithout(ChangeToLeft);
	} 
	if( ((msg[4]==0 && msg[5]==1) || (msg[8]==0 && msg[9]==1)) && actionType == ChangeToRight)
	{
		cout<<"level1 car "<<INDEX+1<<" meet constraint 4: left car close and approaching"<<endl;
		policyWithout(ChangeToRight);
	}
}

void Driver::policyWithout(const int noActionType){
	float randcopy = randnum*(1.0-pol[noActionType-1]);
	float total = 0.0;
	cout<<"policyWithout "<<noActionType<<"; pol:"<<pol[0]<<", "<<pol[1]<<", "<<pol[2]<<", "<<pol[3]<<", "<<pol[4]<<", "<<pol[5]<<", "<<pol[6]<<"; randcopy: "<<randcopy<<endl;
	for(int i = 0; i < 7; i++)
	{
		if(i!=(noActionType-1)) total += pol[i];
		if(randcopy<=total)
		{
			actionType = i+1;
			break;
		}
	}
	if(noActionType==actionType) cout<<"policy normalization failed. total: "<<total<<endl;
}

void Driver::safetyController(){
	double myDistLon, myDistLat;
	double myVelLon, myVelLat;
	getPos(myDistLon, myDistLat, car);
	getVel(myVelLon, myVelLat, car);

	int mylane = car->lanePos;

	inSafety = false;
	dangerousDegree = 0;

	for(int i=0; i<opponents->getNOpponents(); i++)
	{
		tCarElt* oppCar = opponent[i].getCarPtr();
		double oppDistLon, oppDistLat;
		getPos(oppDistLon, oppDistLat, oppCar);
		float dist = oppDistLon - myDistLon;
		
		double oppVelLon, oppVelLat;
		getVel(oppVelLon, oppVelLat, oppCar);
		float rel_velocity = oppVelLon - myVelLon;

		int opplane = classify_lanepos(oppCar);

		inSafetyZone(dist, rel_velocity);

		if(inSafety && opplane==mylane)
		{
			if(dangerousDegree==1)	actionType = Decelerate;
			else if(dangerousDegree==2)  actionType = HardDecelerate;
			cout<<"level1 car "<<INDEX+1<<" meet safety constraints: front car too close, dangerous degree: "<<dangerousDegree<<endl;
		}
	}
}

void Driver::inSafetyZone(float dist, float vel){
	if(dist > 0 && vel < 0 && -dist/vel < 2.0){
		inSafety = true;
		dangerousDegree = 1;
		cout<<"dangerous degree: 1"<<endl;
	}
	if(dist < (3.0*carlength) && dist > 0){
		inSafety = true;
		dangerousDegree = 1;
		cout<<"dangerous degree: 1"<<endl;
	}
	if(dist > 0 && vel < 0 && -dist/vel < 1.0){
		inSafety = true;
		dangerousDegree = 2;
		cout<<"dangerous degree: 2"<<endl;
	}
	if(dist < (1.8*carlength) && dist > 0){
		inSafety = true;
		dangerousDegree = 2;
		cout<<"dangerous degree: 2"<<endl;
	}
}
 
//update observation message
void Driver::ObserveTtoLK()
{
	double myDistLon, myDistLat;
	double myVelLon, myVelLat;
	getPos(myDistLon, myDistLat, car);
	getVel(myVelLon, myVelLat, car);

	float fl_d = max_sight_distance; // front left, distance
	float fl_v = max_speed - myVelLon + speed_margin + 0.2; // front left, velocity
	float fc_d = max_sight_distance; // front center
	float fc_v = max_speed - myVelLon + speed_margin + 0.2;
	float fr_d = max_sight_distance; // front right
	float fr_v = max_speed - myVelLon + speed_margin + 0.2;
	float rl_d = -max_sight_distance; // rear left
	float rl_v = (min_speed - myVelLon) - speed_margin - 0.2;
	float rr_d = -max_sight_distance; // rear right
	float rr_v = (min_speed - myVelLon) - speed_margin - 0.2;
	int mylane = car->lanePos + 1;

	 for (int i = 0; i < opponents->getNOpponents(); i++)  
	 {
	 	tCarElt* oppCar = opponent[i].getCarPtr();
		double oppDistLon, oppDistLat;
		getPos(oppDistLon, oppDistLat, oppCar);
		float rel_position = oppDistLon - myDistLon; 

		double oppVelLon, oppVelLat;
		getVel(oppVelLon, oppVelLat, oppCar);
		float rel_velocity = oppVelLon - myVelLon;

		int opplane = classify_lanepos(oppCar) + 1;
		// cout<<"Level1 "<<INDEX+1<<" sees opponent "<<oppCar->_driverIndex<<", pos: "<<rel_position<<", vel: "<<rel_velocity<<", lane: "<<opplane<<endl;

		if (opplane == mylane + 1) // in lane directly to left
		{ 
			// cout<<"opponent "<<oppCar->_driverIndex<<" on my left";
			if (rel_position < 0 && rel_position > rl_d) 
			{
				// cout<<", case 1";
				rl_d = rel_position;
				rl_v = rel_velocity;
			} 
			else if (rel_position >= 0 && rel_position < fl_d) 
			{
				// cout<<", case 2";
				fl_d = rel_position;
				fl_v = rel_velocity;
			}
		} 
		else if (opplane == mylane - 1) // in lane directly to right
		{ 
			// cout<<"opponent "<<oppCar->_driverIndex<<" on my right";
			if (rel_position < 0 && rel_position > rr_d) 
			{
				// cout<<", case 1";
				rr_d = rel_position;
				rr_v = rel_velocity;
			} 
			else if (rel_position >= 0 && rel_position < fr_d) 
			{
				// cout<<", case 2";
				fr_d = rel_position;
				fr_v = rel_velocity;
			}
		} 
		else if (opplane == mylane) // in same lane
		{ 
			// cout<<"opponent "<<oppCar->_driverIndex<<" on my lane";
			if (rel_position >= 0 && rel_position < fc_d) 
			{
				// cout<<", case 1";
				fc_d = rel_position;
				fc_v = rel_velocity;
			}
		}
		// cout<<endl;
	}

	// cout<<"Level1 "<<INDEX+1<<" sees "<<" rl_v: " << rl_v <<" (rl_d "<<rl_d<<" rl_v "<<rl_v<< ") rr_v: " << msg[9]<<" (rr_d "<<rr_d<<" rr_v "<<rr_v<<")"<<endl;

	msg[0] = classify_dist(fl_d);
	msg[1] = classify_vel(fl_d, fl_v);
	msg[2] = classify_dist(fc_d);
	msg[3] = classify_vel(fc_d, fc_v);
	msg[4] = classify_dist(fr_d);
	msg[5] = classify_vel(fr_d, fr_v);
	msg[6] = classify_dist(rl_d);
	msg[7] = classify_vel(rl_d, rl_v);
	msg[8] = classify_dist(rr_d);
	msg[9] = classify_vel(rr_d, rr_v);
	msg[10] = mylane;

	if(opponents->getNOpponents() > 0) cout<<"Level1 "<<INDEX+1<<" sees "<<"lane: " << msg[10] << " fc_d: " << msg[2] << " fc_v: " << msg[3] << " fl_d: " << msg[0] << " fl_v: " << msg[1] << " fr_d: " << msg[4] << " fr_v: " << msg[5] << " rl_d: " << msg[6] << " rl_v: " << msg[7] << " rr_d: " << msg[8] << " rr_v: " << msg[9]<<endl;
	// cout<<endl;
}

int Driver::classify_dist(float distance)
{
	float d = abs(distance);

	if (d <= close_distance) {
		return car_close;
	} else if (d <= far_distance) {
		return car_nominal;
	} else {
		return car_far;
	}
}

int Driver::classify_vel(float d, float v)
{
	if (-speed_margin <= v && v <= speed_margin) {
		return car_stable;
	} else if ((d >= 0 && v < speed_margin) || (d <= 0 && v > -speed_margin)) { // condition has a trap. Since we should take out condition in above if(), this implementation gives right result.
		return car_approaching;
	} else if ((d > 0 && v > speed_margin) || (d < 0 && v < -speed_margin)) {
		return car_retreating;
	}
}

//determine actionType according to observation message
void Driver::LKController()
{
	int index = 0; // observation index
	for(int i=0; i<11; ++i) index = index + pow(3.0,10.0-i)*msg[i];
	randnum = rand()/double(RAND_MAX);
	float total = 0;
	pol = getPol(index);
	for(int i = 0; i < 7; i++)
	{
		total += pol[i];
		if(randnum<=total)
		{
			actionType = i+1;
			break;
		}
	}

	LKconstraints();
	
	cout<<"Level1 "<<INDEX+1<<" takes actionType: "<<actionType<<endl;
}

//assign the global ID of this car
int Driver::getmyID()
{
	return INDEX;
}
 
//carry out actions
void Driver::Action_LKtoT()
{
	switch (actionType) 
	{
		case Maintain:
			{
				car->_steerCmd = getSteer(car->lanePos);
				car->_gearCmd = getGear();
				SpeedPID(referenceSpeed);
				break;
			}
		case Accelerate:  
			{
				car->_steerCmd = getSteer(car->lanePos);
				car->_gearCmd = getGear();
				// if(referenceSpeed>max_speed) referenceSpeed = max_speed;
				SpeedPID(referenceSpeed);
				break;
			}
		case Decelerate:
			{
				car->_steerCmd = getSteer(car->lanePos);
				car->_gearCmd = getGear();
				// if(referenceSpeed<min_speed) referenceSpeed = min_speed;
				SpeedPID(referenceSpeed);
				break;
			}
		case HardAccelerate:
			{
				car->_steerCmd = getSteer(car->lanePos);
				car->_gearCmd = getGear();
				// if(referenceSpeed>max_speed) referenceSpeed = max_speed;
				SpeedPID(referenceSpeed);
				break;
			}
		case HardDecelerate:
			{
				car->_steerCmd = getSteer(car->lanePos);
				car->_gearCmd = getGear();
				// if(referenceSpeed<min_speed) referenceSpeed = min_speed;
				SpeedPID(referenceSpeed);
				break;
			}
		case ChangeToLeft:
			{
				if(isChangedLane==0)
				{
					isChangedLane = 1;
					car->lanePos+=1;
				}
				if(car->lanePos>1) car->lanePos = 1;

				car->_steerCmd = getSteer(car->lanePos);
				car->_gearCmd = getGear();

				if(inLaneCount<inLaneCountNum){
					inLaneCount++;
				}else{
					referenceSpeed = speedBeforeChange;
				}
				SpeedPID(referenceSpeed);
				break;
			}
		case ChangeToRight:
			{
				if(isChangedLane==0)
				{
					isChangedLane = 1;
					car->lanePos-=1;
				}
				if(car->lanePos<-1) car->lanePos = -1;

				car->_steerCmd = getSteer(car->lanePos);
				car->_gearCmd = getGear();

				if(inLaneCount<inLaneCountNum){
					inLaneCount++;
				}else{
					referenceSpeed = speedBeforeChange;
				}
				SpeedPID(referenceSpeed);
				break;
			}
	}
}

// go through all action
void Driver::actionTypeIterate()
{
	if(actionType==1){
		actionType = 2;
		referenceSpeed += new_accel_rate*actionChangeCountMax/50;
		// speedPID_int = 0;
	}
	else if(actionType==2){
		actionType = 3;
		referenceSpeed += new_decel_rate*actionChangeCountMax/50;
		// speedPID_int = 0;
	}
	else if(actionType==3){
		actionType = 4;
		referenceSpeed += new_hard_accel_rate*actionChangeCountMax/50;
		// speedPID_int = 0;
	}
	else if(actionType==4){
		actionType = 5;
		referenceSpeed += new_hard_decel_rate*actionChangeCountMax/50;
		// speedPID_int = 0;
	}
	else if(actionType==5){
		actionType = 6;
		isChangedLane = 0;
		inLaneCount = 0;
		speedBeforeChange = referenceSpeed;
		referenceSpeed = sqrt(lateral_speed*lateral_speed+referenceSpeed*referenceSpeed);
		// speedPID_int = 0;
	}
	else if(actionType==6){
		actionType = 7;
		isChangedLane = 0;
		inLaneCount = 0;
		speedBeforeChange = referenceSpeed;
		referenceSpeed = sqrt(lateral_speed*lateral_speed+referenceSpeed*referenceSpeed);
		// speedPID_int = 0;
	}
	else if(actionType==7){
		actionType = 1;
		referenceSpeed = prevreferenceSpeed;
		// speedPID_int = 0;
	}
}

//Cruise and test Global ID of the car
void Driver::Cruise()
{
	if(INDEX==2)
	{
		actionType = 6;
		isChangedLane = 0;
		inLaneCount = 0;
		referenceSpeed = sqrt(lateral_speed*lateral_speed+currentSpeed*currentSpeed);
		speedBeforeChange = currentSpeed;
		speedPID_int = 0;
		cout<<"car "<<2<<endl;
	}
	else if(INDEX==1)
	{
		actionType = 7;
		isChangedLane = 0;
		inLaneCount = 0;
		referenceSpeed = sqrt(lateral_speed*lateral_speed+currentSpeed*currentSpeed);
		speedBeforeChange = currentSpeed;
		speedPID_int = 0;
		cout<<"car "<<1<<endl;
	}
	else if(INDEX==0)
	{
		actionType = 1;
		referenceSpeed = currentSpeed;
		speedPID_int = 0;
		cout<<"car "<<0<<endl;
	}	
}

// A PID controller to reach target speed
void Driver::SpeedPID(float setSpeed)
{
	float realAccel = sqrt(car->pub.DynGCg.acc.x*car->pub.DynGCg.acc.x+car->pub.DynGCg.acc.y*car->pub.DynGCg.acc.y);
	if((car->pub.DynGCg.acc.x*car->_speed_x+car->pub.DynGCg.acc.y*car->_speed_y)<0) realAccel = -realAccel;

	// P error
	float speedDelta = (setSpeed - currentSpeed)/setSpeed; // normalized by setSpeed
	// D error
	float accel0; // normalized by speed error over 0.002s
	if(abs(speedDelta)<0.1) accel0 = realAccel/(0.1/0.002); // prevent being divided by a extremem small value
	else accel0 = realAccel/(speedDelta/0.002);
	// I error
	if(abs(currentSpeed-setSpeed) < speedPID_I_margin) speedPID_int += speedDelta;
	// total PID value 
	float PIDvalue = speedPID_accel_P * speedDelta - speedPID_accel_D * accel0 + speedPID_accel_I * speedPID_int;

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

// Set pitstop commands.
int Driver::pitCommand(tSituation *s)
{
	car->_pitRepair = strategy->pitRepair(car, s);
	car->_pitFuel = strategy->pitRefuel(car, s);
	// This should be the only place where the pit stop is set to false!
	pit->setPitstop(false);
	return ROB_PIT_IM; // return immediately.
}


// End of the current race.
void Driver::endRace(tSituation *s)
{
	// Nothing for now.
}


/***************************************************************************
*
* utility functions (created by bt author)
*
***************************************************************************/


void Driver::computeRadius(float *radius)
{
	float lastturnarc = 0.0f;
	int lastsegtype = TR_STR;

	tTrackSeg *currentseg, *startseg = track->seg;
	currentseg = startseg;

	do {
		if (currentseg->type == TR_STR) {
			lastsegtype = TR_STR;
			radius[currentseg->id] = FLT_MAX;
		} else {
			if (currentseg->type != lastsegtype) {
				float arc = 0.0f;
				tTrackSeg *s = currentseg;
				lastsegtype = currentseg->type;

				while (s->type == lastsegtype && arc < PI/2.0f) {
					arc += s->arc;
					s = s->next;
				}
				lastturnarc = arc/(PI/2.0f);
			}
			radius[currentseg->id] = (currentseg->radius + currentseg->width/2.0)/lastturnarc;
		}
		currentseg = currentseg->next;
	} while (currentseg != startseg);

}


// Compute the allowed speed on a segment.
float Driver::getAllowedSpeed(tTrackSeg *segment)
{
	float mu = segment->surface->kFriction*TIREMU*MU_FACTOR;
	float r = radius[segment->id];
	float dr = learn->getRadius(segment);
	if (dr < 0.0f) {
		r += dr;
	} else {
		float tdr = dr*(1.0f - MIN(1.0f, fabs(myoffset)*2.0f/segment->width));
		r += tdr;	
	}
	// README: the outcommented code is the more save version.
	/*if ((alone > 0 && fabs(myoffset) < USE_LEARNED_OFFSET_RANGE) ||
	dr < 0.0f
	) {
	r += dr;
	}*/
	r = MAX(1.0, r);

	return sqrt((mu*G*r)/(1.0f - MIN(1.0f, r*CA*mu/mass)));
}


// Compute the length to the end of the segment.
float Driver::getDistToSegEnd()
{
	if (car->_trkPos.seg->type == TR_STR) {
		return car->_trkPos.seg->length - car->_trkPos.toStart;
	} else {
		return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
	}
}


// Compute fitting acceleration.
float Driver::getAccel()
{
	if (car->_gear > 0) {
		float allowedspeed = getAllowedSpeed(car->_trkPos.seg);
		if (allowedspeed > car->_speed_x + FULL_ACCEL_MARGIN) {
			return 1.0;
		} else {
			float gr = car->_gearRatio[car->_gear + car->_gearOffset];
			float rm = car->_enginerpmRedLine;
			return allowedspeed/car->_wheelRadius(REAR_RGT)*gr /rm;
		}
	} else {
		return 1.0;
	}
}

// If we get lapped reduce accelerator.
float Driver::filterOverlap(float accel)
{
	int i;
	for (i = 0; i < opponents->getNOpponents(); i++) {
		if (opponent[i].getState() & OPP_LETPASS) {
			return MIN(accel, 0.5f);
		}
	}
	return accel;
}


// Compute initial brake value.
float Driver::getBrake()
{
	// Car drives backward?
	if (car->_speed_x < -MAX_UNSTUCK_SPEED) {
		// Yes, brake.
		return 1.0;
	} else {
		// We drive forward, normal braking.
		tTrackSeg *segptr = car->_trkPos.seg;
		float mu = segptr->surface->kFriction;
		float maxlookaheaddist = currentspeedsqr/(2.0f*mu*G);
		float lookaheaddist = getDistToSegEnd();

		float allowedspeed = getAllowedSpeed(segptr);
		if (allowedspeed < car->_speed_x) {
			return MIN(1.0f, (car->_speed_x-allowedspeed)/(FULL_ACCEL_MARGIN));
		}

		segptr = segptr->next;
		while (lookaheaddist < maxlookaheaddist) {
			allowedspeed = getAllowedSpeed(segptr);
			if (allowedspeed < car->_speed_x) {
				if (brakedist(allowedspeed, mu) > lookaheaddist) {
					return 1.0f;
				}
			}
			lookaheaddist += segptr->length;
			segptr = segptr->next;
		}
		return 0.0f;
	}
}


// Compute gear.
int Driver::getGear()
{
	if (car->_gear <= 0) {
		return 1;
	}
	float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
	float omega = car->_enginerpmRedLine/gr_up;
	float wr = car->_wheelRadius(2);

	if (omega*wr*SHIFT < car->_speed_x) {
		return car->_gear + 1;
	} else {
		float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
		omega = car->_enginerpmRedLine/gr_down;
		if (car->_gear > 1 && omega*wr*SHIFT > car->_speed_x + SHIFT_MARGIN) {
			return car->_gear - 1;
		}
	}
	return car->_gear;
}


// Compute steer value.
float Driver::getSteer(int targetLaneType)
{
	float targetAngle;
	vec2f target = getTargetPoint(targetLaneType);

	targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
	angleError = targetAngle - car->_yaw;
	targetAngle =  angleError * steer_P + (angleError - angleError_old) * steer_D;

	NORM_PI_PI(targetAngle);
	angleError_old = angleError;
	return targetAngle / car->_steerLock;
}


// Compute the clutch value.
float Driver::getClutch()
{
	if (car->_gear > 1) {
		clutchtime = 0.0f;
		return 0.0f;
	} else {
		float drpm = car->_enginerpm - car->_enginerpmRedLine/2.0f;
		clutchtime = MIN(CLUTCH_FULL_MAX_TIME, clutchtime);
		float clutcht = (CLUTCH_FULL_MAX_TIME - clutchtime)/CLUTCH_FULL_MAX_TIME;
		if (car->_gear == 1 && car->_accelCmd > 0.0f) {
			clutchtime += (float) RCM_MAX_DT_ROBOTS;
		}

		if (drpm > 0) {
			float speedr;
			if (car->_gearCmd == 1) {
				// Compute corresponding speed to engine rpm.
				float omega = car->_enginerpmRedLine/car->_gearRatio[car->_gear + car->_gearOffset];
				float wr = car->_wheelRadius(2);
				speedr = (CLUTCH_SPEED + MAX(0.0f, car->_speed_x))/fabs(wr*omega);
				float clutchr = MAX(0.0f, (1.0f - speedr*2.0f*drpm/car->_enginerpmRedLine));
				return MIN(clutcht, clutchr);
			} else {
				// For the reverse gear.
				clutchtime = 0.0f;
				return 0.0f;
			}
		} else {
			return clutcht;
		}
	}
}

// Compute target point for steering.
vec2f Driver::getTargetPoint(int targetLaneType)
{
	tTrackSeg *seg = car->_trkPos.seg;
	float lookahead;
	float length = getDistToSegEnd();
	float offset = getOffset();

	lookahead = 5;
	oldlookahead = lookahead;

	// Search for the segment containing the target point.
	while (length < lookahead) {
		seg = seg->next;
		length += seg->length;
	}

	length = lookahead - length + seg->length;
	float fromstart = seg->lgfromstart;
	fromstart += length;

	// Compute the target point.
	offset = myoffset = pit->getPitOffset(offset, fromstart);

	vec2f s;
	s.x = (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x)/2.0f;
	s.y = (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y)/2.0f;

	float width = seg->startWidth+(seg->endWidth-seg->startWidth)/seg->length*length;
	if ( seg->type == TR_STR) {
		vec2f d, n;
		n.x = (seg->vertex[TR_EL].x - seg->vertex[TR_ER].x)/seg->length;
		n.y = (seg->vertex[TR_EL].y - seg->vertex[TR_ER].y)/seg->length;
		n.normalize();
		d.x = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x)/seg->length;
		d.y = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y)/seg->length;
		return s + d*length + targetLaneType*width/3*n;
	} else {
		vec2f c, n;
		c.x = seg->center.x;
		c.y = seg->center.y;
		float arc = length/seg->radius;
		float arcsign = (seg->type == TR_RGT) ? -1.0f : 1.0f;
		arc = arc*arcsign;
		s = s.rotate(c, arc);

		n = c - s;
		n.normalize();
		return s + targetLaneType*arcsign*width/3*n;
	}
}


// Compute offset to normal target point for overtaking or let pass an opponent.
float Driver::getOffset()
{
	int i;
	float catchdist, mincatchdist = FLT_MAX, mindist = -1000.0f;
	Opponent *o = NULL;

	// Increment speed dependent.
	float incfactor = MAX_INC_FACTOR - MIN(fabs(car->_speed_x)/MAX_INC_FACTOR, (MAX_INC_FACTOR - 1.0f));

	// Let overlap or let less damaged team mate pass.
	for (i = 0; i < opponents->getNOpponents(); i++) {
		// Let the teammate with less damage overtake to use slipstreaming.
		// The position change happens when the damage difference is greater than
		// TEAM_DAMAGE_CHANGE_LEAD.
		if (((opponent[i].getState() & OPP_LETPASS) && !opponent[i].isTeamMate()) ||
			(opponent[i].isTeamMate() && (car->_dammage - opponent[i].getDamage() > TEAM_DAMAGE_CHANGE_LEAD) &&
			(opponent[i].getDistance() > -TEAM_REAR_DIST) && (opponent[i].getDistance() < -car->_dimension_x) &&
			car->race.laps == opponent[i].getCarPtr()->race.laps))
		{
			// Behind, larger distances are smaller ("more negative").
			if (opponent[i].getDistance() > mindist) {
				mindist = opponent[i].getDistance();
				o = &opponent[i];
			}
		}
	}

	if (o != NULL) {
		tCarElt *ocar = o->getCarPtr();
		float side = car->_trkPos.toMiddle - ocar->_trkPos.toMiddle;
		float w = car->_trkPos.seg->width/WIDTHDIV-BORDER_OVERTAKE_MARGIN;
		if (side > 0.0f) {
			if (myoffset < w) {
				myoffset += OVERTAKE_OFFSET_INC*incfactor;
			}
		} else {
			if (myoffset > -w) {
				myoffset -= OVERTAKE_OFFSET_INC*incfactor;
			}
		}
		return myoffset;
	}


	// Overtake.
	for (i = 0; i < opponents->getNOpponents(); i++) {
		if ((opponent[i].getState() & OPP_FRONT) &&
			!(opponent[i].isTeamMate() && car->race.laps <= opponent[i].getCarPtr()->race.laps))
		{
			catchdist = MIN(opponent[i].getCatchDist(), opponent[i].getDistance()*CATCH_FACTOR);
			if (catchdist < mincatchdist) {
				mincatchdist = catchdist;
				o = &opponent[i];
			}
		}
	}

	if (o != NULL) {
		// Compute the width around the middle which we can use for overtaking.
		float w = o->getCarPtr()->_trkPos.seg->width/WIDTHDIV-BORDER_OVERTAKE_MARGIN;
		// Compute the opponents distance to the middle.
		float otm = o->getCarPtr()->_trkPos.toMiddle;
		// Define the with of the middle range.
		float wm = o->getCarPtr()->_trkPos.seg->width*CENTERDIV;

		if (otm > wm && myoffset > -w) {
			myoffset -= OVERTAKE_OFFSET_INC*incfactor;
		} else if (otm < -wm && myoffset < w) {
			myoffset += OVERTAKE_OFFSET_INC*incfactor;
		} else {
			// If the opponent is near the middle we try to move the offset toward
			// the inside of the expected turn.
			// Try to find out the characteristic of the track up to catchdist.
			tTrackSeg *seg = car->_trkPos.seg;
			float length = getDistToSegEnd();
			float oldlen, seglen = length;
			float lenright = 0.0f, lenleft = 0.0f;
			mincatchdist = MIN(mincatchdist, DISTCUTOFF);

			do {
				switch (seg->type) {
				case TR_LFT:
					lenleft += seglen;
					break;
				case TR_RGT:
					lenright += seglen;
					break;
				default:
					// Do nothing.
					break;
				}
				seg = seg->next;
				seglen = seg->length;
				oldlen = length;
				length += seglen;
			} while (oldlen < mincatchdist);

			// If we are on a straight look for the next turn.
			if (lenleft == 0.0f && lenright == 0.0f) {
				while (seg->type == TR_STR) {
					seg = seg->next;
				}
				// Assume: left or right if not straight.
				if (seg->type == TR_LFT) {
					lenleft = 1.0f;
				} else {
					lenright = 1.0f;
				}
			}

			// Because we are inside we can go to the border.
			float maxoff = (o->getCarPtr()->_trkPos.seg->width - car->_dimension_y)/2.0-BORDER_OVERTAKE_MARGIN;
			if (lenleft > lenright) {
				if (myoffset < maxoff) {
					myoffset += OVERTAKE_OFFSET_INC*incfactor;
				}
			} else {
				if (myoffset > -maxoff) {
					myoffset -= OVERTAKE_OFFSET_INC*incfactor;
				}
			}
		}
	} else {
		// There is no opponent to overtake, so the offset goes slowly back to zero.
		if (myoffset > OVERTAKE_OFFSET_INC) {
			myoffset -= OVERTAKE_OFFSET_INC;
		} else if (myoffset < -OVERTAKE_OFFSET_INC) {
			myoffset += OVERTAKE_OFFSET_INC;
		} else {
			myoffset = 0.0f;
		}
	}

	return myoffset;
}


// Update my private data every timestep.
void Driver::update(tSituation *s)
{
	// Update global car data (shared by all instances) just once per timestep.
	if (currentsimtime != s->currentTime) {
		currentsimtime = s->currentTime;
		cardata->update();
	}

	// Update the local data rest.
	speedangle = mycardata->getTrackangle() - atan2(car->_speed_Y, car->_speed_X);
	NORM_PI_PI(speedangle);
	mass = CARMASS + car->_fuel;
	currentspeedsqr = car->_speed_x*car->_speed_x;
	opponents->update(s, this);
	strategy->update(car, s);
	if (!pit->getPitstop()) {
		pit->setPitstop(strategy->needPitstop(car, s));
	}
	pit->update();
	alone = isAlone();
	learn->update(s, track, car, alone, myoffset, car->_trkPos.seg->width/WIDTHDIV-BORDER_OVERTAKE_MARGIN, radius);
}


int Driver::isAlone()
{
	int i;
	for (i = 0; i < opponents->getNOpponents(); i++) {
		if (opponent[i].getState() & (OPP_COLL | OPP_LETPASS)) {
			return 0;	// Not alone.
		}
	}
	return 1;	// Alone.
}


// Check if I'm stuck.
bool Driver::isStuck()
{
	if (fabs(mycardata->getCarAngle()) > MAX_UNSTUCK_ANGLE &&
		car->_speed_x < MAX_UNSTUCK_SPEED &&
		fabs(car->_trkPos.toMiddle) > MIN_UNSTUCK_DIST) {
			if (stuck > MAX_UNSTUCK_COUNT && car->_trkPos.toMiddle*mycardata->getCarAngle() < 0.0) {
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


// Compute aerodynamic downforce coefficient CA.
void Driver::initCa()
{
	const char *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
	float rearwingarea = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGAREA, (char*) NULL, 0.0f);
	float rearwingangle = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGANGLE, (char*) NULL, 0.0f);
	float wingca = 1.23f*rearwingarea*sin(rearwingangle);

	float cl = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FCL, (char*) NULL, 0.0f) +
		GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_RCL, (char*) NULL, 0.0f);
	float h = 0.0f;
	int i;
	for (i = 0; i < 4; i++)
		h += GfParmGetNum(car->_carHandle, WheelSect[i], PRM_RIDEHEIGHT, (char*) NULL, 0.20f);
	h*= 1.5f; h = h*h; h = h*h; h = 2.0f * exp(-3.0f*h);
	CA = h*cl + 4.0f*wingca;
}


// Compute aerodynamic drag coefficient CW.
void Driver::initCw()
{
	float cx = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_CX, (char*) NULL, 0.0f);
	float frontarea = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FRNTAREA, (char*) NULL, 0.0f);
	CW = 0.645f*cx*frontarea;
}


// Init the friction coefficient of the the tires.
void Driver::initTireMu()
{
	const char *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
	float tm = FLT_MAX;
	int i;

	for (i = 0; i < 4; i++) {
		tm = MIN(tm, GfParmGetNum(car->_carHandle, WheelSect[i], PRM_MU, (char*) NULL, 1.0f));
	}
	TIREMU = tm;
}


// Reduces the brake value such that it fits the speed (more downforce -> more braking).
float Driver::filterBrakeSpeed(float brake)
{
	float weight = (CARMASS + car->_fuel)*G;
	float maxForce = weight + CA*MAX_SPEED*MAX_SPEED;
	float force = weight + CA*currentspeedsqr;
	return brake*force/maxForce;
}


// Brake filter for pit stop.
float Driver::filterBPit(float brake)
{
	if (pit->getPitstop() && !pit->getInPit()) {
		float dl, dw;
		RtDistToPit(car, track, &dl, &dw);
		if (dl < PIT_BRAKE_AHEAD) {
			float mu = car->_trkPos.seg->surface->kFriction*TIREMU*PIT_MU;
			if (brakedist(0.0f, mu) > dl) {
				return 1.0f;
			}
		}
	}

	if (pit->getInPit()) {
		float s = pit->toSplineCoord(car->_distFromStartLine);
		// Pit entry.
		if (pit->getPitstop()) {
			float mu = car->_trkPos.seg->surface->kFriction*TIREMU*PIT_MU;
			if (s < pit->getNPitStart()) {
				// Brake to pit speed limit.
				float dist = pit->getNPitStart() - s;
				if (brakedist(pit->getSpeedlimit(), mu) > dist) {
					return 1.0f;
				}
			} else {
				// Hold speed limit.
				if (currentspeedsqr > pit->getSpeedlimitSqr()) {
					return pit->getSpeedLimitBrake(currentspeedsqr);
				}
			}
			// Brake into pit (speed limit 0.0 to stop)
			float dist = pit->getNPitLoc() - s;
			if (pit->isTimeout(dist)) {
				pit->setPitstop(false);
				return 0.0f;
			} else {
				if (brakedist(0.0f, mu) > dist) {
					return 1.0f;
				} else if (s > pit->getNPitLoc()) {
					// Stop in the pit.
					return 1.0f;
				}
			}
		} else {
			// Pit exit.
			if (s < pit->getNPitEnd()) {
				// Pit speed limit.
				if (currentspeedsqr > pit->getSpeedlimitSqr()) {
					return pit->getSpeedLimitBrake(currentspeedsqr);
				}
			}
		}
	}

	return brake;
}

// Brake filter for collision avoidance.
float Driver::filterBColl(float brake)
{
	float mu = car->_trkPos.seg->surface->kFriction;
	int i;
	for (i = 0; i < opponents->getNOpponents(); i++) {
		if (opponent[i].getState() & OPP_COLL) {
			if (brakedist(opponent[i].getSpeed(), mu) > opponent[i].getDistance()) {
				return 1.0f;
			}
		}
	}
	return brake;
}


// Steer filter for collision avoidance.
float Driver::filterSColl(float steer)
{
	int i;
	float sidedist = 0.0f, fsidedist = 0.0f, minsidedist = FLT_MAX;
	Opponent *o = NULL;

	// Get the index of the nearest car (o).
	for (i = 0; i < opponents->getNOpponents(); i++) {
		if (opponent[i].getState() & OPP_SIDE) {
			sidedist = opponent[i].getSideDist();
			fsidedist = fabs(sidedist);
			if (fsidedist < minsidedist) {
				minsidedist = fsidedist;
				o = &opponent[i];
			}
		}
	}

	// If there is another car handle the situation.
	if (o != NULL) {
		float d = fsidedist - o->getWidth();
		// Near, so we need to look at it.
		if (d < SIDECOLL_MARGIN) {
			/* compute angle between cars */
			tCarElt *ocar = o->getCarPtr();
			float diffangle = ocar->_yaw - car->_yaw;
			NORM_PI_PI(diffangle);
			// We are near and heading toward the car.
			if (diffangle*o->getSideDist() < 0.0f) {
				const float c = SIDECOLL_MARGIN/2.0f;
				d = d - c;
				if (d < 0.0f) {
					d = 0.0f;
				}

				// Steer delta required to drive parallel to the opponent.
				float psteer = diffangle/car->_steerLock;
				myoffset = car->_trkPos.toMiddle;

				// Limit myoffset to suitable limits.
				float w = o->getCarPtr()->_trkPos.seg->width/WIDTHDIV-BORDER_OVERTAKE_MARGIN;
				if (fabs(myoffset) > w) {
					myoffset = (myoffset > 0.0f) ? w : -w;
				}

				// On straights the car near to the middle can correct more, in turns the car inside
				// the turn does (because if you leave the track on the turn "inside" you will skid
				// back to the track.
				if (car->_trkPos.seg->type == TR_STR) {
					if (fabs(car->_trkPos.toMiddle) > fabs(ocar->_trkPos.toMiddle)) {
						// Its me, I do correct not that much.
						psteer = steer*(d/c) + 1.5f*psteer*(1.0f - d/c);
					} else {
						// Its the opponent, so I correct more.
						psteer = steer*(d/c) + 2.0f*psteer*(1.0f - d/c);
					}
				} else {
					// Who is outside, heavy corrections are less dangerous
					// if you drive near the middle of the track.
					float outside = car->_trkPos.toMiddle - ocar->_trkPos.toMiddle;
					float sign = (car->_trkPos.seg->type == TR_RGT) ? 1.0f : -1.0f;
					if (outside*sign > 0.0f) {
						psteer = steer*(d/c) + 1.5f*psteer*(1.0f - d/c);
					} else {
						psteer = steer*(d/c) + 2.0f*psteer*(1.0f - d/c);
					}
				}

				if (psteer*steer > 0.0f && fabs(steer) > fabs(psteer)) {
					return steer;
				} else {
					return psteer;
				}
			}
		}
	}
	return steer;
}


// Antilocking filter for brakes.
float Driver::filterABS(float brake)
{
	if (car->_speed_x < ABS_MINSPEED) return brake;
	int i;
	float slip = 0.0f;
	for (i = 0; i < 4; i++) {
		slip += car->_wheelSpinVel(i) * car->_wheelRadius(i);
	}
	slip = car->_speed_x - slip/4.0f;
	if (slip > ABS_SLIP) {
		brake = brake - MIN(brake, (slip - ABS_SLIP)/ABS_RANGE);
	}
	return brake;
}


// TCL filter for accelerator pedal.
float Driver::filterTCL(float accel)
{
	float slip = (this->*GET_DRIVEN_WHEEL_SPEED)() - car->_speed_x;
	if (slip > TCL_SLIP) {
		accel = accel - MIN(accel, (slip - TCL_SLIP)/TCL_RANGE);
	}
	return accel;
}


// Traction Control (TCL) setup.
void Driver::initTCLfilter()
{
	const char *traintype = GfParmGetStr(car->_carHandle, SECT_DRIVETRAIN, PRM_TYPE, VAL_TRANS_RWD);
	if (strcmp(traintype, VAL_TRANS_RWD) == 0) {
		GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_RWD;
	} else if (strcmp(traintype, VAL_TRANS_FWD) == 0) {
		GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_FWD;
	} else if (strcmp(traintype, VAL_TRANS_4WD) == 0) {
		GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_4WD;
	}
}


// TCL filter plugin for rear wheel driven cars.
float Driver::filterTCL_RWD()
{
	return (car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) *
		car->_wheelRadius(REAR_LFT) / 2.0f;
}


// TCL filter plugin for front wheel driven cars.
float Driver::filterTCL_FWD()
{
	return (car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) *
		car->_wheelRadius(FRNT_LFT) / 2.0f;
}


// TCL filter plugin for all wheel driven cars.
float Driver::filterTCL_4WD()
{
	return ((car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) *
		car->_wheelRadius(FRNT_LFT) +
		(car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) *
		car->_wheelRadius(REAR_LFT)) / 4.0f;
}


// Hold car on the track.
float Driver::filterTrk(float accel)
{
	tTrackSeg* seg = car->_trkPos.seg;

	if (car->_speed_x < MAX_UNSTUCK_SPEED ||		// Too slow.
		pit->getInPit() ||							// Pit stop.
		car->_trkPos.toMiddle*speedangle > 0.0f)	// Speedvector points to the inside of the turn.
	{
		return accel;
	}

	if (seg->type == TR_STR) {
		float tm = fabs(car->_trkPos.toMiddle);
		float w = (seg->width - car->_dimension_y)/2.0f ;
		if (tm > w) {
			return 0.0f;
		} else {
			return accel;
		}
	} else {
		float sign = (seg->type == TR_RGT) ? -1.0f : 1.0f;
		if (car->_trkPos.toMiddle*sign > 0.0f) {
			return accel;
		} else {
			float tm = fabs(car->_trkPos.toMiddle);
			float w = seg->width/WIDTHDIV;
			if (tm > w) {
				return 0.0f;
			} else {
				return accel;
			}
		}
	}
}


// Compute the needed distance to brake.
float Driver::brakedist(float allowedspeed, float mu)
{
	float c = mu*G;
	float d = (CA*mu + CW)/mass;
	float v1sqr = currentspeedsqr;
	float v2sqr = allowedspeed*allowedspeed;
	return -log((c + v2sqr*d)/(c + v1sqr*d))/(2.0f*d);
}

