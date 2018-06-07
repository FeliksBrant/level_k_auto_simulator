/***************************************************************************
*
*    file        : record.h
*    author      : Guankun Su
*    summary     : record data utilities and main function
*
****************************************************************************/

#include "record.h"


double getSpeedAngle(tCarElt* car);
double getAccelAngle(tCarElt* car);

// REQUIRES:
// MODIFIES: misc
// EFFECT: Start recording at startRecord and prompt user that we have started recording
//         record oen row of data into myfile each second
//         quit recording after we have recorded maxDataAmount rows of data
void record_main(tCarElt* car, tSituation *s, tTrack *curTrack, ofstream &myfile, recordMisc &misc)
{
    // record Observation Space data
    int msg[11];

    if (misc.actChangeCount > actionChangeCountMax)
    {
        // cout<<car->index<<s->cars[0]->index<<s->cars[1]->index<<s->cars[2]->index<<endl;
        // cout<<"me:"<<car->index<<" others:"<<s->cars[0]->index<<s->cars[1]->index<<s->cars[2]->index<<endl;
        classify_lanepos(car);
        ObserveTtoLK(msg, car, s, curTrack, misc);

        misc.simTime = s->currentTime;
        if (misc.simTime > startRecord)
        {
            if (misc.dataIndex == -1)
            {
                // int dummyaction = getAction(car);
                misc.dataIndex += 1;
                cout << "\n"
                     << "--------------\n"
                     << "Data recording initialization complete!\n"
                     << "--------------\n"
                     << "Current time is " << s->currentTime << " s" << endl;
            }

            if (misc.dataIndex == 0) cout << "Start Recording at: " << s->currentTime << "s" << endl;

            if (misc.dataIndex < maxDataAmount)
            {
                misc.actRecord = getAction(car, misc);
                double distLon, distLat, velLong, velLat, accelLong, accelLat, yaw, yawRate;
                ostringstream datatemp;

                getPos(distLon, distLat, car);
                getVel(car, velLong, velLat);
                getAccel(car, accelLong, accelLat);
                yaw = getYaw(car);
                yawRate = getYawRate(car);

                datatemp << misc.simTime << "\t" 
                         << misc.actRecord << "\t" 
                         << msg[0]<<msg[1]<<msg[2]<<msg[3]<<msg[4]<<msg[5]<<msg[6]<<msg[7]<<msg[8]<<msg[9]<<msg[10]<< "\t"
                         << distLon << "\t\t" << distLat << "\t\t" 
                         << velLong << "\t" << velLat << "\t" 
                         << accelLong << "\t\t" << accelLat << "\t\t" 
                         << yaw << "\t"
                         << yawRate << "\t\t" 
                         << "\n";
                myfile << datatemp.str();

                misc.dataIndex += 1;
            }
            if (misc.dataIndex == maxDataAmount)
            {
                cout << "time now: " << s->currentTime << endl;
                cout << "Data acquisition finished." << endl;
                myfile.close();
                misc.dataIndex += 1;
            }
        }

        misc.oldSimTime = misc.simTime;
        misc.actChangeCount = 0;
    }
    else
    {
        misc.actChangeCount ++;
    }

    //end recording
}

// REQUIRES:
// MODIFIES: misc
// EFFECT: prompt user to type in their recording number
//         open the file to record data
//         prompt some description about the recording data
void record_init(recordMisc &misc,ofstream &myfile, tTrack *curTrack)
{
    int number;

    cout << "\n ============== \n Experiment starts now \n ============== \n" << endl;

    do{
        cout << "Please enter your number: ";
        cin >> number;
        ostringstream ss;
        ss << "Player " << number << ".txt";
        misc.filename = misc.path + ss.str();    
    }while(ifstream(misc.filename.c_str())); // check if file alread exists

    cout << "Your file will be: " << misc.filename << endl;    
    myfile.open (misc.filename.c_str(), ios::app | ios::out);
    myfile << "File description: "
           << "track name: " << curTrack->name << " ; "
           << "track width: " << curTrack->width << " m ; "
           << "frame of reference: inertia Earth frame (world axis as in TORCS) ;"
           << endl;
    myfile << misc.dataFormat;
}

// REQUIRES: msg[] should have exact 11 elements
// MODIFIES:
// EFFECT: update observation message
void ObserveTtoLK(int* msg, tCarElt* car, tSituation *s, tTrack *curTrack, recordMisc &misc)
{
    float mySpeed = sqrt(car->_speed_X * car->_speed_X + car->_speed_Y * car->_speed_Y);
    float fl_d = max_sight_distance; // front left, distance
    float fl_v = max_speed - mySpeed + 0.1; // front left, velocity
    float fc_d = max_sight_distance; // front center
    float fc_v = max_speed - mySpeed + 0.1;
    float fr_d = max_sight_distance; // front right
    float fr_v = max_speed - mySpeed + 0.1;
    float rl_d = -max_sight_distance; // rear left
    float rl_v = (min_speed - mySpeed) - 0.1;
    float rr_d = -max_sight_distance; // rear right
    float rr_v = (min_speed - mySpeed) - 0.1; 
    int mylane = car->lanePos;

    for (int i = 0; i < s->_ncars; i++)
    {
        if (s->cars[i]->index == car->index) continue;
        float rel_position = s->cars[i]->_distFromStartLine - car->_distFromStartLine;
        // rationalize relative position
        if (rel_position > 0)
        {
            rel_position = abs(rel_position - curTrack->length) < abs(rel_position) ? (rel_position - curTrack->length) : rel_position;
        }
        else if (rel_position < 0)
        {
            rel_position = abs(rel_position + curTrack->length) < abs(rel_position) ? (rel_position + curTrack->length) : rel_position;
        }

        float rel_velocity = sqrt(s->cars[i]->_speed_X * s->cars[i]->_speed_X + s->cars[i]->_speed_Y * s->cars[i]->_speed_Y) - mySpeed;
        int opplane = s->cars[i]->lanePos;
        // cout<<"Car "<<car->index<<" sees opponent "<<i<<", pos: "<<rel_position<<", vel: "<<rel_velocity<<", lane: "<<opplane<<endl;

        if (opplane == mylane + 1) // in lane directly to left
        {
            if (rel_position < 0 && rel_position > rl_d)
            {
                rl_d = rel_position;
                rl_v = rel_velocity;
            }
            else if (rel_position >= 0 && rel_position < fl_d)
            {
                fl_d = rel_position;
                fl_v = rel_velocity;
            }
        }
        else if (opplane == mylane - 1) // in lane directly to right
        {
            if (rel_position < 0 && rel_position > rr_d)
            {
                rr_d = rel_position;
                rr_v = rel_velocity;
            }
            else if (rel_position >= 0 && rel_position < fr_d)
            {
                fr_d = rel_position;
                fr_v = rel_velocity;
            }
        }
        else if (opplane == mylane) // in same lane
        {
            if (rel_position >= 0 && rel_position < fc_d)
            {
                fc_d = rel_position;
                fc_v = rel_velocity;
            }
        }
    }

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
    msg[10] = mylane + 1;

    // if(s->_ncars > 1) cout<<"Car "<<car->index<<" sees "<<"lane: " << msg[10]<<endl;
    // cout<<"Car "<<car->index<<" sees opponent "<<i<<", pos: "<<rel_position<<", vel: "<<rel_velocity<<", lane: "<<opplane<<endl;
    // if(s->_ncars > 1) cout<<"Car "<<car->index<<" sees "<<"lane: " << msg[10] << " fc_d: " << msg[2] << " fc_v: " << msg[3] << " fl_d: " << msg[0] << " fl_v: " << msg[1] << " fr_d: " << msg[4] << " fr_v: " << msg[5] << " rl_d: " << msg[6] << " rl_v: " << msg[7] << " rr_d: " << msg[8] << " rr_v: " << msg[9]<<endl;
    // cout<<"Car "<<car->index<<" sees "<<" rl_v: " << msg[7] <<" (rl_d "<<rl_d<<" rl_v "<<rl_v<< ") rr_v: " << msg[9]<<" (rr_d "<<rr_d<<" rr_v "<<rr_v<<")"<<endl;
}

// REQUIRES:
// MODIFIES:
// EFFECT: classify the distance to be car_close, car_nominal or car_far
int classify_dist(float distance)
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

// REQUIRES:
// MODIFIES:
// EFFECT: classify the relative velocity to be car_stable, car_approaching or car_retreating
int classify_vel(float d, float v)
{
    if (-speed_margin <= v && v <= speed_margin) {
        return car_stable;
    } else if ((d >= 0 && v < speed_margin) || (d <= 0 && v > -speed_margin)) {
        return car_approaching;
    } else if ((d > 0 && v > speed_margin) || (d < 0 && v < -speed_margin)) {
        return car_retreating;
    }
}

// REQUIRES:
// MODIFIES: car->lanePos
// EFFECT: classify the lane position of the car to be left lane (1), middle lane (0) or right lane (-1)
void classify_lanepos(tCarElt* car)
{
    float Width = car->_trkPos.seg->width;
    if (-Width / 6 < car->_trkPos.toMiddle && car->_trkPos.toMiddle < Width / 6) car->lanePos = 0;
    else if (Width / 6 <= car->_trkPos.toMiddle)   car->lanePos = 1;
    else car->lanePos = -1;
    // cout<<"lanePos: "<<car->lanePos<<endl;
}

// REQUIRES:
// MODIFIES:
// EFFECT: classify car action to be 1~7 (according to action space)
//         and return the action
int getAction(tCarElt* car, recordMisc &misc)
{
    int action = Maintain;
    float mySpeed = sqrt(car->_speed_X * car->_speed_X + car->_speed_Y * car->_speed_Y);
    if (car->lanePos > misc.prevlane) action = ChangeToLeft;
    else if (car->lanePos < misc.prevlane) action = ChangeToRight;
    else
    {
        float speedChange = mySpeed - misc.prevSpeed;
        if (speedChange >= -maintainRange && speedChange <= maintainRange) action = Maintain;
        else if (speedChange > maintainRange && speedChange <= accelerateRange) action = Accelerate;
        else if (speedChange > accelerateRange) action = HardAccelerate;
        else if (speedChange >= -accelerateRange && speedChange < -maintainRange) action = Decelerate;
        else if (speedChange < -accelerateRange) action = HardDecelerate;
    }
    misc.prevSpeed = mySpeed;
    misc.prevlane = car->lanePos;
    return action;
}

// REQUIRES:
// MODIFIES: distLon, distLat
// EFFECT: get the current distLon and distLat from startline
void getPos(double &distLon, double &distLat, tCarElt* car)
{
    distLon = car->_distRaced;
    distLat = car->_trkPos.toMiddle;
    // cout<<"distLon: "<<distLon<<" ; distLat: "<<distLat<<endl;
}

// REQUIRES:
// MODIFIES: velLong, velLat
// EFFECT: in Earth frame (world axis as in TORCS)
//         get the longitudinal and lateral velocity of car along track
void getVel(tCarElt* car, double &velLong, double &velLat)
{
    double speedangle = getSpeedAngle(car);
    double Vel = sqrt(car->_speed_X*car->_speed_X + car->_speed_Y*car->_speed_Y);
    velLong = Vel*cos(speedangle); // > 0 if velLon directs forward
    velLat = Vel*sin(speedangle); // > 0 if velLat directs left
    // cout<<"velLong: "<<velLong<<" ; velLat: "<<velLat<<endl;
}

// REQUIRES:
// MODIFIES: accelLong, accelLat
// EFFECT: in Earth frame (world axis as in TORCS)
//         get the longitudinal and lateral acceleartion of car
void getAccel(tCarElt* car, double  &accelLong, double &accelLat)
{
    double accelangle = getAccelAngle(car);
    double Accel = sqrt(car->pub.DynGCg.acc.x*car->pub.DynGCg.acc.x + car->pub.DynGCg.acc.y*car->pub.DynGCg.acc.y);
    accelLong = Accel*cos(accelangle);
    accelLat = Accel*sin(accelangle);
    // cout<<"accelLong: "<<accelLong<<" ; accelLat: "<<accelLat<<endl;
}

// REQUIRES:
// MODIFIES:
// EFFECT: in Earth frame (world axis as in TORCS)
//         get the Yaw angle of car
double getYaw(tCarElt* car)
{
    double trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    double yaw = car->pub.DynGCg.pos.az - trackangle;
    NORM_PI_PI(yaw);
    // cout<<"yaw: "<<yaw<<endl;
    return yaw;
}

// REQUIRES:
// MODIFIES:
// EFFECT: in Earth frame (world axis as in TORCS)
//         get the Yaw rate of car
double getYawRate(tCarElt* car)
{
    double yawRate = car->pub.DynGCg.vel.az;
    // cout<<"yawRate: "<<yawRate<<"\n"<<endl;
    return yawRate;
}

// REQUIRES:
// MODIFIES:
// EFFECT: get the angle of the speed vector relative to trackangle, > 0.0 points to left
double getSpeedAngle(tCarElt* car)
{
    double trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    double speedangle = atan2(car->_speed_Y, car->_speed_X) - trackangle;
    NORM_PI_PI(speedangle);
    return speedangle;
}

// REQUIRES:
// MODIFIES:
// EFFECT: get the angle of the accel vector relative to trackangle, > 0.0 points to left
double getAccelAngle(tCarElt* car)
{
    double trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    double accelangle = atan2(car->pub.DynGCg.acc.y, car->pub.DynGCg.acc.x) - trackangle;
    NORM_PI_PI(accelangle);
    return accelangle;
}
/***************************************************************************
*
* End of player data recording utilities (created by Guankun Su)
*
***************************************************************************/
