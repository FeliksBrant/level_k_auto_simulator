/***************************************************************************

    file                 : tests.cpp
    created              : 2018-01-30 01:03:34 UTC
    author               : Guankun Su
    email                : guanksu@umich.edu
    modified time        : 2018-02-05 23:11:37
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


#include "unit_test_framework.h" 

const float speed_margin = 2.0 / 3.6f; // m/s
#define car_stable 0 
#define car_approaching 1 
#define car_retreating 2 



int classify_vel(float d, float v);
TEST(classify_vel) {
    ASSERT_EQUAL(classify_vel(0,0),car_stable);
    ASSERT_EQUAL(classify_vel(0,speed_margin),car_stable);
    ASSERT_EQUAL(classify_vel(0,-speed_margin),car_stable);
    ASSERT_EQUAL(classify_vel(10,0),car_stable);
    ASSERT_EQUAL(classify_vel(-10,0),car_stable);

    ASSERT_EQUAL(classify_vel(-0.1,-speed_margin-0.1),car_retreating);
    ASSERT_EQUAL(classify_vel(0.1,speed_margin+0.1),car_retreating);
    ASSERT_EQUAL(classify_vel(0,speed_margin+0.1),car_retreating);

    ASSERT_EQUAL(classify_vel(0,-speed_margin-0.1),car_approaching);
    ASSERT_EQUAL(classify_vel(0.1,-speed_margin-0.1),car_approaching);
    ASSERT_EQUAL(classify_vel(-0.1,speed_margin+0.1),car_approaching);
}


int classify_vel(float d, float v)
{
    if (-speed_margin <= v && v <= speed_margin) {
        return car_stable;
    } else if ((d >= 0 && v < -speed_margin) || (d < 0 && v > speed_margin)) {
        return car_approaching;
    } else if ((d >= 0 && v > speed_margin) || (d < 0 && v < -speed_margin)) {
        return car_retreating;
    }
    return car_retreating; // never reaches
}

#define car_close 0 
#define car_nominal 1 
#define car_far 2 
#define car_stable 0 
#define car_approaching 1 
#define car_retreating 2 
#define Maintain 1
#define Accelerate 2
#define Decelerate 3
#define HardAccelerate 4
#define HardDecelerate 5
#define ChangeToLeft 6
#define ChangeToRight 7
int getL0Pol(int observation);
TEST(level0_Policy){
    ASSERT_EQUAL(getL0Pol(8748),Decelerate);
    ASSERT_EQUAL(getL0Pol(0),Decelerate);
    ASSERT_EQUAL(getL0Pol(79825),Decelerate);
    ASSERT_EQUAL(getL0Pol(88573),Decelerate);

    ASSERT_EQUAL(getL0Pol(2187),HardDecelerate);
    ASSERT_EQUAL(getL0Pol(82012),HardDecelerate);

    ASSERT_EQUAL(getL0Pol(4374),Maintain);
    ASSERT_EQUAL(getL0Pol(6561),Maintain);
    ASSERT_EQUAL(getL0Pol(10935),Maintain);
    ASSERT_EQUAL(getL0Pol(131122),Maintain);
    ASSERT_EQUAL(getL0Pol(15309),Maintain);
    ASSERT_EQUAL(getL0Pol(17496),Maintain);
}

int getL0Pol(int observation){
    int msg2 = int(observation/pow(3,8))%3;
    int msg3 = int(observation/pow(3,7))%3;
    int actType = 0;
    if((msg2==car_nominal && msg3==car_approaching) || (msg2==car_close && msg3==car_stable)) actType = Decelerate;
    else if(msg2==car_close && msg3==car_approaching) actType = HardDecelerate;
    else actType = Maintain;
    return actType;
}

TEST_MAIN()
