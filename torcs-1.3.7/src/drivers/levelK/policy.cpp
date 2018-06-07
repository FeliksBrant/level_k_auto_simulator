/***************************************************************************

    file                 : policy.cpp
    created              : 2018-01-13 02:44:33 UTC
    author               : Guankun Su
    email                : guanksu@umich.edu
    modified time        : 2018-04-02 01:23:09
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

#include "policy.h"
#include "LevelKDriver.h"
#include <string.h>
#include <iostream>
#include <fstream>
#include <math.h>
using namespace std;

const int Policy::MAX_OBSERVATION_NUM = 177147;
const long int Policy::MAX_POLICY_NUM = 1240029;
const int Policy::ACTION_NUM = 7;


int L0_Policy::getL0Pol(int observation) const{
    int msg2 = int(observation/pow(3.0f,8))%3;
    int msg3 = int(observation/pow(3.0f,7))%3;
    int actType = 0;
    if((msg2==car_nominal && msg3==car_approaching) || (msg2==car_close && msg3==car_stable)) actType = Decelerate;
    else if(msg2==car_close && msg3==car_approaching) actType = HardDecelerate;
    // follow two lines are added
    // else if(msg2==car_far && msg3==car_stable) actType = Accelerate;
    // else if(msg2==car_far && msg3==car_retreating) actType = Accelerate;
    // above two lines are added
    else actType = Maintain;
    return actType;
}

const string L1_Policy::policy_file = "/home/guanksu/Desktop/TORCS_ubuntu/torcs-1.3.7/src/drivers/levelK/Level1_Policy.dat";

float L1_Policy::policy[1240029];
bool L1_Policy::isPolicy_assigned = false;

L1_Policy::L1_Policy(): Policy(1) {
    if(!isPolicy_assigned) assign_policy();
}

// MODIFIES: pol
// EFFECT: have pol point to first policy segment for the observation
void L1_Policy::getLKPol(int observation, float** pol) const{
    *pol = (policy + ACTION_NUM*observation);
    // cout<<"l1 pol.cpp: "<<*pol<<endl;
}

void L1_Policy::assign_policy(){
    cout<<"Loading level 1 policy." <<endl;
    cout<<"...." <<endl;

    ifstream fin;
    fin.open(policy_file.c_str(),ios::in);
    for(int i = 0; i < MAX_OBSERVATION_NUM; i++)
    {
        float temp;
        fin >> temp;
        for(int j = ACTION_NUM*i; j < ACTION_NUM*i+ACTION_NUM; j++)
        {
            fin >> temp;
            fin >> temp;
            fin >> temp;
            fin >> temp;
            policy[j] = temp;         
        }
        fin >> temp;
        fin >> temp;
        fin >> temp;
    }
    fin.close();

    // for debug use
    cout<<"Level 1 policy sanity check: "<<endl;
    for(int j = 0; j < 15; j++)
    {
        for(int i = 0; i < ACTION_NUM-1; ++i){
            cout<<policy[ACTION_NUM*j+i]<<", ";
        }
        cout<<policy[ACTION_NUM*(j+1)-1]<<endl;
    }

    isPolicy_assigned = true;
    cout<<"Level 1 policy loading-> "<< (isPolicy_assigned ? "Success!" : "Fail!") <<endl;
}

const string L2_Policy::policy_file = "/home/guanksu/Desktop/TORCS_ubuntu/torcs-1.3.7/src/drivers/levelK/Level2_Policy.dat";

float L2_Policy::policy[1240029];
bool L2_Policy::isPolicy_assigned = false;

L2_Policy::L2_Policy(): Policy(2) {
    if(!isPolicy_assigned) assign_policy();
}

// MODIFIES: pol
// EFFECT: have pol point to first policy segment for the observation
void L2_Policy::getLKPol(int observation, float** pol) const{
    *pol = (policy + ACTION_NUM*observation);
    // cout<<"l2 pol.cpp: "<<*pol<<endl;
}


void L2_Policy::assign_policy(){
    cout<<"Loading level 2 policy." <<endl;
    cout<<"...." <<endl;

    ifstream fin;
    fin.open(policy_file.c_str(),ios::in);
    for(int i = 0; i < MAX_OBSERVATION_NUM; i++)
    {
        float temp;
        fin >> temp;
        for(int j = ACTION_NUM*i; j < ACTION_NUM*i+ACTION_NUM; j++)
        {
            fin >> temp;
            fin >> temp;
            fin >> temp;
            fin >> temp;
            policy[j] = temp;         
        }
        fin >> temp;
        fin >> temp;
        fin >> temp;
    }
    fin.close();

    // for debug use
    cout<<"Level 2 policy sanity check: "<<endl;
    for(int j = 0; j < 15; j++)
    {
        for(int i = 0; i < ACTION_NUM-1; ++i){
            cout<<policy[ACTION_NUM*j+i]<<", ";
        }
        cout<<policy[ACTION_NUM*(j+1)-1]<<endl;
    }

    isPolicy_assigned = true;
    cout<<"Level 2 policy loading-> "<< (isPolicy_assigned ? "Success!" : "Fail!") <<endl;
}
