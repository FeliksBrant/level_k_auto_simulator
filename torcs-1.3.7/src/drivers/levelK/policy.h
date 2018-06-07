/***************************************************************************

    file                 : policy.h
    created              : Thu Nov 2 16:20:00 EST 2017
    copyright            : Guankun Su
    email                : guanksu@umich.edu
    version              : $Id: policy.h,v 0.1 2017/11/02 16:20:00 gunaksu Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _POLICY_H_
#define _POLICY_H_

#include <string>


// base policy class for each level policy
class Policy {
public:
    Policy(int level_): level(level_){};
    virtual ~Policy(){};

    // EFFECT: return policy pointer for level-k and action for level-0
    virtual void getLKPol(int observation, float** pol) const {};
    virtual int  getL0Pol(int observation) const {return 1;};

    static const long int MAX_POLICY_NUM;
    static const int MAX_OBSERVATION_NUM;
    static const int ACTION_NUM;
private:
    virtual void assign_policy(){};

    int level;
};

// level-0 policy class
class L0_Policy : public Policy{
public:
    L0_Policy():Policy(0){};

    virtual int getL0Pol(int observation) const;

private:
};

// level-1 policy class
class L1_Policy : public Policy{
public:
    L1_Policy();
    virtual ~L1_Policy(){};

    virtual void getLKPol(int observation, float** pol) const;

private:
    virtual void assign_policy();

    static float policy[1240029];
    static bool isPolicy_assigned;
    static const std::string policy_file;
};

// level-2 policy class
class L2_Policy : public Policy{
public:
    L2_Policy();
    virtual ~L2_Policy(){};

    virtual void getLKPol(int observation, float** pol) const;

private:
    virtual void assign_policy();

    static float policy[1240029];
    static bool isPolicy_assigned;
    static const std::string policy_file;
};


#endif // _POLICY_H_