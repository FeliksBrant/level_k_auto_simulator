/***************************************************************************

    file                 : driver.h
    created              : 2018-01-10 00:31:19 UTC
    author               : Guankun Su
    email                : guanksu@umich.edu
    modified time        : 2018-01-13 04:12:02
    modified by          : guanksu

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _DRIVER_H_
#define _DRIVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include "LevelKDriver.h"

class Driver: public LevelKDriver{
	public:
		Driver(int index);

	private:
    	// Class constants.
	    static const int   LEVEL;

};

#endif // _DRIVER_H_

