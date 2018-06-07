/***************************************************************************

    file                 : driver.cpp
    created              : 2018-01-10 00:31:19 UTC
    author               : Guankun Su
    email                : guanksu@umich.edu
    modified time        : 2018-02-02 16:56:31
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

#include "driver.h"

const int   Driver::LEVEL = 0;

Driver::Driver(int index): LevelKDriver(index,LEVEL){
}


