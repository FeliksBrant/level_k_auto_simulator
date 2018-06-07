/***************************************************************************

    file                 : level1_3.cpp
    created              : Mon Mar 19 17:49:05 EDT 2018
    copyright            : (C) 2002 Guankun Su

 ***************************************************************************/

#ifdef _WIN32
#include <windows.h> 
#endif

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

#include "driver.h"

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static int pitcmd(int index, tCarElt* car, tSituation *s);
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 

#define NBBOTS 10

static const char* botname[NBBOTS] = {
    "level1 30", "level1 31", "level1 32", "level1 33", "level1 34", 
    "level1 35", "level1 36", "level1 37", "level1 38", "level1 39"
};

static Driver *driver[NBBOTS];

/* 
 * Module entry point  
 */ 
extern "C" int 
level1_3(tModInfo *modInfo) 
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    for (int i = 0; i < NBBOTS; i++) {
        modInfo[i].name    = strdup(botname[i]);    // name of the module (short).
        modInfo[i].desc    = strdup("");    // Description of the module (can be long).
        modInfo[i].fctInit = InitFuncPt;            // Init function.
        modInfo[i].gfId    = ROB_IDENT;             // Supported framework version.
        modInfo[i].index   = i;                     // Indices from 0 to 9.
    }
    return 0; 
} 

/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 

    // Create robot instance for index.
    driver[index] = new Driver(index+30);
    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 

                 /* for every track change or new race */ 
    itf->rbNewRace  = newrace;   /* Start a new race */
    itf->rbDrive    = drive;     /* Drive during race */
    itf->rbPitCmd   = pitcmd;
    itf->rbEndRace  = endrace;   /* End of the current race */
    itf->rbShutdown = shutdown;  /* Called before the module is unloaded */
    itf->index      = index;     /* Index used if multiple interfaces */
    return 0; 
} 

/* Called for every track change or new race. */
static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
    driver[index]->initTrack(track, carHandle, carParmHandle, s);
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{ 
    driver[index]->newRace(car, s);
} 

/* Drive during race. */
static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 
    driver[index]->drive(s);
}

/* Pitstop callback. */
static int pitcmd(int index, tCarElt* car, tSituation *s)
{
    return driver[index]->pitCommand(s);
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
    driver[index]->endRace(s);
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
    delete driver[index];
}

