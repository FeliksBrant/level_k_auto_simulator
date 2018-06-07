#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_

// static float lanechange_time = 2.0f; //s
// static float carwidth = 2.0f; //m
// static float lanewidth = 3.6f; //m
// static float min_initial_separation = 21.0f; //m
// static float init_size = 300.0f;//600;//300; //m -- max distance for initialization

// static double min_speed = 80.0/3.6-2.5*2.0; // m/s
// static double max_speed = 80.0/3.6+2.5*2.0; // m/s
// static double nominal_speed = 80.0/3.6; // m/s
// static float speedPID_accel_P = 10.0f;
// static float speedPID_accel_D = 1.0f;
// static float speedPID_accel_I = 0.1f;
// static float speedPID_brake_P = 10.0f;
// static float speedPID_brake_D = 1.0f;
// static float speedPID_brake_I = 0.1f;
// static double speedPID_I_margin = 5.0/3.6;

// static float accel_rate = 0.25f; // m/s^2
// static float decel_rate = 0.25f; // m/s^2
// static float hard_accel_rate = 0.5f; // m/s^2
// static float hard_decel_rate = 0.5f; // m/s^2

// static float new_accel_rate = 2.5f; // m/s^2
// static float new_decel_rate = -2.5f; // m/s^2
// static float new_hard_accel_rate = 5.0f; // m/s^2
// static float new_hard_decel_rate = -5.0f; // m/s^2

// #define actionChangeCountMax 50 //1s to change action
// #define inLaneCountNum 45 // for lane change

// extern float pol1[];
// extern float pol2[];
// extern int isPol_assigned;

// //targetLaneType
// #define TO_LEFT_LANE 1
// #define TO_MIDDLE_LANE 0
// #define TO_RIGHT_LANE -1
// #define Maintain 1
// #define Accelerate 2
// #define Decelerate 3
// #define HardAccelerate 4
// #define HardDecelerate 5
// #define ChangeToLeft 6
// #define ChangeToRight 7

// //observation message types
// static float max_sight_distance = 63;//400;//200; //m -- max distance for detection of other vehicles in front and behind you
// static float close_distance = 21; //m
// static float far_distance = 42; //m

// #define car_close 0 
// #define car_nominal 1 
// #define car_far 2 
// #define car_stable 0 
// #define car_approaching 1 
// #define car_retreating 2 
// static float speed_margin = 2.0/3.6f; // m/s

// #define level_0 0
// #define level_1 1
// #define level_2 2

// // record player data
// static int startRecord = 15; //start recording data at 15s after sim starts
// static int maxDataAmount = 500; // record 500 data point

// // player action space
// static float maintainRange = 0.1f;
// static float accelerateRange = 2.5f;

// // ADS constants
// static double speed_limit_margin = 5/3.6;

#endif // _PARAMETERS_H_