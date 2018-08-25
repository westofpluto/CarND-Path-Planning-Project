/***************************************************************************************
** constants.h
** Constants used in the path planning system
***************************************************************************************/
#ifndef _PATH_PLANNING_CONSTANTS_H_
#define _PATH_PLANNING_CONSTANTS_H_

constexpr double MAX_TRACK_S = 6945.554;
constexpr double HALF_TRACK_S = (MAX_TRACK_S/2.0);

constexpr double MAX_FRONT_DIST_WE_CARE_ABOUT = 30.0;
constexpr double MAX_BACK_DIST_WE_CARE_ABOUT = 30.0;

constexpr double MIN_SAFE_FRONT_DIST = 25.0;
constexpr double MIN_SAFE_BACK_DIST = 15.0;
constexpr double MIN_SAFE_DIST_ANY = 15.0;

constexpr int OFF_ROAD = -99;
constexpr int SELF_CAR_ID = 0xffff;     // probably any large number would do - any number not likely to be used by another vehicle
constexpr double PREFERRED_BUFFER = 6;

constexpr double MPH_TO_MPS = (1.0/2.23694);

constexpr double SPEED_LIMIT_MPH = 49.0;   // just under 50 MPH speed limit of simulator
constexpr double SPEED_LIMIT = (SPEED_LIMIT_MPH * MPH_TO_MPS);

// to pass, we want the other lane car ahead to be at least 2MPH faster than this lane car ahead
constexpr double PASSING_SPEED_BUFFER = (2.0*MPH_TO_MPS);     

constexpr int LANE_CHANGE_DIRECTION_LEFT     = -1;
constexpr int LANE_CHANGE_DIRECTION_RIGHT    = 1;
constexpr double LANE_CHANGE_FINISHED_BUFFER    = 0.10;

constexpr double TIME_MOVE_FORWARD              = 3.0;
constexpr double LANE_CHANGE_EST_TIME           = 3.0;

constexpr double DELTA_TIME                     = 0.02;
constexpr int NUM_CONTROL_POINTS             = 40;

constexpr double D_CONTROL_FACTOR               = 0.10;
constexpr double D_LANE_CHANGE_CONTROL_FACTOR   = 0.10;

//constexpr double REASONABLE_ACCEL = 0.224;
constexpr double REASONABLE_ACCEL = 0.100;
constexpr double REASONABLE_DECEL = 0.050;

constexpr double ALPHA = 0.90;
#endif

