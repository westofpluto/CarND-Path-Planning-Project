/***************************************************************************************
** vehicle.h
** class Vehicle represents a vehicle, either the "self" car or other cars on the road
***************************************************************************************/
#ifndef _VEHICLE_H
#define _VEHICLE_H

#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "constants.h"

using namespace std;

class Vehicle {
    public:
        //
        // these are the state values used to control trajectory. They are the values at the end of the prior trajectory sent to simulator
        //
        //
        double x;
        double y;
        double yaw;
        double s;
        double d;
        double speed;

        double t;    // time at which the above values hold, =0.0 for first point in a trajectory

        int carID;
        int lane;

        double accel;
        double max_acceleration;

        double x_now;
        double y_now;
        double yaw_now;
        double s_now;
        double d_now;
        double speed_now;

        /**
        * Constructor
        */
        Vehicle();

        /**
        * Destructor
        */
        virtual ~Vehicle();

        Vehicle *newCopy();

        void localize_from_sensors(vector<double> &car_sensor_data, double dt_prop, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

        int get_lane();
        int lane_on_left() {
            return get_lane() + LANE_CHANGE_DIRECTION_LEFT;
        }
        int lane_on_right() {
            return get_lane() + LANE_CHANGE_DIRECTION_RIGHT;
        }
        bool is_valid_lane(int lane) {
            if (lane >=0 && lane <=2) {
                return true;
            } else {
                return false;
            }
        }
        double get_lane_center();

        /*
        ** use these later?
        void increment(int dt);

        float position_at(int t);

        vector<Vehicle> generate_predictions(int horizon=2);
        */
        void printMe(); 
};

class EgoVehicle : public Vehicle {
    public:
        /**
        * Constructor
        */
        EgoVehicle();

        /**
        * Destructor
        */
        virtual ~EgoVehicle();

        EgoVehicle copyMe(); 

        void localize_self(map<string,double> &data);
        EgoVehicle move_forward(double tmove);    
        bool isTooCloseTo(Vehicle *other_car_ahead, double t, double *final_min_dist);
};

/*
** A VehicleTrajectory is a list of Vehicle objects. This class is used for other (non-ego) vhicles. 
** The first item in the list is at t=0.0. Each Vehicle state after that has a time value > 0.0. 
** For some functions that generate a trajectory (like the Behavior planner) we are only guessing at the final
** time. To handle this, we set time_scale. This defaults to 1.0 which means the time values in each Vehicle state
** are accurate. If it is set to any other value, then each of the time values must be multiplied by time_scale
** to get actual clock time
*/
class VehicleTrajectory {
public:
    vector<Vehicle> vehicle_states;
    double time_scale;

    /**
    * Constructor
    */
    VehicleTrajectory() {}

    /**
    * Destructor
    */
    virtual ~VehicleTrajectory() {}

    void add(Vehicle v) {
        vehicle_states.push_back(v);
    }

    Vehicle start() {
        return vehicle_states[0];
    }
};

/*
** This is just like VehicleTrajectory but it is used for the ego Vehicle
*/
class EgoVehicleTrajectory {
public:
    vector<EgoVehicle> vehicle_states;
    double time_scale;

    /**
    * Constructor
    */
    EgoVehicleTrajectory() {}

    /**
    * Destructor
    */
    virtual ~EgoVehicleTrajectory() {}

    void add(EgoVehicle v) {
        vehicle_states.push_back(v);
    }

    EgoVehicle start() {
        return vehicle_states[0];
    }
};

 
#endif

