/******************************************************************************
** behavior.h
** Header file for CarBehaviorState class and FSM classes
******************************************************************************/
#ifndef _CAR_BEHAVIOR_H_
#define _CAR_BEHAVIOR_H_

#include "constants.h"
#include "vehicle.h"
#include "other_vehicles.h"

/*
** define the states in the state machine. This is actually a parameterized state machine where behavior in a state
** depends on the value of parameters.
*/
typedef enum {
    CAR_STATE_KEEP_LANE = 1,
    CAR_STATE_FOLLOW_CAR_AHEAD,
    CAR_STATE_LANE_CHANGE
} tCarState;

string stateNameForState(tCarState stateID);

class CarBehaviorState {
    public:
        tCarState stateID;
        int lane_direction;
        int current_lane;
        int desired_lane;
        double desired_speed;

        CarBehaviorState(tCarState stateID) {
            this->stateID=stateID;
            this->current_lane = 0;
            this->desired_lane = 0;
            this->lane_direction = 0;
            this->desired_speed = SPEED_LIMIT;
        }

        virtual ~CarBehaviorState() {
        }

        string stateName() {
            return stateNameForState(stateID);
        }
};

class CarBehaviorStateMachine {
    public:
        EgoVehicle *car;
        Vehicle *this_lane_car_ahead;
        Vehicle *this_lane_car_behind;
        Vehicle *other_lane_car_ahead;
        Vehicle *other_lane_car_behind;
        int iterationCnt;
        //
        // all the states in the state machine
        //
        vector<CarBehaviorState *> states;
        //
        // current state and state index
        //
        CarBehaviorState *currentState;
        int currentStateIndex;
        bool first_iteration;

        CarBehaviorStateMachine();
        CarBehaviorStateMachine(EgoVehicle *car); 
        virtual ~CarBehaviorStateMachine(); 

        void new_iteration(EgoVehicle *car);
        CarBehaviorState *chooseNextState(OtherVehicleSet *other_vehicle_set, int iteractionCnt);  
        void changeStateTo(tCarState stateID);
        string currentStateName() {
            return currentState->stateName();
        }
        string stateName(tCarState newStateID) {
            return stateNameForState(newStateID);
        }
        bool isStateFollowCarAhead() {
            if (currentState->stateID == CAR_STATE_FOLLOW_CAR_AHEAD) {
                return true;
            } else {
                return false;
            }
        }
        bool isStateLaneChange() {
            if (currentState->stateID == CAR_STATE_LANE_CHANGE) {
                return true;
            } else {
                return false;
            }
        }

    private:
        void clearOtherCars();
        bool get_vehicle_ahead(OtherVehicleSet *other_vehicle_set); 
        bool get_vehicle_behind(OtherVehicleSet *other_vehicle_set); 
        bool get_other_lane_vehicle_ahead(OtherVehicleSet *other_vehicle_set,int lane_of_interest); 
        bool get_other_lane_vehicle_behind(OtherVehicleSet *other_vehicle_set,int lane_of_interest); 
        double get_distance_from_vehicle_ahead(Vehicle *vehicle);
        double get_distance_to_vehicle_behind(Vehicle *vehicle);
        double get_distance_from_vehicle(Vehicle *vehicle);
        Vehicle *get_vehicle_ahead_in_lane(OtherVehicleSet *other_vehicle_set, int lane_of_interest);
        Vehicle *get_vehicle_behind_in_lane(OtherVehicleSet *other_vehicle_set, int lane_of_interest);
        bool shouldChangeToLeftLane(OtherVehicleSet *other_vehicle_set); 
        bool shouldChangeToRightLane(OtherVehicleSet *other_vehicle_set); 
        bool isOtherLaneCarBehindInTheWay();
        bool isOtherLaneCarAheadInTheWay();
        bool isOtherLaneCarAheadGoingFastEnough();
        bool wantToChangeToLane(int new_lane);
        bool shouldChangeToLane(int new_lane,OtherVehicleSet *other_vehicle_set);
        void handle_drive_forward_behavior(OtherVehicleSet *other_vehicle_set);
        void handle_change_lane_behavior(OtherVehicleSet *other_vehicle_set, int desired_lane);
        bool finishedLaneChange();
        bool isAnyCarTooClose(OtherVehicleSet *other_vehicle_set, int lane_of_interest);
};

#endif
