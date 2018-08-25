/******************************************************************************
** behavior.cpp
** methods for CarBehaviorState class and FSM classes
******************************************************************************/
#include <vector>
#include "behavior.h"
#include "utils.hpp"

using namespace std;

string stateNameForState(tCarState stateID) {
    if (stateID==CAR_STATE_KEEP_LANE) {
        string str1 = "CAR_STATE_KEEP_LANE";
        return str1;
    } else if (stateID==CAR_STATE_FOLLOW_CAR_AHEAD) {
        string str2 = "CAR_STATE_FOLLOW_CAR_AHEAD";
        return str2;
    } else if (stateID==CAR_STATE_LANE_CHANGE) {
        string str3 = "CAR_STATE_LANE_CHANGE";
        return str3;
    } else {
        string s="UNKNOWN";
        return s;
    }
}

/*
** We define the structure of our Behavior Planning state machine in the constructor
*/
CarBehaviorStateMachine::CarBehaviorStateMachine() {
    this->car=NULL;
    this->this_lane_car_ahead=NULL;
    this->this_lane_car_behind=NULL;
    this->other_lane_car_ahead=NULL;
    this->other_lane_car_behind=NULL;
    this->iterationCnt=0;

    CarBehaviorState *state_keep_lane = new CarBehaviorState(CAR_STATE_KEEP_LANE);
    states.push_back(state_keep_lane);

    CarBehaviorState *state_follow = new CarBehaviorState(CAR_STATE_FOLLOW_CAR_AHEAD);
    states.push_back(state_follow);
    
    CarBehaviorState *state_lane_change = new CarBehaviorState(CAR_STATE_LANE_CHANGE);
    states.push_back(state_lane_change);

    int current_lane = 0;
    int desired_lane = current_lane;
    currentState = state_keep_lane;
    currentState->current_lane=current_lane;
    currentState->desired_lane=desired_lane;
    currentStateIndex=0;

    first_iteration = true;
}

CarBehaviorStateMachine::CarBehaviorStateMachine(EgoVehicle *car) {
    this->car=car;
    this->this_lane_car_ahead=NULL;
    this->this_lane_car_behind=NULL;
    this->other_lane_car_ahead=NULL;
    this->other_lane_car_behind=NULL;

    CarBehaviorState *state_keep_lane = new CarBehaviorState(CAR_STATE_KEEP_LANE);
    states.push_back(state_keep_lane);

    CarBehaviorState *state_follow = new CarBehaviorState(CAR_STATE_FOLLOW_CAR_AHEAD);
    states.push_back(state_follow);
    
    CarBehaviorState *state_lane_change = new CarBehaviorState(CAR_STATE_LANE_CHANGE);
    states.push_back(state_lane_change);

    int current_lane = 0;
    if (car!=NULL) {
        current_lane = car->get_lane();
    }
    int desired_lane = current_lane;
    currentState = state_keep_lane;
    currentState->current_lane=current_lane;
    currentState->desired_lane=desired_lane;
    currentStateIndex=0;

    first_iteration = true;
}

CarBehaviorStateMachine::~CarBehaviorStateMachine() {
    this->car=NULL;

    clearOtherCars();

    int i=0;
    int n=states.size();
    for (i=0;i<n;i++) {
        CarBehaviorState *state=states[i];
        delete state;
    }
    states.clear();
}

void CarBehaviorStateMachine::clearOtherCars() {
    if (this->this_lane_car_ahead) {
        delete this->this_lane_car_ahead;
        this->this_lane_car_ahead=NULL;
    }
    if (this->this_lane_car_behind) {
        delete this->this_lane_car_behind;
        this->this_lane_car_behind=NULL;
    }
    if (this->other_lane_car_ahead) {
        delete this->other_lane_car_ahead;
        this->other_lane_car_ahead=NULL;
    }
    if (this->other_lane_car_behind) {
        delete this->other_lane_car_behind;
        this->other_lane_car_behind=NULL;
    }
}
 
void CarBehaviorStateMachine::changeStateTo(tCarState stateID) {
    int idx= (int)stateID-1;
    cout << "iteration: " << iterationCnt << ", Changing state from " << currentStateName() << " to " << stateName(stateID) << endl;
    currentStateIndex = idx;
    currentState = states[idx];
}

void CarBehaviorStateMachine::new_iteration(EgoVehicle *car) {
    this->car=car;
    this->clearOtherCars();

    if (first_iteration) {
        int current_lane = car->get_lane();
        int desired_lane = current_lane;
        currentState->current_lane=current_lane;
        currentState->desired_lane=desired_lane;
        first_iteration = false;
    }

}


/***********************************************************************************
** Here is the actual state machine logic:
** If the car is in state CAR_STATE_KEEP_LANE:
**    project ahead and see if staying in this state would result in
**    getting too close to the car ahead of us. If so then switch to 
**    state CAR_STATE_FOLLOW_CAR_AHEAD. Otherwise, just stay in this lane.
** Else if the car is in state CAR_STATE_FOLLOW_CAR_AHEAD:
**    Match our speed to the car ahead speed. Also look for an opportunity to
**    change lanes. An opportunity to change lanes is when any of these conditions hold:
**       1. the other lane has no car ahead or behind, OR
**       2. the other lane has no car ahead, and the car behind will not smash
**             into us if we change lanes in front of it, OR
**       3. the other lane has a car ahead that is going faster than the car 
**             ahead in our lane, and the other lane car ahead is even or farther
**             ahead than the car in fron of us, and the car behind in the other lane
**             will not smash into us if we change lanes in front of it.
**    If at this moment there is an opportunity to change lanes, then switch to
**    state CAR_STATE_LANE_CHANGE.
** Else if the car is in state CAR_STATE_LANE_CHANGE:
**    If the lane change is not yet completed, then stay in this state
**    If the lane change is completed, then look at speed and position of new
**    car ahead (if any). If no car ahead or the car ahead is sufficiently far and 
**    fast enough so we won't crash into it, then switch to state CAR_STATE_KEEP_LANE.
**    But if the car ahead of us is such that staying in this state would result in
**    getting too close to the car ahead of us, then switch to
**    state CAR_STATE_FOLLOW_CAR_AHEAD.
***********************************************************************************/
CarBehaviorState *CarBehaviorStateMachine::chooseNextState(OtherVehicleSet *other_vehicle_set,int iterationCnt) {
    this->iterationCnt=iterationCnt;
    double prev_desired_speed=currentState->desired_speed;
    if (currentState->stateID==CAR_STATE_KEEP_LANE) {
        handle_drive_forward_behavior(other_vehicle_set); 
        return currentState;
     
    } else if (currentState->stateID==CAR_STATE_FOLLOW_CAR_AHEAD) {
        //
        // continue to drive forward, following car ahead
        //
        handle_drive_forward_behavior(other_vehicle_set); 
        // 
        // we are following the (slower) car ahead and we want to look for lane change opportunities
        //
        if (shouldChangeToLeftLane(other_vehicle_set)) {
            changeStateTo(CAR_STATE_LANE_CHANGE);
            currentState->lane_direction = LANE_CHANGE_DIRECTION_LEFT;
            int lane=car->get_lane();
            currentState->current_lane=lane;
            currentState->desired_lane=lane + LANE_CHANGE_DIRECTION_LEFT;
            currentState->desired_speed = prev_desired_speed;
            std::cout << "New desired lane is " << currentState->desired_lane << std::endl;
        } else if (shouldChangeToRightLane(other_vehicle_set)) {
            changeStateTo(CAR_STATE_LANE_CHANGE);
            currentState->lane_direction = LANE_CHANGE_DIRECTION_RIGHT;
            int lane=car->get_lane();
            currentState->current_lane=lane;
            currentState->desired_lane=lane + LANE_CHANGE_DIRECTION_RIGHT;
            currentState->desired_speed = prev_desired_speed;
            std::cout << "New desired lane is " << currentState->desired_lane << std::endl;
        } else {
        }
        return currentState;
    } else if (currentState->stateID==CAR_STATE_LANE_CHANGE) {

        if (finishedLaneChange()) {
            //
            // we finished the lane change so lets see if we can drive at the speed limit in this new lane
            // or if we have to slow down for the car ahead of us in this lane
            //
            std::cout << "FINISHED lane change to lane " << currentState->desired_lane << std::endl;
            changeStateTo(CAR_STATE_KEEP_LANE);
            int lane=car->get_lane();
            currentState->current_lane=lane;
            currentState->desired_lane=lane;
            currentState->desired_speed = prev_desired_speed;
            handle_drive_forward_behavior(other_vehicle_set); 
        } else {
            std::cout << "NOT finished lane change to lane " << currentState->desired_lane << std::endl;
            handle_change_lane_behavior(other_vehicle_set,currentState->desired_lane);
        }
        return currentState;
    } else {
        // should never happen!
        return NULL;
    }
}

bool CarBehaviorStateMachine::finishedLaneChange() {
    int desired_lane = currentState->desired_lane;
    double desired_d = get_lane_center_for_lane(desired_lane);
    if (abs(car->d - desired_d) <= LANE_CHANGE_FINISHED_BUFFER) {
        return true;
    } else {
        return false;
    }
}

/*
** drive forward at desired speed but see if we have to slow down (and change state) becuase of a slow car in front of us
*/
void CarBehaviorStateMachine::handle_drive_forward_behavior(OtherVehicleSet *other_vehicle_set) { 
    bool bCarAhead = get_vehicle_ahead(other_vehicle_set);
    bool bCarBehind = get_vehicle_ahead(other_vehicle_set);
    double prev_desired_speed = currentState->desired_speed;
    double final_min_dist=0.0;
    if (bCarAhead) {
        //std::cout << "car ahead detected, my s is " << car->s << " other car speed is " << this_lane_car_ahead->speed << std::endl;
        if (car->isTooCloseTo(this_lane_car_ahead,TIME_MOVE_FORWARD,&final_min_dist)) {
            //
            // if we keep driving ahead at this speed, we will get too close to the car ahead.
            // So, we slow down and look for lane change opportunities. Since we will be slowing
            // down over a 2 second period, we find a constant deceleration over this time
            //
            //std::cout << "YES too close" << std::endl;
            double des_speed = this_lane_car_ahead->speed;
            if (final_min_dist < MIN_SAFE_FRONT_DIST) {
                double diff_dist = MIN_SAFE_FRONT_DIST - final_min_dist;
                double dv_reduction = diff_dist/TIME_MOVE_FORWARD;
                des_speed -=dv_reduction;
            }
            double desired_speed = ALPHA*prev_desired_speed + (1.0-ALPHA)*des_speed;  
            double speed_diff = car->speed - desired_speed;
            car->accel = -(speed_diff/TIME_MOVE_FORWARD);
            //std::cout << "car speed is " << car->speed << std::endl;

            if (!isStateFollowCarAhead()) {
                changeStateTo(CAR_STATE_FOLLOW_CAR_AHEAD);
                int lane=car->get_lane();
                currentState->current_lane=lane;
                currentState->desired_lane=lane;
            }
            currentState->desired_speed = desired_speed;

        } else {
            //std::cout << "NOT too close" << std::endl;

            double desired_speed = ALPHA*prev_desired_speed + (1.0-ALPHA)*SPEED_LIMIT;  
            double speed_diff = desired_speed - car->speed;
            car->accel = speed_diff/TIME_MOVE_FORWARD;
            currentState->desired_speed = desired_speed;
        }
    } else {
        double desired_speed = ALPHA*prev_desired_speed + (1.0-ALPHA)*SPEED_LIMIT;  
        double speed_diff = desired_speed - car->speed;
        car->accel = speed_diff/TIME_MOVE_FORWARD;
        currentState->desired_speed = desired_speed;
    }
    //std::cout << "End of handle_drive_forward_behavior, desired_speed is " << currentState->desired_speed << std::endl;
}

void CarBehaviorStateMachine::handle_change_lane_behavior(OtherVehicleSet *other_vehicle_set, int desired_lane) { 
    bool bCarAhead = get_vehicle_ahead(other_vehicle_set);
    bool bCarBehind = get_vehicle_ahead(other_vehicle_set);
    bool bOtherLaneCarAhead = get_other_lane_vehicle_ahead(other_vehicle_set, desired_lane);
    bool bOtherLaneCarBehind = get_other_lane_vehicle_behind(other_vehicle_set, desired_lane);

    double prev_desired_speed = currentState->desired_speed;
    double final_min_dist=0.0;
    Vehicle *car_ahead=NULL;
    if (car->get_lane() == desired_lane) {
       car_ahead = other_lane_car_ahead; 
    } else {
       car_ahead = this_lane_car_ahead;
    }
    if (car_ahead) {
        std::cout << "Car ahead right now" << std::endl;
        if (car->isTooCloseTo(car_ahead,TIME_MOVE_FORWARD,&final_min_dist)) {
            //
            // if we keep driving ahead at this speed, we will get too close to the car ahead.
            //
            std::cout << "YES too close" << std::endl;
            double des_speed = car_ahead->speed;
            if (final_min_dist < MIN_SAFE_FRONT_DIST) {
                double diff_dist = MIN_SAFE_FRONT_DIST - final_min_dist;
                double dv_reduction = diff_dist/TIME_MOVE_FORWARD;
                des_speed -=dv_reduction;
            }
            double desired_speed = ALPHA*prev_desired_speed + (1.0-ALPHA)*des_speed;  
            double speed_diff = car->speed - desired_speed;
            car->accel = -(speed_diff/TIME_MOVE_FORWARD);
            //std::cout << "car speed is " << car->speed << std::endl;

            currentState->desired_speed = desired_speed;

        } else {
            std::cout << "NOT too close" << std::endl;

            double desired_speed = ALPHA*prev_desired_speed + (1.0-ALPHA)*SPEED_LIMIT;  
            double speed_diff = desired_speed - car->speed;
            car->accel = speed_diff/TIME_MOVE_FORWARD;
            currentState->desired_speed = desired_speed;
        }

    } else {
        std::cout << "No car ahead right now" << std::endl;
        double desired_speed = ALPHA*prev_desired_speed + (1.0-ALPHA)*SPEED_LIMIT;  
        double speed_diff = desired_speed - car->speed;
        car->accel = speed_diff/TIME_MOVE_FORWARD;
        currentState->desired_speed = desired_speed;
    }

}

bool CarBehaviorStateMachine::get_vehicle_ahead(OtherVehicleSet *other_vehicle_set) {
    int lane_of_interest = car->get_lane();
    this->this_lane_car_ahead = get_vehicle_ahead_in_lane(other_vehicle_set, lane_of_interest);
    return (this->this_lane_car_ahead != NULL); 
}

bool CarBehaviorStateMachine::get_vehicle_behind(OtherVehicleSet *other_vehicle_set) {
    int lane_of_interest = car->get_lane();
    this->this_lane_car_behind = get_vehicle_behind_in_lane(other_vehicle_set, lane_of_interest);
    return (this->this_lane_car_behind != NULL); 
}

bool CarBehaviorStateMachine::get_other_lane_vehicle_ahead(OtherVehicleSet *other_vehicle_set,int lane_of_interest) {
    this->other_lane_car_ahead = get_vehicle_ahead_in_lane(other_vehicle_set, lane_of_interest);
    return (this->other_lane_car_ahead != NULL);
}

bool CarBehaviorStateMachine::get_other_lane_vehicle_behind(OtherVehicleSet *other_vehicle_set,int lane_of_interest) {
    this->other_lane_car_behind = get_vehicle_behind_in_lane(other_vehicle_set, lane_of_interest);
    return (this->other_lane_car_behind != NULL);
}

//
// utility function that returns the distance from the vehicle ahead of us to us.
// The distance is always > 0. If the other vehicle is behind us, we remember that
// the track is a a big closed loop so in that case we add the track length to it
//
double CarBehaviorStateMachine::get_distance_from_vehicle_ahead(Vehicle *vehicle) {
    double diff_s = vehicle->s - this->car->s;
    if (diff_s < 0.0) {
        diff_s += MAX_TRACK_S;
    }
    return diff_s;
}

double CarBehaviorStateMachine::get_distance_from_vehicle(Vehicle *vehicle) {
    double diff_s = abs(vehicle->s - this->car->s);
    if (diff_s > MAX_TRACK_S/2.0) {
        diff_s = MAX_TRACK_S - diff_s;
    }
    return diff_s;
}

//
// utility function that returns the distance from us to the vehicle behind us.
// The distance is always > 0. If the other vehicle is ahead us, we remember that
// the track is a a big closed loop so in that case we add the track length to it
//
double CarBehaviorStateMachine::get_distance_to_vehicle_behind(Vehicle *vehicle) {
    double diff_s = this->car->s - vehicle->s;
    if (diff_s < 0.0) {
        diff_s += MAX_TRACK_S;
    }
    return diff_s;
}

//
// Returns vehicle ahead if a vehicle is found ahead of the current vehicle, NULL otherwise. 
//
Vehicle *CarBehaviorStateMachine::get_vehicle_ahead_in_lane(OtherVehicleSet *other_vehicle_set, int lane_of_interest) {
    bool found_vehicle = false;
    double min_diff_s = 0.0;           
    Vehicle temp_vehicle_obj;
    Vehicle *temp_vehicle=NULL;
    Vehicle front_vehicle;
    //
    // Loop over all the other vehicles and only look at those vehicles in the lane of interest
    //
    for (map<int, VehicleTrajectory>::iterator it = other_vehicle_set->trajectories.begin(); it != other_vehicle_set->trajectories.end(); ++it) {
        VehicleTrajectory other_trajectory = it->second;
        temp_vehicle_obj = other_trajectory.start();
        temp_vehicle = &temp_vehicle_obj;
        //std::cout << "get_vehicle_ahead_in_lane: looking at vehicle " << temp_vehicle->carID << std::endl;
        if (temp_vehicle->lane == lane_of_interest) {
            //std::cout << "    Found in lane of interest: " << lane_of_interest << std::endl;
            //
            // found a vehicle that is in the lane of interest
            //
            if (!found_vehicle) {
                //
                // This is the first vehicle we have found in the lane so for now it is the closest vehicle
                // Remember that the track is a big loop, so a car slightly behind us is also almost one loop
                // ahead of us
                //
                min_diff_s = get_distance_from_vehicle_ahead(temp_vehicle); 
                front_vehicle = temp_vehicle_obj;
                found_vehicle=true;
            } else {
                //
                // we already found at at least one vehicle in the lane of interest, now lets see if this new vehicle
                // is in front of us and closer than whatever closest vehicle we found before
                //
                double diff_s = get_distance_from_vehicle_ahead(temp_vehicle); 
                if (diff_s < min_diff_s) {
                    min_diff_s = diff_s;
                    front_vehicle = temp_vehicle_obj;
                }
            }
        } else {
            //std::cout << "    Not in lane of interest" << std::endl;
        }
    }
    //
    // so at this point we have (in front_vehicle) whatever vehicle is closest to us in front
    // now we make sure that this distance is less than some value, otherwise we don't care
    // about it
    //
    if (found_vehicle && min_diff_s <= MAX_FRONT_DIST_WE_CARE_ABOUT) {
        //std::cout << "found vehicle ahead of us in lane " << lane_of_interest << ", dist is " << min_diff_s << std::endl;
        //front_vehicle.printMe();
        //std::cout << "Printing ret vehicle " << std::endl;
        Vehicle *ret = front_vehicle.newCopy();
        //ret->printMe();
        return ret;
    } else {
        return NULL;
    }
}

//
// Returns vehicle behind if a vehicle is found behind the current vehicle, NULL otherwise. 
//
Vehicle *CarBehaviorStateMachine::get_vehicle_behind_in_lane(OtherVehicleSet *other_vehicle_set, int lane_of_interest) {
    bool found_vehicle = false;
    double min_diff_s = 0.0;           
    Vehicle temp_vehicle_obj;
    Vehicle *temp_vehicle=NULL;
    Vehicle back_vehicle;
    //
    // Loop over all the other vehicles and only look at those vehicles in the lane of interest
    //
    for (map<int, VehicleTrajectory>::iterator it = other_vehicle_set->trajectories.begin(); it != other_vehicle_set->trajectories.end(); ++it) {
        VehicleTrajectory other_trajectory = it->second;
        temp_vehicle_obj = other_trajectory.start();
        temp_vehicle = &temp_vehicle_obj;
        if (temp_vehicle->lane == lane_of_interest) {
            //
            // found a vehicle that is in the lane of interest
            //
            if (!found_vehicle) {
                //
                // This is the first vehicle we have found in the lane so for now it is the closest vehicle
                // Remember that the track is a big loop, so a car slightly ahead us is also almost one loop
                // behind us
                //
                min_diff_s = get_distance_to_vehicle_behind(temp_vehicle); 
                back_vehicle = temp_vehicle_obj;
                found_vehicle=true;
            } else {
                //
                // we already found at at least one vehicle in the lane of interest, now lets see if this new vehicle
                // is in back of us and closer than whatever closest vehicle we found before
                //
                double diff_s = get_distance_to_vehicle_behind(temp_vehicle); 
                if (diff_s < min_diff_s) {
                    min_diff_s = diff_s;
                    back_vehicle = temp_vehicle_obj;
                }
            }
        }
    }
    //
    // so at this point we have (in back_vehicle) whatever vehicle is closest to us in back 
    // now we make sure that this distance is less than some value, otherwise we don't care
    // about it
    //
    if (min_diff_s <= MAX_BACK_DIST_WE_CARE_ABOUT) {
        return back_vehicle.newCopy();
    } else {
        return NULL;
    }
}

//
// isAnyCarTooClose    
//
bool CarBehaviorStateMachine::isAnyCarTooClose(OtherVehicleSet *other_vehicle_set, int lane_of_interest) {
    bool found_vehicle = false;
    double diff_s = 0.0;           
    Vehicle temp_vehicle_obj;
    Vehicle *temp_vehicle=NULL;
    //
    // Loop over all the other vehicles and only look at those vehicles in the lane of interest
    //
    for (map<int, VehicleTrajectory>::iterator it = other_vehicle_set->trajectories.begin(); it != other_vehicle_set->trajectories.end(); ++it) {
        VehicleTrajectory other_trajectory = it->second;
        temp_vehicle_obj = other_trajectory.start();
        temp_vehicle = &temp_vehicle_obj;
        //std::cout << "get_vehicle_ahead_in_lane: looking at vehicle " << temp_vehicle->carID << std::endl;
        if (temp_vehicle->lane == lane_of_interest) {
            //std::cout << "    Found in lane of interest: " << lane_of_interest << std::endl;
            //
            // found a vehicle that is in the lane of interest
            //
            diff_s = get_distance_from_vehicle(temp_vehicle); 
            if (diff_s < MIN_SAFE_DIST_ANY) {
                std::cout << "isAnyCarTooClose: initial distance " << diff_s << " too close to " << temp_vehicle->carID << std::endl;
                return true;
            }
            double speed_diff = abs(car->speed - temp_vehicle->speed);
            double final_diff_s = diff_s - speed_diff*TIME_MOVE_FORWARD; 
            if (final_diff_s < MIN_SAFE_DIST_ANY) {
                std::cout << "isAnyCarTooClose: final distance " << final_diff_s << " too close to " << temp_vehicle->carID << std::endl;
                return true;
            }
        }
    }
    return false;
}


bool CarBehaviorStateMachine::isOtherLaneCarBehindInTheWay() {
    if (!other_lane_car_behind) return false;

    double other_car_s0 = other_lane_car_behind->s;
    double other_car_speed = other_lane_car_behind->speed;
    double dist0 = car->s - other_car_s0;
    if (dist0 < 0.0) {
        dist0 += MAX_TRACK_S;
    }
    if (dist0 < MIN_SAFE_BACK_DIST) {
        return true;
    }   
    double dist1 = dist0 + (car->speed - other_car_speed)*LANE_CHANGE_EST_TIME;
    if (dist1 < MIN_SAFE_BACK_DIST) {
        return true;
    } 
    //
    // no problems so return false
    //
    return false;
}

bool CarBehaviorStateMachine::isOtherLaneCarAheadInTheWay() {
    if (!other_lane_car_ahead) return false;

    double other_car_s0 = other_lane_car_ahead->s;
    double other_car_speed = other_lane_car_ahead->speed;
    double dist0 = other_car_s0 - car->s;
    if (dist0 < 0.0) {
        dist0 += MAX_TRACK_S;
    }
    if (dist0 < MIN_SAFE_FRONT_DIST) {
        return true;
    }   
    double dist1 = dist0 + (other_car_speed - car->speed)*LANE_CHANGE_EST_TIME;
    if (dist1 < MIN_SAFE_FRONT_DIST) {
        return true;
    } 
    //
    // no problems so return false
    //
    return false;

}

bool CarBehaviorStateMachine::isOtherLaneCarAheadGoingFastEnough() {
    if (this_lane_car_ahead != NULL && other_lane_car_ahead != NULL) {
        if (other_lane_car_ahead->speed > (this_lane_car_ahead->speed + PASSING_SPEED_BUFFER)) {
            return true;
        }
    }
    return false;
}

bool CarBehaviorStateMachine::wantToChangeToLane(int new_lane) {
    if (!this_lane_car_ahead) {
        // no car ahead in current lane, no reason to change lanes
        return false;
    } else if (this_lane_car_ahead->speed < SPEED_LIMIT) {
        if (!other_lane_car_ahead || isOtherLaneCarAheadGoingFastEnough()) {
            return true;
        } else {
            return false;
        }
    }
    return false;
}

bool CarBehaviorStateMachine::shouldChangeToLeftLane(OtherVehicleSet *other_vehicle_set) {
    return shouldChangeToLane(car->lane_on_left(),other_vehicle_set);
}

bool CarBehaviorStateMachine::shouldChangeToRightLane(OtherVehicleSet *other_vehicle_set) {
    return shouldChangeToLane(car->lane_on_right(),other_vehicle_set);
}

bool CarBehaviorStateMachine::shouldChangeToLane(int new_lane, OtherVehicleSet *other_vehicle_set) {

    if (new_lane < 0 || new_lane > 2) {
        return false;
    } 
    //
    // lane is at least valid so let's see if we can squeeze in
    //
    if (isAnyCarTooClose(other_vehicle_set,new_lane)) {
        std::cout << "shouldChangeToLane: " << new_lane << ", some car is too close in this lane, return false" << std::endl;
        return false;
    }
    bool bOtherLaneCarAhead = get_other_lane_vehicle_ahead(other_vehicle_set, new_lane);
    bool bOtherLaneCarBehind = get_other_lane_vehicle_behind(other_vehicle_set, new_lane);

    //
    // Do we even want to change to this lane? The car ahead must be going slower than the speed limit
    // and the car ahead in the new lane must be going sufficiently faster than that for us to want
    // to change into this lane
    //
    if (!wantToChangeToLane(new_lane)) {
        return false;
    }
    //
    // If no cars ahead or behind in this lane then we can change lanes
    //
    if (!bOtherLaneCarAhead && !bOtherLaneCarBehind) {
        std::cout << "shouldChangeToLane: " << new_lane << ", no car ahead or behind, return true" << std::endl;
        return true;
    }
    //
    // If there is no car ahead in that lane, but there is a car behind, then check to see if that
    // car behind is in our way (or will be in our way) if we try to change lanes in front of it
    //
    if (!bOtherLaneCarAhead) { 
        if (!isOtherLaneCarBehindInTheWay()) {
            std::cout << "shouldChangeToLane: " << new_lane << ", no car ahead and car behind is fine, return true" << std::endl;
            return true;
        } else {
            std::cout << "shouldChangeToLane: " << new_lane << ", no car ahead but car behind is in the way, return false" << std::endl;
            return false;
        }
    }

    //
    // Finally, if there is a car ahead in that lane and a car behind in that lane, then we must perform several checks to
    // see if we can squeeze in
    //
    if (!isOtherLaneCarBehindInTheWay() && !isOtherLaneCarAheadInTheWay()) {
        std::cout << "shouldChangeToLane: " << new_lane << "car ahead and car behind, neither is in the way, return true" << std::endl;
        return true;
    } else {
        std::cout << "shouldChangeToLane: " << new_lane << "car ahead and car behind, one is in the way, return false" << std::endl;
        return false;
    }
}
