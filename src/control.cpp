/******************************************************************************
** control.cpp
** Functions that control the trajectory for a given state
******************************************************************************/
#include "control.h"
#include "utils.hpp"
#include "spline.h"

void TrajectoryControl::new_iteration(CarBehaviorStateMachine *state_machine, vector<double> &previous_path_x, vector<double> &previous_path_y) {
    this->stm=state_machine;
    this->previous_path_x = previous_path_x;
    this->previous_path_y = previous_path_y;
    //std::cout << "in new_iteration, size of previous_path_x is " << this->previous_path_x.size() << std::endl;
    //std::cout << "in new_iteration, size of previous_path_y is " << this->previous_path_y.size() << std::endl;
    this->next_x_vals.clear();
    this->next_y_vals.clear();
}

void TrajectoryControl::generate_path(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
    int remaining = fill_from_previous();
    if (stm->currentState->stateID==CAR_STATE_KEEP_LANE) {
        //
        // drive forward and move to center of lane
        //
        //std::cout << "CAR_STATE_KEEP_LANE: generate_drive_forward_path" << std::endl;

        generate_drive_forward_path(remaining,SPEED_LIMIT,maps_s, maps_x, maps_y);

    } else if (stm->currentState->stateID==CAR_STATE_FOLLOW_CAR_AHEAD) {
        //
        // drive forward and move to center of lane
        //
        //std::cout << "CAR_STATE_FOLLOW_CAR_AHEAD: generate_drive_forward_path" << std::endl;
       
        generate_drive_forward_path(remaining,stm->currentState->desired_speed, maps_s, maps_x, maps_y);

    } else if (stm->currentState->stateID==CAR_STATE_LANE_CHANGE) {
        //
        // drive forward and move to center of lane
        //
        int desired_lane = stm->currentState->desired_lane;
        double desired_speed = stm->currentState->desired_speed;
        double accel=0.0;
        //
        //std::cout << "CAR_STATE_LANE_CHANGE: generate_lane_change_path" << std::endl;
        generate_lane_change_path(remaining,desired_lane,desired_speed,accel,maps_s, maps_x, maps_y);
    }
}

int TrajectoryControl::fill_from_previous() {
    int prev_size = this->previous_path_x.size();
    //std::cout << "prev_size is " << prev_size << std::endl;
    int remaining = 0;
    if (prev_size < NUM_CONTROL_POINTS) { 
        remaining = NUM_CONTROL_POINTS - prev_size;
        for (int i=0;i<prev_size;i++) {
            double xp=this->previous_path_x[i];
            double yp=this->previous_path_y[i];
            this->next_x_vals.push_back(xp);
            this->next_y_vals.push_back(yp);
        }
    }
    //std::cout << "remaining is " << remaining << std::endl;

    return remaining;
}

void TrajectoryControl::generate_drive_forward_path_bad(int remaining, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
    double a = stm->car->accel;
    double adtsq2 = 0.5*a*DELTA_TIME*DELTA_TIME;
    double v0 = stm->car->speed;
    double s0 = stm->car->s;
    double d0 = stm->car->d;
    double v = v0;
    double s = s0;
    double d = d0;
    int lane=stm->car->get_lane();
    double ctr=stm->car->get_lane_center();
    double dd = ctr - d;
    vector<double> xy = getXY(s, d, maps_s, maps_x, maps_y);
    if (remaining==NUM_CONTROL_POINTS) {
        this->next_x_vals.push_back(xy[0]);
        this->next_y_vals.push_back(xy[1]);
        //std::cout << "car x,y,s,d = " << stm->car->x << ", " << stm->car->y << ", " << stm->car->s << ", " << stm->car->d << ", " << std::endl;
        //std::cout << "0: x,y,s,d = " << xy[0] << ", " << xy[1] << ", " << s << ", " << d << ", " << std::endl;
        for (int i=1; i<remaining;i++) {
            v = v + a*DELTA_TIME;
            s = s + v*DELTA_TIME + adtsq2;
            dd = ctr - d;
            d = d + D_CONTROL_FACTOR*dd;
            xy = getXY(s, d, maps_s, maps_x, maps_y);
            this->next_x_vals.push_back(xy[0]);
            this->next_y_vals.push_back(xy[1]);
            if (i <=5) {
                //std::cout << i << ": x,y,s,d = " << xy[0] << ", " << xy[1] << ", " << s << ", " << d << ", " << std::endl;
            }
        }
    } else {
        for (int i=0; i<remaining;i++) {
            v = v + a*DELTA_TIME;
            s = s + v*DELTA_TIME + adtsq2;
            dd = ctr - d;
            d = d + D_CONTROL_FACTOR*dd;
            xy = getXY(s, d, maps_s, maps_x, maps_y);
            this->next_x_vals.push_back(xy[0]);
            this->next_y_vals.push_back(xy[1]);
            if (i <=5) {
                //std::cout << i << ": x,y,s,d = " << xy[0] << ", " << xy[1] << ", " << s << ", " << d << ", " << std::endl;
            }
        }
    }

    //std::cout << " " << endl;
}

void TrajectoryControl::generate_drive_forward_path(int remaining, double desired_speed, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
    int prev_size = NUM_CONTROL_POINTS - remaining;
    double s = stm->car->s;
    double d = stm->car->d;
    double car_yaw = stm->car->yaw;
    int lane=stm->car->get_lane();

    double desired_d = get_lane_center_for_lane(lane);
    //
    // handle speed control
    //
    double car_speed = stm->car->speed;
    double ref_vel = car_speed;
    if (ref_vel < desired_speed) {
        ref_vel += REASONABLE_ACCEL;
        if (ref_vel > desired_speed) {
            ref_vel = desired_speed;
        }
    } else if (ref_vel > desired_speed) {
        ref_vel -= REASONABLE_DECEL; 
        if (ref_vel < desired_speed) {
            ref_vel = desired_speed;
        }
    }

    double des_speed_mph = desired_speed/MPH_TO_MPS;
    //std::cout << "remaining is " << remaining << std::endl;
    //std::cout << "desired_speed is " << des_speed_mph << " MPH" << std::endl;
    //std::cout << "ref_vel is " << ref_vel << std::endl;

    vector<double> xy = getXY(s, d, maps_s, maps_x, maps_y);
    double car_s=s;
    double car_d=d;
    double car_x=xy[0];
    double car_y=xy[1];
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    vector<double> ptsx;
    vector<double> ptsy;

    if (prev_size < 2) {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);
        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);
        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    } else {
        ref_x = previous_path_x[prev_size-1];
        ref_y = previous_path_y[prev_size-1];
        double ref_x_prev = previous_path_x[prev_size-2];
        double ref_y_prev = previous_path_y[prev_size-2];
        ref_yaw = atan2(ref_y - ref_y_prev,ref_x - ref_x_prev);
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);
        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    //
    // make some widely spaces ref points on which to use spline
    //
    vector<double> next_wp0 = getXY(car_s + 20, desired_d, maps_s,maps_x,maps_y);
    vector<double> next_wp1 = getXY(car_s + 50, desired_d, maps_s,maps_x,maps_y);
    vector<double> next_wp2 = getXY(car_s + 80, desired_d, maps_s,maps_x,maps_y);
    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);
    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i=0; i<ptsx.size(); i++) {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;
        ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
        ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
    }

    //
    // Use spline
    //
    tk::spline spl;
    spl.set_points(ptsx,ptsy);

/***
    for (int i=0; i<previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }
***/
    double target_x = 20.0;
    double target_y = spl(target_x);
    double target_dist = sqrt((target_x*target_x) + (target_y*target_y));
    double x_add_on = 0.0;

    for (int i=1; i <= NUM_CONTROL_POINTS - previous_path_x.size(); i++) {
        double N = (target_dist /(0.02*ref_vel));
        double x_point = x_add_on+(target_x)/N;
        double y_point = spl(x_point);
        x_add_on = x_point;
        double x_ref=x_point;
        double y_ref=y_point;
        x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
        y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
}

//
// generate_lane_change_path: generates a lane change trajectory to the desired lane.
// Note that the forward acceleration is an input. Currently we just use a zero acceleration (constant speed)
// lane change, but we could try some other accel values as well. The lateral d change is done in a closed loop manner,
// i.e., the d error (desired d - current d) is decreased by a fixed factor at each time step
//



void TrajectoryControl::generate_lane_change_path_bad(int remaining, int desired_lane, double desired_speed, double accel, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
    double a = accel;
    double adtsq2 = 0.5*a*DELTA_TIME*DELTA_TIME;
    double v0 = stm->car->speed;
    double s0 = stm->car->s;
    double d0 = stm->car->d;
    double v = v0;
    double s = s0;
    double d = d0;
    int lane=stm->car->get_lane();
    double ctr=get_lane_center_for_lane(desired_lane);
    double dd = ctr - d;
    vector<double> xy = getXY(s, d, maps_s, maps_x, maps_y);
    if (remaining==NUM_CONTROL_POINTS) {
        this->next_x_vals.push_back(xy[0]);
        this->next_y_vals.push_back(xy[1]);
    }
    for (int i=1; i<remaining;i++) {
        v = v + a*DELTA_TIME;
        s = s + v*DELTA_TIME + adtsq2;
        dd = ctr - d;
        d = d + D_LANE_CHANGE_CONTROL_FACTOR*dd;
        xy = getXY(s, d, maps_s, maps_x, maps_y);
        this->next_x_vals.push_back(xy[0]);
        this->next_y_vals.push_back(xy[1]);
    }
}

/*************************************************
** generate_lane_change_path
*************************************************/
void TrajectoryControl::generate_lane_change_path(int remaining, int desired_lane, double desired_speed, double accel, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
    int prev_size = NUM_CONTROL_POINTS - remaining;
    //std::cout << "START generate_lane_change_path " << std::endl;
    double s = stm->car->s;
    double d = stm->car->d;
    double desired_d = get_lane_center_for_lane(desired_lane);
    double car_yaw = stm->car->yaw;
    int lane=stm->car->get_lane();

    //
    // handle speed control
    //
    double car_speed = stm->car->speed;
    double ref_vel = car_speed;
    if (ref_vel < desired_speed) {
        ref_vel += REASONABLE_ACCEL;
        if (ref_vel > desired_speed) {
            ref_vel = desired_speed;
        }
    } else if (ref_vel > desired_speed) {
        ref_vel -= REASONABLE_ACCEL; 
        if (ref_vel < desired_speed) {
            ref_vel = desired_speed;
        }
    }

    double des_speed_mph = desired_speed/MPH_TO_MPS;
    //std::cout << "desired_speed is " << des_speed_mph << " MPH" << std::endl;
    double ref_vel_mph = ref_vel/MPH_TO_MPS;
    //std::cout << "ref_vel is " << ref_vel_mph << " MPH" << std::endl;

    vector<double> xy = getXY(s, d, maps_s, maps_x, maps_y);
    double car_s=s;
    double car_d=d;
    double car_x=xy[0];
    double car_y=xy[1];
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    vector<double> ptsx;
    vector<double> ptsy;

    if (prev_size < 2) {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);
        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);
        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    } else {
        ref_x = previous_path_x[prev_size-1];
        ref_y = previous_path_y[prev_size-1];
        double ref_x_prev = previous_path_x[prev_size-2];
        double ref_y_prev = previous_path_y[prev_size-2];
        ref_yaw = atan2(ref_y - ref_y_prev,ref_x - ref_x_prev);
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);
        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }
    //std::cout << "ref_x, ref_y = " << ref_x << ", " << ref_y << std::endl; 
    //std::cout << "desired_d = " << desired_d << std::endl; 

    //
    // make some widely spaces ref points on which to use spline
    // let's change lane somewhat gradually
    //
    vector<double> next_wp0 = getXY(car_s + 40, desired_d,maps_s,maps_x,maps_y);
    vector<double> next_wp1 = getXY(car_s + 70, desired_d,maps_s,maps_x,maps_y);
    vector<double> next_wp2 = getXY(car_s + 100, desired_d,maps_s,maps_x,maps_y);
    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);
    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    //std::cout << "before transformation: " << std::endl;
    /*
    for (int i=0; i<ptsx.size(); i++) {
        std::cout << "ptsx[" << i <<"] is " << ptsx[i] << std::endl;         
        std::cout << "ptsy[" << i <<"] is " << ptsy[i] << std::endl;         
    }
    */

    //std::cout << "after transformation: " << std::endl;
    for (int i=0; i<ptsx.size(); i++) {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;
        ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
        ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
        //std::cout << "ptsx[" << i <<"] is " << ptsx[i] << std::endl;         
        //std::cout << "ptsy[" << i <<"] is " << ptsy[i] << std::endl;         
    }

    //
    // Use spline
    //
    tk::spline spl;
    spl.set_points(ptsx,ptsy);

/***
    for (int i=0; i<previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }
***/
    double target_x = 40.0;
    double target_y = spl(target_x);
    double target_dist = sqrt((target_x*target_x) + (target_y*target_y));
    double x_add_on = 0.0;

    for (int i=1; i <= NUM_CONTROL_POINTS - previous_path_x.size(); i++) {
        double N = (target_dist /(0.02*ref_vel));
        double x_point = x_add_on+(target_x)/N;
        double y_point = spl(x_point);
        x_add_on = x_point;
        double x_ref=x_point;
        double y_ref=y_point;
        x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
        y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    //std::cout << "num x points = " << this->next_x_vals.size() << std::endl;
    //std::cout << "num y points = " << this->next_y_vals.size() << std::endl;
}
