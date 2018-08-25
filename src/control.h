/******************************************************************************
** control.h
** Header file for TrajectoryControl class 
******************************************************************************/
#ifndef _TRAJECTORY_CONTROL_H_
#define _TRAJECTORY_CONTROL_H_

#include "constants.h"
#include "vehicle.h"
#include "behavior.h"

class TrajectoryControl {
    public:
        CarBehaviorStateMachine *stm;
        vector<double> previous_path_x;
        vector<double> previous_path_y;
        vector<double> next_x_vals;
        vector<double> next_y_vals;

        TrajectoryControl() {
        }

        virtual ~TrajectoryControl() {
        }

        void new_iteration(CarBehaviorStateMachine *state_machine, vector<double> &previous_path_x, vector<double> &previous_path_y);
        void generate_path(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

        void print_trajectory() {
            int n=next_x_vals.size();
            for (int i=0;i<n;i++) {
                double x = next_x_vals[i];
                double y = next_y_vals[i];
                std::cout << "i, x, y = " << i << ", " << x << ", " << y << std::endl;
            }
        }

    private:
        int fill_from_previous();
        void generate_drive_forward_path_bad(int remaining, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y); 
        void generate_drive_forward_path(int remaining, double desired_speed, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y); 
        void generate_lane_change_path_bad(int remaining, int desired_lane, double desired_speed, double accel, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
        void generate_lane_change_path(int remaining, int desired_lane, double desired_speed, double accel, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

};

#endif
