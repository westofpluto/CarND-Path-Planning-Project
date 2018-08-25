/***************************************************************************************
** vehicle.cpp
** class Vehicle represents a vehicle, either the "self" car or other cars on the road.
** More specifically, this class represents a Vehicle state in time, where state is 
** defined as the value of x, y, yaw, s, d, speed. Note that this vehicle state
** X(t) should not be confused with the behvior "state" which is a discrete node in 
** a finite state machine
***************************************************************************************/

#include "vehicle.h"
#include "utils.hpp"

Vehicle::Vehicle() {}

Vehicle::~Vehicle() {}

Vehicle *Vehicle::newCopy() {
    Vehicle *vcopy = new Vehicle();
    vcopy->x = x;
    vcopy->y = y;
    vcopy->yaw = yaw;
    vcopy->s = s;
    vcopy->d = d;
    vcopy->speed = speed;
    vcopy->carID = carID;
    vcopy->lane = lane;
    vcopy->t = t;
    vcopy->accel = accel;
    vcopy->max_acceleration = max_acceleration;

    vcopy->x_now = x_now;
    vcopy->y_now = y_now;
    vcopy->yaw_now = yaw_now;
    vcopy->s_now = s_now;
    vcopy->d_now = d_now;
    vcopy->speed_now = speed_now;

    return vcopy;
}

void Vehicle::printMe() {
    std::cout << "VehicleID: " << carID << std::endl;
    std::cout << "   x, y, s, d: " << x << ", " << y << ", " << s << ", " << d << std::endl;
    std::cout << "   lane, speed, accel " << lane << ", " << speed << ", " << accel << std::endl;
}

/*
** For other vehicles (other than ego car), we get the (estimated) vehicle state from sensor data.
** The format of the data is this:
**     element 0:  car ID
**     element 1:  x
**     element 2:  y
**     element 3:  vx
**     element 4:  vy
**     element 5:  s
**     element 6:  d
*/
void Vehicle::localize_from_sensors(vector<double> &car_sensor_data, double dt_prop, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
    this->carID = static_cast<int>(car_sensor_data[0]);
    this->x = car_sensor_data[1];
    this->y = car_sensor_data[2];
    double vx = car_sensor_data[3];
    double vy = car_sensor_data[4];
    this->yaw=atan2(vy,vx);
    this->speed = sqrt(vx*vx+vy*vy);
    this->s=car_sensor_data[5];
    this->d=car_sensor_data[6];
    this->lane=get_lane();

    this->t = 0.0;

    //
    // propagate ahead
    //
    this->s += this->speed*dt_prop;
    vector<double> xy = getXY(this->s, this->d, maps_s, maps_x, maps_y);
    this->x = xy[0];
    this->y = xy[1];
    this->t = dt_prop;
}

/*
** The lane depends on d
** The drivable lanes are 0,1,2. 
**   Lane 0 has a d between 0.0 and 4.0.
**   Lane 1 has a d between 4.0 and 8.0.
**   Lane 2 has a d between 8.0 and 12.0.
*/
int Vehicle::get_lane() {
    return get_lane_for_d(this->d);
}

double Vehicle::get_lane_center() {
    return get_lane_center_for_d(this->d);
}

EgoVehicle::EgoVehicle() {}

EgoVehicle::~EgoVehicle() {}

EgoVehicle EgoVehicle::copyMe() {
    EgoVehicle vcopy;

    vcopy.x = x;
    vcopy.y = y;
    vcopy.yaw = yaw;
    vcopy.s = s;
    vcopy.d = d;
    vcopy.speed = speed;
    vcopy.carID = carID;
    vcopy.lane = lane;
    vcopy.t = t;
    vcopy.accel = accel;
    vcopy.max_acceleration = max_acceleration;

    vcopy.x_now = x_now;
    vcopy.y_now = y_now;
    vcopy.yaw_now = yaw_now;
    vcopy.s_now = s_now;
    vcopy.d_now = d_now;
    vcopy.speed_now = speed_now;

    return vcopy;
}

void EgoVehicle::localize_self(map<string, double> &data) {
    this->carID = SELF_CAR_ID;

    this->x = data["x"];
    this->y = data["y"];
    this->yaw = data["yaw"];
    this->s = data["s"];
    this->d = data["d"];
    this->speed = data["speed"];

    this->x_now = data["x_now"];
    this->y_now = data["y_now"];
    this->yaw_now = data["yaw_now"];
    this->s_now = data["s_now"];
    this->d_now = data["d_now"];
    this->speed_now = data["speed"];

    this->lane = get_lane();
}

EgoVehicle EgoVehicle::move_forward(double tmove) {
    EgoVehicle propagated_car = copyMe();
    propagated_car.s = s + speed*tmove; 
    return propagated_car;
}

bool EgoVehicle::isTooCloseTo(Vehicle *other_car_ahead, double t, double *final_min_dist) {
    //std::cout << "in isTooCloseTo: printing other_car_ahead" << std::endl;
    //other_car_ahead->printMe();
    double initial_distance = other_car_ahead->s - s;
    if (initial_distance < 0.0) {
        initial_distance += MAX_TRACK_S;
    }
    //std::cout << "initial_distance: " << initial_distance << ", other_car speed: " << other_car_ahead->speed << ", my speed: " << speed << std::endl;
    if (other_car_ahead->speed >= speed) {
        //
        // car ahead is going faster than us, so we are closest to it now
        //
        //std::cout << "other car is going FASTER" << std::endl;
        if (initial_distance < MIN_SAFE_FRONT_DIST) {
            *final_min_dist = initial_distance;
            return true;
        } else {
            return false;
        }
    } else {
        //std::cout << "other car is going SLOWER" << std::endl;
        double speed_diff = speed - other_car_ahead->speed;
        if (speed_diff*t >= initial_distance) {
            //
            // sometime between now and t we collide withthe vehicle
            //
            *final_min_dist = 0.0;
            //std::cout << "COLLISION ALERT: speed_diff, t, initial_distance are " << speed_diff << ", " << t << ", " << initial_distance << std::endl;
            return true;
        } else {
            double final_distance = initial_distance - speed_diff*t;
            //std::cout << "final_distance: " << final_distance << ", other_car speed: " << other_car_ahead->speed << ", my speed: " << speed << std::endl;
            if (final_distance < MIN_SAFE_FRONT_DIST) {
                *final_min_dist = final_distance;
                //std::cout << "TOO CLOSE ALERT: speed_diff, t, initial_distance, final_distance are " << speed_diff << ", " << t << ", " << initial_distance << ", " << final_distance << std::endl;
                return true;
            } else {
                return false;
            }
        }
    }
}

