/***************************************************************************************
** other_vehicles.h
** Contains data for other vehicles on the road, found from sensor fusion
** It contains a map that maps vehicle ID to a vector<Vehicle>. We only get
** element 0 of this vector from sensor fusion, but we use a vector so that we can
** predict where the vehicle will be later
**
** For other vehicles (other than ego car), we get the (estimated) vehicle state from sensor data.
** The format of the data is this:
**     element 0:  car ID
**     element 1:  x
**     element 2:  y
**     element 3:  vx
**     element 4:  vy
**     element 5:  s
**     element 6:  d
**
***************************************************************************************/
#ifndef _OTHER_VEHICLES_H
#define _OTHER_VEHICLES_H

#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "vehicle.h"

using namespace std;

class OtherVehicleSet {
public:
    int numVehicles;
    map<int, VehicleTrajectory> trajectories;

    /**
    * Constructor
    */
    OtherVehicleSet();

    /**
    * Destructor
    */
    virtual ~OtherVehicleSet();
    void Init(vector<vector<double>> &sensor_data, double dt_prop, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

};
 
#endif

