/***************************************************************************************
** other_vehicles.cpp
** Contains data for other vehicles on the road, found from sensor fusion
** It contains a map that maps vehicle ID to a vector<Vehicle>. We only get
** element 0 of this vector from sensor fusion, but we use a vector so that we can
** predict where the vehicle will be later
***************************************************************************************/
#include "other_vehicles.h"

using namespace std;

OtherVehicleSet::OtherVehicleSet() {
    numVehicles=0;
}
OtherVehicleSet::~OtherVehicleSet() {}

void OtherVehicleSet::Init(vector<vector<double>> &sensor_data, double dt_prop, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
    trajectories.clear();
    int numVehicles = sensor_data.size();
    for (int i=0;i<numVehicles;i++) {
        Vehicle other_vehicle;
        VehicleTrajectory other_vehicle_trajectory;
        vector<double> vehicle_data = sensor_data[i];
        other_vehicle.localize_from_sensors(vehicle_data, dt_prop, maps_s, maps_x, maps_y);
        int vehicle_ID = other_vehicle.carID;
        other_vehicle_trajectory.add(other_vehicle);
        trajectories[vehicle_ID] = other_vehicle_trajectory;
    }
}

