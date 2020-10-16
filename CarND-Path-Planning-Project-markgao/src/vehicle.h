#ifndef VEHICLE_H
#define VEHICLE_H

#include <math.h>
#include <vector>
using namespace std;

double cost_of_all(double weight_buffer, double diff_front_s,
                   double weight_crash, double diff_rear_s,
                   double weight_save_time, double front_vehicle_v, double max_v);

bool vehicle(vector<double> prediction_front, vector<double> prediction_left_right, double &ref_vel, double &lane, double max_v);


#endif // VEHICLE_H
