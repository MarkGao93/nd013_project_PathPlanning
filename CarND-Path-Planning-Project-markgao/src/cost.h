#ifndef COST_H
#define COST_H

#include <vector>
using namespace std;

double cost_buffer(double weight_buffer, double diff_s);
double cost_crash(double weight_crash, double diff_front_s,double diff_rear_s);
double cost_save_time(double weight_save_time, double front_vehicle_v, double max_v);

#endif // COST_H
