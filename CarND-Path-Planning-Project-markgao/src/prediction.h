#ifndef PREDICTION_H
#define PREDICTION_H

#include <vector>
using namespace std;

vector<double> prediction_front(vector<vector<double>> sensor_fusion, int path_size, const double &lane, double future_car_s);

vector<double> prediction_left_right(vector<vector<double>> sensor_fusion, int path_size, const double &lane, double future_car_s);

#endif // PREDICTION_H
