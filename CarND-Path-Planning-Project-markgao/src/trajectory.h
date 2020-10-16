#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
using namespace std;

vector<vector<double>> trajectory(const vector<double> &pre_path_x, const vector<double> &pre_path_y,
                                  double ref_x, double ref_y, double ref_yaw,
                                  bool change_lane, int PATH_SIZE, double lane, double ref_vel, double car_s,
                                  const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y, const vector<double> &map_waypoints_s);


#endif // TRAJECTORY_H
