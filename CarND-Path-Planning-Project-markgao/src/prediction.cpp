#include <vector>
#include <cmath>
#include "prediction.h"

vector<double> prediction_front(vector<vector<double>> sensor_fusion, int path_size, const double &lane, double future_car_s)
{
    double range = -1;    // m
    double target_velocity = -1;    // m/s
    double range_min = 1000;    // m

    for(int i=0; i<sensor_fusion.size(); i++)
    {
        double d = sensor_fusion[i][6];    // car's d position in frenet coordinates
        if(d > 4*lane && d < 4*lane+4)    // if car's in ego car's lane
        {
            double front_car_vx = sensor_fusion[i][3];    // car's x velocity in m/s
            double front_car_vy = sensor_fusion[i][4];    // car's y velocity in m/s
            double front_car_v = sqrt(front_car_vx*front_car_vx + front_car_vy*front_car_vy);    // car's absolute velocity
            double front_car_s = sensor_fusion[i][5];    // car's s position in frenet coordinates

            double future_front_car_s = front_car_s + front_car_v*0.02*path_size;    // car's s position based on its velocity

            if(future_front_car_s > future_car_s)    // if car's in front of ego car
            {
                if((future_front_car_s - future_car_s) < range_min)
                {
                    range = future_front_car_s - future_car_s;    // get range of the nearest front car in next time frame
                    range_min = range;
                    target_velocity = front_car_v;    // m/s
                }
            }
        }
    }

    return {range, target_velocity};
}

vector<double> prediction_left_right(vector<vector<double>> sensor_fusion, int path_size, const double &lane, double future_car_s)
{
    // Get s and v of cars in ego car's left and right lane
    vector<double> future_s_left_car, future_s_right_car;
    vector<double> future_v_left_car, future_v_right_car;

    for(int i=0; i<sensor_fusion.size(); i++)
    {
        if(lane > 0)    // if ego car is not in left lane
        {
            double d_lane_l = sensor_fusion[i][6];    // car's d position in frenet coordinates
            if(d_lane_l > 4*(lane-1) && d_lane_l < 4*(lane-1) + 4)    // if car's in ego car's left lane
            {
                double left_car_vx = sensor_fusion[i][3];    // car's x velocity in m/s
                double left_car_vy = sensor_fusion[i][4];    // car's y velocity in m/s
                double left_car_v = sqrt(left_car_vx*left_car_vx + left_car_vy*left_car_vy);    // car's absolute velocity
                double left_car_s = sensor_fusion[i][5];    // car's s position in frenet coordinates
                double future_left_car_s = left_car_s + left_car_v*0.02*path_size;    // car's s position based on its velocity
                future_s_left_car.push_back(future_left_car_s);
                future_v_left_car.push_back(left_car_v);
            }
        }

        if(lane < 2)    // if ego car is not in right lane
        {
            double d_lane_r = sensor_fusion[i][6];    // car's d position in frenet coordinates
            if(d_lane_r > 4*(lane+1) && d_lane_r < 4*(lane+1) + 4)    // if car's in ego car's right lane
            {
                double right_car_vx = sensor_fusion[i][3];    // car's x velocity in m/s
                double right_car_vy = sensor_fusion[i][4];    // car's y velocity in m/s
                double right_car_v = sqrt(right_car_vx*right_car_vx + right_car_vy*right_car_vy);    // car's absolute velocity
                double right_car_s = sensor_fusion[i][5];    // car's s position in frenet coordinates
                double future_right_car_s = right_car_s + right_car_v*0.02*path_size;    // car's s position based on its velocity
                future_s_right_car.push_back(future_right_car_s);
                future_v_right_car.push_back(right_car_v);
            }
        }
    }    // end for


    // Get range and velocity of cars in ego car's left lane in front of and behind ego car
    double left_front_range = -1;    // m
    double left_front_vel = -1;    // m/s
    double left_rear_range = -1;    // m
    double left_rear_vel = -1;    // m/s
    double left_front_min = 1000;    // m
    double left_rear_min = 1000;    // m

    for(int i=0; i<future_s_left_car.size(); i++)
    {
        if(future_s_left_car[i] > future_car_s)    // if car in ego car's left lane is in front of ego car
        {
            if((future_s_left_car[i] - future_car_s) < left_front_min)
            {
                left_front_range = future_s_left_car[i] - future_car_s;    // get range of the nearest front car in next time frame
                left_front_min = left_front_range;
                left_front_vel = future_v_left_car[i];
            }
        }

        if(future_s_left_car[i] < future_car_s)    // if car in ego car's left lane is behind ego car
        {
            if((future_car_s - future_s_left_car[i]) < left_rear_min)
            {
                left_rear_range = future_car_s - future_s_left_car[i];    // get range of the nearest rear car in next time frame
                left_rear_min = left_rear_range;
                left_rear_vel = future_v_left_car[i];
            }
        }
    }    // end for


    // Get range and velocity of cars in ego car's right lane in front of and behind ego car
    double right_front_range = -1;    // m
    double right_front_vel = -1;    // m/s
    double right_rear_range = -1;    // m
    double right_rear_vel = -1;    // m/s
    double right_front_min = 1000;    // m
    double right_rear_min = 1000;    // m

    for(int i=0; i<future_s_right_car.size(); i++)
    {
        if(future_s_right_car[i] > future_car_s)    // if car in ego car's right lane is in front of ego car
        {
            if((future_s_right_car[i] - future_car_s) < right_front_min)
            {
                right_front_range = future_s_right_car[i] - future_car_s;    // get range of the nearest front car in next time frame
                right_front_min = right_front_range;
                right_front_vel = future_v_right_car[i];
            }
        }

        if(future_s_right_car[i] < future_car_s)    // if car in ego car's right lane is behind ego car
        {
            if((future_car_s - future_s_right_car[i]) < right_rear_min)
            {
                right_rear_range = future_car_s - future_s_right_car[i];    // get range of the nearest rear car in next time frame
                right_rear_min = right_rear_range;
                right_rear_vel = future_v_right_car[i];
            }
        }
    }    // end for

    return {left_front_range, right_front_range, left_rear_range, right_rear_range,
            left_front_vel,   right_front_vel,   left_rear_vel,   right_rear_vel};
}
