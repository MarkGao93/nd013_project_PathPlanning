#include "cost.h"
#include <vector>
#include <cmath>
using namespace std;


// Punish small front distance between front car and ego car
double cost_buffer(double weight_buffer, double diff_s)
{
    double cost_buf = -1;

    if(diff_s > 0)
    {
         cost_buf = weight_buffer/diff_s;
    }

    return cost_buf;
}


// Punish small longitudinal distance within certain range between other cars and ego car
double cost_crash(double weight_crash, double diff_front_s, double diff_rear_s)
{
    double cost_crash_front = -1;
    double cost_crash_rear = -1;

    if(diff_front_s > 0)
    {
        if(diff_front_s < 20)
        {
            cost_crash_front = (30 - diff_front_s)*weight_crash;
        }
    }

    if(diff_rear_s > 0)
    {
        if(diff_rear_s < 20)
        {
            cost_crash_rear = (30 - diff_rear_s)*weight_crash;
        }
    }

    double cost_crash_max = max(cost_crash_front, cost_crash_rear);

    return cost_crash_max;
}


// Punish low speed
double cost_save_time(double weight_save_time, double front_vehicle_v, double max_v)
{
    double cost_save_time = -1;

    if((front_vehicle_v > 0) && (front_vehicle_v < max_v))
    {
        cost_save_time = weight_save_time*(max_v - front_vehicle_v)/max_v;
    }

    return cost_save_time;
}

