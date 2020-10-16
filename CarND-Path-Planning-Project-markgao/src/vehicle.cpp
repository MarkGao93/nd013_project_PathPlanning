#include "vehicle.h"
#include <vector>
#include <cmath>
#include "cost.h"
using namespace std;


double cost_of_all(double weight_buffer, double diff_front_s,
                   double weight_crash, double diff_rear_s,
                   double weight_save_time, double front_vehicle_v, double max_v)
{
    double cost_for_buffer = cost_buffer(weight_buffer, diff_front_s);
    double cost_for_crash = cost_crash(weight_crash, diff_front_s, diff_rear_s);
    double cost_for_saving_time = cost_save_time(weight_save_time, front_vehicle_v, max_v);
    double cost_all = cost_for_buffer + cost_for_crash + cost_for_saving_time;

    return cost_all;
}


bool vehicle(vector<double> prediction_front, vector<double> prediction_left_right, double &ref_vel, double &lane, double max_v)
{
    /* Finite State Machine
     * Calculate the cost of the following three states:
     * Keep Lane, Change for Left Lane, Change for Right Lane
     * Then choose the state with the minimun cost
     */

    double weight_crash = 30;
    double weight_save_time = 50;
    double weight_buffer = 1000;
    double cost_keep_lane, cost_change_left, cost_change_right;
    double speed_increase_step = 0.23;
    bool change_lane = false;

    // Case 1: If no front car or front car is far away, speed up to max velocity
    if(prediction_front[0] == -1 || prediction_front[0] > 30)
    {
        if(ref_vel < max_v)
        {
            ref_vel += speed_increase_step;    // speed increase step can not be too big, or might cause jerk
        }
        else
        {
            ref_vel = max_v;
        }
    }
    else
    {
        if(ref_vel < max_v + 0.5)
        {
            if(prediction_front[1] > ref_vel)    // Case 2: If the front car is near but it's faster, speed up to max velocity
            {
                ref_vel += speed_increase_step;
            }            
            else    // Case 3: If the front car is near and it's slower, speed down
            {
                ref_vel -= exp((ref_vel - prediction_front[1])/(max_v + 0.5))/8.0;
            }
        }


        // Calculate the cost for Keep Lane state
        cost_keep_lane = cost_of_all(weight_buffer, prediction_front[0],
                                     weight_crash, -1,
                                     weight_save_time, prediction_front[1], max_v);

        // Calculate the cost for Change for Left Lane state
        // Add a small cost to cost_change_left and cost_change_right to avoid frequent lane changes
        // when cost_change_left and cost_change_right wave aroud cost_keep_lane
        if(lane > 0)
        {
            cost_change_left = 12 + cost_of_all(weight_buffer, prediction_left_right[0],
                                                weight_crash, prediction_left_right[2],
                                                weight_save_time, prediction_left_right[4], max_v);
        }
        else
        {
            cost_change_left = 1000;    // Cost for impossible left change
        }

        // Calculate the cost for Change for Right Lane state
        if(lane < 2)
        {
            cost_change_right = 10 + cost_of_all(weight_buffer, prediction_left_right[1],
                                                 weight_crash, prediction_left_right[3],
                                                 weight_save_time, prediction_left_right[5], max_v);
        }
        else
        {
            cost_change_right = 1000;    // Cost for impossible right change
        }


        // Determine whether to change lane based on cost
        if((cost_keep_lane < cost_change_left) && (cost_keep_lane < cost_change_right))
        {
            change_lane = false;
        }
        else if((cost_change_left < cost_keep_lane) && (cost_change_left < cost_change_right))
        {
            if(lane > 0)
            {
                lane -= 1;
                change_lane = true;
            }
        }
        else if((cost_change_right < cost_keep_lane) && (cost_change_right < cost_change_left))
        {
            if(lane < 2)
            {
                lane += 1;
                change_lane = true;
            }
        }

    }    // end "no front car" if

    return change_lane;
}
