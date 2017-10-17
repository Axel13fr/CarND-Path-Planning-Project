#ifndef TRAJECGENERATOR_H
#define TRAJECGENERATOR_H

#include <vector>
#include "car.h"
#include <math.h>
#include "spline.h"

using namespace std;

class TrajecGenerator
{
public:
    TrajecGenerator();

    // Transform from Frenet s,d coordinates to Cartesian x,y
    static vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
    {
        int prev_wp = -1;

        while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
        {
            prev_wp++;
        }

        int wp2 = (prev_wp+1)%maps_x.size();

        double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
        // the x,y,s along the segment
        double seg_s = (s-maps_s[prev_wp]);

        double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
        double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

        double perp_heading = heading-M_PI/2;

        double x = seg_x + d*cos(perp_heading);
        double y = seg_y + d*sin(perp_heading);

        return {x,y};

    }

    template <class T>
    static void computeRefPoints(vector<double>& ptsx, vector<double>& ptsy, Car& car,
                          const T& previous_path_x,const T& previous_path_y)
    {
        // Number of points already given to the control
        auto prev_size = previous_path_x.size();
        if(prev_size < 2){
            // Compute a point 1m behind the current car position
            double prev_car_x = car.ref_x - cos(car.ref_yaw);
            double prev_car_y = car.ref_y - sin(car.ref_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car.ref_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car.ref_y);
        }else{
            car.ref_x = previous_path_x[prev_size - 1];
            car.ref_y = previous_path_y[prev_size - 1];

            double prev_car_x =  previous_path_x[prev_size - 2];
            double prev_car_y =  previous_path_y[prev_size - 2];
            car.ref_yaw = atan2(car.ref_y - prev_car_y,car.ref_x - prev_car_x);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car.ref_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car.ref_y);
        }
    }

    static void generateSplinePoints(const vector<double> previous_path_x, const vector<double>& previous_path_y,
                              Car& car,
                              const vector<double>& ptsx,const vector<double>& ptsy,
                              vector<double>& next_x_vals, vector<double>& next_y_vals)
    {
        tk::spline s;
        s.set_points(ptsx,ptsy);


        // Previous points from last time
        for(uint i = 0; i < previous_path_x.size() ; i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
        }

        double target_x = 30.0;
        double target_y = s(target_x);
        double target_dist = sqrt(target_x*target_x + target_y*target_y);

        double x_shift = 0;
        constexpr double MPH_TO_METERS_PER_SEC = 1/2.24;
        const auto N = target_dist /(Car::UPDATE_PERIOD_SECS*car.getSpeed_setpoint()*MPH_TO_METERS_PER_SEC);
        for(uint i = 0 ; i <= Car::MAX_WAY_PTS - previous_path_x.size() ; i++){
            double x_point = x_shift + (target_x/N);
            double y_point = s(x_point);

            x_shift = x_point;

            // Temp variables for rotation of x points
            double x_ref = x_point;
            double y_ref = y_point;

            // Back to world coordinates : translate and rotate
            x_point = car.ref_x + x_ref*cos(car.ref_yaw) - y_ref*sin(car.ref_yaw);
            y_point = car.ref_y + x_ref*sin(car.ref_yaw) + y_ref*cos(car.ref_yaw);

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
        }
    }

};

#endif // TRAJECGENERATOR_H
