#ifndef CAR_H
#define CAR_H

#include <vector>
#include <math.h>
#include <iostream>

class Car{

public:
    Car():
        ref_speed(0),
        lane_id(1), // Ego Car starts in the middle lane
        speed_cnt(1)
    {

    }

    // Built from fusion data
    Car(const std::vector<double> other_car):
        ref_x(other_car[1]),
        ref_y(other_car[2]),
        ref_s(other_car[5]),
        ref_d(other_car[6]),
        ref_speed(sqrt(other_car[3]*other_car[3] + other_car[4]*other_car[4])),
        lane_id(0)
    {
        if(ref_d > LANE_WIDTH_M and ref_d < 2*LANE_WIDTH_M){
            lane_id = 1;
        }else if(ref_d > 2*LANE_WIDTH_M and ref_d < 3*LANE_WIDTH_M){
            lane_id = 2;
        }else{
            lane_id = 0;
        }
    }

    bool tooClose(const Car& other_car);
    bool farEnough(const Car &other_car);
    double getSpeed_setpoint();
    void setTarget_speed(double new_speed, double current_speed);

    double ref_x;
    double ref_y;
    double ref_yaw;
    double ref_s;
    double ref_d;
    double ref_speed;

    // Simulator starts the car on the middle lane (0 being the left lane)
    uint8_t lane_id;
    // To handle a progressive acceleration
    double speed_setpoint;
    double speed_delta;
    double start_speed;
    uint32_t speed_cnt;
    static constexpr double SPEED_T_FACTOR = 50.0; // 0.02*200 = 4secs

    static constexpr double MAX_VELOCITY_MPH = 49.9;
    static constexpr uint16_t MAX_WAY_PTS = 50;
    static constexpr double UPDATE_PERIOD_SECS = 0.02;
    static constexpr int LANE_WIDTH_M = 4;
};

class EnvCars{
public:
    EnvCars(const std::vector<std::vector<double>> fusion_data){
        for(const auto& car : fusion_data){
            other_cars.push_back(Car(car));
        }
    }

    bool getFrontClosestSameLaneCar(const Car ref_car,Car& ret_car){
        auto lane = ref_car.lane_id;
        auto s = ref_car.ref_s;
        auto found = false;
        ret_car.ref_s = 10E10;
        for(auto& car : other_cars){
            // Same lane and in front
            if(lane == car.lane_id and s < car.ref_s){
                // Closer than previously found car
                if(car.ref_s < ret_car.ref_s){
                    ret_car = car;
                    found = true;
                }
            }
        }

        return found;
    }

    std::vector<Car> other_cars;
};

#endif // CAR_H
