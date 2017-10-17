#include "car.h"
#include "cassert"

bool Car::tooClose(const Car &other_car){
    constexpr double MIN_SAFETY_DIST_M = 30.0;
    return fabs(other_car.ref_s - this->ref_s) < MIN_SAFETY_DIST_M;
}

bool Car::farEnough(const Car &other_car){
    constexpr double MIN_ACCELERATE_DIST_M = 20.0;
    return fabs(other_car.ref_s - this->ref_s) > MIN_ACCELERATE_DIST_M;
}

double Car::getSpeed_setpoint()
{
    auto factor = (1.0 - exp(-static_cast<double>(speed_cnt/SPEED_T_FACTOR)));
    speed_setpoint = speed_delta*factor + start_speed;
    speed_cnt += 1;
    //std::cout << "Speed Setpoint: " << speed_setpoint << std::endl;
    assert(speed_setpoint > 0);
    return speed_setpoint;
}

void Car::setTarget_speed(double new_speed,double current_speed)
{
    // Change target speed only if different
    if(fabs(new_speed - ref_speed) > 0.01){
        speed_cnt = 1;
        ref_speed = new_speed;
        start_speed = current_speed;
        speed_delta = new_speed - current_speed;
        std::cout << "New Spd: " << new_speed <<
                     " Delta: " << speed_delta <<
                     " Crt Spd: " << start_speed << std::endl;
    }
}
