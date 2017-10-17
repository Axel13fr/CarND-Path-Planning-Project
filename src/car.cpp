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

bool Car::tryOvertake(const EnvCars& other_cars)
{
    auto canOvertake = false;
    // Let's try to overtake
    if(lane_id == 2){
        if(canOvertakeOnLane(other_cars,1)){
            this->setState(Car::OVERTAKING);
            this->lane_id = 1;
            canOvertake = true;
        }
    }else if (lane_id == 1){
        if(canOvertakeOnLane(other_cars,0)){
            this->setState(Car::OVERTAKING);
            this->lane_id = 0;
            canOvertake = true;
        }else if(canOvertakeOnLane(other_cars,2)){
            this->setState(Car::OVERTAKING);
            this->lane_id = 2;
            canOvertake = true;
        }
    }else{ // lane id = 0
        if(canOvertakeOnLane(other_cars,1)){
            this->setState(Car::OVERTAKING);
            this->lane_id = 1;
            canOvertake = true;
        }
    }

    return canOvertake;
}

bool Car::canOvertakeOnLane(const EnvCars &other_cars, uint8_t lane_id)
{
    Car frontClosest;
    Car rearClosest;
    auto car_in_front = other_cars.getFrontClosestCarInLane(*this,lane_id,frontClosest);
    auto car_in_rear = other_cars.getRearClosestCarInLane(*this,lane_id,rearClosest);
    if(car_in_front and car_in_rear){
        if(not this->tooClose(frontClosest) and not this->tooClose(rearClosest)){
            return true;
        }else{
            return false;
        }
    }else if (car_in_front){
        if(not this->tooClose(frontClosest)){
            return true;
        }else{
            return false;
        }
    }else if(car_in_rear){
        if(not this->tooClose(rearClosest)){
            return true;
        }else{
            return false;
        }
    }else{
        // No car, good to overtake !
        return true;
    }
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

void Car::setState(const Car::STATE_E &value)
{
    using namespace std;
    if(state != value){
        state = value;
        switch (state) {
        case DRIVING:
            cout << "New State: driving" << endl;
            break;
       case OVERTAKING:
            cout << "New State: overtaking" << endl;
            break;
        case CAR_FOLLOWING:
            cout << "New State: car following" << endl;
            break;

        }
    }
}
