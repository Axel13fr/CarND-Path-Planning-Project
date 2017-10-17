#include "car.h"
#include "cassert"


bool Car::shouldFollow(const EnvCars& other_cars, double& front_car_speed)
{
    Car closest_car;
    auto ret = other_cars.getFrontClosestSameLaneCar(*this,closest_car)
            and (closest_car.ref_speed <this->ref_speed)
            and this->tooClose(closest_car);
    if(ret){
        front_car_speed = closest_car.ref_speed;
    }

    return ret;
}

bool Car::shouldDrive(const EnvCars& other_cars)
{
    Car closest_car;
    return other_cars.getFrontClosestSameLaneCar(*this,closest_car)
            and this->farEnough(closest_car);
}

void Car::updateState(const EnvCars& other_cars,double car_speed)
{

    auto new_state = state;
    double new_speed = Car::MAX_VELOCITY_MPH;
    switch (state) {
    case DRIVING:
        if(shouldFollow(other_cars,new_speed)){
            new_state = CAR_FOLLOWING;
        }
        break;
    case CAR_FOLLOWING:
        if(shouldDrive(other_cars)){
            new_state = DRIVING;
        }else if(tryOvertake(other_cars)){
            new_state = OVERTAKING;
        }
        break;
    case OVERTAKING:
        const double target_lane = 2+LANE_WIDTH_M*this->lane_id;
        if(fabs(ref_d - target_lane) < 0.2){
            new_state = DRIVING;
        }
        break;
    }

    setState(new_state);
    setTarget_speed(new_speed,car_speed);
}

bool Car::tooClose(const Car &other_car,const double min_dist_m){
    return fabs(other_car.ref_s - this->ref_s) < min_dist_m;
}

bool Car::farEnough(const Car &other_car){
    constexpr double MIN_ACCELERATE_DIST_M = 20.0;
    return fabs(other_car.ref_s - this->ref_s) > MIN_ACCELERATE_DIST_M;
}

double Car::getPlanningDistance()
{
    double distance = 30;
    switch (state) {
    case DRIVING:
        distance = 30;
        break;
    case OVERTAKING:
        // Get a smoother trajectory for overtaking !
        distance = 50;
        break;
    case CAR_FOLLOWING:
        distance = 30;
        break;
    }
    return distance;
}

bool Car::tryOvertake(const EnvCars& other_cars)
{
    auto canOvertake = false;
    // Let's try to overtake
    if(lane_id == 2){
        if(canOvertakeOnLane(other_cars,1)){
            this->lane_id = 1;
            canOvertake = true;
        }
    }else if (lane_id == 1){
        if(canOvertakeOnLane(other_cars,0)){
            this->lane_id = 0;
            canOvertake = true;
        }else if(canOvertakeOnLane(other_cars,2)){
            this->lane_id = 2;
            canOvertake = true;
        }
    }else{ // lane id = 0
        if(canOvertakeOnLane(other_cars,1)){
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
    constexpr auto MIN_REAR_SAFETY_DIST = 10.0;
    auto car_in_front = other_cars.getFrontClosestCarInLane(*this,lane_id,frontClosest);
    auto car_in_rear = other_cars.getRearClosestCarInLane(*this,lane_id,rearClosest);
    if(car_in_front and car_in_rear){
        if(not this->tooClose(frontClosest) and not this->tooClose(rearClosest,MIN_REAR_SAFETY_DIST)){
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
        if(not this->tooClose(rearClosest,MIN_REAR_SAFETY_DIST)){
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
