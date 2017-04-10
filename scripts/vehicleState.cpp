#include "vehicleState.hpp"


VehicleState::VehicleState(double nx, double ny, double nth1, double nth2){
    x = nx;
    y = ny;
    th1 = nth1;
    th2 = nth2;
}

double VehicleState::getX(void){
    return x;
}

double VehicleState::getY(void){
    return y;
}

double VehicleState::getTh1(){
    return th1;
}

double VehicleState::getTh2(){
    return th2;
}

void VehicleState::setValues(double nx, double ny, double nth1, double nth2){
    x = nx;
    y = ny;
    th1 = nth1;
    th2 = nth2;

}
