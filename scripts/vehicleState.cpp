#include "vehicleState.hpp"


VehicleState::VehicleState(double nx, double ny, double nth1, double nth2){
    x = nx;
    y = ny;
    th1 = nth1;
    th2 = nth2;
}


void VehicleState::setValues(double nx, double ny, double nth1, double nth2){
    x = nx;
    y = ny;
    th1 = nth1;
    th2 = nth2;

}
