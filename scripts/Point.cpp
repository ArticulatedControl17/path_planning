#include "Point.hpp"


Point::Point(double nx, double ny){
    x = nx;
    y = ny;
}

double Point::getX(void){
    return x;
}

double Point::getY(void){
    return y;
}

void Point::setValues(double nx, double ny){
    x = nx;
    y = ny;

}
