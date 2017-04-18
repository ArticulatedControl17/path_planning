#include "model.hpp"
#include "helper_functions.hpp"
#include <math.h>
#include <iostream>


Truck::Truck() {
    left_front = new Point(0.0, 0.0);
    right_front = new Point(0.0, 0.0);
    left_back = new Point(0.0, 0.0);
    right_back = new Point(0.0, 0.0);

    left_front_wheel = new Point(0.0, 0.0);
    right_front_wheel = new Point(0.0, 0.0);
    left_back_wheel = new Point(0.0, 0.0);
    right_back_wheel = new Point(0.0, 0.0);
}


Truck::~Truck() {
    delete left_front;
    delete right_front;
    delete left_back;
    delete right_back;

    delete left_front_wheel;
    delete right_front_wheel;
    delete left_back_wheel;
    delete right_back_wheel;
}


void Truck::setCorners(Point *pointFront, double th1, double th2) {
    //std::cout << "theta1: "<< th1 << " cos: " << cos(th1) << " sin: " << sin(th1) << std::endl;
    //std::cout << "theta2: "<< th2 << " cos: " << cos(th2) << " sin: " << sin(th2) << std::endl << std::endl;

    Point *point = backMiddleTrailer(pointFront, th1, th2);

    // Hitch joint
    double x2 = point->x + (TRAILER_LENGTH + TL_BACK) * cos(th2);
    double y2 = point->y + (TRAILER_LENGTH + TL_BACK) * sin(th2);

    // Left back
    double x4 = point->x - cos(M_PI/2 - th2) * TRAILER_WIDTH/2;
    double y4 = point->y + sin(M_PI/2 - th2) * TRAILER_WIDTH/2;
    left_back->setValues(x4, y4);
    std::cout << "left back: " << x4 << " " << y4 << std::endl;

    // Right back
    double x5 = point->x + cos(M_PI/2 - th2) * TRAILER_WIDTH/2;
    double y5 = point->y - sin(M_PI/2 - th2) * TRAILER_WIDTH/2;
    right_back->setValues(x5, y5);
    std::cout << "right back: " << x5 << " " << y5 << std::endl;

    // Left back axis
    double x12 = x4 + TL_BACK * cos(th2);
    double y12 = y4 + TL_BACK * sin(th2);
    left_back_wheel->setValues(x12, y12);
    //std::cout << std::endl << x4 << " " << TL_BACK << " " << cos(th2) << " ekv: " << tl_back * cos(th2) << std::endl;
    //std::cout << y4 << " " << TL_BACK << " " << sin(th2) << " ekv: " << tl_back * sin(th2) << std::endl << std::endl;
    std::cout << "left back axis: " << x12 << " " << y12 << std::endl;

    // Right back axis
    double x13 = x5 + (TL_BACK * cos(th2));
    double y13 = y5 + (TL_BACK * sin(th2));
    right_back_wheel->setValues(x13, y13);
    std::cout << "right back axis: " << x13 << " " << y13 << std::endl;

    // Left joint wheel
    double x6 = x2 - cos(M_PI/2 - th1) * HEADER_WIDTH/2;
    double y6 = y2 + sin(M_PI/2 - th1) * HEADER_WIDTH/2;

    // Right joint wheel
    double x7 = x2 + cos(M_PI/2 - th1) * HEADER_WIDTH/2;
    double y7 = y2 - sin(M_PI/2 - th1) * HEADER_WIDTH/2;

    // Left front axis
    double x8 = x6 + (HEADER_LENGTH-5.0) * cos(th1);
    double y8 = y6 + (HEADER_LENGTH-5.0) * sin(th1);
    left_front_wheel->setValues(x8, y8);
    //std::cout << "theta1: " << th1 << " " << cos(th1) << " " << sin(th1) << std::endl;
    std::cout << "left front axis: " << x8 << " " << y8 << std::endl;

    // Right front axis
    double x9 = x7 + (HEADER_LENGTH-5.0) * cos(th1);
    double y9 = y7 + (HEADER_LENGTH-5.0) * sin(th1);
    right_front_wheel->setValues(x9, y9);
    std::cout << "right front axis: " << x9 << " " << y9 << std::endl;

    // Left front
    double x10 = x6 + ((HEADER_LENGTH-5.0) + HL_FRONT) * cos(th1);
    double y10 = y6 + ((HEADER_LENGTH-5.0) + HL_FRONT) * sin(th1);
    left_front->setValues(x10, y10);
    std::cout << "left front: " << x10 << " " << y10 << std::endl;

    // Right front
    double x11 = x7 + ((HEADER_LENGTH-5.0) + HL_FRONT) * cos(th1);
    double y11 = y7 + ((HEADER_LENGTH-5.0) + HL_FRONT) * sin(th1);
    right_front->setValues(x11, y11);
    std::cout << "right front: " << x11 << " " << y11 << std::endl;

    //std::cout << std::endl << "theta1: "<< th1 << " cos: " << cos(th1) << " sin: " << sin(th1) << std::endl;
    //std::cout << "theta2: "<< th2 << " cos: " << cos(th2) << " sin: " << sin(th2) << std::endl;
}


Point * Truck::backMiddleTrailer(Point *pointFront, double th1, double th2) {
    double jpx = pointFront->x - cos(th1) * (HEADER_LENGTH-5.0);
    double jpy = pointFront->y - sin(th1) * (HEADER_LENGTH-5.0);

    double px = jpx - cos(th2) * (TRAILER_LENGTH + TL_BACK);
    double py = jpy - sin(th2) * (TRAILER_LENGTH + TL_BACK);

    return new Point(px, py);
}



int main() {
    Truck *truck = new Truck();
    truck->setCorners(new Point(200, 200), radians(-90), radians(-90));
    //truck->setCorners(new Point(200, 200), radians(0), radians(0));
}


/*

g++ -std=gnu++11 test_model.cpp pathPlanning.cpp Point.cpp track_checker.cpp vehicleState.cpp vehicleState_error.cpp helper_functions.cpp error_calc.cpp

*/