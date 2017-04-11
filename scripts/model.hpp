#include "Point.hpp"

#ifndef Model_H
#define Model_H


class Truck {
    public:
        Truck();
        ~Truck();
        void setCorners(Point *pointFront, double th1, double th2);
        Point * backMiddleTrailer(Point *pointFront, double th1, double th2);

        Point *left_front;
        Point *right_front;
        Point *left_back;
        Point *right_back;

        Point *left_front_wheel;
        Point *right_front_wheel;
        Point *left_back_wheel;
        Point *right_back_wheel;
};


#endif