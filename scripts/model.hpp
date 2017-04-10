#include <list>

#ifndef Model_H
#define Model_H


class Truck {

    public:
        Truck();
        void setCorners(Point *pointFront, double th1, double th2);
        Point * backMiddleTrailer(Point *pointFront, double th1, double th2);
};


#endif