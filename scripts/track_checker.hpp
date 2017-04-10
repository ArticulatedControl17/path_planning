#include "model.hpp"
#include "error_calc.hpp"

#ifndef TrackChecker_H
#define TrackChecker_H


class TrackChecker {
    public:
        TrackChecker(int **map);
        ~TrackChecker();
        bool checkIfInTrack2(Point *toPoint, double th1, double th2);
        bool checkIfInTrack(Point *prevPoint, double prevth1, double prevth2, Point *toPoint, double th1, double th2, double dt, ErrorCalc *front_ec, ErrorCalc *back_ec);
        Point ** getPointsInBetween(Point *p1, Point *p2, int n);
        void setMap(int **map);

        Truck *model;
        int **map;
};


#endif