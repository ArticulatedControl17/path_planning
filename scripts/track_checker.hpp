#include "model.hpp"
#include "error_calc.hpp"

#ifndef TrackChecker_H
#define TrackChecker_H


class InTrack {
    public:
        InTrack(bool in_track, double error);
        bool in_track;
        double error;
};


class TrackChecker {
    public:
        TrackChecker(int **map);
        ~TrackChecker();
        bool isAllowed(Point *point);
        bool checkIfInTrack2(Point *toPoint, double th1, double th2);
        InTrack * checkIfInTrack(Point *prevPoint, double prevth1, double prevth2, Point *toPoint, double th1, double th2, ErrorCalc *front_ec, ErrorCalc *back_ec);
        Point ** getPointsInBetween(Point *p1, Point *p2, double n);
        void setMap(int **map);

        Truck *truck;
        int **map;
};


#endif