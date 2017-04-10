/*
    Requirements: C++11/gnu++1

    Compile with flag:  -std=gnu++11

                        g++ -c  model.cpp -std=gnu++11
 */
#include "model.hpp"
#include "helper_functions.hpp"
#include <math.h>
#include <list>


// Returns a list of Points, representing the corners of the vehicle
std::list<Point *> Truck::calculateCorners(Point *pointFront, double th1, double th2) {
    Point *point = backMiddleTrailer(pointFront, th1, th2);

    // Hitch joint
    double x2 = point->getX() + (TRAILER_LENGTH + TL_BACK) * cos(th2);
    double y2 = point->getY() + (TRAILER_LENGTH + TL_BACK) * sin(th2);

    // Left back
    double x4 = point->getX() - cos(M_PI/2 - th2) * TRAILER_WIDTH/2;
    double y4 = point->getY() + sin(M_PI/2 - th2) * TRAILER_WIDTH/2;

    // Right back
    double x5 = point->getX() + cos(M_PI/2 - th2) * TRAILER_WIDTH/2;
    double y5 = point->getY() - sin(M_PI/2 - th2) * TRAILER_WIDTH/2;

    // Left back axis
    double x12 = x4 + TL_BACK * cos(th2);
    double y12 = y4 + TL_BACK * sin(th2);

    // Right back axis
    double x13 = x5 + TL_BACK * cos(th2);
    double y13 = y5 + TL_BACK * sin(th2);

    // Left joint wheel
    double x6 = x2 - cos(M_PI/2 - th1) * HEADER_WIDTH/2;
    double y6 = y2 + sin(M_PI/2 - th1) * HEADER_WIDTH/2;

    // Right joint wheel
    double x7 = x2 + cos(M_PI/2 - th1) * HEADER_WIDTH/2;
    double y7 = y2 - sin(M_PI/2 - th1) * HEADER_WIDTH/2;

    // Left front axis
    double x8 = x6 + (HEADER_LENGTH-5.0) * cos(th1);
    double y8 = y6 + (HEADER_LENGTH-5.0) * sin(th1);

    // Right front axis
    double x9 = x7 + (HEADER_LENGTH-5.0) * cos(th1);
    double y9 = y7 + (HEADER_LENGTH-5.0) * sin(th1);

    // Left front
    double x10 = x6 + ((HEADER_LENGTH-5.0) + HL_FRONT) * cos(th1);
    double y10 = y6 + ((HEADER_LENGTH-5.0) + HL_FRONT) * sin(th1);

    // Right front
    double x11 = x7 + ((HEADER_LENGTH-5.0) + HL_FRONT) * cos(th1);
    double y11 = y7 + ((HEADER_LENGTH-5.0) + HL_FRONT) * sin(th1);

    std::list<Point *> points = {
        new Point(x8, y8), new Point(x9, y9), new Point(x6, y6), new Point(x7, y7), new Point(x12, y12),
        new Point(x13, y13), new Point(x10, y10), new Point(x11, y11), new Point(x4, y4), new Point(x5, y5)
    };

    return points;
}


Point * Truck::backMiddleTrailer(Point *pointFront, double th1, double th2) {
    double jpx = pointFront->getX() - cos(th1) * (HEADER_LENGTH-5.0);
    double jpy = pointFront->getY() - sin(th1) * (HEADER_LENGTH-5.0);

    double px = jpx - cos(th2) * (TRAILER_LENGTH + TL_BACK);
    double py = jpy - sin(th2) * (TRAILER_LENGTH + TL_BACK);

    return new Point(px, py);
}
