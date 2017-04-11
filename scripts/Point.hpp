#include <cstddef>
#ifndef Point_H
#define Point_H


class Point {
    public:
        Point(double x, double y);
        void setValues(double x, double y);
        bool operator==(const Point &other) const;

        double x;
        double y;
};


#endif
