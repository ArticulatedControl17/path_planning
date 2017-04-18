#include <cstddef>
#include <string>
#ifndef Point_H
#define Point_H


class Point {
    public:
        Point(double x, double y);
        void setValues(double x, double y);
        bool operator==(const Point &other) const;

        //operator std::string(){ return std::to_string(x) + ", " + std::to_string(y); };

        double x;
        double y;
};


#endif
