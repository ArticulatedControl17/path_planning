#ifndef ErrorCalc_H
#define ErrorCalc_H
#include "Point.hpp"
#include <list>

class ErrorCalc {
    private:
        std::list<Point*> queue;
        Point *pp1;
        Point *pp2 ;
    public:
        ErrorCalc(std::list<Point*> startList);
        double calculateError(double x, double y);
        double getDirection();
        double isAtEnd();
        double getMaxDistPoint(Point *point);
        bool isAboveEnd(Point *begin, Point *end, double x, double y);
        bool is_next_left();
        ErrorCalc getCopy();
};

#endif
