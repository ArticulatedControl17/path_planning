#ifndef Point_H
#define Point_H


class Point {
    public:
        Point(double x, double y);
        double getX();
        double getY();
        void setValues(double x, double y);

    private:
        double x;
        double y;
};


#endif