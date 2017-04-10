#ifndef VehicleState_H
#define VehicleState_H


class VehicleState {
    public:
        VehicleState(double nx, double ny, double nth1, double nth2);
        double getX();
        double getY();
        double getTh1();
        double getTh2();
        void setValues(double nx, double ny, double nth1, double nth2);

    private:
        double x;
        double y;
        double th1;
        double th2;
};


#endif