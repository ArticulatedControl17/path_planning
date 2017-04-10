#ifndef VehicleState_H
#define VehicleState_H


class VehicleState {
    public:
        VehicleState(double nx, double ny, double nth1, double nth2);
        void setValues(double nx, double ny, double nth1, double nth2);

        double x;
        double y;
        double th1;
        double th2;
};


#endif