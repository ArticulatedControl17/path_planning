#include "vehicleState.hpp"

class VehicleState_error {
    public:
        VehicleState_error(VehicleState* vs, double error, ErrorCalc front_ec, ErrorCalc back_ec);
        VehicleState* vs;
        double error;
        ErrorCalc *front_ec;
        ErrorCalc *back_ec;

};
