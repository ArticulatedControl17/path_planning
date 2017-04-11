#include "VehicleState_error"

VehicleState_error::VehicleState_error(VehicleState* nvs, double nerror, ErrorCalc nfront_ec, ErrorCalc nback_ec){
    vs = nvs;
    error = nerror;
    front_ec = nfront_ec;
    back_ec = nback_ec;
}
