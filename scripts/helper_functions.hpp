#include "vehicleState.hpp"
#include "error_calc.hpp"

#ifndef Helper_H
#define Helper_H

#define speed   1
#define dt      25

#define HEADER_LENGTH   27.0
#define TRAILER_LENGTH  44.5 + 10.25 / 2 // + 2.5
#define HL_FRONT        9.5
#define TL_BACK         10.25 / 2 + 8.5 // - 2.5

#define HEADER_WIDTH    18
#define TRAILER_WIDTH   18

#define MAX_LEFT_ANGLE  16
#define MAX_RIGHT_ANGLE -18

#define LANE_WIDTH          19
#define OUTSIDE_TURN_ERROR  9 // 3
#define OTHERLANE_WEIGHT    10
#define PADDING_WEIGHT      20


float radians(float degrees);

VehicleState *calculateNextState(VehicleState *vs, double dd, double steering_angle_rad);
VehicleState *calculate_steering(double steering_min, double steering_max, double dd, int iters, double target_error, VehicleState *vs, ErrorCalc *ec);
VehicleState *rounding(VehicleState *vs,double modPoint,double modTheta);


#endif
