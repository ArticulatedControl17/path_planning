#include "vehicleState.hpp"
#include "error_calc.hpp"

#ifndef Helper_H
#define Helper_H

#define speed   1
#define dt      25.0

#define HEADER_FRONTAXIS_TO_JOINT 22.0
#define HEADER_BACKAXIS_TO_JOINT 5.0

#define HEADER_LENGTH   27.0
#define TRAILER_LENGTH  49.625
#define HL_FRONT        9.5
#define TL_BACK         13.625

#define HEADER_WIDTH    18
#define TRAILER_WIDTH   18

#define MAX_LEFT_ANGLE  16
#define MAX_RIGHT_ANGLE -19

#define LANE_WIDTH          19
#define OUTSIDE_TURN_ERROR  9
#define OTHERLANE_WEIGHT    10
#define PADDING_WEIGHT      20

#define MAP_WIDTH	490
#define MAP_HEIGHT	965


double radians(double degrees);

VehicleState *calculateNextState(VehicleState *vs, double dd, double steering_angle_rad);
VehicleState *calculate_steering(double steering_min, double steering_max, double dd, int iters, double target_error, VehicleState *vs, ErrorCalc *ec);
VehicleState *rounding(VehicleState *vs,double modPoint,double modTheta);


#endif
