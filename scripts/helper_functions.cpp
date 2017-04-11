#include <stdlib.h>
#include <iostream>
#include <math.h>
#include "helper_functions.hpp"
#include "vehicleState.hpp"


// Converting from degrees to radians
float radians(float degrees) {
    return degrees * M_PI / 180;
}

VehicleState *calculateNextState(VehicleState *vs, double dd, double steering_angle_rad){

    double theta1 = vs->th1;
    double theta2 = vs->th2;
    double dt1 = (dd * tan(steering_angle_rad)) / HEADER_LENGTH;
    double next_theta1 = theta1 + dt1;

    double r = 5.0;
    double x = sqrt(dd*dd + (r*dt1)*(r*dt1));

    double t1_avg = (theta1 + next_theta1)/2;

    double next_theta2 = theta2 + (x * sin((atan2(r*dt1, dd) + theta1 - theta2))) / TRAILER_LENGTH;

    double dx = vs->x - HEADER_LENGTH * cos(theta1);
    double dy = vs->y - HEADER_LENGTH * sin(theta1);

    double next_x = dx + dd * cos(t1_avg) + HEADER_LENGTH * cos(next_theta1);
    double next_y = dy + dd * sin(t1_avg) + HEADER_LENGTH * sin(next_theta1);

    return new VehicleState(next_x, next_y, next_theta1, next_theta2);

}

VehicleState *calculate_steering(double steering_min, double steering_max, double dd, int iters, double target_error, VehicleState *vs, ErrorCalc* ec){
    //TODO: might not be working properly?
    //Calculates a point within 1 unit of the optimal path, return the closest possibility if we cant find the optimal path
    double steering_new = (steering_min + steering_max)/2;
    VehicleState *new_vs = calculateNextState(vs, dd, steering_new);
    double error = ec->calculateError(new_vs->x, new_vs->y);
    if (abs(error-target_error)<0.1 || iters==0){
        return new_vs;
    }
    else if (error<target_error){
        //search right
        delete new_vs;
        return calculate_steering(steering_new, steering_max, dd, iters-1, target_error, vs, ec);
    } else{
        //search left
        delete new_vs;
        return calculate_steering(steering_min, steering_new, dd, iters-1, target_error, vs, ec);
    }
}

VehicleState *rounding(VehicleState *vs,double modPoint,double modTheta){

    double x = vs->x;
    double m_x = fmod(x, modPoint);
    std::cout << "fmod, x: " << m_x <<std::endl;
    if (m_x >= modPoint/2){   //round up
        x = x - m_x + modPoint;
    } else{                   //round down
        x = x - m_x;
    }

    double y = vs->y;
    double m_y = fmod(y, modPoint);
    if (m_y >= modPoint/2){   //round up
        y = y - m_y + modPoint;
    } else {                   //round down
        y = y - m_y;
    }

    double th1 = round(vs->th1*10)/10;
    double m_t1 = round(fmod(th1, modTheta)*10)/10;
    if(m_t1 >= modTheta/2){   //round up
        th1 = th1-m_t1 + modTheta;
    } else{                   //round down
        th1 = th1 - m_t1;
    }

    double th2 = round(vs->th2*10)/10;
    double m_t2 = round(fmod(th2, modTheta)*10)/10;
    if(m_t2 >= modTheta/2){   //round up
        th2 = th2-m_t2 + modTheta;
    } else{                   //round down
        th2 = th2 - m_t2;
    }

    return new VehicleState(x, y, th1, th2);
}

/*
int main(){
    VehicleState * vs = new VehicleState(4.0, 7.5, 0.0, 0.0);
    VehicleState *vs2 = calculateNextState(vs, 10, 0.0);
    std::cout << "next state vs2, x: " << vs2->x << " y: " << vs2->y << " th1: " << vs2->th1 << "th2: " << vs2->th2 <<std::endl;
    std::cout << "next state vs, x: " << vs->x << " y: " << vs->y << " th1: " << vs->th1 << "th2: " << vs->th2 <<std::endl;

    std::list<Point*> startList;

    Point *point1 = new Point(0.0, 0.0);
    startList.push_front(point1);
    Point *point2 = new Point(5.0, 0.0);
    startList.push_back(point2);
    Point *point3 = new Point(100.0, 0.0);
    startList.push_back(point3);
    ErrorCalc *ec = new ErrorCalc(startList);

    VehicleState *vs3 = calculate_steering(16, -18, 20, 10, 0, vs, ec);
    std::cout << "calculate steering x: " << vs3->x << " y: " << vs3->y << " th1: " << vs3->th1 << "th2: " << vs3->th2 <<std::endl;

    rounding(vs, 3.0, 0.3);
    std::cout << "rounding, x: " << vs->x << " y: " << vs->y << " th1: " << vs->th1 << "th2: " << vs->th2 <<std::endl;
}

*/
