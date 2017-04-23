#include "track_checker.hpp"
#include "helper_functions.hpp"

#include <iostream>
#include <vector>
#include <limits>
#include <math.h>

#define INF std::numeric_limits<double>::infinity()

// g++ -o main track_checker.cpp model.cpp Point.cpp -std=gnu++11


InTrack::InTrack(bool in_track_, double error_) {
    in_track = in_track_;
    error = error_;
}


TrackChecker::TrackChecker(int *map_) {
    truck = new Truck();         // Model used to calculate error
    map = new int*[MAP_HEIGHT];  // Map with allowed/not allowed areas - an array of rows, where each row is an array of elements

    for (int i = 0; i < MAP_HEIGHT; i++) {
        map[i] = new int[MAP_WIDTH];

        for (int j = 0; j < MAP_WIDTH; j++) {
            map[i][j] = map_[i*MAP_WIDTH+j];
        }
    }
}


TrackChecker::~TrackChecker() {
    delete truck;

    for (int i = 0; i < MAP_HEIGHT; i++) {
        free(map[i]);
    }

    free(map);
}


void TrackChecker::setMap(int *map_) {
    for (int i = 0; i < MAP_HEIGHT; i++) {

        for (int j = 0; j < MAP_WIDTH; j++) {
            map[i][j] = map_[i*MAP_WIDTH+j];
        }
    }
}


bool TrackChecker::checkIfInTrack2(Point *toPoint, double th1, double th2) {
    if (not isAllowed(toPoint)) {
        return false;
    }

    truck->setCorners(toPoint, th1, th2);

    // Check front wheels
    if (not isAllowed(truck->right_front) or not isAllowed(truck->left_front)) {
        return false;
    }

    // Check back wheels
    if (not isAllowed(truck->right_back) or not isAllowed(truck->left_back)) {
        return false;
    }

    return true;
}


InTrack * TrackChecker::checkIfInTrack(Point *prevPoint, double prevth1, double prevth2, Point *toPoint, double th1, double th2, ErrorCalc *front_ec, ErrorCalc *back_ec) {

    // Used to avoid going wrong direction, optimal path should be close enugh that this restriction holds
    if (front_ec->getMaxDistPoint(toPoint) > 80) {
        return new InTrack(false, -1.0);
    }

    if (not isAllowed(toPoint)) {
        return new InTrack(false, -1.0);
    }
    Truck *prev = new Truck();
    prev->setCorners(prevPoint, prevth1, prevth2);
    truck->setCorners(toPoint, th1, th2);

    // Calculate average error for key wheels
    double right_front_err = front_ec->calculateError(truck->right_front->x, truck->right_front->y) - HEADER_WIDTH/2;
    double left_front_err = front_ec->calculateError(truck->left_front->x, truck->left_front->y) + HEADER_WIDTH/2;
    double right_back_wheel_err = back_ec->calculateError(truck->right_back_wheel->x, truck->right_back_wheel->y) - TRAILER_WIDTH/2;
    double left_back_wheel_err = back_ec->calculateError(truck->left_back_wheel->x, truck->left_back_wheel->y) + TRAILER_WIDTH/2;

    // Check right front
    right_front_err = getError(truck->right_front, prev->right_front, right_front_err);
    if (right_front_err == INF) {
        return new InTrack(false, -1.0);
    }

    // Check left front
    left_front_err = getError(truck->left_front, prev->left_front, left_front_err);
    if (left_front_err == INF) {
        return new InTrack(false, -1.0);
    }

    // Check right back wheel
    right_back_wheel_err = getError(truck->right_back_wheel, prev->right_back_wheel, right_back_wheel_err);
    if (right_back_wheel_err == INF) {
        return new InTrack(false, -1.0);
    }

    // Check left back wheel
    left_back_wheel_err = getError(truck->left_back_wheel, prev->left_back_wheel, left_back_wheel_err);
    if (left_back_wheel_err == INF) {
        return new InTrack(false, -1.0);
    }

    // Check right front wheel
    if (getError(truck->right_front_wheel, prev->right_front_wheel, 0.0) == INF) {
        return new InTrack(false, -1.0);
    }

    // Check left front wheel
    if (getError(truck->left_front_wheel, prev->left_front_wheel, 0.0) == INF) {
        return new InTrack(false, -1.0);
    }

    // Check right back
    if (getError(truck->right_back, prev->right_back, 0.0) == INF) {
        return new InTrack(false, -1.0);
    }

    // Check left back
    if (getError(truck->left_back, prev->left_back, 0.0) == INF) {
        return new InTrack(false, -1.0);
    }

    double tot_error = fabs(right_front_err) + fabs(left_front_err) + fabs(right_back_wheel_err) + fabs(left_back_wheel_err);
    delete prev;
    return new InTrack(true, tot_error);

}


std::vector<Point *> TrackChecker::getPointsInBetween(Point *p1, Point *p2, double n) {
    std::vector<Point *> points = {};

    double dx = p2->x - p1->x;
    double dy = p2->y - p1->y;

    double stepX = dx/(n-1);
    double stepY = dy/(n-1);

    for (int i = 0; i < n; i++) {
        double x = round(p1->x + i * stepX);
        double y = round(p1->y + i * stepY);
        points.push_back(new Point(x, y));
    }

    return points;
}


// Returns false if the given Point is outside the map, or in a black area
// Otherwise returns true
bool TrackChecker::isAllowed(Point *point) {
    // Outside the map
    if (point->x < 0 or point->y < 0 or point->x >= MAP_WIDTH or point->y >= MAP_HEIGHT) {
        return false;
    }
    // In black area
    if (map[int(point->y)][int(point->x)] == 0) {
        return false;
    }

    return true;
}


// Returns the total error for given checkpoint
double TrackChecker::getError(Point *truck_point, Point *prev_point, double error) {
    double result = 0.0;
    bool in_padding = false;

    double nbr_points = dt/4;
    std::vector<Point *> between = getPointsInBetween(truck_point, prev_point, nbr_points);

    for(std::vector<Point *>::iterator it = between.begin(); it != between.end(); ++it) {
        if (not isAllowed(*it)) {
            result = INF;
        } else {
            if (map[int((*it)->y)][int((*it)->x)] == 2) {
                in_padding = true;
            }
        }
        delete (*it);
    }

    if (result != INF) {
        if (fabs(error) > LANE_WIDTH/2) {
            result = error * OTHERLANE_WEIGHT;
        }

        else if (in_padding) {
            result = (fabs(error) + 20) * PADDING_WEIGHT;
        }
        else{
            result = error;
        }
    }

    between.clear();
    return result;
}
