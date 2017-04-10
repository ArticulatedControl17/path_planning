#include "track_checker.hpp"
#include "helper_functions.hpp"
#include <iostream>

#define dt 25.0

// g++ -o main track_checker.cpp model.cpp Point.cpp -std=gnu++11


InTrack::InTrack(bool in_track, double error) {
    in_track = in_track;
    error = error;
}


TrackChecker::TrackChecker(int **map) {
    truck = new Truck();  // Model used to calculate error
    map = map;            // Map with allowed/not allowed areas - an array of rows, where each row is an array of elements
}


TrackChecker::~TrackChecker() {
    delete truck;
}


// Returns false if the given Point is outside the map, or in a black area
// Otherwise returns true
bool TrackChecker::isAllowed(Point *point) {
    // Outside the map
    if (point->x < 0 or point->y < 0 or point->x >= MAP_WIDTH or point->y >= MAP_HEIGHT) {
        return false;

        // In black area
        if (map[int(point->y)][int(point->x)] == 0) {
            return false;
        }
    }

    return true;
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
    double totError = 0.0;
    bool inPadding = false;
        
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

    Point **between_back_wheel_right = getPointsInBetween(truck->right_back_wheel, prev->right_back_wheel, dt/4);
    Point **between_back_wheel_left = getPointsInBetween(truck->left_back_wheel, prev->left_back_wheel, dt/4);

    Point **between_front_wheel_right = getPointsInBetween(truck->right_front_wheel, prev->right_front_wheel, dt/4);
    Point **between_front_wheel_left = getPointsInBetween(truck->left_front_wheel, prev->left_front_wheel, dt/4);

    Point **between_front_right = getPointsInBetween(truck->right_front, prev->right_front, dt/4);
    Point **between_front_left = getPointsInBetween(truck->left_front, prev->left_front, dt/4);

    Point **between_back_right = getPointsInBetween(truck->right_back, prev->right_back, dt/4);
    Point **between_back_left = getPointsInBetween(truck->left_back, prev->left_back, dt/4);

    bool right_front_inPadding = false;
    bool left_front_inPadding = false;
    bool right_back_inPadding = false;
    bool left_back_inPadding = false;

    // Check right back wheel
    /*for point in between_back_wheel_right:
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x] ==0:
                return (False, True)
            if self.map[y][x] ==2:
                right_front_inPadding = True
*/
    // Check left back wheel
    // Check right front wheel
    // Check left front wheel
    // Check right front
    // Check left front
    // Check right back
    // Check left back


/*




        #header
        right_back_wheel = Point(points[5][0], points[5][1])    right_back_axis
        left_back_wheel = Point(points[4][0], points[4][1])     left_back_axis
        right_front_wheel = Point(points[1][0], points[1][1])   right_front_axis
        left_front_wheel = Point(points[0][0], points[0][1])    left_front_axis

        left_front = Point(points[6][0], points[6][1])      left_front
        right_front = Point(points[7][0], points[7][1])     right_front

        #trailer
        left_back = Point(points[8][0], points[8][1])       left_back
        right_back = Point(points[9][0], points[9][1])      right_back


##########################################

        #check right back wheel

        #Check left back wheel
        for (x,y) in between_back_wheel_left:
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x]==0:
                return (False, True)
            if self.map[y][x]==2:
                left_front_inPadding = True

        #check right front wheel
        for (x,y) in between_front_wheel_right:
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x] ==0:
                return (False, True)
            if self.map[y][x] ==2:
                right_front_inPadding = True

        #check left front wheel
        for (x,y) in between_front_wheel_left:
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x] ==0:
                return (False, True)
            if self.map[y][x] ==2:
                left_front_inPadding = True

        #trailer back
        for (x,y) in between_back_right:
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x] ==0:
                return (False, True)
            if self.map[y][x] ==2:
                right_back_inPadding = True

        #trailer back
        for (x,y) in between_back_left:
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x]==0:
                return (False, True)
            if self.map[y][x]==2:
                left_back_inPadding = True

        #check right front wheel
        for (x,y) in between_front_right:
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x] ==0:
                return (False, True)
            if self.map[y][x] ==2:
                inPadding = True

        #check left front wheel
        for (x,y) in between_front_left:
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x] ==0:
                return (False, True)
            if self.map[y][x] ==2:
                inPadding = True

         #calculate avarege error for key wheels

        right_front_wheel_err = front_ec.calculateError(right_front) - HEADER_WIDTH/2
        left_front_wheel_err = front_ec.calculateError(left_front) + HEADER_WIDTH/2
        right_back_wheel_err = back_ec.calculateError(right_back_wheel) - TRAILER_WIDTH/2
        left_back_wheel_err = back_ec.calculateError(left_back_wheel) + TRAILER_WIDTH/2

        if abs(right_front_wheel_err) > LANE_WIDTH/2:
            right_front_wheel_err = right_front_wheel_err * OTHERLANE_WEIGHT
        if right_front_inPadding:
            print "Right in padding", toPoint.x, toPoint.y

            right_front_wheel_err = (abs(right_front_wheel_err)+20) * PADDING_WEIGHT
        if abs(left_front_wheel_err) > LANE_WIDTH/2:
            left_front_wheel_err = left_front_wheel_err * OTHERLANE_WEIGHT
        if left_front_inPadding:
            print "left in padding", toPoint.x, toPoint.y
            left_front_wheel_err = (abs(left_front_wheel_err)+20) * PADDING_WEIGHT
        if abs(right_back_wheel_err) > LANE_WIDTH/2:
            right_back_wheel_err = right_back_wheel_err * OTHERLANE_WEIGHT
        if right_back_inPadding:
            right_back_wheel_err = (abs(right_back_wheel_err)+20) * PADDING_WEIGHT
        if abs(left_back_wheel_err) > LANE_WIDTH/2:
            left_back_wheel_err = left_back_wheel_err * OTHERLANE_WEIGHT
        if left_back_inPadding:
            left_back_wheel_err = (abs(left_back_wheel_err)+20) * PADDING_WEIGHT


        totError = abs(right_front_wheel_err) + abs(left_front_wheel_err) + abs(right_back_wheel_err) + abs(left_back_wheel_err)
*/
    delete prev;
    return new InTrack(true, totError);
}


Point ** TrackChecker::getPointsInBetween(Point *p1, Point *p2, double n) {
    return {};
}


void TrackChecker::setMap(int **map) {
    map = map;
}


int main() {
    int **map;
    map = new int*[10];
    for (int i = 0; i < 10; i++) {
        map[i] = new int[5];
        for (int j = 0; j < 5; j++) {
            map[i][j] = i+j;
        }
    }

    std::cout << sizeof(**map) << std::endl;

    TrackChecker *tc = new TrackChecker(map);
}



/*
    def getPointsInBetween(self, p1, p2, n):
        p1x, p1y = p1
        p2x, p2y = p2

        dx = p2x - p1x
        dy = p2y - p1y

        stepX = dx/float(n-1)
        stepY = dy/float(n-1)

        points = []
        for i in range(0, n):
            x = int(round(p1x + i * stepX))
            y = int(round(p1y + i * stepY))
            points.append((x, y))

        return points

    def setMap(self, mapp):
        self.map = mapp
*/
