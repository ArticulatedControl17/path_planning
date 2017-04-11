#include "pathPlanning.hpp"


PathPlanner::PathPlanner(int **map){
    track_checker = new TrackChecker(map);
}

std::list<VehicleState*> PathPlanner::getPath(VehicleState *startVs, Point *endPoint, Point *secondEndPoint
    , double MAX_EXECUTION_TIME, double modPoint, double modTheta){

    //TODO: add recalculate_path
    theta1 = startVs->th1;
    theta2 = startVs->th2;
    front_ec = new ErrorCalc(optimalPath);
    back_ec = new ErrorCalc(optimalPath);
    pos = new Point(startVs->x, startVs->y);

    addPossiblePathes(true);

    //TODO: add rospy and time condition
    while(toVisit.size()>0){
        VehicleState_error *new_visit;
        //loop until all possible nodes have been visited
        while(1){
            if(toVisit.size()== 0){
                break;
            }
            new_visit = toVisit.top();
            toVisit.pop();
            //TODO: ros publish
            //round to not having to visit every mm, to make it faster
            VehicleState *rounded_vs = rounding(new_visit->vs, modPoint, modTheta);
            //TODO: probably gonna compare on the reference, not the values
            auto iter = visited.find(*rounded_vs);
            if( iter == visited.end()){
                break;
            }
        }
        //found new node to visit
        pos->x = new_visit->vs->x;
        pos->y = new_visit->vs->y;
        theta1 = new_visit->vs->th1;
        theta2 = new_visit->vs->th2;
        front_ec = new_visit->front_ec;
        back_ec = new_visit->back_ec;

        double dist = sqrt((endPoint->x - pos->x)*(endPoint->x - pos->x) + (endPoint->y - pos->y)*(endPoint->y - pos->y));
        if(front_ec->isAboveEnd(secondEndPoint,endPoint, pos->x, pos->y) && dist <1*dt && front_ec->isAtEnd()){ //checks if we are above a line of the two last points
            //reached end, gather the path
            std::cout << "reached end, Gathering solution" << std::endl;

            double totError = gatherError(new Point(startVs->x, startVs->y), pos);
            //Gather a new optimized path
            //return self.gatherPath(Point(vs.x, vs.y), endPoint,self.theta1, self.theta2)
            //TODO: add recalcpath
            //std::list<Point*> path = recalculate_path->calculate_path(new Point(vs->x, vs->y), secondEndPoint, endPoint, dt, vs->theta1, vs->theta2, totError, new ErrorCalc(optimal_path), modPoint, modTheta);
            //part = self.recalculate_path.calculate_path(Point(vs.x, vs.y), secondEndPoint, endPoint, self.dt, vs.theta1, vs.theta2, totError, error_calc.errorCalc(self.optimal_path))
            //if part == []{
            return gatherPath( new Point(startVs->x, startVs->y), endPoint, theta1, theta2);
            //} else {
            //    return part;
            //}
            //TODO: Destoy all created objects before return?
        } else {
            //we have not yet found a solution, search for new possible nodes
            double currentError = front_ec->calculateError(pos->x, pos->y); //check if we are left or right of the optimal path

            if (currentError<0){
                //Go right
                //check if the nodes are within the allowed track
                addPossiblePathes(false);
            } else {
                //Go left
                //check if the nodes are within the allowed track
                addPossiblePathes(true);
            }

            //round to not having to visit every mm, for making it faster
            VehicleState *rounded_vs = rounding(new VehicleState(pos->x, pos->y, theta1, theta2), modPoint, modTheta);
            //mark the previous node/state as visited
            //TODO: add the pointer? will that work?
            visited.insert(*rounded_vs);
        }
    }
    std::cout << "no soluton found" << std::endl;
    std::list<VehicleState*> *ret = new std::list<VehicleState*>();
    return *ret;

}

void PathPlanner::addPossiblePathes(bool leftFirst){
    //TODO: Deconstruct created objects


    double dd = speed * dt;
    //add all possible pathes from pos, theta1 and theta2
    double steering_angle_rad = 0;
    VehicleState *current_vs = new VehicleState(pos->x, pos->y, theta1, theta2);

    VehicleState *strait_vs = calculateNextState(current_vs, dd, steering_angle_rad);
    //going right
    steering_angle_rad = radians(MAX_RIGHT_ANGLE); //max right angle
    VehicleState *right_vs = calculateNextState(current_vs, dd, steering_angle_rad);
    //going left
    steering_angle_rad = radians(MAX_LEFT_ANGLE); //max left angle
    VehicleState *left_vs = calculateNextState(current_vs, dd, steering_angle_rad);
    //finding optimal path
    VehicleState *optimal_vs = calculate_steering(radians(MAX_LEFT_ANGLE), radians(MAX_RIGHT_ANGLE), dd, 10, 0, current_vs, front_ec);
    //Optimal outside turn
    bool goingLeft = front_ec->is_next_left();
    VehicleState *optimal_outside_vs;
    if (goingLeft){
        optimal_outside_vs = calculate_steering(radians(MAX_LEFT_ANGLE), radians(MAX_RIGHT_ANGLE), dd, 10, OUTSIDE_TURN_ERROR, current_vs, front_ec);
    } else {
        optimal_outside_vs = calculate_steering(radians(MAX_LEFT_ANGLE), radians(MAX_RIGHT_ANGLE), dd, 10, -OUTSIDE_TURN_ERROR, current_vs, front_ec);
    }
    //Strait
    Point *strait_point = new Point(strait_vs->x, strait_vs->y);
    InTrack * strait_it = track_checker->checkIfInTrack(pos, theta1, theta2, strait_point, strait_vs->th1, strait_vs->th2, front_ec, back_ec);
    if (strait_it->in_track){
        addState(strait_point, strait_vs->th1, strait_vs->th2, strait_it->error);
    }
    Point *right_point = new Point(right_vs->x, right_vs->y);
    Point *left_point = new Point(left_vs->x, left_vs->y);
    if (leftFirst){
        //Right
        InTrack *right_it = track_checker->checkIfInTrack(pos, theta1, theta2, right_point, right_vs->th1, right_vs->th2, front_ec, back_ec);
        if (right_it->in_track){
            addState(right_point, right_vs->th1, right_vs->th2, right_it->error);
        }
        //Left
        InTrack * left_it = track_checker->checkIfInTrack(pos, theta1, theta2, left_point, left_vs->th1, left_vs->th2, front_ec, back_ec);
        if (left_it->in_track){
            addState(left_point, left_vs->th1, left_vs->th2, left_it->error);
        }
    } else {
        //Left
        InTrack * left_it = track_checker->checkIfInTrack(pos, theta1, theta2, left_point, left_vs->th1, left_vs->th2, front_ec, back_ec);
        if(left_it->in_track){
            addState(left_point, left_vs->th1, left_vs->th2, left_it->error);
        }
        //Right
        InTrack *right_it = track_checker->checkIfInTrack(pos, theta1, theta2, right_point, right_vs->th1, right_vs->th2, front_ec, back_ec);
        if(right_it->in_track){
            addState(right_point, right_vs->th1, right_vs->th2, right_it->error);
        }
    }
    //Optimal outside turn
    Point *optimal_outside_point = new Point(optimal_outside_vs->x, optimal_outside_vs->y);
    InTrack * optimal_outside_it = track_checker->checkIfInTrack(pos, theta1, theta2, optimal_outside_point, optimal_outside_vs->th1, optimal_outside_vs->th2, front_ec, back_ec);
    if (optimal_outside_it->in_track){
        addState(optimal_outside_point, optimal_outside_vs->th1, optimal_outside_vs->th2, optimal_outside_it->error);
    }
    //Optimal
    Point *optimal_point = new Point(optimal_vs->x, optimal_vs->y);
    InTrack * optimal_it = track_checker->checkIfInTrack(pos, theta1, theta2, optimal_point, optimal_vs->th1, optimal_vs->th2, front_ec, back_ec);
    if(optimal_it->in_track){
        addState(optimal_point, optimal_vs->th1, optimal_vs->th2, optimal_it->error);
    }
}

std::list<VehicleState*> PathPlanner::gatherPath(Point *startPoint, Point *endPoint, double end_theta1, double end_theta2){
    std::list<VehicleState*> path;
    //TODO: fix lookuptables
    //TODO: not needed?
    //fromPoints.insert({*endPoint, new VehicleState(pos->x, pos->y, theta1, theta2)});
    double prex = pos->x;
    double prey = pos->y;
    double pret1 = theta1;
    double pret2 = theta2;
    Point *f_point;
    while (!(prex== startPoint->x && prey == startPoint->y)){
        path.push_front(new VehicleState(prex, prey, pret1, pret2));
        //TODO: fix lookuptables
        f_point = new Point(prex,prey);
        auto iter = fromPoints.find(*f_point);
        VehicleState_error vs_er = iter->second;
        prex=vs_er.vs->x;
        prey=vs_er.vs->y;
        pret1 = vs_er.vs->th1;
        pret2 = vs_er.vs->th2;
    }
    return path;
}

double PathPlanner::gatherError(Point *startPoint, Point *endPoint){
    double prex = endPoint->x;
    double prey = endPoint->y;
    double totErr = 0;
    Point *f_point;
    while (!(prex== startPoint->x && prey == startPoint->y)){
        f_point = new Point(prex,prey);
        auto iter = fromPoints.find(*f_point);
        VehicleState_error vs_er = iter->second;
        prex=vs_er.vs->x;
        prey=vs_er.vs->y;
        totErr= totErr+ abs(vs_er.error);
    }
    return totErr;
}

void PathPlanner::addState(Point *point, double th1, double th2, double error){
    //add the vector as an adjacent vector to the previous vector in the graph
    //TODO: fix dict function
    //TODO: shuld not be same error for fromPoint and toVisit?
    //TODO: errorcalc copys not needed in fromPoints
    VehicleState_error * vs = new VehicleState_error(new VehicleState(pos->x, pos->y, theta1, theta2), error, front_ec->getCopy(), back_ec->getCopy());
    fromPoints.insert({*point, *vs});
    VehicleState_error * vs_to = new VehicleState_error(new VehicleState(point->x, point->y, th1, th2), error, front_ec->getCopy(), back_ec->getCopy());
    toVisit.push(vs_to);
    //TODO: fix publisher
    //to_visit_pub.publish(Position(point.x, point.y))
}

void PathPlanner::setOptimalpath(std::list<Point*> path){
    optimalPath = path;
    front_ec = new ErrorCalc(path);
    back_ec = new ErrorCalc(path);
}

void PathPlanner::setMap(int **mat){
    track_checker->setMap(mat);
}

bool PathPlanner::checkIfInTrack(VehicleState *vs){
    return track_checker->checkIfInTrack2(new Point(vs->x, vs->y), vs->th1, vs->th2);
}



int main(){
    
    VehicleState * vs = new VehicleState(4.0, 7.5, 0.0, 0.0);
    Point *p_end = new Point(500,0);
    Point *p_snd_end = new Point(450,0);



    std::list<VehicleState*> getPath(vs, p_end, p_snd_end, 10, 3.0, 0.3)

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
