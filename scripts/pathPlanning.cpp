#include "pathPlanning.hpp"


PathPlanner::PathPlanner(int *map){
    std::cout << "in constructor" << std::endl;
    track_checker = new TrackChecker(map);
    std::cout << "end constructor" << std::endl;
    //visited_pub = n.advertise<custom_msgs::Position>("visited_node", 1000);
    //to_visit_pub = n.advertise<custom_msgs::Position>("to_visit_node", 1000);
}

std::list<VehicleState*> PathPlanner::getPath(VehicleState *startVs, Point *endPoint, Point *secondEndPoint
            , double MAX_EXECUTION_TIME, double modPoint, double modTheta, bool returnsIfFeasible){
    std::cout << "in getPath" << std::endl;

    theta1 = startVs->th1;
    theta2 = startVs->th2;
    front_ec = new ErrorCalc(optimalPath);
    back_ec = new ErrorCalc(optimalPath);
    pos = new Point(startVs->x, startVs->y);
    Point * startPoint = new Point(startVs->x, startVs->y);

    //Reset path's
    //TODO: Remove before finishing
    toVisit = new std::stack<VehicleState_error*>();
    visited = new std::unordered_set<VehicleState>();
    fromPoints = new std::unordered_map<Point , VehicleState_error>();

    addPossiblePathes(true);

    //TODO: add rospy and time condition
    while(toVisit->size()>0){
        VehicleState_error *new_visit;
        //loop until all possible nodes have been visited
        while(1){
            if(toVisit->size()== 0){
                //No solution found
                std::cout << "no soluton found" << std::endl;
                std::list<VehicleState*> *ret = new std::list<VehicleState*>();
                return *ret;
            }
            new_visit = toVisit->top();
            toVisit->pop();
            //msg.x = new_visit->vs->x;
            //msg.y = new_visit->vs->y;
            //visited_pub.publish(msg);
            //round to not having to visit every mm, to make it faster
            VehicleState *rounded_vs = rounding(new_visit->vs, modPoint, modTheta);
            //TODO: probably gonna compare on the reference, not the values
            auto iter = visited->find(*rounded_vs);
            if( iter == visited->end()){
                break;
            }
            else{
                delete new_visit;
            }
        }
        //found new node to visit
        pos->x = new_visit->vs->x;
        pos->y = new_visit->vs->y;
        theta1 = new_visit->vs->th1;
        theta2 = new_visit->vs->th2;
        front_ec = new_visit->front_ec;
        back_ec = new_visit->back_ec;
        delete new_visit;

        //std::cout << "visiting x: " << pos->x << "y: " << pos->y << std::endl;

        double dist = sqrt((endPoint->x - pos->x)*(endPoint->x - pos->x) + (endPoint->y - pos->y)*(endPoint->y - pos->y));
        if(front_ec->isAboveEnd(secondEndPoint, endPoint, pos->x, pos->y) && dist <1*dt && front_ec->isAtEnd()){ //checks if we are above a line of the two last points
            //reached end, gather the path
            std::cout << "reached end, Gathering solution" << std::endl;

            // If flag 'returnsIfFeasible' is set,
            // Returns list that translates to true
            if (returnsIfFeasible) {
                std::list<VehicleState*> *ret = new std::list<VehicleState*>();
                ret->push_back(new VehicleState(1, 1, 1, 1));
                return *ret;
            }

            double totError = gatherError(startPoint, pos);
            //Gather a new optimized path
            //return gatherPath( startPoint, endPoint, theta1, theta2);
            RecalculatePath *recalculate_path = new RecalculatePath(track_checker);
            std::list<VehicleState*> path = recalculate_path->calculate_path(startVs, endPoint, secondEndPoint, totError, new ErrorCalc(optimalPath), modPoint, modTheta);
            if(path.size()==0){
                return gatherPath( startPoint, endPoint, theta1, theta2);
            } else {
                return path;
            }
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
            VehicleState *in_r_vs = new VehicleState(pos->x, pos->y, theta1, theta2);
            VehicleState *rounded_vs = rounding(in_r_vs, modPoint, modTheta);
            delete in_r_vs;
            //mark the previous node/state as visited
            //TODO: add the pointer? will that work?
            visited->insert(*rounded_vs);
        }
    }
    std::cout << "no soluton found" << std::endl;
    std::list<VehicleState*> *ret = new std::list<VehicleState*>();

    // If flag 'returnsIfFeasible' is set,
    // Returns list that translates to false
    if (returnsIfFeasible) {
        ret->push_back(new VehicleState(0, 0, 0, 0));
    }

    return *ret;

}

void PathPlanner::addPossiblePathes(bool leftFirst){

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
    delete current_vs;
    //Strait
    Point *strait_point = new Point(strait_vs->x, strait_vs->y);
    InTrack * strait_it = track_checker->checkIfInTrack(pos, theta1, theta2, strait_point, strait_vs->th1, strait_vs->th2, front_ec, back_ec);
    if (strait_it->in_track){
        addState(strait_point, strait_vs->th1, strait_vs->th2, strait_it->error);
    }else {
        delete strait_point;
    }
    delete strait_vs;
    Point *right_point = new Point(right_vs->x, right_vs->y);
    Point *left_point = new Point(left_vs->x, left_vs->y);
    if (leftFirst){
        //Right
        InTrack *right_it = track_checker->checkIfInTrack(pos, theta1, theta2, right_point, right_vs->th1, right_vs->th2, front_ec, back_ec);
        if (right_it->in_track){
            addState(right_point, right_vs->th1, right_vs->th2, right_it->error);
        } else {
            delete right_point;
        }
        //Left
        InTrack * left_it = track_checker->checkIfInTrack(pos, theta1, theta2, left_point, left_vs->th1, left_vs->th2, front_ec, back_ec);
        if (left_it->in_track){
            addState(left_point, left_vs->th1, left_vs->th2, left_it->error);
        } else {
            delete left_point;
        }
    } else {
        //Left
        InTrack * left_it = track_checker->checkIfInTrack(pos, theta1, theta2, left_point, left_vs->th1, left_vs->th2, front_ec, back_ec);
        if(left_it->in_track){
            addState(left_point, left_vs->th1, left_vs->th2, left_it->error);
        } else {delete left_point;}
        //Right
        InTrack *right_it = track_checker->checkIfInTrack(pos, theta1, theta2, right_point, right_vs->th1, right_vs->th2, front_ec, back_ec);
        if(right_it->in_track){
            addState(right_point, right_vs->th1, right_vs->th2, right_it->error);
        }else {delete right_point;}
    }
    delete right_vs;
    delete left_vs;
    //Optimal outside turn
    Point *optimal_outside_point = new Point(optimal_outside_vs->x, optimal_outside_vs->y);
    InTrack * optimal_outside_it = track_checker->checkIfInTrack(pos, theta1, theta2, optimal_outside_point, optimal_outside_vs->th1, optimal_outside_vs->th2, front_ec, back_ec);
    if (optimal_outside_it->in_track){
        addState(optimal_outside_point, optimal_outside_vs->th1, optimal_outside_vs->th2, optimal_outside_it->error);
    } else {delete optimal_outside_point;}
    delete optimal_outside_vs;
    //Optimal
    Point *optimal_point = new Point(optimal_vs->x, optimal_vs->y);
    InTrack * optimal_it = track_checker->checkIfInTrack(pos, theta1, theta2, optimal_point, optimal_vs->th1, optimal_vs->th2, front_ec, back_ec);
    if(optimal_it->in_track){
        addState(optimal_point, optimal_vs->th1, optimal_vs->th2, optimal_it->error);
    } else {
        delete optimal_point;
    }
    delete optimal_vs;
}

std::list<VehicleState*> PathPlanner::gatherPath(Point *startPoint, Point *endPoint, double end_theta1, double end_theta2){
    std::list<VehicleState*> path;
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
        auto iter = fromPoints->find(*f_point);
        VehicleState_error vs_er = iter->second;
        prex=vs_er.vs->x;
        prey=vs_er.vs->y;
        pret1 = vs_er.vs->th1;
        pret2 = vs_er.vs->th2;
        delete f_point;
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
        auto iter = fromPoints->find(*f_point);
        VehicleState_error vs_er = iter->second;
        prex=vs_er.vs->x;
        prey=vs_er.vs->y;
        totErr= totErr+ fabs(vs_er.error);
        delete f_point;
    }
    return totErr;
}

void PathPlanner::addState(Point *point, double th1, double th2, double error){
    //add the vector as an adjacent vector to the previous vector in the graph
    //TODO: shuld not be same error for fromPoint and toVisit?
    VehicleState_error * vs = new VehicleState_error(new VehicleState(pos->x, pos->y, theta1, theta2), error, front_ec->getCopy(), back_ec->getCopy());
    fromPoints->insert({*point, *vs});
    VehicleState_error * vs_to = new VehicleState_error(new VehicleState(point->x, point->y, th1, th2), error, front_ec->getCopy(), back_ec->getCopy());
    toVisit->push(vs_to);

    //msg.x = point->x;
    //msg.y = point->y;
    //to_visit_pub.publish(msg);
}

void PathPlanner::setOptimalpath(std::list<Point*> path){
    //std::cout << "SET OPTIMAL PATH C++" << std::endl;
    optimalPath = path;
    //delete front_ec;
    //delete back_ec;
    front_ec = new ErrorCalc(path);
    back_ec = new ErrorCalc(path);
    //std::cout << "END SET OPTIMAL PATH C++" << std::endl;
}

void PathPlanner::setMap(int *mat){
    track_checker->setMap(mat);
}

bool PathPlanner::checkIfInTrack(VehicleState *vs){
    Point *n_p = new Point(vs->x, vs->y);
    return track_checker->checkIfInTrack2(n_p, vs->th1, vs->th2);
}



/*
int main() {

    int **map;
    map = new int*[10];

    for (int i = 0; i < 10; i++) {
        map[i] = new int[5];

        for (int j = 0; j < 5; j++) {
            map[i][j] = i+j;
        }
    }

    PathPlanner *pp = new PathPlanner(map);
    std::cout << "Created PathPlanner" << std::endl;
    std::cout << "Map: " << **(pp->track_checker->map) << std::endl;
    int **matrix = pp->track_checker->map;

    std::string s = "";
    std::cout << "Map: ";

    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 5; j++) {
            std::cout << matrix[i][j];
        }
    }

    std::cout << std::endl;

}
*/


// g++ -o main -std=gnu++11 pathPlanning.cpp Point.cpp track_checker.cpp vehicleState.cpp vehicleState_error.cpp helper_functions.cpp error_calc.cpp model.cpp

/*
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

    */
