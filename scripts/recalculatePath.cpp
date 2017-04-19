#include "recalculatePath.hpp"

RecalculatePath::RecalculatePath(TrackChecker *track_checker_){
    track_checker = track_checker_;
}

std::list<VehicleState*> RecalculatePath::calculate_path(VehicleState *startVs, Point *endPoint, Point *secondEndPoint
    , double totError_, ErrorCalc *front_ec_, double modPoint_, double modTheta_){

    std::cout << "startError: " << totError_ << std::endl;

    modPoint = modPoint_;
    modTheta = modTheta_;
    theta1 = startVs->th1;
    theta2 = startVs->th2;
    front_ec = front_ec_;
    back_ec = front_ec->getCopy();
    pos = new Point(startVs->x, startVs->y);
    lowest_error = totError_;
    totError = 0;

    //Reset path's
    //TODO: Remove before finishing
    toVisit = new std::stack<VehicleState_error*>();
    visited = new std::unordered_set<VehicleState>();
    fromPoints = new std::unordered_map<Point , VehicleState>();
    errorList = new std::unordered_map<VehicleState , double>();
    path = new std::list<VehicleState*>();

    //add all possible pathes for the first point before looping
    addPossiblePathes();

    while(toVisit->size()>0){
        VehicleState_error *new_visit;
        //loop until all possible nodes have been visited
        while(1){
            if(toVisit->size()== 0){
                //reached end
                std::cout << "reached end" << std::endl;
                return *path;
            }
            new_visit = toVisit->top();
            toVisit->pop();
            //msg.x = new_visit->vs->x;
            //msg.y = new_visit->vs->y;
            //visited_pub.publish(msg);
            //round to not having to visit every mm, to make it faster
            VehicleState *rounded_vs = rounding(new_visit->vs, modPoint, modTheta);
            //find the previous error for this grid
            auto iter_error_list = errorList->find(*rounded_vs);
            double prev_err = iter_error_list->second;

            //visit the node if we haven't visited it before and we have not reached the upper bound, or visit if we reached same grid with lower error
            auto iter_visited = visited->find(*rounded_vs);
            if((iter_visited == visited->end() && new_visit->totError < lowest_error) || prev_err > new_visit->totError){
                break;
            }
            else{
                delete new_visit;
            }
        }
        double err = new_visit->error;
        //found new node to visit
        pos->x = new_visit->vs->x;
        pos->y = new_visit->vs->y;
        theta1 = new_visit->vs->th1;
        theta2 = new_visit->vs->th2;
        front_ec = new_visit->front_ec;
        back_ec = new_visit->back_ec;
        totError = new_visit->totError;
        delete new_visit;

        //std::cout << "visiting x: " << pos->x << "y: " << pos->y << std::endl;

        double dist = sqrt((endPoint->x - pos->x)*(endPoint->x - pos->x) + (endPoint->y - pos->y)*(endPoint->y - pos->y));
        if(front_ec->isAboveEnd(secondEndPoint,endPoint, pos->x, pos->y) && dist <1*dt && front_ec->isAtEnd()){ //checks if we are above a line of the two last points
            //reached a sloution, gather the path if we are on the optimal path
            double dd = speed * dt;
            VehicleState *cur_vs = new VehicleState(pos->x, pos->y, theta1, theta2);
            VehicleState *check_vs = calculate_steering(radians(MAX_LEFT_ANGLE), radians(MAX_RIGHT_ANGLE), dd, 10, 0, cur_vs, front_ec);
            delete cur_vs;
            double nerror = front_ec->calculateError(check_vs->x, check_vs->y);
            if(abs(nerror)< 1 && totError < lowest_error){
                std::cout << "Found better solution, error: " << totError << std::endl;
                Point *new_point = new Point(check_vs->x, check_vs->y);
                Point *startPoint = new Point(startVs->x, startVs->y);
                path = gatherPath(startPoint, new_point);
                lowest_error = totError;
            }
            //Mark visited
            VehicleState *in_r_vs = new VehicleState(pos->x, pos->y, theta1, theta2);
            VehicleState *rounded_vs = rounding(in_r_vs, modPoint, modTheta);
            delete in_r_vs;
            visited->insert(*rounded_vs);

        } else {
            //we have not yet visited all nodes, search for new possible nodes
            addPossiblePathes();

            //round to not having to visit every mm, for making it faster
            VehicleState *in_r_vs = new VehicleState(pos->x, pos->y, theta1, theta2);
            VehicleState *rounded_vs = rounding(in_r_vs, modPoint, modTheta);
            delete in_r_vs;
            //mark the previous node/state as visited
            visited->insert(*rounded_vs);
        }
    }
    //reached end
    std::cout << "reached end" << std::endl;
    return *path;
}

void RecalculatePath::addPossiblePathes(){
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

    //Left
    Point *left_point = new Point(left_vs->x, left_vs->y);
    InTrack * left_it = track_checker->checkIfInTrack(pos, theta1, theta2, left_point, left_vs->th1, left_vs->th2, front_ec, back_ec);
    if(left_it->in_track && totError + left_it->error <= lowest_error){
        addState(left_point, left_vs->th1, left_vs->th2, left_it->error);
    } else {delete left_point;}
    delete left_vs;

    //Strait
    Point *strait_point = new Point(strait_vs->x, strait_vs->y);
    InTrack * strait_it = track_checker->checkIfInTrack(pos, theta1, theta2, strait_point, strait_vs->th1, strait_vs->th2, front_ec, back_ec);
    if (strait_it->in_track && totError + strait_it->error <= lowest_error){
        addState(strait_point, strait_vs->th1, strait_vs->th2, strait_it->error);
    }else { delete strait_point;}
    delete strait_vs;

    //Right
    Point *right_point = new Point(right_vs->x, right_vs->y);
    InTrack *right_it = track_checker->checkIfInTrack(pos, theta1, theta2, right_point, right_vs->th1, right_vs->th2, front_ec, back_ec);
    if(right_it->in_track && totError + right_it->error <= lowest_error){
        addState(right_point, right_vs->th1, right_vs->th2, right_it->error);
    }else {delete right_point;}
    delete right_vs;

    //Optimal outside turn
    Point *optimal_outside_point = new Point(optimal_outside_vs->x, optimal_outside_vs->y);
    InTrack * optimal_outside_it = track_checker->checkIfInTrack(pos, theta1, theta2, optimal_outside_point, optimal_outside_vs->th1, optimal_outside_vs->th2, front_ec, back_ec);
    if (optimal_outside_it->in_track && totError + optimal_outside_it->error <= lowest_error){
        addState(optimal_outside_point, optimal_outside_vs->th1, optimal_outside_vs->th2, optimal_outside_it->error);
    } else {delete optimal_outside_point;}
    delete optimal_outside_vs;

    //Optimal
    Point *optimal_point = new Point(optimal_vs->x, optimal_vs->y);
    InTrack * optimal_it = track_checker->checkIfInTrack(pos, theta1, theta2, optimal_point, optimal_vs->th1, optimal_vs->th2, front_ec, back_ec);
    if(optimal_it->in_track && totError + optimal_it->error <= lowest_error){
        addState(optimal_point, optimal_vs->th1, optimal_vs->th2, optimal_it->error);
    } else {delete optimal_point;}
    delete optimal_vs;

}

std::list<VehicleState*>* RecalculatePath::gatherPath(Point *startPoint, Point *endPoint){
    std::list<VehicleState*>* path = new std::list<VehicleState*>();
    //fromPoints.insert({*endPoint, new VehicleState(pos->x, pos->y, theta1, theta2)});
    double prex = pos->x;
    double prey = pos->y;
    double pret1 = theta1;
    double pret2 = theta2;
    Point *f_point;
    while (!(prex== startPoint->x && prey == startPoint->y)){
        path->push_front(new VehicleState(prex, prey, pret1, pret2));
        f_point = new Point(prex,prey);
        auto iter = fromPoints->find(*f_point);
        VehicleState vs = iter->second;
        prex=vs.x;
        prey=vs.y;
        pret1 = vs.th1;
        pret2 = vs.th2;
        delete f_point;
    }
    return path;
}

void RecalculatePath::addState(Point *point, double th1, double th2, double error){
    //add the point as an adjacent point
    //TODO: shuld not be same error for fromPoint and toVisit?
    VehicleState *vs = new VehicleState(pos->x, pos->y, theta1, theta2);
    fromPoints->insert({*point, *vs});
    VehicleState_error *vs_to = new VehicleState_error(new VehicleState(point->x, point->y, th1, th2), error, front_ec->getCopy(), back_ec->getCopy());
    double total_error = totError + abs(error);
    vs_to->totError = total_error;
    toVisit->push(vs_to);

    VehicleState *in_r_vs = new VehicleState(point->x, point->y, th1, th2);
    VehicleState *rounded_vs = rounding(in_r_vs, modPoint, modTheta);
    errorList->insert({*rounded_vs, total_error});
    delete in_r_vs;

}
