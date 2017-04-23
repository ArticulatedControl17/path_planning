#include "recalculatePath.hpp"

//#include "vehicleState_error.hpp"
#include "Point.hpp"
#include "track_checker.hpp"
#include "vehicleState.hpp"
#include "helper_functions.hpp"
#include <unordered_map>
#include <stack>
#include <unordered_set>
#include <list>
#include <iostream>
#include <math.h>


class PathPlanner {
    //private:
        //ros::NodeHandle n;
        //custom_msgs::Poisition msg;
        //ros::Publisher visited_pub;
        //ros::Publisher to_visit_pub;

    public:
//    private:
        TrackChecker * track_checker;
        double theta1;
        double theta2;

        ErrorCalc *front_ec;
        ErrorCalc *back_ec;
        Point *pos;
        int path_size;

        std::list<Point*> optimalPath;
        std::stack<VehicleState_error*>* toVisit;//stack of toVisit points
        std::unordered_set<VehicleState>* visited;
        std::unordered_map<Point , VehicleState_error>* fromPoints; //Map of fromPoints

        void addPossiblePathes(bool leftFirst);
        std::list<VehicleState*> gatherPath(Point *startPoint, Point *endPoint, double end_theta1, double end_theta2);
        double gatherError(Point *startPoint, Point *endPoint);
        void addState(Point *point, double th1, double th2, double error);

    public:
        PathPlanner(int *mat);
        std::list<VehicleState*> getPath(VehicleState *startVs, Point *endPoint, Point *secondEndPoint, 
                double MAX_EXECUTION_TIME, double modPoint, double modTheta, bool returnsIfFeasible = false);
        void setOptimalpath( std::list<Point*> path);
        void setMap(int *mat);
        bool checkIfInTrack(VehicleState *vs);
};
