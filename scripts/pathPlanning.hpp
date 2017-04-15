#include "Point.hpp"
#include "track_checker.hpp"
#include "vehicleState.hpp"
#include "vehicleState_error.hpp"
#include "helper_functions.hpp"
#include <unordered_map>
#include <stack>
#include <unordered_set>
#include <list>
#include <iostream>
#include <math.h>
//#include "ros/ros.h"
//#include "custom_msgs/Position.h"

namespace std {

    template <>
    struct hash<Point>
    {
      std::size_t operator()(const Point& k) const
      {
        using std::size_t;
        using std::hash;

        // Compute individual hash values for first,
        // second and third and combine them using XOR
        // and bit shifting:

        return (hash<double>()(k.x)
                 ^ (hash<double>()(k.y) << 1));
      }
    };
}
namespace std {
    template <>
    struct hash<VehicleState>
    {
      std::size_t operator()(const VehicleState& k) const
      {
        using std::size_t;
        using std::hash;

        // Compute individual hash values for first,
        // second and third and combine them using XOR
        // and bit shifting:

        return (((hash<double>()(k.x)
                 ^ (hash<double>()(k.y) << 1)
                 ^ hash<double>()(k.th1) << 1)
                 ^ hash<double>()(k.th2) << 1));
      }
    };
}


class PathPlanner {
    private:
        //ros::NodeHandle n;
        //custom_msgs::Poisition msg;
        //ros::Publisher visited_pub;
        //ros::Publisher to_visit_pub;

        TrackChecker * track_checker;
        double theta1;
        double theta2;
        ErrorCalc *front_ec;
        ErrorCalc *back_ec;
        Point *pos;
        std::list<Point*> optimalPath;
        std::stack<VehicleState_error*> toVisit;//stack of toVisit points
        std::unordered_set<VehicleState> visited;
        std::unordered_map<Point , VehicleState_error> fromPoints; //Map of fromPoints
        void addPossiblePathes(bool leftFirst);
        std::list<VehicleState*> gatherPath(Point *startPoint, Point *endPoint, double end_theta1, double end_theta2);
        double gatherError(Point *startPoint, Point *endPoint);
        void addState(Point *point, double th1, double th2, double error);
    public:
        PathPlanner(int** mat);
        std::list<VehicleState*> getPath(VehicleState *startVs, Point *endPoint, Point *secondEndPoint
            , double MAX_EXECUTION_TIME, double modPoint, double modTheta);
        void setOptimalpath( std::list<Point*> path);
        void setMap(int** mat);
        bool checkIfInTrack(VehicleState *vs);
};
