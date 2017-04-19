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


class RecalculatePath {
    //private:
        //ros::NodeHandle n;
        //custom_msgs::Poisition msg;
        //ros::Publisher visited_pub;
        //ros::Publisher to_visit_pub;

    public:
//    private:

        std::unordered_map<VehicleState , double>* errorList; //Map of fromPoints
        std::list<VehicleState*>* path;
        double lowest_error;
        double totError;

        TrackChecker * track_checker;
        double theta1;
        double theta2;
        double modPoint;
        double modTheta;

        ErrorCalc *front_ec;
        ErrorCalc *back_ec;
        Point *pos;
        int path_size;

        std::list<Point*> optimalPath;
        std::stack<VehicleState_error*>* toVisit;//stack of toVisit points
        std::unordered_set<VehicleState>* visited;
        std::unordered_map<Point , VehicleState>* fromPoints; //Map of fromPoints

        void addPossiblePathes();
        std::list<VehicleState*>* gatherPath(Point *startPoint, Point *endPoint);
        void addState(Point *point, double th1, double th2, double error);

    public:
        RecalculatePath(TrackChecker *track_checker_);
        std::list<VehicleState*> calculate_path(VehicleState *startVs, Point *endPoint, Point *secondEndPoint
            , double totError, ErrorCalc *front_ec_, double modPoint, double modTheta);
};
