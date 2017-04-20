#include "pathPlanning.hpp"
#include <iostream>


extern "C" {

    PathPlanner * PP_new(int *matrix) {
        std::cout << "CONSTRUCTOR" << std::endl;
        return new PathPlanner(matrix);
    }
    

    void PP_setMap(PathPlanner *pp, int *matrix) {
        std::cout << "SET MAP" << std::endl;
        pp->setMap(matrix);
    }


    bool PP_checkIfInTrack(PathPlanner *pp, double *vs_arr) {
        std::cout << "CHECK IF IN TRACK" << std::endl;
        VehicleState *vs = new VehicleState(vs_arr[0], vs_arr[1], vs_arr[2], vs_arr[3]);
        return pp->checkIfInTrack(vs);
    }


    void PP_setOptimalPath(PathPlanner *pp, double *path_arr, int n) {
        std::cout << "SET OPT PP" << std::endl;
        std::list<Point *> path = {};

        for (int i = 0; i < n; i++) {
            Point *p = new Point(path_arr[i*2], path_arr[i*2+1]);
            path.push_back(p);
        }

        pp->setOptimalpath(path);
    }


    double ** PP_getPath(PathPlanner *pp, double *data_array, double max_time, double pm, double tm) {
        std::cout << "GET PATH PP" << std::endl;
        VehicleState *vs = new VehicleState(data_array[0], data_array[1], data_array[2], data_array[3]);
        Point *ep = new Point(data_array[4], data_array[5]);
        Point *sep = new Point(data_array[6], data_array[7]);

        std::list<VehicleState *> result = pp->getPath(vs, ep, sep, max_time, pm, tm);

        int size = result.size();
        double **path = new double*[size];

        for (int i = 0; i < size; i++) {
            path[i] = new double[4];
            vs = result.front();
            result.pop_front();

            path[i][0] = vs->x;
            path[i][1] = vs->y;
            path[i][2] = vs->th1;
            path[i][3] = vs->th2;
        }

        delete vs;
        pp->path_size = size;
        return path;
    }


    int PP_getPathSize(PathPlanner *pp) {
        return pp->path_size;
    }
}


int main() {}


/* --- Compiling into shared Library ---------------------------------------------------------------------------------------------------------------------------------------------

1. Recompile any changed C++-files:

g++ -c -std=gnu++11 -fPIC NAME_OF_UPDATED_FILE.cpp

2. Build the library:

g++ -std=gnu++11 -fPIC pp_wrapper.cpp pathPlanning.cpp Point.cpp track_checker.cpp vehicleState.cpp vehicleState_error.cpp helper_functions.cpp error_calc.cpp model.cpp recalculatePath.cpp
g++ -shared -Wl,-soname,libpp.so -o libpp.so  pp_wrapper.o pathPlanning.o Point.o track_checker.o vehicleState.o vehicleState_error.o helper_functions.o error_calc.o model.o recalculatePath.o

*/
