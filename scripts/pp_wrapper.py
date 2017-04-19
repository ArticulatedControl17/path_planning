# Python wrapper for pathplanner
from ctypes import *
import numpy as np
from vehicleState import *

lib = cdll.LoadLibrary('./libpp.so')

lib.PP_new.argtypes = [POINTER(POINTER(c_int))]
lib.PP_setMap.argtypes = [c_int, POINTER(POINTER(c_int))]

lib.PP_checkIfInTrack.argtypes = [c_int, POINTER(c_double)]
lib.PP_checkIfInTrack.restype = c_bool

lib.PP_setOptimalPath.argtypes = [c_int, POINTER(POINTER(c_double)), c_int]
lib.PP_getPath.argtypes =[c_int, POINTER(c_double), c_double, c_double, c_double]
lib.PP_getPath.restype = POINTER(POINTER(c_double))


class PathPlannerCPP(object):

    def __init__(self, matrix):
        matrix_ = np.array(matrix).astype(c_int).ctypes.data_as(POINTER(POINTER(c_int)))
        self.obj = lib.PP_new(matrix_)

    def setMap(self, matrix):
        matrix_ = np.array(matrix).astype(c_int).ctypes.data_as(POINTER(POINTER(c_int)))
        lib.PP_setMap(self.obj, matrix_)

    def checkIfInTrack(self, vs):
        vs_array = [vs.x, vs.y, vs.theta1, vs.theta2]
        vs_array_ = np.array(vs_array).astype(c_double).ctypes.data_as(POINTER(c_double))
        return lib.PP_checkIfInTrack(self.obj, vs_array_)

    def setOptimalPath(self, path):
        path_array = []
        for point in path:
            path_array.append([point[0], point[1]])
        path_array_ = np.array(path_array).astype(c_double).ctypes.data_as(POINTER(POINTER(c_double)))
        lib.PP_setOptimalPath(self.obj, path_array_, len(path))

    def getPath(self, vs, end_point, snd_end_point, max_exec_time, point_mod, theta_mod):
        vs_array = [vs.x, vs.y, vs.theta1, vs.theta2]
        ep_array = [end_point[0], end_point[1]]
        sep_array = [snd_end_point[0], snd_end_point[1]]
        data_array = np.array(vs_array + ep_array + sep_array).astype(c_double).ctypes.data_as(POINTER(c_double))
        
        # REMOVE ------------------------------------------------------------ ->
        """
        print "== vehicle state:", str(vs.x) + ",", str(vs.y) + ",", str(vs.theta1) + ",", vs.theta2 
        print "== end point:", end_point
        print "== second end point:", snd_end_point
        evs_array = np.array([vs.x + 1, vs.y + 1, vs.theta1 + 1, vs.theta2 + 1]).astype(c_double).ctypes.data_as(POINTER(c_double))
        print "== vehicle state2:", str(evs_array[0]) + ",", str(evs_array[1]) + ",", str(evs_array[2]) + ",", evs_array[3] 
        print "== ", str(ep_array[0]) + ", ", ep_array[1]
        print "== ", str(sep_array[0]) + ", ", sep_array[1]
        print "== end point:", end_point, "==", end_point[0], end_point[1]
        print "== second end point:", snd_end_point, "==", snd_end_point[0], snd_end_point[1]
        """
        # <- ------------------------------------------------------------ REMOVE

        res = lib.PP_getPath(self.obj, data_array, max_exec_time, point_mod, theta_mod)
        res_size = lib.PP_getPathSize(self.obj)
        path = []

        for i in range (res_size):
            path.append(VehicleState(res[i][0], res[i][1], res[i][2], res[i][3]))

        return path
