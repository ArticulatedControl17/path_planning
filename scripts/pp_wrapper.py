# Python wrapper for pathplanner
from ctypes import *
import numpy as np

lib = cdll.LoadLibrary('./libpp.so')

lib.PP_new.argtypes = [POINTER(POINTER(c_int))]
lib.PP_setMap.argtypes = [c_int, POINTER(POINTER(c_int))]

lib.PP_checkIfInTrack.argtypes = [c_int, POINTER(c_double)]
lib.PP_checkIfInTrack.restype = c_bool

lib.PP_setOptimalPath.argtypes = [c_int, POINTER(POINTER(c_double)), c_int]
lib.PP_getPath.argtypes =[c_int, POINTER(c_double), POINTER(c_double), POINTER(c_double), c_double, c_double, c_double]


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
        vs_array = np.array([vs.x, vs.y, vs.theta1, vs.theta2]).astype(c_double).ctypes.data_as(POINTER(c_double))
        ep_array = np.array([end_point[0], end_point[1]]).astype(c_double).ctypes.data_as(POINTER(c_double))
        sep_array = np.array([snd_end_point[0], snd_end_point[1]]).astype(c_double).ctypes.data_as(POINTER(c_double))

        result = lib.PP_getPath(self.obj, vs_array, ep_array, sep_array, max_exec_time, point_mod, theta_mod)
        path = []
        for i, vs in enumerate(result):
            path.append(VehicleState(vs[i][0], vs[i][1], vs[i][2], vs[i][3]))

        return path

