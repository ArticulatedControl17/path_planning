from ctypes import *
import numpy as np
from numpy.ctypeslib import ndpointer

lib = cdll.LoadLibrary('./libfb.so')

lib.FB_new.argtypes = [(c_int * 3)]
lib.FB_getArr.restype = POINTER(c_int)

lib.FB_new2.argtypes = [POINTER(POINTER(c_int))]
lib.FB_getMatrix.restype = POINTER(POINTER(c_int))


class FooBar(object):

    def __init__(self, array):
        self.obj = lib.FB_new(array)

    def getArr(self):
        return lib.FB_getArr(self.obj)


class FooBar2(object):

    def __init__(self, matrix):
        self.obj = lib.FB_new2(matrix)

    def getMatrix(self):
        return lib.FB_getMatrix(self.obj)








"""
int_arr = (c_int * 3)
int_matr = (int_arr * 3)
LP_c_int = POINTER(c_int)
LP_LP_c_int = POINTER(LP_c_int)

#lib.FB_new.argtypes = [ndpointer(c_int, flags="C_CONTIGUOUS")]
#lib.FB_new.argtypes = [POINTER(c_int)]
lib.FB_new.argtypes = [int_arr]
lib.FB_getArr.restype = POINTER(c_int)

#lib.FB_new2.argtypes = [int_matr]
lib.FB_new2.argtypes = [POINTER(POINTER(c_int))]
#lib.FB_getMatrix.restype = int_matr
lib.FB_getMatrix.restype = POINTER(POINTER(c_int))
#lib.FB_getMatrix.restype = int_matr
"""