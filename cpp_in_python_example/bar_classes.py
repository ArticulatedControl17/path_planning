import ctypes
from ctypes import cdll
import numpy as np


class MyDouble:

    def __init__(self, val_):
        self.val = val_

    @classmethod
    def getVal(cls, obj):
        return str(obj.val)

    def setVal(self, val_):
        self.val = val_




"""
lib = cdll.LoadLibrary('./libbar.so')

lib.MD_new.argtypes = [ctypes.c_double]
lib.MD_getVal.restype = ctypes.c_double

class MyDouble(object):

    def __init__(self, val):
        self.obj = lib.MD_new(np.float64(val))

    def getVal(self):
        return lib.MD_getVal(self.obj)
"""