import ctypes
from ctypes import *
import numpy as np
from bar_classes import *

ptr = POINTER(ctypes.py_object)

lib = cdll.LoadLibrary('./libbar.so')
lib.C_powwer.restype = ctypes.c_double
lib.C_powwer.argtypes = [ctypes.c_double]

lib.Bar_setVar.argtypes = [ctypes.c_int, ctypes.c_double]

lib.Bar_getFirst.restype = ctypes.c_double
lib.Bar_getFirst2.restype = ctypes.c_double


lib.Bar_setVar2.argtypes = [ctypes.c_int, ptr]


class Bar(object):

    def __init__(self):
        self.obj = lib.Bar_new()

    # Takes double
    def setVar(self, v):
        lib.Bar_setVar(self.obj, np.float64(v))

    # Takes MyDouble object
    def setVar2(self, v):
        lib.Bar_setVar2(self.obj, v)

    def getFirst(self):
        return lib.Bar_getFirst(self.obj)

    def getFirst2(self):
        return lib.Bar_getFirst2(self.obj)

    def getList(self):
        return lib.Bar_getList(self.obj)

    def arrToList(self, arr, length):
        lib.Bar_arrToList(self.obj, arr, length)


def test():
    lib.C_test()  

def powwer(x):
    return lib.C_powwer(np.float64(x))  


# Once you have that you can call it like

"""
b = Bar()
"""


"""
extern "C" {
    Bar * Bar_new() { return new Bar(); }

    void Bar_setVar(Bar *bar, double val) { bar->setVar(val); }
    void Bar_setVar2(Bar *bar, MyDouble *val) { bar->setVar2(val); }

    MyDouble *Bar_getFirst(Bar *bar) { return bar->getFirst(); }
    void Bar_arrToList(Bar *bar, MyDouble **arr, int length) { bar->arrToList(arr, length); }
}
"""