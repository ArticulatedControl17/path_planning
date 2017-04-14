#!/usr/bin/env python
from barWrapper import *

if __name__ == '__main__':

    b = Bar()
    #b.setVar(5.0)
    md = MyDouble(10.0)

    pyobj = ctypes.py_object(md)
    b.setVar2(ctypes.byref(pyobj))

    print b.getFirst()

    #print b.getFirst2()

    #md2 = MyDouble(10.0)
    #print md2.getVal()
