#!/usr/bin/env python
from fooWrapper import *


if __name__ == '__main__':

    f = Foo()
    f.bar()  # prints "Hello"
    test()  # prints "Test"

    print f.multByFive(5)
    print f.multByFive(50)

    print "f.var:", f.getVar()
    f.setVar(100)
    print "f.var:", f.getVar()

    f2 = Foo(10)
    print "f2.var:", f2.getVar()

    f2.setVar(15)
    print "f2.var:", f2.getVar()
