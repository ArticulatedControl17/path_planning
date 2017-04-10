
# And finally you have to write your python wrapper (e.g. in fooWrapper.py):

from ctypes import cdll
lib = cdll.LoadLibrary('./libfoo.so')

class Foo(object):

    def __init__(self, v=None):
        self.obj = lib.Foo_new() if v == None else lib.Foo_new_2(v)

    def setVar(self, v):
        lib.Foo_setVar(self.obj, v)

    def getVar(self):
        return lib.Foo_getVar(self.obj)

    def bar(self):
        lib.Foo_bar(self.obj)

    def multByFive(self, x):
        return lib.Foo_multByFive(self.obj, x)

def test():
  lib.C_test()  


# Once you have that you can call it like

"""
f = Foo()
f.bar() # and you will see "Hello" on the screen
"""
