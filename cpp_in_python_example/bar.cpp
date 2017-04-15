#include "bar_3.hpp"
#include <iostream>
#include <Python.h>

class Bar {
    public:
        MyDoubleList *dl;

        Bar() {
            dl = new MyDoubleList();
        }

        void setVar(double val) {
            //std::cout << "Adding: " << val << std::endl;
            MyDouble *d = new MyDouble(val);
            //std::cout << "MyDouble val: " << d->val << std::endl;
            dl->addVal(d);
            //std::cout << "First item: " << dl->double_list.front()->val << std::endl;
        }

        void setVar2(PyObject *val_) {
            std::cout << val_ << std::endl;
            PyObject *p = PyObject_GetAttrString(val_, "getVal");
            //std::cout << PyString_AsString( PyObject_Str(p) ) << std::endl;
            //MyDouble *d = new MyDouble(*pval);

            //dl->addVal(d);
        }

        double getFirst() {
            //std::cout << "Returning: " << dl->double_list.front()->val << std::endl;
            return dl->double_list.front()->val;
        }

        MyDoubleList * getList() {
            return dl;
        }

        MyDouble * getFirst2() {
            //std::cout << dl->double_list.front() << std::endl;
            return dl->double_list.front();
        }

        void arrToList(MyDouble **arr, int length) {
            for (int i = 0; i < length; i++) {
                dl->addVal(arr[i]);
            }
        }
};

void test() {
    std::cout << "Test" << std::endl;
}

double powwer(double x) {
    return x*x;
}



int main() {}


// Since ctypes can only talk to C functions, you need to provide those declaring them as extern "C":

extern "C" {
    Bar * Bar_new() { return new Bar(); }

    void Bar_setVar(Bar *bar, double val) { bar->setVar(val); }
    void Bar_setVar2(Bar *bar, PyObject *val) { bar->setVar2(val); }

    double Bar_getFirst(Bar *bar) { return bar->getFirst(); }
    MyDouble * Bar_getFirst2(Bar *bar) { return bar->getFirst2(); }

    MyDoubleList * Bar_getList(Bar *bar) {return bar->getList(); }
    void Bar_arrToList(Bar *bar, MyDouble **arr, int length) { bar->arrToList(arr, length); }


    MyDouble * MD_new(double val) { return new MyDouble(val); }
    double MD_getVal(MyDouble *md) { return md->val; }


    void C_test() { test(); }
    double C_powwer(double x) { return powwer(x); }
}


// Next you have to compile this to a shared library:

/*
g++ -c -std=c++11 -fPIC bar_3.cpp

g++ -c -std=c++11 -fPIC bar.cpp -I/usr/include/python2.7 -lpython2.7

g++ -std=c++11 -fPIC bar.cpp bar_2.cpp bar_3.cpp -I/usr/include/python2.7 -lpython2.7
g++ -shared -Wl,-soname,libbar.so -o libbar.so  bar.o bar_2.o bar_3.o

*/
