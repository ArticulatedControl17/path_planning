
// Suppose you have a simple C++ example class you want to talk to in a file called foo.cpp:

#include <iostream>

class Foo {
    public:
        int var;

        Foo() {
            var = 0;
        }

        Foo(int v) {
            var = v;
        }

        int getVar() {
            return var;
        }

        void setVar(int v) {
            var = v;
        }

        void bar() {
            std::cout << "Hello" << std::endl;
        }

        int multByFive(int x) {
            return x * 5;
        }
};


void test() {
    std::cout << "Test" << std::endl;
}


// Since ctypes can only talk to C functions, you need to provide those declaring them as extern "C":

extern "C" {
    Foo* Foo_new() { return new Foo(); }
    Foo* Foo_new_2(int v) { return new Foo(v); }

    int Foo_getVar(Foo* foo) { return foo->var; }
    void Foo_setVar(Foo* foo, int v) { foo->setVar(v); }

    void Foo_bar(Foo* foo) { foo->bar(); }
    int Foo_multByFive(Foo* foo, int x) { foo->multByFive(x); }

    void C_test() { test(); }
}


// Next you have to compile this to a shared library:

/*

g++ -c -fPIC foo.cpp -o foo.o
g++ -shared -Wl,-soname,libfoo.so -o libfoo.so  foo.o

*/
