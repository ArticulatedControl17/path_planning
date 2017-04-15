#include "bar_3.hpp"

// g++ -c bar_3.cpp -std=c++11


MyDoubleList::MyDoubleList() {
    double_list = {};
}


void MyDoubleList::addVal(MyDouble *val) {
    double_list.push_back(val);
}

std::list<MyDouble *> MyDoubleList::getList() {
    return double_list;
}
