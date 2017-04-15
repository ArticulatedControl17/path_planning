#include "bar_2.hpp"
#include <list>

#ifndef Bar3_H
#define Bar3_H


class MyDoubleList {
    public:
        MyDoubleList();
        void addVal(MyDouble *val);
        std::list<MyDouble *> getList();
        
        std::list<MyDouble *> double_list;
};


#endif