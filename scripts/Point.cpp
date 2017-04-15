#include "Point.hpp"
#include <iostream>
#include <functional>
#include <unordered_map>

Point::Point(double nx, double ny){
    x = nx;
    y = ny;
}


void Point::setValues(double nx, double ny){
    x = nx;
    y = ny;

}


bool Point::operator==(const Point &other) const{
    return (x == other.x
     && y == other.y);
}



/*
int main()
{
  std::unordered_map<Point,std::string> m6 = {
    { {Point(3,2)}, "example"},
    { {Point(5,2)}, "another"}
  };
  auto iter = m6.find(Point(4,2));
  std::cout << iter->first;
}
*/
