#include <iostream>
#include <stdlib.h>
#include <math.h>
#include "error_calc.hpp"

ErrorCalc::ErrorCalc(std::list<Point*> startList){
    queue = startList;
    pp1 = queue.front();
    queue.pop_front();
    pp2 = queue.front();
    queue.pop_front();
}

double ErrorCalc::calculateError(double x, double y){
    while(ErrorCalc::isAboveEnd(pp1, pp2, x, y) && queue.size()>0){
        pp1 = pp2;
        pp2 = queue.front();
        queue.pop_front();
  }
    //decides if the error is to the left of centerline or not
    bool isLeft = (((*pp2).getX() - (*pp1).getX()) * (y - (*pp1).getY()) - ((*pp2).getY() - (*pp1).getY()) * (x - (*pp1).getX())) >0;
    double value = abs(((*pp2).getX() - (*pp1).getX())*((*pp1).getY()-y ) - ((*pp1).getX()-x)*((*pp2).getY()-(*pp1).getY()))
        / (sqrt(((*pp2).getX()-(*pp1).getX())*((*pp2).getX()-(*pp1).getX()) + ((*pp2).getY()-(*pp1).getY())*((*pp2).getY()-(*pp1).getY())));

    if(isLeft){
        return -value;
    }else{
        return value;
    }
}

bool ErrorCalc::isAboveEnd(Point *pbegin, Point *pend, double x, double y){
    //checks if a point is passed the end point of a line.
    if ((*pbegin).getX() - (*pend).getX() !=0 && (*pbegin).getY() - (*pend).getY() !=0){
        double slope = (*pbegin).getY() - (*pend).getY() / (*pbegin).getX() - (*pend).getX();
        double prependularSlope = (-1)/slope;
        double prependularM = (*pend).getY() - (*pend).getY()*prependularSlope;
        if((*pbegin).getY() < (*pend).getY()){
            //going up
            return (x*prependularSlope + prependularM - y) < 0;
        }
        else{
            //going down
            return (x*prependularSlope + prependularM - y) > 0;
        }
    } else if( (*pbegin).getX() - (*pend).getX()){
        //going straight in x direction
        if((*pbegin).getX() < (*pend).getX()){
            //going right
            return x > (*pend).getX();
        } else{
            //going left
            return x < (*pend).getX();
        }
    }
    else{
        //going straight in y direction
        if( (*pbegin).getY() < (*pend).getY()){
            //going up
            return y > (*pend).getY();
        }
        else{
            //going down
            return y < (*pend).getY();
        }
    }
}

double ErrorCalc::getDirection(void){
    double dy = (*pp2).getY() - (*pp1).getY();
    double dx = (*pp2).getX() - (*pp1).getX();
    double theta = atan2(dy, dx);
    return theta;
}

double ErrorCalc::isAtEnd(){
    if(queue.size()==0){
        return true;
    } else{
        return false;
    }
}

double ErrorCalc::getMaxDistPoint(Point *point){
    //TODO: Should this be max or min?
    double d1 = sqrt( ((*point).getX() - (*pp2).getX()) * ((*point).getX() - (*pp2).getX())
                + ((*point).getY() - (*pp2).getY()) * ((*point).getY() - (*pp2).getY()) );
    double d0 = sqrt( ((*point).getX() - (*pp1).getX()) * ((*point).getX() - (*pp1).getX())
                + ((*point).getY() - (*pp1).getY()) * ((*point).getY() - (*pp1).getY()) );
    return std::max(d1,d0);

}

bool ErrorCalc::is_next_left(void){
    bool isLeft;
    if(queue.size()>0){
        isLeft = (((*pp2).getX() - (*pp1).getX()) * ((*queue.front()).getY() - (*pp1).getY())
            - ((*pp2).getY() - (*pp1).getY()) * ((*queue.front()).getX() - (*pp1).getX())) >0;
    } else{
        isLeft=true;
    }
    return isLeft;
}

ErrorCalc ErrorCalc::getCopy(){
    std::list<Point*> copy(queue);
    copy.push_front(pp2);
    copy.push_front(pp1);
    return ErrorCalc(copy);
}

/*
int main(){
  std::cout << "IN MAIN ";
  std::list<Point*> startList;

  Point *point1 = new Point(0.0, 0.0);

  std::cout << "p1 : " << (*point1).getX() << " " << (*point1).getY() <<std::endl;
  startList.push_front(point1);
  std::cout << "ASD" <<std::endl;
  Point *point2 = new Point(5.0, 0.0);
  std::cout << "p2 : " << (*point2).getX() << " " << (*point2).getY() <<std::endl;
  startList.push_back(point2);
  Point *point3 = new Point(5.0, 10.0);
  startList.push_back(point3);
  std::cout << "p3 : " << (*point3).getX() << " " << (*point3).getY() <<std::endl;
  ErrorCalc ec(startList);
  Point *point0 = new Point(10.0, 2.0);
  ErrorCalc ec_c = ec.getCopy();
  std::cout << "isLeft : " << ec.is_next_left() <<std::endl;
  std::cout << "maxDist : " << ec.getMaxDistPoint(point0) <<std::endl;
  std::cout << "error : " << ec.calculateError(point0->getX(), point0->getY()) <<std::endl;
  std::cout << "direction : " << ec.getDirection() <<std::endl;
  std::cout << "atEnd? : " << ec.isAtEnd() <<std::endl;
  std::cout << "atEnd copy? : " << ec_c.isAtEnd() <<std::endl;
  return 0;
}
*/
