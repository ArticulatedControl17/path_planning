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
    bool isLeft = ((pp2->x - pp1->x) * (y - pp1->y) - (pp2->y - pp1->y) * (x - pp1->x)) >0;
    double value = abs((pp2->x - pp1->x)*(pp1->y - y ) - (pp1->x-x) * (pp2->y - pp1->y))
        / (sqrt((pp2->x - pp1->x)*(pp2->x - pp1->x) + (pp2->y - pp1->y) * (pp2->y - pp1->y)));

    if(isLeft){
        return -value;
    }else{
        return value;
    }
}

bool ErrorCalc::isAboveEnd(Point *pbegin, Point *pend, double x, double y){
    //checks if a point is passed the end point of a line.
    if (pbegin->x - pend->x !=0 && pbegin->y - pend->y !=0){
        double slope = pbegin->y - pend->y / pbegin->x - pend->x;
        double prependularSlope = (-1)/slope;
        double prependularM = pend->y - pend->y * prependularSlope;
        if(pbegin->y < pend->y){
            //going up
            return (x*prependularSlope + prependularM - y) < 0;
        }
        else{
            //going down
            return (x*prependularSlope + prependularM - y) > 0;
        }
    } else if( pbegin->x - pend->x){
        //going straight in x direction
        if(pbegin->x < pend->x){
            //going right
            return x > pend->x;
        } else{
            //going left
            return x < pend->x;
        }
    }
    else{
        //going straight in y direction
        if( pbegin->y < pend->y){
            //going up
            return y > pend->y;
        }
        else{
            //going down
            return y < pend->y;
        }
    }
}

double ErrorCalc::getDirection(void){
    double dy = pp2->y - pp1->y;
    double dx = pp2->x - pp1->x;
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
    double d1 = sqrt( (point->x - pp2->x) * (point->x - pp2->x)
                + (point->y - pp2->y) * (point->y - pp2->y) );
    double d0 = sqrt( (point->x - pp1->x) * (point->x - pp1->x)
                + (point->y - pp1->y) * (point->y - pp1->y) );
    return std::max(d1,d0);

}

bool ErrorCalc::is_next_left(void){
    bool isLeft;
    if(queue.size()>0){
        isLeft = ((pp2->x - pp1->x) * (queue.front()->y - pp1->y)
            - (pp2->y - pp1->y) * (queue.front()->x - pp1->x)) >0;
    } else{
        isLeft=true;
    }
    return isLeft;
}

ErrorCalc *ErrorCalc::getCopy(){
    std::list<Point*> copy(queue);
    copy.push_front(pp2);
    copy.push_front(pp1);
    return new ErrorCalc(copy);
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
