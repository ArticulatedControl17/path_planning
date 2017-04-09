
class Point {
  public:
    Point(double x, double y);
    double getX();
    double getY();
    void setValues(double x, double y);

  private:
    double x;
    double y;
};

Point::Point(double nx, double ny){
  x = nx;
  y = ny;
}

double Point::getX(void){
  return x;
}

double Point::getY(void){
  return y;
}

void Point::setValues(double nx, double ny){
  x = nx;
  y = ny;

}
