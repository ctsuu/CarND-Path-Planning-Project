
#include "utils.h"
#include <vector>
#include "spline.h"

double Utils::deg2rad(double x){
  return x * M_PI / 180;
}


double Utils::rad2deg(double x){
  return x * 180 / M_PI;
}


double Utils::getDistance(double x1, double y1, double x2, double y2){
   return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


int Utils::getLaneNumberForD(double d){
  if (d < 0.0 || d > 12.0){
    return -1;
  }

  return d > 8.0 ? 2 : d > 4.0 ? 1 : 0;
}


double Utils::getDForLaneNumber(int Lane){

  std::vector<double> sDVector = {2.0, 6.0, 9.75};
  assert(Lane >= 0 && Lane <= 2);
  return sDVector[Lane];
}



