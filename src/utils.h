#ifndef UTILS_H
#define UTILS_H

#include <assert.h>
#include "Eigen-3.3/Eigen/Core"
#include <cmath>
#include <vector>
#include "spline.h"

namespace Utils
{
  double deg2rad(double x);
  double rad2deg(double x);
  double getDistance(double x1, double y1, double x2, double y2);
  int getLaneNumberForD(double d);
  double getDForLaneNumber(int Lane);

}


#endif // UTILS_H

