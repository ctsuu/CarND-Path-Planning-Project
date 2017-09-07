#ifndef ROAD
#define ROAD

#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include "spline.h"
#include "constants.h"
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"


tk::spline s_x, s_y, s_dx, s_dy;
using namespace std;

using Eigen::Vector2d;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }



double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];

		//cout << "map x: "<< map_x <<";"<<"Map_y : "<< map_y << endl;
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}


int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	// add new line
	// double theta_pos = fmod(theta + (2*pi()),2*pi());     
	// double heading_pos = fmod(heading + (2*pi()),2*pi());

	double angle = abs(theta-heading);
	// cout << "map x: "<< map_x <<";\t"<<"Map_y : "<< map_y << endl;
	if(angle > M_PI/4)
	//if(angle > pi()/2)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}
	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];
	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;
	double frenet_d = distance(x_x,x_y,proj_x,proj_y);
	//see if d value is positive or negative by comparing it to a center point
	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);
	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}
	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}
	frenet_s += distance(0,0,proj_x,proj_y);
	return {frenet_s,frenet_d};
}



// Transform from Frenet s,d coordinates to global Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;
	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}
	int wp2 = (prev_wp+1)%maps_x.size();
	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);
	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
	double perp_heading = heading-pi()/2;
	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);
	return {x,y};
}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, tk::spline s_x, tk::spline s_y, tk::spline s_dx, tk::spline s_dy) {
    double path_x = s_x(s);
    double path_y = s_y(s);
    double dx = s_dx(s);
    double dy = s_dy(s);
    double x = path_x + d * dx;
    double y = path_y + d * dy;
    return {x,y};
}

// get first derivative for a spline curve at point x
double deriv1(tk::spline func, double x){
    double dx=1e-8*(1.0+fabs(x));
    double x0=x-dx;
    double x2=x+dx;
    return (func(x2)-func(x0)) / (2.0*dx);
}

// get second derivative for a spline curve at point x
double deriv2(tk::spline func, double x){
    double dx=1e-6*(1.0+fabs(x));
    double x0=x-dx;
    double x2=x+dx;
    return (func(x0)-2.0*func(x)+func(x2)) / (dx*dx);
}


double Error(const Eigen::Vector2d& p, double s, tk::spline s_x, tk::spline s_y ){
    return pow(p(0) - s_x(s), 2) + pow(p(1) - s_y(s), 2);
}


double ErrorDeriv(const Eigen::Vector2d& p, double s, tk::spline s_x, tk::spline s_y){
  return -2.0 * (p(0) - s_x(s)) * deriv1(s_x, s)
         - 2.0 * (p(1) - s_y(s)) * deriv1(s_y,s);
}

Vector2d GetNormalAt(double s, tk::spline s_x, tk::spline s_y )
{
  return Vector2d(deriv1(s_y, s), -deriv1(s_x, s));
  //return Vector2d(-deriv1(s_y, s), deriv1(s_x, s));
}



Vector2d getFrenet(const Eigen::Vector2d& p, double aStartS, tk::spline s_x, tk::spline s_y, tk::spline s_dx, tk::spline s_dy)
{
  // Perform gradient descent in order to find the point on the spline that is closest to p:
  const double eps = 1.0e-6;
  double s = aStartS;
  const double kGamma = 0.001;
  const double kPrecision = 0.001;


  double PreviousStepSize = s;

  while (PreviousStepSize > kPrecision)
  {
    const double next_s = s - kGamma * ErrorDeriv(p, s, s_x, s_y);
    PreviousStepSize = std::abs(next_s - s);
    s = next_s;
    while (s > TRACK_LENGTH)
    {
      s -= TRACK_LENGTH;
    }

    while (s < 0)
    {
      s += TRACK_LENGTH;
    }
    // cout <<s<<", ";
  }

  double x_est = s_x(s);
  double y_est = s_y(s);
  double x_diff = p[0] - x_est;
  double y_diff = p[1] - y_est;
  double dx = s_dx(s);
  double dy = s_dy(s);
  double d_mag = sqrt(pow(dx,2)+pow(dy,2));
  dx = dx / d_mag;
  dy = dy / d_mag;
  double d = dx * x_diff + dy * y_diff;


  // From the established point on the spline find the offset vector to the target
  // point and do a component-wise divide by the normal vector:
  // const Vector2d p_spline(s_x(s), s_y(s));
  // const Vector2d p_delta = (p - p_spline).array() / GetNormalAt(s, s_x, s_y).array();

  // Use the mean of the two resulting components as the d-coordinate:
  // const double d = 0.5 * (p_delta(0) + p_delta(1));
  return Vector2d(s, d);
}




Vector2d getXY(double s, double d, tk::spline s_x, tk::spline s_y){
  while (s > TRACK_LENGTH)
  {
    s -= TRACK_LENGTH;
  }

  while (s < 0)
  {
    s += TRACK_LENGTH;
  }

  return Vector2d(s_x(s), s_y(s)) + GetNormalAt(s, s_x, s_y) * d;
}








// Transform from global Cartesian x,y to local car coordinates x,y
// where x is pointing to the positive x axis and y is deviation from the car's path
vector<double> getLocalXY(double car_x, double car_y, double theta, double wx, double wy) {
  vector<double> results;

  // convert to local coordinates
  float deltax = (wx - car_x);
  float deltay = (wy - car_y);
  results.push_back(deltax*cos(theta) + deltay*sin(theta));
  results.push_back(-deltax*sin(theta) + deltay*cos(theta));
  return results;
}

// Transform from local Cartesian x,y to global car coordinates x,y
vector<double> getWorldXY(double car_x, double car_y, double theta, double lx, double ly) {
  vector<double> results;

  // convert back to global coordinates
  results.push_back(lx*cos(theta) - ly*sin(theta) + car_x);
  results.push_back(lx*sin(theta) + ly*cos(theta) + car_y);
  return results;
}

/*
// segment of 12 x,y coordinates for waypoints (5 in the back, 1 closest and 6 in the front) for a given lane
// get a segment of zoomed in map, such as 3 passed way points and 6 way points look ahead 

vector<vector<double>> getLocalizedWayPointSegement(double car_x, double car_y, double angle, double d, vector<double> maps_x, vector<double> maps_y, vector<double> maps_dx, vector<double> 
maps_dy,vector<double> maps_s) {

  vector<double> wplx;
  vector<double> wply;
  vector<double> wpgx;
  vector<double> wpgy;
  vector<double> wps;
  vector<vector<double>> results;
  double theta;

  int closestWaypoint = ClosestWaypoint(car_x, car_y, maps_x, maps_y);
  //int nextWaypoint = NextWaypoint(car_x, car_y, angle, maps_x, maps_y);
  int previous = closestWaypoint - NUM_WAYPOINTS_BEHIND;
  //int previous = nextWaypoint - NUM_WAYPOINTS_BEHIND;
  if (previous < 0) {
    previous += maps_x.size();
  }
  
  for (int i = 0; i < NUM_WAYPOINTS_BEHIND+1+NUM_WAYPOINTS_AHEAD; i++) {
    int next = (previous+i)%maps_x.size();
    vector<double> localxy = getLocalXY(car_x, car_y, angle, (maps_x[next]+d*maps_dx[next]), (maps_y[next]+d*maps_dy[next]));
    //vector<double> localxy = getLocalXY(car_x, car_y, theta, maps_x[next], maps_y[next]);
    cout <<"Num of waypoints: "<< next << " localized_x: " << localxy[0] << ", localized_y: " << localxy[1] <<", angle: "<< angle;
    cout << ", maps_x: " << maps_x[next] << ", maps_y: " << maps_y[next] << ", maps_s: "<< maps_s[next]<<endl;
    
    wplx.push_back(localxy[0]);
    wply.push_back(localxy[1]);
    wpgx.push_back(maps_x[next]);
    wpgy.push_back(maps_y[next]);
    wps.push_back(maps_s[next]);

  }
    

// cout << endl;
  results.push_back(wplx);
  results.push_back(wply);
  results.push_back(wpgx);
  results.push_back(wpgy);

  results.push_back(wps);
  return results;
}
*/



// convert a set of local x,y vector coordinates to world x y vectors
vector<vector<double>> getWorldPoints(double car_x, double car_y, double car_yaw, vector<double> lx, vector<double> ly) {
  vector<double> wx;
  vector<double> wy;
  vector<vector<double>> results;
  double theta = deg2rad(car_yaw);

  for (int i = 0; i < lx.size(); i++) {
    vector<double> worldxy = getWorldXY(car_x, car_y, theta, lx[i], ly[i]);
    //cout << next << "world_x: " << worldxy[0] << "\t world_y: " << worldxy[1] << ","<< endl;
    wx.push_back(worldxy[0]);
    wy.push_back(worldxy[1]);
  }
  results.push_back(wx);
  results.push_back(wy);

  return results;
}

// generate constant acceletion curve from 0 - certent speed, use dist_inc as speed 
vector<vector<double>> getAccCurve (double car_s, double car_speed, int my_lane){
  vector<double> travel_s;
  vector<double> travel_d;
  vector<vector<double>> results;
  double car_a = 50*POWER*MAX_ACCEL; 
  double car_vel = 0;				
  double car_travel = 0;
  double cum = 0;
  for (int i = 0; i < NUM_POINTS; i++){
    car_vel += car_a * DELTA_T;
    car_travel += car_vel*DELTA_T + 0.5*car_a*DELTA_T*DELTA_T;
    if (car_travel > DISTANCE_INCREMENT_LIMIT){
	car_travel = DISTANCE_INCREMENT_LIMIT;
	
    }			
    cum +=car_travel; 
    //cout << "t:"<< DELTA_T<<"\t "<<i<< "\t"<<car_vel*2.24<<"MPH, \t"<< car_travel<< " m/step \t"<<car_s+cum<<endl;
    travel_d.push_back(2+4*my_lane);
    travel_s.push_back(car_s+ car_travel);
  }
  results.push_back(travel_s);
  results.push_back(travel_d);

  return results;
}	




// smooth extended path
vector<double> interpolate_points(vector<double> pts_x, vector<double> pts_y, double start,
                                  double interval, int output_size) {
  // uses the spline library to interpolate points connecting a series of x and y values
  // output is output_size number of y values beginning at y[0] with specified fixed interval

  if (pts_x.size() != pts_y.size()) {
    cout << "ERROR! SMOOTHER: interpolate_points size mismatch between pts_x and pts_y" << endl;
    return { 0 };
  }

  tk::spline s;
  s.set_points(pts_x,pts_y);    // currently it is required that X is already sorted
  vector<double> output;
  for (int i = 0; i < output_size; i++) {
    //output.push_back(s(pts_x[0] + i * interval));
    output.push_back(s(start + i * interval));
  }
  return output;
}

vector<double> interpolate_points(vector<double> pts_x, vector<double> pts_y, 
                                  vector<double> eval_at_x) {
  // uses the spline library to interpolate points connecting a series of x and y values
  // output is spline evaluated at each eval_at_x point

  if (pts_x.size() != pts_y.size()) {
    cout << "ERROR! SMOOTHER: interpolate_points size mismatch between pts_x and pts_y" << endl;
    return { 0 };
  }

  tk::spline s;
  s.set_points(pts_x,pts_y);    // currently it is required that X is already sorted
  vector<double> output;
  for (double x: eval_at_x) {
    output.push_back(s(x));
  }
  return output;
}



// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y, vector<double> maps_s)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}
	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];
	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;
	double frenet_d = distance(x_x,x_y,proj_x,proj_y);
	//see if d value is positive or negative by comparing it to a center point
	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);
	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}
	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}
	frenet_s += distance(0,0,proj_x,proj_y);
	return {frenet_s,frenet_d};
}




#endif
