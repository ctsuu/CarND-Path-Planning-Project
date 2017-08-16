#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
// Additional ===================================>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <map>
#include <iterator>
#include "spline.h"
#include <cmath>
#include "Eigen-3.3/Eigen/Dense"
#include <iomanip>
#include <random>
//#ifndef VEHICLE_H
//#define VEHICLE_H


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


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
	// if(angle > pi()/4)
	if(angle > pi()/2)
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

// Transform from Frenet s,d coordinates to Cartesian x,y
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


// segment of 25 x,y coordinates for waypoints (6 in the back, 1 closest and 18 in the front) for a given lane
vector<vector<double>> getLocalizedWayPointSegement(double car_x, double car_y, double car_yaw, double d, vector<double> maps_x, vector<double> maps_y, vector<double> maps_dx, vector<double> maps_dy) {
  vector<double> wpx;
  vector<double> wpy;
  vector<vector<double>> results;
  double theta = deg2rad(car_yaw);

  int closestWaypoint = ClosestWaypoint(car_x, car_y, maps_x, maps_y);
  int previous = closestWaypoint - 6;
  if (previous < 0) {
    previous += maps_x.size();
  }
  // cout << "waypoints: ";
  for (int i = 0; i < 25; i++) {
    int next = (previous+i)%maps_x.size();
    vector<double> localxy = getLocalXY(car_x, car_y, theta, (maps_x[next]+d*maps_dx[next]), (maps_y[next]+d*maps_dy[next]));
    // cout << next << ":" << localxy[0] << ":" << localxy[1] << ",";
    wpx.push_back(localxy[0]);
    wpy.push_back(localxy[1]);
  }
  // cout << endl;
  results.push_back(wpx);
  results.push_back(wpy);

  return results;
}


// convert a set of world x,y vector coordinates to local x y vectors
vector<vector<double>> getLocalizedPoints(double car_x, double car_y, double car_yaw, vector<double> wx, vector<double> wy) {
  vector<double> lx;
  vector<double> ly;
  vector<vector<double>> results;
  double theta = deg2rad(car_yaw);

  for (int i = 0; i < wx.size(); i++) {
    vector<double> localxy = getLocalXY(car_x, car_y, theta, wx[i], wy[i]);
    lx.push_back(localxy[0]);
    ly.push_back(localxy[1]);
  }
  results.push_back(lx);
  results.push_back(ly);

  return results;
}


// convert a set of local x,y vector coordinates to world x y vectors
vector<vector<double>> getWorldPoints(double car_x, double car_y, double car_yaw, vector<double> lx, vector<double> ly) {
  vector<double> wx;
  vector<double> wy;
  vector<vector<double>> results;
  double theta = deg2rad(car_yaw);

  for (int i = 0; i < lx.size(); i++) {
    vector<double> worldxy = getWorldXY(car_x, car_y, theta, lx[i], ly[i]);
    wx.push_back(worldxy[0]);
    wy.push_back(worldxy[1]);
  }
  results.push_back(wx);
  results.push_back(wy);

  return results;
}

// generate constant acceletion curve from 0 - certent speed, use dist_inc as speed 
vector<vector<double>> getAccCurve (double max_acc, double delta_t, double max_speed, double pwr, int steps){
  vector<double> inc;
  vector<double> travel;
  vector<vector<double>> results;
  double car_a = pwr*max_acc; 
  double car_vel = 0;				
  double car_travel = 0;
  for (int i = 0; i < steps; i++){
    car_vel += car_a * delta_t;
    car_travel = car_vel*delta_t + 0.5*car_a*delta_t*delta_t;
    if (car_travel > max_speed){
	car_travel = max_speed;
    }			
    // cout << "t:"<< delta_t<<"\t "<<i<< "\t"<<car_vel*2.24<<"MPH, \t"<< car_travel<< " m/step"<<endl;
    inc.push_back(double(i));
    travel.push_back(car_travel);
  }
  results.push_back(inc);
  results.push_back(travel);

  return results;
}	


// generate constant brake G curve from speed 1 to speed 2, use dist_inc as speed
vector<vector<double>> getBrakeCurve (double max_acc, double delta_t, double speed_1, double speed_2, double pwr, int steps){
  vector<double> inc;
  vector<double> travel;
  vector<vector<double>> results;
  double car_a = -pwr*max_acc; 
  double car_vel = speed_1*112.3*0.447 ; // m/s			
  double car_travel;

  for (int i = 0; i < steps; i++){
		
    car_vel += car_a * delta_t;
    car_travel = car_vel*delta_t+0.5*car_a*delta_t*delta_t;
		
    if (car_travel <= speed_2){
	car_travel = speed_2;
    }			
    // cout << "t:"<< delta_t<<"\t "<<i<< "\t"<<car_vel*2.24<<"MPH, \t"<< car_travel<< " m/step"<<endl;
    inc.push_back(double(i));
    travel.push_back(car_travel);
  }
  results.push_back(inc);
  results.push_back(travel);

  return results;
}


// Trajectory Generation
vector<vector<double>> JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    
    MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;
		
	MatrixXd B = MatrixXd(3,1);	    
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];
			    
	MatrixXd Ai = A.inverse();
	
	MatrixXd C = Ai*B;
	
	vector <double> coeff = {start[0], start[1], .5*start[2]};

	for(int i = 0; i < C.size(); i++){
	    	coeff.push_back(C.data()[i]);
	}
	// generate the curve
	vector <double> cum_time;
	vector <double> s_at_time; // 
	vector<vector<double>> results;
	double cum_t = 0;
	for (int i = 0; i < T/0.02; i++){
		cum_t += 0.02;
		double s_t = coeff[0]+coeff[1]*cum_t + coeff[2]*cum_t*cum_t +coeff[3]*cum_t*cum_t*cum_t + coeff[4]*cum_t*cum_t*cum_t*cum_t + coeff[5]*cum_t*cum_t*cum_t*cum_t*cum_t;
		cum_time.push_back(cum_t);
		s_at_time.push_back(s_t);
		//cout<<"cum time \t"<< cum_t<< "s_at_time" << s_t << endl;
	}
	results.push_back(cum_time);
	results.push_back(s_at_time);

    return results;
    
}

// Produce perturbed goals
vector<vector<double>> perturb_goal(vector< double> goal_s, vector <double> goal_d, double goal_t, int num_samples ){


	std::default_random_engine gen;

	// Uncertainty generators for s, d, t around goal mean
	std::normal_distribution<double> ndist_s(goal_s[0], 15); // within typical 2 car length
	std::normal_distribution<double> ndist_d(goal_d[0], 2); // within the typical 4m lane width 
	std::normal_distribution<double> ndist_t(goal_t, 2); // within 2 second reaction time
	
	
	//std::map<int, int> hist_s;
	//std::map<int, int> hist_d;
	//std::map<int, int> hist_t;


	vector<double> new_goal_s;
	vector<double> new_goal_d;
	vector<double> new_goal_t;
	vector<vector<double>> results;

    	for(int n=0; n<num_samples; ++n) {
        	//++hist_s[ndist_s(gen)];
		//++hist_d[ndist_d(gen)];
		//++hist_t[ndist_t(gen)];
		new_goal_s.push_back(abs(ndist_s(gen))); // always generate forward path
		new_goal_d.push_back(ndist_d(gen));
		new_goal_t.push_back(abs(goal_t+ndist_t(gen))); // always generate future value 
    	}

	/*
	for (int i=0; i< new_goal_s.size(); i++){
        	cout <<i<<"\t"<<new_goal_s[i]<<"\t"<<new_goal_d[i]<<"\t"<<new_goal_t[i]<<endl;
	}
	
	
    	for(auto p : hist_d) {
        	std::cout << std::fixed << std::setprecision(1) << std::setw(2)
                  << p.first << ' ' << std::string(p.second/200, '*') << '\n';
    	}
	*/
	results.push_back(new_goal_s);
	results.push_back(new_goal_d);
	results.push_back(new_goal_t);
	return results;
}



// Finds the best trajectory according to WEIGHTED_COST_FUNCTIONS (global).
vector<vector<double>> PTG(vector<double> start_s, vector<double> start_d, int target_vehicle, vector<double>delta, double T, vector<vector<double>>predictions, int num_samples, double delta_t){
   
    /*	
    arguments:
     start_s - [s, s_dot, s_ddot]

     start_d - [d, d_dot, d_ddot]

     target_vehicle - id of leading vehicle (int) which can be used to retrieve
       that vehicle from the "predictions" dictionary. This is the vehicle that 
       we are setting our trajectory relative to.

     delta - a length 6 array indicating the offset we are aiming for between us
       and the target_vehicle. So if at time 5 the target vehicle will be at 
       [100, 10, 0, 0, 0, 0] and delta is [-10, 0, 0, 4, 0, 0], then our goal 
       state for t = 5 will be [90, 10, 0, 4, 0, 0]. This would correspond to a 
       goal of "follow 10 meters behind and 4 meters to the right of target vehicle"

     T - the desired time at which we will be at the goal (relative to now as t=0)

     predictions - dictionary of {v_id : vehicle }. Each vehicle has a method 
       vehicle.state_in(time) which returns a length 6 array giving that vehicle's
       expected [s, s_dot, s_ddot, d, d_dot, d_ddot] state at that time.

    return:
     (best_s, best_d, best_t) where best_s are the 6 coefficients representing s(t)
     best_d gives coefficients for d(t) and best_t gives duration associated w/ 
     this trajectory.
    */
	
	vector<double> target = predictions[target_vehicle];
	// generate alternative goals

	vector<vector<vector<double>>> all_goals;
	double t_inc = T-4*delta_t;
	vector<double> target_state;
	target_state.reserve(target.size());
	vector<vector<double>>s_curve;
	vector<vector<double>>d_curve;
	vector<double> goal_s;
	vector<double> goal_d;

	for (int i =0; i< target.size(); i++){
		target_state.push_back(target[i]+delta[i]);
	}
	for( int i = 0; i< num_samples; i++){
		goal_s = {target_state[0], target_state[1], target_state[2]}; 
 		goal_d = {target_state[3], target_state[4], target_state[5]}; 
		vector<vector<double>> perturbed = perturb_goal(goal_s, goal_d, T, num_samples);
		all_goals.push_back(perturbed);
	}
	
	// find best trajectory
	for (int i=0; i<all_goals.size(); i++){
		// for each goal, get s_curve and d_curve
		s_curve = JMT(start_s, goal_s, all_goals[i][2][i]);
		d_curve = JMT(start_d, goal_d, all_goals[i][2][i]);

	}
	cout<<all_goals.size()<<endl;

	
	return s_curve;

}


// derivate function
vector<double> differentiate(vector<double> coeffs){
    /*
    Calculates the derivative of a polynomial and returns
    the corresponding coefficients.
    
    new_cos = []
    for deg, prev_co in enumerate(coefficients[1:]):
        new_cos.append((deg+1) * prev_co)
        #print(new_cos)
    return new_cos
    */
	
    vector<double> s_dot;
    for (int i =1; i< coeffs.size(); i++){
      s_dot.push_back(coeffs[i]*i);
    }
    return s_dot;
}


// rebuild polynormial equation
double to_equation(vector<double> coeffs, double t){
    /*
    Takes the coefficients of a polynomial and creates a function of
    time from them.
    
    def f(t):
        total = 0.0
        for i, c in enumerate(coefficients): 
            total += c * t ** i
        return total
    return f
    */

	double results = 0;
	for(int i = 0; i < coeffs.size(); i++){
		results += coeffs[i]*pow(t, i);
	}

	return results;
}

// logistic function
double logistic(double x){
    /*
    A function that returns a value between 0 and 1 for x in the 
    range [0, infinity] and -1 to 1 for x in the range [-infinity, 0].

    Useful for cost functions.
    
    return 2.0 / (1 + exp(-x)) - 1.0
    */	
    return 2.0/(1+exp(-x))-1.0;
}

// Project Vehicle move with constant acceleration

vector<double> state_in(vector<double> start_state, double t){
      vector<double> state;
      state = {
	start_state[0]+(start_state[1]*t)+0.5*start_state[2]*t*t,
	start_state[1]+start_state[2]*t,
	start_state[2],
	start_state[3]+(start_state[4]*t)+0.5*start_state[5]*t*t,
	start_state[4]+start_state[5]*t,
	start_state[5]
	};
      return state; 
}

// find closest distance in a trajectory to close by vehicle
double nearest_approach(vector<vector<double>> traj, vector<double> cars){
    // giving testing trajecory s_coeffs, d_coeffs, 
    // any vehicle current state {s, s_dot, s_d_dot, d, d_dot, d_d_dot}, and duration T:
    // output: the closest dist between the testing trajecory and other cars projected path. 

    double closest = 999999;
    vector<double>s_ = traj[0];
    vector<double>d_ = traj[1];
    vector<double>t_ = traj[2];
    //s = to_equation(s_)
    //d = to_equation(d_)
    for(int i = 1; i < 100; i++){
        double t = i / 100 * t_[0];
        double cur_s = to_equation(s_, t); // s coordinate value at time t
        double cur_d = to_equation(d_, t); // d coordinate value at time t

	// giving other car's s and d value
	vector<double> target = state_in(cars, t);
        
        double dist = sqrt(pow((cur_s-target[0]),2) + pow((cur_d-target[3]),2));
        //cout<< dist<<"," ;
        if (dist < closest){
            closest = dist;
        }
    }
    return closest;
}


double nearest_approach_to_any_vehicle(vector<vector<double>> traj, vector<vector<double>>vehicles){
    
    // Calculates the closest distance to any vehicle during a trajectory.
    
    double closest = 999999;
    for(int i = 0; i < vehicles.size(); i++){
        double d = nearest_approach(traj,vehicles[i]);
	cout << d <<",";
        if (d < closest){
            closest = d;
        }

    }
    return closest;
}


// add some cost functions

double time_diff_cost(vector<vector<double>>traj, double T){
    /*
    Penalizes trajectories that span a duration which is longer or 
    shorter than the duration requested.
    */
    double t = traj[2][0];
    return logistic(float(abs(t-T)) / T);
}




int main() {
  uWS::Hub h;

  //struct Vehicle cars;

  
  // initial start state, middle lane,  
  //vector<double> start_state = {10,0.445,0,6,0,0};
/*  
  vector<double> next_state = cars.state_in(start_state, 1);
  for (int i =0; i< next_state.size(); i++){
        cout<<next_state[i]<<",";
  }
  */


  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  double max_s = 6945.554;

  // for our spline fit
  vector<double> vx;
  vector<double> vy;
  vector<double> vd;
  vector<double> xyd;


  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  // The max s value before wrapping around the track back to 0
  double nextd = 6.;
  // max speed ~ 49.75MPH
  const double inc_max = 0.445;
  double dist_inc = inc_max;
  int timestep = 0;
  int stucktimer = 0;
  bool lanechange = false;
  double ref_vel = 0; // starting refering speed in mph
	
  int lane = 2; // 0 will be the inner lane, 2 will be the right lane


  // double s;
  // double v;
  // double a;
  double target_speed;
  int lanes_available;
  int goal_lane;
  
  string state; // car states: "CS", "KL", 
  


  int num_samples = 20; // iteration to find best path
  
  Eigen::VectorXd SIGMA_S(3); 
          SIGMA_S << 10.0 , 4.0 , 2.0 ; //S, S_dot, S_double_dot
  Eigen::VectorXd SIGMA_D(3);
	  SIGMA_D << 1.0 , 1.0 , 1.0 ;  // D, D_dot, D_double_dot
  double SIGMA_T;
  

  double MAX_JERK = 10; // m/s/s/s

  double MAX_ACC = 10; // m/s/s

  double EXPECTED_JERK_IN_ONE_SEC = 2.0; // m/s/s
  double EXPECTED_ACC_IN_ONE_SEC = 1.0; // m/s

  double SPEED_LIMIT = 50; // in MPH
  double VEHICLE_RADIUS = 1.5; // to build the vehicle model

  double prev_v = 0; // previous car speed

  
		


  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // set up logging
  string log_file = "../data/logger.csv";
  ofstream out_log(log_file.c_str(), ofstream::out);
  out_log << "t,x,y,vd,xyd,nd,d,st" << endl;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,
&ref_vel, &vx,&vy,&vd,&xyd,&nextd,&inc_max,&dist_inc,&out_log,&timestep,&stucktimer,&lanechange, &SIGMA_S, &SIGMA_D, & SIGMA_T, &MAX_JERK, &MAX_ACC, &EXPECTED_JERK_IN_ONE_SEC, &EXPECTED_ACC_IN_ONE_SEC, &SPEED_LIMIT, &VEHICLE_RADIUS, &prev_v, &target_speed, &lanes_available, &goal_lane, &state, &num_samples](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);



      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];  
		double car_v = car_speed* 0.447;  // need convert from MPH to m/s
		double angle = deg2rad(car_yaw);

		double delta_t = 0.02;
		



          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];

          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];
		

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
                //cout << sensor_fusion << endl;
		// 
		vector<vector<double>> prediction;
		/*
		for (int i = 0; i < sensor_fusion.size(); i++) {
			vector<double> car;
			int sf_id = sensor_fusion[i][0]; 
			double sf_x = sensor_fusion[i][1];  
			double sf_y = sensor_fusion[i][2];  
 
			double sf_s = sensor_fusion[i][5];  
			double sf_d = sensor_fusion[i][6];  
			double sf_vx = sensor_fusion[i][3];  
			double sf_vy = sensor_fusion[i][4];
			
			double sf_xt1 = sf_x+(0.02*sf_vx); 
			double sf_yt1 = sf_y+(0.02*sf_vy);
			double sf_heading = atan2(sf_yt1-sf_y,sf_xt1-sf_x); 
			vector<double> sf_frenet = getFrenet(sf_xt1, sf_yt1, sf_heading,map_waypoints_x, map_waypoints_y);  
						
			double sf_s_dot = (sf_frenet[0]-sf_s)/0.02;
			double sf_d_dot = (sf_frenet[1]-sf_d)/0.02;
			
			car.push_back(sf_s); // s
			car.push_back(sf_s_dot); // s_dot
			car.push_back(double(0)); // s_d_dot
			car.push_back(sf_d); // d
			car.push_back(sf_d_dot); // d_dot
			car.push_back(double(0)); // d_d_dot
				
			cout<<
			sf_xt1 <<", \t"<<
			sf_yt1 <<", \t"<<
			sf_heading <<", \t"<<
			angle <<", \t"<<
			sf_frenet[0] <<", \t"<<
			sf_frenet[1] <<", \t"<<	
			car.size()<< endl;
			//(sensor_fusion[i][0]) <<", \t"<< 
			//(sensor_fusion[i][1]) <<", \t"<< 
			//(sensor_fusion[i][2]) <<", \t"<< 	
			//(sensor_fusion[i][5]) <<", \t"<< 
			//(sensor_fusion[i][6]) <<", \t"<<  
			//(sensor_fusion[i][3]) <<", \t"<<
			//(sensor_fusion[i][4]) <<", \t"<<endl;
			
			prediction.push_back(car);
		}

		for(int i =0; i< prediction[0].size(); i++){
			cout<< prediction[0][i]<<", \t";
		}
		*/
          	json msgJson;

		//

		double ref_x = car_x;
		double ref_y = car_y;
		double ref_yaw = angle;           	
          	int num_points = 120; // it will take at least 2.4 seconds from 0-50 MPH

          	// for speed and steering/lane change control
          	vector<double> t;
          	vector<double> t2;
          	vector<double> inc;
          	vector<double> nd;


		tk::spline smooth_lanes;
          	tk::spline smooth_speed;
          	tk::spline smooth_local;


          	tk::spline smooth_path;
          	tk::spline accel_curve;
          	tk::spline brake_curve;
		


	       	vector<double> lx; // local coordinate x
          	vector<double> ly; // local coordinate y

		vector<double> next_x_vals;
          	vector<double> next_y_vals;
		// double lx_a;
		
		int path_size = previous_path_x.size();
		int prev_size = previous_path_x.size();
		
		//if(prev_size > 0){
		//	car_s = end_path_s;
		//}
		// somehow it make the car run too fast


		cout << "prev_path_size: " << prev_size << endl; 
		cout << "car_x: "<< car_x <<" car_y: "<< car_y << " car_yaw: "<< car_yaw << " deg" << " car_yaw in rad: "<< angle << endl;
		cout << " car_s: " << car_s <<" car_d: "<< car_d << endl; 
                cout << " end path s: " << end_path_s << " end path d: " << end_path_d << endl;

		// get our frenet coordinates
          	vector<double> frenet = getFrenet(car_x, car_y, deg2rad(car_yaw), map_waypoints_x, map_waypoints_y);

		// set up smooth lane following using spline
          	vector<vector<double>> localwxy = getLocalizedWayPointSegement(car_x, car_y, car_yaw, nextd, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);

		smooth_path.set_points(localwxy[0], localwxy[1]);


		vector<vector<double>> globalwxy = getWorldPoints(car_x, car_y, car_yaw, localwxy[0], localwxy[1]);

		smooth_lanes.set_points(globalwxy[0], globalwxy[1]);
		
		for(int i =0; i< localwxy[0].size(); i++){
			
			// cout <<"localwx: "<< localwxy[0][i] << "\t"<<"localwy: "<< localwxy[1][i]<<"\t" <<"smooth: "<< smooth_lanes(localwxy[0][i]+50) << endl;

			// cout <<"globalwx: "<< globalwxy[0][i] << "\t"<<"globalwy: "<< globalwxy[1][i]  << endl;

		}
		
		
		// accelaration curve from 0 to 50 MPH
		double pwr=1.0; // 100% power, full acceleration

		vector<vector<double>> acc_curve = getAccCurve (MAX_ACC, delta_t, inc_max, pwr, num_points);
		smooth_speed.set_points(acc_curve[0],acc_curve[1]);
            	accel_curve.set_points(acc_curve[0],acc_curve[1]);


		// brake from 50 MPH to 20 MPH
		double speed_1 = 0.445; // equal to 50 MPH
		double speed_2 = 0.200; // equal to 20 MPH
		double steps = 80; // finish the change in 1.5 second
		vector<vector<double>> brake = getBrakeCurve (MAX_ACC, delta_t, speed_1, speed_2, pwr, steps);

		brake_curve.set_points(brake[0], brake[1]);

		// car state include {s, s_dot, s_double_dot}
		// it can also refer to {s, v, a}
		//vector< double > start = {0.0, 10.0, 0.0}; // test 1
		//vector< double > end = {10.0, 10.0, 0.0};  // test 1

		vector< double > start = {0.0, 10.0, 0.0}; // test 2
		vector< double > end = {40.0, 15, 10};  // test 2

		//vector< double > start = {5, 10, 2}; // test 3
		//vector< double > end = {-30, -20, -4};  // test 3

		
		vector<vector<double>> jmt = JMT(start, end, 3);
		//cout << "JMT \t" << jmt[0] <<"\t"<<jmt[1]<<"\t"<<jmt[2]<<"\t"<<jmt[3]<<"\t"<<jmt[4]<<"\t"<<jmt[5]<<endl;

		/*
		// print out the curve
		for (int i = 1; i<= jmt[0].size(); i++){
			cout<<i<<"\t"<<jmt[0][i]<<", \t"<<jmt[1][i]<<"\t"<<jmt[1][i]-jmt[1][i-1]<<endl;

		}
		*/
		
		// generate perturbed goals
		vector< double> goal_s = {30, 0, 0};
		vector <double> goal_d = {10, 0, 0};
		double goal_t = 2;

		vector<vector<double>> new_goals = perturb_goal(goal_s, goal_d, goal_t, num_samples);
		
		// print out the curve
		cout<< "double check tracer: "<< endl;
		for (int i = 0; i< new_goals[0].size(); i++){
			
			cout<<i<<"\t"<<new_goals[0][i]<<", \t"<<new_goals[1][i]<<", \t"<<new_goals[2][i]<<endl;

		}
		
		vector<double> test = {1,1,1,1,1,1};
		//vector<double> test = {1,2,3,4,5};
       		//vector<double> test_result = differentiate(test);
		vector<double> test_result = differentiate(differentiate(test));
                cout <<"s_dot: ";
    		for (int i =0; i<test_result.size(); i++){
        		cout << ", "<<test_result[i];
    		}
		double ft = to_equation(test, 2);
		cout << "Polynormial f(t) = " << ft<< endl; 

		double log = logistic(ft);
		cout << "logistic f(t) = " << log<< endl; 
		
		vector<vector<double>> test_traj = {{1,1,1,1,1,1},{1,1,1,1,1,1},{2}};
		vector<double> test_car = {10,5,0,2,0,0};
		vector<vector<double>> test_cars = {{10,5,0,2,0,0},{100,5,0,2,0,0},{200,5,0,6,0,0}};
                //double shortest_dist = nearest_approach(test_traj, test_car);
                double nearest =  nearest_approach_to_any_vehicle(test_traj,test_cars);
		
                cout <<"nearest = "<<nearest<< endl;

		double t_diff_fast = time_diff_cost(test_traj, 1.5);
		double t_diff_slow = time_diff_cost(test_traj, 2.5);
		double t_diff_right = time_diff_cost(test_traj, 2.1);

		cout << t_diff_fast << "\t" <<t_diff_slow<< "\t"<<t_diff_right<<endl; 





		double nextlwpx = 0.;
		double nextlwpy;
            	for (int i = 0; i<num_points; i++){
              		nextlwpx += smooth_speed(double(i));
              		nextlwpy = smooth_path(nextlwpx);
              		lx.push_back(nextlwpx);
              		ly.push_back(nextlwpy);
              		if (i > 0){
                		vd.push_back(distance(lx[i-1], ly[i-1], lx[i], ly[i]));
              		}else{
                		vd.push_back(smooth_speed(double(0)));
            		}
		}


		// calculate the smoother path
            	double localxx = 0.;
            	double localxy = 0.;
            	for(int i = 1; i <= num_points; i++){
              		ly[i] = smooth_lanes(lx[i]);
              		double dist = distance(localxx, localxy, lx[i], ly[i]);
			// cout << "dist_inc :" << dist<<endl;
              		double speed = smooth_speed(double(i));
              		if (dist > speed || dist < speed*0.8){
                 		double heading = atan2(ly[i]-localxy,lx[i]-localxx);
                 		lx[i] = localxx + smooth_speed(double(i))*cos(heading);
                 		ly[i] = smooth_lanes(lx[i]);
                 		dist = distance(localxx, localxy, lx[i], ly[i]);
				//cout << lx[i]<<" \t dist_inc :" << dist<<endl;
              		}
              		localxx = lx[i];
              		localxy = ly[i];
			// cout << lx[i]<< " \t dist_inc :" << dist<<"\t"<< ly[i]<<endl;
			
            	}

		// transform lx, ly way points into global coordinate
		vector<vector<double>> worldxy = getWorldPoints(car_x, car_y, car_yaw, lx, ly);
            	for (int i=path_size; i<worldxy[0].size(); i++) {
              		vx.push_back(worldxy[0][i]);
              		vy.push_back(worldxy[1][i]);
              		//next_x_vals.push_back(worldxy[0][i]);
              		//next_y_vals.push_back(worldxy[1][i]);
            	}

		// print out the lx, ly, gx, gy
		for (int i = 0; i< lx.size(); i++){
			//cout <<" lx: " << lx[i]<< " ly: "<<ly[i]<<"\t gx:"<<vx[i]<<"\t gy:"<<vy[i]<< endl;
		}


		bool too_close = false;
		bool left_lane_clear = true;
		bool right_lane_clear = true;
		
		

		// speed control, avoid front end collision 
		for (int i = 0; i < sensor_fusion.size(); i++){
			// find the d value 
			float d = sensor_fusion[i][6];

			vector<double> car;
			int sf_id = sensor_fusion[i][0]; 
			double sf_x = sensor_fusion[i][1];  
			double sf_y = sensor_fusion[i][2];  
 
			double sf_s = sensor_fusion[i][5];  
			double sf_d = sensor_fusion[i][6];  
			double sf_vx = sensor_fusion[i][3];  
			double sf_vy = sensor_fusion[i][4];
			
			double sf_xt1 = sf_x+(0.02*sf_vx); 
			double sf_yt1 = sf_y+(0.02*sf_vy);
			double sf_heading = atan2(sf_yt1-sf_y,sf_xt1-sf_x); 
			vector<double> sf_frenet = getFrenet(sf_xt1, sf_yt1, sf_heading,map_waypoints_x, map_waypoints_y);  
						
			double sf_s_dot = (sf_frenet[0]-sf_s)/0.02;
			double sf_d_dot = (sf_frenet[1]-sf_d)/0.02;

			

			car.push_back(sf_id); // id
			car.push_back(sf_s); // s
			car.push_back(sf_s_dot); // s_dot
			car.push_back(double(0)); // s_d_dot
			car.push_back(sf_d); // d
			car.push_back(sf_d_dot); // d_dot
			car.push_back(double(0)); // d_d_dot
			

			//double vx = sensor_fusion[i][3];
			//double vy = sensor_fusion[i][4];
			double check_speed = sqrt(sf_vx*sf_vx+sf_vy*sf_vy); // m/s
			double check_car_s = sensor_fusion[i][5];
			double check_car_d = sensor_fusion[i][6];
			// lead time from any car, in seconds
			if(check_speed !=0){
				double check_lead = (check_car_s - car_s)/check_speed; 
				car.push_back(check_lead); // lead time
				// simplified car_s and car_d prediction 50 steps for 1 second 
				check_car_s += (50*0.02*sf_s_dot); 
				check_car_d += (50*0.02*sf_d_dot); 
				cout <<"check speed \t" << check_speed*2.24 <<", \t"<<sf_s_dot*2.24<<"\t lead time: \t" <<check_lead<<", "<<sf_d<<", "<<check_car_d<<endl;
			}

			// safety buffer zone set to front 40m, back 40m
			double safe_zone_front = 40.0;
			double safe_zone_back = 40.0;
 
			if(d < (2+4*lane+2) && d >(2+4*lane-2)){
				// in the same lane
				if((check_car_s > car_s) && ((check_car_s - car_s) < safe_zone_front )){
					too_close = true;
			
				}
			}
		
					
			if (too_close){
				double keep_lane_speed = check_speed;
				// prepare for lane change, shoulder check, match left lane speed, check gap to the front and behind.
				
				int left_lane = lane-1;
				double left_lane_speed;
				if (lane > 0 && left_lane_clear && d < (2+4*left_lane+2) && d > (2+4*left_lane-2)) {
					if(((check_car_s > car_s) && ((check_car_s-car_s) < safe_zone_front)) || ((check_car_s < car_s) && ((car_s-check_car_s) < safe_zone_back))){
						left_lane_clear = false;
					}
					if(check_speed<SPEED_LIMIT/2.24){
						left_lane_speed = SPEED_LIMIT*0.9;
					}
					left_lane_speed = check_speed*2.24;
					cout << "match left lane speed: "<<left_lane_speed <<endl;
				}

				// check right lane as well
				int right_lane = lane+1;
				double right_lane_speed;
				if (lane < 2 && right_lane_clear && d < (2+4*right_lane+2) && d > (2+4*right_lane-2)) {
					
					if (((check_car_s > car_s) && ((check_car_s-car_s) < safe_zone_front)) || ((check_car_s < car_s) && ((car_s-check_car_s) < safe_zone_back))) {
						right_lane_clear = false;
					}
					// prefer left lane pass first, not legal to pass on the right. Also when pass is over, return to the right lane.
					if (!left_lane_clear){
						right_lane_speed = check_speed*2.24;
						cout << "match right lane speed: \t "<<right_lane_speed*2.24<< endl;
					}
				}
			} // end of shoulder check

			prediction.push_back(car);
		}// end of sensor fusion check


		/*
		cout<<"sensor_fusion: " << endl;
		for (int i = 0; i < prediction.size(); i++){
			cout<<"sf : ";
			for(int j =0; j< prediction[0].size(); j++){
				cout<<prediction[i][j]<<", \t";
			}
		}
		*/
			
		

		vector<double> ptsx;
		vector<double> ptsy;

		
		
		//if the previous path size is almost empty, use the current car state

		if(prev_size <10){
			double prev_car_x = car_x -car_v*0.02*cos(angle);
			double prev_car_y = car_y -car_v*0.02*sin(angle);

			ptsx.push_back(prev_car_x);  // add point 1
			ptsx.push_back(car_x);       // add point 2

			
			ptsy.push_back(prev_car_y); // add point 1
			ptsy.push_back(car_y);      // add point 2
		}
		else{
			ref_x = previous_path_x[prev_size-1];
			ref_y = previous_path_y[prev_size-1];

			double ref_x_prev = previous_path_x[prev_size-2];
			double ref_y_prev = previous_path_y[prev_size-2];

			ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);


			ptsx.push_back(ref_x_prev);  // add point 1
			ptsx.push_back(ref_x);       // add point 2

			
			ptsy.push_back(ref_y_prev); // add point 1
			ptsy.push_back(ref_y);      // add point 2
	
		}

		
		
  		vector<double> next_wp0 = getXY(car_s +30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

  		vector<double> next_wp1 = getXY(car_s +60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

  		vector<double> next_wp2 = getXY(car_s +90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

		ptsx.push_back(next_wp0[0]);  // add next way point 0
		ptsx.push_back(next_wp1[0]);  // add next way point 1
		ptsx.push_back(next_wp2[0]);  // add next way point 2

		ptsy.push_back(next_wp0[1]);  // add next way point 0
		ptsy.push_back(next_wp1[1]);  // add next way point 1
		ptsy.push_back(next_wp2[1]);  // add next way point 2

		// print out the ptsx and ptsy
		for (int i = 0; i< ptsx.size(); i++){
			cout <<" ptsx: " << ptsx[i]<< " ptsy: "<<ptsy[i]<< endl;
		}

		

		// transform current way points into car local coordinate
		for (int i = 0; i < ptsx.size(); i++){
			double shift_x = ptsx[i] - ref_x;
			double shift_y = ptsy[i] - ref_y;

			ptsx[i] = (shift_x*cos(-ref_yaw) + shift_y*sin(-ref_yaw));
			ptsy[i] = (-shift_x*sin(-ref_yaw) + shift_y*cos(-ref_yaw));
		}

		// print out the ptsx and ptsy
		for (int i = 0; i< ptsx.size(); i++){
			cout <<"In car Coordinate: ptsx: " << ptsx[i]<< " ptsy: "<<ptsy[i]<< endl;
		}
		
		
		tk::spline s;
		s.set_points(ptsx, ptsy);




          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

		// double dist_inc = 0.445; 

		
		double prev_s = car_s;
		double prev_d = car_d;
		double prev_x = car_x;
		double prev_y = car_y;
		
		double adj_v = 0.9;
		double car_acc = (prev_v - car_v)/0.02; // car accelection m/s/s feed back
 

    		for(int i = 0; i < num_points; i++){
			

			double next_s = car_s + adj_v*dist_inc * (i+1);
			double next_d = 2+ 4*lane;

			double travel = car_v*0.02 + 0.5*car_acc*0.02*0.02;

			// get way points in global coordinate  
			vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);  
			/*
			double dist_ = distance(prev_x, prev_y, xy[0], xy[1]);

			cout <<" Car Traveled :" << travel<< endl; 
			//cout<< "projected dist :" <<dist_<< prev_x<<"\t "<< prev_y << "\t"<< "\t"<< xy[0] <<"\t"<< xy[1] <<endl;

			if(dist_<0.9*dist_inc){
				adj_v *=1.01;
			}else{
				adj_v *= 0.99;
			}
			*/
			// smooth out the way points
			double next_y = smooth_lanes(xy[0]);

			// cout<<"next_y: "<< next_y<< endl;

	
			next_x_vals.push_back(xy[0]);
			next_y_vals.push_back(xy[1]);
			//next_y_vals.push_back(next_y);
          		
			//lx.push_back(xy[0]);
			//ly.push_back(xy[1]);
			prev_x = xy[0];
			prev_y = xy[1];

		}
		
      		msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;



          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































