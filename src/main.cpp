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


using namespace std;

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

// Reuse some code from term 2 projects 
// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Evaluate a polynomial derivative
double polyderv(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += coeffs[i]*i*CppAD::pow(x, i-1);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
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

	double angle = abs(theta-heading);
	cout << "map x: "<< map_x <<";"<<"Map_y : "<< map_y << endl;
	if(angle > pi()/4)
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


int main() {
  uWS::Hub h;

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

  int num_samples = 10; // iteration to find best path
  
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
&ref_vel, &vx,&vy,&vd,&xyd,&nextd,&inc_max,&dist_inc,&out_log,&timestep,&stucktimer,&lanechange, &SIGMA_S, &SIGMA_D, & SIGMA_T, &MAX_JERK, &MAX_ACC, &EXPECTED_JERK_IN_ONE_SEC, &EXPECTED_ACC_IN_ONE_SEC, &SPEED_LIMIT, &VEHICLE_RADIUS, &prev_v](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

		



          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];

          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];
		

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
                cout << sensor_fusion << endl;
		// 
		

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
		


	       	vector<double> lx; // local coordinate x
          	vector<double> ly; // local coordinate y

		vector<double> next_x_vals;
          	vector<double> next_y_vals;
		// double lx_a;
		
		int path_size = previous_path_x.size();
		int prev_size = previous_path_x.size();
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
		double delta_t = 0;
		double init_v = 0;
		double car_a = 1.0*MAX_ACC; 
		double car_vel = 0;				
		double car_travel = 0;
		for (int i = 0; i < num_points; i++){
			delta_t = 0.02;
			car_vel += car_a * delta_t;
			car_travel = car_vel*delta_t + 0.5*car_a*delta_t*delta_t;
			if (car_travel > inc_max){
				car_travel = inc_max;
			}			
			cout << "t:"<< delta_t<<"\t "<<i<< "\t"<<car_vel<<"\t"<< car_travel<< endl;
			t.push_back(double(i));
			inc.push_back(car_travel);
		}
 


            	smooth_speed.set_points(t,inc);

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
			cout << lx[i]<< " \t dist_inc :" << dist<<"\t"<< ly[i]<<endl;
			
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
		
		

		// speed control, avoid front end collision 
		for (int i = 0; i < sensor_fusion.size(); i++){
			// find the d value 
			float d = sensor_fusion[i][6];
			if (d < (2+4*lane+2) && d > (2+ 4*lane -2)){
				double vx = sensor_fusion[i][3];
				double vy = sensor_fusion[i][4];
				double check_speed = sqrt(vx*vx+vy*vy);
				double check_car_s = sensor_fusion[i][5];
				// simplified car_s prediction 
				check_car_s += ((double)prev_size*0.02*vx); 
				// buffer zoom set to 30m 
				if((check_car_s > car_s) && ((check_car_s - car_s) < 30 )){
			
					too_close = true;
					if(lane >0){
						lane -= 1;
					}
				}
			}
		}
					
		if (too_close){
			
			ref_vel = 22.4; // slow down
			dist_inc = 0.3; 
		// }else if(ref_vel < 49.5){
		}else if(dist_inc <0.445){
			ref_vel = 44.5; // speed up
			dist_inc = 0.445;
		}

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
			// next_y_vals.push_back(xy[1]);
			next_y_vals.push_back(next_y);
          		
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
















































































