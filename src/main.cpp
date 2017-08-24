#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
// Additional ===================================>
//#include <cppad/cppad.hpp>
//#include <cppad/ipopt/solve.hpp>
#include <map>
#include <algorithm>
#include <iterator>
#include "spline.h"
#include <cmath>
#include "Eigen-3.3/Eigen/Dense"
#include <iomanip>
#include <random>

#include "road.h"
#include "constants.h"
#include "vehicle.h"
#include "costs.h"


using namespace std;




// for convenience
using json = nlohmann::json;


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






int main() {
  uWS::Hub h;


  Vehicle my_car = Vehicle();
  Vehicle sdc = Vehicle();
  Vehicle obs = Vehicle();


  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;


  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);
	ofstream log_file;
	log_file.open("path_planning_log.csv");

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

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &my_car, &sdc, &obs, &log_file](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
	  double angle = deg2rad(car_yaw);
	  double car_speed = j[1]["speed"];
	  car_speed *= 0.44704;    	// convert mph to m/s
	  // Previous path data given to the Planner
	  auto previous_path_x = j[1]["previous_path_x"];
	  auto previous_path_y = j[1]["previous_path_y"];
	  // Previous path's end s and d values 
	  double end_path_s = j[1]["end_path_s"];
	  double end_path_d = j[1]["end_path_d"];
	  // Sensor Fusion Data, a list of all other cars on the same side of the road.
	  auto sensor_fusion = j[1]["sensor_fusion"];

  	  double my_lane = 1;
	  double nextd = 6; 
 	  json msgJson;

	  vector<double> next_x_vals;
	  vector<double> next_y_vals;
	  tk::spline smooth_waypoints;
	  
	  // set up smooth lane following using spline
          vector<vector<double>> localwxy = getLocalizedWayPointSegement(car_x, car_y, angle, nextd, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy, map_waypoints_s);

	  //vector<vector<double>> zoomwxy = zoom_in_wps(car_x, car_y, angle, nextd, map_waypoints_x, map_waypoints_y, map_waypoints_s);

	  // once a while, the angle flipped 180 deg, pi in rad 
          if (localwxy[0][0] > 0. ) {
          	car_yaw += 180;
		angle += pi();
        	cout << "wrong direction detected! car x,y,yaw: " << car_x << "," << car_y << "," << car_yaw-180 << " new yaw: " << car_yaw << endl;
		// get map section again
        	localwxy = getLocalizedWayPointSegement(car_x, car_y, angle, nextd, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy, map_waypoints_s);
          }

	  smooth_waypoints.set_points(localwxy[0], localwxy[1]);
	  
          int path_size = previous_path_x.size();
	  int prev_size = previous_path_x.size();
	  vector<double> ptsx, ptsy, ptss, ptsd;
	  double ref_x = car_x;
	  double ref_y = car_y;
	  double ref_yaw = deg2rad(car_yaw);
	  double ref_x2;
	  double ref_y2;

	  // Prepare the Vehicle class in order s,s_d,s_dd,d,d_d,d_dd 
	  double pos_s, s_dot, s_ddot;
	  double pos_d, d_dot, d_ddot;
	  // Other values necessary for determining these based on future points in previous path
	  double pos_x, pos_y, pos_x2, pos_y2, angle2, vel_x1, vel_y1, pos_x3, pos_y3, vel_x2, vel_y2, acc_x, acc_y;

	  int subpath_size = min(PREVIOUS_PATH_POINTS_TO_KEEP, (int)previous_path_x.size());
	  double traj_start_time = subpath_size * PATH_DT;

	  vector<vector<double>> s_jmt, d_jmt, rampup, cs;
	  vector<double> s_start, s_end, d_start, d_end, xy, xy_;
		
	  double next_s, next_d;
	

	  double max_x_cs = -99999;
	  double max_x_ra = -99999;
	  if(subpath_size< 1){

		pos_x = car_x;
		pos_y = car_y;
		angle2 = deg2rad(car_yaw);
		pos_s = car_s;
		pos_d = car_d;
		s_dot = car_speed;
		d_dot = 0;
		s_ddot = 0;
		d_ddot = 0;

		s_start.push_back(pos_s);
		s_start.push_back(s_dot);
		s_start.push_back(s_ddot);
		
		s_end.push_back(pos_s+20);
		s_end.push_back(22.0);
		s_end.push_back(s_ddot);
		//generate ramp up curve for 200 meter in 20 seconds
		rampup = sdc.JMT(s_start, s_end, 0.1);
		for(int i = 0; i<rampup[1].size(); i++){
			next_s = rampup[1][i];
			next_d = 2+ 4*my_lane;
			// get waypoints in global coordinate  
			xy_ = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x,
map_waypoints_y);  
			// sort in the ptsx
			if (xy_[0]>max_x_ra){
				ptsx.push_back(xy_[0]);
				ptsy.push_back(xy_[1]);
				//next_x_vals.push_back(xy_[0]);
				//next_y_vals.push_back(xy_[1]);
				max_x_ra = xy_[0];
			}
		}
          } else {

		// save some previous points
		for(int i =0; i< subpath_size; i++){
			ptsx.push_back(previous_path_x[i]);
			ptsy.push_back(previous_path_y[i]);
			//next_x_vals.push_back(previous_path_x[i]);
			//next_y_vals.push_back(previous_path_y[i]);
		}

		ref_x = previous_path_x[subpath_size-1];
		ref_y = previous_path_y[subpath_size-1];
		double ref_x2 = previous_path_x[subpath_size-2];
		double ref_y2 = previous_path_y[subpath_size-2];
		ref_yaw = atan2(ref_y-ref_y2, ref_x-ref_x2);

		vector<double> frenet = getFrenet_org(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);
		pos_s = frenet[0];
		pos_d = frenet[1];

		double disc_travel = distance(ref_x, ref_y, ref_x2, ref_y2);
		s_dot = disc_travel/DELTA_T;
		s_ddot = 0;

		s_start.push_back(pos_s);
		s_start.push_back(s_dot);
		s_start.push_back(s_ddot);
		
		s_end.push_back(pos_s+220);
		s_end.push_back(22.0);
		s_end.push_back(s_ddot);

		//generate constant speed path for 44 meter in 2 seconds
		cs = sdc.JMT(s_start, s_end, 50);
		

		for(int i = 1; i<cs[1].size(); i++){
			next_s = cs[1][i];
			next_d = 2+ 4*my_lane;
			// get waypoints in global coordinate  
			xy_ = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);  
			// sort in the ptsx
			if (xy_[0]>max_x_cs){
				ptsx.push_back(xy_[0]);
				ptsy.push_back(xy_[1]);

				//next_x_vals.push_back(xy_[0]);
				//next_y_vals.push_back(xy_[1]);

				max_x_cs = xy_[0];
			}
		}

	  }

	  vector<double>target_states = sensor(sensor_fusion, my_lane, car_speed, car_s, car_d,  map_waypoints_x, map_waypoints_y);	
	  cout<<"sensor output: ";
	  for(int i =0; i < target_states.size(); i++){
		cout <<target_states[i]<<",";
	  }
	  cout<<endl;

	  cout<<"previous size: "<<prev_size;
	  cout<<" car_x: "<<car_x<<", ref_x: "<<ref_x;
	  cout<<" car_y: "<<car_y<<", ref_y: "<<ref_y;
	  cout<<" car_yaw: "<<angle<<", ref_yaw: "<<ref_yaw;
	  cout<<" end_s: "<<end_path_s<<" end_d: "<<end_path_d;
	  cout<<" car_s: "<<car_s<<" car_d: "<<car_d;



	  for(int i = 0; i < NUM_POINTS*2; i++){
			
		double next_s = car_s + POWER*DISTANCE_INCREMENT_LIMIT * (i+1);
		double next_d = 2+ 4*my_lane;
			
			
		// get waypoints in global coordinate  
		vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);  

		// get waypoints in car coordinate

		double diff_x = xy[0] - car_x;
		double diff_y = xy[1] - car_y;

		double lx = diff_x*cos(angle) + diff_y*sin(angle);
		double ly = -diff_x*sin(angle) + diff_y*cos(angle);
		double sy = smooth_waypoints(lx);

		// transform back to global coordinate
		double gx = lx*cos(angle)-sy*sin(angle)+car_x;
		double gy = lx*sin(angle)+sy*cos(angle)+car_y;

		//next_x_vals.push_back(xy[0]);
		//next_y_vals.push_back(xy[1]);


		//smoother path
		next_x_vals.push_back(gx);
		next_y_vals.push_back(gy);
		
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

	log_file.close();
}
















































































