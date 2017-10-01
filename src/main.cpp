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
#include "utils.h"


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



int main() {
  uWS::Hub h;


  Vehicle sdc = Vehicle();
  Vehicle obs = Vehicle();


  //sdc.initialize_state_machine();

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x, hd_x;
  vector<double> map_waypoints_y, hd_y;
  vector<double> map_waypoints_s, hd_s;
  vector<double> map_waypoints_dx, hd_dx;
  vector<double> map_waypoints_dy, hd_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  // Start in lane 1 (0 is left, 1 is middle, 2 is right)
  int lane = 1;
  int counter =1;
  double target_d= 6;
 

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


  int num_itp = (map_waypoints_s[map_waypoints_s.size()-1] - map_waypoints_s[0])/HD_RES;
  // first interpolated s 
  hd_s.push_back(map_waypoints_s[0]);
  for (int i = 1; i < num_itp; i++) {
	hd_s.push_back(map_waypoints_s[0] + i * HD_RES);
  }

  hd_x = getHD(map_waypoints_s, map_waypoints_x, HD_RES, num_itp);
  hd_y = getHD(map_waypoints_s, map_waypoints_y, HD_RES, num_itp);
  hd_dx = getHD(map_waypoints_s, map_waypoints_dx, HD_RES, num_itp);
  hd_dy = getHD(map_waypoints_s, map_waypoints_dy, HD_RES, num_itp);



  h.onMessage([&sdc, &obs, &lane, &counter, &hd_x, &hd_y, &hd_dx, &hd_dy, &hd_s, &target_d, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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


          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

		int path_size = previous_path_x.size();


		// Setup the start and end vectors for s_state and d_state
		double s_, s_dot, s_ddot;
	  	double d_, d_dot, d_ddot;

		// at cold start, car speed is actually equal 0, given a small number to avoid spline error
		s_ = car_s;
		s_dot = max(car_speed, 0.1);
		s_ddot = 0;

		// at cold start, car always keep in the same lane
		d_ = car_d;
		d_dot = 0;
		d_ddot = 0;

		double ref_start_time, ref_s, ref_d;

 		
		if(path_size <2){
		    ref_start_time = 0;
		    ref_s = s_;
		    ref_d = d_;
		} else {
		    ref_start_time = path_size*DELTA_T;
		    ref_s = end_path_s;
		    ref_d = end_path_d;
		}

		//cout << "ref_s "<< ref_s << endl;

	
	  	// sort sensor fusion info by vehicle id
	  	map<int, vector<vector<double>>> predictions;
	  	for (auto sf: sensor_fusion) {
			double obs_v = sqrt(pow((double)sf[3], 2) + pow((double)sf[4], 2));
			Vehicle obs = Vehicle(sf[5], obs_v, 0, sf[6], 0, 0);
			int v_id = sf[0];
			//vector<vector<double>> preds = obs.getPredictions(ref_start_time, hd_s, hd_x, hd_y, car_x, car_y, angle);
			vector<vector<double>> preds = obs.getPredictions(ref_start_time);
		    	predictions[v_id] = preds;
	  	}

		vector<double> lane_target_states = sdc.getLaneState(sensor_fusion, lane, car_speed, car_s, car_d,  hd_x, hd_y);
		vector<vector<double>> states_pool, seed_states;
		vector<double> seed_state;
		    
	  	cout<<"sensor output: "<<lane_target_states.size()<<endl;
		    
	  	for(int i = 0 ; i < lane_target_states.size()/7; i++){
			
			cout<< "state"<<i<<": \t";
			cout <<lane_target_states[i*7+0]<<",";
			cout <<lane_target_states[i*7+1]<<",";
			cout <<lane_target_states[i*7+2]<<",";
			cout <<lane_target_states[i*7+3]<<",";
			cout <<lane_target_states[i*7+4]<<",";
			cout <<lane_target_states[i*7+5]<<",";
			cout <<lane_target_states[i*7+6]<<","<<endl;
			
			seed_state.push_back(lane_target_states[i*7+0]);
			seed_state.push_back(lane_target_states[i*7+1]);
			seed_state.push_back(lane_target_states[i*7+2]);
			seed_state.push_back(lane_target_states[i*7+3]);
			seed_state.push_back(lane_target_states[i*7+4]);
			seed_state.push_back(lane_target_states[i*7+5]);
			seed_state.push_back(lane_target_states[i*7+6]);
		    
		        seed_states.push_back(seed_state);
			seed_state.clear();
	  	}
			

		states_pool = sdc.perturb_goal(seed_states);
		
		//int car_lane = Utils::getLaneNumberForD(d_);
		//int l_lane = Utils::getLaneNumberForD(d_-4.0);
		//int r_lane = Utils::getLaneNumberForD(d_+4.0);

		//cout<< "seed speed for current lane" << seed_states[car_lane][1]<< endl;
		/*   
		cout << " states pool"<<endl;
		for (int i = 0; i < states_pool.size(); i++){
		    for(int j = 0; j<states_pool[i].size(); j++){
			cout<<i<<" : "<<states_pool[i][j];
		    }
		    cout <<endl;
		} */
		int best_lane, c_lane;
		c_lane = Utils::getLaneNumberForD(car_d);
		vector<double> clearance = sdc.checkClearance(sensor_fusion, car_s, c_lane);
			
                best_lane = sdc.findBestLane(sensor_fusion, car_s);
       		cout << "Best target lane: " << best_lane << endl;
		
		sdc.update_states(s_,s_dot, s_ddot, d_, d_dot, d_ddot, ref_s, ref_d, ref_start_time,  "KL", lane, c_lane, clearance[2]);
  

		sdc.getLaneChangeClearance(sensor_fusion, predictions);


		string display = sdc.display();
                cout << display << endl;
		// avoid double shift
		c_lane = sdc.execute(best_lane, c_lane, sensor_fusion, counter);


		// Regen the driving path
		vector<double> m_ptsx, m_ptsy, m_ptss;
    		vector<double> f_ptsx, f_ptsy, f_ptss;

		double m_yaw;
		double first_x, before_first_x, first_y, before_first_y, diff_my, diff_mx;


		if (path_size < 2) {
			double prev_s = car_s - 1;
			double prev_x = car_x - 1*cos(angle);
			double prev_y = car_y - 1*sin(angle);
			first_x = car_x;
			first_y = car_y;
			m_ptss.push_back(prev_s);
			m_ptsx.push_back(prev_x);
			m_ptsy.push_back(prev_y);
			m_ptss.push_back(s_);
			m_ptsx.push_back(car_x);
			m_ptsy.push_back(car_y);
			//cout<<"path size "<< path_size<<endl;
		} else {
			s_ = end_path_s;
			double prev_s = s_ - s_dot*DELTA_T;
			m_ptss.push_back(prev_s);
			m_ptsx.push_back(previous_path_x[path_size-2]);
			m_ptsy.push_back(previous_path_y[path_size-2]);
			m_ptss.push_back(s_);
			m_ptsx.push_back(previous_path_x[path_size-1]);
			m_ptsy.push_back(previous_path_y[path_size-1]);
			
			first_x = (previous_path_x[path_size-1]);
			before_first_x = (previous_path_x[path_size-2]);
			first_y = (previous_path_y[path_size-1]); 
			before_first_y = (previous_path_y[path_size-2]);
			diff_mx = first_x - before_first_x;
			diff_my = first_y - before_first_y;
			m_yaw = atan2(diff_my, diff_mx);
			//cout<<"path size "<< path_size<<endl;
		} 



		// last two points of coarse trajectory, use target_d and current s + 30,60
		double target_s1 = s_ + 30;
		double target_d1 = Utils::getDForLaneNumber(c_lane);
                this_thread::sleep_for(chrono::milliseconds(300));
		vector<double> target_xy1 = getXY(target_s1, target_d1, hd_s, hd_x, hd_y);
		m_ptss.push_back(target_s1);
		m_ptsx.push_back(target_xy1[0]);
		m_ptsy.push_back(target_xy1[1]);


		double target_s2 = target_s1 + 30;
		double target_d2 = target_d1;
		vector<double> target_xy2 = getXY(target_s2, target_d2, hd_s, hd_x, hd_y);

		m_ptss.push_back(target_s2);
		m_ptsx.push_back(target_xy2[0]);
		m_ptsy.push_back(target_xy2[1]);
		
		vector<double> mc_ptsx, mc_ptsy;

		// convert to car local coordinate system
		for (int i = 0; i < m_ptsx.size(); ++i){
			double dx = m_ptsx[i]-first_x;
			double dy = m_ptsy[i]-first_y;
			mc_ptsx.push_back(dx*cos(m_yaw) + dy*sin(m_yaw));
			mc_ptsy.push_back(-dx*sin(m_yaw)+ dy*cos(m_yaw));
		}
		 /*   
		    cout<<" m points s array ";
		    for(int i = 0; i<m_ptsx.size(); i++){
			cout<<m_ptss[i]<<", ";
			//cout<<mc_ptsx[i]<<", ";
			//cout<<m_ptsy[i]<<", ";
		    }
		    cout<<endl;
		
		    cout<<" m points y array ";
		    for(int i = 0; i<m_ptsx.size(); i++){
			cout<<m_ptss[i]<<", ";
			//cout<<m_ptsx[i]<<", ";
			//cout<<mc_ptsy[i]<<", ";
		    }
		    cout<<endl;
                    
		*/


		tk::spline mc_s, mc_sx, mc_sy, hd_sx, hd_sy;
		mc_s.set_points(mc_ptsx, mc_ptsy);

		//double target_v = max(states_pool[best_traj_idx][1], target_vd[1]);
		

		double target_v = min(clearance[2], SPEED_LIMIT);
		//cout<<"target_v "<<target_v*2.24<<endl;

                double current_s = car_s;
		double current_v = s_dot;
		//double current_a = s_ddot;
		// break up previous path points to achieve correct velocity
		double horizon_x = 30.0;
		double horizon_y = mc_s(horizon_x);
		double horizon_dist = sqrt((horizon_x*horizon_x)+(horizon_y*horizon_y));

		// save some of the previous path as start point for new path
		for(int i = 0; i < path_size; i++) {
		        next_x_vals.push_back(previous_path_x[i]);
		        next_y_vals.push_back(previous_path_y[i]);
		}


		// speed P controller
		double v_inc;
		double diff_v = target_v - current_v;
		if (fabs(diff_v) < 2.0*V_INC){
		    current_v = target_v;
		} else {
		    v_inc = (diff_v)/(fabs(diff_v))*V_INC;
	   	    current_v +=v_inc+0.10*diff_v;
	   	    
		} 

		double x_inc = 0;
		
		for (int i = 0; i< NUM_POINTS-path_size; i++){

		    double N = (horizon_dist/(0.02*current_v)); 
  		    double x_point = x_inc+(horizon_x)/N;
		    double y_point = mc_s(x_point);


		    x_inc = x_point;
			
		    // convert back to global coordinate
		    double x_ref = x_point;
              	    double y_ref = y_point;

              	    x_point = (x_ref*cos(m_yaw)-y_ref*sin(m_yaw));
              	    y_point = (x_ref*sin(m_yaw)+y_ref*cos(m_yaw));

              	    x_point += first_x;
              	    y_point += first_y;

		    //cout <<"x_point: "<< x_point << "y_point: "<<y_point<<endl; 
			
		    next_x_vals.push_back(x_point);
             	    next_y_vals.push_back(y_point);
		}


          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(50));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                counter += 1;
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
















































































