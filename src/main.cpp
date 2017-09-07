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


using Eigen::Vector2d;

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
    vector<double> map_waypoints_x, wp_x;
    vector<double> map_waypoints_y, wp_y;
    vector<double> map_waypoints_s, wp_s;
    vector<double> map_waypoints_dx, wp_dx;
    vector<double> map_waypoints_dy, wp_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

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
    

    // Credit to https://github.com/ericlavigne/ 
    // and https://github.com/WolfgangSteiner/CarND-Path-Planning-Project/blob/master/src/Waypoints.cpp
    // Splines to support conversion from s,d to x,y.
    // warp around waypoints 
    double first_s = map_waypoints_s.front();
    double first_x = map_waypoints_x.front();
    double first_y = map_waypoints_y.front();
    double first_dx = map_waypoints_dx.front();
    double first_dy = map_waypoints_dy.front();

    double last_s = map_waypoints_s.back();
    double last_x = map_waypoints_x.back();
    double last_y = map_waypoints_y.back();
    double last_dx = map_waypoints_dx.back();
    double last_dy = map_waypoints_dy.back();

    
    
    wp_s.push_back(last_s-max_s);
    wp_x.push_back(last_x);
    wp_y.push_back(last_y);
    wp_dx.push_back(last_dx);
    wp_dy.push_back(last_dy);

    for(int i=0; i<map_waypoints_s.size(); i++){
	wp_s.push_back(map_waypoints_s[i]);
	wp_x.push_back(map_waypoints_x[i]);
	wp_y.push_back(map_waypoints_y[i]);
	wp_dx.push_back(map_waypoints_dx[i]);
	wp_dy.push_back(map_waypoints_dy[i]);
    }

    wp_s.push_back(first_s+max_s);
    wp_x.push_back(first_x);
    wp_y.push_back(first_y);
    wp_dx.push_back(first_dx);
    wp_dy.push_back(first_dy);


    tk::spline s_x, s_y, s_dx, s_dy;
    s_x.set_boundary(tk::spline::second_deriv, 0.0, tk::spline::first_deriv, -2.0, false);
    s_y.set_boundary(tk::spline::second_deriv, 0.0, tk::spline::first_deriv, -2.0, false);

    s_x.set_points(wp_s,wp_x);
    s_y.set_points(wp_s,wp_y);
    s_dx.set_points(wp_s,wp_dx);
    s_dy.set_points(wp_s,wp_dy);




    // Start in lane 1 (0 is left, 1 is middle, 2 is right)
    int lane = 1;
    int counter =1;
    //double speed_limit = 49.5; // mph
    double ref_vel = POWER*SPEED_LIMIT; // m/s
	/*
	const Vector2d p(735.0, 1136.0);
	//const Vector2d p(747.618, 1155.37);
	double s = 6914;
	double error = Error(p, s, s_x, s_y);
	double errorDeriv = ErrorDeriv(p, s, s_x, s_y);
	Vector2d normal = GetNormalAt(s, s_x, s_y );
	Vector2d getxyitp = getXY(6900, -6, s_x, s_y);
	vector<double> getxy = getXY(6900, 6, s_x, s_y, s_dx, s_dy);

	const Vector2d q(getxy[0],getxy[1]);
	Vector2d s_d = getFrenet(q, 6920, s_x, s_y, s_dx, s_dy);
	
	cout <<"p and s error " << error<<" error deriv "<<errorDeriv<< endl;
	cout <<"Get normal at" << normal[0] <<", " << normal[1]<<endl;
	cout <<"Get XY interpolated " <<getxyitp[0] <<", "<<getxyitp[1]<< endl;
	cout <<"Get XY spline " <<getxy[0] <<", "<<getxy[1]<< endl;
	cout <<"Get S& D " << s_d[0] << "," << s_d[1] << endl;	    
	*/

    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy,&s_x, &s_y, &s_dx, &s_dy, &lane, &ref_vel, &my_car, &sdc, &obs, &counter](
            uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
            uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;

        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            //cout << "Receiving message: " << s << endl;

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

		    //double target_s_diff= car_speed*ref_duration; // expected distance ahead
		    double target_s_diff= 12; // expected distance ahead
		    double target_speed= 0; // expected speed difference
		    double target_accel = 0; // expected accel difference
		    double target_d = 0; // expected lane change
		    double target_t = 3; // expected 2 second target leading time 
		    double target_lane = 0; // expected lane change difference
		    /*
		    // change lane to right
			if (lane<RIGHT_LANE){
			    if (counter %100 ==0){
				lane +=1;
				//state = "LCR";
			    }
			}

		    // change lane to left
  			if( lane>LEFT_LANE){
			    if(counter %300 ==0){
				lane -=2;
				//state = "LCL";
			    }
			}

		    cout <<"lane number " << lane << " counter "<< counter<< endl;
		    */
		    vector<double> lane_target_states = sdc.get_target_state(sensor_fusion, lane, car_speed, car_s, car_d,  map_waypoints_x, map_waypoints_y);
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

		    
		    cout << " states pool"<<endl;
		    for (int i = 0; i < states_pool.size(); i++){
			for(int j = 0; j<states_pool[i].size(); j++){
			    cout<<i<<" : "<<states_pool[i][j];
			}
		    cout <<endl;
		    } 
		    

		    

                    //int prev_size = previous_path_x.size();
		    int path_size = min(RESERVED_PATH_POINT, (int)previous_path_x.size());
	  	      	    

                    vector<double> ptsx;
                    vector<double> ptsy;

		    // Setup the reference point for self car and other car's trajectory generation. 
                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);
		    double ref_start_time = (path_size) * DELTA_T;
		    double ref_duration = 0.6; // from reference point up, add new path 
		    double rampup_speed = POWER*MAX_ACCEL*ref_duration;
		    double rampup_dist = 0.5*POWER*MAX_ACCEL*ref_duration*ref_duration;  

		    // Setup the start and end vectors for s_state and d_state
		    double s_, s_dot, s_ddot;
	  	    double d_, d_dot, d_ddot;
	  	    vector<vector<double>> s_jmt, d_jmt, rampup, cs, cs_s, cs_d, smooth_gx, smooth_gy;
	    	    vector<double> s_start, s_end, d_start, d_end, xy, xy_, cs_xy_;
		    vector<double> jmt_ptsx, jmt_ptsy;
		    vector<double> best_s, best_d;
		    double best_t;
		    double next_s, next_d;
	  	    // sort sensor fusion info by vehicle id
	  	    map<int, vector<vector<double>>> predictions;
	  	    for (auto sf: sensor_fusion) {
			double obs_v = sqrt(pow((double)sf[3], 2) + pow((double)sf[4], 2));
			Vehicle obs = Vehicle(sf[5], obs_v, 0, sf[6], 0, 0);
			int v_id = sf[0];
			vector<vector<double>> preds = obs.generate_predictions(ref_start_time, ref_duration, map_waypoints_s, map_waypoints_x, map_waypoints_y, car_x, car_y, angle);
		    	/*
		    	cout << v_id<<" obs car local x predictions";
		    	for (int i = 0; i < preds[0].size(); i++){
				cout<<": "<<preds[2][i];

		    	} 
		    	cout <<endl;
		    	cout << v_id<<" obs car local y predictions";
		    	for (int i = 0; i < preds[0].size(); i++){
				cout<<": "<<preds[3][i];

		    	} 
		    	cout <<endl;

		    	cout << v_id<<" obs car s predictions";
		    	for (int i = 0; i < preds[0].size(); i++){
				cout<<": "<<preds[4][i];

		    	} 
		    	cout <<endl;

		    	cout << v_id<<" obs car d predictions";
		    	for (int i = 0; i < preds[0].size(); i++){
				cout<<": "<<preds[5][i];

		    	} 
		    	cout <<endl;

			cout << v_id<<" obs car cum t projection";

		    	for (int i = 0; i < preds[0].size(); i++){
				cout<<": "<<preds[6][i];
		    	} 

		    	cout <<endl;
			*/

		    	predictions[v_id] = preds;
	  	    }



                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
		    int best_traj_idx;


                    if(path_size < 2) {
			// start point car_x, and car_y, at time = 0
                        double prev_car_x = car_x - cos(ref_yaw);
                        double prev_car_y = car_y - sin(ref_yaw);
                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);
                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);
				
			// at cold start, car speed is actually equal 0, 
			s_ = car_s;
			s_dot = car_speed;
			s_ddot = 0;

			// at cold start, car always keep in the same lane
			d_ = car_d;
			d_dot = 0;
			d_ddot = 0;

			s_start.push_back(s_);
			s_start.push_back(s_dot);
			s_start.push_back(s_ddot);
		
			s_end.push_back(s_+0.5*rampup_dist);
			//s_end.push_back(s_dot+rampup_speed);  
			s_end.push_back(0.5*rampup_speed);  
			s_end.push_back(s_ddot);

			d_start.push_back(d_);
			d_start.push_back(d_dot);
			d_start.push_back(d_ddot);

			d_end.push_back(d_);
			d_end.push_back(d_dot);
			d_end.push_back(d_ddot);
			
			//generate ramp up curve 
			rampup = sdc.JMT(s_start, s_end, ref_duration);
			next_d = 2+ 4*lane;
			cout<<"if path_size less than 2, next_d: "<<next_d<<" next_s: ";
			for(int i = 0; i<rampup[1].size(); i++){
				next_s = rampup[1][i];
			
				cout<<next_s<<",";
			

				// get waypoints in global coordinate  
				//xy_ = getXY(next_s, next_d, s_x, s_y, s_dx, s_dy);  
				Vector2d xy_ = getXY(next_s, next_d, s_x, s_y);
				/*
				// get waypoints in car coordinate

				double diff_x = xy_[0] - car_x;
				double diff_y = xy_[1] - car_y;

				double init_lx =  diff_x*cos(angle) + diff_y*sin(angle);
				double init_ly = -diff_x*sin(angle) + diff_y*cos(angle);
				//init_sy = smooth_waypoints(init_lx);


				// transform back to global coordinate
				double gx = init_lx*cos(angle)-init_ly*sin(angle)+car_x;
				double gy = init_lx*sin(angle)+init_ly*cos(angle)+car_y;

				*/
				ptsx.push_back(xy_[0]);
				ptsy.push_back(xy_[1]);


  				//next_x_vals.push_back(xy_[0]);
				//next_y_vals.push_back(xy_[1]);

			
			}
			cout<<" rampup size: "<< rampup[1].size()<<endl;
			

                    } else {
			// from reference point, typical 30 points away from car_x and car_y
                        ref_x = previous_path_x[path_size-1];
                        ref_y = previous_path_y[path_size-1];
                        double ref_x_prev = previous_path_x[path_size-2];
                        double ref_y_prev = previous_path_y[path_size-2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        //ptsx.push_back(ref_x_prev);
                        //ptsx.push_back(ref_x);
                        //ptsy.push_back(ref_y_prev);
                        //ptsy.push_back(ref_y);

			ptsx.clear();
			ptsy.clear();
                    	// Keep previously generated points
                    	for(int i = 0; i < path_size; i++) {
                        	//next_x_vals.push_back(previous_path_x[i]);
                        	//next_y_vals.push_back(previous_path_y[i]);
				ptsx.push_back(previous_path_x[i]);
				ptsy.push_back(previous_path_y[i]);
                    	}
			

			//for(int i = 0; i < next_x_vals.size(); i++){
			//	cout <<next_x_vals[i] <<", "<< next_y_vals[i]<<endl;
			//}
			
			



			// for a rolling car, speed at reference point is, 
			vector<vector<double>> check_dots = sdc.getDOTs(ptsx, ptsy, car_x, car_y, car_yaw, car_speed);

			double ref_s = check_dots[0][path_size-1];
			double ref_s_dot = check_dots[1][path_size-1];

			//cout<<"car_s: "<< car_s <<" ref_s: "<< ref_s <<"ref_s_dot: " << ref_s_dot<< endl;

			
			s_ = car_s + check_dots[0][path_size-1];
			s_dot = check_dots[1][path_size-1];
			s_ddot = 0;//check_dots[2][path_size];

			d_ = car_d;
			d_dot = 0;
			d_ddot = 0;
			
			//vector<double> check_xy = getXY(s_, d_, s_x, s_y, s_dx, s_dy); 
			//cout<< "check xy: "<< check_xy[0]<<", "<< check_xy[1]<< endl;
			s_start.push_back(s_);
			s_start.push_back(s_dot);
			s_start.push_back(s_ddot);


			s_end.push_back(s_+target_s_diff);
			s_end.push_back(s_dot-target_speed-0.3);  // negative gain
			//s_end.push_back(SPEED_LIMIT);  
			s_end.push_back(s_ddot+target_accel);

			d_start.push_back(d_);
			d_start.push_back(d_dot);
			d_start.push_back(d_ddot);


			d_end.push_back(d_+target_lane);			
			//d_end.push_back(2+4*lane);
			d_end.push_back(d_dot);

			d_end.push_back(d_ddot);

		    	// test out all trajectors
		    	double lowest_cost=1e6, cost;
		    	vector<double> s_target, d_target;
		    	vector<vector<double>> jmt_s, jmt_d;
		    	vector<double> best_jmt_s, best_jmt_d;
		    	vector<vector<double>> jmt_pool, best_jmt;
			

		    
		    	for(int i = 0; i<states_pool.size(); i++){
			    //cout<<"Traj num "<<i<<": time_diff cost ";
			    
			    s_target.push_back(states_pool[i][0]);
			    //s_target.push_back(0);
			    //s_target.push_back(0);

			    s_target.push_back(states_pool[i][1]);
			    s_target.push_back(states_pool[i][2]);
			    d_target.push_back(states_pool[i][3]);
			    //d_target.push_back(0);			    				    //d_target.push_back(0);

			    d_target.push_back(states_pool[i][4]);
			    d_target.push_back(states_pool[i][5]); 


			    jmt_s = sdc.JMT(s_start, s_target, states_pool[i][7]);
			    jmt_d = sdc.JMT(d_start, d_target, states_pool[i][7]);
			    vector<double> s_coeffs = sdc.JMT_coeffs(s_start, s_target, states_pool[i][7]);
		    	    //Long_Safety_Cost(lane, ref_start_time, s_start, s_coeffs, predictions);


		  	    //cout<<states_pool[i][j]<<", ";
			    //cost = time_diff_cost(states_pool[i][6], states_pool[i][7]);
			    //cost += open_speed_cost(states_pool[i][1]); //
			    //cost += avg_accel_cost(jmt_s[0]);
			    cost = 10000*Long_Safety_Cost(lane, ref_start_time, s_start, s_coeffs, predictions);


			    if( cost < lowest_cost){
				best_traj_idx = i;
				lowest_cost = cost;
			    }
			    
			    //cout<<cost<<";"<<endl;
		        }
		        cout<<"lowest cost "<<lowest_cost<<" index "<<best_traj_idx<<endl;
			

			
		    	for(int i = 0; i< states_pool[best_traj_idx].size(); i++){
			    cout<< states_pool[best_traj_idx][i]<<",";
			   
		        }
		        cout<<" keep lane speed "<< states_pool[best_traj_idx][1]*2.24<<endl;

			/*best_s.push_back(states_pool[best_traj_idx][0]);
			best_s.push_back(states_pool[best_traj_idx][1]);
			best_s.push_back(states_pool[best_traj_idx][2]);
			best_d.push_back(states_pool[best_traj_idx][3]);
			best_d.push_back(states_pool[best_traj_idx][4]);
			best_d.push_back(states_pool[best_traj_idx][5]);*/

			best_s.push_back(states_pool[best_traj_idx][0]);
			best_s.push_back(states_pool[best_traj_idx][1]);
			best_s.push_back(0);
			best_d.push_back(states_pool[best_traj_idx][3]);
			best_d.push_back(0);
			best_d.push_back(0);


			best_t = states_pool[best_traj_idx][7];
		        /*
			for(int i = 0; i< best_jmt_s.size(); i++){
			    cout<< best_jmt_d[i]<<",";
		        }
		        cout<<endl; 		
			*/


			vector<double> chk_csx, chk_csy;
						
			// generate constant speed path
			cs_s = sdc.JMT(s_start, s_end, ref_duration);
			cs_d = sdc.JMT(d_start, d_end, ref_duration);
			//cs_s = sdc.JMT(s_start, best_s, best_t);
			//cs_d = sdc.JMT(d_start, best_d, best_t);
			//cs = sdc.JMT(s_start, s_end, ref_duration);
			next_d = 2+ 4*lane;
			//cout<<"next_d: "<<next_d<<" next_s: ";
			//cout<<"best_t: "<<best_t<<" ref_duration: "<< ref_duration<< endl;
			//int min_s = min(cs[1].size(), cs_s[1].size());
			for(int i = 0; i<cs_s[1].size(); i++){
				next_s = cs_s[1][i];
				//next_d = cs_d[1][i];
				//next_s +=0.45;
			
			
			//for(int i = 0; i< best_jmt_s.size(); i++){
				//next_s = best_jmt_s[i];
				//next_d = best_jmt_d[i];
			
				// get waypoints in global coordinate  
				//cs_xy_ = getXY(next_s, next_d, s_x, s_y, s_dx, s_dy); //can't warp around
				Vector2d cs_xy_ = getXY(next_s, next_d, s_x, s_y); //not that smooth at end

				/*
				chk_csx.push_back(cs_xy_[0]);
				chk_csy.push_back(cs_xy_[1]);

				// get waypoints in car coordinate
				double cs_diff_x = cs_xy_[0] - ref_x;
				double cs_diff_y = cs_xy_[1] - ref_y;

				double cs_lx =  cs_diff_x*cos(ref_yaw) + cs_diff_y*sin(ref_yaw);
				double cs_ly = -cs_diff_x*sin(ref_yaw) + cs_diff_y*cos(ref_yaw);
				//cs_sy = smooth_waypoints(cs_lx);

				// transform back to global coordinate
				double gx = cs_lx*cos(ref_yaw)-cs_ly*sin(ref_yaw)+ref_x;
				double gy = cs_lx*sin(ref_yaw)+cs_ly*cos(ref_yaw)+ref_y;

				*/
				ptsx.push_back(cs_xy_[0]);
				ptsy.push_back(cs_xy_[1]);
 			
				//next_x_vals.push_back(cs_xy_[0]);
				//next_y_vals.push_back(cs_xy_[1]);
 	
				cout<<next_s<<",";
			
			}
			cout<<" constant speed size: "<< cs_s[1].size()<<endl;
			
		    }

		
		    cout<<"s_start: ";
		    for (int i = 0; i< s_start.size(); i++){
			cout<<s_start[i]<<", "<<ref_duration<<" , ";
		    }
		    cout<<endl;
		    cout<<"s_end: ";
		    for (int i = 0; i< s_end.size(); i++){
			cout<<s_end[i]<<", "<<ref_duration<<" , ";
		    }
		    cout<<endl;
		    cout<<"d_start: ";
		    for (int i = 0; i< d_start.size(); i++){
			cout<<d_start[i]<<", "<<ref_duration<<" , ";
		    }
		    cout<<endl;
		    cout<<"d_end: ";
		    for (int i = 0; i< d_end.size(); i++){
			cout<<d_end[i]<<", "<<ref_duration<<" , ";
		    }
		    cout<<endl;
		    cout<<"best_s_end: ";
		    for (int i = 0; i< best_s.size(); i++){
			cout<<best_s[i]<<", "<<best_t<<" , ";
		    }
		    cout<<endl;
		    cout<<"best_d_end: ";
		    for (int i = 0; i< best_d.size(); i++){
			cout<<best_d[i]<<", "<<best_t<<" , ";
		    }
		    cout<<endl;

		    // Regenerate the path again
		    
		    vector<double> m_ptsx, m_ptsy, m_ptss;
    		    vector<double> f_ptsx, f_ptsy, f_ptss;
		    double prev_s = s_ - s_dot*DELTA_T;
		    double m_yaw;
		    double first_x, before_first_x, first_y, before_first_y, diff_my, diff_mx;

		    if (path_size >= 2) {
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
		    } else {
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
		    }

		    // last two points of coarse trajectory, use target_d and current s + 30,60
		    double target_s1 = s_ + 30;
		    double target_d1 = states_pool[best_traj_idx][3];
		    vector<double> target_xy1 = getXY(target_s1, target_d1, s_x, s_y, s_dx, s_dy);
		    //vector<double> target_xy1 = getXY(target_s1, target_d1, interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y);
		    double target_x1 = target_xy1[0];
		    double target_y1 = target_xy1[1];
		    m_ptss.push_back(target_s1);
		    m_ptsx.push_back(target_x1);
		    m_ptsy.push_back(target_y1);


		    double target_s2 = target_s1 + 30;
		    double target_d2 = target_d1;
		    vector<double> target_xy2 = getXY(target_s2, target_d2, s_x, s_y, s_dx, s_dy);
		    //vector<double> target_xy2 = getXY(target_s2, target_d2, interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y);
		
		    double target_x2 = target_xy2[0];
		    double target_y2 = target_xy2[1];
		    vector<double> mc_ptsx, mc_ptsy;
		    m_ptss.push_back(target_s2);
		    m_ptsx.push_back(target_x2);
		    m_ptsy.push_back(target_y2);
		
		    /*cout<<" m points array ";
		    for(int i = 0; i<m_ptss.size(); i++){
			cout<<m_ptss[i]<<", ";
			cout<<m_ptsx[i]<<", ";
			cout<<m_ptsy[i]<<", ";
		    }
		    cout<<endl;*/
		    // convert to car local coordinate system
		    for (int i = 0; i < m_ptsx.size(); ++i){
			double dx = m_ptsx[i]-first_x;
			double dy = m_ptsy[i]-first_y;
			mc_ptsx.push_back(dx*cos(m_yaw) + dy*sin(m_yaw));
			mc_ptsy.push_back(-dx*sin(m_yaw)+ dy*cos(m_yaw));
		    }
		
		    tk::spline s;

		    s.set_points(mc_ptsx, mc_ptsy);

		    for(int i = 0; i < path_size; i++) {
		    	next_x_vals.push_back(previous_path_x[i]);
		    	next_y_vals.push_back(previous_path_y[i]);
		    } 
		



		    double target_v = max(states_pool[best_traj_idx][1], 13.0);
		    double current_s = s_;
		    double current_v = s_dot;
		    double current_a = s_ddot;
		    double target_hx = 30;
		    double target_hy = s(target_hx);
		    double target_hd = sqrt(target_hx*target_hy + target_hy*target_hy);
		    double x_inc = 0;


		    // lane 0 speed limit reduced, 
		    if (target_d1 <4 && target_d1 > 0 ){
			current_v *= 0.95;
		    }

		    for (int i = 0; i< 100-path_size; i++){

			/*
			double N = target_hd/(DELTA_T*target_v);
			double x_point = x_inc + (target_hx/N);
			double y_point = s(x_point);
			
			x_inc = x_point;*/
		   
			
			double v_inc=0;
			double diff_v = target_v - current_v;
			if (fabs(diff_v) >=2 * VELOCITY_INCREMENT_LIMIT){
			  v_inc = (diff_v)/(fabs(diff_v))*VELOCITY_INCREMENT_LIMIT;
			}
			current_v += v_inc;
			current_s += current_v*DELTA_T;
			double x_point = x_inc + current_v*DELTA_T;
			double y_point = s(x_point);
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

			//f_ptsx.push_back(x_point);
			//f_ptsy.push_back(y_point);
			//f_ptss.push_back(current_s);
		    }
		    /*
		    f_ptsx = interpolate_points(m_ptss, m_ptsx, f_ptss);
		    f_ptsy = interpolate_points(m_ptss, m_ptsy, f_ptss);
				

			
		    // add xy points from newly generated path
		    for (int i = 0; i < f_ptss.size(); i++) {
			//if (subpath_size == 0 && i == 0) continue; // maybe skip start position as a path point?
		    	//next_x_vals.push_back(f_ptsx[i]);
		    	//next_y_vals.push_back(f_ptsy[i]);
                    } 
		    cout <<"car_x "<< car_x <<"car_y "<< car_y<<endl;*/

		    counter += 1;

                    json msgJson;
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;
                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    //cout << "Sending message: " << msg << endl;

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
















































































