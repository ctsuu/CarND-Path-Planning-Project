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
    // Splines to support conversion from s,d to x,y.
    // Other direction is also possible but more difficult.
    tk::spline s_x, s_y, s_dx, s_dy;
    s_x.set_points(map_waypoints_s,map_waypoints_x);
    s_y.set_points(map_waypoints_s,map_waypoints_y);
    s_dx.set_points(map_waypoints_s,map_waypoints_dx);
    s_dy.set_points(map_waypoints_s,map_waypoints_dy);

    // Start in lane 1 (0 is left, 1 is middle, 2 is right)
    int lane = 2;
    //double speed_limit = 49.5; // mph
    double ref_vel = POWER*SPEED_LIMIT; // m/s

    h.onMessage([&s_x, &s_y, &s_dx, &s_dy, &lane, &ref_vel, &my_car, &sdc, &obs](
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
	  	    vector<vector<double>> s_jmt, d_jmt, rampup, cs, smooth_gx, smooth_gy;
	    	    vector<double> s_start, s_end, d_start, d_end, xy, xy_, cs_xy_;
		    vector<double> jmt_ptsx, jmt_ptsy;
		    double next_s, next_d;
		    //double target_s_diff= car_speed*ref_duration; // expected distance ahead
		    double target_s_diff= 12; // expected distance ahead
		    double target_speed= -0.1; // expected speed difference
		    double target_accel = 0; // expected accel difference
		    double target_d = 0; // expected lane change
		    double target_t = 3; // expected 2 second target leading time 
		    double target_lane = 0; // expected lane change difference


                    vector<double> next_x_vals;
                    vector<double> next_y_vals;


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
				xy_ = getXY(next_s, next_d, s_x, s_y, s_dx, s_dy);  
			
				// get waypoints in car coordinate

				double diff_x = xy_[0] - car_x;
				double diff_y = xy_[1] - car_y;

				double init_lx =  diff_x*cos(angle) + diff_y*sin(angle);
				double init_ly = -diff_x*sin(angle) + diff_y*cos(angle);
				//init_sy = smooth_waypoints(init_lx);


				// transform back to global coordinate
				double gx = init_lx*cos(angle)-init_ly*sin(angle)+car_x;
				double gy = init_lx*sin(angle)+init_ly*cos(angle)+car_y;


				ptsx.push_back(gx);
				ptsy.push_back(gy);


  				next_x_vals.push_back(gx);
				next_y_vals.push_back(gy);

			
			}
			cout<<" rampup size: "<< rampup[1].size()<<endl;
			

                    } else {
			// from reference point, typical 30 points away from car_x and car_y
                        ref_x = previous_path_x[path_size-1];
                        ref_y = previous_path_y[path_size-1];
                        double ref_x_prev = previous_path_x[path_size-2];
                        double ref_y_prev = previous_path_y[path_size-2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);
                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);


                    	// Keep previously generated points
                    	for(int i = 0; i < path_size; i++) {
                        	next_x_vals.push_back(previous_path_x[i]);
                        	next_y_vals.push_back(previous_path_y[i]);
                    	}


			//for(int i = 0; i < next_x_vals.size(); i++){
			//	cout <<next_x_vals[i] <<", "<< next_y_vals[i]<<endl;
			//}
			
			



			// for a rolling car, speed at reference point is, 
			vector<vector<double>> check_dots = sdc.getDOTs(next_x_vals, next_y_vals, car_x, car_y, car_yaw, car_speed);

			double ref_s = check_dots[0][path_size-1];
			double ref_s_dot = check_dots[1][path_size-1];

			cout<<"car_s: "<< car_s <<"ref_s: "<< ref_s <<"ref_s_dot: " << ref_s_dot<< endl;

			
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
			s_end.push_back(s_dot+target_speed);  
			//s_end.push_back(SPEED_LIMIT);  
			s_end.push_back(s_ddot+target_accel);

			d_start.push_back(d_);
			d_start.push_back(d_dot);
			d_start.push_back(d_ddot);

			d_end.push_back(d_+target_lane);
			d_end.push_back(d_dot);
			d_end.push_back(d_ddot);

			//generate constant speed path 
			cs = sdc.JMT(s_start, s_end, ref_duration);
			next_d = 2+ 4*lane;
			cout<<"next_d: "<<next_d<<" next_s: ";
			vector<double> chk_csx, chk_csy;
			for(int i = 0; i<cs[1].size(); i++){
				next_s = cs[1][i];
			
				// get waypoints in global coordinate  
				cs_xy_ = getXY(next_s, next_d, s_x, s_y, s_dx, s_dy); 
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


				ptsx.push_back(gx);
				ptsy.push_back(gy);
 			
				next_x_vals.push_back(gx);
				next_y_vals.push_back(gy);
 	
				cout<<next_s<<",";
			
			}
			cout<<" constant speed size: "<< cs[1].size()<<endl;

			//vector<vector<double>> check_dots_cs = sdc.getDOTs(chk_csx, chk_csy, ref_x, ref_y, ref_yaw, ref_s_dot);

			
		    }

		
		    cout<<"s_start: ";
		    for (int i = 0; i< 3; i++){
			cout<<s_start[i]<<", ";
		    }
		    cout<<endl;
		    cout<<"s_end: ";
		    for (int i = 0; i< 3; i++){
			cout<<s_end[i]<<", ";
		    }
		    cout<<endl;
		    cout<<"d_start: ";
		    for (int i = 0; i< 3; i++){
			cout<<d_start[i]<<", ";
		    }
		    cout<<endl;
		    cout<<"d_end: ";
		    for (int i = 0; i< 3; i++){
			cout<<d_end[i]<<", ";
		    }
		    cout<<endl;
		    

                    vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), s_x, s_y, s_dx, s_dy);
                    vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), s_x, s_y, s_dx, s_dy);
                    vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), s_x, s_y, s_dx, s_dy);

                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);

                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);

                    //cout << "Original ptsx:" << endl << ptsx << endl;
		    
                    // Convert ptsx/y to car's coordinate system
                    for(int i = 0; i < ptsx.size(); i++) {
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;
                        ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
                        ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
                    }

                    
                    tk::spline traj;
                    traj.set_points(ptsx,ptsy);


                    // Keep previously generated points
                    for(int i = 0; i < path_size; i++) {
                        //next_x_vals.push_back(previous_path_x[i]);
                        //next_y_vals.push_back(previous_path_y[i]);
                    }

                    // Determine distance between points
                    double target_x = 30.0;
                    double target_y = traj(target_x);
                    double target_dist = sqrt(target_x * target_x + target_y * target_y);
                    double x_add_on = 0;

                    for(int i = 1; i <= 50-path_size; i++) {
                        double N = target_dist / (0.02*ref_vel); //  m/s
                        double x_point = x_add_on + target_x / N;
                        double y_point = traj(x_point);
                        x_add_on = x_point;
                        double x_ref = x_point;
                        double y_ref = y_point;

                        // Shift back to map coordinates
                        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw) + ref_x;
                        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw) + ref_y;

                        //next_x_vals.push_back(x_point);
                        //next_y_vals.push_back(y_point);
                    }
		    vector<vector<double>> check_dots = sdc.getDOTs(next_x_vals, next_y_vals, car_x, car_y, car_yaw, car_speed);

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
















































































