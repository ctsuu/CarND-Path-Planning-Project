#include "vehicle.h"
#include "constants.h"
//#include "fusion_planner.h"
//#include "road.h"
//#include "costs.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <iterator>
#include <random>
#include <algorithm>
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


/**
 * Initializes Vehicle
 */
Vehicle::Vehicle() {}

Vehicle::Vehicle(double s, double s_d, double s_dd, double d, double d_d, double d_dd) {

    this->s    = s;             // s position
    this->s_d  = s_d;           // s dot - velocity in s
    this->s_dd = s_dd;          // s dot-dot - acceleration in s
    this->d    = d;             // d position
    this->d_d  = d_d;           // d dot - velocity in d
    this->d_dd = d_dd;          // d dot-dot - acceleration in d
    state = "CS";

}

Vehicle::~Vehicle() {}

double Vehicle::distance4(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


int Vehicle::ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];

		//cout << "map x: "<< map_x <<";"<<"Map_y : "<< map_y << endl;
		double dist = distance4(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}



int Vehicle::NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> Vehicle::getFrenetSD(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
	double frenet_d = distance4(x_x,x_y,proj_x,proj_y);
	//see if d value is positive or negative by comparing it to a center point
	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance4(center_x,center_y,x_x,x_y);
	double centerToRef = distance4(center_x,center_y,proj_x,proj_y);
	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}
	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance4(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}
	frenet_s += distance4(0,0,proj_x,proj_y);
	return {frenet_s,frenet_d};
}


// sensor fusion data processing
vector<double> Vehicle::get_target_state(vector<vector<double>> sensor_fusion, int my_lane, double car_v, double car_s, double car_d, vector<double> map_waypoints_x, vector<double> map_waypoints_y){
	/* Given sensor fusion in format{ sf_id, sf_x, sf_y, sf_vx, sf_vy, sf_s, sf_d}

	calculate the missing sf_s_dot, sf_s_d_dot, sf_d_dot, sf_d_d_dot, 
	sf_lead_time( how far other car ahead or behind my car
	sf_lane ( which lane other cars are in)
 
	Return Target position in next few seconds.
	Format:{s, s_dot, s_d_dot, d, d_dot, d_d_dot, T} for path planner input. 
	*/

	vector<vector<double>>same_lane;
	vector<double> target_time;
	vector<double> dummy_car; 
	int sf_id;
	double sf_x; 
	double sf_y;
	double sf_vx;
	double sf_vy;
	double sf_s;
	double sf_s_dot;
	double sf_s_d_dot = 0; 
	double sf_d;
	double sf_d_dot;
	double sf_d_d_dot = 0;  

	
	double target_s; // car lead s
	double target_s_dot; // car lead s_dot
	double target_s_d_dot; // car lead s_d_dot
	double target_d; // car lead d
	double target_d_dot; // car lead d_dot
	double target_d_d_dot; // car lead d_d_dot
	double target_t = 2.5; // initial lane following gap in sec
	int target_lane = my_lane; // initial lane following
	int target_id; // car id

	double check_car_s; 
	double check_car_d;
	double self_car_s;

	vector<double> start_state;
	
	vector<double> check_car_state_in;

	double lead_s; // in meter
	double lead_t; // in second

	int sf_lane;

	bool left_lane_clear = true;
	bool right_lane_clear = true;
	bool too_close = false;
	bool lane_keep = true;
	bool prepare_left_lane_change = false;
	bool prepare_right_lane_change = false;

	// for the car in my lane, check the following time, unit in second 
	double front_safety_gap = 2.0;   
	double back_safety_gap = 2.0; 
	double following_gap = 2.0; // for curise following 

	std::map<double, double> lane_map0;
	std::map<double, double> lane_map1;
	std::map<double, double> lane_map2;

	vector<double> lane1;
        vector<double> lane2;
        vector<double> lane3;
        vector<vector<double>> lanes;
	lanes.push_back(lane1);
        lanes.push_back(lane2);
        lanes.push_back(lane3);
	vector<double> available_lane;
	vector<double> results;


	// if target car is too far, create a dummy car in close range, such as 3-5 seconds
	
	for (int i = 0; i < sensor_fusion.size(); i++){
		sf_id = sensor_fusion[i][0];
		sf_x = sensor_fusion[i][1];
		sf_y = sensor_fusion[i][2];
		sf_vx = sensor_fusion[i][3];
		sf_vy = sensor_fusion[i][4];
		sf_s = sensor_fusion[i][5];
		sf_d = sensor_fusion[i][6];
		
		double sf_xt1 = sf_x+(0.02*sf_vx); 
		double sf_yt1 = sf_y+(0.02*sf_vy);
		double sf_heading = atan2(sf_yt1-sf_y,sf_xt1-sf_x); 
		vector<double> sf_frenet = getFrenetSD(sf_xt1, sf_yt1, sf_heading,map_waypoints_x, map_waypoints_y);  
					
		sf_s_dot = (sf_frenet[0]-sf_s)/0.02;
		sf_d_dot = (sf_frenet[1]-sf_d)/0.02;
		
		start_state.push_back(sf_s);
		start_state.push_back(sf_s_dot);
		start_state.push_back(0);  // assume constant accelection 0
		start_state.push_back(sf_d);
		start_state.push_back(sf_d_dot);
		start_state.push_back(0); // assume constant accelection 0

		//check_car_state_in = state_in(start_state, 1); // state after 1 sec
		

		// double check_speed = sqrt(sf_vx*sf_vx+sf_vy*sf_vy); // m/s
		//check_car_s = check_car_state_in[0];
		//check_car_d = check_car_state_in[3];
		//self_car_s = sf_s + sf_s_dot*1;	
		
		// lead time from any car, in seconds
					
		// simply projected car_s and car_d 50 steps for 1 second 
		//check_car_s += (50*0.02*sf_s_dot); 
		//check_car_d += (50*0.02*sf_d_dot); 
		//lead_s = check_car_s - self_car_s; // in meter
		lead_s = sf_s - car_s; // in meter
		lead_t = lead_s/car_v; // in seconds
		//sf_lane = int(round(sf_d-2)/4);

		//lanes[sf_lane].push_back(sf_s);
		//lanes[sf_lane].push_back(lead_t);
	
	
	
		for (int i =0; i < 3; i++){
	
		
			if (sf_d < (2+4*i+2) && sf_d > (2+4*i-2)) {
				//lanes[i].push_back(sf_s);
				lanes[i].push_back(lead_t);
			}
		}
	
	}

	// find closest car, and open lanes
	for (int i =0; i < lanes.size(); i++){
		cout << "lane"<<i<<": ";
		double closest_car = 9999;
		for (int j =0; j < lanes[i].size(); j++){
			//cout <<lanes[i][j]<<", ";
			if( abs(lanes[i][j])<closest_car){
				closest_car = abs(lanes[i][j]);
			}
		}
		cout <<"closest car " << closest_car<< ", "; 
		if(closest_car > following_gap){
			cout<<"lane open"<<endl;
			//left_lane_clear = false;
			//available_lane.push_back(i);
			target_d = 2+4*i;
			
			target_s_dot = car_v;
			target_t = closest_car*0.9;
			target_s = car_s + car_v*target_t;
			
			results.push_back(target_s);
			results.push_back(target_s_dot);
			results.push_back(0);
			results.push_back(target_d);
			results.push_back(0); // target_d_dot
			results.push_back(0); // target_d_d_dot
			results.push_back(target_t);
			//return results;
			
		} else{
			cout << "too close, keep lane " << endl;
			too_close = true;
			target_d = 2+4*my_lane;
			target_s_dot = car_v*0.6; //slow down
			target_t = closest_car*0.8;
			target_s = car_s + car_v*target_t-15; 

			
			results.push_back(target_s);
			results.push_back(target_s_dot);
			results.push_back(0);
			results.push_back(target_d);
			results.push_back(0); // target_d_dot
			results.push_back(0); // target_d_d_dot
			results.push_back(target_t);
			//return results;
			
		}
		

	}
	return results;
}	




vector<vector<double>> Vehicle::smoother(vector<double> ptsx, vector<double> ptsy){
    // Given list of points {x0, x1, ...xn} and {y0, y1, ...yn} in global coordinate
    // processing in car coordinate
    // Return lists of smoother interval points  
    vector<vector<double>> get_lines = oneDOT(ptsx, ptsy);
    vector<vector<double>> results;
    vector<double> new_ptx, new_pty, timeline;
    double cum_dist=0, cum_time=0, N;
    // Create the time line
    for(int i = 0; i<get_lines[0].size(); i++){
	timeline.push_back(cum_time);
	cum_time += DELTA_T;
    }

    tk::spline smooth_ptx;
    tk::spline smooth_pty;
    tk::spline smooth_dist;
    tk::spline smooth_yaw;
    tk::spline smooth_cum;
    smooth_ptx.set_points(get_lines[1], get_lines[3]); // ptsx relate to cumulated distance
    smooth_pty.set_points(get_lines[1], get_lines[4]); // ptsy relate to cumulated distance
    smooth_dist.set_points(get_lines[1], get_lines[0]); // inc dist related to cumulated dist
    smooth_yaw.set_points(get_lines[1], get_lines[5]); // cum yaw related to cumulated dist
    smooth_cum.set_points(timeline, get_lines[1]); // cum dist related to timeline

    double total_disc = get_lines[1][get_lines[1].size()];
    //N = total_disc/(0.02*ref_vel); 
    for(int i = 0; i<get_lines[1].size(); i++){
	//double check_accel = (get_lines[2][i]-get_lines[2][i-1])/DELTA_T;
	//double dist_inc = get_lines[2][i-1]*DELTA_T + 0.5*check_accel*DELTA_T*DELTA_T; 
	//double inc_dist = get_lines[0][i-1]; 
	
    	
	
        //double n_ptx = smooth_ptx(cum_dist);
	//double n_pty = smooth_pty(cum_dist);
	double n_ptx = get_lines[3][i];
	double n_pty = get_lines[4][i];

        new_ptx.push_back(n_ptx);
	new_pty.push_back(n_pty);
	/*
	cout<< "smooth path "<< n_ptx <<",  \t" << n_pty <<",  \t"<< inc_dist<<",  \t"<< dist_inc <<",  \t"<<N << endl;
	if (inc_dist < DISTANCE_INCREMENT_LIMIT && check_accel< MAX_ACCEL){
	    cum_dist += inc_dist; 
	}
	else{ 
	    cum_dist += dist_inc;
	}
	*/
	//cum_dist += inc_dist;

    }

    results.push_back(new_ptx);
    results.push_back(new_pty);
    return results;
}



vector<vector<double>> Vehicle::oneDOT(vector<double> ptsx, vector<double> ptsy){
    // Given list of points {x0, x1, ...xn} and {y0, y1, ...yn}
    // Return lists of one dot between two points 
    double cum_dist = 0;
    double prev_dist ;
    double prev_x = ptsx[0];
    double prev_y = ptsy[0];
    double prev_speed;
    double cum_heading = 0;
    double prev_accel =0;
   
    vector<vector<double>> dots;
    vector<double> cum_s, dist, _dot, d_dot, dd_dot, ptx, pty, heading;
    //cout<<"inc dist, \tcum distance, \t speed, \t ptsx,   \t ptsy,    \t cum yaw :"<<endl;
    for (int i = 1; i <ptsx.size(); i++){  
	double diff_x = ptsx[i]-ptsx[i-1];
	double diff_y = ptsy[i]-ptsy[i-1];
    	double check_dist = sqrt(diff_x*diff_x+diff_y*diff_y);
        double check_speed = check_dist/DELTA_T;
	double check_yaw = atan2(diff_y, diff_x);


	
	//cout<<check_dist<< ",   \t"<< cum_dist<<",   \t"<<check_speed*2.24<<",   \t"<<ptsx[i-1]<<",   \t"<<ptsy[i-1]<<",  \t"<< cum_heading<<endl;

	prev_x = ptsx[i-1];
	prev_y = ptsy[i-1];
	cum_dist += check_dist; 
	prev_dist = check_dist;
	prev_speed = check_speed;
	cum_heading += check_yaw;
	

	cum_s.push_back(cum_dist);
        dist.push_back(check_dist);
	_dot.push_back(check_speed);
	ptx.push_back(ptsx[i]);
	pty.push_back(ptsy[i]);
	heading.push_back(cum_heading);
    }
    dots.push_back(dist);
    dots.push_back(cum_s);
    dots.push_back(_dot);
    dots.push_back(ptx);
    dots.push_back(pty);
    dots.push_back(heading);

    return dots;
}

	
// From points to dots
vector<vector<double>>Vehicle::getDOTs(vector<double> ptsx, vector<double> ptsy, double ref_x_, double ref_y_, double ref_yaw_, double ref_v_){
    // Given list of points {x0, x1, ...xn} and {y0, y1, ...yn}, start point location, direction and speed
    // Return lists of speed, acceleration and jert between next two points 
    double cum_dist = 0;
    double prev_dist = ref_v_*DELTA_T;
    double prev_x = ref_x_;
    double prev_y = ref_y_;
    double prev_speed = ref_v_;
    double prev_accel =0;
    double max_speed = 0, max_accel = 0, max_jerk = 0;
    vector<vector<double>> dots;
    vector<double> ref_s, dist, _dot, d_dot, dd_dot;
    //cout<<"check distance, check speed, check accel, check jerk:"<<endl;
    for (int i = 0; i <ptsx.size(); i++){
    	double check_dist = sqrt((prev_x-ptsx[i])*(prev_x-ptsx[i])+(prev_y-ptsy[i])*(prev_y-ptsy[i]));
        double check_speed = check_dist/DELTA_T;
	double check_accel = (check_speed-prev_speed)/DELTA_T;
	double check_jerk = (check_accel-prev_accel)/DELTA_T;

	
	//cout<< cum_dist<<",   \t"<<check_speed*2.24<<",   \t"<<check_accel<<",   \t"<<check_jerk<<endl;

	prev_x = ptsx[i];
	prev_y = ptsy[i];
	cum_dist += check_dist; 
	prev_dist = check_dist;
	prev_speed = check_speed;
	prev_accel = check_accel;

	ref_s.push_back(cum_dist);
        dist.push_back(check_dist);
	_dot.push_back(check_speed);
	d_dot.push_back(check_accel);
	dd_dot.push_back(check_jerk);
    }
    dots.push_back(ref_s);
    dots.push_back(_dot);
    dots.push_back(d_dot);
    dots.push_back(dd_dot);
    dots.push_back(dist);
    return dots;
}

 
/*

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Vehicle::getFDGCXY(double s, double d, tk::spline s_x, tk::spline s_y, tk::spline s_dx, tk::spline s_dy) {
    double path_x = s_x(s);
    double path_y = s_y(s);
    double dx = s_dx(s);
    double dy = s_dy(s);
    double x = path_x + d * dx;
    double y = path_y + d * dy;
    return {x,y};
}
*/



// Transform from Frenet s,d coordinates to global Cartesian x,y
vector<double> Vehicle::getFD_GC(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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
	double perp_heading = heading-M_PI/2;
	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);
	return {x,y};
}

// Transform from global Cartesian x,y to local car coordinates x,y
// where x is pointing to the positive x axis and y is deviation from the car's path
vector<double> Vehicle::getGC_LC(double car_x, double car_y, double theta, double gc_x, double gc_y) {
  vector<double> results;

  // convert to local coordinates
  float diff_x = (gc_x - car_x);
  float diff_y = (gc_y - car_y);
  results.push_back( diff_x*cos(theta) + diff_y*sin(theta));
  results.push_back(-diff_x*sin(theta) + diff_y*cos(theta));
  return results;
}


// Transform from local Cartesian x,y to global car coordinates x,y
vector<double> Vehicle::getLC_GC(double car_x, double car_y, double theta, double lc_x, double lc_y) {
  vector<double> results;

  // convert back to global coordinates
  results.push_back(lc_x*cos(theta) - lc_y*sin(theta) + car_x);
  results.push_back(lc_x*sin(theta) + lc_y*cos(theta) + car_y);
  return results;
}



vector<double> Vehicle::get_traj_coeffs(vector<double> start, vector<double> end, double T)
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
    */

    MatrixXd a(3,3);
    double T2 =  T*T, 
           T3 = T2*T, 
           T4 = T3*T,
           T5 = T4*T;
    a <<  T3,    T4,    T5, 
        3*T2,  4*T3,  5*T4, 
         6*T, 12*T2, 20*T3;
    MatrixXd aInv = a.inverse();
    
    VectorXd b(3);
    b << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T2),
         end[1] - (           start[1]   +     start[2]*T),
         end[2] - (                            start[2]);
    VectorXd alpha = aInv * b;
    
    vector<double> output = {start[0], start[1], 0.5*start[2], alpha[0], alpha[1], alpha[2]};
    return output;
}


void Vehicle::increment(double dt ) {

    this->s += this->s_d * dt;
    this->s_d += this->s_dd * dt;
    this->d += this->d_d * dt;
    this->d_d += this->d_dd * dt;
}

vector<double> Vehicle::state_at(double t) {

    /*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
    double s = this->s + this->s_d * t + this->s_dd* t * t / 2;
    double s_d = this->s_d + this->s_dd * t;
    double s_dd = this->s_dd;
    double d = this->d + this->d_d * t + this->d_dd* t * t / 2;
    double d_d = this->d_d + this->d_dd * t;
    double d_dd = this->d_dd;
    
    return {s, s_d, s_dd, d, d_d, d_dd};
}
/*
void Vehicle::realize_state(map<int,vector < vector<double> > > predictions) {
   
    //Given a state, realize it by adjusting acceleration and lane.
    //Note - lane changes happen instantaneously.
    
    string state = this->state;
    if(state.compare("CS") == 0)
    {
    	realize_constant_speed();
    }
    else if(state.compare("KL") == 0)
    {
    	realize_keep_lane(predictions);
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change(predictions, "L");
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change(predictions, "R");
    }
    else if(state.compare("PLCL") == 0)
    {
    	realize_prep_lane_change(predictions, "L");
    }
    else if(state.compare("PLCR") == 0)
    {
    	realize_prep_lane_change(predictions, "R");
    }

}

void Vehicle::realize_constant_speed() {
    this->s_d = 0;
}

double Vehicle::_max_accel_for_lane(map<int,vector<vector<double> > > predictions, int lane, double s) {

  double delta_v_til_target = target_speed - this->s_d;
  double max_acc = min(MAX_ACCEL, delta_v_til_target);

    map<int, vector<vector<double> > >::iterator it = predictions.begin();
    vector<vector<vector<double> > > in_front;
    while(it != predictions.end())
    {
       
    	int v_id = it->first;
    	
        vector<vector<double> > sf_out = it->second;
        
        if((sf_out[0][6] == (lane*4+2)) && (sf_out[0][5] > s+2))
        {
        	in_front.push_back(sf_out);
		cout<<"Car in front of me: "<<v_id<<endl;

        }
        it++;
    }
    
    if(in_front.size() > 0)
    {
    	int min_s = 1000;
    	vector<vector<double>> leading = {};
    	for(int i = 0; i < in_front.size(); i++)
    	{
    		if((in_front[i][0][1]-s) < min_s)
    		{
    			min_s = (in_front[i][0][1]-s);
    			leading = in_front[i];
    		}
    	}
    	
    	int next_pos = leading[1][1];
    	int my_next = s + this->s_d*DELTA_T;
    	int separation_next = next_pos - my_next;
    	int available_room = separation_next - preferred_buffer;
    	max_acc = min(MAX_ACCEL, available_room);
    }
    
    return max_acc;

}
*/


// JMT Trajectory Generation
vector<vector<double>> Vehicle::JMT(vector< double> start, vector <double> end, double T)
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


vector<vector<double>> Vehicle::get_best_frenet_trajectory(map<int, vector<vector<double>>> predictions, double duration) {
    
    // NOTE: THIS METHOD IS FROM AN ALTERNATE IMPLEMENTATION AND NO LONGER USED
    bool car_to_left = false, car_to_right = false, car_just_ahead = false;
    update_available_states(car_to_left, car_to_right);

    // // DEBUG
    // cout << "available states: "; 
    // for (auto st: available_states) cout << st << " ";
    // cout << endl; 

    vector<vector<double>> best_frenet_traj, best_target;
    double best_cost = 999999;
    string best_traj_state = "";
/*
    for (string state : available_states) {

        // target state (s and d) and time
        vector<vector<double>> target_s_and_d = get_target_for_state(state, predictions);
        double target_time = duration;

        // DEBUG
        cout << "target s&d for state " << state << ": ";
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 3; j++) {
                cout << target_s_and_d[i][j];
                if (j != 2) cout << ", ";
            }
            cout << "; ";
        }
        cout << endl;

        // Perturb trajectories, first by duration
        for (int i = -NUM_TIMESTEPS_TO_PERTURB; i <= NUM_TIMESTEPS_TO_PERTURB; i++) {
            double perturbed_time = target_time + (i * SIGMA_T);

            // Perturb by sigma s and d values
            for (int i = 0; i < NUM_RANDOM_TRAJ_TO_GEN; i++) {
                vector<vector<double>> perturbed_target = perturb(target_s_and_d);

                 // DEBUG
                cout << "perturbed target s&d for time " << perturbed_time << ": ";
                for (int i = 0; i < 2; i++) {
                    for (int j = 0; j < 3; j++) {
                        cout << perturbed_target[i][j];
                        if (j != 2) cout << ", ";
                    }
                    cout << "; ";
                }
                cout << endl;

                vector<vector<double>> possible_traj = generate_traj_for_target(perturbed_target, perturbed_time);

                // // DEBUG
                // cout << "possible_traj: ";
				// for (int i = 0; i < N_SAMPLES; i += N_SAMPLES/3-1) {
                //     cout << "(" << possible_traj[0][i] <<  "," << possible_traj[1][i] << ") ";
                // }
                // cout << endl;

                double current_cost = calculate_total_cost(possible_traj[0], possible_traj[1], predictions, target_s_and_d[0], target_s_and_d[1], target_time, perturbed_time);

                // // DEBUG
                // cout << "cost: " << current_cost << endl;

                if (current_cost < best_cost) {
                    best_cost = current_cost;
                    best_frenet_traj = possible_traj;
                    best_traj_state = state;
                    best_target = perturbed_target;
                }
            }
        }
    } 
    
    // have to call generate_traj_for_target to reset this->s_coeffs etc.
    generate_traj_for_target(best_target, duration);
*/
    // DEBUG - ONLY KEEP LANE AND NO PERTURB
    state = "KL";
    best_target = get_target_for_state(state, predictions, duration, car_just_ahead);
    best_frenet_traj = generate_traj_for_target(best_target, duration);

    // // DEBUG
    // cout << "chosen state: " << best_traj_state << ", cost: " << best_cost << ", ";
    // cout << "target (s,sd,sdd - d,dd,ddd): (";
    // for (int i = 0; i < 2; i++) {
    //     for (int j = 0; j < 3; j++) {
    //         cout << best_target[i][j];
    //         if (j != 2) cout << ", ";
    //     }
    //     cout << "; ";
    // }
    // cout << ")" << endl;

    // // DEBUG
    // cout << "best frenet trajectory (s,d):" << endl;
    // for (int i = 0; i < best_frenet_traj[0].size(); i++) {
    //     cout << best_frenet_traj[0][i] << ", " << best_frenet_traj[1][i] << endl;
    // }
    // cout << endl << endl;

    return best_frenet_traj;
}

void Vehicle::update_available_states(bool car_to_left, bool car_to_right) {
	/*  Updates the available "states" based on the current state:
    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.
    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane. */

    this->available_states = {"KL"};
    if (this->d > 4 && !car_to_left) {
        this->available_states.push_back("LCL");
    }
    if (this->d < 8 && !car_to_right) {
        this->available_states.push_back("LCR");
    }
}

vector<vector<double>> Vehicle::get_target_for_state(string state, map<int, vector<vector<double>>> predictions, double duration, bool car_just_ahead) {
    // Returns two lists s_target and d_target in a single vector - s_target includes 
    // [s, s_dot, and s_ddot] and d_target includes the same
    // If no leading car found target lane, ego car will make up PERCENT_V_DIFF_TO_MAKE_UP of the difference
    // between current velocity and target velocity. If leading car is found set target s to FOLLOW_DISTANCE
    // and target s_dot to leading car's s_dot based on predictions
    int target_lane, current_lane = this->d / 4; 
    double target_d; 
    // **** TARGETS ****
    // lateral displacement : depends on state
    // lateral velocity : 0
    double target_d_d = 0;
    // lateral acceleration : 0
    double target_d_dd = 0;
    // longitudinal velocity : current velocity + max allowed accel * duration
    double target_s_d = min(this->s_d + MAX_INSTANTANEOUS_ACCEL/4 * duration, SPEED_LIMIT);
    target_s_d = SPEED_LIMIT;    
    // longitudinal acceleration : zero ?
    double target_s_dd = 0;
    // longitudinal acceleration : difference between current/target velocity over trajectory duration?
    //double target_s_dd = (target_s_d - this->s_d) / (N_SAMPLES * DT);
    // longitudinal displacement : current displacement plus difference in current/target velocity times 
    // trajectory duration
    double target_s = this->s + (this->s_d + target_s_d) / 2 * duration;

    vector<double> leading_vehicle_s_and_sdot;

    if(state.compare("KL") == 0)
    {
        target_d = (double)current_lane * 4 + 2;
        target_lane = target_d / 4;
    }
    else if(state.compare("LCL") == 0)
    {
        target_d = ((double)current_lane - 1) * 4 + 2;
        target_lane = target_d / 4;
    }
    else if(state.compare("LCR") == 0)
    {
        target_d = ((double)current_lane + 1) * 4 + 2;
        target_lane = target_d / 4;
    }
    
    // replace target_s variables if there is a leading vehicle close enough
    leading_vehicle_s_and_sdot = get_leading_vehicle_data_for_lane(target_lane, predictions, duration);
    double leading_vehicle_s = leading_vehicle_s_and_sdot[0];
    if (leading_vehicle_s - target_s < FOLLOW_DISTANCE && leading_vehicle_s > this->s) {

        target_s_d = leading_vehicle_s_and_sdot[1];

        if (fabs(leading_vehicle_s - target_s) < 0.5 * FOLLOW_DISTANCE) {
            //cout << "TOO CLOSE IN LANE " << target_lane << "!! current target speed: " << target_s_d;
            target_s_d -= 1; // slow down if too close
            //cout << "  new target speed: " << target_s_d << endl;
        }

        target_s = leading_vehicle_s - FOLLOW_DISTANCE;
        // target acceleration = difference between start/end velocities over time duration? or just zero?
        //target_s_dd = (target_s_d - this->s_d) / (N_SAMPLES * DT);

        // // DEBUG
        // cout << "NEARBY LEAD VEHICLE DETECTED!  ";
        // cout << "s: " << leading_vehicle_s_and_sdot[0]
        //      << ", lane: " << target_lane 
        //      << ", speed: " << leading_vehicle_s_and_sdot[1] << endl;
    }

    // emergency brake
    if (car_just_ahead) {
        target_s_d = 0.0;
    }

    return {{target_s, target_s_d, target_s_dd}, {target_d, target_d_d, target_d_dd}};
}

vector<double> Vehicle::get_leading_vehicle_data_for_lane(int target_lane, map<int, vector<vector<double>>> predictions, double duration) {
    // returns s and s_dot for the nearest (ahead) vehicle in target lane
    // this assumes the dummy vehicle will keep its lane and velocity, it will return the end position
    // and velocity (based on difference between last two positions)
    double nearest_leading_vehicle_speed = 0, nearest_leading_vehicle_distance = 99999;
    for (auto prediction : predictions) {
        vector<vector<double>> pred_traj = prediction.second;
        int pred_lane = pred_traj[0][1] / 4;
        if (pred_lane == target_lane) {
            double start_s = pred_traj[0][0];
            double predicted_end_s = pred_traj[pred_traj.size()-1][0];
            double next_to_last_s = pred_traj[pred_traj.size()-2][0];
            double dt = duration / TRAJ_SAMPLES;
            double predicted_s_dot = (predicted_end_s - next_to_last_s) / dt;
            if (predicted_end_s < nearest_leading_vehicle_distance && start_s > this->s) {
                nearest_leading_vehicle_distance = predicted_end_s;
                nearest_leading_vehicle_speed = predicted_s_dot;
            }
        }
    }
    return {nearest_leading_vehicle_distance, nearest_leading_vehicle_speed};
}

vector<vector<double>> Vehicle::perturb(vector<vector<double>> target_s_and_d) {
    // randomly perturb the target of the trajectory 
    double perturbed_s, perturbed_s_dot, perturbed_s_ddot,
           perturbed_d, perturbed_d_dot, perturbed_d_ddot;
    // pull out the individual targets
    vector<double> target_s_vars = target_s_and_d[0];
    vector<double> target_d_vars = target_s_and_d[1];
    double target_s = target_s_vars[0];
    double target_s_dot = target_s_vars[1];
    double target_s_ddot = target_s_vars[2];
    double target_d = target_d_vars[0];
    double target_d_dot = target_d_vars[1];
    double target_d_ddot = target_d_vars[2];

    random_device rd;
    mt19937 e2(rd());
    normal_distribution<> nd1(target_s, SIGMA_S);
    perturbed_s = nd1(e2);
    normal_distribution<> nd2(target_s_dot, SIGMA_S_DOT);
    perturbed_s_dot = nd2(e2);
    normal_distribution<> nd3(target_s_ddot, SIGMA_S_DDOT);
    perturbed_s_ddot = nd3(e2);
    normal_distribution<> nd4(target_d, SIGMA_D);
    perturbed_d = nd4(e2);
    normal_distribution<> nd5(target_d_dot, SIGMA_D_DOT);
    perturbed_d_dot = nd5(e2);
    normal_distribution<> nd6(target_d_ddot, SIGMA_D_DDOT);
    perturbed_d_ddot = nd6(e2);

    return {{perturbed_s, perturbed_s_dot, perturbed_s_ddot},
            {perturbed_d, perturbed_d_dot, perturbed_d_ddot}};
}



vector<vector<double>> Vehicle::perturb_goal(vector<vector<double>> seed_states){
    // Given seed states, num_samples, and Sigma's
    // Produce perturbed goals for cost function
    vector<double> new_goal;
    vector<vector<double>> results;
    std::default_random_engine gen;
    for(int n=0; n<TRAJ_SAMPLES; n++) {	
      for(int i =0; i<seed_states.size(); i++){
    	// Uncertainty generators for s, d, t around seed state mean
    	std::normal_distribution<double> ndist_s(seed_states[i][0], SIGMA_S); 
    	std::normal_distribution<double> ndist_s_dot(seed_states[i][1], SIGMA_S_DOT); 
    	std::normal_distribution<double> ndist_s_ddot(seed_states[i][2], SIGMA_S_DDOT); 
	
    	std::normal_distribution<double> ndist_d(seed_states[i][3], SIGMA_D); 
    	std::normal_distribution<double> ndist_d_dot(seed_states[i][4], SIGMA_D_DOT); 
    	std::normal_distribution<double> ndist_d_ddot(seed_states[i][5], SIGMA_D_DDOT); 

    	std::normal_distribution<double> ndist_t(seed_states[i][6], SIGMA_T); 
	
  	
    	new_goal.push_back(abs(ndist_s(gen))); // always generate forward path
    	new_goal.push_back(ndist_s_dot(gen)); 
    	new_goal.push_back(ndist_s_ddot(gen)); 
    	new_goal.push_back(ndist_d(gen)); 
    	new_goal.push_back(ndist_d_dot(gen)); 
        new_goal.push_back(ndist_d_ddot(gen));
	new_goal.push_back(abs(ndist_t(gen))); // always generate future value 
    
	results.push_back(new_goal);
        new_goal.clear();
      }
    }
    return results;
}


vector<vector<double>> Vehicle::generate_traj_for_target(vector<vector<double>> target, double duration) {
    // takes a target {{s, s_dot, s_ddot}, {d, d_dot, d_ddot}} and returns a Jerk-Minimized Trajectory
    // (JMT) connecting current state (s and d) to target state in a list of s points and a list of d points
    // ex. {{s1, s2, ... , sn}, {d1, d2, ... , dn}}
    vector<double> target_s = target[0];
    vector<double> target_d = target[1];
    vector<double> current_s = {this->s, this->s_d, this->s_dd};
    vector<double> current_d = {this->d, this->d_d, this->d_dd};

    // determine coefficients of optimal JMT 
    this->s_traj_coeffs = get_traj_coeffs(current_s, target_s, duration);
    this->d_traj_coeffs = get_traj_coeffs(current_d, target_d, duration);

    // // DEBUG
    // cout << "s coeffs: ";
    // for (auto s : this->s_traj_coeffs) cout << s << ",";
    // cout << endl;
    // cout << "d coeffs: ";
    // for (auto d : this->d_traj_coeffs) cout << d << ",";
    // cout << endl << endl;

    vector<double> s_traj;
    vector<double> d_traj;

    // populate s and t trajectories at each time step
    for (int i = 0; i < TRAJ_SAMPLES; i++) {
        double t = i * duration/TRAJ_SAMPLES;
        double s_val = 0, d_val = 0;
        for (int j = 0; j < s_traj_coeffs.size(); j++) {
            s_val += this->s_traj_coeffs[j] * pow(t, j);
            d_val += this->d_traj_coeffs[j] * pow(t, j);
        }
        s_traj.push_back(s_val);
        d_traj.push_back(d_val);
    }

    return {s_traj, d_traj};
}

vector<double> Vehicle::differentiate_coeffs(vector<double> coeffs) {
    vector<double> diff_coeffs;
    for (int i = 1; i < coeffs.size(); i++) {
        diff_coeffs.push_back(i * coeffs[i]);
    }
    return diff_coeffs;
}

double Vehicle::evaluate_coeffs_at_time(vector<double> coeffs, double time) {
    double eval = 0;
    for (int i = 0; i < coeffs.size(); i++) {
        eval += coeffs[i] * pow(time, i);
    }
    return eval;
}

vector<vector<double>> Vehicle::generate_predictions(double traj_start_time, double duration, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y, double car_x, double car_y, double theta) {

    // Generates a list of predicted s and d positions for dummy constant-speed vehicles
    // Because ego car trajectory is considered from end of previous path, we should also consider the 
    // trajectories of other cars starting at that time.

    vector<vector<double>> predictions;
    vector<double> sf_gcx, sf_gcy, sf_lcx, sf_lcy, sf_s, sf_d;
    
    for( int i = 0; i < TRAJ_SAMPLES; i++)
    {
        //double t = traj_start_time + (i * duration/TRAJ_SAMPLES);
	double t = i*(traj_start_time + duration)/TRAJ_SAMPLES;
        double new_s = this->s + this->s_d * t;
	//vector<double> s_and_d = {new_s, this->d}; 
	// assuming the car in the same lane
	sf_s.push_back(new_s);
	sf_d.push_back(this->d);
	vector<double> gcxy = getFD_GC(new_s, this->d, maps_s, maps_x, maps_y);
	sf_gcx.push_back(gcxy[0]);
	sf_gcy.push_back(gcxy[1]);
        //predictions.push_back(s_and_d);
        vector<double> gclc = getGC_LC(car_x, car_y, theta, gcxy[0], gcxy[1]);
	sf_lcx.push_back(gclc[0]);
	sf_lcy.push_back(gclc[1]);

    }
    predictions.push_back(sf_gcx);
    predictions.push_back(sf_gcy);
    predictions.push_back(sf_lcx);
    predictions.push_back(sf_lcy);

    predictions.push_back(sf_s);
    predictions.push_back(sf_d);
    return predictions;
}

string Vehicle::display() {

    ostringstream oss;
	
    oss << "s:    "    << this->s    << "\n";
    oss << "s_d:    "  << this->s_d  << "\n";
    oss << "s_dd:    " << this->s_dd << "\n";
    oss << "d:    "    << this->d    << "\n";
    oss << "d_d:    "  << this->d_d  << "\n";
    oss << "d_dd:    " << this->d_dd << "\n";
    
    return oss.str();
}
