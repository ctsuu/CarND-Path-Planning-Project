#include "vehicle.h"
#include "constants.h"
#include "utils.h"
#include "costs.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <iterator>
#include <random>
#include <algorithm>
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"
tk::spline s_x, s_y, s_dx, s_dy;
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

    this->state = "KL";   
    // all available states are: "KL", "PLCL", "LCL", "PLCR", "LCR"
    //this->isInitialized = false;
    // use first measurement to update each vehicle
}

Vehicle::~Vehicle() {}

/*
void Vehicle::initialize_state_machine(){
  
    this->left_lane_open = false;
    this->right_lane_open = false;
    this->lane_follow = true;
    this->prepare_left_lane_change = false;
    this->prepare_right_lane_change = false;
    this->left_lane_changing = false;
    this->right_lane_changing = false;
    this->open_lanes.clear(); // all lanes are closed.
    this->target_speed = 0;
    //this->current_lane = 1;
    //this->target_lane = 1;
    //this->isInitialized = true;
  
  return;
}
*/




double Vehicle::distance4(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


int Vehicle::ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++){
	double map_x = maps_x[i];
	double map_y = maps_y[i];

	//cout << "map x: "<< map_x <<";"<<"Map_y : "<< map_y << endl;
	double dist = distance4(x,y,map_x,map_y);
	if(dist < closestLen){
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


        

vector<double> Vehicle::getLaneState(vector<vector<double>> sensor_fusion, int my_lane, double car_v, double car_s, double car_d, vector<double> maps_x, vector<double> maps_y){
    /* Given sensor fusion in format{ sf_id, sf_x, sf_y, sf_vx, sf_vy, sf_s, sf_d}

    Return 
    Format:{s, s_dot, s_d_dot, d, d_dot, d_d_dot, T} for path planner input. 
    */

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
    vector<double> start_state;
    double lead_s; // in meter
    double lead_t; // in second


    vector<double> lane1;
    vector<double> lane2;
    vector<double> lane3;
    vector<vector<double>> lanes;
    lanes.push_back(lane1);
    lanes.push_back(lane2);
    lanes.push_back(lane3);

    vector<double> results;
	
    for (int i = 0; i < sensor_fusion.size(); i++){
	sf_id = sensor_fusion[i][0];
	sf_x = sensor_fusion[i][1];
	sf_y = sensor_fusion[i][2];
	sf_vx = sensor_fusion[i][3];
	sf_vy = sensor_fusion[i][4];
	sf_s = sensor_fusion[i][5];
	sf_d = sensor_fusion[i][6];


	double sf_xt1 = sf_x+(DELTA_T*sf_vx); 
	double sf_yt1 = sf_y+(DELTA_T*sf_vy);
	double sf_heading = atan2(sf_yt1-sf_y,sf_xt1-sf_x); 
	vector<double> sf_frenet = getFrenetSD(sf_xt1, sf_yt1, sf_heading,maps_x, maps_y);  
					
	sf_s_dot = (sf_frenet[0]-sf_s)/DELTA_T;
	sf_d_dot = (sf_frenet[1]-sf_d)/DELTA_T;
		
	start_state.push_back(sf_s);
	start_state.push_back(sf_s_dot);
	start_state.push_back(0);  // assume constant accelection 0
	start_state.push_back(sf_d);
	start_state.push_back(sf_d_dot);
	start_state.push_back(0); // assume constant accelection 0

	lead_s = sf_s - car_s; // in meter
	if(lead_s < -400)
	    lead_s += MAX_S;

	if(lead_s > 400)
	    lead_s -= MAX_S;

         //cout<<"lead_s "<< lead_s<<endl;
	lead_t = lead_s/car_v; // in seconds
	
	for (int i =0; i < 3; i++){
		
	    if (sf_d < (2+4*i+2) && sf_d > (2+4*i-2)) {
		lanes[i].push_back(lead_t);
   	    }
	}
    }

    // find closest car, and open lanes
    this->open_lanes.clear();
    for (int i =0; i < lanes.size(); i++){
	cout << "lane"<<i<<": ";
	double closest_car = 9999;
	for (int j =0; j < lanes[i].size(); j++){
	    if( abs(lanes[i][j])<closest_car){
		closest_car = abs(lanes[i][j]);
	    }
	}
	cout <<"closest car " << closest_car<< ", "; 
	if(closest_car > FOLLOWING_GAP){
	    cout<<"lane open"<<endl;
	    this->open_lanes.push_back("OPEN,");

	    this->lane_follow = true; // only can overwrite by state machine

	    target_d = 2+4*i;
			
	    target_s_dot = min(car_v, SPEED_LIMIT);
	    target_t = min(closest_car, FOLLOWING_GAP);
	    target_s = car_s + target_s_dot *target_t;

	    while (target_s > MAX_S){
		target_s -=MAX_S;
	    }
	    while (target_s <0){
		target_s += MAX_S;
	    }
			
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

	    this->too_close = true;
	    this->open_lanes.push_back("KL,");
	    this->lane_follow = true;
	    this->left_lane_open = false;
	    this->right_lane_open = false;

	    target_d = 2+4*my_lane;
	    target_s_dot = min(sf_s_dot, SPEED_LIMIT); 
	    target_t = closest_car;
	    target_s = car_s + target_s_dot*target_t; 

	    while (target_s > MAX_S){
		target_s -=MAX_S;
	    }
	    while (target_s <0){
		target_s += MAX_S;
	    }

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

 

void Vehicle::increment(double dt ) {

    this->s += this->s_d * dt;
    this->s_d += this->s_dd * dt;
    this->d += this->d_d * dt;
    this->d_d += this->d_dd * dt;
}

void Vehicle::update_states(double s_, double s_dot, double s_ddot, double d_, double d_dot, double d_ddot, double ref_s, double ref_d, double ref_start_time, string state, int lane, int target_lane, double speed) {

    this->s = s_;
    this->s_d = s_dot;
    this->s_dd = s_ddot;
    this->d = d_;
    this->d_d = d_dot;
    this->d_dd = d_ddot;
    this->ref_s = ref_s;
    this->ref_d = ref_d;
    this->ref_start_time = ref_start_time;
    this->state = state;
    this->current_lane = Utils::getLaneNumberForD(d_);
    this->target_lane = lane;
    this->target_speed = speed;
    return;
}


void Vehicle::getLaneChangeClearance(vector<vector<double>> sensor_fusion, map<int, vector<vector<double>>> predictions){
    vector<double> front_left, front_right, clear_left, clear_right;
 
    if (this->current_lane-1 >=0){
      clear_left = checkClearance(sensor_fusion, this->s, this->current_lane -1);
      if(clear_left[0] > FOLLOW_DISTANCE){
    	this->FL_clear = true;
      } else {
	this->FL_clear = false;
      }
      if(clear_left[1] > FOLLOW_DISTANCE+4){
    	this->RL_clear = true;
      } else {
        this->RL_clear = false;
      }
    }

    if (this->current_lane+1 <=2){
      clear_right = checkClearance(sensor_fusion, this->s, this->current_lane +1);
      if(clear_right[0] > FOLLOW_DISTANCE){
    	this->FR_clear = true;
      } else {
    	this->FR_clear = false;
      }    
      if(clear_right[1] > FOLLOW_DISTANCE+4){
    	this->RR_clear = true;
      } else {
	this->RR_clear = false;
      }
    }
    //cout << clear_left[0]<<", "<<clear_right[0] <<", "<<clear_left[1] <<", " <<clear_right[1]<<", "<< endl;
    cout << this->FL_clear<<", "<< this->FR_clear<<",\n"<< this->RL_clear<<", "<< this->RR_clear<<", "<<endl;
    return;
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


// JMT Coeffs only
vector<double> Vehicle::JMT_coeffs(vector< double> start, vector <double> end, double T)
{
    
    MatrixXd A = MatrixXd(3, 3);
    A <<   T*T*T, T*T*T*T, T*T*T*T*T,
	 3*T*T, 4*T*T*T, 5*T*T*T*T,
	 6*T,  12*T*T,  20*T*T*T;
		
    MatrixXd B = MatrixXd(3,1);	    
    B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
	 end[1]-(start[1]+start[2]*T),
	 end[2]-start[2];
			    
    MatrixXd Ai = A.inverse();
	
    MatrixXd C = Ai*B;
    vector <double> coeffs = {start[0], start[1], .5*start[2]};

    for(int i = 0; i < C.size(); i++){
      coeffs.push_back(C.data()[i]);
    }
    return coeffs;
    
}



int Vehicle::findBestLane(vector<vector<double> > sensor_fusion, double car_s) {
  // stay away from lane 2 if possible
  vector<double> lane_cost = {0.000001, 0, 0.00001};
  double dist;
  int lane;
  double speed;

  for (int i = 0; i < (sensor_fusion).size(); i++) {
    dist = (sensor_fusion)[i][5] - car_s;
    lane = Utils::getLaneNumberForD((sensor_fusion)[i][6]);
    speed = sqrt((sensor_fusion)[i][3] * (sensor_fusion)[i][3] +
                 (sensor_fusion)[i][4] * (sensor_fusion)[i][4]);

    if (lane == -1) continue;

    if (dist > 0) {
      lane_cost[lane] += distance_time_cost(dist, speed);
    } else {
      dist = abs(dist);
      if (dist <= FOLLOW_DISTANCE) {
        dist = FOLLOW_DISTANCE;
        lane_cost[lane] += distance_time_cost(dist, speed);
      }
    }
  }

  cout << "lane cost: " << lane_cost[0] << ", " << lane_cost[1] << ", "
       << lane_cost[2] << endl;

  vector<double>::iterator result =
      min_element(begin(lane_cost), end(lane_cost));

  return std::distance(begin(lane_cost), result);
}


vector<double> Vehicle::checkClearance(vector<vector<double> > sensor_fusion,
                              double car_s, int target_lane) {
  double front_min = 999999;
  double rear_min = 99999;
  double check_speed = 2.20;
  int lane;
  double clearance;


  for (int i = 0; i < (sensor_fusion).size(); i++) {
    lane = Utils::getLaneNumberForD((sensor_fusion)[i][6]);
    if (lane == -1) continue;

    // check only cars in specific target lane
    if (target_lane == lane) {
      clearance = (sensor_fusion)[i][5] - car_s;
      if (clearance >= 0) {
        if (front_min > clearance) {
          front_min = clearance;
          check_speed = sqrt((sensor_fusion)[i][3] * (sensor_fusion)[i][3] +
                            (sensor_fusion)[i][4] * (sensor_fusion)[i][4]);
	  check_speed = max(check_speed, 13.0);
        }
      } else {
        clearance = abs(clearance);
        if (rear_min > clearance) {
          rear_min = clearance;
        }
      }
    }
  }
  if (front_min > 50){
    check_speed = SPEED_LIMIT;
  }

  //cout <<"check lane "<<target_lane << " front clearance " << front_min << " front speed " << check_speed*2.24 << " rear approach " << rear_min << endl;   

  return {front_min, rear_min, check_speed};

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
	
	
	double nd_s = abs(ndist_s(gen));// always generate forward path
        while (nd_s > MAX_S){
	    nd_s -= MAX_S;
	}

    	new_goal.push_back(nd_s); 
    	new_goal.push_back(ndist_s_dot(gen)); 
    	new_goal.push_back(ndist_s_ddot(gen)); 
    	new_goal.push_back(ndist_d(gen)); 
    	new_goal.push_back(ndist_d_dot(gen)); 
        new_goal.push_back(ndist_d_ddot(gen));
        new_goal.push_back(seed_states[i][6]); // keep org duration for cost function
	new_goal.push_back(abs(ndist_t(gen))); // always generate future value 
    
	results.push_back(new_goal);
        new_goal.clear();
      }
    }
    return results;
}







vector<vector<double>> Vehicle::getPredictions(double traj_start_time){
    // Generates a list of predicted s and d positions for dummy constant-speed vehicles
    // Because ego car trajectory is considered from end of previous path, we should also consider the trajectories of other cars starting at the reference time.

    vector<vector<double>> predictions;
    vector<double> sf_gcx, sf_gcy, sf_lcx, sf_lcy, sf_s, sf_d, sf_t;
    double cum_t = 0;
    for( int i = 0; i < COST_SAMPLES; i++)
    {
        double t = traj_start_time + (i * COST_DT);
        double pred_s = this->s + this->s_d * t;
	//cout <<"predicted_s "<< pred_s << endl;
	// assuming the car in the same lane, same speed
	sf_s.push_back(pred_s);
	sf_d.push_back(this->d);
	/*
	// transfor to global coordinate
	vector<double> gcxy = getFD_GC(pred_s, this->d, maps_s, maps_x, maps_y);
	sf_gcx.push_back(gcxy[0]);
	sf_gcy.push_back(gcxy[1]);

	// transfor to local coordinate as well
        vector<double> gclc = getGC_LC(car_x, car_y, theta, gcxy[0], gcxy[1]);
	sf_lcx.push_back(gclc[0]);
	sf_lcy.push_back(gclc[1]);
	*/
	// add cumulated time steps as well
	sf_t.push_back(cum_t);
	cum_t += COST_DT;

    }
    //predictions.push_back(sf_gcx);
    //predictions.push_back(sf_gcy);
    //predictions.push_back(sf_lcx);
    //predictions.push_back(sf_lcy);

    predictions.push_back(sf_s);
    predictions.push_back(sf_d);
    predictions.push_back(sf_t);

    return predictions;
}

int Vehicle::execute(int best_lane, int c_lane, vector<vector<double >> sensor_fusion, int counter){
    if( this->lane_follow ){
	this->target_speed -=V_INC;
        // find open lane, update states
	for( int i = 0; i<open_lanes.size(); i++){
 	  string st = open_lanes[i];
	  
    	  if(st.compare("KL,") == 0 && (i==(this->current_lane-1)) )
	    this->left_lane_open = false;
	  if(st.compare("OPEN,") == 0 && (i==(this->current_lane-1)))
	    this->left_lane_open = true;

	  if(st.compare("KL,") == 0 && (i==(this->current_lane+1)) )
	    this->right_lane_open = false;
	  if(st.compare("OPEN,") == 0 && (i==(this->current_lane+1)))
	    this->right_lane_open = true;
	}

	if (abs(c_lane - best_lane) != 2) {
    	  if (this->FL_clear && this->RL_clear && best_lane < c_lane && 
	      this->left_lane_open){
            c_lane = best_lane;
	    this->lane_follow = false;
    	  } else if (this->FR_clear && this->RR_clear && best_lane > c_lane && 
		     this->right_lane_open){
            c_lane = best_lane;
	    this->lane_follow = false;
          }
        } else {
  	  // check middle lane clearance
    	  vector<double> middle_lane = checkClearance(sensor_fusion, this->s, 1);

          if (middle_lane[0] >= FOLLOW_DISTANCE && middle_lane[1] >= FOLLOW_DISTANCE+4) {
	    if(c_lane = 2 && left_lane_open ){
              c_lane = 1;
              this->lane_follow = false;
              cout << "change from right lane to middle lane \n";
	    } else if(c_lane = 0 && right_lane_open ){
	      c_lane = 1;
              this->lane_follow = false;
              cout << "change from left lane to middle lane \n";
	    }
          } else {
            cout << "no double shift \n";
	    this->lane_follow = true;
	    this->left_lane_open = false;
            this->right_lane_open = false;
          }
	}
  }
  return c_lane;
}




string Vehicle::display() {

    ostringstream oss;
	
    oss << "s:       " << this->s     << "\n";
    oss << "s_d:     " << this->s_d   << "\n";
    oss << "s_dd:    " << this->s_dd  << "\n";
    oss << "d:       " << this->d     << "\n";
    oss << "d_d:     " << this->d_d   << "\n";
    oss << "d_dd:    " << this->d_dd  << "\n";
    oss << "ref_s    " << this->ref_s << "\n";
    oss << "state:   " << this->state << "\n";
    //oss << "target_duration: " << this->target_duration << "\n";
    oss << "current lane     " << this->current_lane  << "\n"; 
    oss << "left  lane open? " << this->left_lane_open<< "\n";
    oss << "lane follow?     " << this->lane_follow<< "\n";
    oss << "right lane open? " << this->right_lane_open<< "\n";
    //oss << "target_d         " << this->target_d<< "\n";
    oss << "target_speed     " << this->target_speed*2.24<< "\n";
    oss << "open_lane        " << this->open_lanes[0] << open_lanes[1]<< open_lanes[2]<<"\n";
   

    return oss.str();
}
