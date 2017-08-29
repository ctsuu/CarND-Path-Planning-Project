#ifndef ROAD
#define ROAD

#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include "spline.h"
#include "constants.h"
#include <math.h>

using namespace std;



// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }



double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}




/*


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet_org(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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

*/
/*
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

*/



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
vector<double> path_extender(vector<double> pts_x, vector<double> pts_y, double start,
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
/*
// sensor fusion data processing
vector<double> sensor(vector<vector<double>> sensor_fusion, int my_lane, double car_v, double car_s, double car_d, vector<double> map_waypoints_x, vector<double> map_waypoints_y){
	/* Given sensor fusion in format{ sf_id, sf_x, sf_y, sf_vx, sf_vy, sf_s, sf_d}

	calculate the missing sf_s_dot, sf_s_d_dot, sf_d_dot, sf_d_d_dot, 
	sf_lead_time( how far other car ahead or behind my car
	sf_lane ( which lane other cars are in)
 
	Return Target position in next few seconds.
	Format:{s, s_dot, s_d_dot, d, d_dot, d_d_dot, T} for JMT input. 
	

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
		vector<double> sf_frenet = getFrenet_org(sf_xt1, sf_yt1, sf_heading,map_waypoints_x, map_waypoints_y);  
					
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
		cout <<"closest car" << closest_car<< ", "; 
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
			return results;
		}
	}
	return results;
}	
*/
/*
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
*/



#endif
