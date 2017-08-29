#ifndef VEHICLE
#define VEHICLE

#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  double s;
  double s_d;
  double s_dd;
  double d;
  double d_d;
  double d_dd;
  double safety_distance = 6;
  double target_speed;
  int lanes_available, goal_lane;
  double max_acceleration;
  double goal_s;
  string state;
  vector<string> available_states;
  vector<double> s_traj_coeffs, d_traj_coeffs; // 6 element array
  vector<double> s_start, s_end, d_start, d_end; // 3 element array
  

  double T; // duration time to reach target

  /**
  * Constructors
  */
  Vehicle();
  Vehicle(double s, double s_d, double s_dd, double d, double d_d, double d_dd);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  double distance4(double x1, double y1, double x2, double y2);

  int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);

  int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

  vector<double> getFrenetSD(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

  vector<double> get_target_state(vector<vector<double>> sensor_fusion, int my_lane, double car_v, double car_s, double car_d, vector<double> map_waypoints_x, vector<double> map_waypoints_y);

  vector<vector<double>> oneDOT(vector<double> ptsx, vector<double> ptsy);

  vector<vector<double>> smoother(vector<double> ptsx, vector<double> ptsy);

  vector<vector<double>> getDOTs(vector<double> ptsx, vector<double> ptsy, double ref_x, double ref_y, double ref_yaw, double ref_v);

  //vector<double> getFDGCXY(double s, double d, tk::spline s_x, tk::spline s_y, tk::spline s_dx, tk::spline s_dy);

  vector<double> getFD_GC(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);
  
  vector<double> getGC_LC(double car_x, double car_y, double theta, double gc_x, double gc_y);

  vector<double> getLC_GC(double car_x, double car_y, double theta, double lc_x, double lc_y);

  vector<double> get_traj_coeffs(vector<double> start, vector<double> end, double T);

  void increment(double dt );

  vector<double> state_at(double t);

  //void realize_state(map<int,vector < vector<double> > > predictions);

  //void realize_constant_speed();

  //double _max_accel_for_lane(map<int,vector<vector<double> > > predictions, int lane, double s);

  vector<vector<double>> JMT(vector< double> start, vector <double> end, double T);

  vector<vector<double>> get_best_frenet_trajectory(map<int, vector<vector<double>>> predictions, double duration);

  void update_available_states(bool car_to_left, bool car_to_right);

  vector<vector<double>> get_target_for_state(string state, map<int, vector<vector<double>>> predictions, double duration, bool car_just_ahead);

  vector<double> get_leading_vehicle_data_for_lane(int target_lane, map<int, vector<vector<double>>> predictions, double duration);

  vector<vector<double>> perturb(vector<vector<double>> target_s_and_d);

  vector<vector<double>> perturb_goal(vector<vector<double>> seed_states);

  vector<vector<double>> generate_traj_for_target(vector<vector<double>> perturbed_target, double duration);

  vector<double> differentiate_coeffs(vector<double> coeffs); 

  double evaluate_coeffs_at_time(vector<double> coeffs, double time);

  vector<vector<double>> generate_predictions(double traj_start_time, double duration, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y, double car_x, double car_y, double theta);

  string display();

};

#endif
