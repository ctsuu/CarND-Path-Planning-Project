#ifndef VEHICLE
#define VEHICLE

#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  double s, s_d, s_dd;
  double d, d_d, d_dd;
  double ref_s, ref_d, ref_start_time;
  double safety_distance = 6;
  vector<double> target_dist_and_speed;
  double target_speed, target_distance, target_d, target_duration;
  double timer;
  double LL_speed, LL_clearance, RL_speed, RL_clearance, KL_speed, KL_clearance;
  bool FL_clear, FR_clear, RL_clear, RR_clear;
  int lanes_available, target_lane, current_lane;
  double max_acceleration;
  double goal_s;
  bool isInitialized;
  bool left_lane_open;
  bool right_lane_open;
  bool lane_follow;
  bool prepare_left_lane_change;
  bool prepare_right_lane_change;
  bool left_lane_changing;
  bool right_lane_changing;
  bool too_close;
  map<int, vector<vector<double>>> predictions;

  // available states : "KL", "LCL", "LCR", "PLCL", "PLCR" 
  string state;
  vector<string> available_states;
  vector<double> s_traj_coeffs, d_traj_coeffs; // 6 element array
  vector<double> s_start, s_end, d_start, d_end; // 3 element array
  vector<string> open_lanes;
  

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

  void initialize_state_machine();

  double distance4(double x1, double y1, double x2, double y2);

  int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);

  int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

  vector<double> getFrenetSD(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

  vector<double> getLaneState(vector<vector<double>> sensor_fusion, int my_lane, double car_v, double car_s, double car_d, vector<double> map_waypoints_x, vector<double> map_waypoints_y);

  vector<vector<double>> oneDOT(vector<double> ptsx, vector<double> ptsy);

  void state_machine(int counter, int target_lane, map<int, vector<vector<double>>> predictions);

  vector<vector<double>> getDOTs(vector<double> ptsx, vector<double> ptsy, double ref_x, double ref_y, double ref_yaw, double ref_v);

  void increment(double dt );

  void update_states(double s_, double s_dot, double s_ddot, double d_, double d_dot, double d_ddot, double ref_s, double ref_d, double ref_start_time, string state, int lane, int target_lane, double speed);

  vector<double> state_at(double t);

  vector<vector<double>> JMT(vector< double> start, vector <double> end, double T);

  vector<double> JMT_coeffs(vector< double> start, vector <double> end, double T);

  void getLaneChangeClearance(vector<vector<double>> sensor_fusion, map<int, vector<vector<double>>> predictions);

  int findBestLane(vector<vector<double> > sensor_fusion, double car_s);

  vector<double> checkClearance(vector<vector<double> > sensor_fusion,
                              double car_s, int target_lane);

  vector<vector<double>> perturb_goal(vector<vector<double>> seed_states);

  vector<vector<double>> getPredictions(double traj_start_time);

  int execute(int best_lane, int c_lane, vector<vector<double >> sensor_fusion, int counter);

  string display();

};

#endif
