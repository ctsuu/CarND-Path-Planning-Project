#ifndef COSTS
#define COSTS

#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include <cmath>
#include "constants.h"
#include "spline.h"


using namespace std;

// All function meanly port from Udacity classroom provided python codes.
// I kept near the same naming convension for easy reading.  


double logistic(double x){
  // A function that returns a value between 0 and 1 for x in the range[0, infinity] and - 1 to 1 for x in the range[-infinity, infinity]. Useful for cost functions.
  return 2.0 / (1 + exp(-x)) - 1.0;
}

vector<double> state_in(vector<double> start_state, double t){
  // Project Vehicle move in time t, with constant speed
  // start_state format {s, s_dot, s_d_dot, d, d_dot, d_d_dot}
  vector<double> state;
  state = {
  start_state[0]+(start_state[1]*t)+0.5*start_state[2]*t*t,
  start_state[1]+start_state[2]*t,
  start_state[2],
  start_state[3]+(start_state[4]*t)+0.5*start_state[5]*t*t,
  start_state[4]+start_state[5]*t,
  start_state[5]
  };
  return state; 
}

double eval_poly_equation(vector<double> coeffs, double t){
    // given all coeffs, evaluate the polynormial equation at time f(t)
    /*
    
    def f(t):
        total = 0.0
        for i, c in enumerate(coefficients): 
            total += c * t ** i
        return total
    return f
    */
    double results = 0;
    for(int i = 0; i < coeffs.size(); i++){
	results += coeffs[i]*pow(t, i);
    }
    return results;
}


double nearest_approach(vector<double> s_traj, vector<double> d_traj, vector<vector<double>> prediction) {

  // s_traj, d_traj, are list of points 
  // prediction include estimated s_traj and d_traj for a given vehicle 	
  double closest = 999999;
  for (int i = 0; i < N_SAMPLES; i++) {
    double current_dist = sqrt(pow(s_traj[i] - prediction[i][0], 2) + pow(d_traj[i] - prediction[i][1], 2));
    if (current_dist < closest) {
      closest = current_dist;
    }
  }
  return closest;
}



double cs_nearest_approach(vector<vector<double>> traj, vector<double> car_state){
  // giving testing trajecory s_coeffs, d_coeffs, t combo package
  // any vehicle current state {s, s_dot, s_d_dot, d, d_dot, d_d_dot, and duration T}:
  // output: the closest dist between the testing trajecory and other cars projected path. 

  double closest = 999999;
  vector<double>s_ = traj[0]; // 6 elements
  vector<double>d_ = traj[1]; // 6 elements
  vector<double>t_ = traj[2];
   for(int i = 1; i < 100; i++){
        double t = i / 100 * t_[0];
        double cur_s = eval_poly_equation(s_, t); // s coordinate value at time t
        double cur_d = eval_poly_equation(d_, t); // d coordinate value at time t

	// giving sdc car's s and d states value
	vector<double> target = state_in(car_state, t);
        
        double dist = sqrt(pow((cur_s-target[0]),2) + pow((cur_d-target[3]),2));
        //cout<< dist<<"," ;
        if (dist < closest){
            closest = dist;
        }
    }
    return closest;
}



double nearest_approach_to_any_vehicle(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // Determines the nearest the vehicle comes to any other vehicle throughout a trajectory
  double closest = 999999;
  for (auto prediction : predictions) {
    double current_dist = nearest_approach(s_traj, d_traj, prediction.second);
    if (current_dist < closest) {
      closest = current_dist;
    }
  }
  return closest;
}



double nearest_approach_to_any_vehicle_in_lane(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // Determines the nearest the vehicle comes to any other vehicle throughout a trajectory
  double closest = 999999;
  for (auto prediction : predictions) {
    double my_final_d = d_traj[d_traj.size() - 1];
    int my_lane = my_final_d / 4;
    vector<vector<double>> pred_traj = prediction.second;
    double pred_final_d = pred_traj[pred_traj.size() - 1][1];
    int pred_lane = pred_final_d / 4;
    if (my_lane == pred_lane) {
      double current_dist = nearest_approach(s_traj, d_traj, prediction.second);
      if (current_dist < closest && current_dist < 120) {
        closest = current_dist;
      }
    }
  }
  return closest;
}


vector<double> velocities_for_trajectory(vector<double> traj) {
  // given a trajectory (a vector of positions), return the average velocity between each pair as a vector
  // also can be used to find accelerations from velocities, jerks from accelerations, etc.
  // (i.e. discrete derivatives)
  vector<double> velocities;
  for (int i = 1; i < traj.size(); i++) {
    velocities.push_back((traj[i] - traj[i-1]) / DT);
  }
  return velocities;
}



vector<double> differentiate_t(vector<double> traj) {
  // given a trajectory (a vector of positions), return the average velocity between each pair as a vector
  // also can be used to find accelerations from velocities, jerks from accelerations, etc.
  // (i.e. discrete derivatives)
  vector<double> one_dot;
  for (int i = 1; i < traj.size(); i++) {
    one_dot.push_back((traj[i] - traj[i-1]) / DELTA_T);
  }
  return one_dot;
}

vector<double> differentiate_s(vector<double> coeffs){
    // given all coeffs of a polynomial, calculates the derivative of it and returns the corresponding coefficients.
    /*
    new_cos = []
    for deg, prev_co in enumerate(coefficients[1:]):
        new_cos.append((deg+1) * prev_co)
        #print(new_cos)
    return new_cos
    */
	
    vector<double> s_dot;
    for (int i =1; i< coeffs.size(); i++){
      s_dot.push_back(coeffs[i]*i);
    }
    return s_dot;
}





// COST FUNCTIONS

double time_diff_cost(double target_time, double actual_time) {
  // Penalizes trajectories that span a duration which is longer or shorter than the duration requested.
  return logistic(fabs(actual_time - target_time) / target_time);
}

double traj_diff_cost(vector<double> s_traj, vector<double> target_s) {
  // Penalizes trajectories whose s coordinate (and derivatives) differ from the goal. Target is s, s_dot, and s_ddot.
  // can be used for d trajectories as well (or any other 1-d trajectory)
  int s_end = s_traj.size();
  double s1, s2, s3, s_dot1, s_dot2, s_ddot, cost = 0;
  s1 = s_traj[s_end - 1];
  s2 = s_traj[s_end - 2];
  s3 = s_traj[s_end - 3];
  s_dot1 = (s1 - s2) / DT;
  s_dot2 = (s2 - s3) / DT;
  s_ddot = (s_dot1 - s_dot2) / DT;
  cost += fabs(s1 - target_s[0]) / SIGMA_S;
  cost += fabs(s_dot1 - target_s[1]) / SIGMA_S_DOT;
  cost += fabs(s_ddot - target_s[2]) / SIGMA_S_DDOT;
  return logistic(cost);
}

double s_diff_cost(vector<vector<double>> traj, vector<double> target_vel,vector<double>delta){
  // Given trajectories s and d polynormaile coeffs   
  // Penalizes trajectories whose s coordinate (and derivatives) differ from the goal.
  // 
  vector<double> s_coeffs = traj[0]; 
  vector<double> s_dot_coeffs;
  vector<double> s_d_dot_coeffs;
  double s, s_dot, s_d_dot, cost = 0;
  double t = traj[2][0];
  vector<double> est = state_in(target_vel, t); 
  vector<double> est_s; // with safety buffer zone
  // add the delta state( safety buffer zone) 
  for (int i =0; i< est.size(); i++){
	est_s.push_back(target_vel[i]-delta[i]);
  }
    
  // the projected {s, s_dot, s_d_dot} at time t by given trajectory
  

  s_dot_coeffs = differentiate_s(s_coeffs);
  s_d_dot_coeffs = differentiate_s(s_dot_coeffs);

  s = eval_poly_equation(s_coeffs,t);
  s_dot = eval_poly_equation(s_dot_coeffs,t);
  s_d_dot = eval_poly_equation(s_d_dot_coeffs,t);

  
  cost += logistic(fabs(s - est_s[0]) / SIGMA_S);
  cost += logistic(fabs(s_dot - est_s[1]) / SIGMA_S_DOT);
  cost += logistic(fabs(s_d_dot - est_s[2]) / SIGMA_S_DDOT);
  return cost;
}

double d_diff_cost(vector<vector<double>> traj, vector<double> target_vel, vector<double>delta){
  // Given trajectories s and d polynormaile coeffs   
  // Penalizes trajectories whose d coordinate (and derivatives) differ from the goal.

  // use delta as fixed safety zone
  vector<double> d_coeffs = traj[1]; 
  vector<double> d_dot_coeffs;
  vector<double> d_d_dot_coeffs;
  double d, d_dot, d_d_dot, cost = 0;
  double t = traj[2][0];
  vector<double> est = state_in(target_vel, t); 
  vector<double> est_d; // with safety buffer zone
  // add the delta state( safety buffer zone) 
  for (int i =0; i< est.size(); i++){
	est_d.push_back(target_vel[i]-delta[i]);
  }
    
  // the projected {s, s_dot, s_d_dot} at time t by given trajectory
  

  d_dot_coeffs = differentiate_s(d_coeffs);
  d_d_dot_coeffs = differentiate_s(d_dot_coeffs);

  d = eval_poly_equation(d_coeffs,t);
  d_dot = eval_poly_equation(d_dot_coeffs,t);
  d_d_dot = eval_poly_equation(d_d_dot_coeffs,t);

  
  cost += logistic(fabs(d - est_d[0]) / SIGMA_D);
  cost += logistic(fabs(d_dot - est_d[1]) / SIGMA_D_DOT);
  cost += logistic(fabs(d_d_dot - est_d[2]) / SIGMA_D_DDOT);
  return cost;
}


double collision_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // Binary cost function which penalizes collisions.
  double nearest = nearest_approach_to_any_vehicle(s_traj, d_traj, predictions);
  if (nearest < 2 * VEHICLE_RADIUS) {
    return 1;
  } else { 
    return 0;
  }
}

double buffer_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // Penalizes getting close to other vehicles.
  double nearest = nearest_approach_to_any_vehicle(s_traj, d_traj, predictions);
  return logistic(2 * VEHICLE_RADIUS / nearest);
}

double in_lane_buffer_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // Penalizes getting close to other vehicles.
  double nearest = nearest_approach_to_any_vehicle_in_lane(s_traj, d_traj, predictions);
  return logistic(2 * VEHICLE_RADIUS / nearest);
}

double exceeds_speed_limit_cost(vector<double> s_traj) {
  // Penalty if ds/dt for any two points in trajectory is greater than SPEED_LIMIT
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  for (double s_dot : s_dot_traj) {
    if (s_dot > SPEED_LIMIT) {
      return 1;
    }
  }
  return 0;
}

double efficiency_cost(vector<double> s_traj) {
  // Rewards high average speeds.
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  double final_s_dot, total = 0;

  // cout << "DEBUG - s_dot: ";
  // for (double s_dot: s_dot_traj) {
  //   cout << s_dot << ", ";
  //   total += s_dot;
  // }
  // cout << "/DEBUG" << endl;
  // double avg_vel = total / s_dot_traj.size();

  final_s_dot = s_dot_traj[s_dot_traj.size() - 1];
  // cout << "DEBUG - final s_dot: " << final_s_dot << endl;
  return logistic((SPEED_LIMIT - final_s_dot) / SPEED_LIMIT);
} 

double max_accel_cost(vector<double> s_traj) {
  // Penalize exceeding MAX_INSTANTANEOUS_ACCEL
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  vector<double> s_ddot_traj = velocities_for_trajectory(s_dot_traj);
  for (double s_ddot : s_ddot_traj) {
    if (s_ddot > MAX_INSTANTANEOUS_ACCEL) {
      return 1;
    }
  }
  return 0;
}

double avg_accel_cost(vector<double> s_traj) {
  // Penalize higher average acceleration
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  vector<double> s_ddot_traj = velocities_for_trajectory(s_dot_traj);
  double total = 0;
  for (double s_ddot: s_ddot_traj) {
    total += s_ddot;
  }
  double avg_accel = total / s_ddot_traj.size();
  return logistic(avg_accel / EXPECTED_ACC_IN_ONE_SEC );
}

double max_jerk_cost(vector<double> s_traj) {
  // Penalize exceeding MAX_INSTANTANEOUS_JERK
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  vector<double> s_ddot_traj = velocities_for_trajectory(s_dot_traj);
  vector<double> s_dddot_traj = velocities_for_trajectory(s_ddot_traj);
  for (double s_dddot : s_dddot_traj) {
    if (s_dddot > MAX_INSTANTANEOUS_JERK) {
      return 1;
    }
  }
  return 0;
}

double avg_jerk_cost(vector<double> s_traj) {
  // Penalize higher average jerk
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  vector<double> s_ddot_traj = velocities_for_trajectory(s_dot_traj);
  vector<double> s_dddot_traj = velocities_for_trajectory(s_ddot_traj);
  double total = 0;
  for (double s_dddot: s_dddot_traj) {
    total += s_dddot;
  }
  double avg_jerk = total / s_dddot_traj.size();
  return logistic(avg_jerk / EXPECTED_JERK_IN_ONE_SEC );
}

double not_middle_lane_cost(vector<double> d_traj) {
  // penalize not shooting for middle lane (d = 6)
  double end_d = d_traj[d_traj.size()-1];
  return logistic(pow(end_d-6, 2));
}

double lane_departing_cost(vector<double> d_traj, double d) {
  // penalize not stay in the middle of any lane (d = 2, 6, 10)
  vector<double> ptsx = {0, 2, 4, 6, 8, 10, 12};
  vector<double> ptsy = {1, 0.01, 1, 0.05, 1, 0.1, 1};
  
  //double end_d = d_traj[d_traj.size()-1];
  tk::spline lane_depart;
  lane_depart.set_points(ptsx, ptsy);
  return lane_depart(d);
  //return lane_depart(d_traj[d_traj.size()-1]);
}

double lane_following_cost(vector<double> s_traj, double T) {
  // penalize too close to the leading car, or too far behind the leading car
  // the sweat spot is between 2-3 second
  vector<double> ptsx = {0.5, 1, 2, 3, 4, 5, 10, 15};
  vector<double> ptsy = {1, 0.5, 0.1, 0.1, 0.2, 0.3, 0.5};
  
  tk::spline lane_follow;
  lane_follow.set_points(ptsx, ptsy);
  return lane_follow(T);
}

double left_lane_passing_cost(vector<double> s_traj, double T) {
  // penalize stay too long in other car's blind spot, and passing from left
  vector<double> ptsx = {-5, -4,  -3,  -2, -1, 0, 0.5, 1, 2, 3, 4};
  vector<double> ptsy = {0.15, 0.2, 0.5, 1, 1, 1, 1, 0.5, 0.3, 0.2, 0.15};
  
  tk::spline left_lane_passing;
  left_lane_passing.set_points(ptsx, ptsy);
  return left_lane_passing(T);
}

double right_lane_passing_cost(vector<double> s_traj, double T) {
  // penalize stay too long in other car's blind spot, and passing from right
  vector<double> ptsx = {-5, -4,  -3,  -2, -1, 0, 0.5, 1, 2, 3, 4};
  vector<double> ptsy = {0.2, 0.25, 0.6, 1, 1, 1, 1, 0.6, 0.4, 0.3, 0.25};
  
  tk::spline right_lane_passing;
  right_lane_passing.set_points(ptsx, ptsy);
  return right_lane_passing(T);
}

double blind_spot_cost(vector<double> s_traj, double T) {
  // penalize stay in other car's blind spot, left, right or behind
  vector<double> ptsx = {-7, -6, -5, -4,  -3,  -2, -1, 0, 0.5, 1, 2, 3, 4};
  vector<double> ptsy = {0.1, 0.15, 0.2, 0.25, 0.6, 1, 1, 1, 1, 0.6, 0.3, 0.2, 0.15};
  
  tk::spline blind_spot;
  blind_spot.set_points(ptsx, ptsy);
  return blind_spot(T);
}

double open_speed_cost(vector<double> s_traj, double speed) {
  // penalize the max car speed in a giving traj is close to or exceed speed limit, or too slow in traffic
  vector<double> ptsx = {0, 10, 20, 30, 40, 45, 48, 49, 50, 60, 80};
  vector<double> ptsy = {0.4, 0.3, 0.2, 0.1, 0.1, 0.1, 0.5, 0.6, 1, 1, 1};
  
  tk::spline open_speed;
  open_speed.set_points(ptsx, ptsy);
  return open_speed(speed);
}




double calculate_total_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {

  double total_cost = 0;
  double col = collision_cost(s_traj, d_traj, predictions) * COLLISION_COST_WEIGHT;
  double buf = buffer_cost(s_traj, d_traj, predictions) * BUFFER_COST_WEIGHT;
  double ilb = in_lane_buffer_cost(s_traj, d_traj, predictions) * IN_LANE_BUFFER_COST_WEIGHT;
  double eff = efficiency_cost(s_traj) * EFFICIENCY_COST_WEIGHT;
  double nml = not_middle_lane_cost(d_traj) * NOT_MIDDLE_LANE_COST_WEIGHT;
  //double esl = exceeds_speed_limit_cost(s_traj) * SPEED_LIMIT_COST_WEIGHT;
  //double mas = max_accel_cost(s_traj) * MAX_ACCEL_COST_WEIGHT;
  //double aas = avg_accel_cost(s_traj) * AVG_ACCEL_COST_WEIGHT;
  //double mad = max_accel_cost(d_traj) * MAX_ACCEL_COST_WEIGHT;
  //double aad = avg_accel_cost(d_traj) * AVG_ACCEL_COST_WEIGHT;
  //double mjs = max_jerk_cost(s_traj) * MAX_JERK_COST_WEIGHT;
  //double ajs = avg_jerk_cost(s_traj) * AVG_JERK_COST_WEIGHT;
  //double mjd = max_jerk_cost(d_traj) * MAX_JERK_COST_WEIGHT;
  //double ajd = avg_jerk_cost(d_traj) * AVG_JERK_COST_WEIGHT;
  //double tdiff = time_diff_cost(target_time, actual_time) * TIME_DIFF_COST_WEIGHT;
  //double strajd = traj_diff_cost(s_traj, target_s) * TRAJ_DIFF_COST_WEIGHT;
  //double dtrajd = traj_diff_cost(d_traj, target_d) * TRAJ_DIFF_COST_WEIGHT;

  total_cost += col + buf + ilb + eff + nml;// + esl + mas + aas + mad + aad + mjs + ajs + mjd + ajd;

  // // DEBUG
  // cout << "costs - col: " << col << ", buf: " << buf << ", ilb: " << ilb << ", eff: " << eff << ", nml: " << nml; 
  // //cout << ", " << esl 
  // //cout << ", " << mas << ", " << aas << ", " << mad << ", " << aad;
  // //cout << ", " << mjs << ", " << ajs << ", " << mjd << ", " << ajd;
  // cout << "  ** ";
  // //cout << endl;
  // //cout << "total cost: " << total_cost << endl;

  return total_cost;
}

#endif
