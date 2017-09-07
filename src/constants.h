#ifndef CONSTANTS
#define CONSTANTS


//#define PREVIOUS_PATH_POINTS_TO_KEEP 30
#define RESERVED_PATH_POINT 40
#define NUM_POINTS 50

//#define PATH_DT 0.02                    // seconds
#define DELTA_T 0.02                    // seconds, delta t to match simulator refresh rate
#define COST_DT 0.15
#define COST_SAMPLES 15

#define TRACK_LENGTH 6945.554           // meters
#define LANE_WIDTH 4.0			// meters
#define LEFT_LANE 0
#define RIGHT_LANE 2


// number of waypoints to use for interpolation
#define NUM_WAYPOINTS_BEHIND 5
#define NUM_WAYPOINTS_AHEAD 10

// for trajectory generation/evaluation and non-ego car predictions
#define TRAJ_SAMPLES 10
#define TRAJ_DT 0.20                         // seconds


// traffic rules
#define SPEED_LIMIT 22.3                // 50MPH/2.24 = 22.3m/s
#define LANE_ZERO_SPEED_FACTOR 0.95     // speed correction
#define LANE_ONE_SPEED_FACTOR 0.975     // speed correction
#define LANE_TWO_SPEED_FACTOR 1.0     // speed correction
//#define SPEED_LIMIT 50.0                  // 50MPH

#define VELOCITY_INCREMENT_LIMIT 0.125
#define DISTANCE_INCREMENT_LIMIT 0.445
#define VEHICLE_RADIUS 2.25              // meters
#define FOLLOW_DISTANCE 8.0              // distance to keep behind leading cars
#define FRONT_SAFETY_GAP 2.0		 // front safety distance, present in sec
#define BACK_SAFETY_GAP 2.0		 // back safety distance, present in sec
#define FOLLOWING_GAP 2.0		 // front safety distance, present in sec

// comfort rule
#define POWER 0.99                       // 99% output
#define MAX_INSTANTANEOUS_JERK 9.8       // m/s/s/s
#define MAX_INSTANTANEOUS_ACCEL 9.8      // m/s/s

#define MAX_JERK 9.8       		// m/s/s/s
#define MAX_ACCEL 9.8     		// m/s/s

#define EXPECTED_JERK_IN_ONE_SEC 2      // m/s/s
#define EXPECTED_ACC_IN_ONE_SEC 1       // m/s


// cost function weights
#define COLLISION_COST_WEIGHT 1.0
#define BUFFER_COST_WEIGHT 1
#define IN_LANE_BUFFER_COST_WEIGHT 1.0
#define NOT_MIDDLE_LANE_COST_WEIGHT 0.01
#define SPEED_LIMIT_COST_WEIGHT 1.0
#define MAX_ACCEL_COST_WEIGHT 0.013
#define AVG_ACCEL_COST_WEIGHT 0.01
#define MAX_JERK_COST_WEIGHT 1.0
#define AVG_JERK_COST_WEIGHT 1.0
#define TIME_DIFF_COST_WEIGHT 0.05
#define TRAJ_DIFF_COST_WEIGHT 0.61
#define S_TRAJ_DIFF_COST_WEIGHT 0.061
#define D_TRAJ_DIFF_COST_WEIGHT 0.05
#define EFFICIENCY_COST_WEIGHT 1.0
#define LANE_DEPARTURE_COST_WEIGHT 1.0
#define LANE_FOLLOWING_COST_WEIGHT 1.0
#define BLIND_SPOT_COST_WEIGHT 1.0
#define OPEN_SPEED_COST_WEIGHT 1.0




// DEPRECATED CONSTANTS
#define NUM_RANDOM_TRAJ_TO_GEN 20        // the number of perturbed trajectories to generate (for each perturbed duration)
#define NUM_TIMESTEPS_TO_PERTURB 5      // the number of timesteps, +/- target time, to perturb trajectories

// sigma values for perturbing targets
#define SIGMA_S 7.0                     // s
#define SIGMA_S_DOT 3.0                 // s_dot
#define SIGMA_S_DDOT 0.1                  // s
#define SIGMA_D 0.5                       // d
#define SIGMA_D_DOT 0.1                   // d_dot
#define SIGMA_D_DDOT 0.1                  // d_double_dot
#define SIGMA_T 0.5			  // +/- 0.5s

//#define SIGMA_S 10.0                    // s
//#define SIGMA_S_DOT 4.0                 // s_dot
//#define SIGMA_S_D_DOT 2.0               // s
//#define SIGMA_D 1.0                       // d
//#define SIGMA_D_DOT 1.0                   // d_dot
//#define SIGMA_D_D_DOT 1.0                  // d_double_dot
//#define SIGMA_T 0.1


#define PERCENT_V_DIFF_TO_MAKE_UP 0.5   // the percent difference between current velocity and target velocity to allow ego car to make up in a single trajectory  

#endif
