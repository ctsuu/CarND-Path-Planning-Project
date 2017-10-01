#ifndef CONSTANTS
#define CONSTANTS


// Display
#define NUM_POINTS 50

// 
#define DELTA_T 0.02                    // seconds, delta t to match simulator refresh rate
#define COST_DT 0.15
#define COST_SAMPLES 15

// map rules
#define MAX_S 6945.554                  // meters
#define HD_RES 0.5			// meters


// for trajectory generation
#define TRAJ_SAMPLES 10
#define TRAJ_DT 0.20                         // seconds


// traffic rules
#define SPEED_LIMIT 22.10               // 50MPH/2.24 = 22.3m/s
#define V_INC 0.225
#define DIST_INC 0.445

#define VEHICLE_RADIUS 2.25              // meters
#define FOLLOW_DISTANCE 12.0              // distance to keep behind leading cars
#define FOLLOWING_GAP 1.2		 // front safety distance, present in sec

// comfort rule
#define POWER 0.99                       // 99% output
#define MAX_INSTANTANEOUS_JERK 9.8       // m/s/s/s
#define MAX_INSTANTANEOUS_ACCEL 9.8      // m/s/s

#define MAX_JERK 9.8       		// m/s/s/s
#define MAX_ACCEL 9.8     		// m/s/s

#define EXPECTED_JERK_IN_ONE_SEC 2      // m/s/s
#define EXPECTED_ACC_IN_ONE_SEC 1       // m/s


// sigma values for perturbing targets
#define SIGMA_S 7.0                       // s
#define SIGMA_S_DOT 3.0                   // s_dot
#define SIGMA_S_DDOT 0.1                  // s
#define SIGMA_D 0.5                       // d
#define SIGMA_D_DOT 0.1                   // d_dot
#define SIGMA_D_DDOT 0.1                  // d_double_dot
#define SIGMA_T 0.5			  // +/- 0.5s



#endif
