# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

This is by far the most difficulte project in the program. Here is the story how I solve it. 

### Project Overview
This is a multi-agent problem. We are dealing with 12 other cars on the road, and they are not nice drivers. The highway is a 6946 meters long 3 lane highway loop. The project requires the self-driving car driving in very smooth manner. Accelection and jerk are less than 10 m/s^2 and 50 m/s^3. Within 4.32 miles, one full loop, no more than 3 second of the three lanes, no speeding, no collision. 

The ego car is equiped with sensor fusion unit which will provide nesscery information for all other cars. A low resolution rough waypoints is about 30 meter apart between two next points. The lane is standard 4 meter wide. Don't cross the double yellow line or off the road. 

I basicly divided the problem into: road and map related problems, sensor fusion related, decision making, and smooth path. 

### Highway mapping
The provided 181 waypoints are not enough to map a smooth trajectory that can stay in the middle of the three lanes all the time. They are represent the middle of the double yellow line. In this project, we are using Frenet s,d coordinates system. Therefore, we need to a good base to transform global coordinates to Frenet or verse vise.  
I decided to make high resolution waypoints using spline tools.    
```
vector<double> getHD(vector<double> pts_x, vector<double> pts_y, 
                                  double interval, int output_size) {
    // X needs be sorted
    tk::spline s;
    s.set_points(pts_x,pts_y); 
    vector<double> output;
    for (int i = 0; i < output_size; i++) {
        output.push_back(s(pts_x[0] + i * interval));
    }
    return output;
}
```
Using the rough x, y points to fit the spline tool, which will create a smooth function. The distance between first x to last x is made up by interval times by number of intervals. Because this track is a loop, a small section between first point to the last point will connect again, and it is left out in low resulation. Due to the imprefect waypoint recording from udacity, skip the last waypoint actually solved the problem. Otherwise, the route will ends up with a s shape at the overlap section. 

I set the waypoint resolution to 0.5 meter. All waypoint calculation is at beginning, and only run once. It does not affect the performance at all.  

### Sensor Fusion Data Processing

Sensor fusion is the eyes of the self-driving car. It shows 12 cars in range and sameside of the highway. We are managed to get some useful informations from it, such as prediction of other cars, lane clearance( nearest front car, nearest rear car, front car speed), and feed in to decision making cost function. 

The prediction has two parts: One part is assume all cars run on constant velocity based on last observation. Another part is predictions are not always start at time zero, once the ego car is moving, we need to prodict all car's movements start at the end time in ego car's previous_path.     
```
    int path_size = previous_path_x.size();
    
    if(path_size <2){
		    ref_start_time = 0;
		    ref_s = s_;
		    ref_d = d_;
		} else {
		    ref_start_time = path_size*DELTA_T;
		    ref_s = end_path_s;
		    ref_d = end_path_d;
		}
```

The second use of sensor fusion is to check the clearance for each lane, find the closest front car distance and speed, also the closest rear car distance. The key is the check speed.
```
check_speed = sqrt((sensor_fusion)[i][3] * (sensor_fusion)[i][3] +
                            (sensor_fusion)[i][4] * (sensor_fusion)[i][4]);
check_speed = max(check_speed, 13.0);			    
```
The speed info will feed into the speed controller to drive the car. 

The third use of sensor fusion is the decision making module. I will explain it in decision making section.  

### Decision making

I have tried many options to determine which lane is the best lane to proceed.  
There are basicly two groups of approach. One is to produce many possible trajectories, than check which trajectory is the best one, then use that trajectory. Another approach is to find which lane has less traffic, and higer speed, then follow the lane. 

Since I can get accurcy speed of the target car, and PD controller to follow that speed, I am favority to the second approach. There is an extra trick in the check clearance function:
```
  if (front_min > 50){
    check_speed = SPEED_LIMIT;
  }
```
If the lane is quite open, go to the speed limit, to catch up the traffic. 

There are three layers of detection for lane states, is the lane open or I have to do keep lane operation?   

The first layer is the getLaneState() function. It will take the sensor fusion data, and ego car current state, returns each lane weither it is open or keep lane. The function will take the ego car current speed into account:
```
	lead_t = lead_s/car_v; // in seconds
```
All calculation is based on time to reach the last waypoint. 

The second layer is the findBestLane() function. It will take the sensor fusion data, based on the distance to the ego car and speed of the car, the cost function is in favour to less traffice and faster speed lane. Also, there are some spots on lane 2 will cause "out of lane", when I initiated three lanes, the priority will be in order of lane 1, lane 0 and lane 2.    
```
  vector<double> lane_cost = {0.000001, 0, 0.00001};

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

```

The third layer is the execute() function. 
```
int Vehicle::execute(int best_lane, int c_lane, vector<vector<double >> sensor_fusion){
    if( this->lane_follow ){
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
    	  } else if (this->FR_clear && this->RR_clear && best_lane > c_lane && 
		     this->right_lane_open){
            c_lane = best_lane;
          }
        } else {
  	  // check middle lane clearance
    	  vector<double> middle_lane = checkClearance(sensor_fusion, this->s, 1);

          if (middle_lane[0] >= FOLLOW_DISTANCE && middle_lane[1] >= FOLLOW_DISTANCE+4) {
	    if(c_lane = 2 && left_lane_open){
              c_lane = 1;
	      this->lane_follow = false;
              cout << "change from right lane to middle lane \n";
	    } else if(c_lane = 0 && right_lane_open){
	      c_lane = 1;
	      this->lane_follow = false;
              cout << "change from left lane to middle lane \n";
	    }
          } else {
            cout << "no double shift \n";
            this->lane_follow = true;
          }
	}
  }
  return c_lane;
}
```
If the car is in the middle lane, check both side and change lane when the target lane is clear. 
If the car is in lane 0 or lane 2, check the middle lane again, and change lane if it is clear. This also prevent the car cross two lanes at once. The car have to wait for next circle to cross another lane. 

### Regen the trajectory



### Results

### Reflection



















   
### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
