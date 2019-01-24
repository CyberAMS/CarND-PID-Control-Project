# Project: Path Planning

This project has been prepared by Andre Strobel.

The goal of this project is to program the behavior of a self-driving car during highway traffic. It must avoid accidents, follow traffic rules and also apply comfortable trajectories.

Key aspects like using splines for [Frenet](https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas) coordinate conversion and the solution for jerk minimizing trajectories follow [Eddie Forson's solution](https://towardsdatascience.com/teaching-cars-to-drive-highway-path-planning-109c49f9f86c).

The following table shows an overview of the most important files:

| File                          | Description                                                                                          |
|-------------------------------|------------------------------------------------------------------------------------------------------|
| README.md                     | This file                                                                                            |
| build.sh                      | Script to build the path planning executable                                                         |
| run.sh                        | Script to run the path planning executable                                                           |
| data/highway_map.csv          | Provided map data inlcuding data for Frenet conversion                                               |
| src/main.cpp                  | Source code of the main function of the path planning program                                        |
| src/json.hpp                  | Source code for reading and writing [JAVAScript Object Notation](https://en.wikipedia.org/wiki/JSON) |
| src/Map.{h, cpp}              | Source code of the map object                                                                        |
| src/Driver.{h, cpp}           | Source code of the driver object                                                                     |
| src/Vehicle.{h, cpp}          | Source code of the vehicle object                                                                    |
| src/Path.{h, cpp}             | Source code of the path object                                                                       |
| src/Trajectory.{h, cpp}       | Source code of the trajectory object                                                                 |
| src/State.{h, cpp}            | Source code of the state object                                                                      |
| src/spline.{h, cpp}           | Source code of the [spline object](https://kluge.in-chemnitz.de/opensource/spline/)                  |
| src/helper_functions.{h, cpp} | Source code of helper functions for the path planning program                                        |
| out.txt                       | Contains an example debugging output for a full run                                                  |

---

## Content

1. Tool chain setup
    1. Gcc, Cmake, Make and uWebSocketIO
    1. Udacity Simulator
1. Data objects and structures
    1. Map as well as simulator input and output
    1. Driver, Vehicle, Path, Trajectory and State classes
1. Path planning implementation
    1. Program flow
    1. Finite state model
    1. Jerk minimizing trajectories
    1. Cost functions
    1. Debugging environment
1. Execution
    1. Commands to start the simulation
    1. Simulation results
1. Discussion

[//]: # (Image References)

[image1]: ./docu_images/190120_StAn_Path_Planning_Program_Flow.jpg
[image2]: ./docu_images/190120_StAn_Finite_State_Model.jpg
[image3]: ./docu_images/190120_StAn_Frenet_Problem.jpg

---

## 1. Tool chain setup

### 1. Gcc, Cmake, Make and uWebSocketIO

This project requires the following programs:

* gcc/g++ >= 5.4
  - Linux: gcc / g++ is installed by default on most Linux distros
  - Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  - Windows: recommend using [MinGW](http://www.mingw.org/)
  
* cmake >= 3.5
  - All OSes: [click here for installation instructions](https://cmake.org/install/)
  
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  - Linux: make is installed by default on most Linux distros
  - Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  - Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
  
* [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)
  - Works with Linux and Mac systems
  - Windows: Use Docker, VMware or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) (although I wasn't able to get it working with the latest Ubuntu app in Windows 10)

### 2. Udacity Simulator

The path planning program connects to the [Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases) version [Term 3 Simulator v1.2](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2) via [uWebSocketIO](https://github.com/uWebSockets/uWebSockets). The simulator is available for Linux, Mac and Windows.

## 2. Data objects and structures

### 1. Map as well as simulator input and output

The map information is loaded from the file `highway_map.csv`. It is a list of waypoints with the following structure:

| Column | Description                                    |
|--------|------------------------------------------------|
| 1      | x position of waypoints                        |
| 2      | y position of waypoints                        |
| 3      | s coordinate (longitudinal distance)           |
| 4      | x component of lateral unit normal vector (dx) |
| 5      | y component of lateral unit normal vector (dy) |

The `Map` class is used to store the map and execute all conversions from cartesian xy coordinates to Frenet sd coordinates (longitudinal and lateral distances) and back. It also manages that the track is a loop and longitudinal Frenet s coordinates must be treated accordingly. Therefore, every assignment of an s coordinate must be done using the `AssignS()` method and every difference in s coordinates must be calculated using the `DeltaS()` method. Comparing two s coordinates also uses the `DeltaS()` method by checking whether the difference is smaller, equal or larger than 0. The following list contains the most important methods of this class:

| Method | Description |
|-|-|
| `Xy2Frenet()` | Convert cartesian xy coordinates to Frenet sd coordinates. |
| `Frenet2Xy()` | Convert Frenet sd coordinates to cartesian xy coordinates. |
| `AssignS()`   | Assign longitudinal Frenet s coordinates considering that the track is a loop when s values exceed the length of the track. |
| `DeltaS()`    | Calculate the delta between two longitudinal Frenet s coordinates considering that the track is a loop when s values exceed the length of the track. |

The behavior of this class is controlled with the following constants which define the missing information of the map:

```C
// map parameters
const unsigned int MAX_TRACK_S = 6945.554;

// lane parameters
const double LANE_WIDTH = 4.0;
const double LANE_CENTER_WIDTH = 3.0;
const unsigned int LANE_1 = 1;
const unsigned int LANE_2 = 2;
const unsigned int LANE_3 = 3;
const vector<unsigned int> LANES = {LANE_1, LANE_2, LANE_3};
```

The simulator sends information about the own vehicle's current state in the following format:

| Variable | Value or Description                                                                  |
|----------|---------------------------------------------------------------------------------------|
| `x`      | x position                                                                            |
| `y`      | y position                                                                            |
| `s`      | s coordinate (longitudinal distance from start of map)                                |
| `d`      | d coordinate (lateral distance from center of the road, positive values to the right) |
| `yaw`    | yaw angle relative to cartesian xy coordinates                                        |
| `speed`  | speed in mph                                                                          |

The simulator also sends the `sensor_fusion` message about the other vehicles on the road as perceived by the own vehicle:

| Variable | Value or Description                                                                  |
|----------|---------------------------------------------------------------------------------------|
| `id`     | unique identification number for observed vehicle                                     |
| `x`      | x position                                                                            |
| `y`      | y position                                                                            |
| `v`      | vehicle velocity in m/s                                                               |
| `s`      | s coordinate (longitudinal distance from start of map)                                |
| `d`      | d coordinate (lateral distance from center of the road, positive values to the right) |

The simulator receives lists of x coordinates `next_x` and y coordinates `next_y` for points the own vehicle should follow. The points are spaced in 20 ms increments which is the [sampling rate](https://en.wikipedia.org/wiki/Sampling_(signal_processing)) of the simulator. All points that the simulator did not use up before sending the next message are listed in the `previous_path_x` and `previous_path_y` messages.

### 2. Driver, Vehicle, Path, Trajectory and State classes

The path planning program contains several objects that interact which each other. All objects that are based on these classes keep their data private and use access methods to exchange data with other objects. The access methods start with `Get` and `Set`.

The `Driver` class is responsible for receiving feedback from the environment, determining the best next behavior and sending instructions back to the own vehicle in the simulator. If no valid trajectories are found for the next possible behaviors, the driver defaults back to a predicted trajectory based on its own vehicle state, i.e. no change. A driver owns a vehicle, information about other vehicles and the environment including a map as well as a behavioral state. The following list contains the most important methods of this class:

| Method | Description |
|-|-|
| `PlanBehavior()` | Update vehicle, vehicle trajectory and state with information from the simulator. Pick the next possible behaviors and calculate a trajectory for each. Select the minimum cost trajectory and send it to the simulator. |

The `Vehicle` class is storing and processing all information about a single vehicle. It knows its state, dimensions and future trajectory. It can locate itself in a map. Therefore, an object of this class is also capable of determining the cost of different trajectories. The following list contains the most important methods of this class:

| Method | Description |
|-|-|
| `PredictTrajectory()` | Predict a future trajectory for a vehicle based on its current location and speed. |
| `TrajectoryCost()` | Calculate the full cost of a trajectory. |
| `CostStepsToCollision()` | Calculate the cost for having a collision after the given number of steps (the more steps the lower the cost). |
| `CostSpaceAhead()` | Calculate the cost for being too close to the vehicle in front in the intended lane (the farther away the lower the cost). |
| `CostSpaceInIntendedLane()` | Calculate the cost for having or not having space in the intended lane at the vehicle's longitudinal position (high cost if no space). |
| `CostSpeedInIntendedLane()` | Calculate the cost for the expected speed in the intended lane (the higher the speed the lower the cost). |
| `CostTravelDistance()` | Calculate the cost for how far a trajectory travels (the further it travels the lower the cost). |
| `Ahead()` | Identify the vehicles ahead of the own vehicle in a given lane. |
| `Behind()` | Identify the vehicles behind the own vehicle in a given lane. |
| `DetectCollision()` | Compare the own trajectory with the trajectories of other vehicles and determine at what step a collision will occur next if any. |

The behavior of this class is controlled with the below mostly self explaining constants. The `COST_STEPS_TO_COLLISION_SHAPE_FACTOR`, `COST_SPACE_AHEAD_SHAPE_FACTOR`, `COST_SPEED_IN_INTENDED_LANE_SHAPE_FACTOR` and `COST_TRAVEL_DISTANCE_SHAPE_FACTOR` parameters are used to influence the shape of the cost functions that are explained further below. The desired distance in time to the vehicle in front in the intended lane is defined by the parameter `AHEAD_DISTANCE_TIME`. The necessary gap in the left or right lane before a lane change is defined by multiples of the ahead and behind vehicle's length using the factors `AHEAD_SPACE_FACTOR` and `BEHIND_SPACE_FACTOR`. In order to determine the speed of a lane, the algorithm only looks for vehicles ahead that are closer than `VEHICLE_AHEAD_WITHIN_DISTANCE`. Finally, the parameter `MAX_TRAVEL_DISTANCE` defines the maximum expected travel of a trajectory to determine a balanced normalized cost function.

```C
// vehicle parameters
const unsigned int EGO_CAR_ID = 0;
const double EGO_CAR_SV_INIT = 0.0;
const double EGO_CAR_SA_INIT = 0.0;
const double EGO_CAR_SJ_INIT = 0.0;
const double EGO_CAR_DV_INIT = 0.0;
const double EGO_CAR_DA_INIT = 0.0;
const double EGO_CAR_DJ_INIT = 0.0;
const double STANDARD_VEHICLE_WIDTH = 2.0;
const double STANDARD_VEHICLE_LENGTH = 4.0;
const double SAFETY_BOX_DISTANCE = 0.5; // must have 0.5 m distance to all vehicles around own vehicle

// cost parameters
const double DESIRED_LONGITUDINAL_TIME_DISTANCE = 1.0; // keep a distance of 1 s
const double NO_HARMFUL_COLLISION_STEPS = DESIRED_LONGITUDINAL_TIME_DISTANCE / SAMPLE_TIME;
const double COST_STEPS_TO_COLLISION_SHAPE_FACTOR = 10.0;
const double AHEAD_DISTANCE_TIME = 1.5; // desired time to vehicle in front 1.5 s
const double COST_SPACE_AHEAD_SHAPE_FACTOR = 10.0;
const double AHEAD_SPACE_FACTOR = 2.0;
const double BEHIND_SPACE_FACTOR = 4.0;
const double VEHICLE_AHEAD_WITHIN_DISTANCE = 50.0;
const double COST_SPEED_IN_INTENDED_LANE_SHAPE_FACTOR = 10.0;
const double MAX_TRAVEL_DISTANCE = MAX_SPEED * STEP_TIME_INTERVAL;
const double COST_TRAVEL_DISTANCE_SHAPE_FACTOR = 10.0;

// cost weights
const double ZERO_COST = 0.0;
const double MAX_NORMALIZED_COST = 1.0;
const double COST_COLLISON_WEIGHT = 10.0;
const double COST_SPACEAHEAD_WEIGHT = 5.0;
const double COST_SPACEININTENDEDLANE_WEIGHT = 5.0;
const double COST_SPEEDININTENDEDLANE_WEIGHT = 1.0;
const double COST_TRAVELDISTANCE_WEIGHT = 1.0;
```

The `Path` class is only used to store the segment of the vehicle's path that has not yet been executed by the simulator.

The `Trajectory` class defines a trajectory using cartesian xy positions including the yaw angle for each position. It also contains a full state description in Frenet sd coordinates (location, velocity, acceleration and jerk). And finally it remembers in which lane of the map it is supposed to end. The following list contains the most important methods of this class:

| Method | Description |
|-|-|
| `Generate()` | Generate a full trajectory. |
| `Add()` | Add a point to the trajectory. |
| `AddJerkMinimizingTrajectory()` | Add a jerk minimized segment to the trajectory. |
| `Valid()` | Check whether a trajectory is valid and adjust it accordingly if necessary and possible. |

The behavior of this class is controlled with the below mostly self explaining constants. The parameter using `TRAJECTORY_VALID_GAIN` defines which state values are used to validate and adjust a trajectory. The selected `SV_SA_V` option uses the velocity and acceleration in longitudinal Frenet coordinates as well as the velocity in cartesian coordinates. As the longitudinal Frenet coordinate direction curves through the cartesian coordinate system, the magnitude of longitudinal distances, velocities and accelerations in the Frenet coordinate system differ from the same values in the cartesian coordinate system. Due to the sometimes slightly inaccurate Frenet to cartesian conversion the `Valid` method offers the option to not only look at cartesian coordinates. The parameter `TARGET_SPEED_FROM_ZERO` indirectly defines a higher accerleration when starting from standstill compared to the accelerations used when crusing on the highway. This enables to get the vehicle going as quickly and comfortable as possible.

```C
// general settings
const double SAMPLE_TIME = 0.020; // 20 ms sample time of simulator (50 Hz)
const double STEP_TIME_INTERVAL = 1.7; // number of seconds from step to step
const double NEUTRAL_GAIN = 1.0;
enum TRAJECTORY_VALID_GAIN {NOTHING, SV_V, SV_SA, SV_SA_V, ALL};
const TRAJECTORY_VALID_GAIN TRAJECTORY_VALID_GAIN_SELECTION = SV_SA_V;

// trajectory definitions
const long NUM_PREVIOUS_PATH_STEPS = 10;
const long MIN_PREVIOUS_PATH_STEPS = 0;

// longitudinal definitions
const double ZERO_SPEED = 0.0;
const double SAFETY_DELTA_SPEED = 0.25 * MPH2MS; // travel 0.25 mph below maximum speed
const double MAX_SPEED = (50 * MPH2MS) - SAFETY_DELTA_SPEED; // 50 mph minus safety delta in m/s
const double SAFETY_DELTA_ACCELERATION = 1.0; // keep maximum acceleration 1 m/s below limit
const double MAX_ACCELERATION_S = 10.0 - SAFETY_DELTA_ACCELERATION; // maximum total acceleration is 10 m/s^2 - longitudinal acceleration is treated independently here
const double MAX_DECELERATION_S = -MAX_ACCELERATION_S;
const double NORMAL_ACCELERATION_S = (MAX_ACCELERATION_S / 10);
const double NORMAL_DECELERATION_S = (MAX_DECELERATION_S / 5);
const double TARGET_SPEED_FROM_ZERO = (MAX_ACCELERATION_S / 2) * STEP_TIME_INTERVAL; // start with half of the maximum acceleration

// lateral definitions
const double MAX_ACCELERATION_D = 10.0; // maximum total acceleration is 10 m/s^2 - lateral acceleration is treated independently here
```

The `State` class contains a finite state model to manage behaviors. The following list contains the most important methods of this class:

| Method | Description |
|-|-|
| `GetNextPossibleBehaviors()` | Select the next possible behaviors based on the current behavior and given vehicle's state. |
| `GenerateTrajectoryFromBehavior()` | Determine the trajectory target values (longitudinal and lateral position, velocity and acceleration) based on the selected behavior. |

The behavior of this class is controlled with the below mostly self explaining constants. It is important to mention that the complete behavior of the finite state model is defined with these paramneters. The parameter `LANE_CHANGE_TRANSITION_TIME` defines the number of steps needed for a lane change. The lane change maneuver must be fully executed before another behavior can be selected.

```C
// define types
enum LONGITUDINALSTATE {ACCELERATE, KEEP_SPEED, DECELERATE};
enum LATERALSTATE {KEEP_LANE, PREPARE_LANE_CHANGE_LEFT, PREPARE_LANE_CHANGE_RIGHT, CHANGE_LANE_LEFT, CHANGE_LANE_RIGHT};
struct behavior_state {
	
	LONGITUDINALSTATE longitudinal_state;
	LATERALSTATE lateral_state;
	
};
struct transition {
	
	LATERALSTATE name;
	vector<behavior_state> next;
	
};

// define constants
const unsigned long INITIAL_STEP = 0;
const behavior_state INITIAL_STATE {.longitudinal_state = ACCELERATE, .lateral_state = KEEP_LANE};
const vector<transition> TRANSITIONS
	{{.name = KEEP_LANE,
	  .next = {{.longitudinal_state = ACCELERATE, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = DECELERATE, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = PREPARE_LANE_CHANGE_LEFT},
	           {.longitudinal_state = DECELERATE, .lateral_state = PREPARE_LANE_CHANGE_LEFT},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = PREPARE_LANE_CHANGE_RIGHT},
	           {.longitudinal_state = DECELERATE, .lateral_state = PREPARE_LANE_CHANGE_RIGHT}}},
	 {.name = PREPARE_LANE_CHANGE_LEFT,
	  .next = {{.longitudinal_state = KEEP_SPEED, .lateral_state = CHANGE_LANE_LEFT},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = PREPARE_LANE_CHANGE_LEFT},
	           {.longitudinal_state = DECELERATE, .lateral_state = PREPARE_LANE_CHANGE_LEFT}}},
	 {.name = PREPARE_LANE_CHANGE_RIGHT,
	  .next = {{.longitudinal_state = KEEP_SPEED, .lateral_state = CHANGE_LANE_RIGHT},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = PREPARE_LANE_CHANGE_RIGHT},
	           {.longitudinal_state = DECELERATE, .lateral_state = PREPARE_LANE_CHANGE_RIGHT}}},
	 {.name = CHANGE_LANE_LEFT,
	  .next = {{.longitudinal_state = KEEP_SPEED, .lateral_state = KEEP_LANE}}},
	 {.name = CHANGE_LANE_RIGHT,
	  .next = {{.longitudinal_state = KEEP_SPEED, .lateral_state = KEEP_LANE}}}};
const long LANE_CHANGE_TRANSITION_TIME = 0.5 * STEP_TIME_INTERVAL / SAMPLE_TIME; // in steps
const long NO_STEP_INCREASE = 0;
```

## 3. Path planning implementation

### 1. Program flow

The flow of the path planning program is defined by the interactions of its objects. The following chart visualizes on a very high level how the individual objects work together. The core of the program is the `PlanBehavior()` method of the `driver` object inside the `main()` function. It initializes and updates the state and trajectory objects, determines the next possible behaviors (using the state object), calculates trajectories for all of these behaviors (using the state object which generates new trajectory objects) including their costs (using the individual trajectory objects as well as the objects for the own vehicle and other vehicles) and finally selects the lowest cost behavior.

![alt text][image1]

### 3. Finite state model

The below finite state model is used to determine the next possible behavior. Longitudinal states (ACCELERATE, KEEP SPEED and DECELERATE) thermselves allow every possible transition inbetween them. But these transitions get limited by the selection of the lateral state (KEEP LANE, PREPARE LANE CHANGE LEFT, PREPARE LANE CHANGE RIGHT, CHANGE LANE LEFT and CHANGE LANE RIGHT) as indicated by the colors.

![alt text][image2]

### 4. Jerk minimizing trajectories

The jerk minimizing trajectory formulas are implemented in the `helper_functions.cpp` file. The first step is to determine the coefficients for the polynomial. This is done using the matrix and vector functions below. The inputs are the state vector at the start `start` and end `end` of the trajectory as well as the duration `T` of the trajectory segment.

```C
// determine polynomial coefficients for jerk minimizing trajectory
vector<double> JerkMinimizingTrajectoryCoefficients(vector<double> start, vector<double> end, double T) {
	
	// define variables
	double T2 = 0.0;
	double T3 = 0.0;
	double T4 = 0.0;
	double T5 = 0.0;
	MatrixXd T_matrix(3, 3);
	VectorXd diff_vector(3);
	VectorXd poly_vector;
	vector<double> poly;
	
	// determine time values
	T2 = pow(T, 2);
	T3 = pow(T, 3);
	T4 = pow(T, 4);
	T5 = pow(T, 5);
	
	// determine time matrix
	T_matrix <<     T3,      T4,      T5,
	            3 * T2,  4 * T3,  5 * T4,
	             6 * T, 12 * T2, 20 * T3;
	
	// determine difference based on start and end
	diff_vector << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * T2),
	               end[1] - (start[1] + start[2] * T),
	               end[2] - (start[2]);
	
	// calculate polynomial coefficients vector
	poly_vector = T_matrix.inverse() * diff_vector;
	
	// determine polynomial coefficients
	poly = (vector<double>){start[0], start[1], (0.5 * start[2]), poly_vector[0], poly_vector[1], poly_vector[2]};
	
	return poly;
	
}
```

The final step is to calculate the state at any given time `t` within the trajectory segment. This is also implemented in the `helper_functions.cpp` file as follows:

```C
// determine states with jerk minimizing trajectory
vector<double> JerkMinimizingTrajectoryState(vector<double> poly, vector<double> start, double t) {
	
	// define variables
	double t2 = 0.0;
	double t3 = 0.0;
	double t4 = 0.0;
	double t5 = 0.0;
	double state = 0.0;
	double state_d = 0.0;
	double state_dd = 0.0;
	double state_ddd = 0.0;
	
	//initialize outputs
	vector<double> states;
	
	// determine time values
	t2 = pow(t, 2);
	t3 = pow(t, 3);
	t4 = pow(t, 4);
	t5 = pow(t, 5);
		
	// determine states
	state     =      (start[0]) +       (start[1]) * t + (0.5 * start[2]) * t2 +        (poly[3]) * t3 +       (poly[4]) * t4 + (poly[5]) * t5;
	state_d   =      (start[1]) +       (start[2]) * t +  (3.0 * poly[3]) * t2 +  (4.0 * poly[4]) * t3 + (5.0 * poly[5]) * t4;
	state_dd  =      (start[2]) +  (6.0 * poly[3]) * t + (12.0 * poly[4]) * t2 + (20.0 * poly[5]) * t3;
	state_ddd = (6.0 * poly[3]) + (24.0 * poly[4]) * t + (60.0 * poly[5]) * t2;
	
	return (vector<double>){state, state_d, state_dd, state_ddd};
	
}
```

### 5. Cost functions

The absolute core of a path planning algorithm is the tuning of cost functions to ensure expected and safe behavior of the artificial driver. Only 5 cost functions are needed to safely and efficiently drive in the simulator environment.

First we need to always ensure that there is no collision. The `CostStepsToCollision()` method within the `Vehicle` class calculates a cost based on the number of steps before a collision. 50 steps have been identified to be a good value for an average normalized cost of 0.5. Less steps lead to higher cost and more steps lead to lower cost as shown in the first diagram further below.

```C
// determine collision cost
double Vehicle::CostStepsToCollision(Trajectory trajectory, vector<Vehicle> vehicles, const double &weight) {
	
	// define variables
	unsigned long collision_steps = 0;
	double cost_exp = 0.0;
	
	// initialize outputs
	double cost = ZERO_COST;
	
	// check for collision and adjust cost
	collision_steps = DetectCollision(trajectory, vehicles);
	
	// calculate cost
	cost_exp = exp((NO_HARMFUL_COLLISION_STEPS - collision_steps) / COST_STEPS_TO_COLLISION_SHAPE_FACTOR);
	cost = cost_exp / (cost_exp + 1);
	
	return cost;
	
}
```

Second even if there is no collision possible right now, our vehicle might follow the vehicle in front of us in very close distance with the exact same speed. Therefore, a collision is likely in the future and must be prevented by keeping a larger distance. The `CostSpaceAhead()` method within the `Vehicle` class calculates a cost based on the distance to the vehicle in front of us in the intended lane. The desired distance is calculated as distance travelled in 1.5 seconds at the current speed and set as an average normalized cost of 0.5. Shorter distances lead to higher cost and longer distances lead to lower cost.

```C
// determine whether there is enough space to the vehicle in front
double Vehicle::CostSpaceAhead(Map map, Trajectory trajectory, vector<Vehicle> vehicles, const double &weight) {
	
	// define variables
	vector<Vehicle> vehicles_ahead;
	unsigned int count = 0;
	Vehicle current_vehicle;
	double distance_to_current_vehicle = 0.0;
	double minimum_distance_ahead = std::numeric_limits<double>::max();
	Vehicle vehicle_ahead;
	double desired_distance = 0.0;
	
	// initialize outputs
	double cost = ZERO_COST;
	
	// get vehicles in front of own vehicle in intended lane
	vehicles_ahead = this->Ahead(map, vehicles, trajectory.Get_intended_lane());
	
	// determine vehicle and minimum distance directly in front of own vehicle
	...
	
	// calculate desired distance to vehicle in front of own vehicle in intended lane
	desired_distance = this->Get_v() * AHEAD_DISTANCE_TIME;
	
	// calculate cost
	cost = weight * (-(minimum_distance_ahead - desired_distance) / ((COST_SPACE_AHEAD_SHAPE_FACTOR * minimum_distance_ahead) + desired_distance));
	
	return cost;
	
}
```

Third we need ensure that there is always enough space on the left or right side of the own vehicle before making a lane change. The `CostSpaceInIntendedLane()` method within the `Vehicle` class calculates either zero or maximum normalized cost based on whether there is space or there is not.

```C
// determine whether there is enough space in the intended lane
double Vehicle::CostSpaceInIntendedLane(Trajectory trajectory, vector<Vehicle> vehicles, const double &weight) {
	
	// define variables
	vector<Vehicle> vehicles_ahead;
	vector<Vehicle> vehicles_behind;
	unsigned int count = 0;
	Vehicle current_vehicle;
	double distance_to_current_vehicle = 0.0;
	double minimum_distance_ahead = std::numeric_limits<double>::max();
	Vehicle vehicle_ahead;
	double minimum_distance_behind = std::numeric_limits<double>::max();
	Vehicle vehicle_behind;
	bool enough_space = false;
	
	// initialize outputs
	double cost = ZERO_COST;
	
	// get vehicles in front and behind of own vehicle in intended lane
	vehicles_ahead = this->Ahead(vehicles, trajectory.Get_intended_lane());
	vehicles_behind = this->Behind(vehicles, trajectory.Get_intended_lane());
	
	// determine vehicle and minimum distance directly in front of own vehicle
	...
	
	// determine vehicle and minimum distance directly behind of own vehicle
	...
	
	// determine space needed
	enough_space = ((minimum_distance_ahead >= (AHEAD_SPACE_FACTOR * vehicle_ahead.Get_length())) && (minimum_distance_behind >= (BEHIND_SPACE_FACTOR * vehicle_behind.Get_length())));
	
	// calculate cost
	if (enough_space) {
		
		cost = ZERO_COST;
		
	} else {
		
		cost = weight * MAX_NORMALIZED_COST;
		
	}
	
	return cost;
	
}
```

Fourth we need to ensure that we always pick the fastest feasible lane to advance as quickly as possible. The `CostSpeedInIntendedLane()` method within the `Vehicle` class calculates a cost based on the speed of the vehicle in the intended lane in front of our own vehicle. The cost is 0 at the maximum allowable speed and increases to 1 during a standstill as shown in the second diagram further below.

```C
// determine cost for speed in intended lane
double Vehicle::CostSpeedInIntendedLane(Trajectory trajectory, vector<Vehicle> vehicles, const double &weight) {
	
	// define variables
	vector<Vehicle> vehicles_ahead;
	unsigned int count = 0;
	Vehicle current_vehicle;
	double distance_to_current_vehicle = 0.0;
	double minimum_distance = std::numeric_limits<double>::max();
	Vehicle vehicle_ahead;
	double lane_speed = 0.0;
	
	// initialize outputs
	double cost = ZERO_COST;
	
	// get vehicles in front of own vehicle in intended lane
	vehicles_ahead = this->Ahead(vehicles, trajectory.Get_intended_lane());
	
	// determine vehicle directly in front of own vehicle
	...
	
	// get speed of intended lane
	if (minimum_distance > VEHICLE_AHEAD_WITHIN_DISTANCE) {
		
		lane_speed = MAX_SPEED;
		
	} else {
		
		lane_speed = min(vehicle_ahead.Get_v(), MAX_SPEED);
		
	}
	
	// calculate cost
	cost = weight * (-(lane_speed - MAX_SPEED) / ((COST_SPEED_IN_INTENDED_LANE_SHAPE_FACTOR * lane_speed) + MAX_SPEED));
	
	return cost;
	
}
```

Fifth we must not forget that while driving safe is key we also need to advance. The `CostTravelDistance()` method within the `Vehicle` class calculates a cost based on how far the trajectory reaches. The cost is 0 at the distance that you can achieve driving at the maximum allowable speed limit in the given time interval and increases to 1 during a standstill with no travel as shown in the third diagram further below.

```C
// determine cost for travel distance
double Vehicle::CostTravelDistance(Trajectory trajectory, const double &weight) {
	
	// define variables
	double travel_distance = 0.0;
	
	// initialize outputs
	double cost = ZERO_COST;
	
	// calculate travel distance
	travel_distance = trajectory.Get_s()[trajectory.Get_s().size() - 1] - this->Get_s();
	
	// calculate cost
	cost = weight * (-(travel_distance - MAX_TRAVEL_DISTANCE) / ((COST_TRAVEL_DISTANCE_SHAPE_FACTOR * travel_distance) + MAX_TRAVEL_DISTANCE));
	
	return cost;
	
}
```

<img src="docu_images/190119_StAn_Udacity_SDCND_PP_Cost_Function_Collision.jpg" width="32%"> <img src="docu_images/190119_StAn_Udacity_SDCND_PP_Cost_Function_Speed.jpg" width="32%"> <img src="docu_images/190119_StAn_Udacity_SDCND_PP_Cost_Function_Travel.jpg" width="32%">

It is important to note that the absolute cost is only relevant to balance the different cost functions amongst themselves. This is what the weights of the normalized cost functions are used for. For example it is much more important to avoid a collision than travelling a longer distance per time interval.

Tuning cost functions also requires to look at the relative cost difference between the individual trajectories when looking at a single cost function. Therefore, areas in the cost function with large changes lead to larger changes between trajectories with different input values to the cost function. For example a less aggressive driver that stays further away from other vehicles would need a `CostStepsToCollision()` method that has larger changes at lower number of steps before a collision (above diagram on the left). The current setting uses high, but also very flat cost at lower number of steps and hence leads to a more aggressive driving behavior.

### 6. Debugging environment

In order to debug the path planning program efficiently, several functions have been added to display the content of all the input and output variables of each relevant object method.

The debug options are controlled by the following constants within the `helper_functions.h()` file. If `bFILEOUTPUT` is `true`, the standard output is redirected into the file `out.txt` (inside the `build` folder). If `bDISPLAY` is `true`, more information about input and output variables is displayed to the standard output. There is a constant boolean for each relevant method to turn debugging of its inputs and outputs on or off. As vehicle and trajectory objects can have a lot of content, there are two global parameters that control whether the content of these objects is displayed or not (`bDISPLAY_VEHICLES`, `bDISPLAY_TRAJECTORIES`).

```C
// debug settings
const bool bFILEOUTPUT = true;
const string OUTPUT_FILENAME = "out.txt";
const bool bDISPLAY = true;
const bool bDISPLAY_DRIVER_PLANBEHAVIOR = true;
const bool bDISPLAY_DRIVER_SETVEHICLES = false;
const bool bDISPLAY_MAP_INIT = false;
const bool bDISPLAY_MAP_XY2FRENET = false;
const bool bDISPLAY_MAP_FRENET2XY = false;
const bool bDISPLAY_MAP_ASSIGNS = false;
const bool bDISPLAY_MAP_DELTAS = false;
const bool bDISPLAY_MAP_CLOSESTWAYPOINT = false;
const bool bDISPLAY_MAP_NEXTWAYPOINT = false;
const bool bDISPLAY_MAP_REFERENCES = false;
const bool bDISPLAY_VEHICLES = false;
const bool bDISPLAY_VEHICLE_UPDATE = false;
const bool bDISPLAY_VEHICLE_AHEAD = false;
const bool bDISPLAY_VEHICLE_BEHIND = false;
const bool bDISPLAY_VEHICLE_PREDICTTRAJECTORY = false;
const bool bDISPLAY_VEHICLE_GETLANED = false;
const bool bDISPLAY_VEHICLE_DETERMINELANE = false;
const bool bDISPLAY_VEHICLE_CHECKINSIDELANE = false;
const bool bDISPLAY_VEHICLE_DETECTCOLLISION = false;
const bool bDISPLAY_VEHICLE_COSTSTEPSTOCOLLISION = false;
const bool bDISPLAY_VEHICLE_COSTSPACEAHEAD = false;
const bool bDISPLAY_VEHICLE_COSTSPACEININTENDEDLANE = false;
const bool bDISPLAY_VEHICLE_COSTSPEEDININTENDEDLANE = false;
const bool bDISPLAY_VEHICLE_COSTTRAVELDISTANCE = false;
const bool bDISPLAY_VEHICLE_TRAJECTORYCOST = true;
const bool bDISPLAY_PATH_SET = false;
const bool bDISPLAY_TRAJECTORIES = false;
const bool bDISPLAY_TRAJECTORY_INIT = false;
const bool bDISPLAY_TRAJECTORY_START = false;
const bool bDISPLAY_TRAJECTORY_ADD = false;
const bool bDISPLAY_TRAJECTORY_ADDJERKMINIMIZINGTRAJECTORY = false;
const bool bDISPLAY_TRAJECTORY_GENERATE = false;
const bool bDISPLAY_TRAJECTORY_VALID = false;
const bool bDISPLAY_TRAJECTORY_REMOVEFIRSTSTEPS = false;
const bool bDISPLAY_TRAJECTORY_KEEPFIRSTSTEPS = false;
const bool bDISPLAY_STATE_INIT = true;
const bool bDISPLAY_STATE_SETBEHAVIOR = false;
const bool bDISPLAY_STATE_GETNEXTPOSSIBLEBEHAVIORS = true;
const bool bDISPLAY_STATE_GENERATETRAJECTORYFROMBEHAVIOR = false;
const string DISPLAY_PREFIX = "    ";
const unsigned int DISPLAY_COLUMN_WIDTH = 15;
```

The below functions inside the `helper_functions.cpp` file are used to convert the variable contents into a single string that can be displayed. Also, each object contains a `CreateString()` method to convert its content into a single string.

```C
string CreateDoubleVectorString(const vector<double> &double_vector);
string CreateDoubleVectorsString(const vector<vector<double>> &double_vectors);
string CreateUnsignedIntegerVectorString(const vector<unsigned int> &int_vector);
```

## 4. Execution

### 1. Commands to start the simulation

The program is compiled using the `.\build.sh` command. After this it can be started using the `.\run.sh` command. Once the program is running and listening on port 4567 the simulator can be started.

### 2. Simulation results

The vehicle starts very smooth from standstill without violating any of the jerk and acceleration criteria. It also stays below the speed limit at all times. During straight driving the trajectory follows the center of the lane.

<img src="docu_images/190119_StAn_Udacity_SDC_PP_start_small.gif" width="48%"> <img src="docu_images/190119_StAn_Udacity_SDC_PP_straight_01_small.gif" width="48%">

Passing other vehicles in traffic are the most exciting situations. The driver always picks the fastest lane and uses gaps between vehicles to change lanes and advance.

<img src="docu_images/190119_StAn_Udacity_SDC_PP_passing_01_small.gif" width="48%"> <img src="docu_images/190119_StAn_Udacity_SDC_PP_passing_02_small.gif" width="48%">

<img src="docu_images/190119_StAn_Udacity_SDC_PP_passing_03_small.gif" width="48%"> <img src="docu_images/190119_StAn_Udacity_SDC_PP_passing_05_small.gif" width="48%">

The debugging output of this run can be found in [./out.txt](./out.txt).

## 5. Discussion

Connecting to the simulator that executes a varying amount of time steps per iteration can be very challenging at the beginning. It is important to provide a first trajectory that gets the vehicle going. After this the executed number of time steps needs to be carefully subtracted from the intended trajectory. And finally a continuous smooth extension and update of the intended trajectory is key. I implemented an object based framework for the path planning program that manages this very well and is easily extendable.

The originally provided [Frenet](https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas) conversion routines are very poor. They look for the next waypoint and then convert from Frenet to [cartesian coordinates](https://en.wikipedia.org/wiki/Cartesian_coordinate_system) using a linear approach. When applied to full trajectories this can lead to jumps in the cartesian coordinates when switching from one waypoint to the next. You can smoothen the final trajectory in cartesian coordinates, but all previous calculations and validations of the trajectory will not be valid anymore. Therefore, I applied [Eddie Forson's solution](https://towardsdatascience.com/teaching-cars-to-drive-highway-path-planning-109c49f9f86c) to smoothen the conversion from Frenet to carthesian coordinates instead of smoothening the final trajectory.

![alt text][image3]

When the track widens in sharper corners, the simulator sometimes issues an "Outside of lane!" warning when being in the most outer lane. It actually doesn't look like the vehicle left the lane. My assumption is that the detection of the vehicle position inside the simulator is also based on the poor Frenet conversion and therefore issues the warning by mistake.

The parameters are set for a very aggressive driver that looks for small gaps between vehicles to advance as fast as possible. Sometimes the driver gets very close to the vehicle in front before being able to start a lane change to pass it. After starting the lane change, the own vehicle can be faster than the vehicle in front. In this case the cost for collision spikes and the driver immediately makes another lane change further to the side to avoid a collision.

<img src="docu_images/190119_StAn_Udacity_SDC_PP_passing_04_small.gif" width="48%">

At the end of the track the path planning program terminates with a [core dump](https://en.wikipedia.org/wiki/Core_dump), because several distances and velocities are calculated using longitudinal Frenet coordinates. These switch from the maximal value back to zero which leads to errors.

<img src="docu_images/190119_StAn_Udacity_SDC_PP_end_small.gif" width="48%">




























# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

