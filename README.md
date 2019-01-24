# Project: PID Controller

This project has been prepared by Andre Strobel.

The goal of this project is to tune the parameters of a PID controller to steer a vehicle around a test track in the best possible way.

The following table shows an overview of the most important files:

| File                          | Description                                                                                          |
|-------------------------------|------------------------------------------------------------------------------------------------------|
| README.md                     | This file                                                                                            |
| build.sh                      | Script to build the PID controller executable                                                        |
| run.sh                        | Script to run the PID controller executable                                                          |
| src/main.cpp                  | Source code of the main function of the PID controller program                                       |
| src/json.hpp                  | Source code for reading and writing [JAVAScript Object Notation](https://en.wikipedia.org/wiki/JSON) |
| src/PID.{h, cpp}              | Source code of the PID controller object                                                             |

---

## Content

1. Tool chain setup
    1. Gcc, Cmake, Make and uWebSocketIO
    1. Udacity Simulator
1. Data objects and structures
    1. Simulator input and output
    1. PID class
1. PID controller implementation
    1. PID controller
    1. Twiddle algorithm
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

The path planning program connects to the [Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases) version [Term 2 Simulator v1.45](https://github.com/udacity/self-driving-car-sim/releases/tag/v1.45) via [uWebSocketIO](https://github.com/uWebSockets/uWebSockets). The simulator is available for Linux, Mac and Windows.

## 2. Data objects and structures

### 1. Simulator input and output

The simulator sends the cross track error `cte` to the `main` function. In return the `main` function sends the steering angle `steering_angle` to the simulator.

### 2. PID class

The `PID` class contains all the functions to implement a PID controller and the Twiddle algorithm to tune its control parameters. The following list contains the most important methods of this class:

| Method | Description |
|-|-|
| `UpdateError()` | Update the proportional, integral and differential error values as well as twiddle the controller gains. |
| `TotalError()` | Calculate the next steering angle based on the proportional, integral and differential gains and errors. |

The behavior of this class is controlled with the below mostly self explaining constants. The parameter `TWIDDLE_FACTOR` defines how small the initial controller gain changes in the Twiddle algorithm are in relation to their default values. The parameter `NUM_CHANGE_STATES` defines the number of different changes that are possible during the Twiddle algorithm (increase Kp, decrease Kp, increase Ki, decrease Ki, increase Kd, decrease Kd).

```C
// define constants
const bool TWIDDLE = true;
const unsigned int NUM_CONVERGED_STEPS = 100;
const unsigned int NUM_LOOP_STEPS = 2000;
const double DEFAULT_KP = 0.2;
const double DEFAULT_KI = 0.0001;
const double DEFAULT_KD = 3.0;
const double TWIDDLE_FACTOR = 10.0;
const unsigned int NUM_CHANGE_STATES = 6;
```

## 3. PID controller implementation

### 1. PID controller

XXX

### 2. Twiddle algorithm

XXX

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

