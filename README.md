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

The behavior of this class is controlled with the below mostly self explaining constants. The number of steps per full loop `NUM_LOOP_STEPS` must be adjusted to the performance of the computer system. The faster the computer system the more steps are used for each loop, because information is exchanged more often between the simulator and the controller. The parameter `TWIDDLE_FACTOR` defines how small the initial controller gain changes in the Twiddle algorithm are in relation to their default values. The parameter `NUM_CHANGE_STATES` defines the number of different changes that are possible during the Twiddle algorithm (increase Kp, decrease Kp, increase Ki, decrease Ki, increase Kd, decrease Kd).

```C
// define constants
const bool TWIDDLE = true;
const unsigned int NUM_CONVERGED_STEPS = 100;
const unsigned int NUM_LOOP_STEPS = 1000;
const double DEFAULT_KP = 0.2;
const double DEFAULT_KI = 0.0001;
const double DEFAULT_KD = 3.0;
const double TWIDDLE_FACTOR = 10.0;
const unsigned int NUM_CHANGE_STATES = 6;
```

## 3. PID controller implementation

### 1. PID controller

The main control functions of the PID controller are implemented with the following code:

```C
// calculate error terms
d_error = cte - p_error;
p_error = cte;
i_error += cte;

...

// calculate steering angle
steer_value = -((Kp * p_error) + (Kd * d_error) + (Ki * i_error));

// limit steering angle
steer_value = max(min(steer_value, 1.0), -1.0);
```

### 2. Twiddle algorithm

Before we record the cross track error `cte` over a full loop of the track as `error`, we need to set the new control gains and then wait a few steps until the control behavior settles on these parameters. The full loop error `error` is calculated as sum of all squared cross track errors `cte` over roughly one loop of the track. We use the squared cross track errors, because individual positive and negative errors shouldn't cancel each other out and larger deviations should be avoided at all if possible.

After each full loop the Twiddle algorithm varies one of the three PID controller parameters at a time. And for each gain it first tries to increase the value. If this leads to an improvement, the algorithm remembers that larger gain changes might make sense and then jumps to the next control parameter. If it doesn't lead to an improvement, it tries the decrease the gain. If this leads to an improvement, the algorithm remembers that larger gain changes might make sense and then jumps to the next control parameter. If not, the algorithm reverts to the original gain value and remembers that smaller gain changes might make sense and then jumps to the next control parameter.

Cycling through all three PID controller parameters with the option to either increase or decrease the value leads to 6 possible changes as shown in the following:

```C
// determine full loop status
full_loop_steps = fmod((full_loop_steps + 1), (NUM_CONVERGED_STEPS + NUM_LOOP_STEPS));

// check whether controller is converged
if (!is_converged) {
	
	// increase counter to converged state
	converge_steps += 1;
	
	// check whether necessary conversion steps are reached
	if (converge_steps >= NUM_CONVERGED_STEPS) {
		
		// set converged status to true
		is_converged = true;
		
	}
	
} else {
	
	// increase error
	error += pow(cte, 2);
	
	// check whether we restart a cycle/loop and need to twiddle
	if (full_loop_steps == 0) {
		
		// remember current change
		current_change = change;
		current_Kp = Kp;
		current_Ki = Ki;
		current_Kd = Kd;
		
		switch (change) {
			
			case 0: // increase Kp
				
				// check whether last error is an improvement
				if (error < best_error) {
					
					// remember best error, increase changing controller parameter and move to next controller parameter
					best_error = error;
					dKp *= 1.1;
					change = fmod((change + 2), NUM_CHANGE_STATES);
					
					// adjust controller parameter for change == 2
					Ki += dKi;
					
					} else {
					
					// move to next change
					change = fmod((change + 1), NUM_CHANGE_STATES);
					
					// adjust controller parameter for change == 1
					Kp -= 2 * dKp;
					
				}
				
				break; // switch
				
			case 1: // decrease Kp
				
				// check whether last error is an improvement
				if (error < best_error) {
					
					// remember best error and increase changing controller parameter
					best_error = error;
					dKp *= 1.1;
					
				} else {
					
					// revert and reduce changing controller parameter
					Kp += dKp;
					dKp *= 0.9;
					
				}
				
				// move to next change
				change = fmod((change + 1), NUM_CHANGE_STATES);
				
				// adjust controller parameter for change == 2
				Ki += dKi;
				
				break; // switch
				
			case 2: // increase Ki
				
				// check whether last error is an improvement
				if (error < best_error) {
					
					// remember best error, increase changing controller parameter and move to next controller parameter
					best_error = error;
					dKi *= 1.1;
					change = fmod((change + 2), NUM_CHANGE_STATES);
					
					// adjust controller parameter for change == 4
					Kd += dKd;
					
				} else {
					
					// move to next change
					change = fmod((change + 1), NUM_CHANGE_STATES);
					
					// adjust controller parameter for change == 3
					Ki -= 2 * dKi;
					
				}
				
				break; // switch
				
			case 3: // decrease Ki
				
				// check whether last error is an improvement
				if (error < best_error) {
					
					// remember best error and increase changing controller parameter
					best_error = error;
					dKi *= 1.1;
					
				} else {
					
					// revert and reduce changing controller parameter
					Ki += dKi;
					dKi *= 0.9;
					
				}
				
				// move to next change
				change = fmod((change + 1), NUM_CHANGE_STATES);
				
				// adjust controller parameter for change == 4
				Kd += dKd;
				
				break; // switch
				
			case 4: // increase Kd
				
				// check whether last error is an improvement
				if (error < best_error) {
					
					// remember best error, increase changing controller parameter and move to next controller parameter
					best_error = error;
					dKd *= 1.1;
					change = fmod((change + 2), NUM_CHANGE_STATES);
					
					// adjust controller parameter for change == 0
					Kp += dKp;
					
				} else {
					
					// move to next change
					change = fmod((change + 1), NUM_CHANGE_STATES);
					
					// adjust controller parameter for change == 5
					Kd -= 2 * dKd;
					
				}
				
				break; // switch
				
			case 5: // decrease Kd
				
				// check whether last error is an improvement
				if (error < best_error) {
					
					// remember best error and increase changing controller parameter
					best_error = error;
					dKd *= 1.1;
					
				} else {
					
					// revert and reduce changing controller parameter
					Kd += dKd;
					dKd *= 0.9;
					
				}
				
				// move to next change
				change = fmod((change + 1), NUM_CHANGE_STATES);
				
				// adjust controller parameter for change == 0
				Kp += dKp;
				
				break; // switch
				
		}
		
		// prepare next twiddle cycle/loop
		error = 0.0;
		is_converged = false;
		
		// display status
		cout << setw(DISPLAY_COLUMN_WIDTH) << setfill(' ') << "Current change: " << current_change << " Current Kp: " << current_Kp << " Current Ki: " << current_Ki << " Current Kd: " << current_Kd << " Best error: " << best_error << " Next change: " << change << " Next Kp: " << Kp << " Next Ki: " << Ki << " Next Kd: " << Kd << endl;
		
	}
	
}
```

## 4. Execution

### 1. Commands to start the simulation

The program is compiled using the `.\build.sh` command. After this it can be started using the `.\run.sh` command. Once the program is running and listening on port 4567 the simulator can be started.

### 2. Simulation results

First I needed to manually tune the controller parameters to have the car successfully navigate the track without leaving it or getting stuck. I settled on the above mentioned default parameters `DEFAULT_KP = 0.2`, `DEFAULT_KI = 0.0001` and `DEFAULT_KD = 3.0`.

Then I used the Twiddle algorithm to further optimize the controller parameters. These are the steps the Twiddle algorithm took:

```
Current change: X Current Kp: X Current Ki: X Current Kd: X Best error: 1.79769e+308 Next change: 0 Next Kp: 0.2 Next Ki: 0.0001 Next Kd: 3
Current change: 0 Current Kp: 0.2 Current Ki: 0.0001 Current Kd: 3 Best error: 382.24 Next change: 2 Next Kp: 0.22 Next Ki: 0.0001 Next Kd: 3
Current change: 2 Current Kp: 0.22 Current Ki: 0.0001 Current Kd: 3 Best error: 298.918 Next change: 4 Next Kp: 0.22 Next Ki: 0.00011 Next Kd: 3
Current change: 4 Current Kp: 0.22 Current Ki: 0.00011 Current Kd: 3 Best error: 298.918 Next change: 5 Next Kp: 0.22 Next Ki: 0.00011 Next Kd: 3.3
Current change: 5 Current Kp: 0.22 Current Ki: 0.00011 Current Kd: 3.3 Best error: 298.918 Next change: 0 Next Kp: 0.22 Next Ki: 0.00011 Next Kd: 3
Current change: 0 Current Kp: 0.22 Current Ki: 0.00011 Current Kd: 3 Best error: 298.918 Next change: 1 Next Kp: 0.242 Next Ki: 0.00011 Next Kd: 3
```

As the starting conditions for each full loop are not absolutely the same, the accumulated full loop cross track error `error` is noisy. Therefore, the Twiddle algorithm cannot really converge. For example if a lucky loop didn't have a large deviation, it might be forever considered to have the best error although other parameter settings would actually be better.

Also, some parameter settings lead to the vehicle leaving the track or getting stuck. When this occured, I restartet the simulation and the Twiddle algorithm continued. As leaving the track or getting stuck leads to a large cross track error `cte` and hence large full loop error `error`, these parameter settings are automatically excluded from being considered as best solution.

The following animation shows a short section of the track with the optimal controller parameters `KP = 0.2`, `KI = 0.0001` and `KD = 3.0`.

<img src="docu_images/190119_StAn_Udacity_SDC_PP_start_small.gif" width="48%"> <img src="docu_images/190119_StAn_Udacity_SDC_PP_straight_01_small.gif" width="48%">

## 5. Discussion

I only implemented the tuning of the PID controller parameters for a fixed throttle/speed value. In the example above the throttle value has been selected as 30%. The throttle influences the speed range and the speed significantly influences the lateral dynamics of the vehicle. Therefore, the PID controller parameters should be tuned for different throttle settings. These values should be put into look-up tables that define all gains depending on the throttle value.

While having higher steering angles when going through curves the speed drops with a constant throttle setting. Therefore, an additional PID speed controller can help keep the speed at a constant value at all times. The parameters of this PID speed controller should be twiddled after the optimal parameters for the PID steering controller have been determined.

Finally, the PID steering controller can further be improved by using the output steering angle signal `steering_angle` from the [Behavioral Cloning](https://github.com/CyberAMS/CarND-Behavioral-Cloning-P3/) project as [feedforward control](https://en.wikipedia.org/wiki/Feed_forward_(control)):

```C
// calculate steering angle
steer_value = steering_angle - ((Kp * p_error) + (Kd * d_error) + (Ki * i_error));
```

In this case the steering angle `steering_angle` is estimated based on the trained behavior. The PID steering controller only needs to compensate for the errors in this estimation and not take care of the complete steering angle signal `steer_value`. Therefore, the Twiddle tuning will result in different PID control parameters which are typically much smaller than without the feedforward control. As a result the control behavior will be much smoother.