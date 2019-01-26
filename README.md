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

[image1]: ./docu_images/190126a_StAn_Udacity_SDC_PID_start_small_condensed2.gif
[image2]: ./docu_images/190126b_StAn_Udacity_SDC_PID_optimal_small_condensed2.gif

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

After each full loop the Twiddle algorithm varies one of the three PID controller parameters at a time. And for each gain it first tries to increase the value. If this leads to an improvement, the algorithm remembers that larger gain changes might make sense and then jumps to the next control parameter. If it doesn't lead to an improvement, it tries the decrease the gain. If this leads to an improvement, the algorithm remembers that larger gain changes might make sense and then jumps to the next control parameter. If no change led to an improvement, the algorithm reverts to the original gain value and remembers that smaller gain changes might make sense and then jumps to the next control parameter.

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
Current change: X Current Kp: X        Current Ki: X        Current Kd: X        Current error: X         Best error: 1.798e+308 Next change: 0 Next Kp: 0.200000 Next Ki: 0.000100 Next Kd: 3.000000
Current change: 0 Current Kp: 0.200000 Current Ki: 0.000100 Current Kd: 3.000000 Current error:   366.267 Best error: 366.267000 Next change: 2 Next Kp: 0.200000 Next Ki: 0.000110 Next Kd: 3.000000
Current change: 2 Current Kp: 0.200000 Current Ki: 0.000110 Current Kd: 3.000000 Current error:   317.012 Best error: 317.012000 Next change: 4 Next Kp: 0.200000 Next Ki: 0.000110 Next Kd: 3.300000
Current change: 4 Current Kp: 0.200000 Current Ki: 0.000110 Current Kd: 3.300000 Current error:   432.507 Best error: 317.012000 Next change: 5 Next Kp: 0.200000 Next Ki: 0.000110 Next Kd: 2.700000
Current change: 5 Current Kp: 0.200000 Current Ki: 0.000110 Current Kd: 2.700000 Current error:  1388.120 Best error: 317.012000 Next change: 0 Next Kp: 0.222000 Next Ki: 0.000110 Next Kd: 3.000000
Current change: 0 Current Kp: 0.222000 Current Ki: 0.000110 Current Kd: 3.000000 Current error:   389.198 Best error: 317.012000 Next change: 1 Next Kp: 0.178000 Next Ki: 0.000110 Next Kd: 3.000000
Current change: 1 Current Kp: 0.178000 Current Ki: 0.000110 Current Kd: 3.000000 Current error:   507.279 Best error: 317.012000 Next change: 2 Next Kp: 0.200000 Next Ki: 0.000121 Next Kd: 3.000000
Current change: 2 Current Kp: 0.200000 Current Ki: 0.000121 Current Kd: 3.000000 Current error:   407.556 Best error: 317.012000 Next change: 3 Next Kp: 0.200000 Next Ki: 0.000099 Next Kd: 3.000000
Current change: 3 Current Kp: 0.200000 Current Ki: 0.000099 Current Kd: 3.000000 Current error:   442.980 Best error: 317.012000 Next change: 4 Next Kp: 0.200000 Next Ki: 0.000110 Next Kd: 3.270000
Current change: 4 Current Kp: 0.200000 Current Ki: 0.000110 Current Kd: 3.270000 Current error:   366.363 Best error: 317.012000 Next change: 5 Next Kp: 0.200000 Next Ki: 0.000110 Next Kd: 2.730000
Current change: 5 Current Kp: 0.200000 Current Ki: 0.000110 Current Kd: 2.730000 Current error:   287.476 Best error: 287.476000 Next change: 0 Next Kp: 0.219800 Next Ki: 0.000110 Next Kd: 2.730000
Current change: 0 Current Kp: 0.219800 Current Ki: 0.000110 Current Kd: 2.730000 Current error:   265.436 Best error: 265.436000 Next change: 2 Next Kp: 0.219800 Next Ki: 0.000120 Next Kd: 2.730000
Current change: 2 Current Kp: 0.219800 Current Ki: 0.000120 Current Kd: 2.730000 Current error:   310.098 Best error: 265.436000 Next change: 3 Next Kp: 0.219800 Next Ki: 0.000100 Next Kd: 2.730000
Current change: 3 Current Kp: 0.219800 Current Ki: 0.000100 Current Kd: 2.730000 Current error:   713.812 Best error: 265.436000 Next change: 4 Next Kp: 0.219800 Next Ki: 0.000110 Next Kd: 3.027000
Current change: 4 Current Kp: 0.219800 Current Ki: 0.000110 Current Kd: 3.027000 Current error: 16418.800 Best error: 265.436000 Next change: 5 Next Kp: 0.219800 Next Ki: 0.000110 Next Kd: 2.433000
Current change: 5 Current Kp: 0.219800 Current Ki: 0.000110 Current Kd: 2.433000 Current error:   885.307 Best error: 265.436000 Next change: 0 Next Kp: 0.241580 Next Ki: 0.000110 Next Kd: 2.730000
Current change: 0 Current Kp: 0.241580 Current Ki: 0.000110 Current Kd: 2.730000 Current error:  1020.150 Best error: 265.436000 Next change: 1 Next Kp: 0.198020 Next Ki: 0.000110 Next Kd: 2.730000
Current change: 1 Current Kp: 0.198020 Current Ki: 0.000110 Current Kd: 2.730000 Current error:   364.262 Best error: 265.436000 Next change: 2 Next Kp: 0.219800 Next Ki: 0.000119 Next Kd: 2.730000
Current change: 2 Current Kp: 0.219800 Current Ki: 0.000119 Current Kd: 2.730000 Current error:   320.988 Best error: 265.436000 Next change: 3 Next Kp: 0.219800 Next Ki: 0.000101 Next Kd: 2.730000
Current change: 3 Current Kp: 0.219800 Current Ki: 0.000101 Current Kd: 2.730000 Current error:   289.336 Best error: 265.436000 Next change: 4 Next Kp: 0.219800 Next Ki: 0.000110 Next Kd: 2.997300
Current change: 4 Current Kp: 0.219800 Current Ki: 0.000110 Current Kd: 2.997300 Current error:   456.750 Best error: 265.436000 Next change: 5 Next Kp: 0.219800 Next Ki: 0.000110 Next Kd: 2.462700
Current change: 5 Current Kp: 0.219800 Current Ki: 0.000110 Current Kd: 2.462700 Current error:   425.623 Best error: 265.436000 Next change: 0 Next Kp: 0.239402 Next Ki: 0.000110 Next Kd: 2.730000
Current change: 0 Current Kp: 0.239402 Current Ki: 0.000110 Current Kd: 2.730000 Current error:   484.228 Best error: 265.436000 Next change: 1 Next Kp: 0.200198 Next Ki: 0.000110 Next Kd: 2.730000
Current change: 1 Current Kp: 0.200198 Current Ki: 0.000110 Current Kd: 2.730000 Current error:   290.651 Best error: 265.436000 Next change: 2 Next Kp: 0.219800 Next Ki: 0.000118 Next Kd: 2.730000
Current change: 2 Current Kp: 0.219800 Current Ki: 0.000118 Current Kd: 2.730000 Current error:   276.839 Best error: 265.436000 Next change: 3 Next Kp: 0.219800 Next Ki: 0.000102 Next Kd: 2.730000
Current change: 3 Current Kp: 0.219800 Current Ki: 0.000102 Current Kd: 2.730000 Current error:   318.784 Best error: 265.436000 Next change: 4 Next Kp: 0.219800 Next Ki: 0.000110 Next Kd: 2.970570
Current change: 4 Current Kp: 0.219800 Current Ki: 0.000110 Current Kd: 2.970570 Current error:   359.974 Best error: 265.436000 Next change: 5 Next Kp: 0.219800 Next Ki: 0.000110 Next Kd: 2.489430
Current change: 5 Current Kp: 0.219800 Current Ki: 0.000110 Current Kd: 2.489430 Current error:   363.069 Best error: 265.436000 Next change: 0 Next Kp: 0.237442 Next Ki: 0.000110 Next Kd: 2.730000
Current change: 0 Current Kp: 0.237442 Current Ki: 0.000110 Current Kd: 2.730000 Current error:   283.794 Best error: 265.436000 Next change: 1 Next Kp: 0.202158 Next Ki: 0.000110 Next Kd: 2.730000
Current change: 1 Current Kp: 0.202158 Current Ki: 0.000110 Current Kd: 2.730000 Current error:   301.927 Best error: 265.436000 Next change: 2 Next Kp: 0.219800 Next Ki: 0.000117 Next Kd: 2.730000
Current change: 2 Current Kp: 0.219800 Current Ki: 0.000117 Current Kd: 2.730000 Current error:   363.878 Best error: 265.436000 Next change: 3 Next Kp: 0.219800 Next Ki: 0.000103 Next Kd: 2.730000
Current change: 3 Current Kp: 0.219800 Current Ki: 0.000103 Current Kd: 2.730000 Current error:   275.332 Best error: 265.436000 Next change: 4 Next Kp: 0.219800 Next Ki: 0.000110 Next Kd: 2.946510
Current change: 4 Current Kp: 0.219800 Current Ki: 0.000110 Current Kd: 2.946510 Current error:   299.762 Best error: 265.436000 Next change: 5 Next Kp: 0.219800 Next Ki: 0.000110 Next Kd: 2.513490
Current change: 5 Current Kp: 0.219800 Current Ki: 0.000110 Current Kd: 2.513490 Current error:   387.683 Best error: 265.436000 Next change: 0 Next Kp: 0.235678 Next Ki: 0.000110 Next Kd: 2.730000
Current change: 0 Current Kp: 0.235678 Current Ki: 0.000110 Current Kd: 2.730000 Current error:   256.504 Best error: 256.504000 Next change: 2 Next Kp: 0.235678 Next Ki: 0.000116 Next Kd: 2.730000
Current change: 2 Current Kp: 0.235678 Current Ki: 0.000116 Current Kd: 2.730000 Current error:   262.096 Best error: 256.504000 Next change: 3 Next Kp: 0.235678 Next Ki: 0.000104 Next Kd: 2.730000
Current change: 3 Current Kp: 0.235678 Current Ki: 0.000104 Current Kd: 2.730000 Current error:   238.830 Best error: 238.830000 Next change: 4 Next Kp: 0.235678 Next Ki: 0.000104 Next Kd: 2.924860
Current change: 4 Current Kp: 0.235678 Current Ki: 0.000104 Current Kd: 2.924860 Current error:   256.220 Best error: 238.830000 Next change: 5 Next Kp: 0.235678 Next Ki: 0.000104 Next Kd: 2.535140
Current change: 5 Current Kp: 0.235678 Current Ki: 0.000104 Current Kd: 2.535140 Current error:   323.443 Best error: 238.830000 Next change: 0 Next Kp: 0.253143 Next Ki: 0.000104 Next Kd: 2.730000
Current change: 0 Current Kp: 0.253143 Current Ki: 0.000104 Current Kd: 2.730000 Current error:  1380.610 Best error: 238.830000 Next change: 1 Next Kp: 0.218212 Next Ki: 0.000104 Next Kd: 2.730000
Current change: 1 Current Kp: 0.218212 Current Ki: 0.000104 Current Kd: 2.730000 Current error:   336.034 Best error: 238.830000 Next change: 2 Next Kp: 0.235678 Next Ki: 0.000111 Next Kd: 2.730000
Current change: 2 Current Kp: 0.235678 Current Ki: 0.000111 Current Kd: 2.730000 Current error:   248.467 Best error: 238.830000 Next change: 3 Next Kp: 0.235678 Next Ki: 0.000096 Next Kd: 2.730000
Current change: 3 Current Kp: 0.235678 Current Ki: 0.000096 Current Kd: 2.730000 Current error:   240.329 Best error: 238.830000 Next change: 4 Next Kp: 0.235678 Next Ki: 0.000104 Next Kd: 2.905380
Current change: 4 Current Kp: 0.235678 Current Ki: 0.000104 Current Kd: 2.905380 Current error:   257.675 Best error: 238.830000 Next change: 5 Next Kp: 0.235678 Next Ki: 0.000104 Next Kd: 2.554620
Current change: 5 Current Kp: 0.235678 Current Ki: 0.000104 Current Kd: 2.554620 Current error:   351.012 Best error: 238.830000 Next change: 0 Next Kp: 0.251396 Next Ki: 0.000104 Next Kd: 2.730000
Current change: 0 Current Kp: 0.251396 Current Ki: 0.000104 Current Kd: 2.730000 Current error:   335.588 Best error: 238.830000 Next change: 1 Next Kp: 0.219959 Next Ki: 0.000104 Next Kd: 2.730000
Current change: 1 Current Kp: 0.219959 Current Ki: 0.000104 Current Kd: 2.730000 Current error:   344.951 Best error: 238.830000 Next change: 2 Next Kp: 0.235678 Next Ki: 0.000110 Next Kd: 2.730000
Current change: 2 Current Kp: 0.235678 Current Ki: 0.000110 Current Kd: 2.730000 Current error:   470.976 Best error: 238.830000 Next change: 3 Next Kp: 0.235678 Next Ki: 0.000097 Next Kd: 2.730000
Current change: 3 Current Kp: 0.235678 Current Ki: 0.000097 Current Kd: 2.730000 Current error:   277.018 Best error: 238.830000 Next change: 4 Next Kp: 0.235678 Next Ki: 0.000104 Next Kd: 2.887840
Current change: 4 Current Kp: 0.235678 Current Ki: 0.000104 Current Kd: 2.887840 Current error:   248.468 Best error: 238.830000 Next change: 5 Next Kp: 0.235678 Next Ki: 0.000104 Next Kd: 2.572160
Current change: 5 Current Kp: 0.235678 Current Ki: 0.000104 Current Kd: 2.572160 Current error:   371.574 Best error: 238.830000 Next change: 0 Next Kp: 0.249825 Next Ki: 0.000104 Next Kd: 2.730000
Current change: 0 Current Kp: 0.249825 Current Ki: 0.000104 Current Kd: 2.730000 Current error:   333.975 Best error: 238.830000 Next change: 1 Next Kp: 0.221531 Next Ki: 0.000104 Next Kd: 2.730000
Current change: 1 Current Kp: 0.221531 Current Ki: 0.000104 Current Kd: 2.730000 Current error:   368.330 Best error: 238.830000 Next change: 2 Next Kp: 0.235678 Next Ki: 0.000109 Next Kd: 2.730000
Current change: 2 Current Kp: 0.235678 Current Ki: 0.000109 Current Kd: 2.730000 Current error:   265.825 Best error: 238.830000 Next change: 3 Next Kp: 0.235678 Next Ki: 0.000098 Next Kd: 2.730000
Current change: 3 Current Kp: 0.235678 Current Ki: 0.000098 Current Kd: 2.730000 Current error:   254.431 Best error: 238.830000 Next change: 4 Next Kp: 0.235678 Next Ki: 0.000104 Next Kd: 2.872050
Current change: 4 Current Kp: 0.235678 Current Ki: 0.000104 Current Kd: 2.872050 Current error:   393.930 Best error: 238.830000 Next change: 5 Next Kp: 0.235678 Next Ki: 0.000104 Next Kd: 2.587950
Current change: 5 Current Kp: 0.235678 Current Ki: 0.000104 Current Kd: 2.587950 Current error:   282.995 Best error: 238.830000 Next change: 0 Next Kp: 0.248410 Next Ki: 0.000104 Next Kd: 2.730000
Current change: 0 Current Kp: 0.248410 Current Ki: 0.000104 Current Kd: 2.730000 Current error:   332.991 Best error: 238.830000 Next change: 1 Next Kp: 0.222945 Next Ki: 0.000104 Next Kd: 2.730000
Current change: 1 Current Kp: 0.222945 Current Ki: 0.000104 Current Kd: 2.730000 Current error:   329.923 Best error: 238.830000 Next change: 2 Next Kp: 0.235678 Next Ki: 0.000109 Next Kd: 2.730000
Current change: 2 Current Kp: 0.235678 Current Ki: 0.000109 Current Kd: 2.730000 Current error:   339.263 Best error: 238.830000 Next change: 3 Next Kp: 0.235678 Next Ki: 0.000098 Next Kd: 2.730000
Current change: 3 Current Kp: 0.235678 Current Ki: 0.000098 Current Kd: 2.730000 Current error:   251.548 Best error: 238.830000 Next change: 4 Next Kp: 0.235678 Next Ki: 0.000104 Next Kd: 2.857850
Current change: 4 Current Kp: 0.235678 Current Ki: 0.000104 Current Kd: 2.857850 Current error:   258.474 Best error: 238.830000 Next change: 5 Next Kp: 0.235678 Next Ki: 0.000104 Next Kd: 2.602150
Current change: 5 Current Kp: 0.235678 Current Ki: 0.000104 Current Kd: 2.602150 Current error:   248.337 Best error: 238.830000 Next change: 0 Next Kp: 0.247137 Next Ki: 0.000104 Next Kd: 2.730000
Current change: 0 Current Kp: 0.247137 Current Ki: 0.000104 Current Kd: 2.730000 Current error:   237.664 Best error: 237.664000 Next change: 2 Next Kp: 0.247137 Next Ki: 0.000108 Next Kd: 2.730000
Current change: 2 Current Kp: 0.247137 Current Ki: 0.000108 Current Kd: 2.730000 Current error:   540.843 Best error: 237.664000 Next change: 3 Next Kp: 0.247137 Next Ki: 0.000099 Next Kd: 2.730000
Current change: 3 Current Kp: 0.247137 Current Ki: 0.000099 Current Kd: 2.730000 Current error:   348.200 Best error: 237.664000 Next change: 4 Next Kp: 0.247137 Next Ki: 0.000104 Next Kd: 2.845060
Current change: 4 Current Kp: 0.247137 Current Ki: 0.000104 Current Kd: 2.845060 Current error:   271.860 Best error: 237.664000 Next change: 5 Next Kp: 0.247137 Next Ki: 0.000104 Next Kd: 2.614940
Current change: 5 Current Kp: 0.247137 Current Ki: 0.000104 Current Kd: 2.614940 Current error:   264.601 Best error: 237.664000 Next change: 0 Next Kp: 0.259742 Next Ki: 0.000104 Next Kd: 2.730000
Current change: 0 Current Kp: 0.259742 Current Ki: 0.000104 Current Kd: 2.730000 Current error:   214.911 Best error: 214.911000 Next change: 2 Next Kp: 0.259742 Next Ki: 0.000108 Next Kd: 2.730000
Current change: 2 Current Kp: 0.259742 Current Ki: 0.000108 Current Kd: 2.730000 Current error:   259.811 Best error: 214.911000 Next change: 3 Next Kp: 0.259742 Next Ki: 0.000099 Next Kd: 2.730000
Current change: 3 Current Kp: 0.259742 Current Ki: 0.000099 Current Kd: 2.730000 Current error:   326.024 Best error: 214.911000 Next change: 4 Next Kp: 0.259742 Next Ki: 0.000104 Next Kd: 2.833560
Current change: 4 Current Kp: 0.259742 Current Ki: 0.000104 Current Kd: 2.833560 Current error:   326.538 Best error: 214.911000 Next change: 5 Next Kp: 0.259742 Next Ki: 0.000104 Next Kd: 2.626440
Current change: 5 Current Kp: 0.259742 Current Ki: 0.000104 Current Kd: 2.626440 Current error:   467.431 Best error: 214.911000 Next change: 0 Next Kp: 0.273607 Next Ki: 0.000104 Next Kd: 2.730000
Current change: 0 Current Kp: 0.273607 Current Ki: 0.000104 Current Kd: 2.730000 Current error:   251.693 Best error: 214.911000 Next change: 1 Next Kp: 0.245876 Next Ki: 0.000104 Next Kd: 2.730000
Current change: 1 Current Kp: 0.245876 Current Ki: 0.000104 Current Kd: 2.730000 Current error:   263.893 Best error: 214.911000 Next change: 2 Next Kp: 0.259742 Next Ki: 0.000107 Next Kd: 2.730000
Current change: 2 Current Kp: 0.259742 Current Ki: 0.000107 Current Kd: 2.730000 Current error:   448.497 Best error: 214.911000 Next change: 3 Next Kp: 0.259742 Next Ki: 0.000100 Next Kd: 2.730000
Current change: 3 Current Kp: 0.259742 Current Ki: 0.000100 Current Kd: 2.730000 Current error:   234.383 Best error: 214.911000 Next change: 4 Next Kp: 0.259742 Next Ki: 0.000104 Next Kd: 2.823200
Current change: 4 Current Kp: 0.259742 Current Ki: 0.000104 Current Kd: 2.823200 Current error:   273.311 Best error: 214.911000 Next change: 5 Next Kp: 0.259742 Next Ki: 0.000104 Next Kd: 2.636800
Current change: 5 Current Kp: 0.259742 Current Ki: 0.000104 Current Kd: 2.636800 Current error:   458.652 Best error: 214.911000 Next change: 0 Next Kp: 0.272220 Next Ki: 0.000104 Next Kd: 2.730000
Current change: 0 Current Kp: 0.272220 Current Ki: 0.000104 Current Kd: 2.730000 Current error:   250.279 Best error: 214.911000 Next change: 1 Next Kp: 0.247263 Next Ki: 0.000104 Next Kd: 2.730000
Current change: 1 Current Kp: 0.247263 Current Ki: 0.000104 Current Kd: 2.730000 Current error:   670.927 Best error: 214.911000 Next change: 2 Next Kp: 0.259742 Next Ki: 0.000107 Next Kd: 2.730000
Current change: 2 Current Kp: 0.259742 Current Ki: 0.000107 Current Kd: 2.730000 Current error:   224.333 Best error: 214.911000 Next change: 3 Next Kp: 0.259742 Next Ki: 0.000100 Next Kd: 2.730000
Current change: 3 Current Kp: 0.259742 Current Ki: 0.000100 Current Kd: 2.730000 Current error:   397.589 Best error: 214.911000 Next change: 4 Next Kp: 0.259742 Next Ki: 0.000104 Next Kd: 2.813880
Current change: 4 Current Kp: 0.259742 Current Ki: 0.000104 Current Kd: 2.813880 Current error:   334.954 Best error: 214.911000 Next change: 5 Next Kp: 0.259742 Next Ki: 0.000104 Next Kd: 2.646120
Current change: 5 Current Kp: 0.259742 Current Ki: 0.000104 Current Kd: 2.646120 Current error:   206.139 Best error: 206.139000 Next change: 0 Next Kp: 0.270973 Next Ki: 0.000104 Next Kd: 2.646120
Current change: 0 Current Kp: 0.270973 Current Ki: 0.000104 Current Kd: 2.646120 Current error:   230.494 Best error: 206.139000 Next change: 1 Next Kp: 0.248511 Next Ki: 0.000104 Next Kd: 2.646120
Current change: 1 Current Kp: 0.248511 Current Ki: 0.000104 Current Kd: 2.646120 Current error:   256.767 Best error: 206.139000 Next change: 2 Next Kp: 0.259742 Next Ki: 0.000107 Next Kd: 2.646120
Current change: 2 Current Kp: 0.259742 Current Ki: 0.000107 Current Kd: 2.646120 Current error:   320.315 Best error: 206.139000 Next change: 3 Next Kp: 0.259742 Next Ki: 0.000100 Next Kd: 2.646120
Current change: 3 Current Kp: 0.259742 Current Ki: 0.000100 Current Kd: 2.646120 Current error:   498.391 Best error: 206.139000 Next change: 4 Next Kp: 0.259742 Next Ki: 0.000104 Next Kd: 2.738390
Current change: 4 Current Kp: 0.259742 Current Ki: 0.000104 Current Kd: 2.738390 Current error:   227.241 Best error: 206.139000 Next change: 5 Next Kp: 0.259742 Next Ki: 0.000104 Next Kd: 2.553850
Current change: 5 Current Kp: 0.259742 Current Ki: 0.000104 Current Kd: 2.553850 Current error:   237.555 Best error: 206.139000 Next change: 0 Next Kp: 0.269850 Next Ki: 0.000104 Next Kd: 2.646120
Current change: 0 Current Kp: 0.269850 Current Ki: 0.000104 Current Kd: 2.646120 Current error:   492.138 Best error: 206.139000 Next change: 1 Next Kp: 0.249634 Next Ki: 0.000104 Next Kd: 2.646120
Current change: 1 Current Kp: 0.249634 Current Ki: 0.000104 Current Kd: 2.646120 Current error:   245.758 Best error: 206.139000 Next change: 2 Next Kp: 0.259742 Next Ki: 0.000106 Next Kd: 2.646120
Current change: 2 Current Kp: 0.259742 Current Ki: 0.000106 Current Kd: 2.646120 Current error:   274.178 Best error: 206.139000 Next change: 3 Next Kp: 0.259742 Next Ki: 0.000101 Next Kd: 2.646120
Current change: 3 Current Kp: 0.259742 Current Ki: 0.000101 Current Kd: 2.646120 Current error:   352.695 Best error: 206.139000 Next change: 4 Next Kp: 0.259742 Next Ki: 0.000104 Next Kd: 2.729160
Current change: 4 Current Kp: 0.259742 Current Ki: 0.000104 Current Kd: 2.729160 Current error:   271.952 Best error: 206.139000 Next change: 5 Next Kp: 0.259742 Next Ki: 0.000104 Next Kd: 2.563080
Current change: 5 Current Kp: 0.259742 Current Ki: 0.000104 Current Kd: 2.563080 Current error:   220.953 Best error: 206.139000 Next change: 0 Next Kp: 0.268839 Next Ki: 0.000104 Next Kd: 2.646120
Current change: 0 Current Kp: 0.268839 Current Ki: 0.000104 Current Kd: 2.646120 Current error:   508.985 Best error: 206.139000 Next change: 1 Next Kp: 0.250644 Next Ki: 0.000104 Next Kd: 2.646120
Current change: 1 Current Kp: 0.250644 Current Ki: 0.000104 Current Kd: 2.646120 Current error:   231.985 Best error: 206.139000 Next change: 2 Next Kp: 0.259742 Next Ki: 0.000106 Next Kd: 2.646120
Current change: 2 Current Kp: 0.259742 Current Ki: 0.000106 Current Kd: 2.646120 Current error:   262.412 Best error: 206.139000 Next change: 3 Next Kp: 0.259742 Next Ki: 0.000101 Next Kd: 2.646120
Current change: 3 Current Kp: 0.259742 Current Ki: 0.000101 Current Kd: 2.646120 Current error:   284.827 Best error: 206.139000 Next change: 4 Next Kp: 0.259742 Next Ki: 0.000104 Next Kd: 2.720860
Current change: 4 Current Kp: 0.259742 Current Ki: 0.000104 Current Kd: 2.720860 Current error:   318.402 Best error: 206.139000 Next change: 5 Next Kp: 0.259742 Next Ki: 0.000104 Next Kd: 2.571380
Current change: 5 Current Kp: 0.259742 Current Ki: 0.000104 Current Kd: 2.571380 Current error:   216.804 Best error: 206.139000 Next change: 0 Next Kp: 0.267929 Next Ki: 0.000104 Next Kd: 2.646120
Current change: 0 Current Kp: 0.267929 Current Ki: 0.000104 Current Kd: 2.646120 Current error:   332.618 Best error: 206.139000 Next change: 1 Next Kp: 0.251554 Next Ki: 0.000104 Next Kd: 2.646120
Current change: 1 Current Kp: 0.251554 Current Ki: 0.000104 Current Kd: 2.646120 Current error:   221.049 Best error: 206.139000 Next change: 2 Next Kp: 0.259742 Next Ki: 0.000106 Next Kd: 2.646120
Current change: 2 Current Kp: 0.259742 Current Ki: 0.000106 Current Kd: 2.646120 Current error:  1865.850 Best error: 206.139000 Next change: 3 Next Kp: 0.259742 Next Ki: 0.000101 Next Kd: 2.646120
Current change: 3 Current Kp: 0.259742 Current Ki: 0.000101 Current Kd: 2.646120 Current error: 18945.000 Best error: 206.139000 Next change: 4 Next Kp: 0.259742 Next Ki: 0.000104 Next Kd: 2.713380
Current change: 4 Current Kp: 0.259742 Current Ki: 0.000104 Current Kd: 2.713380 Current error: 20472.100 Best error: 206.139000 Next change: 5 Next Kp: 0.259742 Next Ki: 0.000104 Next Kd: 2.578850
```

As the starting conditions for each full loop are not absolutely the same and the number of full loop steps `NUM_LOOP_STEPS` doesn't guarantee an exact start and end position of one single loop, the accumulated full loop cross track error `error` is noisy. Therefore, the Twiddle algorithm cannot really converge. For example if a lucky loop didn't have a large deviation, it might be forever considered to have the best error although other parameter settings would actually be better.

Also, some parameter settings lead to the vehicle leaving the track or getting stuck. When this occured, I restartet the simulator and the Twiddle algorithm continued. As leaving the track or getting stuck leads to a large cross track error `cte` and hence large full loop error `error`, these parameter settings are automatically excluded from being considered as best solution.

The following animations shows a short section of the track with the starting controller parameters `DEFAULT_KP = 0.2`, `DEFAULT_KI = 0.0001` and `DEFAULT_KD = 3.0` on the left and the optimal controller parameters `KP = 0.259742`, `KI = 0.000104` and `KD = 2.646120` on the right.

<img src="docu_images/190126a_StAn_Udacity_SDC_PID_start_small_condensed2.gif" width="48%"> <img src="docu_images/190126b_StAn_Udacity_SDC_PID_optimal_small_condensed2.gif" width="48%">

The optimal parameters on average allow faster speeds with constant throttle which is also an indication of less aggressive steering behavior.

It is important to mention that the stability of the PID controller and the controller parameters are also highly dependend on the dead time of the computer system from sending a new steering angle to the vehicle to receiving a new cross track error from the simulator. The faster the computer system the more exchanges happen and the more stable the control.

## 5. Discussion

I only implemented the tuning of the PID controller parameters for a fixed throttle/speed value. In the example above the throttle value has been selected as 30%. The throttle influences the speed range and the speed significantly influences the lateral dynamics of the vehicle. Therefore, the PID controller parameters should be tuned for different throttle settings. These values should be put into look-up tables that define all gains depending on the throttle value.

While having higher steering angles when going through curves the speed drops with a constant throttle setting. Therefore, an additional PID speed controller can help keep the speed at a constant value at all times. The parameters of this PID speed controller should be twiddled after the optimal parameters for the PID steering controller have been determined.

Finally, the PID steering controller can further be improved by using the output steering angle signal `steering_angle` from the [Behavioral Cloning](https://github.com/CyberAMS/CarND-Behavioral-Cloning-P3/) project as [feedforward control](https://en.wikipedia.org/wiki/Feed_forward_(control)):

```C
// calculate steering angle
steer_value = steering_angle - ((Kp * p_error) + (Kd * d_error) + (Ki * i_error));
```

In this case the steering angle `steering_angle` is estimated based on the trained behavior. The PID steering controller only needs to compensate for the errors in this estimation and not take care of the complete steering angle signal `steer_value`. Therefore, the Twiddle tuning will result in different PID control parameters which are typically much smaller than without the feedforward control. As a result the control behavior can be much smoother.