#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>
#include "PID.h"

// using std::fabs;
using std::min;
using std::max;
using std::fmod;
using std::cout;
using std::endl;
using std::setw;
using std::setfill;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
	
	// initialize status variables
	is_converged = false;
	converge_steps = 0;
	full_loop_steps = 0;
	change = 0;
	
	// initialize PID parameters
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;
	dKp = DEFAULT_KP / TWIDDLE_FACTOR;
	dKi = DEFAULT_KI / TWIDDLE_FACTOR;
	dKd = DEFAULT_KD / TWIDDLE_FACTOR;
	
	// initialize error variables
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
	error = 0.0;
	best_error = std::numeric_limits<double>::max();
	
	// display status
	cout << setw(DISPLAY_COLUMN_WIDTH) << setfill(' ') << "Current change: " << "X" << " Current Kp: " << "X" << " Current Ki: " << "X" << " Current Kd: " << "X" << " Current error: " << "X" << " Best error: " << best_error << " Next change: " << change << " Next Kp: " << Kp << " Next Ki: " << Ki << " Next Kd: " << Kd << endl;
	
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
	
	// define variables
	unsigned int current_change = 0;
	double current_Kp = 0.0;
	double current_Ki = 0.0;
	double current_Kd = 0.0;
	
	// calculate error terms
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
	
	// check whether controller parameters should be twiddled
	if (TWIDDLE) {
		
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
				
				// display status
				cout << setw(DISPLAY_COLUMN_WIDTH) << setfill(' ') << "Current change: " << current_change << " Current Kp: " << current_Kp << " Current Ki: " << current_Ki << " Current Kd: " << current_Kd << " Current error: " << error << " Best error: " << best_error << " Next change: " << change << " Next Kp: " << Kp << " Next Ki: " << Ki << " Next Kd: " << Kd << endl;
				
				// prepare next twiddle cycle/loop
				error = 0.0;
				is_converged = false;
				
			}
			
		}
		
	}
	
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
	 
	// define variables
	double steer_value;
	
	// calculate steering angle
	steer_value = -((Kp * p_error) + (Kd * d_error) + (Ki * i_error));
	
	// limit steering angle
	steer_value = max(min(steer_value, 1.0), -1.0);
	
	return steer_value;  // TODO: Add your total error calc here!
	
}