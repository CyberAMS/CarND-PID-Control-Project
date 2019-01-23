#include <cmath>
#include "PID.h"

using std::min;
using std::max;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
	
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;
	
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
	
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
	
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
	steer_value = max(min(steer_value, 1), -1);
	
	return steer_value;  // TODO: Add your total error calc here!
	
}