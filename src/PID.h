#ifndef PID_H
#define PID_H

#include <limits>

// define constants
const bool TWIDDLE = true;
unsigned int NUM_CONVERGED_STEPS = 100;
unsigned int NUM_LOOP_STEPS = 2000;
double DEFAULT_KP = 0.2;
double DEFAULT_KI = 0.0001;
double DEFAULT_KD = 3.0;
double TWIDDLE_FACTOR = 10.0;
enum CHANGE_STATES {INCREASE_KP, DECREASE_KP, INCREASE_KI, DECREASE_KI, INCREASE_KD, DECREASE_KD};
unsigned int NUM_CHANGE_STATES = 6;

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

 private:
  /**
   * PID Errors
   */
  double p_error = 0.0;
  double i_error = 0.0;
  double d_error = 0.0;
	double error = 0.0;
	double best_error = std::numeric_limits<double>::max();

  /**
   * PID Coefficients
   */ 
  double Kp = DEFAULT_KP;
  double Ki = DEFAULT_KP;
  double Kd = DEFAULT_KD;
	double dKp = DEFAULT_KP / TWIDDLE_FACTOR;
	double dKi = DEFAULT_KI / TWIDDLE_FACTOR;
	double dKd = DEFAULT_KD / TWIDDLE_FACTOR;
	
	// status variables
	bool is_converged = false;
	unsigned int converge_steps = 0;
	unsigned int full_loop_steps = 0;
	CHANGE_STATES change = INCREASE_KP;
	
};

#endif  // PID_H