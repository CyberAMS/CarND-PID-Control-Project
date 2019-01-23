#ifndef PID_H
#define PID_H

// define constants
const bool DO_TWIDDLE = false; 
dp = {0.1*Kp,0.1*Kd,0.1*Ki}; 
step = 1; 
param_index = 2;  // this will wrap back to 0 after the first twiddle loop 
n_settle_steps = 100; 
n_eval_steps = 2000; 
total_error = 0; 
best_error = std::numeric_limits<double>::max(); 
tried_adding = false;  
tried_subtracting = false; 


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
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
};

#endif  // PID_H