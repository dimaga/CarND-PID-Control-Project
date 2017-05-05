#ifndef SRC_PID_H_
#define SRC_PID_H_

class PID {
 public:
  /*
  * Errors
  */
  double p_error_{0};
  double i_error_{0};
  double d_error_{0};

  /*
  * Coefficients
  */ 
  double Kp_{0};
  double Ki_{0};
  double Kd_{0};

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  *
  * If apply_integral is false, integral error is not accumulated.
  * This is a measure to prevent integral wind up
  */
  void UpdateError(double cte, bool apply_integral);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif  // SRC_PID_H_
