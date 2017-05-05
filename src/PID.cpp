#include "PID.h"


void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void PID::UpdateError(double cte, bool apply_integral) {
  d_error_ = cte - p_error_;
  p_error_ = cte;

  if (apply_integral) {
    i_error_ += cte;
  }
}

double PID::TotalError() {
  return -p_error_ * Kp_ - d_error_ * Kd_ - i_error_ * Ki_;
}

