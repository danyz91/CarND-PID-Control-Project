#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  p_error = std::numeric_limits<double>::max();
  i_error = std::numeric_limits<double>::max();
  d_error = std::numeric_limits<double>::max();

  prev_cte = 0.0;
  sum_cte = 0.0;

  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  double d_cte = cte - prev_cte;
  sum_cte += cte;
  p_error = -Kp * cte;
  i_error = -Ki * sum_cte;
  d_error = -Kd * d_cte;

  prev_cte = cte;
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  return p_error + i_error + d_error;
}
