#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;

  /*
  * Coefficients
  */
  double Kp_;
  double Ki_;
  double Kd_;

  /*
  * Helpers
  */
  double cte_last_time_step_;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(const double Kp, const double Ki, const double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(const double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
