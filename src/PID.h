#ifndef PID_H
#define PID_H
#include <uWS/uWS.h>
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
  *  twiddle params
  */
  double dKp_;
  double dKi_;
  double dKd_;

  double current_averaged_error_;

  double best_averaged_error_;

  double num_twiddle_time_steps_;

  int current_tweak_dimension_;
  
  int current_time_step_;

  bool tried_decreasing_;

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

  /*
  * Send restart message to simulator
  */
  void Restart(uWS::WebSocket<uWS::SERVER> ws);


  /*
  * Check whether twiddle time interval has elapsed
  */
  bool IsTwiddleTimeIntervalComplete();

  /*
  * Run run twiddle step
  */
  void DoTwiddleTweak();

  /*
  * Print corrent controller parameters to std::cout
  */
  void PrintCurrentSettings();

  /*
  * Increase controller parameters (twidlle step)
  */
  void IncreaseParams();

  /*
  * Reset all controller parameter
  */
  void ResetParams();

  /*
  * Decrease controller parameters (twidlle step)
  */
  void DecreaseParams();

  /*
  * Decrease delta parameters (twidlle step)
  */
  void DecreaseDeltaParams();


};

#endif /* PID_H */
