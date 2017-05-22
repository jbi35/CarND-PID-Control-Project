#include "PID.h"

//using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(){}

PID::~PID(){}

void PID::Init(const double Kp, const double Ki,const double Kd)
{
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  cte_last_time_step_ = 0.0;
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void PID::UpdateError(const double cte)
{
  p_error_ = cte;
  d_error_ = (cte - cte_last_time_step_);
  i_error_ += cte;
  cte_last_time_step_ = cte;
}

double PID::TotalError()
{
  return    Kp_ * p_error_  // propportonal part
          + Kd_ * d_error_  // differential part
          + Ki_ * i_error_; // integral part
}
