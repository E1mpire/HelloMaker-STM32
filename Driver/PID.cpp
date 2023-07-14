#include "PID.h"

PID::PID(float min_val, float max_val, float kp, float ki, float kd)
{
  min_val_ = min_val;
  max_val_ = max_val;
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}
void PID::clear_pid(void)
{
  integral_  = 0;
  derivative_= 0;
  prev_error_= 0;
}

double PID::compute(float setpoint, float measured_value)
{
  #if 1
  double error;
  double pid;
  //setpoint is constrained between min and max to prevent pid from having too much error
  error = setpoint - measured_value;
  integral_ += error;
  if(integral_ > 1500) integral_ = 1500;
  else if(integral_ < -1500) integral_ = -1500;
  derivative_ = error - prev_error_;
  if(setpoint == 0 && error == 0){
    integral_ = 0;
  }
  pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
  prev_error_ = error;
  return constrain(pid, min_val_, max_val_);
  #else
  static float Bias,Pwm,Last_bias;
  Bias=setpoint-measured_value;				  //计算偏差
  Pwm+=kp_*(Bias-Last_bias)+ki_*Bias;
  if(Pwm>240)Pwm=240;
  if(Pwm<-240)Pwm=-240;
  Last_bias=Bias;//保存上一次偏差 
  if(setpoint == 0 && Bias == 0)Pwm = 0; 
  return constrain(Pwm, min_val_, max_val_);
  #endif
  
 
}

void PID::updateConstants(float kp, float ki, float kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}
