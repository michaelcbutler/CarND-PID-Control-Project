#include "PID.h"
#include <iostream>

PID::PID(double Kp, double Ki, double Kd, int N) 
: Kp_(Kp), Ki_(Ki), Kd_(Kd), N_(N) {
  iter_ = 0;
  Etotal_ = 0.0;
  Ed_ = Ep_ = Ei_ = 0.0;
  std::cout << "\tKp = " << Kp_ << "\tKd = " << Kd_ << "\tKi = " << Ki_ << std::endl;
}

PID::~PID() {}

void PID::UpdateError(double error) {
  Ed_ = error - Ep_; // Ep_ is prev error
  Ep_ = error;
  Ei_ += error;

  ++iter_;
  if (iter_ == 1) {
    Ed_ = 0.0; // prev error unknown
  }
  if (iter_ > N_) { 
    Etotal_ += error*error;
  }
}

double PID::Correction() {
  return -1.0*(Kd_*Ed_ + Ki_*Ei_ + Kp_*Ep_);
}
