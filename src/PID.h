#ifndef PID_H
#define PID_H

class PID {
private:
  /*
  * Errors
  */
  double Ep_; // proportional
  double Ei_; // integral
  double Ed_; // derivative

  double Etotal_;

  /*
  * Coefficients
  */ 
  const double Kp_; // proportional
  const double Ki_; // integral
  const double Kd_; // derivative

  int iter_; // iteration number
  const int N_; // iteration threshold for total error computation

public:

  /*
  * Constructor
  */
  PID(double Kp, double Ki, double Kd, int N = 0);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Update the PID error variables given error.
  */
  void UpdateError(double error);

  /*
  * Calculate the total PID error.
  */
  double TotalError() {
    return Etotal_;
  }

  /*
  * Correction term
  */
  double Correction();
};

#endif /* PID_H */
