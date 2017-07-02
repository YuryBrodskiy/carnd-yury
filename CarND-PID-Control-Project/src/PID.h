#ifndef PID_H
#define PID_H

class PID {
  enum Dimention
  {
    P = 0,
    I = 1,
    D = 2,
  };

  enum TwiddleState
  {
     start,
     change,
  };
  /*
  * Coefficients
  */ 
  double K[3];
  double dK[3];
  Dimention opt_dim;
  TwiddleState twiddle_state;


  double optimization_value;
  double optimization_value_best;

  double prev_time;
  double prev_error;
  double int_error;

  double control_value;
public:



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
  PID& Init(double Kp, double Ki, double Kd);

  /*
   * Initialize Twiddle.
   */
  void InitTwiddle(double dKp, double dKi, double dKd);
  /*
  * Update the PID error variables given cross track error.
  */
  PID& UpdateError(double error);

  /*
  * Calculate the total PID error.
  */
  void twiddle_step();
  /**
   *  return control signal
   */
  double control();
};

#endif /* PID_H */
