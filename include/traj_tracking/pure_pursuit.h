#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H
class Tracking_controller {
 public:
  double Kp = 0.3;  // proportional gain
  // system config
  double Ld = 2.6;  // look ahead distance
  double kf = 0.1;  // look forward gain
  double dt = 0.1; // T step
  double dist_stop = 0.7; // stop distance
  double dc = 0.0;
  // vehicle config
  double RF = 3.3; // [m] distance from rear to vehicle front end of vehicle
  double RB = 0.8; // [m] distance from rear to vehicle back end of vehicle
  double W = 2.4 ; //[m] width of vehicle
  double WD = 0.7 * W; // [m] distance between left-right wheels
  double WB = 2.5; //[m] Wheel base
  double TR = 0.44; // [m] Tyre radius
  double TW = 0.7; // [m] Tyre width
  double MAX_STEER = 0.30;
  double MAX_ACCELERATION = 5.0;
  double limit_input(double phi); 
  double cal_target_index(double vel);
};
#endif 