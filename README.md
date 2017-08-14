# Term 2 Project 5 : CarND-Controls-MPC Project
Self-Driving Car Engineer Nanodegree Program
---

## Objective
The objective is to get information from the simulator which includes waypoints and vehicle state and use it to determine a new steering angle and throttle which is then sent back to the simulator to control the vehicle.


## Model Predictive Control (MPC) Implementation
A kinematic MPC was used and described as follows:
* Receive waypoints (x and y points) and state information (x position, y position, orientation, velocity, steering angle, and throttle) of the car from simulator.
* Shift and rotate waypoints to make calculations and derivations easier. See *Polynomial Fitting and MPC Preprocessing* below.
* Create polynomial line given the waypoints for use as the reference. See *Polynomial Fitting and MPC Preprocessing* below.
* In order to account for latency, calculate initial/predictive values for the state of the vehicle 100ms in the future.  See *Model Predictive Control with Latency* below.
* Set up constraints mainly for the steering angle and throttle.
* Set reference velocity.
* Set timestep length (dt) and number of time steps (N).  See *Timestep Length and Elapsed Duration (N & dt)* below.
* Tune cost.
Here is the code that sets and tunes the cost of relevent variables. See comments for details.
```
// The part of the cost based on the reference state.
for (size_t t = 0; t < N; t++) {
  fg[0] += 2000*CppAD::pow(vars[cte_start + t], 2);  //Emphasize keeping car in middle of waypoint polynomial
  fg[0] += 2000*CppAD::pow(vars[epsi_start + t], 2); //Emphasize keeping car in middle of waypoint polynomial
  fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2); //Direct car to go at a reference velocity
}

// Minimize the use of actuators.
for (size_t t = 0; t < N - 1; t++) {
  fg[0] += 5*CppAD::pow(vars[delta_start + t], 2);  
  fg[0] += 5*CppAD::pow(vars[a_start + t], 2);      
}

// Minimize the value gap between sequential actuations.
for (size_t t = 0; t < N - 2; t++) {
  fg[0] += 200*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2); //Soften sharp turning 
  fg[0] += 10*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);          //Soften throttle
}
```
* Using the initial state and reference polynomial, determine steering angle and throttle values using predictive calculations N steps into the future along with error and contraints in order find the best fit (Note: "best fit" meaning the best polynomial that can be found in .5 seconds) future polynomial.  The best fit polynomial is found using the Ipopt 3.12.1 library 'solve' method which does all the partial derivitive calculations with in and set, in the case .5 seconds, amount of time.  From the best fit polynomial, the first found steering angle and throttle values are sent back the simulator to control the vehicle.  
```
MPC.c: Calling solve method...
CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);
```
```
main.c: Where steering angle and throttle are sent back ...
msgJson["steering_angle"] = vars[0]/(deg2rad(25)*Lf); //steer_value;
msgJson["throttle"] = vars[1]; //throttle_value;
```


## Timestep Length and Elapsed Duration (N & dt)
A value of 10 for N and .1 seconds for dt was used for a total predition of 1 second into the future.  This seems like a reasonable number of steps to accurately predict and not use too many cpu cycles.  I got this value from the class.  When I kept N = 10 and switched  dt ~ .2, I was able to get the car to make it around the track without any latency handling but the car would decrease speed to ~ 20/mph.


## Polynomial Fitting and MPC Preprocessing
```
main.c: Shift and rotate waypoints for easier calculation...
for ( size_t i = 0; i< ptsx.size(); i++ )
{
   double shift_x = ptsx[i]-px;
   double shift_y = ptsy[i]-py;
   //Shift and rotate
   ptsx[i] = (shift_x * cos(0-psi) - shift_y * sin(0-psi));
   ptsy[i] = (shift_x * sin(0-psi) + shift_y * cos(0-psi));
}
```
```
main.c: Load processed waypoints into a type the polyfit() function understands and create 3 order polynomial from those points...
double* ptrx = &ptsx[0];
Eigen::Map<Eigen::VectorXd> ptsx_trans(ptrx,6);
double* ptry = &ptsy[0];
Eigen::Map<Eigen::VectorXd> ptsy_trans(ptry,6);

auto coeffs = polyfit(ptsx_trans, ptsy_trans, 3);
```

## Model Predictive Control with Latency
To address latency, take the current state and alter it to 100ms in to the future.
```
double latency_adj = .1;  //100ms
...
double init_x = v * latency_adj;  //Because we shifted and rotated, the velicity is in x direction only
double init_y = 0; //Because we shifed and rotated the y position will always be 0
double init_psi = v * -steer_value / Lf * latency_adj;  
double init_v = v + throttle_value * latency_adj;      
double init_cte = cte +  (v * sin(epsi) * latency_adj);
double init_epsi = epsi + v * -steer_value / Lf * latency_adj;

state << init_x, init_y, init_psi, init_v, init_cte, init_epsi;  //Set current state vector 
```
