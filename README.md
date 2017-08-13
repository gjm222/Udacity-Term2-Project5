# Term 2 Project 5 : CarND-Controls-MPC Project
Self-Driving Car Engineer Nanodegree Program

---
## Objective
The objective is to get information from the simulator which includes waypoints and vehicle state and use it to determine steering angle and accelleration that is sent back to the simulator to control the vehicle.

## Model Predictive Control (MPC) Implementation
MPC was used and described as follows:
* Receive waypoints (x and y points) and state information (x position, y position, orientation, velocity, steering angle, and throttle) of the car from simulator
* Shift and rotate waypoints to make calculations and derivations easier.
* Create polynomial line given the waypoints for use as the reference.
* In order to account for latency, calculate initial/predictive values for the state of the vehicle 100ms in the future.
* Set up constraints mainly for the steering angle and throttle.
* Set reference velocity.
* Set timestep length (dt) and number of time steps (N).
* Using the initial state and reference polynomial, determine steering angle and throttle values using predictive calculations N steps into the future along with error in order find the best fit future polynomial.  From the best fit polynomial, send back the steering angle and throttle values send back to the simulator.


## Timestep Length and Elapsed Duration (N & dt)
## Polynomial Fitting and MPC Preprocessing
## Model Predictive Control with Latency
