## Reflection
The control module is a very important stage of the AD pipeline. In this project, the inputs states are retrieved from the outputs of the path planning module in the form of waypoints sets. The objective of the controller is to track the path plan as a reference as close as possible, using two actuator commands (steering angle and throttle) within a certain physical constraints. This particular control problem is framed into an optimization problem with the help of MPC, and the output commands cost functions are solved by the Ipopt Nonlinear Programming Solver.

<p align="center">
     <img src="./Pipeline.PNG" alt="Pipeline" width="40%" height="40%">
     <br>Pipeline.PNG
</p>

# Vehicle Model
An classic Bicycle Model is use here to deliver the vehicle dynamic of each time instance:
<p align="center">
     <img src="./Model.PNG" alt="Model" width="40%" height="40%">
     <br>Model.PNG
</p>
The local coordinates/states are updated through the following dynamics:

```cpp
          px[t+dt] = px[t] + v[t] * cos(psi[t]) * dt;
          py[t+dt] = py[dt] + v[t] * sin(psi[t]) * dt;
          psi[t+dt] = psi[t] - v[t] * delta[t] / Lf * dt;
          v[t+dt] = v[t] + a[t] * dt;
          cte[t+dt] = cte[t] + v[t] * sin(epsi[t]) * dt;
          epsi[t+dt] = epsi[t]+ v[t] * delta[t] / Lf * dt;
```
Where px,py is the vehicle position and psi is the heading orientation. The velocity v is updated with the throttle(accel) a. The sampling time is selected as 100ms for this project.
<p align="center">
     <img src="./dynamic local.png" alt="dynamic local" width="40%" height="40%">
     <br>dynamic local.png
</p>

The error feedbacks are updated through the following equations: 
```cpp
          cte[t+dt] = cte[t] + v[t] * sin(epsi[t]) * dt;
          epsi[t+dt] = epsi[t]+ v[t] * delta[t] / Lf * dt;
```
Where cte is the cross track error, which corresponds to distance of vehicle from the planned trajectory. The epsi is the is the angle difference of the vehicle trajectory with the planned trajector. 
<p align="center">
     <img src="./dynamic map.png" alt="dynamic map" width="40%" height="40%">
     <br>dynamic map.png
</p>

# Cost Functions and MPC Tuning 
The cost function part is similar to a classic convex optimal control problem:
<p align="center">
     <img src="./CF.PNG" alt="CF" width="40%" height="40%">
     <br>CF.PNG
</p>
Based on the objectives, the cost function get penalized everytime:
* cte and epsi increase - to ensure steady state error
* high usage of control effort - to minimize the use of actuators
* high change rate of control effort - to ensure smooth drive 

The weight for each penalty is defined based on the importance of the states. Refference tracking is set to the highest priority, then smoothness is very important as well to damp the overshoot of the response. 

For the rest of states feed into the CppAD are served as a regulator which had the constraints set to zero. They will be updated with the new control outputs each loop to solve for the cost function. 
<p align="center">
     <img src="./solver_in.png" alt="solver_in" width="40%" height="40%">
     <br>solver_in.png
</p>

# Prediction Horizon and Latency Effects
The prediction horizon is selected as 10 steps with a sampling time of 100ms. The control horizon is not specified (can be improved since we working with a simulator and the response is fairly fast). Some trade-off were made to ensure the visibility of the prediction and also limited the computation cost.
The states are updated for an interval of 100ms before feeding into the solver, to compensate the latancy effects(if introduced).

# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
