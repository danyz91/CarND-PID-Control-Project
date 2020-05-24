[//]: # (Image References)

[image1]: ./img/bp2.png "Behavior Planner" 

# CarND-Controls-PID
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This repository contains all the code for the final project for the Control lessons in Udacity's Self-Driving Car Nanodegree.
---

## Overview

In this project the goal is to implement a PID algorithm to control a car running in the simulator. The goal is to keep the Cross Track Error as close as possible to zero.

...

...

...

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

## Installation & Run

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems : `install_ubuntu.sh` and `install_mac.sh`. 

Once the install for uWebSocketIO is complete, the main program can be built and ran.
 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


## Code Style

Code is compliant with [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

It is formatted following [clangformat specification](https://clang.llvm.org/docs/ClangFormat.html)

## PID Implementation

The implementation of the PID algorithm is contained in the class `PID` defined in files [PID.h](./src/PID.h) and [PID.cpp](./src/PID.cpp).

The methods implementing the algorithm are:

- `void Init(double Kp_, double Ki_, double Kd_)`
  
  ...
  This method is defined at **lines 11-25** of PID.cpp file. 
  

- `void UpdateError(double cte)`
  
  ...
  This method is defined at **lines 27-38** of PID.cpp file. 


- `double TotalError()`
  
  ...
  This method is defined at **lines 40-45** of PID.cpp file. 


The file `main.cpp` contains the steps to read data from the simulator, execute the PID Control algorithm and send `steering wheel` and `throttle` commands to the simulator accordingly.

In the current implementation there are **two PID control algorithms** applied to determine:

- Steering wheel to apply to correct the Cross Track Error
- Throttle value to follow a predefined target speed

Target speed is defined via pre-compiler declaration at **line 12** of main.cpp.

    #define MAX_SPEED 30  // MPH

The steps in `main.cpp` are:

1. **Determine Errors**: 
  - Read Cross Track Error from the simulator for the steering wheel error. 
  - Compute gap between current speed and `MAX_SPEED` for speed error.
2. **Run PID algorithm**:
  
  Using the error, compute the correction to apply for steering and throttle.

3. **Clamp values**:

  Values determined are clamped between [-1, 1] for steering wheel and [0, 1] for throttle command

4. **Send value**:

  Values are sent to simulator. Wait for other data from the simulator and return to step 1.

## PID Tuning

In order to tune the PID parameters, manual tuning has been firstly chosen.

The evaluation arose during this process and the final results are showed above.

Manual tuning has proved to be effective for such task. 

## Results


## Future work

Manual tuning has proved to be effective for such task. In more general cases two possible algorithm to try are:

- [Twiddle]
- [Stochastic Gradient Descent](https://en.wikipedia.org/wiki/Stochastic_gradient_descent)





