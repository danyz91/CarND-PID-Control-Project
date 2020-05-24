[//]: # (Image References)

[image1]: ./img/1_only_prop_10.png "Image1"
[image2]: ./img/2_P10_D10.png "Image2"
[image3]: ./img/3_P01_D1.png "Image3"
[image4]: ./img/4_P01_I01_D10.png "Image4"
[image5]: ./img/5_P01_I00001_D10.png "Image5"
[image6]: ./img/6_P01_I00001_D20.png "Image6"
[image7]: ./img/7_P_01_I00001_D50.png "Image7"
[image9]: ./img/30MPH_same/speed.png "Image8"
[image8]: ./img/30MPH_same/steer.png "Image9"
[image11]: ./img/70MPH_01/speed.png "Image10"
[image10]: ./img/70MPH_01/steer.png "Image11"
[image13]: ./img/30MPH/speed.png "Image12"
[image12]: ./img/30MPH/steer.png "Image13"
[image15]: ./img/50MPH/speed.png "Image14"
[image14]: ./img/50MPH/steer.png "Image15"
[image17]: ./img/70MPH/speed.png "Image16"
[image16]: ./img/70MPH/steer.png "Image17"


# CarND-Controls-PID
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This repository contains all the code for the final project for the Control lessons in Udacity's Self-Driving Car Nanodegree.

## Overview

In this project the goal is to implement a PID algorithm to control a car running in the simulator. The goal is to keep the Cross Track Error as close as possible to zero.

The Cross Track Error is the lateral difference between vehicle position and center line of the race track. This value is provided by the simulator.

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
  
  This method setups the three gain for PID algorithm.

  This method is defined at **lines 11-25** of PID.cpp file. 
  

- `void UpdateError(double cte)`
  
  This method computes the error relative to proportional, derivative and integral contribution in the PID algorithm.

  This method is defined at **lines 27-38** of PID.cpp file. 


- `double TotalError()`
  
  This method returns the total PID error taking into account all single contributions.

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

The process has started firstly by tuning the parameters for the steering PID, with a fixed throttle value of `0.3`.

Then, the speed PID has been tuned.

Manual tuning has been proved by plotting the error value while the car was running in the simulator.

From the theory we would expect that:

- **Proportional Contribution**

  Proportional error measures how far we are from central track and its contribution will be that larger the error the more vehicle turns the steering wheel to trajectory line

- **Derivative Contribution**

  Derivative error measures how fast is changing the error over a time horizon and its contribution will be that while steering for reaching trajecotry, control will see that error is reducing over time, and counter steer. This is done to remove overshoot problem.

- **Integral Contribution**

  Integral error measures if the vehicle see a sustained error over time and its contribution will be to rejects this systematic bias present in the system.

### Steering PID

#### `K_p = 1.0, K_i = 0.0, K_d = 0.0`

![alt text][image1]

#### Comment

From this image we have the evidence that the car can't manage to complete the track, since the timesteps on x-axis stop before 1000, while a complete lap usually requires more than 4000 time steps for being completed.

From this image we could see two important aspects of P-controller:

- It is marginally stable, this can be seen by the oscillations between steps 200-600
- It starts to oscillate quickly and it diverges when it found the first curve of the track, around time step 700

So it has been decided to raise the derivative contribution in order to counter steer when the car is approaching the center line

#### `K_p = 1.0, K_i = 0.0, K_d = 1.0`

![alt text][image2]

#### Comment

Here we can see that derivative contribution manage to improve the stability of the controller in the straight track between steps 200 and 600.

This parametrization can't handle the first curve though.

It has been decided to reduce the proportional contribution in order to give importance to the derivative contribution.


#### `K_p = 0.1, K_i = 0.0, K_d = 1.0`

![alt text][image3]

#### Comment

This is the first parametrization that allows the vehicle to complete the whole lap. 

It is possible to see some remaining oscillations when reverting from a curve, but it could be considered satisfiable for the moment.

So far, we have not taken into account possible systematic error that neither proportional contribution can handle.

So we start considering the integral contribution of the PID.

#### `K_p = 0.1, K_i = 0.01, K_d = 1.0`

![alt text][image4]

#### Comment

With this parametrization it is clear that the vehicle heavily sub-performs. The car start to steer as soon as it enters the simulation. 

The integral contribution must be scaled a lot.

#### `K_p = 0.1, K_i = 0.0001, K_d = 1.0`

![alt text][image5]

#### Comment

This parametrization is satisfiable and take into account also the integral contribution.

Now it is possible to focus again on oscillations and try to absorb them.

To reduce oscillations it has been decided to raise the derivative contribution as it takes into account **how** the vehicle is reducing the error.

It has been decided to raise derivative contribution to give more long term focus to the controller.


#### `K_p = 0.1, K_i = 0.0001, K_d = 2.0`

![alt text][image6]

#### Comment

This parametrization seems great. Oscillations are reduced a lot.

Why not raising the derivative contribution more, in order to reject oscillations completely?

It has been tried an even bigger derivative contribution.

#### `K_p = 0.1, K_i = 0.0001, K_d = 5.0`

![alt text][image7]

#### Comment

Here things are getting worse. We have spikes during curves as the one clear at time steps between 2000 and 2500.

So it has been decided to get back to the previous parametrization.

### Final PID Parameters for Steering

Parameter | Value
--- | ---
**K_p** | **0.1**
**K_i** | **0.0001**
**K_d** | **2.0**

Now it is the turn of the speed PID.

### Speed PID

Here it will be showed also the `MAX_SPEED` parameter value of each experiment.

The first parametrization tried is the same as the final parametrization for the Steering PID.

#### `K_p = 0.1, K_i = 0.0001, K_d = 2.0`, `MAX_SPEED = 30 MPH`

Steering Error | Speed Error
--- | ---
![alt text][image8] | ![alt text][image9] 

It seems satisfiable but it has been decided to inspect what happens when `MAX_SPEED` is raised

#### `K_p = 0.1, K_i = 0.0001, K_d = 2.0`, `MAX_SPEED = 70MPH`

Steering Error | Speed Error
--- | ---
![alt text][image10] | ![alt text][image11]

#### Comment

Here we can see that this parametrization is not robust enough w.r.t. higher speeds.

So it has been decided to raise the proportional contribution in order to be more reactive to errors and return quickly to center line.

Final result are showed for 30, 50 and 70 MPH as reference speed to reack.

#### `K_p = 0.3, K_i = 0.0001, K_d = 2.0`,  `MAX_SPEED = 30MPH`

Steering Error | Speed Error
--- | ---
![alt text][image12] | ![alt text][image13]

#### `K_p = 0.3, K_i = 0.0001, K_d = 2.0` `MAX_SPEED = 50MPH`

Steering Error | Speed Error
--- | ---
![alt text][image14] | ![alt text][image15]


#### `K_p = 0.3, K_i = 0.0001, K_d = 2.0`,  `MAX_SPEED = 70MPH`

Steering Error | Speed Error
--- | ---
![alt text][image16] | ![alt text][image17]

#### Comment

In all the three configuration the same speed PID parametrization manages to drive the vehicle for the whole lap.

Of course, the faster the vehicle is moving, the more it will be subject to oscillations, but at maximum speed of 70 MPH the result is acceptable.


### Final PID Parameters for Steering

Parameter | Value
--- | ---
**K_p** | **0.3**
**K_i** | **0.0001**
**K_d** | **2.0**

## Results

Final parameters tuned are : 

Parameter | Steering PID | Speed PID
--- | --- | ---
**K_p** | **0.1** | **0.3**
**K_i** | **0.0001** | **0.0001**
**K_d** | **2.0** | **2.0**

Here there are shown 3 videos of the final parametrization of the two controllers:

- **Target Speed = 30 MPH**

  [![final](https://img.youtube.com/vi/Vz5K1Fy5ZE4/0.jpg)](https://www.youtube.com/watch?v=Vz5K1Fy5ZE4) 


- **Target Speed = 50 MPH**

  [![final](https://img.youtube.com/vi/dRd4IjG1BiA/0.jpg)](https://www.youtube.com/watch?v=dRd4IjG1BiA) 

- **Target Speed = 70 MPH**

  [![final](https://img.youtube.com/vi/NV6xqEDX8hs/0.jpg)](https://www.youtube.com/watch?v=NV6xqEDX8hs) 

## Future work

Manual tuning has proved to be effective for such task. In more general cases two possible algorithm to try are:

- [Twiddle Algorithm](https://www.youtube.com/watch?v=2uQ2BSzDvXs)

  Explained by Sebastian Thrun during the Udacity lesson, whose explanation video can be found on YouTube

- [Stochastic Gradient Descent](https://en.wikipedia.org/wiki/Stochastic_gradient_descent)





