# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


---
## PID Project Report

#### *Abstract*
The goals of this project are the following:

* Make PID controller
* Use PID controller for steering using cross-track error (CTE)
* Tune PID controller parameters to achieve a save driving in the simulator
* (Optional) Use PID controller to control speed of the vehicle

The PID controller is implemented and used to control the steering and speed of the vehicle. The controller class includes self-tuning based on twiddle. The parameters of the steering controller are tuned to achieve a save driving on speed up to 15 mph.

### 1. Files Submitted 

The code is accessible via this [link](https://github.com/YuryBrodskiy/carnd-yury/tree/master/CarND-PID-Control-Project).

List of changes to orignal project repository:

* CMakeLists.txt + forced compilation of binaries into build folder
* src/main.cpp + speed and steering PID is added and used to controll vehicle
* src/PID.cpp  + PID class with tuning is implemented
* src/PID.h + singnature of PID class is modified
* README.md + added summarizing the results
* video.mov record of successful test drive
 
### 2. PID 

The PID class implements the PID controller. It has two functions generate a control signal based on the error signal and update parameters at the end of each run.  

#### 2.1 Controller

The `UpdateError` function should be called every cycle. In this function, the derivative and cumulative error are calculated.  To minimize the effect of variable cycle time (it is not constant since the controller is event driven and non-real time OS is used to run simulator) the actual time between cycles is measured and used for derivative and integral calculations. Moreover, to make sure that connection delay does not cause erroneous input if the time between cycles is too big (double of expected) then controller-internal variables are reset. 

The control signal of the PID controller is constructed using base on an instantaneous error, a derivative of the error and an integral error, with respective gains (Kp, Kd, Ki). When the PID controller is applied to the steering of a vehicle each component in controller signal has a distinct role. The instantaneous cross track error provides the basis for steering angle i.e. always steer to minimize the error. The derivative of the error minimize the overshoot when vehicle lands on the desired trajectory, it also removes oscillatory behavior that would result from pure proportional controller. The integral error correction is typically needed to compensate for static error such as steering wheel miss-alignment. Although this problem is not present in this project, the integral error correction results in better behavior in the curves. When vehicle should be continuously turning the integral error is build up and the controller oversteers in the direction of the turn.

#### 2.1 Tuning

The tuning of the steering PID parameters is set up as a series of independent experiments. The vehicle drives approximately one lap in the simulated environment. During the drive an optimization function is calculated. The optimization function is constructed as the sum of squared cross track error and squared control signal. This optimization function is chosen to achieve a control with small cross track errors and avoid big control inputs.

The `twiddle_step` function implements the tuning of the controller based on twiddle algorithm. The function is set up to be called recurrently at the end of each experiment. The function will update parameters and prepare for next experiment. When parameter update steps become too small (squared length of update vector is less than 1e-5), the optimization is stopped.


### 3. Results

The PID controller is implemented and used to control the steering and speed of the vehicle. The controller class includes self-tuning based on twiddle. The parameters of the steering controller are tuned to achieve a save driving on speed up to 15 mph. The test drive is recorded into a [video](./video.mov). The tuning procedure and the test drive can be verified using Udacity simulator and following the Basic Build Instruction above.
