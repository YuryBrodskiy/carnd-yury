# Unscented Kalman Filter Project 

Project produce two binaries one to use with simulator  (UnscentedKF) and one to use with text file (CompareKF).

CompareKF is used for tuning, thus it compute the percent of samples with expected NIS region. 
CompareKF runs EKF and UKF side by side to compare performance.

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4
* openssl 
* libuv 
* zlib

## Basic Build Instructions

1. Clone this repo.
2. Run install-mac.sh or install-ubuntu.sh `chmod u+x ./install-mac.sh && ./install-mac.sh`  
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./CompareKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./CompareKF ../data/obj_pose-laser-radar-synthetic-input.txt`
5. Run it with [simulator](https://github.com/udacity/self-driving-car-sim/releases) 
   Start the simulator and execute `./UnscentedKF`
