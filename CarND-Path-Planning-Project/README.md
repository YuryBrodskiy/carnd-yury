# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
* Simulator.
 * You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

## The path planner external interface

#### The map of the highway
+ The map of the highway be supplied in data/highway_map.txt
+ Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position; the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.
+ The highway's waypoints loop around so the frenet s value, the distance along the road, goes from 0 to 6945.554.

#### Main car's localization Data (No Noise)
+ ["x"] The car's x position in map coordinates
+ ["y"] The car's y position in map coordinates
+ ["s"] The car's s position in Frenet coordinates
+ ["d"] The car's d position in Frenet coordinates
+ ["yaw"] The car's yaw angle in the map
+ ["speed"] The car's speed in MPH

#### Previous path data given to the Planner
The unprocessed list of points that were sent on the previous cycle.

+ ["previous_path_x"] The previous list of x points previously given to the simulator
+ ["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 
+ ["end_path_s"] The previous list's last point's Frenet s value
+ ["end_path_d"] The previous list's last point's Frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)
["sensor_fusion"] A 2d vector of cars and then that car's 
 
 + car's unique ID, car's x position in map coordinates,
 + car's y position in map coordinates
 + car's x velocity in m/s
 + car's y velocity in m/s
 + car's s position in frenet coordinates
 + car's d position in frenet coordinates. 

## Project instructions
### Goals
In this project, your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data; there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying **to go 50 MPH**, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total **acceleration over 10 m/s^2** and **jerk that is greater than 50 m/s^3**.



### Details
1. The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner receives should not have a total acceleration that goes over 10 m/s^2. Also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.
2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually it's not very long maybe just 1-3 duration steps. During this delay, the simulator will continue using points that it was last given, because of this it's a good idea to store the last points you have used so you can have a smooth transition. previous_path_x and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or makes sure to create a new path that has a smooth transition with this last path.


## Path planner report
#### *Abstract*

The goals of this project are the following:

* Create path planner that allows driving the car around the track in Udacity simulator
* The car should drive within Jerk, Acceleration and Velocity limits
* The car should take over the slower traffic cars
* The car should drive within lanes on the road
* The car should not collide with other vehicles.

A path planner is created that allows driving on the track without accidents. The planner is capable of planning trajectories to take over other vehicles and keep the lanes. The planner creates trajectories within specified dynamical limits 50 mph of speed and 10[m/s^3] jerk, 50[m/s^2] acceleration. 

###  Design

+ main.cpp - responsible for initialization and communication with the simulator 
+ utils - contains various helper functions, types and struct definitions 
+ Road class - responsible for road description. This includes maintaining the map of the road, Frenet to cartesian transformations, data fusion management, and cost function related to driving on the road, such as collision and driving out of a lane.
+ EgoVehicle class - responsible for trajectory generation to meet comfort specification. This includes generating max speed trajectory for a lane, stitching previous and new trajectories.
+ Option class - responsible for proposing various trajectories with associated costs.

### Driving in the lane
Using Fernet coordinates is a convenient way to express position and speeds of the vehicles on the road. However there are two main problems, that impair the use of the Fernet coordinates as the main coordinate system in this project:

1. The provided map of the highway has a low resolution (1 point every 30 meters). Thus if these points are used naively, the car will drive in straight lines and make a sharp turn every 30 meters.
2. The mapping between cartesian and Fernet coordinates is not linear. Thus distances between points in Frenet coordinates and the same points in cartesian coordinate will be different. Such differences will distort trajectories generated in Fernet coordinates and the dynamical limits on vehicle driving will be violated. 

The original map is interpolated by path planner start to address the problem of low resolution. The interpolated map is then used to transform Frenet coordinates to cartesian. [getHighDefinitionMap function in Road class]

To preserve linear speeds when transforming trajectory from Frenet to Cartesian coordinates, we adjust a time steps at which the Frenet polynomial is evaluated.[getWarpedTime function in utils] This concept works as follows:
 
 + Define the distance to be traveled (Euclidian distance between previous point in Frenet coordinate and point in Frenet coordinates in 0.02 seconds)
 + Find a time step such that the distance to be traveled in Frenet coordinates is equal to the distance traveled in Cartesian coordinates.
 + The resulting 2 points will be located on the desired polynomial and have distance between such that linear speed along polynomial in Cartesian coordinates is preserved.
 
 
### Time delay and stitching trajectories.

During trajectory generation, the car is continuing to move along the trajectory supplied in the previous cycle. To create uninterrupted trajectories 15 points from the previous cycle are preserved and the new trajectory is appended. The new trajectory is generated up to 100 points thus simulator has ample time to execute before requesting the new set of points. On each request, the excess of the points from the previous cycle is thrown away. Also, it is possible to see the future plan in the simulator as the displayed trajectory spanes 3 seconds ahead.

### Collision prediction

The other vehicles positions and movement are predicted during the planned trajectory using sensor fusion data. I have assumed that all vehicles will keep the lane and keep the speed in the lane. Although it is possible to create better predictions using for example combination of Kalman Filter and trajectory classifiers, experiments show that a simple assumption already provides a required result.

### Path planning

The proposed path planner attempts to select the fastest lane on the road and generates trajectories to change to that lane. 

The path planning is done in two steps:
 
+ For each lane  the maximum safe speed is determined as the minimum between the speed limit and the speed of the vehicle in front of the ego vehicle. A JMT polynomial trajectory is genererate that will not violate jerk or acceleration limits [getPoly in EgoVehicle class]. This achieved by gradually increasing the trajectory time until limits are satisfied. The trajectories are always planned to have maximum allowed/safe speed in the lane. Target  Thus all trajectories generated are always conforming to dynamic limits.

+ The best lane is selected based on the cost of the trajectory generated in the first step. Thus 3 possible actions one for each road lane are evaluated based on the cost function. The cost function includes weighted: 
 + final speed
 + price to change lanes
 + collision penalty
 + occupancy of the lane.
 
The action with the lowest cost is chosen and executed for one cycle. In the next cycle the process is repeated and the new plan is created. 


To visualize the future trajectories points at each cycle the car movement is simulated up to 2 seconds ahead, ad the planned trajectory is appended to the points sent to the simulator. Thus it is possible to see the future plan of the car.

