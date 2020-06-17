[//]: # (Image References)

[image1]: ./media/path_follow_state.png "Behavior Planner"

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Overview
The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.\
The car should go as close as possible to the 50 MPH speed limit, performing lane change to overtake slower cars.\
The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.

## Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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
---

## Algorithm

The algorithm is implemented in the class **Planner**.\
It provides four methods that must be used in the following order to solve the problem:
1. ***setState(...)*** this method updates the internal class attributes that regards the ego state;

2. ***findObstacles(...)*** this method analyzes the obstacles provided by the sensor fusion in order to select the ones closer to the ego vehicle;
3. ***chooseManeuvre()*** this method selects the best maneuver to perform according to the information provided by the previous two methods;
4. ***computeTrajectory(...)*** this method is the one that computes the waypoints that the control will follow in order to drive the ego vehicle along the track.

In order to have more flexibility and solve compiling errors, the helpers file has been reorganized separating the functions [declaration](./src/helpers.h) from their [implementations](./src/helpers.cpp).

To simplify the implementation of the ***computeTrajectory(...)*** methods, the [spline library](./src/spline.h) has been [downloaded](https://kluge.in-chemnitz.de/opensource/spline/) and used un this project.

### setState
This method (**line 50-81** of [planner.cpp](./src/planner.cpp)) updates the ego state according to the current inputs.
The ego state has the following fields:
* x: x coordinate on the map;
* y: y coordinate on the map;
* yaw: orientation on the map;
* s: curvilinear abscissa in the frenet system reference;
* d: lateral displacement in the frenet system reference;
* speed: ego speed;
* ref_speed: planner target speed;
* lane: ego lane;
* target_lane: planner target lane.

This method updates the following field of the ego state: **x**, **y**, **yaw**, **s**, **d**, **speed**, **lane**.
If the previous path has less than 2 points, the ego state will be set according to the current ego value, otherwise this method will perform an estimation of the ego state at the end of the previous path.
This policy guarantees a continuity in the planned trajectory.

### findObstacles
This method (**line 142-205** of [planner.cpp](./src/planner.cpp)) looks over all the obstacles provided by the sensor fusion and selects the five obstacles closest to the ego vehicle. In particular, it fills a class attribute that contains:
* the front left obstacles;
* the ahead obstacle;
* the front right obstacle;
* the rear left obstacle;
* the rear right obstacle;

Each obstacle has the following attributes:
* lane: current lane;
* d: current lateral displacement in the frenet system reference;
* s: predicted abscissa in the frenet system reference (the prediction is refered to the last point of the ego previous path);
* vx: current x component of the speed;
* vy: current y component of the speed;

### chooseManeuvre
This method (**line 207-270** of [planner.cpp](./src/planner.cpp)) is the core of the behavioral planning. It has the task to choose the best maneuver to perform to increase the ego speed and avoid collision with the other cars on the road (named obstacles in the class representation).
It implements a two state machine:
* ***PATH_FOLLOW***: this is the main state in which the class evaluates the best lane to follow to increase the ego speed and avoid the other cars (**line 209-253** of [planner.cpp](./src/planner.cpp));
* ***LANE_CHANGE***: this state adjusts the ego speed according to the front obstacle state in the destination lane. When the ego reaches the destination lane, it reset the state machine to the ***PATH_FOLLOW*** state(**line 255-268** of [planner.cpp](./src/planner.cpp)).

The following flow chart diagram shows the logic behind the ***PATH_FOLLOW*** state.

![alt text][image1]

When the `ego.lane != ego.target_lane`, the state will change in ***LANE_CHANGE***.

### computeTrajectory
This method (**line 272-369** of [planner.cpp](./src/planner.cpp)) updates the vehicle trajectory according to the previous computed path and the pair (target lane, target speed) provided by ***chooseManeuvre()*** method.\
It is divided into two main part.\
From **line 276** to **line 334**, it performs a reference frame conversion from cartesian to Frenet system and uses the spline to compute the waypoints according to the previous path and the target lane.\
From **line 336** to **line 368**, it updates the trajectory according to the previously computed waypoints and the target speed, and finally it converts back the trajectory from Frenet to cartesian reference frame.

## Results
The submitted program is able to drive on the highway path without hitting other cars and violating any comfort constraints.\
In this [video](./media/simulator_record.mp4) there is a demo in which the car drives for 5 miles.

## Open points
There are some aspects that could be improved from the current solution:
* the ***chooseManeuvre()*** method has a policy that evaluates the right lane change only if the left one is not allowed. A possible improvement could be to evaluate both lanes and chose the best solution. For example, choosing the empty lane or the one with the higher final speed.

* the ***findObstacles(...)*** method uses a kinematic prediction with constant speed and lane for the final obstacle state. A possible improvement could be the use of a model that considers the two speed components or a data-driven approach to make more realistic the obstacle behavior.
