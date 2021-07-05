# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

## Description of the model for lane change maneuvers and path generation
Path generation algorithm consists of the following steps:
1. Analysis of the road situation
All the cars from the fusioned list of detected cars are analysed and the closest car in the ego lane is checked with regard to the distance towards it from the ego vehicle.
If the distance to the closest car in the ego lane is less than the safety distance (can be set in the helpers.h as a multiplier to the ego velocity in kmh using parameter SAFETY_DISTANCE_FRONT), then the following decisions can be made:
- if the car drives in the leftmost lane, then lane change to the right is considered. Safety of the lane change is checked in the function 'lane_is_busy', found in helpers.h. Separate configuration values can be set to ensure safety distance during lane change: SAFETY_DISTANCE_FRONT_OVERTAKE and SAFETY_DISTANCE_BACK_OVERTAKE. The first parameter uses ego car's speed to determine if the safety towards the front vehicle is ensured after the lane change. The second parameter is used to leave enough safe space behind the ego car after the lane change. For this, closest vehicle on the back in the target lane is found and it's velocity is used for safety distance calculation
- if the car drives in the rightmost lane, then lane change to the left is considered after checking with the 'lane_is_busy' function
- if the car drives in one of the middle lanes (more than 3-lanes highways are supported), then either lane change to the left or right is considered. Decision is made using a cost function based on the velocity of the next closest vehicle in a possible target lane. 'lane_speed' function from helpers.h is used to calculate it for a given lane
2. After situation is analysed and decisions are made, target velocity of the ego vehicle is adjusted:
- if the ego car sees another car too close in front, braking maneuver is started and ego velocity is decreased
- if the road ahead is clean, ego velocity increases (almost) up to the speed limit. Actual limit is specified as SPEED_LIMIT in the helpers.h
3. As a next step, additional check is executed to see if the car drives long and fast enough in a lane without seeing obstacles. In this situation, the ego car initiates lane change maneuver to the right (if the right lane is not busy). Activation of this maneuver is controlled by the parameter MIN_CRUISING_SPEED defined in the helpers.h. If the value for this parameter is set to something higher than the speed limit - it never engages since the car never goes over the speed limit. Such behaviour implements a concept of "Rechtsfahrgebot", which is required in Germany but is also valid in many other countries
4. Next, lane change maneuver counter is checked to avoid too often lane change maneuvers (can result in too high jerk values). MIN_LANE_CHANGE_INTERVAL in helpers.h defines minimal interval between lane change maneuvers.
5. After all the above checks and decisions are done, trajectory generation starts:
- Spline library by Tino Kluge is used for this (see spline.h for more details)
- Spline input is initialized with the ego car position OR last two points of the unfinished path, followed by three additional points with bigger intervals (sparse points)
- Spline generation is done in the car coordinates. To achieve this, input points for the spline are first converted into the car coordinate system by shifting to the reference point and rotating with the reference yaw value
- After spline generation is done, additional points for the car trajectry are calculated:
  - x coordinate is iterated using fixed time intervals and target velocity of the ego car
  - y coordinate is generated by the spline library for given x coordinates
  - generated points are converted back to the global coordinate system by first rotating around reference yaw ahngle and then shifting using the reference point coordinates
  - resulting vectors of the travel points for the ego car are always filled up to 50 points
- when the vectors of the points are filled up, they're transferred back to the simulator
