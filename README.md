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


### Reflection

To begin the path generation model, the code lines 21-52 of the main() function in <em>/src/main.cpp</em> starts by parsing the map data from the provided <em>/data/highway_map.csv</em> file. This contains waypoint information of the center of the track, for its entire length. This data will be used to zero the d-coordinate of the Frenet coordinate system for lane position identification. The waypoint information is stored in individual vectors for the x, y, s, dx, and dy values.

Next, on code lines 55-63, initial variables and constants were defined to constrain the model maximums for vehicle speed limit (SPEED_LIMIT), maximum acceleration (MAX_ACCEL), maximum jerk (MAX_JERK), update rate (UPDATE_RATE). Variables were defined for the acceleration increment (max_accel_increment) that would provide a jerk value that is within the MAX_JERK threshold, a safe following distance (safe_following_distance) to define the distance from the next vehicle that should be maintained to prevent forward collisions, the reference velocity (ref_v), which will store the vehicle’s target velocity, and the initial lane to begin in (lane), which is the middle lane on the right side of the road (index 1). These initial values are passed to the web socket hub’s “onMessage” event handler on code line 65.

Inside of the “onMessage” event handler, the ego vehicle’s state and path values, as well as sensor fusion data are parsed from the incoming stream on code lines 87-105. The sensor fusion data is analyzed next on code lines 118-168. Boolean flags are defined to reflect the existence of objects around the ego vehicle that will affect the decision of the path planner. Each object in the sensor_fusion vector is iterated through to determine:

-	Which lane the object is in, calculated by the d-coordinate
-	The object’s velocity, s-coordinate, and the distance between the ego vehicle and the object
-	If the object is in the same lane as the ego vehicle, and if the ego vehicle is within a specified following distance range.
-	If the object is in a lane to the left or right of the ego vehicle, within a specified distance drawn fore and aft of the object that would create an unsafe lane change condition

Once the flags have been set, they can be used to decide the next behavior of the ego vehicle. This is performed on code lines 170-195. One of four behaviors can be selected:
-	Change lanes to the left – If the object is in the current lane and the left lane is clear
-	Change lanes to the right – If the object is in the current lane and the right lane is clear
-	Decrease speed – If the object is in the current lane, but the left and right lanes are blocked
-	Increase speed up to the speed limit – If there is no object in the current lane within the “safe following distance”

Adjustments to the vehicle speed are clamped to the maximum acceleration and jerk limits set forth by the project, which are 10 m/s^2 and 10 m/s^3, respectively. The vehicle will also move back to the center lane when the center lane is clear to allow for additional freedom in lane selection upon reaching the next object, and to allow for safer driving conditions as the left lane is typically used for passing.

After the next behavior of the vehicle has been decided, it can then be implemented by the path planner. This is performed on code lines 203-318. A path must be established for the vehicle to visit at the specified update rate of 50Hz. The “Cubic Spline Interpolation” library (https://kluge.in-chemnitz.de/opensource/spline) was utilized because it provided a simple method of generating a smooth path that is desired for the vehicle to follow.

Vectors of x and y waypoints are created and filled with two points from the vehicle’s previous path (code lines 203-238). If the path does not contain two previous path points, a new trajectory of the vehicle can be calculated from the ego vehicle’s current position and heading angle. The new trajectory’s starting and ending coordinates can then be used.

Next on code lines 242-253, evenly spaced points are created in front of the vehicle at a distance of 30, 60, and 90 meters. These are generated in the Frenet coordinate system, so the s- and d-coordinates are then passed to the getXY() function to perform a shift and a rotation that produces x- and y-coordinates for the simulator.

A spline is created, and the points vectors are then passed to the spline set_points() function (code lines 267-268). On code lines 271-278, vectors are created for the path planner x and y-values, and all of the previous path points are appended to them. It is then necessary to generate additional points for the path planner to follow, based upon the selected behavior. 

Code lines 293-298 determine the velocity of the vehicle for each point. It is helpful to perform the velocity adjustment here, per point, as it allows for smoother acceleration transitions. On code lines 310-317, the new velocity and lane position points are converted to x and y-coordinates, and then added to the next_x_vals and next_y_vals vectors for a total of 50 points. These vectors are then passed to the simulator to follow.

After multiple simulations, the vehicle is able to meet the rubric criteria for:
-	Driving at least 4.32 miles without an incident
-	Driving according to the speed limit (never exceeding)
-	Driving within maximum acceleration and jerk criteria
-	Driving without collision
-	The ability for the vehicle to maintain its lane
-	The ability for the vehicle to change lanes


## Future Plans

When additional time can be committed to this project, there is some desired functionality that would be beneficial to implement:
-	Adjust vehicle speed proportionally to smooth the approach to a vehicle ahead and ultimately speed match at a safe following distance for the target speed.
-	Implement cost functions for lane change decisions and prevent lane change confusion when vehicles are all travelling at the same speed.
-	Determine a safe vehicle speed for lane changes to prevent forward collisions
