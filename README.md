# Project 6 - Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

Implemented a C++ Extended Kalman Filter program to fusion Lidar and Rador sensor data input. Lidar data can accurately define an object's location. Radar data provides not just location but also  speed data that can be used to track an object's velocity relative to the sensor. 

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: 
   `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt` 
   and
   `./ExtendedKF ../data/sample-laser-radar-measurement-data-2.txt output2.txt`

## Results

Root Mean Square Error (RMSE) Accuracy were calculated for each set of data:

sample-laser-radar-measurement-data-1.txt
* px	       0.0620209
* py	       0.057777
* vx	       0.521125
* vy	       0.53581

sample-laser-radar-measurement-data-2.txt
* px	       0.185638
* py	       0.190432
* vx	       0.481652
* vy	       0.830509

## Future Work

I'd like to generate your own radar and lidar data, using the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.





