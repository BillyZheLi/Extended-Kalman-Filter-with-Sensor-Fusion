# Extended Kalman Filter with Sensor Fusion

In this project a kalman filter is utilized to estimate the state of a moving object of interest with noisy lidar and radar measurements.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

The following images show screenshots of the EKF tracking the position and velocity of a car using Lidar and Radar data in the simulator, where the blue and red dots represent Lidar and Radar measurements respectively, and the green dots represent the estimated car position.

<kbd><img src="https://github.com/BillyZheLi/Extended-Kalman-Filter-with-Sensor-Fusion/blob/master/Docs/Capture.PNG" width="600" style="border:5px solid black"/></kbd> <kbd><img src="https://github.com/BillyZheLi/Extended-Kalman-Filter-with-Sensor-Fusion/blob/master/Docs/Capture1.PNG" width="300" style="border:5px solid black"/></kbd>

Tips for setting up your environment can be found in the classroom lesson for this project.

Here is a brief overview of what happens when you run the code files:

```Main.cpp``` reads in the data and sends a sensor measurement to ```FusionEKF.cpp```. ```FusionEKF.cpp``` takes the sensor data and initializes variables and updates variables. The Kalman filter equations are not in this file. ```FusionEKF.cpp``` has a variable called ```ekf_```, which is an instance of a KalmanFilter class. The ```ekf_``` will hold the matrix and vector values. The ```ekf_``` instance is used to call the predict and update equations. The KalmanFilter class is defined in ```kalman_filter.cpp``` and ```kalman_filter.h```. 

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Generating Additional Data

If you'd like to generate more radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

