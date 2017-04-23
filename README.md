# CarND-Unscented-Kalman-Filter
## Project Description
This project illustrates Sensor Fusion flow based on an Unscented Kalman Filter for tracking an object.The Flow includes prediction and updating steps. The last one uses data from LIDAR and RADAR.

This project uses constant turn rate and velocity magnitude model (CTRV), which state vector consists of:
- px - x-position
- py - y-position
- v - speed
- psi - yaw angle
- psi dot - yaw rate

The diagram of the CTRV model (source: http://www.udacity.com/):
<img src="https://github.com/SergeiDm/CarND-Unscented-Kalman-Filter/blob/master/illustrations/CTRV_model.png" width="300" height="300"/>

## Project files
The project includes the following folder/files:
- illustrations - the folder with pictures for README.md.
- input_data – the folder with samples of measurement data from both LIDAR and RADAR.
- output_data - the folder with output files, produced by the project pipeline.
- src - the folder with c++ files with Unscented Kalman Filter algorithm.
- CMakeLists.txt - the file for building program.

## Compiling and running the project
The project can be compiled and run by using, for example, the following command:

`mkdir build && cd build`

`cmake .. && make`

`./UnscentedKF path/to/input.txt path/to/output.txt`

The project uses Eigen library, so for compilation Eigen library files must be put to in 'src' folder.

## Results
The output results introduced in "output_data" folder. Additionaly, here are RMSE values (calculated for 2D position and 2D velocity):
<img src="https://github.com/SergeiDm/CarND-Unscented-Kalman-Filter/blob/master/illustrations/RMSE.png" width="200" height="100"/>
