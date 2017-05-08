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

The project uses Eigen library, so for compilation Eigen library files must be put in 'src' folder.

## Results
The output results are introduced in 'output_data' folder. Additionaly, here are RMSE values (calculated for 2D position and 2D velocity):

<img src="https://github.com/SergeiDm/CarND-Unscented-Kalman-Filter/blob/master/illustrations/RMSE.png" width="270" height="100"/>

It is possible to run the project in three different modes: considering laser only, with considering radar only, or with using both. There are 'use_laser_' and 'use_radar_' flags in 'ukf.cpp'. 

Here is comparison performance Laser, Radar and Sensor Fusion for [obj_pose-laser-radar-synthetic-input.txt](https://github.com/SergeiDm/CarND-Unscented-Kalman-Filter/blob/master/input_data/obj_pose-laser-radar-synthetic-input.txt):

<img src="https://github.com/SergeiDm/CarND-Unscented-Kalman-Filter/blob/master/illustrations/Sensor_fusion.png" width="270" height="100"/>

As shown in the picture above, Sensor fusion gives better results.

The model used in the project has hyperparameters which were adjusted:
- standard deviation longitudinal acceleration in m/s^2
- standard deviation yaw acceleration in rad/s^2

To control their adjustment 'Normalized Innovation Squared' check was used. NIS calculation is based on the difference between the predicted measurement and the actual measurement.

The following picture shows that NIS (for Sensor Fusion) is under chi-square 5% point most of the time:

<img src="https://github.com/SergeiDm/CarND-Unscented-Kalman-Filter/blob/master/illustrations/NIS.png" width="600" height="400"/>
