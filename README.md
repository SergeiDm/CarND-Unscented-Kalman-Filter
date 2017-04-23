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
![CTRV model](https://github.com/SergeiDm/CarND-Unscented-Kalman-Filter/blob/master/illustrations/CTRV_model.png)

## Project files
The project includes the following folder/files:
- input_data – the folder with samples of measurement data from both LIDAR and RADAR.
- output_data - the folder with output files, produced by the project pipeline.
- Input_Output_Data_Format.txt - description input and output files formats.
- CMakeLists.txt - the file for building program.
- Source - the folder with c++ files with Extended Kalman Filter algorithm.

## Compiling and running the project
The project can be compiled and run by using, for example, the following command:

`mkdir build && cd build`

`cmake .. && make`

`./ExtendedKF path/to/input.txt path/to/output.txt`

## Results
The output results introduced in "output_data" folder. Additionaly, here are RMSE values (calculated for 2D position and 2D velocity) for data 1:
- 0.065165
- 0.0605294
- 0.5497
- 0.544984

and data 2:
- 0.185692
- 0.190208
- 0.47926
- 0.827925

