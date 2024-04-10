# Extended-Kalman-Filter-for-GPS-IMU-sensor-fusion

## Collecting Data
Collecting raw sensor data with data_collector.py
- odomOrt(X, Y, Z, W) : Orientation data with quaternion format.
- odomPos(X, Y, Z) : Position data (lat, long, alt).
- odomPoseCovariance : Covariance of position data.
- odomLinVel(X, Y, Z) : Linear velocity.
- odomAngVel(X, Y, Z) : Angular velocity.
- imuOrt(X, Y, Z, W) : Orientation data from IMU sensor.
- imuOrtCovariance : Orientation data covariance.
- imuAngVel(X, Y ,Z) : Angular velocity from IMU sensor.
- imuAngVelCovariance : Angular velocity covariance.
- imuLinAcc(X, Y, Z) : Linear acceleration.
- imuLinAccCovariance : Covariance of linear acceleration.

## Processing Data 
After collecting data we use LinVel and YawRate(angVelZ) to calculate u. And we transform Pos data to ndarray form for XTrue.

#### Simulation Parameters:
- INPUT_NOISE : Because of we work on simulation env we need to add noise on raw input data for simulate the real world. 
- GPS_NOISE : Because of we work on simulation env we need to add noise on raw gps data for simulate the real world.
- DT : Time tick.
- SIM_TIME : Simulation time.

