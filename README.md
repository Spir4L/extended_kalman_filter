# extended_kalman_filter

Implementation of a sensor fusion localization with Extended Kalman Filter(EKF).

# Problem definition

Consider an aircraft with dynamics as shown in equation 


that has a sensor (at its origin) that is able to determine the <img src="https://render.githubusercontent.com/render/math?math=x">, <img src="https://render.githubusercontent.com/render/math?math=y"> and <img src="https://render.githubusercontent.com/render/math?math=z"> distances as well as the unique landmark
number of landmarks in the environment. The landmarks are stationary in the inertial frame in which the aircraft flies.

The position and unique ID of the landmarks are given in the “landmarks.csv” file. 

The measurements to each landmark during operation is given in “measurements.csv” file. The measurement in the file is
left blank if the landmark is outside of the sensors range.



The
aircraft has an initial position of ,  , , zero translational rates and zero  within
the inertial coordinate system in which the aircraft operates. It is assumed
that the aircraft navigates in an inertial NED coordinate system with the
reference point at position (0,0,0).

# Filter design
