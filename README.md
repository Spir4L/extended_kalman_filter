# extended_kalman_filter

Implementation of a sensor fusion localization with Extended Kalman Filter(EKF).

# Problem definition

Consider an aircraft with dynamics as shown in equation 

![Image of equation](/data/Equation.gif)


that has a sensor (at its origin) that is able to determine the x,y and z distances as well as the unique landmark
number of landmarks in the environment. The landmarks are stationary in the inertial frame in which the aircraft flies.

The position and unique ID of the landmarks are given in the “landmarks.csv” file. 

The measurements to each landmark during operation is given in “measurements.csv” file. The measurement in the file is
left blank if the landmark is outside of the sensors range.

The aircraft has an initial position of 1500,  1500, -250, zero translational rates and zero  within
the inertial coordinate system in which the aircraft operates. It is assumed
that the aircraft navigates in an inertial NED coordinate system with the
reference point at position (0,0,0).

# Localization for Observations
The algorithm used to obtain the current observation of the x,y,z position is implemented following the TOA (Time of Arrival) algorithm.

Reference:
           [1] A. H. Sayed et al., Network-based Wireless Location,
               IEEE Signal Processing Magazine, Vol. 24, No. 4, 2005
               URL: http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1458275


# Filter design
The designed filter is an Extended Kalman Filter implemented as follows:

- Prediction

<img src="https://render.githubusercontent.com/render/math?math=x_{Pred} = Fx_{k-1}+Bu_{k-1}">
<img src="https://render.githubusercontent.com/render/math?math=P_{Pred} = J_FP_{k-1} J_F^T + Q">
  
- Update

 <img src="https://render.githubusercontent.com/render/math?math=z_{Pred} = Hx_{Pred}">
 <img src="https://render.githubusercontent.com/render/math?math=y = z - z_{Pred}">
 <img src="https://render.githubusercontent.com/render/math?math=S = J_H P_{Pred}.J_H^T + R">
 <img src="https://render.githubusercontent.com/render/math?math=K = P_{Pred}.J_H^T S^{-1}">
 <img src="https://render.githubusercontent.com/render/math?math=x_{k} = x_{Pred} + Ky">
 <img src="https://render.githubusercontent.com/render/math?math=P_{k} = ( I - K J_H) P_{Pred}">
