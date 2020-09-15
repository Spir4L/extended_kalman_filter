#include "EKF.h"

#define PI 3.14159265


void EKF::Init(VectorXd &x_in,VectorXd &u_in, MatrixXd &P_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  u_ = u_in;
  P_ = P_in;
  R_ = R_in;
  Q_ = Q_in;
}


void EKF::Predict(const VectorXd &u, double delta_T) {

  u_ = u;

  //Use the state using the state transition matrix
  MatrixXd F_(7,7);
  F_  <<   1, 0, 0, delta_T,       0,        0,    0,
           0, 1, 0,       0, delta_T,        0,    0, 
           0, 0, 1,       0,       0,  delta_T,    0,
           0, 0, 0,       1,       0,        0,    0, 
           0, 0, 0,       0,       1,        0,    0,
           0, 0, 0,       0,       0,        1,    0, 
           0, 0, 0,       0,       0,        0,    1;
  
  MatrixXd B_(7,3); 

  B_ <<   0 ,                                     0,          0, 
          0 ,                                     0,          0,
          0 ,                                     0,          0,
          delta_T*cos(x_(6)),                     0,          0,
          delta_T*sin(x_(6)),                     0,          0,
          0 ,                               delta_T,          0,
          0 ,                                     0,    delta_T;


  x_ = F_ * x_ + B_ * u_;   

  MatrixXd jF(7,7);
  jF << 1, 0, 0, delta_T,         0,        0,                                      0,
        0, 1, 0,       0,   delta_T,        0,                                      0,
        0, 0, 1,       0,         0,  delta_T,                                      0,
        0, 0, 0,       1,         0,        0,              -delta_T*sin(x_(6))*u_(0),
        0, 0, 0,       1,         0,        0,               delta_T*cos(x_(6))*u_(0),
        0, 0, 0,       0,         0,        1,                                      0,
        0, 0, 0,       0,         0,        0,                                      1;


  //Update the covariance matrix using the process noise and state transition matrix
  MatrixXd jFt = jF.transpose();

  P_ = jF * P_ * jFt + Q_;

}

void EKF::Update(const VectorXd &z) {

  MatrixXd H_(3,7); 
  H_ << 1, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0;

  MatrixXd jH(3,7);
  jH << 1, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0;
                   

  MatrixXd jHt = H_.transpose();

  VectorXd y = z - H_ * x_;

  MatrixXd S = jH * P_ * jHt + R_;
  
  MatrixXd K =  P_ * jHt* S.inverse();

  //Update State
  x_ = x_ + (K * y);

  //Update covariance matrix
  long x_size = x_.size();
  
  MatrixXd I = MatrixXd::Identity(x_size, x_size);  
  
  P_ = (I - K*jH) * P_;

}
