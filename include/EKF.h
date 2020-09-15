#ifndef EKF_H
#define EKF_H

#include "../lib/eigen/Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;


class EKF{

  public:  

    // state vector
    Eigen::VectorXd x_;

    // input vector
    Eigen::VectorXd u_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;

  /**
   * Constructor
   */
  EKF() {};

  /**
   * Destructor
   */
  virtual ~EKF() {};

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param u_in Initial input
   * @param P_in Initial state covariance
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::VectorXd &u_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(const Eigen::VectorXd &u, double delta_T);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);



};

#endif 