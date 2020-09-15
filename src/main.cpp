#include <iostream>
#include "Landmarks.h"
#include "Measurements.h"
#include "Inputs.h"
#include "EKF.h"
#include "../lib/eigen/Eigen/Core"
#include "CSVWriter.h"


template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}



VectorXd localize3d_toa(VectorXd data, MatrixXd map){
/*       (matrix) DATA : The measured distances from landmarks (Nx1 matrix)
         (matrix) MAP  : The corresponding landmark map (Nx6 matrix)
         (matrix) POSE : The estimated pose (1x6 matrix)
         Reference:
           [1] A. H. Sayed et al., Network-based Wireless Location,
               IEEE Signal Processing Magazine, Vol. 24, No. 4, 2005
               URL: http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1458275
*/

MatrixXd H(map.rows()-1, map.cols());
VectorXd b(map.rows()-1);
VectorXd orig_dist(map.rows()-1);
VectorXd origin(3);
origin << map(0, 0), map(0, 1), map(0, 2);

for (unsigned int i =1; i < map.rows(); i++){
    H(i-1,Eigen::seq(0,2)) = map(i,Eigen::seq(0,2)) - map(0,Eigen::seq(0,2));
    orig_dist(i-1) = data(0);

}

for (unsigned int i =0; i < H.rows(); i++){

    b(i) = 0.5*(H(i,0)*H(i,0) + H(i,1)*H(i,1) + H(i,2)*H(i,2)- data(i+1)*data(i+1)+ orig_dist(i)*orig_dist(i));

}

VectorXd x = pseudoinverse(H)*b + origin;

return x ;

}


int main() {
    // Creating an object of Landmarks
    Landmarks landmarks("data/landmarks.csv");
    Measurements measurements("data/measurements.csv");
    Inputs inputs("data/inputs.csv"); 

    CSVWriter csv;
    u newU = inputs.getInputData();

    std::cout << newU.timestamp <<" " << newU.tau <<" " << newU.z_dd <<" " << newU.theta_d << std::endl;

    EKF ekf;

    Eigen::VectorXd x0(7);
    x0 <<  1500, 1500, -250, 0, 0, 0, 0;

    Eigen::VectorXd u0(3);
    u0 << newU.tau, newU.z_dd, newU.theta_d; 

    double t_old = newU.timestamp;

    
    MatrixXd P0 = MatrixXd::Identity(7, 7);  

    MatrixXd Q(7,7);

    Q << 0.1*0.1,     0,          0,  0, 0, 0,       0,
               0, 0.1*0.1,        0,  0, 0, 0,       0,
               0,     0,    0.1*0.1,  0, 0, 0,       0,
               0,     0,          0,  1, 0, 0,       0,
               0,     0,          0,  0, 1, 0,       0,
               0,     0,          0,  0, 0, 1,       0,
               0,     0,          0,  0, 0, 0,   0.001*0.001;

    MatrixXd R(3,3);

    R << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    // EKF Init
    ekf.Init(x0, u0, P0, R, Q);

        newU = inputs.getInputData();
    
    while (newU.timestamp != -1 ){
    
        std::cout <<"New Input "<< newU.timestamp <<" " << newU.tau <<" " << newU.z_dd <<" " << newU.theta_d << std::endl;

        double deltaT = newU.timestamp - t_old;
        Eigen::VectorXd u(3);
        u << newU.tau, newU.z_dd, newU.theta_d; 

        // EKF prediction step
        ekf.Predict(u, deltaT);
        
        std::cout << "Prediction" << ekf.x_ << std::endl;

        // get distance measurements from Landmarks
        measure newMeas = measurements.getSensorData();  
        VectorXd data = Eigen::Map<Eigen::VectorXd>(newMeas.norm_dists.data(),newMeas.norm_dists.size());

        // build Map of available landmarks
        MatrixXd map = landmarks.getMap(newMeas.id);

        // 3d localization
        VectorXd newObs = localize3d_toa(data, map);
        std::cout <<"New Observation "<< newU.timestamp <<" " << newU.tau <<" " << newU.z_dd <<" " << newU.theta_d << std::endl;
    
        // EKF correction step
        ekf.Update(newObs);
    
        std::cout << "Update" << ekf.x_ << std::endl;
        csv.newRow() << ekf.x_(0)<< ekf.x_(1)<< ekf.x_(2)<< ekf.x_(3)<< ekf.x_(4)<< ekf.x_(5)<< ekf.x_(6);
        // read new input
        newU = inputs.getInputData();
    }
    csv.writeToFile("State.csv", false);
    return 0;
}