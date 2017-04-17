

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <fstream>
#include <cmath>
// #include "std_msgs/Float32MultiArray.h"

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <iostream>

using namespace gtsam; 
using namespace std; 


#define D2R(d) (((d)*M_PI)/180)

string fname("/home/david/work/data/sr4k/imu_bdat/etas_f5/imu_v100.log"); 
ros::Publisher euler_pub; 

class A{
  
public:
  static boost::shared_ptr<PreintegratedCombinedMeasurements::Params> getParam(); 
};



int main(int argc, char* argv[])
{
   A::getParam();
   ros::init(argc, argv, "test_imu_vn100"); 
   ros::NodeHandle n; 
   // euler_pub = n.advertise<std_msgs::Float32MultiArray>("/euler_msg", 10); \

   A a; 
   a.getParam()->n_gravity(2) = 1; 

   cout <<"Hello world"<<endl;

  return 0; 
}

boost::shared_ptr<PreintegratedCombinedMeasurements::Params> A::getParam()
{
  // use the sensor specs to build the noise model for the IMU factor 
  double accel_noise_sigma = 0.0003924; 
  double gyro_noise_sigma = 0.000205689024915; 
  double accel_bias_rw_sigma = 0.004905; 
  double gyro_bias_rw_sigma = 0.000001454441043; 
  Matrix33 measured_acc_cov = Matrix33::Identity(3,3)*pow(accel_noise_sigma, 2); 
  Matrix33 measured_omega_cov = Matrix33::Identity(3,3)*pow(gyro_noise_sigma, 2); 
  Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8; // error committed in integration position from veloctiy
  Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma, 2);
  Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma, 2);
  Matrix66 bias_acc_omega_int = Matrix::Identity(6,6) * 1e-5; // error in the bias used for preintegration 
  
  static boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0); 
  p->accelerometerCovariance = measured_acc_cov; 
  p->integrationCovariance = integration_error_cov; 
  p->gyroscopeCovariance = measured_omega_cov; 
  p->biasAccCovariance = bias_acc_cov; 
  p->biasOmegaCovariance = bias_omega_cov; 
  p->biasAccOmegaInt = bias_acc_omega_int; 

  return p; 
}


