/*
 *  Nov. 7, 2016, David Z
 *
 * interface for imu_vn100 
 *
 * */

#ifndef IMU_VN100_H
#define IMU_VN100_H

#include "imu_base.h"

namespace Eigen{
  typedef Matrix<double, 3, 1> Vector3d; 
}

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > stdv_eigen_vector3d;

class CImuVn100 : public CImuBase
{
  public:
    CImuVn100(double dt, gtsam::imuBias::ConstantBias prior_bias); 
    virtual ~CImuVn100();
    virtual void setStartPoint(double t); 
    virtual bool readImuData(string f);         // read all imu data
    bool getRPYAt(double t, Eigen::Vector3d& rpy);  // 
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> getIMUParams(); // parameters for VN100 
    
    void resetGravity(int n = 1);  // For the first frame, camera may not be vertical to the ground, use the first n readings to estimate initial gravity  

    stdv_eigen_vector3d mv_rpy; // rpy 
    Eigen::Vector3d mp_ini_rpy; // initial rpy 

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};



#endif
