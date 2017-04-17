/*
 * Oct. 5, 2016, David Z 
 *
 * imu interface for read the data from MEMS 
 *
 * */


#ifndef IMU_MEMS_H
#define IMU_MEMS_H

#include <string>
#include <vector>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <Eigen/Core>
// #include <Eigen/StdVector>

using namespace std; 

namespace Eigen{
  typedef Matrix<double, 6, 1> Vector6d; 
}

typedef std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d> > stdv_eigen_vector6d; 

class CImuMEMS
{
  public:
    CImuMEMS();
    virtual ~CImuMEMS(); 
    
    bool readImuData(string f);         // read all imu data
    void computePriorBias();            // compute prior imu bias 
    gtsam::NavState predictNext(int next_i); // predict pose between [curr_i, next_i]
    int m_curr_i;                       // id that preintegration has arrived 

    // predict [pj, vj] between [i, j] given [pi, vi]
    gtsam::NavState predictBetween(int i, int j, gtsam::NavState& state_i);    
    
    void resetPreintegrationAndBias();  // reset 
    void setState(gtsam::NavState& );   // set previous state

    int m_syn_start_id;                 // camera data syn with this id 
    gtsam::imuBias::ConstantBias m_prior_imu_bias;   // 
    stdv_eigen_vector6d mv_measurements;  // gx gy gz, ax ay az 
    float m_dt;                         // time difference between two measurements
    gtsam::NavState m_prev_state;       // imu prev state for preintegration 
    gtsam::PreintegrationType* mp_combined_pre_imu;  

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};





#endif
