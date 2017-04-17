/*
 * Nov.7, 2016 David Z 
 *
 * Imu Base class, that functions all the IMU preintegration process 
 *
 * */


#ifndef IMU_BASE_H
#define IMU_BASE_H

#include <string>
#include <vector>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <Eigen/Core>
#include <cmath>
// #include <Eigen/StdVector>

using namespace std; 

#define D2R(d) (((d)*M_PI)/180)
#define R2D(r) (((r)*180)/M_PI)

namespace Eigen{
  typedef Matrix<double, 6, 1> Vector6d; 
}

typedef std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d> > stdv_eigen_vector6d; 

class CImuBase
{
  public:
    CImuBase(double delta_t, gtsam::imuBias::ConstantBias prior_bias);
    virtual ~CImuBase(); 
    
    virtual void setStartPoint(double t);       // the first imu measurement synchronized with camera data 
    virtual bool readImuData(string f);         // read all imu data
    virtual int findIndexAt(double t);          // find imu index at t
    virtual bool predictNextFlag(double t, gtsam::NavState&) ;   // return failed, if t cannot be matched 
    virtual bool predictNextFlag(int next_i, gtsam::NavState& ); // return failed, if next_i cannot be found 
    virtual gtsam::NavState predictNext(int next_i); // predict pose between [curr_i, next_i]
    virtual gtsam::NavState predictNext(double t);    // 
    int m_curr_i;                       // id that preintegration has arrived 
    double getLastTimeStamp();          // get the last timestamp stored in the mv_timestamps
    void resetGravity(double gx, double gy, double gz);     // reset gravity vector 
    void initializeGravity(int id);     // assume in the initial period, imu stay steady 

    // generate imu parameters first 
    static boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> getParam();
    virtual boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> getIMUParams() = 0; // this function must be done 

    // predict [pj, vj] between [i, j] given [pi, vi]
    virtual gtsam::NavState predictBetween(int i, int j, gtsam::NavState& state_i, gtsam::imuBias::ConstantBias bias_i);    
    virtual gtsam::NavState predictBetween(double sti, double stj, gtsam::NavState& state_i, gtsam::imuBias::ConstantBias bias_i); 

  virtual void resetPreintegrationAndBias(gtsam::imuBias::ConstantBias bias);  // reset 
  virtual void resetPreintegrationAndBias(); 
    void setState(gtsam::NavState& );   // set previous state

    int m_syn_start_id;                 // camera data syn with this id 
    gtsam::imuBias::ConstantBias m_prior_imu_bias;   // prior bias 
    gtsam::imuBias::ConstantBias m_prev_imu_bias;    // imu bias for current state
    stdv_eigen_vector6d mv_measurements;  // gx gy gz, ax ay az 
    vector<double> mv_timestamps;         // timestamps for imu measurement 
    float m_dt;                         // time difference between two measurements
    gtsam::NavState m_prev_state;       // imu prev state for preintegration 
    gtsam::PreintegrationType* mp_combined_pre_imu;  

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
