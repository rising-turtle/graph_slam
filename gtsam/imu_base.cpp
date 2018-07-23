
#include "imu_base.h"
#include <iostream>
#include <fstream>

using namespace gtsam; 

CImuBase::CImuBase(double delta_t, gtsam::imuBias::ConstantBias prior_bias) : 
  m_curr_i(0),
  m_syn_start_id(0),
  m_dt(delta_t), 
  m_prior_imu_bias(prior_bias)
{
  m_prev_state = NavState(); 
  // boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = getIMUParams(); 
  // mp_combined_pre_imu = new PreintegratedCombinedMeasurements(p, m_prior_imu_bias); 
  m_prev_imu_bias = m_prior_imu_bias; 
}
CImuBase::~CImuBase()
{
  if(mp_combined_pre_imu)
  {
    delete mp_combined_pre_imu; 
    mp_combined_pre_imu = 0; 
  }
}


double CImuBase::getLastTimeStamp()
{
  if(mv_timestamps.size() > 0)
  {
    return mv_timestamps[mv_timestamps.size()-1]; 
  }

  cerr << __FILE__<<" at "<<__LINE__ << " no imu timestamps available!"<<endl;
  return 0; 
}
bool CImuBase::predictNextFlag(double t, gtsam::NavState& s)
{
  int index = findIndexAt(t); 
  if(index < 0)
  {
     cerr<<__FILE__<<" failed to predictNext given t = "<<t<<endl;
    return false; 
  }
  return predictNextFlag(index, s);
}

gtsam::NavState CImuBase::predictNext(double t)
{
  gtsam::NavState ret; 
  int index = findIndexAt(t); 
  if(index < 0)
  {
    cerr<<__FILE__<<" failed to predictNext given t = "<<t<<endl;
    return ret; 
  }
  
  return predictNext(index); 
}

bool CImuBase::predictNextFlag(int next_t, gtsam::NavState& s)
{
  if(next_t < 0){
    return false; 
  }
  s= predictNext(next_t); 
  return true; 
}

gtsam::NavState CImuBase::predictNext(int next_i )
{
  // Eigen::Matrix<double, 6, 1> imu = Eigen::Matrix<double, 6, 1>::Zero(); 
  for(int i=m_syn_start_id + m_curr_i; i<m_syn_start_id + next_i; i++)
  {
    if(i  >= mv_measurements.size())
    {
      printf("%s i >= mv_measurements.size()\n", __FILE__); 
      break; 
    }
    Eigen::Vector6d& imu = mv_measurements[i]; 
    mp_combined_pre_imu->integrateMeasurement(imu.tail<3>(), imu.head<3>(), m_dt); 
  }
  m_curr_i = next_i; 
  return mp_combined_pre_imu->predict(m_prev_state, m_prev_imu_bias); 
}

void CImuBase::resetPreintegrationAndBias(gtsam::imuBias::ConstantBias bias)
{
  // m_prior_imu_bias = bias; 
  m_prev_imu_bias = bias; 
  mp_combined_pre_imu->resetIntegrationAndSetBias(bias); 
}

void CImuBase::resetPreintegrationAndBias()
{
  mp_combined_pre_imu->resetIntegrationAndSetBias(m_prev_imu_bias); 
}

bool CImuBase::readImuData(string fname)
{
  // TODO: 
  printf("%s readImuData not implemented\n", __FILE__); 
  return false; 
}

void CImuBase::setStartPoint(double t)
{
  m_syn_start_id = 0; 
  int index = findIndexAt(t); 
  if(index < 0) 
  {
    cerr << __FILE__<<" failed to synchronize with timestamp t = "<<t<<endl; 
    return ; 
  }

  // initialize Gravity 
  // initializeGravity(index);
  m_syn_start_id = index; 
}

int CImuBase::findIndexAt(double t)
{
  if(mv_timestamps.size() != mv_measurements.size())
  {
    cerr << __FILE__<< " something is wrong: mv_timestamps.size() != mv_measurements.size()"<<endl;
    return -1; 
  }
  
  int s = 0; 
  int e = mv_timestamps.size()-1; 
  int i;

  for(i=s; i + m_syn_start_id <=e; i++)
  {
    if(mv_timestamps[i+m_syn_start_id] > t)
    {
      if(i>=1)
      {
        if(mv_timestamps[i + m_syn_start_id] - t > t - mv_timestamps[i + m_syn_start_id-1] && i > 0)
          return i-1;
        else 
          return i; 
      }
      cout <<__FILE__<< " timestamp[0] > t "<<endl; 
      return i; 
    }
  }

  cout <<__FILE__<<" timestamp[-1] < t = "<<std::fixed<<t<<" i="<< i<<" m_syn_start_id : "<< m_syn_start_id <<"size: "<<e<<" last timestamp : "<<std::fixed<< mv_timestamps[e] <<endl; 
  // return mv_timestamps.size()-1 - m_syn_start_id;
  return -1; 
}

gtsam::NavState CImuBase::predictBetween(int i, int j, gtsam::NavState& state_i, gtsam::imuBias::ConstantBias bias_i)
{
  resetPreintegrationAndBias(bias_i); 
    for(int m=i; m<j; m++)
    {
      if(m  >= mv_measurements.size())
      {
        printf("%s m >= mv_measurements.size()\n", __FILE__); 
        break; 
      }
      Eigen::Vector6d& imu = mv_measurements[m]; 
      mp_combined_pre_imu->integrateMeasurement(imu.tail<3>(), imu.head<3>(), m_dt); 
    }
  return mp_combined_pre_imu->predict(state_i, m_prev_imu_bias); 
}

gtsam::NavState CImuBase::predictBetween(double sti, double stj, gtsam::NavState& state_i, gtsam::imuBias::ConstantBias bias_i)
{
  // TODO
  printf("%s predictBetween(sti, stj) not implemented, return identity\n"); 
  gtsam::NavState ret; 
  return ret; 
}

void CImuBase::setState(gtsam::NavState& ns)   // set previous state
{
  m_prev_state = ns; 
}

void CImuBase::getNormalizedAcc(double& ax, double& ay, double& az)
{
    if(m_syn_start_id <= 0)
	return getNormalizedAcc(1, ax, ay, az); 
    return getNormalizedAcc(m_syn_start_id, ax, ay, az); 
}

void CImuBase::getNormalizedAcc(int index, double& ax, double& ay, double& az)
{
  if(index > mv_measurements.size()) index = mv_measurements.size(); 
  if(index <= 0 )
  {
    cerr << __FILE__<<" getNormalizedAcc at index = "<<index<<endl;
    return ;
  }
  double wx = 0, wy = 0, wz = 0; 
  for(int i=0; i<index; i++)
  {
   // debug m [gx gy gz ax ay az]
    Eigen::Vector6d& m = mv_measurements[i]; 
    // gx += m(3); gy += m(4); gz += m(5); 
    // wx += m(0); wy += m(1); wz += m(2); 
    wx += m(3); wy += m(4); wz += m(5); 
  }
  wx /= (double)(index);  
  wy /= (double)(index); 
  wz /= (double)(index); 
  double norm = sqrt(wx*wx + wy*wy + wz*wz); 
  ax = wx/norm; ay = wy/norm; az = wz/norm; 
  return ; 
}

void CImuBase::initializeGravity(int index)
{
  if(index > mv_measurements.size()) index = mv_measurements.size(); 
  if(index <= 0 )
  {
    cerr << __FILE__<<" initializeGravity at index = "<<index<<endl;
    return ;
  }
  double gx = 0, gy = 0, gz = 0; 
  for(int i=0; i<index; i++)
  {
    Eigen::Vector6d& m = mv_measurements[i]; 
    // gx += m(3); gy += m(4); gz += m(5); 
    gx += m(0); gy += m(1); gz += m(2); 
  }
  
  gx /= (double)(index);  
  gy /= (double)(index); 
  gz /= (double)(index); 
  
  double wx, wy, wz; 
  getNormalizedAcc(index, wx, wy, wz); 

  cout <<__FILE__<<" in initializeGravity() compute gx = "<<gx<<" gy = "<<gy<<" gz = "<<gz<<endl;
  // resetGravity(gx, gy, gz); 
  
  Vector3 biasAcc(gx, gy, gz); 
  Vector3 biasGyro(wx, wy, wz); 
  m_prior_imu_bias = gtsam::imuBias::ConstantBias(biasAcc, biasGyro); 
  m_prev_imu_bias = m_prior_imu_bias; 

  return ;
}

void CImuBase::resetGravity(double gx, double gy, double gz)
{
  Vector3 g(gx, gy, gz); 
  getParam()->n_gravity = g; 
  return; 
}

boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> CImuBase::getParam()
{
  static boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p 
    = PreintegratedCombinedMeasurements::Params::MakeSharedD(9.71); // 9.71 0 
  return p;   
}

boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> CImuBase::getIMUParams()
{
  cerr <<__FILE__<<" what? should never arrive here !"<<endl;
  return 0;   
}
