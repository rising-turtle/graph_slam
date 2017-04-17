#include "imu_MEMS.h"
#include <fstream>
#include <iostream>

#define D2R(d) (((d)*M_PI)/180.)

#define DG (9.81)

// read i d/s (r*80/1092)*M_PI/180.
#define Gi2V(i) (((((float)(i*80.))/1092.)) * M_PI/180.)  

// read i /2.522mg 
#define Ai2V(i) ((i)*(0.002522)*DG)

using namespace gtsam; 

namespace{

static const double gkGyroSigma = D2R(3.6)/60;  // 3.6 degree Angular Random Walk
static const double gkAccelSigma = 0.1/60; // not shown in the specs, approximation

boost::shared_ptr<PreintegratedCombinedMeasurements::Params> Params()
{
  boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p 
    = PreintegratedCombinedMeasurements::Params::MakeSharedD(DG); 
  p->gyroscopeCovariance = gkGyroSigma * gkGyroSigma * I_3x3;
  p->accelerometerCovariance = gkAccelSigma * gkAccelSigma * I_3x3; 
  p->integrationCovariance = 0.0001 * I_3x3; 
  p->biasAccCovariance = 1e-8 * I_3x3; 
  p->biasOmegaCovariance = 1e-8 * I_3x3; 
  p->biasAccOmegaInt = 1e-5 * Matrix::Identity(6,6);
  return p; 
}

}
CImuMEMS::CImuMEMS():
  m_curr_i(0), 
  m_syn_start_id(0),
 m_dt(0.01) // 10 ms
// m_dt(0.01*0.33) // 3.3 ms
// m_dt(0.009) // 8 ms
{
  m_prev_state = NavState();
  boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = Params(); 
  mp_combined_pre_imu = new PreintegratedCombinedMeasurements(p, m_prior_imu_bias); 
  // cout <<"p.gravity() = "<<p->n_gravity<<endl;
}
CImuMEMS::~CImuMEMS()
{
  if(mp_combined_pre_imu) 
  {
    delete mp_combined_pre_imu; 
    mp_combined_pre_imu = 0;
  }
}

bool CImuMEMS::readImuData(string fname)
{
  ifstream inf(fname.c_str());
  if(!inf.is_open())
  {
    cerr<<"imu_MEMS.cpp: failed to open file "<<fname<<endl;
    return false;
  }
  
  int id1, id2, gx, gy, gz, ax, ay, az; 
  int last_id = -1;  
  int cnt = 0;

  vector<int> gr(3);
  Eigen::Vector6d m; 

  while(!inf.eof())
  {
    inf>>id1>>gr[0]>>gr[1]>>gr[2]>>ax>>ay>>az>>id2; 
    if(last_id != -1 && last_id > id1 && id1 == 1)
    {
      cout<<"gyro_euler.cpp: good syn start from frame: "<<id2-1<<endl;
      m_syn_start_id = cnt; // id2-1;
    }
    // printf("gyro_euler.cpp: input %d %d %d %d %d %d %d %d\n", id1, gr[0], gr[1], gr[2], ax, ay, az, id2);
    // gyro_xyz_.push_back(gr);
    
    m << Gi2V(gr[0]) , Gi2V(gr[1]) , Gi2V(gr[2]) , Ai2V(ax) , Ai2V(ay) , Ai2V(az); 
    mv_measurements.push_back(m);   

    ++ cnt;
    last_id = id1;
  }
  inf.close(); 
  printf("gyro_euler.cpp: retrieve %d , start_syn = %d\n", cnt, m_syn_start_id);
  // for(int i=syn_start_id_; i<gyro_xyz_.size(); i++ )
  // {
    // printf("gyro_euler.cpp: read gyro reading %d %d %d\n", gyro_xyz_[i][0], gyro_xyz_[i][1], gyro_xyz_[i][2]);
  // }
  return true;
}

void CImuMEMS::computePriorBias()
{
  double gx_bias = 0;   double gy_bias = 0;  double gz_bias = 0; 
  double ax_bias = 0;   double ay_bias = 0;  double az_bias = 0; 
  for(int i=0 ;i<m_syn_start_id; i++)
  {
    Eigen::Vector6d & m = mv_measurements[i]; 
    gx_bias += m(0); gy_bias += m(1); gz_bias += m(2); 
    ax_bias += m(3); ay_bias += m(4); az_bias += (m(5) + DG); 
  }
  if(m_syn_start_id > 0)
  {
    gx_bias /= m_syn_start_id; gy_bias /= m_syn_start_id; gz_bias /= m_syn_start_id; 
    ax_bias /= m_syn_start_id; ay_bias /= m_syn_start_id; az_bias /= m_syn_start_id; 
  }
  gtsam::Vector6 v6; 
  v6 << ax_bias , ay_bias , az_bias , gx_bias , gy_bias , gz_bias;
  m_prior_imu_bias = imuBias::ConstantBias(v6); 
  resetPreintegrationAndBias(); 
}

void CImuMEMS::resetPreintegrationAndBias()
{
  mp_combined_pre_imu->resetIntegrationAndSetBias(m_prior_imu_bias); 
}

NavState CImuMEMS::predictNext(int next_i)
{
  // Eigen::Matrix<double, 6, 1> imu = Eigen::Matrix<double, 6, 1>::Zero(); 
  for(int i=m_curr_i; i<next_i; i++)
  {
    if(i+m_syn_start_id  >= mv_measurements.size())
    {
      printf("%s i+m_syn_start_id >= mv_measurements.size()\n", __FILE__); 
      break; 
    }
    Eigen::Vector6d& imu = mv_measurements[i+m_syn_start_id]; 
    mp_combined_pre_imu->integrateMeasurement(imu.tail<3>(), imu.head<3>(), m_dt); 
  }
  m_curr_i = next_i; 
  return mp_combined_pre_imu->predict(m_prev_state, m_prior_imu_bias); 
}

gtsam::NavState CImuMEMS::predictBetween(int i, int j, gtsam::NavState& state_i)
{
  resetPreintegrationAndBias(); 
    for(int m=i; m<j; m++)
    {
      if(m+m_syn_start_id  >= mv_measurements.size())
      {
        printf("%s m+m_syn_start_id >= mv_measurements.size()\n", __FILE__); 
        break; 
      }
      Eigen::Vector6d& imu = mv_measurements[m+m_syn_start_id]; 
      mp_combined_pre_imu->integrateMeasurement(imu.tail<3>(), imu.head<3>(), m_dt); 
    }
  return mp_combined_pre_imu->predict(state_i, m_prior_imu_bias); 
}

void CImuMEMS::setState(NavState& ns)
{
  m_prev_state = ns;
}

