
#include "imu_vn100.h"
#include <fstream>

using namespace std; 
using namespace gtsam; 

#define D2R(d) (((d)*M_PI)/180.)

CImuVn100::CImuVn100(double dt, gtsam::imuBias::ConstantBias prior_bias) : 
  CImuBase(dt, prior_bias)
{
  // mp_ini_rpy = new Eigen::Vector3d; 
  mp_ini_rpy = Eigen::Matrix<double , 3, 1>::Zero(); 
  boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = getIMUParams(); 
  mp_combined_pre_imu = new PreintegratedCombinedMeasurements(p, m_prior_imu_bias); 
}

CImuVn100::~CImuVn100()
{
  // if(mp_ini_rpy) delete mp_ini_rpy; 
}

boost::shared_ptr<PreintegratedCombinedMeasurements::Params> CImuVn100::getIMUParams()
{
  boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p  = CImuBase::getParam(); 
  static bool b_once = true; 
  if(b_once)
  {
    float fps = 200;  // 40 ,200
    float dt = 1./fps;
    int hour = 3600; 
    double g = 9.81; // m/s^2 
    double gyro_noise_density = 0.0035; // '/s/sqrt(hz) -> rad / s / sqrt(Hz)
    double accel_noise_density = 0.14; // 0.14  mg/√Hz  -> m / s^2 / sqrt(Hz)
    double gyro_bias_stability = 10;  // < 10'/hr * sqrt(Hz) -> rad / s^2 / sqrt(Hz)
    double accel_bias_stability = 0.04; // 0.04 mg * sqrt(Hz) -> m / s^3 / sqrt(Hz)
    
    // use the sensor specs to build the noise model for the IMU factor 
    double accel_noise_sigma = accel_noise_density * 1e-3 * g ; // 100 delta_a (m/(s^2*sqrt(HZ))) ~1.4e-3  0.0003924; 
    double gyro_noise_sigma =  D2R(gyro_noise_density) ; // delta_g (rad/(s*sqrt(HZ))) ~0.6e-4 0.000205689024915; 
    double accel_bias_rw_sigma = (accel_bias_stability*1e-3*g)*sqrt(fps);  // 100 delta_ba (m/(s^3*sqrt(hz))) ~5.5e-3 0.004905; 
    double gyro_bias_rw_sigma = (D2R(gyro_bias_stability)/hour)*sqrt(fps); // delta_bg (rad/(s^2*sqrt(hz))) ~0.5e-4 0.000001454441043; 

    // cout <<" HI I AM HERE 14"<<endl; 
    // according to https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
    // 
    Matrix33 measured_acc_cov = Matrix33::Identity(3,3)*pow(accel_noise_sigma, 2) * 10. /dt ; // sigma_a * sigma_a / dt * 10, ~2e-6 * 200 *10 = 4e-3
    Matrix33 measured_omega_cov = Matrix33::Identity(3,3)*pow(gyro_noise_sigma, 2) /dt; // sigma_g * sigma_g / dt ~ 0.36e-8 * 200 = 0.72e-6
    Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-4; // error committed in integration position from veloctiy
    Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma, 2) * dt; // sigma_ba * sigma_ba * dt, ~ 3e-5 * 0.05 = 1.5e-6
    Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma, 2) * dt; // sigma_bg * sigma_bg * dt, ~ 0.25e-8 * 0.05 = 1.25e-10
    Matrix66 bias_acc_omega_int = Matrix::Identity(6,6) * 1e-3; // error in the bias used for preintegration 

    // cout <<"HI I AM HERE 12"<<endl; 
    p->gyroscopeCovariance = measured_omega_cov; 
    p->accelerometerCovariance = measured_acc_cov; 
    p->integrationCovariance = integration_error_cov; 
    p->biasAccCovariance = bias_acc_cov; 
    p->biasOmegaCovariance = bias_omega_cov;  
    p->biasAccOmegaInt = bias_acc_omega_int; 
    b_once = false;
  }
  // cout <<"HI I AM HERE 13"<<endl;
  return p; 
}

bool CImuVn100::getRPYAt(double t, Eigen::Vector3d& rpy)
{
  int index = findIndexAt(t); 
  if(index < 0) return false; 
  
  rpy = mv_rpy[m_syn_start_id + index] - (mp_ini_rpy);  // difference 
  return true; 
}

bool CImuVn100::readImuData(string fname)
{
  ifstream inf(fname.c_str()); 
  
  if(!inf.is_open())
  {
    printf("%s failed to open imu file %s\n", __FILE__, fname.c_str()); 
    return false; 
  }
  double t; 
  float ax, ay, az, gx, gy, gz, yaw, pitch, roll; 
  Eigen::Vector6d m; 
  Eigen::Vector3d rpy; 
  while(!inf.eof())
  {
    
    inf>>t>>ax>>ay>>az>>gx>>gy>>gz>>yaw>>pitch>>roll; 
    
    m << gx , gy, gz, ax, ay, az; 
    rpy << roll, pitch, yaw; 
    mv_measurements.push_back(m); 
    mv_timestamps.push_back(t);
    mv_rpy.push_back(rpy); 
  }
  printf("%s succeed to load %i imu measurements\n", __FILE__, mv_timestamps.size()); 
  inf.close(); 
  return true; 
}

void CImuVn100::setStartPoint(double t) // set start point 
{
  CImuBase::setStartPoint(t); 
  
  // cout <<"HI I AM HERE 6 m_syn_id ="<<m_syn_start_id<<" mv_rpy.size() "<<mv_rpy.size()<<endl;
  // use the average of 1 camera readings to reset gravity, and base 
  // (*mp_ini_rpy) = mv_rpy[m_syn_start_id]; 
  mp_ini_rpy = mv_rpy[m_syn_start_id]; 

  // cout <<"HI I AM HERE 7"<<endl;
  // cout <<__FILE__<< " reset initial rpy: "<<mv_ini_rpy(0)<<" "<<mv_ini_rpy(1)<<" "<<mv_ini_rpy(2)<<endl; 
  // resetGravity(1); 
  
  /*
  // reset initial state 
  double roll = D2R((*mp_ini_rpy)(0)); 
  double pitch = D2R((*mp_ini_rpy)(1)); 
  double yaw = D2R((*mp_ini_rpy)(2)); 

  cout <<" ini attitude : "<<(*mp_ini_rpy)<<" radius: "<<roll<<" "<<pitch<<" "<<yaw<<endl; 
  Rot3 R = Rot3::RzRyRx(roll, pitch, yaw); 
  Point3 tt = Point3(); 
  Velocity3 v; v << 0, 0, 0; 
  NavState ini_state = NavState(R, tt, v); 

  // ini_state.attitude() = Rot3::RzRyRx(roll, pitch, yaw); 

  // setState(ini_state); 
*/
  // cout<<"HI I AM HERE 8"<<endl;
  return ;
} 

void CImuVn100::resetGravity(int n)
{
  if(n + m_syn_start_id > mv_measurements.size()) 
  {
    n = mv_measurements.size() - m_syn_start_id; 
  }
  
  double ax = 0; 
  double ay = 0; 
  double az = 0; 
  
  // cout<<"HI I AM HERE 9 mv_measurements.size() = "<<mv_measurements.size()<<endl;

  for(int i=0; i<n; i++)
  {
    ax += mv_measurements[i+m_syn_start_id](3); 
    ay += mv_measurements[i+m_syn_start_id](4); 
    az += mv_measurements[i+m_syn_start_id](5); 
  }

  // cout <<"HI I AM HERE 10"<<endl;
  if(n > 0)
  {
    double dn =double(n); 
    ax /= dn;  ay /= dn; az /= dn; 
    Vector3 new_gv; new_gv << ax , ay, az; 
    // cout <<"HI I AM HERE 11"<<endl;
    this->getIMUParams()->n_gravity = new_gv; 
    cout << __FILE__<< " reset initial gravity = "<<ax<<" "<<ay<<" "<<" "<<az<<endl;
  }
  // cout <<"HI I AM HERE 10"<<endl;
}
