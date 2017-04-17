// #include "global_def.h"
#include "gyro_euler.h"
#include <stdio.h>
#include <string.h>
#include <cmath>

#define D2R(d) (((d)*M_PI)/180.)

CGyroEuler::CGyroEuler()
{
  // ros::NodeHandle n; 
}
CGyroEuler::~CGyroEuler(){}

bool CGyroEuler::readGyro(string fname)
{
  ifstream inf(fname.c_str());
  if(!inf.is_open())
  {
    cerr<<"gyro_euler.cpp: failed to open file "<<fname<<endl;
    return false;
  }
  
  int id1, id2, gx, gy, gz, ax, ay, az; 
  int last_id = -1;  
  int cnt = 0;

  vector<int> gr(3);
  while(!inf.eof())
  {
    inf>>id1>>gr[0]>>gr[1]>>gr[2]>>ax>>ay>>az>>id2; 
    if(last_id != -1 && last_id > id1 && id1 == 1)
    {
      cout<<"gyro_euler.cpp: good syn start from frame: "<<id2-1<<endl;
      syn_start_id_ = cnt; // id2-1;
    }
    // printf("gyro_euler.cpp: input %d %d %d %d %d %d %d %d\n", id1, gr[0], gr[1], gr[2], ax, ay, az, id2);
    gyro_xyz_.push_back(gr);
    ++ cnt;
    last_id = id1;
  }
  inf.close(); 
  printf("gyro_euler.cpp: retrieve %d , start_syn = %d\n", cnt, syn_start_id_);
  // for(int i=syn_start_id_; i<gyro_xyz_.size(); i++ )
  // {
    // printf("gyro_euler.cpp: read gyro reading %d %d %d\n", gyro_xyz_[i][0], gyro_xyz_[i][1], gyro_xyz_[i][2]);
  // }
  return true;
}

void CGyroEuler::computeBias()
{
  // initialize 
  memset(gyro_xyz_bias_, 0, sizeof(gyro_xyz_bias_));
  for(int i=0; i<syn_start_id_; i++)
  {
    gyro_xyz_bias_[0] += gyro_xyz_[i][0]; 
    gyro_xyz_bias_[1] += gyro_xyz_[i][1];
    gyro_xyz_bias_[2] += gyro_xyz_[i][2]; 
  }
  
  gyro_xyz_bias_[0] /= syn_start_id_; 
  gyro_xyz_bias_[1] /= syn_start_id_; 
  gyro_xyz_bias_[2] /= syn_start_id_;
  printf("gyro_euler.cpp: bias gx : %d gy: %d, gz: %d\n", gyro_xyz_bias_[0], gyro_xyz_bias_[1], gyro_xyz_bias_[2]);
}

void CGyroEuler::computeAllEulers()
{
  int N = gyro_xyz_.size() - syn_start_id_; 
  if(N <= 0) return; 
  gyro_rpy_.resize(N); 
  vector<float> t(3, 0); 
  float rpy[3] = {0}; 
  for(int i=0; i<N; i++)
  {
    getEulerAt(i, rpy); 
    t[0] = rpy[0]; t[1] = rpy[1]; t[2] = rpy[2]; 
    gyro_rpy_[i] = t;
  }
}



void CGyroEuler::computeEuler(float rpy[3], int gyro_xyz[3])
{
  // remove bias 
  int bias_free[3] = {0}; 
  for(int i=0; i<3; i++)
  {
    bias_free[i] = gyro_xyz[i] - gyro_xyz_bias_[i]; 
  }

  // compute angular velocity in body reference 
  float omega[3] = {0};
  for(int i=0; i<3; i++)
  {
    omega[i] = D2R((float)bias_free[i]*80./1092.);
  }
  
  // compute the angular velocity in world rederence 
  float erate[3] = {0}; 
  // eangle[3]: current Euler angles at which the gyro's rate of turn (omega[3]) is read
  // erate[3]: Euler rate at the current Euler angles      
  erate[0]=omega[0]*cos(rpy[1]) + omega[2]*sin(rpy[1]);
  erate[2]=(-omega[0]*sin(rpy[1]) + omega[2]*cos(rpy[1]))/cos(rpy[0]);
  erate[1]=omega[1] - erate[2]*sin(rpy[0]);

  // compute the euler angle 
  float dt = 0.01; 
  rpy[0] += erate[0]*dt; 
  rpy[1] += erate[1]*dt; 
  rpy[2] += erate[2]*dt;
}

bool CGyroEuler::readEulerAt(int i, float rpy[3])
{
  if(i <0 || i >= gyro_rpy_.size())
    return false; 
  vector<float>& t = gyro_rpy_[i]; 
  rpy[0] = t[0];  rpy[1] = t[1]; rpy[2] = t[2];
  return true;
}

// notice, rpy[3] is also input, that is the current euler angle
bool CGyroEuler::getEulerAt(int i, float rpy[3])
{
  // int N = gyro_xyz_.size() - syn_start_id_; 
  if( i< 0 || i + syn_start_id_ >= gyro_xyz_.size())
  {
    printf("gyro_euler.cpp: failed to access frame id : %d\n", i); 
    return false;
  }

  int gyro_xyz[3]; 
  int index = i + syn_start_id_; 
  gyro_xyz[0] = gyro_xyz_[index][0]; gyro_xyz[1] = gyro_xyz_[index][1]; gyro_xyz[2] = gyro_xyz_[index][2]; 
  
  computeEuler(rpy, gyro_xyz); 
  // printf("gyro_euler.cpp: read euler: %f %f %f with reading %d %d %d\n", 
     //  rpy[0], rpy[1], rpy[2], gyro_xyz[0], gyro_xyz[1], gyro_xyz[2]);
  return true; 
}




