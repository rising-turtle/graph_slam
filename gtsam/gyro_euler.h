/*
 *  David Z, April 15, 2016 
 *  
 *  read gyro file, first compute bias, then compute euler angle at each step 
 * 
 *
 * */

#ifndef GYRO_EULER_H
#define GYRO_EULER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
// #include "ros/ros.h"

using namespace std;

class CGyroEuler
{
  public:
    CGyroEuler(); 
    ~CGyroEuler(); 

    bool readGyro(string fname); 
    void computeBias(); 
    void computeEuler(float rpy[3], int gyro_xyz[3]); // gyro_xyz: read from the gyro, rpy[3]: output roll, pitch, yaw 
    void computeAllEulers();                 // compute the eulers given all the gyro reading 
    bool getEulerAt(int i, float rpy[3]);    // get rpy of frame i 
    bool readEulerAt(int i, float rpy[3]);   // read ith rpy 
    vector< vector<int> > gyro_xyz_; // gyro read 
    vector< vector<float> > gyro_rpy_;  // rpy computed
    int gyro_xyz_bias_[3];           // gyro bias 
    int syn_start_id_;               // camera data syn with this id
    // float euler_rpy_[3];             // current euler_angle 
    
};



#endif
