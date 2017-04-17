#ifndef MISC_H
#define MISC_H

#include <tf/tf.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

template<typename T>
tf::Transform eigenTransf2TF(const T& transf)
{
  tf::Transform result; 
  tf::Vector3 translation; 
  translation.setX(transf.translation().x()); 
  translation.setY(transf.translation().y()); 
  translation.setZ(transf.translation().z()); 
  
  tf::Quaternion rotation; 
  Eigen::Quaterniond quat; 
  quat = transf.rotation(); 
  rotation.setX(quat.x()); 
  rotation.setY(quat.y()); 
  rotation.setZ(quat.z()); 
  rotation.setW(quat.w()); 
  
  result.setOrigin(translation); 
  result.setRotation(rotation);
  return result;
}

#endif
