
#include <ros/ros.h> 
#include <sstream>
#include <string>

#include <gtsam/geometry/Rot3.h>

using namespace gtsam;
using namespace std;

void show_q(string what, Quaternion& q); 

int main()
{
  
  Rot3 R1; 

  Quaternion q1 = R1.toQuaternion(); 

  show_q("q1:", q1); 
  Rot3 R2 = Rot3::RzRyRx(-M_PI/3., 0.2, -M_PI/2.); // roll, pitch, yaw, in WORLD coordinate system
  Quaternion q2 = R2.toQuaternion() ; 

  show_q("q2:", q2); 
  Rot3 R3(q2.w(), q2.x(), q2.y(), q2.z());
  
  q2 = R3.toQuaternion(); 
  show_q("q2: ", q2); 

  cout <<"Hello bug"<<endl;
  return 0; 
}


void show_q(string what, Quaternion& q)
{
  cout<<what<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<" "<<endl;
}


