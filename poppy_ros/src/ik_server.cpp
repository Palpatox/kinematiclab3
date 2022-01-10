#include <ros/ros.h>
#include <Kinematics.h>
#include <poppy_ros/ik.h>


Kinematics kinematics;

std::vector<uint64_t> position;
position.resize(3);
position[0] = ik.srv.x;
position[1] = ik.srv.y;
position[2] = ik.srv.z;
kinematics.inverseKinematics(position);
ik.srv(m1,m2,m3,m4,m5,m6).qValues = kinematics.getQiValues();
