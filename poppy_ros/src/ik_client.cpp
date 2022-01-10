double deg2rad(double angle)

{

              // WATCH OUT THE REFERENCE FRAMES OF THE SIMULATOR ARE REVERSED COMPARED TO THE IK LIBRARY

              return -angle / 180.0 * M_PI;

}

 

int main(int argc, char** argv)

{

              // ADD ROS INIT

             

              ros::Publisher jointCmdPublisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

             

              // ADD RECEPTION OF THE SERVICE

              // IN THIS CASE RECEIVED AS A TWIST

              std::vector<double> jointCmdValueArray = {deg2rad(srv.response.response.linear.x), deg2rad(srv.response.response.linear.y), deg2rad(srv.response.response.linear.z),

                                                                                                                                                deg2rad(srv.response.response.angular.x), deg2rad(srv.response.response.angular.y), deg2rad(srv.response.response.angular.z)};

                            

              std::vector<std::string> jointCmdNameArray = {"m1", "m2", "m3", "m4", "m5", "m6"};

                                                                                                                                               

              while (ros::ok())

              {

                             sensor_msgs::JointState jointCmdMsg;

                             jointCmdMsg.header.stamp = ros::Time::now();

                             jointCmdMsg.header.seq++;

                             jointCmdMsg.position = jointCmdValueArray;

                             jointCmdMsg.name = jointCmdNameArray;

                            

                             jointCmdPublisher.publish(jointCmdMsg);

                             ros::spinOnce();

                             loopRate.sleep();

              }                                         

                                          

 

              return 0;

}
