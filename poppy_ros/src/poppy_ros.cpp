#include <ros/ros.h>
#include <math.h>
#include <DynamixelHandler.h>
#include <geometry_msgs/Twist.h>



DynamixelHandler dynamix ;

float map (float x)
{	
	x = x +180;
	x = x/360;
	x = x*1023;
return x;
}

//Subscriber
void jointCallBack(const geometry_msgs::Twist& msg)
{

	std::vector<uint16_t> vTargetJointPosition;
	vTargetJointPosition.resize(6);
	vTargetJointPosition[0] = map(msg.linear.x);
	vTargetJointPosition[1] = map(msg.linear.y);
	vTargetJointPosition[2] = map(msg.linear.z);
	vTargetJointPosition[3] = map(msg.angular.x);
	vTargetJointPosition[4] = map(msg.angular.y);
	vTargetJointPosition[5] = map(msg.angular.z);
	
	dynamix.sendTargetJointPosition(vTargetJointPosition);
		
}



int main (int argc, char **argv)
{
	ros::init(argc, argv, "poppy_ros");
	
	//Dynamixel
	
	dynamix.setDeviceName("/dev/ttyUSB0");
	dynamix.setProtocolVersion(2.0);
	dynamix.openPort();
	dynamix.setBaudRate(1000000);
	dynamix.enableTorque(true);
	ros::NodeHandle nh;
	
	
	
	//Publisher
	ros::Publisher poppyPublisher = nh.advertise<geometry_msgs::Twist>("joint_position",1);
	ros::Rate loopRate(10);
	
	//Subscriber
	ros::Subscriber subscriber = nh.subscribe("joint_cmd",10,jointCallBack);

	
	//Publisher
	while (ros::ok())
	{
		std::vector<uint16_t> vCurrentJointPosition;
		geometry_msgs::Twist pos;
		dynamix.readCurrentJointPosition(vCurrentJointPosition);
		if (vCurrentJointPosition.size() >=6)
		{
			pos.linear.x = vCurrentJointPosition[0];
			pos.linear.y = vCurrentJointPosition[1];
			pos.linear.z = vCurrentJointPosition[2];
			pos.angular.x = vCurrentJointPosition[3];
			pos.angular.y = vCurrentJointPosition[4];
			pos.angular.z = vCurrentJointPosition[5];
		
			ROS_INFO_STREAM("My position is");
			ROS_INFO_STREAM(pos);
			poppyPublisher.publish(pos);
		}
		ros::spinOnce();
		loopRate.sleep();
		
	}
	dynamix.closePort();
	return 0;
}
