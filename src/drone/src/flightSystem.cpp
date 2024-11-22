#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

void poseSensorCallback(const geometry_msgs::Pose& pose)
{
	ROS_INFO("my Position X: [%f]", pose.position.x);
	ROS_INFO("my Position Y: [%f]", pose.position.y);
	ROS_INFO("my Position Z: [%f]", pose.position.z);
	ROS_INFO("my Orientation X: [%f]", pose.orientation.x);
	ROS_INFO("my Orientation Y: [%f]", pose.orientation.y);
	ROS_INFO("my Orientation Z: [%f]", pose.orientation.z);
	ROS_INFO("my Orientation W: [%f]", pose.orientation.w);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "flightSystem");

	ros::NodeHandle n;

	ros::Subscriber poseSensorSubscriber = n.subscribe("poseSensor", 1000, poseSensorCallback);
	ros::Publisher poseActuatorPublisher = n.advertise<geometry_msgs::Pose>("poseActuator", 1000);

	ros::Rate loop_rate(10); // 设置循环频率为10Hz
	while (ros::ok()) {
		geometry_msgs::Pose pose;
		pose.position.x = 0;
		pose.position.y = 0;
		pose.position.z = 1;
		poseActuatorPublisher.publish(pose);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}