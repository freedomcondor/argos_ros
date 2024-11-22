#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

#include "drone/math/vector3.h"

#include<vector>

using namespace ARGOS_ROS_SWARM;
using std::vector;

CVector3 currentPose = CVector3(0,0,0);
uint16_t wayPointID = 0;



void poseSensorCallback(const geometry_msgs::Pose& pose)
{
	currentPose = CVector3(
		pose.position.x,
		pose.position.y,
		pose.position.z
	);
}

void setPoseAcutator(const CVector3& P, ros::Publisher poseActuatorPublisher)
{
	geometry_msgs::Pose pose;
	pose.position.x = P.GetX();
	pose.position.y = P.GetY();
	pose.position.z = P.GetZ();
	poseActuatorPublisher.publish(pose);
}

int main(int argc, char **argv)
{
	vector<CVector3> wayPoints;
	wayPoints.emplace_back(0,0,2);
	wayPoints.emplace_back(5,0,2);
	wayPoints.emplace_back(5,5,2);
	wayPoints.emplace_back(0,5,2);
	wayPoints.emplace_back(0,0,2);

	ros::init(argc, argv, "flightSystem");
	ros::NodeHandle n;
	ros::Subscriber poseSensorSubscriber = n.subscribe("poseSensor", 1000, poseSensorCallback);
	ros::Publisher poseActuatorPublisher = n.advertise<geometry_msgs::Pose>("poseActuator", 1000);

	ros::Rate loop_rate(10); // 设置循环频率为10Hz
	while (ros::ok()) {
		setPoseAcutator(wayPoints[wayPointID], poseActuatorPublisher);
		if ((currentPose - wayPoints[wayPointID]).Length() < 0.5 ) wayPointID = (wayPointID + 1) % wayPoints.size();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}