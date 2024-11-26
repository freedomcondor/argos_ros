#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

#include "drone/math/vector3.h"
#include "drone/math/quaternion.h"

#include<vector>
#include<cmath>

using namespace ARGOS_ROS_SWARM;
using std::vector;

CVector3 currentPose = CVector3(0,0,0);
double currentYaw = 0;
uint16_t wayPointID = 0;



void poseSensorCallback(const geometry_msgs::Pose& pose)
{
	currentPose = CVector3(
		pose.position.x,
		pose.position.y,
		pose.position.z
	);
	currentYaw = pose.orientation.z;
}

void setPoseAcutator(const CVector3& P, double yaw, ros::Publisher poseActuatorPublisher)
{
	geometry_msgs::Pose pose;
	pose.position.x = P.GetX();
	pose.position.y = P.GetY();
	pose.position.z = P.GetZ();
	pose.orientation.z = yaw;  // abuse orientation.z as yaw
	poseActuatorPublisher.publish(pose);
}

void drawArrow(const CVector3& start, const CVector3& end, ros::Publisher drawArrowsPublisher)
{
	geometry_msgs::Pose pose;
	pose.position.x = start.GetX();
	pose.position.y = start.GetY();
	pose.position.z = start.GetZ();
	pose.orientation.x = end.GetX();
	pose.orientation.y = end.GetY();
	pose.orientation.z = end.GetZ();
	drawArrowsPublisher.publish(pose);
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
	ros::Publisher drawArrowsPublisher = n.advertise<geometry_msgs::Pose>("drawArrows", 1000);

	ros::Rate loop_rate(10); // 设置循环频率为10Hz
	while (ros::ok()) {
		CVector3 targetWaypoint = wayPoints[wayPointID];
		CVector3 offset = targetWaypoint - currentPose;

		double targetYaw = atan2(offset.GetY(),
		                         offset.GetX());

		CVector3 targetPosition = targetWaypoint;

		double threshold = 1;
		if ((currentPose - targetPosition).Length() > threshold)
			targetPosition = currentPose + offset.Normalize() * threshold;

		setPoseAcutator(targetPosition, targetYaw, poseActuatorPublisher);

		CVector3 middle = currentPose + CVector3(4, 0, -1.8).Rotate(CQuaternion(currentYaw, CVector3(0,0,1)));
		CVector3 left   = currentPose + CVector3(4, -1, -1.8).Rotate(CQuaternion(currentYaw, CVector3(0,0,1)));
		CVector3 right  = currentPose + CVector3(4, 1, -1.8).Rotate(CQuaternion(currentYaw, CVector3(0,0,1)));
		drawArrow(middle, left, drawArrowsPublisher);
		drawArrow(middle, right, drawArrowsPublisher);
		drawArrow(currentPose, targetPosition, drawArrowsPublisher);

		// check arrival
		if ((currentPose - wayPoints[wayPointID]).Length() < 0.5 ) wayPointID = (wayPointID + 1) % wayPoints.size();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}