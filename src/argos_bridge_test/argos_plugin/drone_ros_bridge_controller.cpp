/**
 *
 * @author Michael Allwright <mallwright@learnrobotics.io>
 */

#include "drone_ros_bridge_controller.h"

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>

using namespace std;

namespace argos {
	// Initialize ROS node.  There will be only one ROS node no matter how many robots are created in
	// ARGoS.  However, we will have one instance of the CArgosRosBot class for each ARGoS robot.
	ros::NodeHandle* initROS() {
		int argc = 0;
		char *argv = (char *) "";
		ros::init(argc, &argv, "argos_ros_bridge");
		return new ros::NodeHandle();
	}

	ros::NodeHandle* CDroneController::nodeHandle = initROS();

	/****************************************/
	/****************************************/

	void CDroneController::Init(TConfigurationNode& t_tree) {
		//Get flight system actuator / sensor
		m_pcFlightSystemActuator = GetActuator<CCI_DroneFlightSystemActuator>("drone_flight_system");
		m_pcFlightSystemSensor = GetSensor<CCI_DroneFlightSystemSensor>("drone_flight_system");

		//m_pcFlightSystemActuator->SetTargetPosition(CVector3(4.0, 0.0, 1.0));

		//Get Camera
		m_pcCameraSensor = GetSensor<CCI_DroneCamerasSystemSensor>("drone_cameras_system");
		m_pcCameraSensor->Visit(
			[&](CCI_DroneCamerasSystemSensor::SInterface& s_interface) {
				m_pcCameraInterface = &s_interface;
			}
		);
		m_pcCameraInterface->Enable();

		m_poseSensorPublisher = nodeHandle->advertise<geometry_msgs::Pose>(GetId() + "/poseSensor", 1000);
		m_poseActuatorSubscriber = nodeHandle->subscribe(GetId() + "/poseActuator", 1000, &CDroneController::poseActuatorCallback, this);
	}

	/****************************************/
	/****************************************/

	void CDroneController::ControlStep() {
		// draw debug arrow
		m_pcDebugActuator = GetActuator<CDebugDefaultActuator>("debug");
		m_pcDebugActuator->m_pvecArrows->emplace_back(CVector3(0,0,1), CVector3(0,0,2), CColor::GREEN);

		// read pose readings and publish to poseSensor topic
		CVector3 currentPosition = m_pcFlightSystemSensor->GetPosition();
		CVector3 currentOrientationInEuler = m_pcFlightSystemSensor->GetOrientation();
		CQuaternion currentOrientation;
		currentOrientation.FromEulerAngles(
			CRadians(currentOrientationInEuler.GetZ()),
			CRadians(currentOrientationInEuler.GetY()),
			CRadians(currentOrientationInEuler.GetX())
		);

		geometry_msgs::Pose pose;
		pose.position.x = currentPosition.GetX();
		pose.position.y = currentPosition.GetY();
		pose.position.z = currentPosition.GetZ();
		pose.orientation.x = currentOrientation.GetX();
		pose.orientation.y = currentOrientation.GetY();
		pose.orientation.z = currentOrientation.GetZ();
		pose.orientation.w = currentOrientation.GetW();
		m_poseSensorPublisher.publish(pose);

		// spin
		ros::spinOnce();
	}

	void CDroneController::poseActuatorCallback(const geometry_msgs::Pose& pose) {
		m_pcFlightSystemActuator->SetTargetPosition(CVector3(
			pose.position.x,
			pose.position.y,
			pose.position.z
		));
	}

	REGISTER_CONTROLLER(CDroneController, "test_controller");

}



