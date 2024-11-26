/**
 *
 * @author Michael Allwright <mallwright@learnrobotics.io>
 */

#include "drawer_controller.h"

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>

using namespace std;

namespace argos {
	// Initialize ROS node.  There will be only one ROS node no matter how many robots are created in
	// ARGoS.  However, we will have one instance of the CArgosRosBot class for each ARGoS robot.
	ros::NodeHandle* initROS() {
		int argc = 0;
		char *argv = (char *) "";
		ros::init(argc, &argv, "drawer");
		return new ros::NodeHandle();
	}

	//ros::NodeHandle* CDrawerController::nodeHandle = initROS();

	/****************************************/
	/****************************************/

	void CDrawerController::Init(TConfigurationNode& t_tree) {
		nodeHandle = initROS();
		m_pcDebugActuator = GetActuator<CDebugDefaultActuator>("debug");
		m_debugActuatorSubscriber = nodeHandle->subscribe("drawArrows", 1000, &CDrawerController::debugActuatorCallback, this);
	}

	/****************************************/
	/****************************************/

	void CDrawerController::ControlStep() {
		// spin
		ros::spinOnce();

		for (std::tuple<CVector3, CVector3, CColor> arrow : arrowVec)
			m_pcDebugActuator->m_pvecArrows->push_back(arrow);
	}

	void CDrawerController::debugActuatorCallback(const geometry_msgs::Pose& pose) {
		CVector3 start = CVector3(
			pose.position.x,
			pose.position.y,
			pose.position.z
		);
		CVector3 end = CVector3(
			pose.orientation.x,
			pose.orientation.y,
			pose.orientation.z
		);

		arrowVec.emplace_back(start, end, CColor::GREEN);
	}

	REGISTER_CONTROLLER(CDrawerController, "drawer_controller");

}



