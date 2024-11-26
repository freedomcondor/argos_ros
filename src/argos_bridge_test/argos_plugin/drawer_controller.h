/**
 *
 * @author Michael Allwright <mallwright@learnrobotics.io>
 */

#include <argos3/core/control_interface/ci_controller.h>

#include <argos3/plugins/robots/drone/control_interface/ci_drone_flight_system_actuator.h>
#include <argos3/plugins/robots/drone/control_interface/ci_drone_flight_system_sensor.h>
#include <argos3/plugins/robots/drone/control_interface/ci_drone_cameras_system_sensor.h>
#include <extensions/debug/debug_default_actuator.h>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

namespace argos {

	class CDrawerController : public CCI_Controller {

	public:

		CDrawerController() {}

		virtual ~CDrawerController() {}

		virtual void Init(TConfigurationNode& t_tree);
		virtual void ControlStep();

		void debugActuatorCallback(const geometry_msgs::Pose& pose);

	public:
	// We need only a single ROS node, although there are individual publishers
	// and subscribers for each instance of the class.
		ros::NodeHandle* nodeHandle;
		ros::Subscriber m_debugActuatorSubscriber;

		CDebugDefaultActuator* m_pcDebugActuator;
		CDebugDefaultActuator::TArrowVec arrowVec;
	};
}
