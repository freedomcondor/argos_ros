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

	class CDroneController : public CCI_Controller {

	public:

		CDroneController() {}

		virtual ~CDroneController() {}

		virtual void Init(TConfigurationNode& t_tree);
		virtual void ControlStep();

		void poseActuatorCallback(const geometry_msgs::Pose& pose);

	public:
	// We need only a single ROS node, although there are individual publishers
	// and subscribers for each instance of the class.
		static ros::NodeHandle* nodeHandle;
		ros::Publisher m_poseSensorPublisher;
		ros::Subscriber m_poseActuatorSubscriber;

		CCI_DroneFlightSystemActuator* m_pcFlightSystemActuator;
		CCI_DroneFlightSystemSensor* m_pcFlightSystemSensor;
		CCI_DroneCamerasSystemSensor* m_pcCameraSensor;
		CCI_DroneCamerasSystemSensor::SInterface* m_pcCameraInterface;
		CDebugDefaultActuator* m_pcDebugActuator;
	};
}
