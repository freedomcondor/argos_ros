/**
 *
 * @author Michael Allwright <mallwright@learnrobotics.io>
 */

#include "drone_ros_bridge_controller.h"

#include <argos3/plugins/robots/drone/control_interface/ci_drone_flight_system_actuator.h>
#include <argos3/plugins/robots/drone/control_interface/ci_drone_flight_system_sensor.h>
#include <argos3/plugins/robots/drone/control_interface/ci_drone_cameras_system_sensor.h>

#include <extensions/debug/debug_default_actuator.h>

//ros
#include "ros/ros.h"
#include "std_msgs/String.h"

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

   ros::NodeHandle* CTestController::nodeHandle = initROS();

   /****************************************/
   /****************************************/

   void CTestController::Init(TConfigurationNode& t_tree) {
      CCI_DroneFlightSystemActuator* pcActuator =
         GetActuator<CCI_DroneFlightSystemActuator>("drone_flight_system");
      pcActuator->SetTargetPosition(CVector3(4.0, 0.0, 1.0));

      CCI_DroneCamerasSystemSensor* pcCameraSensor =
         GetSensor<CCI_DroneCamerasSystemSensor>("drone_cameras_system");
      pcCameraSensor->Visit(
         [](CCI_DroneCamerasSystemSensor::SInterface& s_interface) {
            s_interface.Enable();
         }
      );

      stringstream sensorTopic;
      sensorTopic << "/" << GetId() << "/sensorTopic";

      chatter_pub = nodeHandle->advertise<std_msgs::String>(sensorTopic.str(), 1000);
   }

   /****************************************/
   /****************************************/

   void CTestController::ControlStep() {
      CDebugDefaultActuator* pcDebugActuator =
         GetActuator<CDebugDefaultActuator>("debug");
      pcDebugActuator->m_pvecArrows->emplace_back(CVector3(0,0,1), CVector3(0,0,2), CColor::GREEN);



      cout << "I am step" << GetId() << endl;

      std_msgs::String msg;

      std::stringstream ss;
      ss << "I am " << GetId();
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());

      chatter_pub.publish(msg);
      ros::spinOnce();
   }

   REGISTER_CONTROLLER(CTestController, "test_controller");

}



