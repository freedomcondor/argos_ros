/**
 *
 * @author Michael Allwright <mallwright@learnrobotics.io>
 */

#include <argos3/core/control_interface/ci_controller.h>

#include "ros/ros.h"

namespace argos {

   class CTestController : public CCI_Controller {

   public:

      CTestController() {}

      virtual ~CTestController() {}

      virtual void Init(TConfigurationNode& t_tree);
      virtual void ControlStep();	

   public:
   // We need only a single ROS node, although there are individual publishers
   // and subscribers for each instance of the class.
      static ros::NodeHandle* nodeHandle;
      ros::Publisher chatter_pub;
   };
}
