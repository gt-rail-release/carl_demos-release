#ifndef SPECIFIC_ACTIONS_H_
#define SPECIFIC_ACTIONS_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>
#include <carl_demos/ObtainObjectAction.h>
#include <rail_manipulation_msgs/ArmAction.h>
#include <rail_manipulation_msgs/PickupAction.h>
#include <rail_manipulation_msgs/StoreAction.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <std_srvs/Empty.h>

#define NUM_JACO_JOINTS 6
#define MAX_HOME_ATTEMPTS 3

class SpecificActions
{

public:

  /**
  * \brief Constructor
  */
  SpecificActions();

private:
  ros::NodeHandle n;

  ros::Subscriber recognizedObjectsSubscriber;

  ros::ServiceClient segmentClient;

  actionlib::SimpleActionClient<rail_manipulation_msgs::ArmAction> armClient;
  actionlib::SimpleActionClient<rail_manipulation_msgs::PickupAction> pickupClient;
  actionlib::SimpleActionClient<rail_manipulation_msgs::StoreAction> storeClient;
  actionlib::SimpleActionServer<carl_demos::ObtainObjectAction> obtainObjectServer;

  int recognizedObjectsCounter;

  boost::mutex recognizedObjectsMutex;

  rail_manipulation_msgs::SegmentedObjectList recognizedObjects;

  /**
  * \brief Pickup a specified object, if it can be found in front of the robot
  *
  * Action server to perform object recognition, followed by a pickup of the specified object, followed by storing the
  * the picked-up object.
  *
  * \param goal Specification of the object to be obtained.
  */
  void executeObtainObject(const carl_demos::ObtainObjectGoalConstPtr &goal);

  /**
   * \brief Store the latest recognized objects.
   *
   * Stores the latest recognized objects from the segmentation topic.
   *
   * \param objects The latest recognized objects.
   */
  void recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &objects);
};

#endif
