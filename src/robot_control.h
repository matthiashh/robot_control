#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include <ros/ros.h>                    //general ROS-header
#include <kobuki_msgs/ButtonEvent.h>    //needed for kobuki button eventhandling
#include <kobuki_msgs/Led.h>            //needed for kobuki led handling
#include <move_base_msgs/MoveBaseAction.h>          //needed for navigation_goals
#include <actionlib/client/simple_action_client.h>  //needed for navigation_goals
#include <actionlib/server/simple_action_server.h>
#include <robot_control/RobotTaskAction.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Time.h>
#include <database_binding/DatabaseConnection.h>    //to process the database connection messages

//general connection enom, used for database_connection
namespace robot_control
{
    enum connectionStatus
  {
      connected,
      unsure,
      notConnected
  };

  enum robotMode
  {
    database,
    speechControlled,
    manual
  };
}

//needed for actionserver
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::ActionServer<robot_control::RobotTaskAction> TaskServer_;

class RobotControl
{
private:
    //ros-stuff
    ros::NodeHandle n_;
    // ActionServer
    actionlib::ActionServer<robot_control::RobotTaskAction> TaskServer_;
    actionlib::ActionClient<robot_control::RobotTaskAction> TaskClient_;

    robot_control::RobotTaskActionResult TaskServerResult_;
    robot_control::RobotTaskActionFeedback TaskServerFeedback_;

    robot_control::RobotTaskGoal goal_;
    TaskServer_::GoalHandle current_gh_;

    void TaskServerGoalCallback_(TaskServer_::GoalHandle gh);
    void TaskServerPreemptCallback_(TaskServer_::GoalHandle gh);
    //test-function
    void test_server();

    //database
    robot_control::connectionStatus db_connect_;
    std_msgs::Time db_connect_last_update;
    ros::Subscriber connection_state_sub_;
    void connectionCallback_ (const database_binding::DatabaseConnection &state);
    //kobuki_base
    ros::Subscriber button_;
    ros::Publisher led1_pub;
    kobuki_msgs::Led led1_;
    ros::Publisher led2_pub_;
    kobuki_msgs::Led led2_;
    ros::Publisher kob_sound_;
    void buttonCallback(const kobuki_msgs::ButtonEvent button);
    bool button0_;
    bool button1_;
    bool button2_;
    robot_control::robotMode robot_control_mode;

    //localization and map
    ros::ServiceClient map_loc_;
    int fullTurn();
    //panorama
    ros::ServiceClient take_pano_;

public:
    RobotControl(std::string name);
    int run();
};

#endif // ROBOT_CONTROL_H
