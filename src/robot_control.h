#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include <ros/ros.h>                    //general ROS-header
#include <kobuki_msgs/ButtonEvent.h>    //needed for kobuki button eventhandling
#include <kobuki_msgs/Led.h>            //needed for kobuki led handling
#include <move_base_msgs/MoveBaseAction.h>          //needed for navigation_goals
#include <robot_control/RobotTaskAction.h>
#include <actionlib/client/simple_action_client.h>  // for a move_base client
#include <actionlib/client/action_client.h>
#include <actionlib/server/action_server.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Time.h>
#include <database_interface/postgresql_database.h>   // to connect to a postgresql database
#include <robot_control/RegisterTaskServer.h>                     // the registration message

//needed for actionserver
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::ActionServer<robot_control::RobotTaskAction> TaskServer;


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

  struct externalServer
  {
    std::string task_name;
    std::string task_server_name;
    actionlib::ActionClient<robot_control::RobotTaskAction>* as;
  };

  struct srvClGoalPair
  {
    TaskServer::GoalHandle* srv;
    actionlib::ClientGoalHandle<robot_control::RobotTaskAction> cl;
  };

  struct GoalClientPair
  {
    RobotTaskGoal goal;
    actionlib::ClientGoalHandle<robot_control::RobotTaskAction> client_gh;
  };
}


class RobotControl
{
private:
    //ros
    ros::NodeHandle n_;
    // Main ActionServer
    actionlib::ActionServer<robot_control::RobotTaskAction> main_task_server_;
    actionlib::ActionClient<robot_control::RobotTaskAction> db_task_client_;

    robot_control::RobotTaskActionResult TaskServerResult_;
    robot_control::RobotTaskActionFeedback TaskServerFeedback_;

    robot_control::RobotTaskGoal goal_;
    TaskServer::GoalHandle* current_gh_;
    std::vector<TaskServer::GoalHandle> all_goals_server_;
    bool goal_active_;

    // External Actionserver
    std::vector<robot_control::externalServer> all_external_as_;
    std::vector<robot_control::srvClGoalPair> all_external_goals_;
    ros::Subscriber sub_reg_taskserver;

    // Database Actionclient

    std::vector<robot_control::GoalClientPair> all_db_goals_;

    // Kobuki base
    ros::Publisher motors_;
    ros::Subscriber button_;
    ros::Publisher led1_pub;
    kobuki_msgs::Led led1_;
    ros::Publisher led2_pub_;
    kobuki_msgs::Led led2_;
    ros::Publisher kob_sound_;
    bool button0_;
    bool button1_;
    bool button2_;

    // Robot Control
    robot_control::robotMode robot_mode;
    bool running;
    // Database
    robot_control::connectionStatus db_connect_;
    //! A database object. It is public to avoid get and set methods. It will be made private in the later process //TODO Make PostgresqlDatabase private
    database_interface::PostgresqlDatabase* database_;
    //localization and map
    ros::ServiceClient map_loc_;

    //panorama
    ros::ServiceClient take_pano_;

    // FUNCTIONS
    // Callbacks
    void buttonCallback(const kobuki_msgs::ButtonEvent button);
    void registerCallback(const robot_control::RegisterTaskServer reg);
    void transitionCallbackExternalGoals(actionlib::ClientGoalHandle<robot_control::RobotTaskAction> cgh);
    void transitionCallbackDatabaseGoals(actionlib::ClientGoalHandle<robot_control::RobotTaskAction> cgh);
    // Database
    bool processNotification(database_interface::Notification no);
    bool getTasks();
    void checkDatabaseConn();
    // Main Actionserver
    void TaskServerGoalCallback(TaskServer::GoalHandle gh);
    void TaskServerCancelCallback(TaskServer::GoalHandle gh);
    bool setNewMainGoal();

    // Robot Control
    bool dbStart();
    bool dbStop();
    bool stop_;
    //test-function
    void test_server();

    int fullTurn();

public:
    RobotControl(std::string name);
    // Database
    bool connectDb();
    bool getConfig();
    int run();
};

#endif // ROBOT_CONTROL_H
