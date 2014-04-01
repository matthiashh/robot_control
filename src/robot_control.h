#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include <ros/ros.h>                                  // general ROS-header
#include <kobuki_msgs/ButtonEvent.h>                  // for kobuki button eventhandling
#include <kobuki_msgs/Led.h>                          // for kobuki led handling
#include <move_base_msgs/MoveBaseAction.h>            // for navigation_goals
#include <robot_control/RobotTaskAction.h>            // the generic task action
#include <actionlib/client/simple_action_client.h>    // for a move_base client
#include <actionlib/client/action_client.h>           // to use an actionclient
#include <actionlib/server/action_server.h>           // to use an actionserver
#include <std_msgs/Time.h>
#include <database_interface/postgresql_database.h>   // to connect to a postgresql database
#include <robot_control/RegisterTaskServer.h>         // the registration message

//needed for actionserver
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::ActionServer<robot_control::RobotTaskAction> TaskServer;


//general connection enom, used for database_connection
namespace robot_control
{
  //! Enum to support multiple states of a service robot
  /*! It is intended to allow users to shut off database support */
  enum robotMode
  {
    database,             //!< Accept tasks from the database
    speechControlled,     //!< Allows the user to control the robot by speech
    manual                //!< For example for debugging or developing
  };

  //! Struct to store the connection and task executing capabilities
  /*! Every connection to an external task executing server is stored in element of this type  */
  struct externalServer
  {
    std::string task_name;                                        //!< Task identifying name
    std::string task_server_name;                                 //!< Topicname of the task server
    actionlib::ActionClient<robot_control::RobotTaskAction>* as;  //!< Access to the connection
    ros::Time last_contact;                                       //!< Frequenctly updated on every connection check
  };

  //! A struct for extern running tasks
  struct srvClGoalPair
  {
    TaskServer::GoalHandle* srv;                                      //!< Reference to the refering goal handle on the main_task_server
    actionlib::ClientGoalHandle<robot_control::RobotTaskAction> cl;   //!< Goalhandle of the extern running task
  };

  //! Struct for the database client
  struct GoalClientPair
  {
    RobotTaskGoal goal;                                                     //!< Goal of the task, because it can't be accessed through the goalhandle
    actionlib::ClientGoalHandle<robot_control::RobotTaskAction> client_gh;  //!< Corresponding goalhandle
  };
}


//! The main task running the whole robot controlling software

class RobotControl
{
private:
    //ros
    ros::NodeHandle n_;                                                                 //!< Mandatory nodehandle
    // Main ActionServer
    //! The main task server
    /*! The main task server is the central task server. Every task has to be sent to that server in order to be executed.*/
    actionlib::ActionServer<robot_control::RobotTaskAction> main_task_server_;
    actionlib::ActionClient<robot_control::RobotTaskAction> db_task_client_;            //!< The database client of the main task server


    std::vector<TaskServer::GoalHandle> all_goals_server_;                              //!< All goals of the main task server are stored here
    bool goal_active_;                                                                  //!< True if an external goal is running

    // External Actionserver
    std::vector<robot_control::externalServer> all_external_as_;                        //!< All connected external taskserver
    std::vector<robot_control::srvClGoalPair> all_external_goals_;                      //!< All external running goals
    ros::Subscriber sub_reg_taskserver;                                                 //!< Subscriber for the registration topic

    // Database Actionclient

    std::vector<robot_control::GoalClientPair> all_db_goals_;                           //!< All database goals and goalhandles

    // Kobuki base
    ros::Publisher motors_;                                                             //!< Publisher to turn the motors on and off
    ros::Subscriber button_;                                                            //!< Subscriber to button input
    ros::Publisher led1_pub;                                                            //!< Publisher for the first LED
    kobuki_msgs::Led led1_;                                                             //!< Msg to see the last state
    ros::Publisher led2_pub_;                                                           //!< Publisher for the second LED
    kobuki_msgs::Led led2_;                                                             //!< Msg to see the last state
    ros::Publisher kob_sound_;                                                          //!< Can make sounds on the turtlebot
    bool button0_;                                                                      //!< Latest state of button 0
    bool button1_;                                                                      //!< Latest state of button 1
    bool button2_;                                                                      //!< Latest state of button 2

    // Robot Control
    robot_control::robotMode robot_mode;                                                //!< Storing the state of the robot (not used yet)
    bool running;                                                                       //!< To see if the database told the robot to stop
    // Database

    database_interface::PostgresqlDatabase* database_;                                  //!< The connection to the database
    // FUNCTIONS
    // Callbacks

    //! Callback for turtlebot buttons
    /*! \param button the received button message */
    void buttonCallback(const kobuki_msgs::ButtonEvent button);

    //! Callback for the registration process
    /*! \param reg Received registration message */
    void registerCallback(const robot_control::RegisterTaskServer reg);

    //! Callback if an external goals state is changed
    /*! \param cgh Received goalhandle of the goal running externally */
    void transitionCallbackExternalGoals(actionlib::ClientGoalHandle<robot_control::RobotTaskAction> cgh);

    //! Callback if an database goals state is changed
    /*! \param cgh Received goalhandle of the goal running on the main task server */
    void transitionCallbackDatabaseGoals(actionlib::ClientGoalHandle<robot_control::RobotTaskAction> cgh);

    //! Callback for feedback of external goals
    /*! \param cgh Received goalhandle delivering the information */
    void feedbackCallbackExternalGoals(actionlib::ClientGoalHandle<robot_control::RobotTaskAction> cgh);

    //! Callback for feedback of database goals
    /*! \param cgh Goalhandle carying the feedback */
    void feedbackCallbackDatabaseGoals(actionlib::ClientGoalHandle<robot_control::RobotTaskAction> cgh);

    // Database
    //! Reacts based on information in the notification
    /*! \param no Notication delivering the information
        \return False if the notfication doesn't fit to the known ones */
    bool processNotification(database_interface::Notification no);

    //! Get new tasks if the database triggers
    /*! \return fails if the call failed */
    bool getTasks();

    //! Checks the database connection, sets the LED and checks for notifications
    /*! \return false if there's no connection */
    bool checkDatabaseConn();
    // Main Actionserver

    //! Called if a new goal is receieved
    /*! \param gh The new goal */
    void TaskServerGoalCallback(TaskServer::GoalHandle gh);

    //! Called if a goal is cancelled
    /*! \param gh The cancelled goal */
    void TaskServerCancelCallback(TaskServer::GoalHandle gh);

    //! Finds the goal with the highest priority and sets it
    bool setNewMainGoal();

    // External Actionserver
    //! Checks contact to all external actionserver
    /*! \param max_time Maximum time a connection can stay dropped until the external server is deleted and the corresponding goals are aborted */
    void checkContact(ros::Duration max_time);

    // Robot Control
    //! Starts robot services if the database asks devices to run
    void dbStart();

    //! Stop robot services if the database asks for a stop
    void dbStop();

    //! Deprecated function to do a full turn
    int fullTurn();

public:
    //! Constructor of the main class
    /*! \param name Topic name of the actionserver */
    RobotControl(std::string name);
    // Database

    //! Connect to the database
    /*! \return true if it worked */
    bool connectDb();

    //! Gets the config from the database
    /*! \return  */
    bool getConfig();

    //! Runs the controller and just returns on critical errors
    int run();
};

#endif // ROBOT_CONTROL_H
