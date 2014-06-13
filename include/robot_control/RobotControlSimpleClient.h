#include <ros/ros.h>                              // mandatory ros header
#include <string>                                 // to process strings
#include <robot_control/RegisterTaskServer.h>     // to register an task_server
#include <robot_control/RobotTaskAction.h>        // the action for the goals
#include <actionlib/server/action_server.h>       // the header for the server

typedef actionlib::ActionServer<robot_control::RobotTaskAction> _task_server;

//! A utility class to allow an easy implementation of modules

class RobotControlSimpleClient
{
private:
  ros::Publisher pub_registration_;                                               //!< Publisher to register at robot_control
  std::string task_name_;                                                         //!< The name of the offered task
  std::string task_server_name_;                                                  //!< The address/topic name of the task server
  void TaskServerGoalCallback(_task_server::GoalHandle gh);                       //!< The callback for incoming goals
  void TaskServerCancelCallback(_task_server::GoalHandle gh);                     //!< The callback for canceled goals
protected:
  ros::NodeHandle n_;                                                             //!< Mandatory ROS nodehandle
  _task_server::GoalHandle task_goal_;                                            //!< The current goal
  robot_control::RobotTaskResult result_;                                         //!< A template result for easier usage
  robot_control::RobotTaskFeedback feedback_;                                     //!< A template feedback for easier usage
  
  //! Function to register at robot_control
  /*! \return true if success*/
  bool registerServer();
  
  //! Function to deregister at robot_control
  /*! \return true if success */
  bool deregisterServer();
  
  //! A possibility to check the incoming goal before it gets accepted
  /*! \param goal The new goal
      \param res  The returned result if the goal is rejected
      \return true if accepted; false if rejected */
  virtual bool checkIncomingGoal(robot_control::RobotTaskGoalConstPtr goal, robot_control::RobotTaskResult &res);
  
  //! Draft: A possibility to prepare before a new goal is running
  /*! \param goal The new goal
      \param res  The result for the old goal */
  virtual bool prepareForNewGoal(robot_control::RobotTaskGoalConstPtr goal, robot_control::RobotTaskResult &res);
  
  //! A possibility to clean up the canceled old goal
  /*! \param res Result of the old goal */
  virtual bool cleanupCancelledGoal(robot_control::RobotTaskResult &res);
  
  //! Executes a ros::spinOnce
  void spinServer();
  
  //! Easy identifier if a goal is currently active
  bool goal_active_;
public :
  //! The task server object
  /*! Has to be public as this is the only way to get it work.*/
  actionlib::ActionServer<robot_control::RobotTaskAction> task_server_;
  
  //! The constructor
  /*! \param task_server_name The address/topic name of the task server
   *  \param task_name        The name of the implemented task */
  RobotControlSimpleClient(std::string task_server_name, std::string task_name);
  ~RobotControlSimpleClient();
};
