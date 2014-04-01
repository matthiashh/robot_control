#include <robot_control/RobotControlSimpleClient.h>

RobotControlSimpleClient::RobotControlSimpleClient(std::string task_server_name, std::string task_name) :
  task_server_ (n_,task_server_name,false)
{
  pub_registration_ = n_.advertise <robot_control::RegisterTaskServer>("/robot_control/register_task_server",10);
  task_server_.registerGoalCallback(boost::bind(&RobotControlSimpleClient::TaskServerGoalCallback, this, _1));
  task_server_.registerCancelCallback(boost::bind(&RobotControlSimpleClient::TaskServerCancelCallback,this, _1));
  task_server_.start();
  task_server_name_ = task_server_name;
  task_name_ = task_name;
  goal_active_ = false;
  ros::Duration(3).sleep();
  registerServer();
}

bool RobotControlSimpleClient::registerServer()
{
  robot_control::RegisterTaskServer reg;
  reg.task_name = task_name_;
  reg.task_server_name = task_server_name_;
  reg.reg = true;
  pub_registration_.publish(reg);
  return true;
}

bool RobotControlSimpleClient::deregisterServer()
{
  robot_control::RegisterTaskServer reg;
  reg.task_name = task_name_;
  reg.task_server_name = task_server_name_;
  reg.reg = false;
  pub_registration_.publish(reg);
  return true;
}

/*! You may override this function. It gives the possibility to check if a goal can be executed. If a false is returned the goal will be rejected.*/

bool RobotControlSimpleClient::checkIncomingGoal(robot_control::RobotTaskGoalConstPtr goal, robot_control::RobotTaskResult &res)
{
  return true;
}

bool RobotControlSimpleClient::cleanupCancelledGoal(robot_control::RobotTaskResult &res)
{
  return true;
}

bool RobotControlSimpleClient::prepareForNewGoal(robot_control::RobotTaskGoalConstPtr goal, robot_control::RobotTaskResult &res)
{
  return true;
}

RobotControlSimpleClient::~RobotControlSimpleClient()
{
  robot_control::RegisterTaskServer reg;
  reg.task_name = task_name_;
  reg.task_server_name = task_server_name_;
  reg.reg = false;
  pub_registration_.publish(reg);
}

void RobotControlSimpleClient::TaskServerGoalCallback(_task_server::GoalHandle gh)
{
  ROS_INFO("External server received goal with ID %i",(*gh.getGoal()).task_id);
  robot_control::RobotTaskResult res;
  if (checkIncomingGoal(gh.getGoal(),res))
  {
  if (goal_active_)
    {
      cleanupCancelledGoal(result_);
      task_goal_.setAborted(result_,"Preempted by another goal");
    }
    task_goal_ = gh;
    task_goal_.setAccepted("Accepted this goal");
    goal_active_ = true;
  }
  else
  {
    gh.setRejected(res,"Goal didn't pass checkIncomingGoal");
  }
}

void RobotControlSimpleClient::spinServer()
{
  ros::spinOnce();
}

void RobotControlSimpleClient::TaskServerCancelCallback(_task_server::GoalHandle gh)
{
  task_goal_.setCanceled();
  goal_active_ = false;
}


