#include <ros/ros.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/Sound.h>
#include "robot_control.h"
#include <turtlebot_msgs/TakePanorama.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <robot_control/returnConfiguration.h>
#include <kobuki_msgs/MotorPower.h>                               // to stop the motors
#include <robot_control/returnTasks.h>                            // to receive a task object from the database
//#include <robot_control/returnConfiguration.h>                  // to receive

//needed for setting navigation goals
//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_control");
    RobotControl mainControl("robot_control/main_task_server");
    if (!mainControl.connectDb())
    {
      ROS_ERROR("Aborting. Can't start without database connection.");
      return -1;
    }
    if (!mainControl.getConfig())
    {
      ROS_ERROR("Aborting. Can't start without configuration,");
      return -1;
    }

    //run-function is never exited
    mainControl.run();
    return 0;
}

void RobotControl::TaskServerGoalCallback(TaskServer::GoalHandle gh)
{
  robot_control::RobotTaskGoal tmp_goal;
  tmp_goal = *gh.getGoal();
  ROS_INFO("Actionserver received new task. The ID is %i, priority is %i, task_name is %s",tmp_goal.task_id,tmp_goal.priority,tmp_goal.task_name.c_str());
  // search for a corresponding registered task_server
  bool found = false;
  for (unsigned int it = 0; it < all_external_as_.size(); it++)
  {
    if (all_external_as_[it].task_name == tmp_goal.task_name)
    {
      found = true;
      break;
    }
  }
  if (!found)
  {
    ROS_INFO("No corresponding task_server to the task_name %s is registered. Rejecting goal.",tmp_goal.task_name.c_str());
    robot_control::RobotTaskResult res;
    res.end_result = "Rejected, because no corresponding task_server was registered";
    res.success = false;
    std::string empty_str;
    gh.setRejected(res,empty_str);
    return;
  }
  all_goals_server_.push_back(gh);
}

void RobotControl::TaskServerCancelCallback(TaskServer::GoalHandle gh)
{
  robot_control::RobotTaskGoal tmp_goal;
  tmp_goal = *gh.getGoal();
  ROS_INFO("The cancelled goal looks like: The ID is %i, priority is %i, task_name is %s",tmp_goal.task_id,tmp_goal.priority,tmp_goal.task_name.c_str());
  //check if the goal is running
  actionlib_msgs::GoalStatus stat;
  stat = gh.getGoalStatus();
  for (unsigned int it = 0; it < all_external_goals_.size(); it++)
  {
    if (*all_external_goals_[it].srv == gh)
    {
      all_external_goals_[it].cl.cancel();
      ROS_INFO("Cancelled running external goal as well");
      all_external_goals_.erase(all_external_goals_.begin()+it);
    }
  }
  for (unsigned int it = 0; it < all_goals_server_.size(); it++)
  {
    if (all_goals_server_[it] == gh)
    {
      all_goals_server_.erase(all_goals_server_.begin()+it);
    }
  }
}

/*! Changes the state of the button bools */
void RobotControl::buttonCallback(const kobuki_msgs::ButtonEvent button) {
        ROS_INFO("Received buttons event of button [%i] with state [%i]",button.button,button.state );
        //Changing class variables in order to the button event
        if (button.state) {
            switch (button.button) {
            case 0 : button0_ = true;
                break;
            case 1: button1_ = true;
                break;
            case 2: button2_ = true;
            }
        } else {
            switch (button.button) {
            case 0 :
                {
                  button0_ = false;
                  robot_mode = robot_control::database;
                  led1_.value = led1_.GREEN;
                  led1_pub.publish(led1_);
                  break;
                }
            case 1:
                {
                  button1_ = false;
                  robot_mode = robot_control::speechControlled;
                  led1_.value = led1_.ORANGE;
                  led1_pub.publish(led1_);
                  break;
                }
            case 2:
                {
                  button2_ = false;
                  robot_mode = robot_control::manual;
                  led1_.value = led1_.RED;
                  led1_pub.publish(led1_);
                }
            }
        }
}

/*! Either add, updates or terminates a connection to an external task executing server */
void RobotControl::registerCallback(const robot_control::RegisterTaskServer reg)
{
  //try to find the task_name
  for (unsigned int it = 0; it < all_external_as_.size(); it++)
    {
      if (all_external_as_[it].task_name == reg.task_name)
      {
        //register
        if (reg.reg == true)
        {
          if (reg.task_server_name == all_external_as_[it].task_server_name)
          {
            ROS_INFO("A TaskServer for task_action %s and with the task_server_name %s was already registered. Nothing to do.",reg.task_name.c_str(),reg.task_server_name.c_str());
            return;
          }
          else
          {
            ROS_INFO("Updating TaskServer with the new server address %s",reg.task_server_name.c_str());
            delete all_external_as_[it].as;
            all_external_as_[it].as = new actionlib::ActionClient<robot_control::RobotTaskAction>(reg.task_server_name);
            return;
          }
        }
        //unregister - erase
        else
        {
          delete all_external_as_[it].as;
          all_external_as_.erase(all_external_as_.begin()+it);
          /* Search for pending goals associated with that server */
          ROS_INFO("Deleted task_server reference for task_name %s and task_server_name %s.",reg.task_name.c_str(),reg.task_server_name.c_str());
          return;
        }
      }
    }
  //register a new server
  if (reg.reg)
  {
    robot_control::externalServer new_srv;
    new_srv.task_name = reg.task_name;
    new_srv.task_server_name = reg.task_server_name;
    new_srv.as = new actionlib::ActionClient<robot_control::RobotTaskAction>(reg.task_server_name);
    ros::spinOnce();
    new_srv.as->waitForActionServerToStart(ros::Duration(3));
    ros::spinOnce();
    if (!new_srv.as->isServerConnected())
    {
      ROS_WARN("Tried to connect to the new task_server '%s' for task_name '%s', but the connection couldn't be established. Discarding this registration!",reg.task_server_name.c_str(),reg.task_name.c_str());
      delete new_srv.as;
      return;
    }
    all_external_as_.push_back(new_srv);
    ROS_INFO("New external TaskServer registered with task_server_name: %s and task_name: %s",reg.task_server_name.c_str(),reg.task_name.c_str());
  }
  //unregister a server not known server
  else
  {
    ROS_INFO("A task_server associated with the task_name %s isn't registered. Nothing to do.",reg.task_name.c_str());
  }
}

/*! Finds the corresponding goalhandle in the list of all running tasks and updates that goal on the main task server as well */
void RobotControl::transitionCallbackExternalGoals(actionlib::ClientGoalHandle<robot_control::RobotTaskAction> cgh)
{
  ROS_INFO("External goals status changed to %s",cgh.getCommState().toString().c_str());
  //refind goal
  for (unsigned int it = 0; it < all_external_goals_.size(); it++)
  {
    if (cgh == all_external_goals_[it].cl)
    {
      if (cgh.getCommState() == cgh.getCommState().DONE)
      {
        ROS_INFO("To be more precise it is %s",cgh.getTerminalState().toString().c_str());
        if (cgh.getTerminalState() == cgh.getTerminalState().ABORTED)
        {
          //abort the parent goal as well
          robot_control::RobotTaskResultConstPtr res = cgh.getResult();
          all_external_goals_[it].srv->setAborted(*res,"Aborted by the external taskserver");
          all_external_goals_.erase(all_external_goals_.begin()+it);
          goal_active_ = false;
        }
        else if (cgh.getTerminalState() == cgh.getTerminalState().SUCCEEDED)
        {
          //set the parent goal as suceeded
          all_external_goals_[it].srv->setSucceeded(*cgh.getResult());
          all_external_goals_.erase(all_external_goals_.begin()+it);
          goal_active_ = false;
        }
        else if (cgh.getTerminalState() == cgh.getTerminalState().LOST)
        {
          //lost the goal - this shouldn't happen
          all_external_goals_[it].srv->setAborted();
          all_external_goals_.erase(all_external_goals_.begin()+it);
          goal_active_ = false;
        }
        else if (cgh.getTerminalState() == cgh.getTerminalState().PREEMPTED)
        {
          all_external_goals_[it].srv->setAborted();
          all_external_goals_.erase(all_external_goals_.begin()+it);
          goal_active_ = false;
        }
        //delete the goal on the server site
        for (unsigned int is = 0; is < all_goals_server_.size(); is++)
        {
          if ((*all_goals_server_[is].getGoal()).task_id == (*all_external_goals_[it].srv->getGoal()).task_id)
          {
            all_goals_server_.erase(all_goals_server_.begin()+is);
            break;
          }
      }
      }
      return;
    }
  }
  ROS_ERROR("The goal causing the callback didn't exist anymore. This shouldn't happen");
}

/*! Allows to reflect the state changes back to the database */
void RobotControl::transitionCallbackDatabaseGoals(actionlib::ClientGoalHandle<robot_control::RobotTaskAction> cgh)
{
  for (unsigned int it = 0; it < all_db_goals_.size(); it++)
  {
    if (all_db_goals_[it].client_gh == cgh)
    {
      if (cgh.getCommState() == cgh.getCommState().DONE)
      {
        ROS_INFO("Status of the database goal with ID %i became %s",all_db_goals_[it].goal.task_id,cgh.getTerminalState().toString().c_str());
      }
      else
      {
          ROS_INFO("Status of the database goal with ID %i became %s",all_db_goals_[it].goal.task_id,cgh.getCommState().toString().c_str());
      }
      return;
    }
  }
  ROS_ERROR("Couldn't refind the database goal in the transistion callback. This shouldn't happen.");
}

/*! Feedback of extern running goals arrives here and is reflected to the corresponding goal running on the main task server */
void RobotControl::feedbackCallbackExternalGoals(actionlib::ClientGoalHandle<robot_control::RobotTaskAction> cgh)
{
  //ROS_INFO_THROTTLE(5,"Feedback callback for external goals called");
}

/*! Feedback of database goals running on the main task server arrives here and is reflected to the database */
void RobotControl::feedbackCallbackDatabaseGoals(actionlib::ClientGoalHandle<robot_control::RobotTaskAction> cgh)
{

}

/*! For example react to 'start', 'stop', etc. */
bool RobotControl::processNotification(database_interface::Notification no)
{
  ROS_INFO("Notification has the following attributes: PID: %i, Channel %s, Payload: %s",no.sending_pid,no.channel.c_str(),no.payload.c_str());
  //switch depending on the channel
  if (no.channel == "start")
  {
    ROS_INFO("Received START notification");
    dbStart();
    return true;
  }
  else if (no.channel == "stop")
  {
    ROS_INFO("Received STOP notification");
    dbStop();
    return true;
  }
  else if (no.channel == "global_start")
  {
    ROS_INFO("Received GLOBAL_START notification");
    dbStart();
    return true;
  }
  else if (no.channel == "global_stop")
  {
    ROS_INFO("Received GLOBAL_STOP notification");
    dbStop();
    return true;
  }
  else if (no.channel == "new_task")
  {
    ROS_INFO("Recieved NEW_TASK notification");
    getTasks();
    return true;
  }
  else
  {
    ROS_WARN("Received an notification on an unknown channel. This shouldn't happen. Dazed and confused, but continuing");
    return false;
  }
}

/*! Calls the database based on the API between the robot and the database */
bool RobotControl::getTasks()
{
  ROS_INFO("Getting new tasks");
  std::vector< boost::shared_ptr<returnTasks> > tasks;
  database_interface::FunctionCallObj call;
  call.name = "gettasks";
  if (!database_->callFunction(tasks,call))
  {
    ROS_WARN("Calling gettasks failed. Probably the connection dropped. Exiting.");
    return false;
  }
  robot_control::RobotTaskGoal goal;
  for (size_t i=0; i<tasks.size(); i++)
  {
    ROS_INFO("New goal has ID: %i, Priority %i, Task_name: %s",tasks[i]->task_id_.data(),tasks[i]->priority_.data(),tasks[i]->task_name_.data().c_str());
    goal.priority = tasks[i]->priority_.data();
    goal.task_id = tasks[i]->task_id_.data();
    goal.task_name = tasks[i]->task_name_.data();
    robot_control::GoalClientPair pair;
    pair.client_gh = db_task_client_.sendGoal(goal,boost::bind(&RobotControl::transitionCallbackDatabaseGoals, this, _1),boost::bind(&RobotControl::feedbackCallbackDatabaseGoals, this, _1));
    pair.goal = goal;
    all_db_goals_.push_back(pair);
  }
}

/*! Notifications are received in here */
bool RobotControl::checkDatabaseConn()
{
  if (!database_->isConnected())
  {
      ROS_INFO_THROTTLE(5,"Not connected to database. Trying to reconnect");
      database_->reconnect();
      led2_.value = led2_.RED;
      led2_pub_.publish(led2_);
      return false;
  }
  else
  {
    led2_.value = led2_.GREEN;
    led2_pub_.publish(led2_);
    database_interface::Notification no;
    if (!database_->checkNotify(no))
    {
      ROS_WARN_THROTTLE(10,"The check for notifications failed. This shouldn't happen.");
    }
    else if (!no.channel.empty())
    {
      processNotification(no);
    }
    return false;
  }
}


RobotControl::RobotControl(std::string name) :
  main_task_server_ (n_,name,false),
  db_task_client_ (name)
{
    //Main Actionserver
    main_task_server_.registerGoalCallback(boost::bind(&RobotControl::TaskServerGoalCallback, this, _1));
    main_task_server_.registerCancelCallback(boost::bind(&RobotControl::TaskServerCancelCallback,this, _1));
    main_task_server_.start();
    goal_active_ = false;

    // External TaskServer
    sub_reg_taskserver = n_.subscribe("/robot_control/register_task_server", 10, &RobotControl::registerCallback,this);

    // initialize kobuki_base with buttons, led, sounds and motors
    led1_pub = n_.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1",10);
    led2_pub_ = n_.advertise<kobuki_msgs::Led>("/mobile_base/commands/led2",10);
    kob_sound_ = n_.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound",10);
    button_ = n_.subscribe("/mobile_base/events/button", 10, &RobotControl::buttonCallback, this);
    button0_ = false;
    button1_ = false;
    button2_ = false;
    robot_mode = robot_control::manual;
    led1_.value = led1_.RED;
    led2_.value = led2_.RED;
    led1_pub.publish(led1_);
    led2_pub_.publish(led2_);
    motors_ = n_.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power",10);
    ROS_INFO("Created robot_control");
}


int RobotControl::run()
{
  unsigned int run_rate = 1;
  ros::Rate r(run_rate);
  ROS_INFO("RobotControl started running at a rate of %i Hz",run_rate);

  while (ros::ok())
  {
    checkContact(ros::Duration(17));
    checkDatabaseConn();
    if(!goal_active_)
    {
      setNewMainGoal();
    }
    ros::spinOnce();
    r.sleep();
  }
}

/*! Connects to the database using default values and upload the connection information on the parameter server */
bool RobotControl::connectDb()
{
  std::string host = "192.168.10.100";
  std::string port = "5432";
  std::string user = "turtlebot";
  std::string passwd = "";
  std::string db = "rosdb";
  ROS_INFO("Trying to connect with host %s, port %s, user %s, passwd %s, db %s",host.c_str(), port.c_str(),user.c_str(),passwd.c_str(),db.c_str());
  if (this->database_ = new database_interface::PostgresqlDatabase (host,port,user,passwd,db))
  {
      ROS_INFO("Connection to database successfully established");
      //push params to parameter server
      n_.setParam("/database/hostname",host);
      n_.setParam("/database/port",port);
      n_.setParam("/database/db_user",user);
      n_.setParam("/database/db_passwd",passwd);
      n_.setParam("/database/db_name",db);
      return true;
  }
  else
  {
      ROS_ERROR("Connection to database failed.");
      return false;
  }
}

/*! Gets the config from the database and sets the channels */
bool RobotControl::getConfig()
{
  std::vector< boost::shared_ptr<returnConfiguration> > configuration;
  database_interface::FunctionCallObj parameter;
  parameter.name = "get_configuraton";
// temporary disabled because of missing implementation in the database
//  if (!database_->callFunction(configuration,parameter))
//  {
//      ROS_ERROR("Get configuration failed. The connection to the database probably doesn't work");
//      return false;
//  }
  //write configuration to local variables.
  database_->listenToChannel("start");
  database_->listenToChannel("global_start");
  database_->listenToChannel("stop");
  database_->listenToChannel("global_stop");
  database_->listenToChannel("new_task");
  database_->listenToChannel("cancel_task");
  ROS_INFO("Finished configuration of the agent");
  return true;
}

bool RobotControl::setNewMainGoal()
{
  if (all_goals_server_.empty())
  {
    ROS_INFO_THROTTLE(30,"No new goals to set.");
    return false;
  }
  //search for the goal with the highest priority
  TaskServer::GoalHandle* next_goal;
  next_goal = &all_goals_server_.front();

  for (unsigned int it = 0; it < all_goals_server_.size(); it++)
  {
    if (((*all_goals_server_[it].getGoal()).priority > (*next_goal->getGoal()).priority) )
    {
      next_goal = &all_goals_server_[it];
    }
  }
  // find corresponding actionserver
  bool found = false;
  robot_control::externalServer* ts;
  for (unsigned int it = 0; it < all_external_as_.size(); it++)
  {
    if (all_external_as_[it].task_name == (*next_goal->getGoal()).task_name)
    {
      found = true;
      ts = &all_external_as_[it];
      break;
    }
  }
  if (!found)
  {
    ROS_ERROR("The next goal doesn't have an corresponding registered server to task_name %s. This shouldn't happen.",(*next_goal->getGoal()).task_name.c_str());
    robot_control::RobotTaskResult res;
    res.end_result = "Rejected, because no corresponding task_server was registered during runtime.";
    res.success = false;
    next_goal->setRejected(res);
  }
  else
  {
    robot_control::srvClGoalPair pair;
    pair.srv = next_goal;
    pair.cl = ts->as->sendGoal(*next_goal->getGoal(),boost::bind(&RobotControl::transitionCallbackExternalGoals, this, _1),boost::bind(&RobotControl::feedbackCallbackExternalGoals, this, _1));
    all_external_goals_.push_back(pair);
    next_goal->setAccepted("Goal started running on external ActionServer");
    ROS_INFO("Send new goal with ID %i and task_name %s to external task_server %s",(*next_goal->getGoal()).task_id, ts->task_name.c_str(), ts->task_server_name.c_str());
    goal_active_ = true;
  }
  return true;
}

void RobotControl::checkContact(ros::Duration max_time)
{
  if (all_external_as_.empty())
  {
    return;
  }
  for (unsigned int it = 0; it < all_external_as_.size(); it++)
  {
    if (all_external_as_[it].as->isServerConnected())
    {
        all_external_as_[it].last_contact = ros::Time::now();
    }
    else
    {
      ros::Duration diff;
      diff = ros::Time::now() - all_external_as_[it].last_contact;
      if (diff < max_time)
      {
          ROS_INFO("Couldn't contact external task server %s anymore. Last contact was %f s ago.",all_external_as_[it].task_server_name.c_str(),diff.toSec());
      }
      else
      {
        ROS_WARN("Couldn't contact external server %s for more than %f s. Terminating connection and aborting tasks.",all_external_as_[it].task_server_name.c_str(),max_time.toSec());
        for (unsigned int ig = 0; ig < all_external_goals_.size(); ig++)
        {
          if ((*all_external_goals_[ig].srv->getGoal()).task_name == all_external_as_[it].task_name)
          {
            ROS_INFO("Aborted running goal with ID %i, because the actionserver is not responding",(*all_external_goals_[ig].srv->getGoal()).task_id);
            std::string reason = "Lost connection to the server";
            robot_control::RobotTaskResult result;
            result.success = false;
            result.end_result = reason;
            all_external_goals_[ig].srv->setAborted(result, reason);
            all_external_goals_.erase(all_external_goals_.begin()+ig);
            goal_active_ = false;
            //delete that goal from the list of all goals
            for (unsigned int ih = 0; ih < all_goals_server_.size(); ih++)
            {
              if ((*all_goals_server_[ih].getGoal()).task_id == (*all_external_goals_[ig].srv->getGoal()).task_id)
                {
                  all_goals_server_.erase(all_goals_server_.begin()+ih);
                  break;
                }
            }
          }
        for (unsigned int ig = 0; ig < all_goals_server_.size(); ig++)
        {
          if ((*all_goals_server_[ig].getGoal()).task_name == all_external_as_[it].task_name)
          {
            ROS_INFO("Rejected Goal %i, because the actionserver is not responding anymre",(*all_goals_server_[ig].getGoal()).task_id);
            std::string reason ("Actionserver is not reachable anymore");
            robot_control::RobotTaskResult result;
            result.success = false;
            result.end_result = reason;
            all_goals_server_[ig].setRejected(result,reason);
            all_goals_server_.erase(all_goals_server_.begin()+ig);
          }
        }
        }
        delete all_external_as_[it].as;
        all_external_as_.erase(all_external_as_.begin()+it);
      }
    }
  }
}

void RobotControl::dbStart()
{
  //unblock motors
  kobuki_msgs::MotorPower pwr;
  pwr.state = kobuki_msgs::MotorPower::ON;
  motors_.publish(pwr);
  running = true;
  ROS_INFO("Fired up the motors. We're ready to go");
}

/*! \todo abort tasks */

void RobotControl::dbStop()
{
  // block motors
  kobuki_msgs::MotorPower pwr;
  pwr.state = kobuki_msgs::MotorPower::OFF;
  motors_.publish(pwr);
  running = false;
  ROS_WARN("Blocked the motors in behalf of the database");
  //abort all tasks
}

int RobotControl::fullTurn()
{
  //make a move_base client to be able to send goals
  MoveBaseClient ac_ ("move_base",true);
  while(!ac_.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal_;
  // target frame is base_link, because we want to turn around ourselves
  goal_.target_pose.header.frame_id = "base_link";
  goal_.target_pose.header.stamp = ros::Time::now();

  //45° turn clockwise
  goal_.target_pose.pose.position.x = 0;
  goal_.target_pose.pose.orientation.w = 0.9238;
  goal_.target_pose.pose.orientation.z = -0.3826;
  for ( int i = 0; i < 8; i++)
  {
    ROS_INFO("Sending goal nr %i",i+1);
    ac_.sendGoal(goal_);
    ac_.waitForResult();
    //sleep because AMCL needed some computation time
    ros::Duration(7).sleep();
  }
//  ROS_INFO("Waiting for results");
//  ac_.waitForResult();

//  if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//    ROS_INFO("Hooray, the base moved 1 meter forward");
//  else
//    ROS_INFO("The base failed to move forward 1 meter for some reason");
  ROS_INFO("Finished full turn");
  return 0;
}
