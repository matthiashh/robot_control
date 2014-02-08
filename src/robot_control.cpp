#include <ros/ros.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/Sound.h>
#include "robot_control.h"
#include <turtlebot_msgs/TakePanorama.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>


//needed for setting navigation goals
//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_control");
    RobotControl mainControl("robot_control");
    //run-function is never exited
    mainControl.run();
    return 0;
}

void RobotControl::TaskServerCallback_(const robot_control::RobotTaskGoalConstPtr &goal)
{
  ROS_INFO("WOHO - A new Goal");
  TaskServerResult_.result.end_result = "woho";
  //TaskServer_->setSucceeded(TaskServerResult_);
}

void RobotControl::connectionCallback(const database_binding::DatabaseConnection &state)
{
  if (state.connection == 0)
  {
    db_connect_ = robot_control::connected;
    led2_.value = led2_.GREEN;
    led2_pub_.publish(led2_);
  }
  else if (state.connection == 1)
  {
    db_connect_ = robot_control::unsure;
    led2_.value = led2_.ORANGE;
    led2_pub_.publish(led2_);
  }
  else
  {
    db_connect_ = robot_control::notConnected;
    led2_.value = led2_.RED;
    led2_pub_.publish(led2_);
  }
  db_connect_last_update.data = state.header.stamp;
}

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
                  robot_control_mode = robot_control::database;
                  led1_.value = led1_.GREEN;
                  led1_pub.publish(led1_);
                  break;
                }
            case 1:
                {
                  button1_ = false;
                  robot_control_mode = robot_control::speechControlled;
                  led1_.value = led1_.ORANGE;
                  led1_pub.publish(led1_);
                  break;
                }
            case 2:
                {
                  button2_ = false;
                  robot_control_mode = robot_control::manual;
                  led1_.value = led1_.RED;
                  led1_pub.publish(led1_);
                }
            }
        }
}

RobotControl::RobotControl(std::string name) :
  TaskServer_ (n_,name,boost::bind(&RobotControl::TaskServerCallback_, this, _1),false),
  TaskClient_ ("robot_control",true)

{
    //initializing variables
    //actionserver
      //    TaskServer_->registerGoalCallback(boost::bind(&RobotControl::TaskServerGoalCallback_, this));
//    TaskServer_->registerPreemptCallback(boost::bind(&RobotControl::TaskServerGoalCallback,this));
//    subTaskTopic_ = n_.subscribe("/robot_control/task_server",1,&RobotControl::TaskServerAnalysisCallback,this);
    //database
    db_connect_ = robot_control::notConnected;
    connection_state_sub_ = n_.subscribe("/database_binding/connection_status", 10, &RobotControl::connectionCallback, this);
    // initialize kobuki_base with buttons, leds and sounds
    led1_pub = n_.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1",10);
    led2_pub_ = n_.advertise<kobuki_msgs::Led>("/mobile_base/commands/led2",10);
    kob_sound_ = n_.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound",10);
    button_ = n_.subscribe("/mobile_base/events/button", 10, &RobotControl::buttonCallback, this);
    button0_ = false;
    button1_ = false;
    button2_ = false;
    robot_control_mode = robot_control::manual;
    led1_.value = led1_.RED;
    led2_.value = led2_.RED;
    led1_pub.publish(led1_);
    led2_pub_.publish(led2_);
    //map and localization
    map_loc_ = n_.serviceClient<std_srvs::Empty>("/global_localization");
    //taking panorama
    take_pano_ = n_.serviceClient<turtlebot_msgs::TakePanorama>("/turtlebot_panorama/take_pano");
    ROS_INFO("Created robot_control");
}


int RobotControl::run()
{
  ROS_INFO("RobotControl started running");
  ros::Rate r(10);
  while (ros::ok())
  {
    //do self localization if button0 is pressed
//    if (button0_)
//    {
//      std_srvs::Empty empty;
//      map_loc_.call(empty);
//      ros::Duration(5).sleep();
//      //TODO: it shouldn't block the main-loop
//      fullTurn();
//    }
      if ((ros::Time::now()-db_connect_last_update.data).toSec() > 30)
        {
          db_connect_ = robot_control::notConnected;
          led2_.value = led2_.RED;
          led2_pub_.publish(led2_);
        }
      else if ((ros::Time::now()-db_connect_last_update.data).toSec() > 15)
        {
          db_connect_ = robot_control::unsure;
          led2_.value = led2_.ORANGE;
          led2_pub_.publish(led2_);
        }
    ros::spinOnce();
    r.sleep();
  }
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
