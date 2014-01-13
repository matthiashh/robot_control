#include <ros/ros.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/Sound.h>
#include "robot_control.h"
#include <turtlebot_msgs/TakePanorama.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

//needed for setting navigation goals
//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_control");
    RobotControl mainControl;
    //run-function is never exited
    mainControl.run();
    return 0;
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
            case 0 : button0_ = false;
                break;
            case 1: button1_ = false;
                break;
            case 2: button2_ = false;
            }
        }
}

RobotControl::RobotControl()
{
    //initializing variables
    //databaset
    db_connect_ = notConnected;
    // initialize kobuki_base with buttons, leds and sounds
    led1_pub = n_.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1",10);
    led2_pub_ = n_.advertise<kobuki_msgs::Led>("/mobile_base/commands/led2",10);
    kob_sound_ = n_.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound",10);
    button_ = n_.subscribe("/mobile_base/events/button", 10, &RobotControl::buttonCallback, this);
    button0_ = false;
    button1_ = false;
    button2_ = false;
    led1_.value = 0;
    led2_.value = 0;
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
    if (button0_) 
    {
      std_srvs::Empty empty;
      map_loc_.call(empty);
      ros::Duration(5).sleep();
      //TODO: it shouldn't block the main-loop
      fullTurn();
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
