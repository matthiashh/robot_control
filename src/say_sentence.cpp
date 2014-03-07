#include <ros/ros.h>                                // mandatory ROS_header
#include <human_interface/SpeechRequest.h>          // to perform the speech request
#include <robot_control/RobotControlSimpleClient.h> // to connect it to robot controll
#include <string>

class say : public RobotControlSimpleClient
{
private:
  ros::Publisher pub_speech_;
  bool checkIncomingGoal(robot_control::RobotTaskGoalConstPtr goal, robot_control::RobotTaskResult &res);
public:
  say(std::string task_server_name, std::string task_name);
  void run();
};

bool say::checkIncomingGoal(robot_control::RobotTaskGoalConstPtr goal, robot_control::RobotTaskResult &res)
{
  // we could check the database here
  return true;
}

say::say(std::string task_server_name, std::string task_name) : RobotControlSimpleClient(task_server_name,task_name)
{
  pub_speech_ = n_.advertise<human_interface::SpeechRequest>("/human_interface/speech_request",10);
}


void say::run()
{
  ros::Rate r(3);
  while (ros::ok())
  {
    if (goal_active_)
    {
      human_interface::SpeechRequest req;
      req.text_to_say = "Yeah - it worked";
      pub_speech_.publish(req);
      result_.success = true;
      result_.end_result = "Everything worked";
      ros::Duration(5).sleep();
      task_goal_->setSucceeded(result_,"Everything worked");
      goal_active_ = false;
    }
    r.sleep();
    spinServer();
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "say_sentence");
  say say_object("/robot_control_basics/say_task_server","say");

  ROS_INFO("Finished initialization, now running in the loop");
  //This loop is supposed to run endless
  say_object.run();
  return 0;
}
