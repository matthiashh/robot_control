#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include <ros/ros.h>                    //general ROS-header
#include <kobuki_msgs/ButtonEvent.h>    //needed for kobuki button eventhandling
#include <kobuki_msgs/Led.h>            //needed for kobuki led handling
#include <move_base_msgs/MoveBaseAction.h>          //needed for navigation_goals
#include <actionlib/client/simple_action_client.h>  //needed for navigation_goals
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

//general connection enom, used for database_connection
enum connection_status {
    connected,
    buildConnect,
    lostConnect,
    notConnected
};

//needed for navigation goals
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RobotControl
{
private:
    ros::NodeHandle n_;
    //database
    connection_status db_connect_;
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

    //localization and map
    ros::ServiceClient map_loc_;
    int fullTurn();
    //panorama
    ros::ServiceClient take_pano_;

public:
    RobotControl();
    int run();
};

#endif // ROBOT_CONTROL_H
