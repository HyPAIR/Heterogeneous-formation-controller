


#include "serial_com_tlt.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include <string>

#ifndef TLTNODE_H
#define TLTNODE_H

class TltNode
{
public:
    TltNode(ros::NodeHandle );
    ~TltNode();



private:
    SerialComTlt srl_;
    thread com_thread_;
    thread join_states_thread_;

    string joint_name;
    
    // Publishers
    ros::Publisher pub_column_pose_;

    // Subscribers
    ros::Subscriber sub_column_size_;
    ros::Subscriber sub_column_duration_down_;
    ros::Subscriber sub_column_duration_up_;
    ros::Subscriber sub_column_stop_;
    ros::Subscriber sub_joy_;

    // ROS Callback
    void cbColumnSize( std_msgs::Float32);
    void cbDurationUp( std_msgs::Int16);
    void cbDurationDown( std_msgs::Int16);
    void cbJoy( sensor_msgs::Joy);
    void publishJoinStates();



    

};

#endif //TLTNODE_H