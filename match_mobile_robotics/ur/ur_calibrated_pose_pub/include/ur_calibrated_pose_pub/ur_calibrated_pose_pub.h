#ifndef UR_CALIBRATED_POSE_PUB_H_INCLUDED
#define UR_CALIBRATED_POSE_PUB_H_INCLUDED

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <XmlRpc.h>
#include <Eigen/Dense>
#include <vector>

#include <ur_client_library/comm/parser.h>
#include <ur_client_library/comm/pipeline.h>
#include <ur_client_library/comm/producer.h>
#include <ur_client_library/comm/stream.h>
#include <ur_client_library/primary/primary_parser.h>

#include <ur_calibrated_pose_pub/utils/ur_calibration_consumer.h>
#include <ur_calibrated_pose_pub/utils/dh_transformation.h>

namespace ur_calibrated_pose_pub
{
    class URCalibratedPosePub
    {
        public:
            URCalibratedPosePub(ros::NodeHandle nh, ros::NodeHandle private_nh);
            void init();
            void execute();

        private:
            ros::NodeHandle nh_;
	        ros::NodeHandle private_nh_;

            ros::Subscriber joint_state_subscriber_;
            ros::Publisher ur_calibrated_pose_publisher_;

            tf::TransformBroadcaster end_effector_broadcaster_;
            tf::TransformListener listener;

            std::string robot_ip_;
            std::string ur_joint_state_topic_name_;
            std::string joint_prefix_;
            std::string dh_parameter_switch_;
            
            std::vector<dh_utils::DHTransformation> ideal_dh_transformations_list_;
            std::vector<dh_utils::DHTransformation> calibrated_dh_transformations_list_;

            void readParams(); // Read ROS parameter from param server
            void getCalibratedDHParameter(); // Read calibrated DH parameters from robot controller

            // Callback functions
            void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state_msg);
    };
}
#endif  // ifndef UR_CALIBRATED_POSE_PUB_H_INCLUDED