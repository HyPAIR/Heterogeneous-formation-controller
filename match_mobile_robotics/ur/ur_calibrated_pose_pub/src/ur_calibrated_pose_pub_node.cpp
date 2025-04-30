#include "ros/ros.h"

#include <ur_calibrated_pose_pub/ur_calibrated_pose_pub.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur_calibrated_pose_pub_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	ur_calibrated_pose_pub::URCalibratedPosePub ur_calibrated_pose_pub(nh, private_nh);
	ROS_INFO_STREAM("UR Calibrated Publisher: Starting Init");
	ur_calibrated_pose_pub.init();
	ROS_INFO_STREAM("UR Calibrated Publisher: Init finished");
	ROS_INFO_STREAM("UR Calibrated Publisher: Starting Execution");
	ur_calibrated_pose_pub.execute();
	ROS_INFO_STREAM("UR Calibrated Publisher: Execution finished");
}
