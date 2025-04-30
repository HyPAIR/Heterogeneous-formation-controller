#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher joint_vel_pub;

class TwistControllerFeedback
{
public:
    TwistControllerFeedback() : nh("") //nh("~")
    {
        ros::NodeHandle nh_private("~");

        // std::string robot_description;
        // nh_private.param<std::string>("ur_prefix", robot_description, "/mur620/robot_description");
        std::string ur_prefix;
        nh_private.param<std::string>("prefix_ur", ur_prefix, "UR10_l/");
        nh_private.param<std::string>("group_name", PLANNING_GROUP, "UR_arm_l");

        // Set up parameters for the feedback control (e.g., PID gains)
        nh_private.param<double>("kp", kp, 1.0);
        nh_private.param<double>("ki", ki, 0.0);
        nh_private.param<double>("kd", kd, 0.0);
        nh_private.param<double>("kp_angular", kp_angular, 1.0);
        nh_private.param<double>("ki_angular", ki_angular, 0.0);
        nh_private.param<double>("kd_angular", kd_angular, 0.0);

        // Initialize variables for the feedback control
        previous_error = Eigen::Vector3d::Zero();
        integral = Eigen::Vector3d::Zero();
        previous_error_angular = Eigen::Vector3d::Zero();
        integral_angular = Eigen::Vector3d::Zero();



        // init twistVector with zeros
        twistVector = Eigen::VectorXd::Zero(6);

        // Set up a MoveIt! MoveGroup for the robot
        move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
        // auto opt = moveit::planning_interface::MoveGroupInterface::Options(PLANNING_GROUP, "/mur620/robot_description");
        // move_group.reset(new moveit::planning_interface::MoveGroupInterface(opt));
        
        ROS_INFO("Planning group: %s", move_group->getName().c_str());
        ROS_INFO("Planning frame: %s", move_group->getPlanningFrame().c_str());

        ROS_INFO("Robot descr.: %s", move_group->ROBOT_DESCRIPTION.c_str());


        joint_model_group = move_group->getCurrentState(5.0)->getJointModelGroup(PLANNING_GROUP);

        // Get base link:
        ros::V_string link_names = joint_model_group->getLinkModelNames();
        auto base_link = link_names[0];
        ROS_INFO("Base link of joint_model_group: %s", base_link.c_str());

        // Get rotation between base_link and move_group->getPlanningFrame():
        auto tf_base = move_group->getCurrentState(5.0)->getGlobalLinkTransform(base_link);
        tf_rotation = tf_base.rotation();
        tf_rotation.block(0,0,2,2) = -tf_rotation.block(0,0,2,2); // rotation of base_link to base_link_inertia
        Eigen::Quaternion<double> q(tf_rotation);
        q_tf_rotation = tf2::Quaternion(q.x(), q.y(), q.z(), q.w());
        ROS_INFO_STREAM("Rotation between base_link and planning_frame: " << tf_rotation);

        // Initialize the desired end-effector position and orientation
        geometry_msgs::Pose cur_pose = move_group->getCurrentPose().pose;
        Eigen::Vector3d cur_pos_vec(cur_pose.position.x, cur_pose.position.y, cur_pose.position.z);
        cur_pos_vec = tf_rotation * cur_pos_vec;
        desired_position.x = cur_pos_vec[0];
        desired_position.y = cur_pos_vec[1];
        desired_position.z = cur_pos_vec[2];

        desired_orientation = tf2::Quaternion(cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w);
        desired_orientation = q_tf_rotation * desired_orientation;
        desired_orientation.normalize();

        // Set up a publisher for joint velocity commands
        joint_vel_pub = nh.advertise<std_msgs::Float64MultiArray>(ur_prefix+"joint_group_vel_controller/command", 1);

        // Set up a subscriber for the Cartesian twist command
        twist_sub = nh.subscribe("twist_command", 10, &TwistControllerFeedback::twistCallback, this);


        run();
    }

    void twistCallback(const geometry_msgs::Twist::ConstPtr& twist_msg)
    {
        // Extract the Cartesian twist command from the message
        geometry_msgs::Twist twist = *twist_msg;
        twistVector = Eigen::VectorXd::Map(&twist.linear.x, 6);
        twistVector.block(0,0,3,1) = tf_rotation*twistVector.block(0,0,3,1);
        twistVector.block(3,0,3,1) = tf_rotation*twistVector.block(3,0,3,1);

    }

    void integrateTwist(double dt)
    {
        // Integrate the twist to update the desired end-effector position and orientation
        desired_position.x += twistVector[0] * dt;
        desired_position.y += twistVector[1]* dt;
        desired_position.z += twistVector[2] * dt;

        tf2::Quaternion rotation_quaternion;
        // tf2::fromMsg(twist.angular, rotation_quaternion); // Convert angular part to a quaternion
        // Get rotation_quaternion from twistVector instead twist.angular
        rotation_quaternion.setRPY(twistVector[3] * dt, twistVector[4] * dt, twistVector[5] * dt);


        // Rotate the desired orientation by the integrated twist
        desired_orientation = rotation_quaternion*desired_orientation;
        desired_orientation.normalize();
    }

    // feedback control to adjust the twist command
    Eigen::VectorXd adjustTwist()
    {
        geometry_msgs::Pose current_pose = move_group->getCurrentPose().pose;
        Eigen::Vector3d current_pos_vec(current_pose.position.x, current_pose.position.y, current_pose.position.z);
        current_pos_vec = tf_rotation * current_pos_vec;

        // Calculate the position and orientation errors
        geometry_msgs::Point position_error;
        position_error.x = desired_position.x - current_pos_vec[0];
        position_error.y = desired_position.y - current_pos_vec[1];
        position_error.z = desired_position.z - current_pos_vec[2];


        tf2::Quaternion q_orientation_error;
        tf2::fromMsg(current_pose.orientation, q_orientation_error);
        // TODO: rotate quaternion by rotation  matrix tf_rotation
        q_orientation_error = q_tf_rotation * q_orientation_error;
        q_orientation_error = desired_orientation * q_orientation_error.inverse();

        //orientation error in Euler angles:
        Eigen::Vector3d orientation_error;
        tf2::Matrix3x3(q_orientation_error).getRPY(orientation_error.x(), orientation_error.y(), orientation_error.z());
        // ROS_INFO_STREAM("orientation_error: " << orientation_error);


        // Implement feedback control to adjust the twist command
        Eigen::VectorXd corrected_twist_vec = Eigen::VectorXd::Zero(6);
        corrected_twist_vec[0] = twistVector[0] + kp * position_error.x + ki * integral.x() + kd * (position_error.x - previous_error.x());
        corrected_twist_vec[1] = twistVector[1] + kp * position_error.y + ki * integral.y() + kd * (position_error.y - previous_error.y());
        corrected_twist_vec[2] = twistVector[2] + kp * position_error.z + ki * integral.z() + kd * (position_error.z - previous_error.z());

        // Implement feedback control to adjust the twist command for angular velocity
        corrected_twist_vec[3] = twistVector[3] + kp_angular * orientation_error.x() + ki_angular * integral_angular.x() + kd_angular * (orientation_error.x() - previous_error_angular.x());
        corrected_twist_vec[4] = twistVector[4] + kp_angular * orientation_error.y() + ki_angular * integral_angular.y() + kd_angular * (orientation_error.y() - previous_error_angular.y());
        corrected_twist_vec[5] = twistVector[5] + kp_angular * orientation_error.z() + ki_angular * integral_angular.z() + kd_angular * (orientation_error.z() - previous_error_angular.z());

        // Update integrals and previous errors for the next iteration
        integral.x() += position_error.x;
        integral.y() += position_error.y;
        integral.z() += position_error.z;
        previous_error.x() = position_error.x;
        previous_error.y() = position_error.y;
        previous_error.z() = position_error.z;

        
        integral_angular.x() += orientation_error.x();
        previous_error_angular.x() = orientation_error.x();
        
        integral_angular.y() += orientation_error.y();
        previous_error_angular.y() = orientation_error.y();
        
        integral_angular.z() += orientation_error.z();
        previous_error_angular.z() = orientation_error.z();

        return corrected_twist_vec;
    }

    void run()
    {
        ros::Rate rate(50);
        ROS_INFO("Twist controller node ready.");
        while (ros::ok())
        {
            // Calculate the Jacobian matrix
            Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_inv;
            // joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
            jacobian_inv = move_group->getCurrentState()->getJacobian(joint_model_group).inverse();

            // Calculate joint velocities from the Jacobian and the Cartesian twist
            Eigen::VectorXd joint_velocities = jacobian_inv * adjustTwist();    // twistVector;

            // Publish joint velocity commands
            std_msgs::Float64MultiArray joint_vel_msg;
            joint_vel_msg.data.resize(joint_velocities.size());
            for (size_t i = 0; i < joint_velocities.size(); ++i) {
                joint_vel_msg.data[i] = joint_velocities(i);
            }
            joint_vel_pub.publish(joint_vel_msg);

            integrateTwist(rate.cycleTime().toSec());
            
            rate.sleep();
        }
        
    }
    
private:
    ros::NodeHandle nh;
    ros::Subscriber twist_sub;
    ros::Publisher joint_vel_pub;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    std::string PLANNING_GROUP;
    const robot_state::JointModelGroup* joint_model_group;
    Eigen::Matrix3d tf_rotation;
    tf2::Quaternion q_tf_rotation;
    Eigen::VectorXd twistVector;

    // Feedback control parameters
    double kp;
    double ki;
    double kd;
    double kp_angular;
    double ki_angular;
    double kd_angular;

    // Variables for feedback control
    Eigen::Vector3d integral;
    Eigen::Vector3d previous_error;
    Eigen::Vector3d integral_angular;
    Eigen::Vector3d previous_error_angular;
    

    geometry_msgs::Point desired_position;
    tf2::Quaternion desired_orientation;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "twist_controller_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    TwistControllerFeedback twist_controller;

    // ros::waitForShutdown();

    return 0;
}
