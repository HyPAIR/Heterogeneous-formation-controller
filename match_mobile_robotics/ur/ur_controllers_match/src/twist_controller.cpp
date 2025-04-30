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

ros::Publisher joint_vel_pub;

class TwistController
{
public:
    TwistController() : nh("") //nh("~")
    {
        ros::NodeHandle nh_private("~");

        // std::string robot_description;
        // nh_private.param<std::string>("ur_prefix", robot_description, "/mur620/robot_description");
        std::string ur_prefix;
        nh_private.param<std::string>("prefix_ur", ur_prefix, "UR10_l/");
        nh_private.param<std::string>("group_name", PLANNING_GROUP, "UR_arm_l");

        // init twistVector with zeros
        twistVector = Eigen::VectorXd::Zero(6);

        // Set up a MoveIt! MoveGroup for the robot
        move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
        // auto opt = moveit::planning_interface::MoveGroupInterface::Options(PLANNING_GROUP, "/mur620/robot_description");
        // move_group.reset(new moveit::planning_interface::MoveGroupInterface(opt));
        
        ROS_INFO("Planning frame: %s", move_group->getPlanningFrame().c_str());

        ROS_INFO("Robot descr.: %s", move_group->ROBOT_DESCRIPTION.c_str());


        joint_model_group = move_group->getCurrentState(5.0)->getJointModelGroup(PLANNING_GROUP);


        // Set up a publisher for joint velocity commands
        joint_vel_pub = nh.advertise<std_msgs::Float64MultiArray>(ur_prefix+"joint_group_vel_controller/command", 1);

        // Set up a subscriber for the Cartesian twist command
        twist_sub = nh.subscribe("twist_command", 10, &TwistController::twistCallback, this);


        run();
    }

    void twistCallback(const geometry_msgs::Twist::ConstPtr& twist_msg)
    {
        // Extract the Cartesian twist command from the message
        geometry_msgs::Twist twist = *twist_msg;
        twistVector = Eigen::VectorXd::Map(&twist.linear.x, 6);

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
            Eigen::VectorXd joint_velocities = jacobian_inv * twistVector;

            // Publish joint velocity commands
            std_msgs::Float64MultiArray joint_vel_msg;
            joint_vel_msg.data.resize(joint_velocities.size());
            for (size_t i = 0; i < joint_velocities.size(); ++i) {
                joint_vel_msg.data[i] = joint_velocities(i);
            }
            joint_vel_pub.publish(joint_vel_msg);
            
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
    Eigen::VectorXd twistVector;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "twist_controller_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    TwistController twist_controller;

    // ros::waitForShutdown();

    return 0;
}
