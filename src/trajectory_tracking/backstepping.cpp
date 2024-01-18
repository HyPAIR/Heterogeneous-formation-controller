// Created by weijian on 17/11/23.

// include ROS Libraries
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

// function to get robot position and orientation from odom topic
void robot_odom(const nav_msgs::Odometry msg);

// variable declaration
ros::Publisher robot_vel_pub,errors_pub,desired_traj_pub; // node publishers
tf::Point Odom_pos;    //odometry position (x,y)
double Odom_yaw;    //odometry orientation (yaw)

geometry_msgs::Pose2D qd,robot_pose,err;
geometry_msgs::Twist vel_msg;

double vd,wd,vr,wr,u1,u2,sigma,t;
double a,b,ep;
double kx,ky,kz;
double xd,yd,xdd,ydd;
double freq=2*M_PI/35;
double vm = 0.22; // Turtlebot3 maximum linear velocity
double wm = 2.8;  // Turtlebot3 maximum angular velocity



int main(int argc, char **argv)
{

	// initialization of ROS node
    ros::init(argc, argv, "backstepping_trajectory_tracking_node");
    ros::NodeHandle n;
    ros::Subscriber sub_odometry = n.subscribe("/odom", 1000 , robot_odom);
    robot_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
	errors_pub = n.advertise<geometry_msgs::Pose2D>("/errors_pub",1000);
	desired_traj_pub = n.advertise<geometry_msgs::Pose2D>("/desired_traj_pub",1000);

	ros::Rate loop_rate(10); // 10Hz

	// wait for all publishers and subscribers to connected to avoid losing msgs
	while (errors_pub.getNumSubscribers() == 0 || desired_traj_pub.getNumSubscribers() == 0 
		|| robot_vel_pub.getNumSubscribers() == 0 )
	{
		loop_rate.sleep();
	}

	t = 0.0;


	while(t<=40 && ros::ok())
	{
		
		// calculate the desired trajectory coordinates and velocities
		qd.x = -1.0 + cos(freq*t);
		qd.y = 0.0 + sin(freq*t);
		xd = -freq*sin(freq*t); yd = freq*cos(freq*t);
		xdd = -freq*freq*cos(freq*t);ydd = -freq*freq*sin(freq*t);
		qd.theta = atan2(yd,xd);
		vd = (sqrt(pow(xd,2) + pow(yd,2)));
		wd = ( (xd * ydd) - (yd * xdd) )/ (pow(xd,2) + pow(yd,2));

		desired_traj_pub.publish(qd); // publish the desired trajectory coordinates to the plotter node
		
		// controller coffecients
		ep = 0.9 ; b = 15.0;
		a = sqrt(pow(wd,2)+ b * pow(vd,2)) ; 
		kx = 2 * ep *a; ky = b * fabs(vd) ;
		kz = 2 * ep *a;


		// calculate Tracking errors between the robot and desired trajectory coordinates
		err.x = (qd.x-robot_pose.x) * cos(robot_pose.theta) + (qd.y-robot_pose.y) * sin(robot_pose.theta);
		err.y = -(qd.x-robot_pose.x) * sin(robot_pose.theta) + (qd.y-robot_pose.y) * cos(robot_pose.theta);
		err.theta = qd.theta - robot_pose.theta;
		err.theta = atan2(sin(err.theta),cos(err.theta));// Normalize theta_e between -pi and oi
		errors_pub.publish(err); // publish errors to the plotter node

		// robot velocities based on the backstepping controller 
		// ref -> Tracking Control of Mobile Robots: A Case Study in Backstepping. 
		vr = vd * cos(err.theta) + kx * err.x;
		wr = wd + (( ky * err.y * sin(err.theta) ) / err.theta) + kz * err.theta ;

		// A saturation function of the command velocities that preserves the  curvature
		// ref -> Tracking-error model-based predictive control for mobile robots
		// in real time Robotics and Autonomous Systems
		sigma = std::max((fabs(vr)/vm),std::max((fabs(wr)/wm),1.0));

		if (sigma == (fabs(vr)/vm))
		{	
			u1 = std::copysignf(1.0,vr)*vm;
			u2 = wr/sigma;			
		}

		if (sigma == (fabs(wr)/wm))
		{		
			u2 = std::copysignf(1.0,wr)*wm;
			u1 = vr/sigma;
		}

		if (sigma == 1)
		{			
			u1 = vr;
			u2 = wr;
		}				
				
		vel_msg.linear.x = u1;
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =u2;
		robot_vel_pub.publish(vel_msg); // publish the robot velocities
		t = t+0.1;
		
		ros::spinOnce();
		loop_rate.sleep();		
	}
		// publish 0 velocities to stop the robot
		vel_msg.linear.x = 0;
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =0;
		robot_vel_pub.publish(vel_msg);
    return 0;
}

// function to get robot position and orientation from the odom topic
void robot_odom(const nav_msgs::Odometry msg)
{
    tf::pointMsgToTF(msg.pose.pose.position, Odom_pos);
    Odom_yaw = tf::getYaw(msg.pose.pose.orientation);
	robot_pose.x = msg.pose.pose.position.x;
	robot_pose.y = msg.pose.pose.position.y;

    robot_pose.theta = Odom_yaw;   
}

