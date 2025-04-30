// Created by weijian on 17/11/23.

// include ROS Libraries
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <eigen3/Eigen/Dense>

// function to get robot position and orientation from odom topic
void robot_odom(const nav_msgs::Odometry msg);

// variable declaration
ros::Publisher robot_vel_pub,errors_pub,desired_traj_pub; // node publishers
tf::Point Odom_pos;    //odometry position (x,y)
double Odom_yaw;    //odometry orientation (yaw)

geometry_msgs::Pose2D qd,robot_pose,err;
geometry_msgs::Twist vel_msg;

double vd,wd,vr,wr,u1,u2,sigma,t,ts;
double a,b,ep;
double kx,ky,kz;
double xd,yd,xdd,ydd;
double freq=2*M_PI/35;
double vm = 0.22; // Turtlebot3 maximum linear velocity
double wm = 2.8;  // Turtlebot3 maximum angular velocity

using namespace Eigen;
using namespace std;

int main(int argc, char **argv)
{

	// initialization of ROS node
    ros::init(argc, argv, "model_predictive_trajectory_tracking_node");
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
	ts = 0.1;

	VectorXd ur(2),uf(2),e(3),kmpc(2),c(2),Q(3),R(2);
	
	MatrixXd A0(3,3),Ar(3,3),B(3,2),H(12,8),Z(3,2),
				Qt(12,12),Rt(8,8),F(12,3),Fr(12,3),K1(8,8),k2(8,3),kk(2,3);

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
		ur << vd,wd;

		desired_traj_pub.publish(qd); // publish the desired trajectory coordinates to the plotter node
		

		// calculate Tracking errors between the robot and desired trajectory coordinates
		err.x = (qd.x-robot_pose.x) * cos(robot_pose.theta) + (qd.y-robot_pose.y) * sin(robot_pose.theta);
		err.y = -(qd.x-robot_pose.x) * sin(robot_pose.theta) + (qd.y-robot_pose.y) * cos(robot_pose.theta);
		err.theta = qd.theta - robot_pose.theta;
		err.theta = atan2(sin(err.theta),cos(err.theta));// Normalize theta_e between -pi and oi
		errors_pub.publish(err); // publish errors to the plotter node

		A0 << 1,ts*ur(1),0,
			-ts*ur(1),1,ts*ur(0), 
			0,0,1;	

		Ar << 0.65,0,0  ,0,0.65,0, 0,0,0.65;	
		Q << 0.5,2.5,0.1;	
		R << pow(10,-7),pow(10,-7);	
		B << ts,0  ,0,0,  0,ts;
		Z = MatrixXd::Zero(3,2);

		Qt = Q.replicate(4,1).asDiagonal();
		Rt = R.replicate(4,1).asDiagonal();

		H << A0*B,	Z,	Z,	Z,
			 A0*A0*B,	A0*B,	Z,	Z, 
			 A0*A0*A0*B,	A0*A0*B,	A0*B,	Z,
			 A0*A0*A0*A0*B,	A0*A0*A0*B,	A0*A0*B,	A0*B;		

		F << A0*A0,	A0*A0*A0,	A0*A0*A0*A0,	A0*A0*A0*A0*A0;
		Fr << Ar,	Ar*Ar,	Ar*Ar*Ar,	Ar*Ar*Ar*Ar;

		uf << vd*cos(err.theta),wd;
		e << err.x , err.y , err.theta;

		K1 = H.transpose() * Qt * H + Rt   ;
		k2 = K1.inverse()*(H.transpose() * Qt *(Fr-F));
		kk << k2.row(0),k2.row(1);
		kmpc = -kk*e;

		c = kmpc + uf;
		vr = c(0);
		wr = c(1);


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
		robot_vel_pub.publish(vel_msg);
		t = t+0.1;

		ros::spinOnce();
		loop_rate.sleep();		
	}

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

