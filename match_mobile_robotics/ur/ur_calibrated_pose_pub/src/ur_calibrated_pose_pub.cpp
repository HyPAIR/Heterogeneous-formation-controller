#include <ur_calibrated_pose_pub/ur_calibrated_pose_pub.h>

namespace ur_calibrated_pose_pub
{
	URCalibratedPosePub::URCalibratedPosePub(ros::NodeHandle nh,
											 ros::NodeHandle private_nh) : nh_(nh), private_nh_(private_nh)
	{ }

	void URCalibratedPosePub::init()
	{
		this->readParams();
		this->getCalibratedDHParameter();

		// Initialze subscriber, publisher, service servers and service clients
		this->joint_state_subscriber_ = this->nh_.subscribe(this->ur_joint_state_topic_name_,
															1,
															&URCalibratedPosePub::jointStateCallback,
															this);
		this->ur_calibrated_pose_publisher_ = this->nh_.advertise<geometry_msgs::PoseStamped>("ur_calibrated_pose", 1);
	}

	void URCalibratedPosePub::execute()
	{
		ros::Rate publish_rate = ros::Rate(100.0);
		while(ros::ok())
		{
			// Debugging of each transformation by broadcasting it to tf
			// Eigen::Matrix4d tf_matrix_0 = Eigen::Matrix4d::Identity();
			// Eigen::Matrix4d tf_matrix_1 = Eigen::Matrix4d::Identity();
			// Eigen::Matrix4d tf_matrix_2 = Eigen::Matrix4d::Identity();
			// Eigen::Matrix4d tf_matrix_3 = Eigen::Matrix4d::Identity();
			// Eigen::Matrix4d tf_matrix_4 = Eigen::Matrix4d::Identity();
			
			// for(int counter = 0; counter <= 1; counter++)
			// {
			// 	tf_matrix_0 = tf_matrix_0 * this->ideal_dh_transformations_list_[counter].getTransformationMatrix();
			// }
			// tf::Transform transform_0;
			// transform_0.setOrigin(tf::Vector3(tf_matrix_0(0, 3),
			// 								tf_matrix_0(1, 3),
			// 								tf_matrix_0(2, 3)));
			// tf::Quaternion q_0;
			// q_0.setRPY(0.0, 0.0, 0.0);
			// transform_0.setRotation(q_0);
			// this->end_effector_broadcaster_.sendTransform(tf::StampedTransform(transform_0, ros::Time::now(), "mur620b/UR10_r/base_link", "mur620b/UR10_r/KS_1"));

			// for(int counter = 0; counter <= 2; counter++)
			// {
			// 	tf_matrix_1 = tf_matrix_1 * this->ideal_dh_transformations_list_[counter].getTransformationMatrix();
			// }
			// tf::Transform transform_1;
			// transform_1.setOrigin(tf::Vector3(tf_matrix_1(0, 3),
			// 								tf_matrix_1(1, 3),
			// 								tf_matrix_1(2, 3)));
			// tf::Quaternion q_1;
			// q_1.setRPY(0.0, 0.0, 0.0);
			// transform_1.setRotation(q_1);
			// this->end_effector_broadcaster_.sendTransform(tf::StampedTransform(transform_1, ros::Time::now(), "mur620b/UR10_r/base_link", "mur620b/UR10_r/KS_2"));

			// for(int counter = 0; counter <= 3; counter++)
			// {
			// 	tf_matrix_2 = tf_matrix_2 * this->ideal_dh_transformations_list_[counter].getTransformationMatrix();
			// }
			// tf::Transform transform_2;
			// transform_2.setOrigin(tf::Vector3(tf_matrix_2(0, 3),
			// 								tf_matrix_2(1, 3),
			// 								tf_matrix_2(2, 3)));
			// tf::Quaternion q_2;
			// q_2.setRPY(0.0, 0.0, 0.0);
			// transform_2.setRotation(q_2);
			// this->end_effector_broadcaster_.sendTransform(tf::StampedTransform(transform_2, ros::Time::now(), "mur620b/UR10_r/base_link", "mur620b/UR10_r/KS_3"));

			// for(int counter = 0; counter <= 4; counter++)
			// {
			// 	tf_matrix_3 = tf_matrix_3 * this->ideal_dh_transformations_list_[counter].getTransformationMatrix();
			// }
			// tf::Transform transform_3;
			// transform_3.setOrigin(tf::Vector3(tf_matrix_3(0, 3),
			// 								tf_matrix_3(1, 3),
			// 								tf_matrix_3(2, 3)));
			// tf::Quaternion q_3;
			// q_3.setRPY(0.0, 0.0, 0.0);
			// transform_3.setRotation(q_3);
			// this->end_effector_broadcaster_.sendTransform(tf::StampedTransform(transform_3, ros::Time::now(), "mur620b/UR10_r/base_link", "mur620b/UR10_r/KS_4"));

			// for(int counter = 0; counter <= 5; counter++)
			// {
			// 	tf_matrix_4 = tf_matrix_4 * this->ideal_dh_transformations_list_[counter].getTransformationMatrix();
			// }
			// tf::Transform transform_4;
			// transform_4.setOrigin(tf::Vector3(tf_matrix_4(0, 3),
			// 								tf_matrix_4(1, 3),
			// 								tf_matrix_4(2, 3)));
			// tf::Quaternion q_4;
			// q_4.setRPY(0.0, 0.0, 0.0);
			// transform_4.setRotation(q_4);
			// this->end_effector_broadcaster_.sendTransform(tf::StampedTransform(transform_4, ros::Time::now(), "mur620b/UR10_r/base_link", "mur620b/UR10_r/KS_5"));

			Eigen::Matrix4d complete_transformation_matrix = Eigen::Matrix4d::Identity();

			// This selects which dh parameter are used to calculate the end effector pose
			if(this->dh_parameter_switch_ == "ideal")
			{
				for(dh_utils::DHTransformation &dh_transformation: this->ideal_dh_transformations_list_)
				{
					complete_transformation_matrix = complete_transformation_matrix * dh_transformation.getTransformationMatrix();
				}
			}
			else if(this->dh_parameter_switch_ == "calibrated")
			{
				for(dh_utils::DHTransformation &dh_transformation: this->calibrated_dh_transformations_list_)
				{
					complete_transformation_matrix = complete_transformation_matrix * dh_transformation.getTransformationMatrix();
				}
			}
			
			// Get namespace of the node:
			std::string node_namespace = ros::this_node::getNamespace();
			

			geometry_msgs::PoseStamped ur_calibrated_pose_msg;
			ur_calibrated_pose_msg.header.stamp = ros::Time::now();
			ur_calibrated_pose_msg.header.frame_id = node_namespace + "/base";	// TODO: change to generic name
			ur_calibrated_pose_msg.pose.position.x = complete_transformation_matrix(0, 3);
			ur_calibrated_pose_msg.pose.position.y = complete_transformation_matrix(1, 3);
			ur_calibrated_pose_msg.pose.position.z = complete_transformation_matrix(2, 3);
			
			Eigen::Matrix3d rotation_matrix = complete_transformation_matrix.block(0,0,3,3);
			Eigen::Vector3d euler_angle = rotation_matrix.eulerAngles(0, 1, 2);
			Eigen::Quaternion<float> eigen_q;
			eigen_q = Eigen::AngleAxisf(euler_angle(0), Eigen::Vector3f::UnitX()) *
					  Eigen::AngleAxisf(euler_angle(1), Eigen::Vector3f::UnitY()) *
					  Eigen::AngleAxisf(euler_angle(2), Eigen::Vector3f::UnitZ());

			ur_calibrated_pose_msg.pose.orientation.x = eigen_q.x();
			ur_calibrated_pose_msg.pose.orientation.y = eigen_q.y();
			ur_calibrated_pose_msg.pose.orientation.z = eigen_q.z();
			ur_calibrated_pose_msg.pose.orientation.w = eigen_q.w();
						
			ur_calibrated_pose_publisher_.publish(ur_calibrated_pose_msg);

			tf::Transform transform;
			transform.setOrigin(tf::Vector3(complete_transformation_matrix(0, 3),
											complete_transformation_matrix(1, 3),
											complete_transformation_matrix(2, 3)));
			tf::Quaternion q;
			q.setX(eigen_q.x());
			q.setY(eigen_q.y());
			q.setZ(eigen_q.z());
			q.setW(eigen_q.w());
			transform.setRotation(q);

			this->end_effector_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), node_namespace + "/base", node_namespace + "/calibrated_ee_pose"));	// change to generic name

			ros::spinOnce();
			publish_rate.sleep();
		}
	}

	void URCalibratedPosePub::readParams()
	{
		// Get parameter from parameter server
		this->private_nh_.param<std::string>("robot_ip", this->robot_ip_, "");
		if(this->robot_ip_ == "")
		{
			ROS_ERROR("No robot_ip parameter found");
			return;
		}

		this->private_nh_.param<std::string>("ur_joint_state_topic_name", this->ur_joint_state_topic_name_, "");
		if(this->ur_joint_state_topic_name_ == "")
		{
			ROS_ERROR("No ur_joint_state_topic_name parameter found");
			return;
		}

		this->private_nh_.param<std::string>("joint_prefix", this->joint_prefix_, "");

		this->private_nh_.param<std::string>("dh_parameter_switch", this->dh_parameter_switch_, "");
		if(this->dh_parameter_switch_ == "")
		{
			ROS_ERROR("No dh_parameter_switch parameter found");
			return;
		}

		// Get DH parameters from parameter server
		XmlRpc::XmlRpcValue dh_param_list;
		this->private_nh_.getParam("dh_params", dh_param_list);
		if(dh_param_list.getType() == XmlRpc::XmlRpcValue::TypeStruct)
		{
			XmlRpc::XmlRpcValue::ValueStruct::const_iterator joint_iterator;
			for(joint_iterator = dh_param_list.begin(); joint_iterator != dh_param_list.end(); joint_iterator++)
			{
				XmlRpc::XmlRpcValue joint_info_xmlrpc = joint_iterator->second;
				if(joint_info_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeStruct)
				{
					double theta = 0.0;
					double d = 0.0;
					double a = 0.0;
					double alpha = 0.0;

					if(joint_info_xmlrpc.hasMember("theta"))
					{
						if(joint_info_xmlrpc["theta"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
						{
							theta = (((double)joint_info_xmlrpc["theta"] * M_PI) / 180.0);
						}
						else
						{
							ROS_ERROR("DH parameter list - theta is not well formed");
							return;
						}
					}
					else
					{
						ROS_ERROR("DH parameter list - theta is not well formed");
						return;
					}
					
					if(joint_info_xmlrpc.hasMember("d"))
					{
						if(joint_info_xmlrpc["d"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
						{
							d = (double)joint_info_xmlrpc["d"];
						}
						else
						{
							ROS_ERROR("DH parameter list - d is not well formed");
							return;
						}
					}
					else
					{
						ROS_ERROR("DH parameter list - d is not well formed");
						return;
					}

					if(joint_info_xmlrpc.hasMember("a"))
					{
						if(joint_info_xmlrpc["a"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
						{
							a = (double)joint_info_xmlrpc["a"];
						}
						else
						{
							ROS_ERROR("DH parameter list - a is not well formed");
							return;
						}
					}
					else
					{
						ROS_ERROR("DH parameter list - a is not well formed");
						return;
					}

					if(joint_info_xmlrpc.hasMember("alpha"))
					{
						if(joint_info_xmlrpc["alpha"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
						{
							alpha = (((double)joint_info_xmlrpc["alpha"] * M_PI) / 180.0);
						}
						else
						{
							ROS_ERROR_STREAM("DH parameter list - alpha is not well formed.");
							return;
						}
					}
					else
					{
						ROS_ERROR("DH parameter list - alpha is not well formed");
						return;
					}

					dh_utils::DHTransformation dh_transformation = dh_utils::DHTransformation(theta, d, a, alpha);
					this->ideal_dh_transformations_list_.push_back(dh_transformation);
				}
				else
				{
					ROS_ERROR("DH parameter list is not well formed");
					return;
				}
			}
		}
		else
		{
			ROS_ERROR("DH parameter list is not well formed");
			return;
		}
	}

	void URCalibratedPosePub::getCalibratedDHParameter()
	{
		// Get the delta dh parameter directly from the UR controller
		urcl::comm::URStream<urcl::primary_interface::PrimaryPackage> stream(this->robot_ip_, urcl::primary_interface::UR_PRIMARY_PORT);
		urcl::primary_interface::PrimaryParser parser;
		urcl::comm::URProducer<urcl::primary_interface::PrimaryPackage> prod(stream, parser);
		ur_launch_hardware::URCalibrationConsumer consumer;

		urcl::comm::INotifier notifier;

		urcl::comm::Pipeline<urcl::primary_interface::PrimaryPackage> pipeline(prod, &consumer, "Pipeline", notifier);
		pipeline.run();
		while (!consumer.isCalibrated())
		{
			ros::Duration(0.1).sleep();
		}

		this->calibrated_dh_transformations_list_ = consumer.getDHTransformationsList();
	}

	void URCalibratedPosePub::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
	{
		// Add each the dh_parameter into the for loop. Otherwise the active joint values are not updated
		for(int joint_counter = 0; joint_counter < joint_state_msg->name.size(); joint_counter++)
		{
			if(joint_state_msg->name[joint_counter] == this->joint_prefix_+"shoulder_pan_joint")
			{
				this->ideal_dh_transformations_list_[0].setJointState(joint_state_msg->position[joint_counter]);
				this->calibrated_dh_transformations_list_[0].setJointState(joint_state_msg->position[joint_counter]);
			}
			else if(joint_state_msg->name[joint_counter] == this->joint_prefix_+"shoulder_lift_joint")
			{
				this->ideal_dh_transformations_list_[1].setJointState(joint_state_msg->position[joint_counter]);
				this->calibrated_dh_transformations_list_[1].setJointState(joint_state_msg->position[joint_counter]);
			}
			else if(joint_state_msg->name[joint_counter] == this->joint_prefix_+"elbow_joint")
			{
				this->ideal_dh_transformations_list_[2].setJointState(joint_state_msg->position[joint_counter]);
				this->calibrated_dh_transformations_list_[2].setJointState(joint_state_msg->position[joint_counter]);
			}
			else if(joint_state_msg->name[joint_counter] == this->joint_prefix_+"wrist_1_joint")
			{
				this->ideal_dh_transformations_list_[3].setJointState(joint_state_msg->position[joint_counter]);
				this->calibrated_dh_transformations_list_[3].setJointState(joint_state_msg->position[joint_counter]);
			}
			else if(joint_state_msg->name[joint_counter] == this->joint_prefix_+"wrist_2_joint")
			{
				this->ideal_dh_transformations_list_[4].setJointState(joint_state_msg->position[joint_counter]);
				this->calibrated_dh_transformations_list_[4].setJointState(joint_state_msg->position[joint_counter]);
			}
			else if(joint_state_msg->name[joint_counter] == this->joint_prefix_+"wrist_3_joint")
			{
				this->ideal_dh_transformations_list_[5].setJointState(joint_state_msg->position[joint_counter]);
				this->calibrated_dh_transformations_list_[5].setJointState(joint_state_msg->position[joint_counter]);
			}
		}
	}
}
