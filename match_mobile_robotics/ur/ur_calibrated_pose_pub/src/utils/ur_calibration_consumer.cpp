#include <ur_calibrated_pose_pub/utils/ur_calibration_consumer.h>

namespace ur_launch_hardware
{
	URCalibrationConsumer::URCalibrationConsumer() : calibrated_(false)
	{
		this->dh_transformations_list_ = std::vector<dh_utils::DHTransformation>();
	}

	bool URCalibrationConsumer::consume(std::shared_ptr<urcl::primary_interface::PrimaryPackage> product)
	{
		auto kin_info = std::dynamic_pointer_cast<urcl::primary_interface::KinematicsInfo>(product);
		if (kin_info != nullptr)
		{
			ROS_INFO_STREAM(product->toString().c_str());
			// DHRobot my_robot;
			for (size_t i = 0; i < kin_info->dh_a_.size(); ++i)
			{
				dh_utils::DHTransformation dh_transformation = dh_utils::DHTransformation(kin_info->dh_theta_[i],
																						  kin_info->dh_d_[i],
																						  kin_info->dh_a_[i],
																						  kin_info->dh_alpha_[i]);

				this->dh_transformations_list_.push_back(dh_transformation);
			}
			
			calibrated_ = true;
		}
		return true;
	}

	std::vector<dh_utils::DHTransformation> URCalibrationConsumer::getDHTransformationsList()
	{
		if (!calibrated_)
		{
			throw(std::runtime_error("Cannot get calibration, as no calibration data received yet"));
		}
		return this->dh_transformations_list_;  
	}
}  // namespace ur_calibration
