# include <ur_calibrated_pose_pub/utils/dh_transformation.h>

namespace dh_utils
{
	DHTransformation::DHTransformation()
	{

	}

	DHTransformation::DHTransformation(double theta, double d, double a, double alpha)
	{
		this->d_ = d;
		this->a_ = a;
		this->theta_ = theta;
		this->alpha_ = alpha;
	}

	// Getter Methods
	double DHTransformation::getTheta()
	{
		return this->theta_;
	}

	double DHTransformation::getd()
	{
		return this->d_;
	}

	double DHTransformation::geta()
	{
		return this->a_;
	}

	double DHTransformation::getAlpha()
	{
		return this->alpha_;
	}

	Eigen::Matrix4d DHTransformation::getTransformationMatrix()
	{
		this->calcTransformationMatrices();
		return this->transformation_matrix_;
	}

	void DHTransformation::setJointState(double current_joint_state)
	{
		this->current_joint_state_ = current_joint_state;
	}

	void DHTransformation::calcTransformationMatrices()
	{
		this->theta_transformation_matrix_ << cos(this->getTheta() + this->current_joint_state_), -sin(this->getTheta() + this->current_joint_state_), 0, 0,
			sin(this->getTheta() + this->current_joint_state_), cos(this->getTheta() + this->current_joint_state_), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		this->d_transformation_matrix_ << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, this->getd(),
			0, 0, 0, 1;

		this->a_transformation_matrix_ << 1, 0, 0, this->geta(),
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		this->alpha_transformation_matrix_<< 1, 0, 0, 0,
			0, cos(this->getAlpha()), -sin(this->getAlpha()), 0,
			0, sin(this->getAlpha()), cos(this->getAlpha()), 0,
			0, 0, 0, 1;

		// According to the UR documentation the transformation matrix is calculated as follows:
		// trans in Z, rot in Z, trans in X and rot in X (trans in z and rot in z could be swapped)
		this->transformation_matrix_ = this->d_transformation_matrix_ * this->theta_transformation_matrix_ * this->a_transformation_matrix_ * this->alpha_transformation_matrix_;
	}
}