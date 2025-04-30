#ifndef DH_TRANSFORMATION_H_INCLUDED
#define DH_TRANSFORMATION_H_INCLUDED

#include <vector>
#include <memory>
#include <Eigen/Dense>

namespace dh_utils
{
    class DHTransformation
    {
    public:
        DHTransformation();
        DHTransformation(double theta, double d, double a, double alpha);
        virtual ~DHTransformation() = default;

        double getTheta();
        double getd();
        double geta();
        double getAlpha();

        Eigen::Matrix4d getTransformationMatrix();

        void setJointState(double current_joint_state);

        void calcTransformationMatrices();
        
    private:
        double current_joint_state_;

        double theta_;
        double d_;
        double a_;
        double alpha_;

        Eigen::Matrix4d theta_transformation_matrix_;
        Eigen::Matrix4d d_transformation_matrix_;
        Eigen::Matrix4d a_transformation_matrix_;
        Eigen::Matrix4d alpha_transformation_matrix_;
        Eigen::Matrix4d transformation_matrix_;
    };
}  // namespace dh_utils
#endif  // ifndef DH_TRANSFORMATION_H_INCLUDED
