///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////DANGEROUS PROTOTYPE////////////////////////////////////////////////////////////////////////////////
////////////////////////////DANGEROUS PROTOTYPE////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/WrenchStamped.h>

namespace ur_controllers_extended{

class CompliantAxisController :public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
    public:
        enum Direction{
            x_axis=0,
            y_axis=1,
            z_axis=2
        };
        bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n) override;
        void update(const ros::Time& time, const ros::Duration& period)override;
        void starting(const ros::Time& time) override;
        void stopping(const ros::Time& time) override;
    private:
        ros::Subscriber wrench_sub_;
        std::vector<std::string>  joint_names_;       
        std::vector<hardware_interface::JointHandle> joints_;
        bool measure_;
        double vel_old_;
        double d_gain_;
        double stiffness_;
        double torque_;
        double position_equi_;
        Direction direction_;

        void wrenchCallback(geometry_msgs::WrenchStamped msg);

};

}//namespace