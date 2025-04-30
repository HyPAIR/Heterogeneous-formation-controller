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

#include <dynamic_reconfigure/server.h>
#include <ur_controllers_extended/CompliantVelocityConfig.h>
#include <std_msgs/Float64.h>

namespace ur_controllers_extended{

class CompliantAxisVelController :public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
    public:
        enum Direction{
            x_axis=0,
            y_axis=1,
            z_axis=2
        };
        bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n) override;
        void update(const ros::Time& time, const ros::Duration& period)override;
        void starting(const ros::Time& time) override;
        void stopping(const ros::Time& time) override;
    private:
        std::unique_ptr<dynamic_reconfigure::Server<ur_controllers_extended::CompliantVelocityConfig>> config_server_;
        ros::Subscriber wrench_sub_;        

        std::vector<std::string>  joint_names_;       
        std::vector<hardware_interface::JointHandle> joints_;
        bool measure_;
        double p_gain_;        
        double virtual_damping_;
        double theta_model_;
        double force_thresh_;

        double torque_;
        double acceleration_;
        double vel_old_;

        ros::Time time_old_;
        ros::Duration d_time_;
        
        Direction direction_;

        void wrenchCallback(geometry_msgs::WrenchStamped msg);
        void dynConfigcallback(ur_controllers_extended::CompliantVelocityConfig &config, uint32_t level);

};

}//namespace