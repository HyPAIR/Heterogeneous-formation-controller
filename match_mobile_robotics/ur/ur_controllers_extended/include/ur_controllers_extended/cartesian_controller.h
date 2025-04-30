///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////DANGEROUS PROTOTYPE////////////////////////////////////////////////////////////////////////////////
////////////////////////////DANGEROUS PROTOTYPE////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace ur_controllers_extended{

class CartesianController : public controller_interface::Controller<hardware_interface::PositionJointInterface>{

    public:
    
        bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n) override;
        void update(const ros::Time& time, const ros::Duration& period)override;
        void starting(const ros::Time& time) override;
        void stopping(const ros::Time& time) override;    

    private:
        KDL::Chain kinematics_;
        // boost::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
        // boost::shared_ptr<KDL::ChainFkSolverPos_recursive> dk_solver_;

        // std::vector<std::string> joint_names_;
        // std::vector< hardware_interface::JointHandle> joints_;
        
        // std::string base_link_name_;
        // std::string ee_link_name_;

        // Eigen::VectorXd getJointAngles();
};


}//end namespace