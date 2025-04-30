///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////DANGEROUS PROTOTYPE////////////////////////////////////////////////////////////////////////////////
////////////////////////////DANGEROUS PROTOTYPE////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <ur_controllers_extended/cartesian_controller.h>
namespace ur_controllers_extended{
    
bool CartesianController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
{
    std::string robot_desc_string;
    KDL::Tree tree;
    ros::NodeHandle priv("~");

    n.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string,tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }   
    
    // if(!priv.getParam("base_link_frame",this->base_link_name_))
    // {
    //     ROS_ERROR("Failed to load name of base link");
    //     return false;
    // } 
    // if(!priv.getParam("ee_link_frame",this->base_link_name_))
    // {
    //     ROS_ERROR("Failed to load name of endeffector link");
    //     return false;
    // } 
    // if(!tree.getChain(this->base_link_name_,this->ee_link_name_,this->kinematics_))
    // {
    //     ROS_ERROR("Failed to load kinematic chain from tree");
    //     return false;
    // } 

    // std::string param_name = "joints";
    // if(!n.getParam(param_name, this->joint_names_))
    // {
    //   ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
    //   return false;
    // }
    // for(unsigned int i=0; i<this->joints_.size(); i++)
    // {
    //   try
    //   {
    //     this->joints_.push_back(hw->getHandle(this->joint_names_[i]));
    //   }
    //   catch (const hardware_interface::HardwareInterfaceException& e)
    //   {
    //     ROS_ERROR_STREAM("Exception thrown: " << e.what());
    //     return false;
    //   }
    // }
    // if(this->kinematics_.getNrOfJoints()!=this->joints_.size())
    // {
    //     ROS_ERROR("Number of commandable joints doesnt fit number of joints in kinematics!");
    //     return false;
    // }
    // this->ik_solver_=boost::make_shared<KDL::ChainIkSolverPos_LMA>(this->kinematics_);
    // this->dk_solver_=boost::make_shared<KDL::ChainFkSolverPos_recursive>(this->kinematics_);

    return true;
}

void CartesianController::update(const ros::Time& time, const ros::Duration& period)
{
    
}

void CartesianController::starting(const ros::Time& time)
{

    // KDL::JntArray angles;
    // angles.data=this->getJointAngles();
    // KDL::Frame ee;
    // this->dk_solver_->JntToCart(angles,ee);
    // ROS_INFO_STREAM(ee(1,1)<<"\t"<<ee(1,2)<<"\t"<<ee(1,3)<<"\t"<<ee(1,4)<<"\t"<<
    //                 ee(2,1)<<"\t"<<ee(2,2)<<"\t"<<ee(2,3)<<"\t"<<ee(2,4)<<"\t"<<
    //                 ee(3,1)<<"\t"<<ee(3,2)<<"\t"<<ee(3,3)<<"\t"<<ee(3,4)<<"\t"<<
    //                 ee(4,1)<<"\t"<<ee(4,2)<<"\t"<<ee(4,3)<<"\t"<<ee(4,4)<<"\t");
}

void CartesianController::stopping(const ros::Time& time)
{

}

// Eigen::VectorXd CartesianController::getJointAngles()
// {
//     Eigen::VectorXd angles(this->joints_.size());
//     for(int i=0;i<this->joints_.size();i++)
//     {
//         angles(i)=this->joints_[i].getPosition();
//     }
//     return angles;
    
// }

}//end namespace

PLUGINLIB_EXPORT_CLASS(ur_controllers_extended::CartesianController,
                       controller_interface::ControllerBase)
