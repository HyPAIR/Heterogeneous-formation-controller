///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////DANGEROUS PROTOTYPE////////////////////////////////////////////////////////////////////////////////
////////////////////////////DANGEROUS PROTOTYPE////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <ur_controllers_extended/compliant_axis_controller.h>

namespace ur_controllers_extended{


bool CompliantAxisController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
{
    if(!n.getParam("joint_stiffness", this->stiffness_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "joint_stiffness" << "' (namespace: " << n.getNamespace() << ").");
    }
    if(!n.getParam("d_gain", this->d_gain_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "d_gain" << "' (namespace: " << n.getNamespace() << ").");
    }
    // List of controlled joints
    if(!n.getParam("joints", this->joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "joints" << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    if(this->joint_names_.size() == 0){
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }
    for(unsigned int i=0; i<this->joint_names_.size(); i++)
    {
      try
      {
        this->joints_.push_back(hw->getHandle(this->joint_names_[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }
    }

    std::string direction;
    if(!n.getParam("direction", direction))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "direction" << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }  
    else
    {
      if(direction=="x")
      {
        this->direction_=Direction::x_axis;
      }
      else if(direction=="y")
      {
        this->direction_=Direction::y_axis;
      }
      else if(direction=="z")
      {
        this->direction_=Direction::z_axis;
      }
      else
      {
        ROS_ERROR_STREAM("Wrong torque direction specified! Possibilities are: 'x', 'y', 'z'");
        return false;
      }
      
    }

    std::string topic_name;
    if(!n.getParam("topic_name", topic_name))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "topic_name" << "' (namespace: " << n.getNamespace() << ").");
      return false;
    } 
    this->wrench_sub_=n.subscribe(topic_name,1,&CompliantAxisController::wrenchCallback,this);

    
    this->torque_=0.0; 
    this->vel_old_=0.0;
    return true;

}

void CompliantAxisController::starting(const ros::Time& time)
{
  //Holding current position
  for(unsigned int i=0; i<this->joints_.size(); i++)
  {
    this->joints_[i].setCommand(this->joints_[i].getPosition());
    
  }
  this->position_equi_=this->joints_.back().getPosition();
}



void CompliantAxisController::stopping(const ros::Time& time)
{}


void CompliantAxisController::update(const ros::Time& time, const ros::Duration& period)
{ 
  // double acc=(this->joints_.back().getVelocity()-this->vel_old_)/period.toSec();
  // this->vel_old_=this->joints_.back().getVelocity();

  // double angle=(this->torque_+acc*0.0006)/this->stiffness_+this->position_equi_;
  double angle=this->torque_/this->stiffness_+this->position_equi_+this->d_gain_*this->joints_.back().getPosition();
  this->joints_.back().setCommand(angle);
}

void CompliantAxisController::wrenchCallback(geometry_msgs::WrenchStamped msg)
{  
  switch (this->direction_)
  {
  case Direction::x_axis:
    this->torque_=msg.wrench.torque.x;
    break;
  case Direction::y_axis:
    this->torque_=msg.wrench.torque.y;
    break;
  case Direction::z_axis:
    this->torque_=msg.wrench.torque.z;
    break;  
  default:
    ROS_ERROR_STREAM("Direction of torque is not specified!");
    throw std::runtime_error("Direction of torque is not specified!");
    break;
  }  
}

}//namespace

PLUGINLIB_EXPORT_CLASS(ur_controllers_extended::CompliantAxisController,
                       controller_interface::ControllerBase)
