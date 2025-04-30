///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////DANGEROUS PROTOTYPE////////////////////////////////////////////////////////////////////////////////
////////////////////////////DANGEROUS PROTOTYPE////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <ur_controllers_extended/compliant_axis_vel_controller.h>

namespace ur_controllers_extended{


bool CompliantAxisVelController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n)
{
    //Virtual damping
    if(!n.getParam("virtual_damping", this->virtual_damping_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "virtual_damping" << "' (namespace: " << n.getNamespace() << ").");
    }
    //Force threshold
    if(!n.getParam("force_thresh", this->force_thresh_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "force_thresh" << "' (namespace: " << n.getNamespace() << ").");
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
    this->wrench_sub_=n.subscribe("wrench",1,&CompliantAxisVelController::wrenchCallback,this);
    this->config_server_=std::make_unique<
      dynamic_reconfigure::Server<ur_controllers_extended::CompliantVelocityConfig>>(
      n);
    this->config_server_->setCallback(boost::bind(&CompliantAxisVelController::dynConfigcallback,this,_1,_2));     



    this->time_old_=ros::Time::now();
    this->torque_=0.0;

    return true;

}

void CompliantAxisVelController::starting(const ros::Time& time)
{
  //Holding current position
  for(unsigned int i=0; i<this->joints_.size(); i++)
  {
    this->joints_[i].setCommand(0.0);    
  }
}



void CompliantAxisVelController::stopping(const ros::Time& time)
{}


void CompliantAxisVelController::update(const ros::Time& time, const ros::Duration& period)
{ 

  double vel=0.0;
  if(this->virtual_damping_>0.0)
  {
    vel=1.0/this->virtual_damping_*this->torque_;
  }
  else
  {
    vel=0.0;
  }
  ROS_INFO_STREAM(vel);
  
  this->joints_.back().setCommand(vel);  
}

void CompliantAxisVelController::wrenchCallback(geometry_msgs::WrenchStamped msg)
{
  double torque_measure=0.0;
  switch (this->direction_)
  {   
    case Direction::x_axis:
      torque_measure=msg.wrench.torque.x;
      break;
    case Direction::y_axis:
      torque_measure=msg.wrench.torque.y;
      break;
    case Direction::z_axis:
      torque_measure=msg.wrench.torque.z;
      break;  
  default:
    ROS_ERROR_STREAM("Direction of torque is not specified!");
    throw std::runtime_error("Direction of torque is not specified!");
    break;
  }
  if(std::abs(torque_measure)<this->force_thresh_)  
  {
    torque_measure=0.0;
  }
  this->torque_=torque_measure;
}
void CompliantAxisVelController::dynConfigcallback(ur_controllers_extended::CompliantVelocityConfig &config, uint32_t level) {
    ROS_INFO_STREAM("Reconfigure Request: "<<config.virtual_damping<<"\t"<<config.force_thresh);
    this->virtual_damping_=config.virtual_damping;   
    this->force_thresh_=config.force_thresh;
}
}//namespace

PLUGINLIB_EXPORT_CLASS(ur_controllers_extended::CompliantAxisVelController,
                       controller_interface::ControllerBase)
