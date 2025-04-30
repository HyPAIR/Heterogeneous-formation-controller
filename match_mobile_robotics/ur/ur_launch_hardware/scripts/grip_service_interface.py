#!/usr/bin/env python3
import rospy
from std_srvs.srv import SetBool,SetBoolResponse
from std_msgs.msg import String
from ur_msgs.srv import SetIO,SetIORequest
from ur_msgs.msg import IOStates


def gripCallback(req):
    global client
    goal=SetIORequest()
    io=rospy.wait_for_message("ur_hardware_interface/io_states",IOStates)

    states=io.digital_out_states
    
    if not req.data:
        goal.pin=16
        goal.state=False     
        goal.fun=1   
        client.call(goal)    
        msg="Released"
    else:
        goal.pin=16
        goal.state=True     
        goal.fun=1   
        client.call(goal)    
        msg="Gripped"
  
    return {"success":True,"message":msg}
        


if __name__=="__main__":    
    global client
    rospy.init_node("grip_service_interface")
    client=rospy.ServiceProxy("ur_hardware_interface/set_io",SetIO)
    rospy.Service("grip",SetBool,gripCallback)
    rospy.spin()
    