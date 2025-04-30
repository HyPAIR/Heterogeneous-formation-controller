#!/usr/bin/env python3
import rospy
import tf
from tf import transformations
from geometry_msgs.msg import Twist
import numpy
import math



class fake_nullspace_controller():
    
    
    def __init__(self):
        rospy.init_node('fake_nullspace_controller_node', anonymous=True)
        self.config()
        self.listener = tf.TransformListener()
        self.cmd_vel = Twist()
        self.error_trans = [0.0,0.0,0.0]
        self.local_error_trans=[0.0,0.0,0.0]
        
        self.pub = rospy.Publisher("/twist_controller/command",Twist,queue_size=1)
        self.listener.waitForTransform("map", "tool0", rospy.Time(), rospy.Duration(4.0))
        (self.initial_trans,self.initial_rot) = self.listener.lookupTransform('map', 'tool0', rospy.Time(0))
        self.euler_init = transformations.euler_from_quaternion(self.initial_rot)
        self.run()
        rospy.spin()
    
    def run(self):
        
        self.listener.waitForTransform("map", "tool0", rospy.Time(), rospy.Duration(4.0))
        Rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                (trans_mur,rot_mur) = self.listener.lookupTransform('map', 'tool0', rospy.Time(0))
                (trans_mir,rot_mir) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
                euler = transformations.euler_from_quaternion(rot_mur)
                #print(trans_mur,self.initial_trans)
                #quat=transformations.quaternion_about_axis(math.pi/2, (0,0,1))
                #rot_mir_q = transformations.quaternion_multiply(rot_mir,quat)
                matrix = transformations.quaternion_matrix(rot_mir)
                matrix_inv = numpy.linalg.inv(matrix)
                
                self.error_trans[0] = self.initial_trans[0] - trans_mur[0]
                self.error_trans[1] = self.initial_trans[1] - trans_mur[1]
                self.error_trans[2] = self.initial_trans[2] - trans_mur[2]
                error_rot = self.euler_init[2] - euler[2]
                
                self.local_error_trans[0] = matrix[0,0] * self.error_trans[0]  + matrix[0,1] * self.error_trans[1]
                self.local_error_trans[1] = matrix[1,0] * self.error_trans[0]  + matrix[1,1] * self.error_trans[1]
                #print(self.local_error_trans)
                
                cmd_vel_x = 0.8 * self.local_error_trans[0]
                cmd_vel_y = 0.8 * self.local_error_trans[1]
                
                if abs(cmd_vel_x) > 0.2:
                    cmd_vel_x = cmd_vel_x / abs(cmd_vel_x) * 0.2
                if abs(cmd_vel_y) > 0.2:
                    cmd_vel_y = cmd_vel_y / abs(cmd_vel_y) * 0.2
                     
                self.cmd_vel.linear.x = cmd_vel_x
                self.cmd_vel.linear.y = cmd_vel_y
                
                print(self.cmd_vel)
                self.pub.publish(self.cmd_vel)
                
                
                #print(self.error_trans,error_rot)
                Rate.sleep()
                #print(euler)
            except (tf.LookupException, tf.ConnectivityException):
                continue
            
            
    def config(self):
        pass
            

if __name__ == '__main__':
    fake_nullspace_controller()