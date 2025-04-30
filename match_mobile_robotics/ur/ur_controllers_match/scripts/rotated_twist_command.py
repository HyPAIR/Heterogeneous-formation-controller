#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import tf
from tf import transformations
import numpy as np

class PubRotatedTwist():
    def __init__(self, from_frame: str = None, to_frame: str = None):
        tf_listener = tf.TransformListener()
        tf_listener.waitForTransform(from_frame, to_frame, rospy.Time(0), rospy.Duration(5.0))
        # rot = tf_listener.lookupTransform(from_frame, to_frame, rospy.Time(0))[1] # rotation from base_footprint to ur/base
        rot = tf_listener.lookupTransform(to_frame, from_frame, rospy.Time(0))[1] # rotation from base_footprint to ur/base
        rot = transformations.quaternion_matrix(rot)[0:3,0:3] # rot as matrix
        # rot6 is 6x3 matrix filled with rot for rotation of all 6 degrees
        rot6 = np.zeros((6,6))
        rot6[:3,:3], rot6[3:,3:] = rot, rot
        self.rot6=rot6
        
        self.pub = rospy.Publisher('rotated_twist', Twist, queue_size=1)
        rospy.Subscriber("twist_cmd_ideal", Twist, self.cb_twist)
        
    def cb_twist(self, msg:Twist):
        v_in = np.array([*msg.linear.__reduce__()[2], *msg.angular.__reduce__()[2]])
        v_out = self.rot6@v_in
        
        msg.linear.x, msg.linear.y, msg.linear.z = v_out[:3]
        msg.angular.x, msg.angular.y, msg.angular.z = v_out[3:]
        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("rotate_twist")
    frame_from=rospy.get_param("~fromFrame", "/mur620c/base_footprint")
    frame_to=rospy.get_param("~toFrame", "/mur620c/UR10_l/base")
    PubRotatedTwist(frame_from, frame_to)
    rospy.spin()