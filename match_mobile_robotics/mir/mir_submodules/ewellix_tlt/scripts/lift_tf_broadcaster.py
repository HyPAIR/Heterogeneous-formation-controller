#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import tf

class Lift_tf_broadcaster():

    def __init__(self):
        rospy.init_node('lift_tf_broadcaster')
        self.get_params()
        self.lift_height_sub = rospy.Subscriber(self.lift_height_topic, JointState, self.lift_height_callback)
        rospy.spin()

    def get_params(self):
        self.lift_height_topic = rospy.get_param("~lift_height_topic")
        self.lift_link_name = rospy.get_param("~lift_link_name")
        self.lift_base_name = rospy.get_param("~lift_base_name")


    def lift_height_callback(self, msg):
        # broadcast lift height as tf
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, msg.position[0]),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     self.lift_link_name,
                     self.lift_base_name )




if __name__ == '__main__':
    lift_tf_broadcaster = Lift_tf_broadcaster()