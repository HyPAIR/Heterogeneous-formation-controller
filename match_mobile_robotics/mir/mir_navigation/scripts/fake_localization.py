#!/usr/bin/env python3 

# this node is used to publish a fake localization for all robots using the ground truth pose data
import rospy

import tf
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseWithCovarianceStamped

class FakeLocalization():

    # The transformation is published between these two frames
    parent_frame: str
    child_frame: str

    pose_old: Odometry # Position of robot in last cycle
    time_stamp_old: rospy.Time # Time stamp of last cycle
    redundant_timestamp_index: int
    
    def __init__(self):
        rospy.logerr("Starting fake_localization_node")

        # Initialize ros node and member variables
        rospy.init_node('fake_localization_node')
        self.pose_old = Odometry()
        self.time_stamp_old = rospy.Time.now()
        self.redundant_timestamp_index = 0

        # Read params
        self.parent_frame = rospy.get_param('~parent_frame',"")
        self.child_frame = rospy.get_param('~child_frame',"")

        # Initialize Subscriber/Publisher/Services/Actions
        rospy.Subscriber('ground_truth', Odometry, self.__fake_localization_handler)
        
        rospy.spin()
    
    def __fake_localization_handler(self,msg: Odometry):
        t: rospy.Time = rospy.Time.now()
        br: tf.TransformBroadcaster = tf.TransformBroadcaster()
        if (msg.pose.pose.position.x == self.pose_old.pose.pose.position.x and msg.pose.pose.position.y == self.pose_old.pose.pose.position.y and msg.pose.pose.orientation.x == self.pose_old.pose.pose.orientation.x 
        and msg.pose.pose.orientation.y == self.pose_old.pose.pose.orientation.y and msg.pose.pose.orientation.z == self.pose_old.pose.pose.orientation.z and msg.pose.pose.orientation.w == self.pose_old.pose.pose.orientation.w):
            pass
        elif abs(t-self.time_stamp_old) > rospy.Duration(0.001):
            # publish fake localization
            self.redundant_timestamp_index = 0
            br.sendTransform((msg.pose.pose.position.x,msg.pose.pose.position.y,0), # publish the footprint of the robot
                            (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
                            t,
                            self.child_frame,
                            self.parent_frame)
        else:
            self.redundant_timestamp_index += 1
            br.sendTransform((msg.pose.pose.position.x,msg.pose.pose.position.y,0), # publish the footprint of the robot
                            (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
                            t + rospy.Duration(0.0001)*self.redundant_timestamp_index, # avoid redundant timestamp
                            self.child_frame,
                            self.parent_frame)
        self.time_stamp_old = t

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    fake_localization: FakeLocalization = FakeLocalization()
    fake_localization.run()
