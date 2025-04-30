#!/usr/bin/env python3
import rospy
import tf
from tf import transformations
from geometry_msgs.msg import PoseStamped, Pose
from math import cos, sin, pi

class ExternalLocalizationBroadcaster:
    def __init__(self):
        self.tf_prefix = rospy.get_param('~tf_prefix', 'mur620d')
        self.pose_broadcaster = tf.TransformBroadcaster()
        self.localization_topic = rospy.get_param('~localization_topic', '/qualisys/mur620d/pose')
        self.mocap_offset = rospy.get_param('~mocap_offset', [37.30436983166483, 34.421838301291274, 0.3967520289935211])
        self.sub = rospy.Subscriber(self.localization_topic, PoseStamped, self.callback)
        self.timestamp = rospy.Time.now()
        rospy.spin()

    def callback(self, msg = PoseStamped()):
        now = rospy.Time.now()
        if (now - self.timestamp) <= rospy.Duration(0.0):
            now = self.timestamp + rospy.Duration(0.001)
            # alter the timestamp to avoid tf2 buffer error
        self.timestamp = now
        pose = self.transform_pose_to_mir_map(msg.pose)
        self.pose_broadcaster.sendTransform(
            (pose.position.x, pose.position.y, pose.position.z), 
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            now,
            'map',
            self.tf_prefix + '/base_footprint_transformed'   
        )
        print("pose", pose.position.x, pose.position.y, pose.position.z)
              
              
    def transform_pose_to_mir_map(self, pose = Pose()):
        pose_out = Pose()
        # apply offset
        pose_out.position.x = self.mocap_offset[0] + pose.position.x * cos(self.mocap_offset[2]) - pose.position.y * sin(self.mocap_offset[2])
        pose_out.position.y = self.mocap_offset[1] + pose.position.x * sin(self.mocap_offset[2]) + pose.position.y * cos(self.mocap_offset[2])
        orientation = self.mocap_offset[2] + transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]
        q = transformations.quaternion_from_euler(0, 0, orientation)
        pose_out.orientation.x = q[0]
        pose_out.orientation.y = q[1]
        pose_out.orientation.z = q[2]
        pose_out.orientation.w = q[3]
        
        return pose_out

            
if __name__ == '__main__':
    rospy.init_node('external_localization_broadcaster')
    broadcaster = ExternalLocalizationBroadcaster()
