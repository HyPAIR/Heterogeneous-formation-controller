#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from tf import transformations
import tf2_ros
from copy import deepcopy
from helper_nodes.virtual_leader_helper import Virtual_leader_helper
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

class virtual_leader:

    def __init__(self):
        rospy.init_node("virtual_leader_node")
        rospy.loginfo("virtual_leader_node running")
        Virtual_leader_helper(self)
        self.run()
        rospy.spin()

    # The virutal leader node simulates the leader of the formation. It publishes the leader pose and velocity. The target leader pose is computed 
    # from the target leader pose and the target leader velocity by integration. 
    def run(self):
        Rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            _master_vel = deepcopy(self.master_vel) # copy the master_vel to avoid overwriting the original
            _mrs_vel_subscriber_time = deepcopy(self.mrs_vel_subscriber_time) # copy the master_vel to avoid overwriting the original

            time_current = rospy.get_time()
            dt = time_current - self.time_old
            self.time_old = time_current
            
            # calculate the distance the leader has moved since the last time
            self.d_pose[0] = _master_vel.linear.x * dt
            self.d_pose[1] = _master_vel.linear.y * dt
            self.d_pose[2] = _master_vel.angular.z * dt
            
            # transform the distance to the global frame
            R = transformations.euler_matrix(0,0,self.leader_orientation)
            self.d_pose_R[0] = R[0,0] * self.d_pose[0] + R[0,1] * self.d_pose[1] 
            self.d_pose_R[1] = R[1,0] * self.d_pose[0] + R[1,1] * self.d_pose[1] 
            self.d_pose_R[2] = self.d_pose[2]
            
            # calculate the new pose of the leader
            self.leader_pose.pose.position.x = self.leader_pose.pose.position.x + self.d_pose_R[0]
            self.leader_pose.pose.position.y = self.leader_pose.pose.position.y + self.d_pose_R[1]
            self.leader_orientation = self.leader_orientation + self.d_pose_R[2]
            
            q = transformations.quaternion_from_euler(0,0,self.leader_orientation)
            self.leader_pose.pose.orientation.x = q[0]
            self.leader_pose.pose.orientation.y = q[1]
            self.leader_pose.pose.orientation.z = q[2]
            self.leader_pose.pose.orientation.w = q[3]
            
            self.leader_pose.header.stamp = rospy.Time.now()
            self.leader_pose.header.frame_id = 'map'

            self.pub.publish(self.leader_pose)

            br = tf2_ros.TransformBroadcaster()
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "virtual_leader/odom"
            t.transform.translation = self.leader_pose.pose.position
            t.transform.rotation = self.leader_pose.pose.orientation
            br.sendTransform(t)

            self.pub_vel.publish(_master_vel)
                
            Rate.sleep()


if __name__=="__main__":
    vl = virtual_leader()
    vl.run()