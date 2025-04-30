#!/usr/bin/env python3
import rospy
import tf
from tf import transformations
from geometry_msgs.msg import PoseStamped, Pose
from copy import deepcopy
import math

class ReferenceMocapToMirMap:
    def __init__(self):
        self.mir_pose_topic = rospy.get_param('~mir_pose_topic', '/mur620c/mir/robot_pose')
        self.mocap_pose_topic = rospy.get_param('~mocap_pose_topic', '/qualisys/mur620c/pose')
        rospy.Subscriber(self.mir_pose_topic, Pose, self.mir_pose_callback)
        rospy.Subscriber(self.mocap_pose_topic, PoseStamped, self.mocap_pose_callback)

        self.mir_poses = []
        self.mocap_poses = []
        self.thetas_mir = []
        self.thetas_mocap = []
        
        # wait for first messages
        rospy.loginfo('Waiting for first messages...')
        rospy.wait_for_message(self.mir_pose_topic, Pose)
        rospy.loginfo('Got first message from mir_pose_topic')
        rospy.wait_for_message(self.mocap_pose_topic, PoseStamped)
        rospy.loginfo('Got first message from mocap_pose_topic')
        rospy.sleep(3.0)
        self.run()
        # self.compute_theta()
        
    def mir_pose_callback(self, msg):
        self.mir_poses.append(deepcopy(msg))
        self.thetas_mir.append(transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])[2])
        self.thetas_mocap.append(self.current_mocap_theta)
        
    def mocap_pose_callback(self, msg):
        self.mocap_poses.append(deepcopy(msg.pose))
        self.current_mocap_theta = transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]
    
    
    def compute_theta(self):
        diff = [0.0 for i in range(len(self.thetas_mir))]
        for i in range(len(self.thetas_mir)):
            diff[i] = self.thetas_mir[i] - self.thetas_mocap[i]
            if diff[i] > math.pi:
                diff[i] -= 2 * math.pi
            elif diff[i] < -math.pi:
                diff[i] += 2 * math.pi
                
        print('diff: ', diff)
            
        # compute average
        avg = sum(diff) / len(diff)
        print('avg: ', avg)
            
    
    
    def run(self):
        # compute average pose
        mir_pose_avg_start = self.average_pose(self.mir_poses)
        mocap_pose_avg_start = self.average_pose(self.mocap_poses)
        diff = 1.254877135437004
        
        theta_mir = transformations.euler_from_quaternion([mir_pose_avg_start.orientation.x, mir_pose_avg_start.orientation.y, mir_pose_avg_start.orientation.z, mir_pose_avg_start.orientation.w])[2]
        theta_mocap = transformations.euler_from_quaternion([mocap_pose_avg_start.orientation.x, mocap_pose_avg_start.orientation.y, mocap_pose_avg_start.orientation.z, mocap_pose_avg_start.orientation.w])[2]
        # print('mir_pose_avg_start: ', mir_pose_avg_start)
        # print('theta_mir: ', theta_mir)
        # print('mocap_pose_avg_start: ', mocap_pose_avg_start)
        # print('theta_mocap: ', theta_mocap)
        
        R = transformations.rotation_matrix(-diff, (0, 0, 1))
        
        mocap_pose_transformed = Pose()
        mocap_pose_transformed.position.x = R[0, 0] * mocap_pose_avg_start.position.x + R[0, 1] * mocap_pose_avg_start.position.y
        mocap_pose_transformed.position.y = R[1, 0] * mocap_pose_avg_start.position.x + R[1, 1] * mocap_pose_avg_start.position.y
        
        pose_diff = [mocap_pose_transformed.position.x - mir_pose_avg_start.position.x, mocap_pose_transformed.position.y - mir_pose_avg_start.position.y]
        
        print("pose_diff", pose_diff)
        
        return 0
        

        # wait for the user to move the robot
        rospy.loginfo('Please move the robot to a new position and press enter...')
        input()
        
        self.mir_poses = []
        self.mocap_poses = []
        rospy.sleep(3.0)
        
        mir_pose_avg_end = self.average_pose(self.mir_poses)
        mocap_pose_avg_end = self.average_pose(self.mocap_poses)
        
        theta_mir = transformations.euler_from_quaternion([mir_pose_avg_end.orientation.x, mir_pose_avg_end.orientation.y, mir_pose_avg_end.orientation.z, mir_pose_avg_end.orientation.w])[2]
        theta_mocap = transformations.euler_from_quaternion([mocap_pose_avg_end.orientation.x, mocap_pose_avg_end.orientation.y, mocap_pose_avg_end.orientation.z, mocap_pose_avg_end.orientation.w])[2]
        
        print('mir_pose_avg_end: ', mir_pose_avg_end)
        print('theta_mir: ', theta_mir)
        print('mocap_pose_avg_end: ', mocap_pose_avg_end)
        print('theta_mocap: ', theta_mocap)
        
        
        
        
        
        # check if dist is the same
        mir_poses_dist = ((mir_pose_avg_end.position.x - mir_pose_avg_start.position.x)**2 + (mir_pose_avg_end.position.y - mir_pose_avg_start.position.y)**2 + (mir_pose_avg_end.position.z - mir_pose_avg_start.position.z)**2)**0.5
        mocap_poses_dist = ((mocap_pose_avg_end.position.x - mocap_pose_avg_start.position.x)**2 + (mocap_pose_avg_end.position.y - mocap_pose_avg_start.position.y)**2 + (mocap_pose_avg_end.position.z - mocap_pose_avg_start.position.z)**2)**0.5
        
        if abs(mir_poses_dist - mocap_poses_dist) > 0.05:
            rospy.logerr('Distance traveled by MIR and mocap is not the same. Please try again.')
            
        
        


    

        
        
    def average_pose(self,poses):
        avg = Pose()
        for pose in poses:
            avg.position.x += pose.position.x * 1.0/len(poses)
            avg.position.y += pose.position.y * 1.0/len(poses)
            avg.position.z += pose.position.z * 1.0/len(poses)
            # todo: average quaternion instead of summing
            avg.orientation.x += pose.orientation.x * 1.0/len(poses)
            avg.orientation.y += pose.orientation.y * 1.0/len(poses)
            avg.orientation.z += pose.orientation.z * 1.0/len(poses)
            avg.orientation.w += pose.orientation.w * 1.0/len(poses)

        # normalize quaternion
        norm = (avg.orientation.x**2 + avg.orientation.y**2 + avg.orientation.z**2 + avg.orientation.w**2)**0.5
        
        if norm == 0:
            rospy.logerr('quaternion not valid')
            return None
                    
        avg.orientation.x /= norm
        avg.orientation.y /= norm
        avg.orientation.z /= norm
        avg.orientation.w /= norm
        
        return avg
            
if __name__ == '__main__':
    rospy.init_node('reference_mocap_to_mir_map')
    ReferenceMocapToMirMap()
