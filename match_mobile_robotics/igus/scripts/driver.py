#!/usr/bin/env python3
import rospy
import socket

from sensor_msgs.msg import JointState



class igus_TCP_driver():


    def __init__(self):
        rospy.init_node("igus_TCP_driver")
        TCP_IP = '192.168.3.11'
        self.mode = "joint"
        TCP_PORT = 3920
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((TCP_IP, TCP_PORT))
        self.dq = [0.0 , 0.0, 0.0, 0.0, 0.0]
        self.q = [0.0 , 0.0, 0.0, 0.0, 0.0]
        self.q_act = [0.0 , 0.0, 0.0, 0.0, 0.0]
        self.pose_act = [0.0 , 0.0, 0.0, 0.0, 0.0]
        self.eI = [0.0 , 0.0, 0.0, 0.0, 0.0]
        self.Kp = 1
        self.Ki = 0.0
        rospy.Subscriber("/igus/cmd_joint_vel", JointState, self.joint_cmd_cb)
        self.run()
        
        
        rospy.spin()

    def run(self):  
        message_idx = 1
        #rate = rospy.Rate(10)
        
        if self.mode == "cartasian_base":
            MESSAGE = "CRISTART " + str(message_idx) + " CMD MotionTypeCartBase CRIEND"
            self.socket.send(MESSAGE.encode())
            self.socket.recv(1024)
            message_idx += 1
        elif self.mode == "joint":
            MESSAGE = "CRISTART " + str(message_idx) + " CMD MotionTypeJoint CRIEND"
            self.socket.send(MESSAGE.encode())
            message_idx += 1
            rospy.sleep(0.1)
            self.socket.recv(1024)
        elif self.mode == "cartasian_tool":
            MESSAGE = "CRISTART " + str(message_idx) + " CMD MotionTypeCartTool CRIEND"
            self.socket.send(MESSAGE.encode())
            message_idx += 1
            rospy.sleep(0.1)
            self.socket.recv(1024)
            
            
        while not rospy.is_shutdown():
            message_idx += 1
            for i in range(0,4):
                if abs(self.dq[i]) < 0.01:
                    self.dq[i] = 0 
            MESSAGE = "CRISTART " + str(message_idx) + " ALIVEJOG " + str(self.dq[0]) + " " + str(self.dq[1]) + " " + str(self.dq[2]) + " " + str(self.dq[3]) + " " + str(self.dq[4]) + " 0.0 0.0 0.0 0.0 CRIEND"
            #print(MESSAGE)

            self.socket.send(MESSAGE.encode())
            self.respone = self.socket.recv(1024)
            R = self.respone.split(" ")
            #print(R)
            if len(R)> 10 and R[3] == "MODE":
                self.q_act[0] = float(R[6])
                self.q_act[1] = float(R[7])
                self.q_act[2] = float(R[8])
                self.q_act[3] = float(R[9])
                self.q_act[4] = float(R[10])
                self.pose_act[0] = float(R[40])
                self.pose_act[1] = float(R[41])
                self.pose_act[2] = float(R[42])
                self.pose_act[3] = float(R[43])
                self.pose_act[4] = float(R[44])
                rospy.loginfo_throttle(1,self.pose_act)
            if message_idx > 9998:
                message_idx = 1
        self.socket.close()

    
    def joint_pose_controller(self):
        e = [0.0 , 0.0, 0.0, 0.0, 0.0]
        e[0] = self.q[0] - self.q_act[0] 
        e[1] = self.q[1] - self.q_act[1] 
        e[2] = self.q[2] - self.q_act[2] 
        e[3] = self.q[3] - self.q_act[3] 
        e[4] = self.q[4] - self.q_act[4] 
        
        self.dq[0] = self.Kp * e[0]
        self.dq[1] = self.Kp * e[1]
        self.dq[2] = self.Kp * e[2]
        self.dq[3] = self.Kp * e[3]
        self.dq[4] = self.Kp * e[4]
        #print(self.dq)
        
    def cart_pose_controller(self):
        e = [0.0 , 0.0, 0.0, 0.0, 0.0]
        e[0] = self.q[0] - self.pose_act[0] 
        e[1] = self.q[1] - self.pose_act[1] 
        e[2] = self.q[2] - self.pose_act[2] 
        e[3] = self.q[3] - self.pose_act[3] 
        e[4] = self.q[4] - self.pose_act[4] 
        
        self.eI[0] = self.eI[0] * 0.99 + e[0] 
        self.eI[1] = self.eI[1] * 0.99 + e[1]
        self.eI[2] = self.eI[2] * 0.99 + e[2]
        self.eI[3] = self.eI[3] * 0.99 + e[3]
        self.eI[4] = self.eI[4] * 0.99 + e[4]
        
        self.dq[0] = self.Kp * e[0] + self.Ki * self.eI[0]
        self.dq[1] = self.Kp * e[1] + self.Ki * self.eI[1]
        self.dq[2] = self.Kp * e[2] + self.Ki * self.eI[2]
        self.dq[3] = self.Kp * e[3] + self.Ki * self.eI[3]
        self.dq[4] = self.Kp * e[4] + self.Ki * self.eI[4]
        #print(self.dq)


    def joint_cmd_cb(self,data):
        dq = data.velocity
        q = data.position
        if len(q) > 4:
            self.q = q
            if self.mode == "cartasian_base":
                self.cart_pose_controller()
            elif self.mode == "joint":
                self.joint_pose_controller()
        elif len(dq) > 4:
            self.dq = dq
        else:
            rospy.logerr_throttle(1,"Joint command not correctly formated")


if __name__=="__main__":
    igus_TCP_driver()
