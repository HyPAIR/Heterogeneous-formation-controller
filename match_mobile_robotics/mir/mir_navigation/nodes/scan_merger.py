#!/usr/bin/env python3

from typing import List, Dict

from yaml import scan

import rospy

from sensor_msgs.msg import LaserScan


class ScanMerger():
    def __init__(self) -> None:
        rospy.init_node('scan_merger', anonymous=False)

        rospy.loginfo("ScanMerger is running")

        if rospy.has_param("~scan_input_topics"):
            self.scan_input_topics = rospy.get_param("~scan_input_topics")
        if rospy.has_param("~scan_output_topic"):
            self.scan_output_topic = rospy.get_param("~scan_output_topic")

        self.scan_input_data_list: Dict[str, LaserScan] = dict()

        self.scan_input_pub_list: List[rospy.Subscriber] = list()
        for scan_input_topic in self.scan_input_topics:
            self.scan_input_pub_list.append(rospy.Subscriber(rospy.get_namespace() + scan_input_topic, LaserScan, callback=self.scan_input_cb, callback_args=scan_input_topic))
            self.scan_input_data_list[scan_input_topic] = None # Initialize dict member
        
        self.scan_pub = rospy.Publisher(rospy.get_namespace() + "scan", LaserScan, queue_size=10)

    def run(self) -> None:
        rospy.spin()

    def scan_input_cb(self, scan_data: LaserScan, topic_name):
        for key, laser_scan_data in self.scan_input_data_list.items():
            # If the current element in dict is the received laser scanner, skip it
            if(key == topic_name):
                continue
            
            # If the laser scan data of one scanner was not yet received, skip it
            if(laser_scan_data == None):
                continue
            
            # If timestamp matches another one, increase this timestamp by one nanosec to prevent the TF_REPEATED_DATA error
            if(laser_scan_data.header.stamp.nsecs == scan_data.header.stamp.nsecs):
                scan_data.header.stamp.nsecs += 1

        self.scan_input_data_list[topic_name] = scan_data

        self.scan_pub.publish(scan_data)


    # def f_scan_cb(self, scan_data: LaserScan):
    #     self.f_scan_data = scan_data

    #     if(self.b_scan_data == None):
    #         return

    #     if(self.f_scan_data.header.stamp.nsecs == self.b_scan_data.header.stamp.nsecs):
    #         self.f_scan_data.header.stamp.nsecs += 1

    #     self.scan_pub.publish(self.f_scan_data)
        
    # def b_scan_cb(self, scan_data: LaserScan):
    #     self.b_scan_data = scan_data

    #     if(self.f_scan_data == None):
    #         return

    #     if(self.b_scan_data.header.stamp.nsecs == self.f_scan_data.header.stamp.nsecs):
    #         self.b_scan_data.header.stamp.nsecs += 1

    #     self.scan_pub.publish(self.b_scan_data)


if __name__ == '__main__':
    scan_merger: ScanMerger = ScanMerger()
    scan_merger.run()
