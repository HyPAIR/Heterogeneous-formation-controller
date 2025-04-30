#!/usr/bin/env python3
from __future__ import print_function, annotations

import sys
import rospy
from ur_dashboard_msgs.srv import GetRobotMode, GetRobotModeResponse
from std_srvs.srv import Trigger


class UR:
    
    def __init__(self, ur_ns: str):
        self.trigger_service = rospy.Service(ur_ns+'restart_fault', Trigger, self._trigger_cb)
        self._robot_mode = None
        self._robot_mode_service = rospy.ServiceProxy(ur_ns+'ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)
        self._brake_release_service = rospy.ServiceProxy(ur_ns+'ur_hardware_interface/dashboard/brake_release', Trigger)
        self._play_service = rospy.ServiceProxy(ur_ns+'ur_hardware_interface/dashboard/play', Trigger)
        self._stop_service = rospy.ServiceProxy(ur_ns+'ur_hardware_interface/dashboard/stop', Trigger)
        
        # self._restart() # use trigger instead

    def _get_robot_mode(self):
        self._robot_mode = self._robot_mode_service().robot_mode.mode
        rospy.loginfo("Robot mode: {}".format(self._robot_mode))

    def _restart(self):
        self._get_robot_mode()
        # if self._robot_mode == 7:
        #     self._restart_service()
        rospy.loginfo(f"Called service stop: {self._stop_service()}")
        rospy.sleep(1)
        rospy.loginfo(f"Called service brake release: {self._brake_release_service()}")
        rospy.sleep(5)
        rospy.loginfo(f"Called service play: {self._play_service()}")
        self._get_robot_mode()

    def _trigger_cb(self, req):
        self._restart()
        return {"success": True, "message": "UR restarted"}

if __name__ == "__main__":
    rospy.init_node("ur_restart")
    urs = sys.argv[1:]
    if not urs:
        urs = rospy.get_param("prefixs_ur", ["UR10_l", "UR10_r"])

    rospy.loginfo(f"namespaces URs: {urs}")
    for ur in urs:
        rospy.loginfo(f"Setting up {ur}")
        UR(ur)
    rospy.spin()
